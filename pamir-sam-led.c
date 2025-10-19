// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) LED Handler
 *
 * LED control functionality.
 *
 * Copyright (C) 2025 PamirAI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"
#include <linux/leds.h>
#include <linux/timer.h>
#include <linux/version.h>

/*
 * Timer API compatibility
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
	#define sam_timer_delete_sync(timer)    timer_delete_sync(timer)
	#define sam_timer_shutdown_sync(timer)  timer_shutdown_sync(timer)
#else
	#define sam_timer_delete_sync(timer)    del_timer_sync(timer)
	#define sam_timer_shutdown_sync(timer)  del_timer_sync(timer)
#endif

/* LED class devices for multiple LED support */
#define MAX_LEDS 15  /* supports 15 individual LEDs (0-14) + broadcast (15) */
struct led_classdev *pamir_leds[MAX_LEDS];

/* RGB LED state for each LED */
struct pamir_led_state {
	uint8_t r, g, b;
	uint8_t mode;
	uint8_t brightness;
	uint8_t timing;
	struct timer_list animation_timer;
	int animation_state;
	uint8_t led_id;
};

static struct pamir_led_state led_states[MAX_LEDS];

/* Forward declarations */
static void register_rgb_led_triggers(void);
static void unregister_rgb_led_triggers(void);

/* LED trigger definitions */
static struct led_trigger *heartbeat_rgb_trigger;
static struct led_trigger *breathing_rgb_trigger;
static struct led_trigger *rainbow_rgb_trigger;

/* Trigger timer and state */
static struct timer_list rgb_trigger_timer;
static int trigger_state;
static int active_trigger_led = -1;

/**
 * RGB LED trigger timer function
 */
static void rgb_trigger_timer_function(struct timer_list *t)
{
	struct sam_protocol_data *priv;
	uint8_t r, g, b;
	int ret;

	/* Safely access global pointer with reference counting */
	mutex_lock(&g_sam_driver_mutex);
	if (atomic_read(&g_sam_driver_refcount) == 0) {
		mutex_unlock(&g_sam_driver_mutex);
		return;
	}
	priv = g_sam_protocol_data;
	atomic_inc(&g_sam_driver_refcount);
	mutex_unlock(&g_sam_driver_mutex);

	if (!priv || !priv->serdev || active_trigger_led < 0 || active_trigger_led >= MAX_LEDS) {
		atomic_dec(&g_sam_driver_refcount);
		return;
	}

	/* Get current trigger for the LED */
	if (pamir_leds[active_trigger_led]->trigger == heartbeat_rgb_trigger) {
		/* Heartbeat pattern: quick double blink */
		switch (trigger_state % 8) {
		case 0: case 2: /* On phases */
			r = 255; g = 0; b = 0; /* Red heartbeat */
			break;
		case 1: case 3: case 4: case 5: case 6: case 7: /* Off phases */
			r = 0; g = 0; b = 0;
			break;
		default:
			r = 0; g = 0; b = 0;
			break;
		}
		mod_timer(&rgb_trigger_timer, jiffies + HZ/4); /* 250ms */

	} else if (pamir_leds[active_trigger_led]->trigger == breathing_rgb_trigger) {
		/* Breathing pattern: gradual fade in/out */
		int brightness = (trigger_state % 20);

		if (brightness > 10)
			brightness = 20 - brightness;
		brightness = (brightness * 255) / 10;

		r = 0; g = 0; b = brightness; /* Blue breathing */
		mod_timer(&rgb_trigger_timer, jiffies + HZ/10); /* 100ms */

	} else if (pamir_leds[active_trigger_led]->trigger == rainbow_rgb_trigger) {
		/* Rainbow pattern: cycle through colors */
		int hue = (trigger_state * 10) % 360;
		/* Simple HSV to RGB conversion for rainbow */
		if (hue < 60) {
			r = 255; g = (hue * 255) / 60; b = 0;
		} else if (hue < 120) {
			r = ((120 - hue) * 255) / 60; g = 255; b = 0;
		} else if (hue < 180) {
			r = 0; g = 255; b = ((hue - 120) * 255) / 60;
		} else if (hue < 240) {
			r = 0; g = ((240 - hue) * 255) / 60; b = 255;
		} else if (hue < 300) {
			r = ((hue - 240) * 255) / 60; g = 0; b = 255;
		} else {
			r = 255; g = 0; b = ((360 - hue) * 255) / 60;
		}
		mod_timer(&rgb_trigger_timer, jiffies + HZ/10); /* 100ms */

	} else {
		/* No active trigger, stop timer */
		atomic_dec(&g_sam_driver_refcount);
		return;
	}

	/* Send LED command to firmware */
	ret = send_led_command(priv, active_trigger_led,
			      (r * 15) / 255, (g * 15) / 255, (b * 15) / 255,
			      LED_MODE_STATIC, LED_TIME_100MS);
	if (ret)
		dev_warn(&priv->serdev->dev, "Trigger LED update failed: %d\n", ret);

	/* Update LED state */
	led_states[active_trigger_led].r = r;
	led_states[active_trigger_led].g = g;
	led_states[active_trigger_led].b = b;

	trigger_state++;
	atomic_dec(&g_sam_driver_refcount);
}

/**
 * Activate RGB LED trigger
 */
static int rgb_trigger_activate(struct led_classdev *led_cdev)
{
	/* Find LED ID */
	int led_id = -1;
	int i;

	for (i = 0; i < MAX_LEDS; i++) {
		if (pamir_leds[i] == led_cdev) {
			led_id = i;
			break;
		}
	}

	if (led_id < 0) {
		pr_err("pamir-sam: Could not find LED for trigger activation\n");
		return -EINVAL;
	}

	/* Stop any existing RGB trigger timer */
	sam_timer_delete_sync(&rgb_trigger_timer);

	/*
	 * Stop animation timer for this LED
	 * Note: triggers take precedence over modes
	 */
	sam_timer_delete_sync(&led_states[led_id].animation_timer);

	/* Set active LED and reset state */
	active_trigger_led = led_id;
	trigger_state = 0;

	/* Start the trigger timer */
	mod_timer(&rgb_trigger_timer, jiffies + HZ/10);

	return 0;
}

/**
 * Deactivate RGB LED trigger
 */
static void rgb_trigger_deactivate(struct led_classdev *led_cdev)
{
	int led_id = -1;
	int i;
	unsigned long next_interval;

	/* Find LED ID */
	for (i = 0; i < MAX_LEDS; i++) {
		if (pamir_leds[i] == led_cdev) {
			led_id = i;
			break;
		}
	}

	/* Stop the trigger timer */
	sam_timer_delete_sync(&rgb_trigger_timer);

	/* Clear active LED */
	active_trigger_led = -1;
	trigger_state = 0;

	/* If LED has animation mode set, restart animation timer */
	if (led_id >= 0 && led_id < MAX_LEDS && led_states[led_id].mode != LED_MODE_STATIC) {
		/* Reset animation state */
		led_states[led_id].animation_state = 0;

		/* Calculate timer interval based on timing value */
		switch (led_states[led_id].timing & 0x03) {
		case 0:  /* LED_TIME_100MS */
			next_interval = HZ / 10;
			break;
		case 1:  /* LED_TIME_200MS */
			next_interval = HZ / 5;
			break;
		case 2:  /* LED_TIME_500MS */
			next_interval = HZ / 2;
			break;
		case 3:  /* LED_TIME_1000MS */
			next_interval = HZ;
			break;
		default:
			next_interval = HZ / 2;
			break;
		}

		/* Restart animation timer */
		mod_timer(&led_states[led_id].animation_timer, jiffies + next_interval);
	}
}

/**
 * LED animation timer function for looping modes (blink, fade, rainbow)
 * This provides continuous animation for user-specified RGB colors
 */
static void led_animation_timer_function(struct timer_list *t)
{
	struct pamir_led_state *led_state = container_of(t, struct pamir_led_state, animation_timer);
	struct sam_protocol_data *priv;
	uint8_t led_id = led_state->led_id;
	uint8_t r, g, b;
	uint8_t mode = led_state->mode;
	unsigned long next_interval;
	int ret;

	/* Validate LED ID */
	if (led_id >= MAX_LEDS) {
		pr_err("pamir-sam: Invalid LED ID %u in animation timer\n", led_id);
		return;
	}

	/* Safely access global pointer with reference counting */
	mutex_lock(&g_sam_driver_mutex);
	if (atomic_read(&g_sam_driver_refcount) == 0) {
		mutex_unlock(&g_sam_driver_mutex);
		return;
	}
	priv = g_sam_protocol_data;
	atomic_inc(&g_sam_driver_refcount);
	mutex_unlock(&g_sam_driver_mutex);

	if (!priv || !priv->serdev) {
		atomic_dec(&g_sam_driver_refcount);
		return;
	}

	/* Calculate timing interval based on timing value */
	switch (led_state->timing & 0x03) {
	case 0:  /* LED_TIME_100MS */
		next_interval = HZ / 10;  /* 100ms */
		break;
	case 1:  /* LED_TIME_200MS */
		next_interval = HZ / 5;   /* 200ms */
		break;
	case 2:  /* LED_TIME_500MS */
		next_interval = HZ / 2;   /* 500ms */
		break;
	case 3:  /* LED_TIME_1000MS */
		next_interval = HZ;       /* 1000ms */
		break;
	default:
		next_interval = HZ / 2;   /* Default 500ms */
		break;
	}

	/* Calculate RGB based on animation mode and state */
	switch (mode) {
	case LED_MODE_BLINK:
		/* Blink: alternate between stored RGB and off */
		if (led_state->animation_state % 2 == 0) {
			r = led_state->r;
			g = led_state->g;
			b = led_state->b;
		} else {
			r = 0;
			g = 0;
			b = 0;
		}
		led_state->animation_state++;
		break;

	case LED_MODE_FADE:
		/* Fade: smooth in/out using stored RGB as max brightness */
		{
			int brightness = led_state->animation_state % 20;

			if (brightness > 10)
				brightness = 20 - brightness;

			/* Scale stored RGB by fade brightness (0-10) */
			r = (led_state->r * brightness) / 10;
			g = (led_state->g * brightness) / 10;
			b = (led_state->b * brightness) / 10;

			led_state->animation_state++;
		}
		break;

	case LED_MODE_RAINBOW:
		/* Rainbow: cycle through hues */
		{
			int hue = (led_state->animation_state * 10) % 360;
			/* Simple HSV to RGB conversion */
			if (hue < 60) {
				r = 255; g = (hue * 255) / 60; b = 0;
			} else if (hue < 120) {
				r = ((120 - hue) * 255) / 60; g = 255; b = 0;
			} else if (hue < 180) {
				r = 0; g = 255; b = ((hue - 120) * 255) / 60;
			} else if (hue < 240) {
				r = 0; g = ((240 - hue) * 255) / 60; b = 255;
			} else if (hue < 300) {
				r = ((hue - 240) * 255) / 60; g = 0; b = 255;
			} else {
				r = 255; g = 0; b = ((360 - hue) * 255) / 60;
			}

			led_state->animation_state++;
		}
		break;

	default:
		r = led_state->r;
		g = led_state->g;
		b = led_state->b;
		atomic_dec(&g_sam_driver_refcount);
		return;  /* Don't reschedule for static mode */
	}

	/* Send LED command with calculated RGB using STATIC mode
	 * (firmware just displays the color, kernel does the animation)
	 */
	ret = send_led_command(priv, led_id,
			      (r * 15) / 255, (g * 15) / 255, (b * 15) / 255,
			      LED_MODE_STATIC, led_state->timing);
	if (ret)
		dev_warn(&priv->serdev->dev, "Animation LED %u update failed: %d\n", led_id, ret);

	/* Reschedule timer for next animation frame */
	mod_timer(&led_state->animation_timer, jiffies + next_interval);

	atomic_dec(&g_sam_driver_refcount);
}

/**
 * process_led_packet() - Process LED control packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle LED control messages from the RP2040.
 */
void process_led_packet(struct sam_protocol_data *priv,
			const struct sam_protocol_packet *packet)
{
	uint8_t led_id;

	/* Input validation */
	if (!priv || !packet) {
		pr_warn("pamir-sam: Invalid parameters in %s\n", __func__);
		return;
	}

	/* Extract LED ID from type_flags bits 3-0 */
	led_id = packet->type_flags & LED_ID_MASK;

	/* Validate LED ID is within acceptable range */
	if (led_id >= MAX_LEDS) {
		dev_warn(&priv->serdev->dev, "Invalid LED ID %u (max: %u)\n", led_id, MAX_LEDS - 1);
		return;
	}
	uint8_t execute = (packet->type_flags & LED_CMD_EXECUTE) ? 1 : 0;

	/* Extract RGB values from data[0] */
	uint8_t r = (packet->data[0] >> 4) & 0x0F;
	uint8_t g = packet->data[0] & 0x0F;

	/* Extract blue, mode, and timing from data[1] */
	uint8_t b = (packet->data[1] >> 4) & 0x0F;
	uint8_t mode = (packet->data[1] >> LED_MODE_SHIFT) & LED_MODE_MASK;
	uint8_t timing = (packet->data[1] >> LED_TIME_SHIFT) & LED_TIME_MASK;

	/* Scale RGB values to 0-255 range */
	r = (r * 255) / 15;
	g = (g * 255) / 15;
	b = (b * 255) / 15;

	dev_dbg(&priv->serdev->dev, "LED packet - LED: %u, Execute: %u, Mode: %u, R: %u, G: %u, B: %u, Timing: %u\n",
	     led_id, execute, mode, r, g, b, timing);

	/*
	 * Validate LED ID
	 * allow individual LEDs (0-14) and broadcast (15)
	 */
	if (led_id >= MAX_LEDS && led_id != LED_BROADCAST) {
		dev_warn(&priv->serdev->dev, "Invalid LED ID: %u\n", led_id);
		return;
	}

	/* Handle broadcast vs individual LED */
	if (led_id == LED_BROADCAST) {
		/* Broadcast: update all individual LEDs */
		int i;

		for (i = 0; i < MAX_LEDS; i++) {
			led_states[i].r = r;
			led_states[i].g = g;
			led_states[i].b = b;
			led_states[i].mode = mode;

			/* Update LED class device if registered */
			if (pamir_leds[i]) {
				/* Convert RGB to brightness using luminance formula */
				int brightness = (r * 299 + g * 587 + b * 114) / 1000;

				led_states[i].brightness = brightness;
				led_set_brightness(pamir_leds[i], brightness);
			}
		}
		dev_info(&priv->serdev->dev, "LED broadcast: all LEDs set to RGB(%u,%u,%u) mode %u\n",
			 r, g, b, mode);
	} else {
		/* Individual LED: update single LED */
		led_states[led_id].r = r;
		led_states[led_id].g = g;
		led_states[led_id].b = b;
		led_states[led_id].mode = mode;

		/* Update LED class device if registered */
		if (pamir_leds[led_id]) {
			/* Convert RGB to brightness using luminance formula */
			int brightness = (r * 299 + g * 587 + b * 114) / 1000;

			led_states[led_id].brightness = brightness;
			led_set_brightness(pamir_leds[led_id], brightness);
		}
		dev_info(&priv->serdev->dev, "LED %u set to RGB(%u,%u,%u) mode %u\n",
			 led_id, r, g, b, mode);
	}
}

/**
 * sam_led_brightness_set() - Set LED brightness from host
 * @led_cdev: LED class device
 * @brightness: Brightness value (0-255)
 *
 * Convert brightness to RGB LED command and send to RP2040.
 * Preserves existing RGB color and adjusts brightness.
 */
void sam_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	uint8_t r, g, b;
	uint8_t scaled_brightness;
	int ret;

	if (!led_cdev) {
		pr_err("pamir-sam: LED brightness_set: invalid LED device\n");
		return;
	}

	/* Parse LED ID from device name (pamir:led0, pamir:led1, etc.) */
	if (sscanf(led_cdev->name, "pamir:led%hhu", &led_id) != 1) {
		pr_warn("pamir-sam: LED brightness_set: unknown LED %s\n", led_cdev->name);
		return;
	}

	/* Validate LED ID */
	if (led_id >= MAX_LEDS) {
		pr_warn("pamir-sam: LED brightness_set: invalid LED ID %u\n", led_id);
		return;
	}

	/* Check if driver is available */
	if (!priv || !priv->serdev) {
		pr_warn("pamir-sam: LED brightness_set: driver not available\n");
		return;
	}

	/* Convert brightness (0-255) to 4-bit value (0-15) */
	scaled_brightness = (brightness * 15) / 255;

	/* Use existing RGB color if available, otherwise use white */
	if (led_states[led_id].r || led_states[led_id].g || led_states[led_id].b) {
		/* Scale existing RGB by brightness */
		r = (led_states[led_id].r * scaled_brightness) / 15;
		g = (led_states[led_id].g * scaled_brightness) / 15;
		b = (led_states[led_id].b * scaled_brightness) / 15;
	} else {
		/* Use white color for brightness control */
		r = g = b = scaled_brightness;
	}

	/* Convert back to 4-bit values for protocol */
	r = (r * 15) / 255;
	g = (g * 15) / 255;
	b = (b * 15) / 255;

	ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, LED_TIME_100MS);

	if (ret) {
		dev_warn(&priv->serdev->dev, "Failed to set LED %u brightness: %d\n", led_id, ret);
	} else {
		led_states[led_id].brightness = brightness;
		dev_dbg(&priv->serdev->dev, "LED %u brightness set to %u\n", led_id, brightness);
	}
}

/**
 * RGB sysfs attributes for LED control
 */
static ssize_t led_red_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	return sprintf(buf, "%u\n", led_states[led_id].r);
}

static ssize_t led_red_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	unsigned long value;
	int ret;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	if (value > 255)
		return -EINVAL;

	/* Store new red value */
	led_states[led_id].r = value;

	/* Send updated RGB command */
	if (priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;

		ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, LED_TIME_100MS);
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t led_green_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	return sprintf(buf, "%u\n", led_states[led_id].g);
}

static ssize_t led_green_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	unsigned long value;
	int ret;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	if (value > 255)
		return -EINVAL;

	/* Store new green value */
	led_states[led_id].g = value;

	/* Send updated RGB command */
	if (priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;

		ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, LED_TIME_100MS);
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t led_blue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	return sprintf(buf, "%u\n", led_states[led_id].b);
}

static ssize_t led_blue_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	unsigned long value;
	int ret;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	if (value > 255)
		return -EINVAL;

	/* Store new blue value */
	led_states[led_id].b = value;

	/* Send updated RGB command */
	if (priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;

		ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, LED_TIME_100MS);
		if (ret)
			return ret;
	}

	return count;
}

/**
 * LED animation mode sysfs attributes
 */
static ssize_t led_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;
	const char *mode_name = "static";

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	/* Convert mode to string */
	switch (led_states[led_id].mode) {
	case LED_MODE_STATIC:
		mode_name = "static";
		break;
	case LED_MODE_BLINK:
		mode_name = "blink";
		break;
	case LED_MODE_FADE:
		mode_name = "fade";
		break;
	case LED_MODE_RAINBOW:
		mode_name = "rainbow";
		break;
	default:
		mode_name = "unknown";
		break;
	}

	return sprintf(buf, "%s\n", mode_name);
}

static ssize_t led_mode_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	uint8_t mode = LED_MODE_STATIC;
	int ret;
	char mode_str[16];
	unsigned long next_interval;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	/* Parse mode string */
	if (sscanf(buf, "%15s", mode_str) != 1)
		return -EINVAL;

	if (strcmp(mode_str, "static") == 0)
		mode = LED_MODE_STATIC;
	else if (strcmp(mode_str, "blink") == 0)
		mode = LED_MODE_BLINK;
	else if (strcmp(mode_str, "fade") == 0)
		mode = LED_MODE_FADE;
	else if (strcmp(mode_str, "rainbow") == 0)
		mode = LED_MODE_RAINBOW;
	else
		return -EINVAL;

	sam_timer_delete_sync(&led_states[led_id].animation_timer);

	led_states[led_id].mode = mode;

	if (mode == LED_MODE_STATIC) {
		if (priv && priv->serdev) {
			uint8_t r = (led_states[led_id].r * 15) / 255;
			uint8_t g = (led_states[led_id].g * 15) / 255;
			uint8_t b = (led_states[led_id].b * 15) / 255;
			uint8_t time_value = led_states[led_id].timing;

			/* For static mode: send command once and don't start timer */
			ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, time_value);
			if (ret)
				return ret;
		}
		return count;
	}

	/* Reset animation state */
	led_states[led_id].animation_state = 0;

	/* Calculate initial timer interval based on timing value */
	switch (led_states[led_id].timing & 0x03) {
	case 0:  /* LED_TIME_100MS */
		next_interval = HZ / 10;
		break;
	case 1:  /* LED_TIME_200MS */
		next_interval = HZ / 5;
		break;
	case 2:  /* LED_TIME_500MS */
		next_interval = HZ / 2;
		break;
	case 3:  /* LED_TIME_1000MS */
		next_interval = HZ;
		break;
	default:
		next_interval = HZ / 2;
		break;
	}

	mod_timer(&led_states[led_id].animation_timer, jiffies + next_interval);

	return count;
}

static ssize_t led_timing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	/* Convert timing value to milliseconds (2-bit: 0=100ms, 1=200ms, 2=500ms, 3=1000ms) */
	uint16_t timing_ms;

	switch (led_states[led_id].timing & 0x03) {
	case 0:
		timing_ms = 100;
		break;
	case 1:
		timing_ms = 200;
		break;
	case 2:
		timing_ms = 500;
		break;
	case 3:
		timing_ms = 1000;
		break;
	default:
		timing_ms = 100;
		break;
	}
	return sprintf(buf, "%u\n", timing_ms);
}

static ssize_t led_timing_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t led_id = 0;
	unsigned long value;
	uint8_t time_value;
	unsigned long next_interval;
	int ret;

	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1)
		return -EINVAL;

	if (led_id >= MAX_LEDS)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;

	/* Convert milliseconds to 2-bit timing value */
	if (value <= 100)
		time_value = 0;  /* 100ms */
	else if (value <= 200)
		time_value = 1;  /* 200ms */
	else if (value <= 500)
		time_value = 2;  /* 500ms */
	else
		time_value = 3;  /* 1000ms */

	/* Store new timing value */
	led_states[led_id].timing = time_value;

	/* If animation timer is active (mode is not static), restart it with new timing */
	if (led_states[led_id].mode != LED_MODE_STATIC && timer_pending(&led_states[led_id].animation_timer)) {
		/* Calculate new timer interval */
		switch (time_value & 0x03) {
		case 0:  /* LED_TIME_100MS */
			next_interval = HZ / 10;
			break;
		case 1:  /* LED_TIME_200MS */
			next_interval = HZ / 5;
			break;
		case 2:  /* LED_TIME_500MS */
			next_interval = HZ / 2;
			break;
		case 3:  /* LED_TIME_1000MS */
			next_interval = HZ;
			break;
		default:
			next_interval = HZ / 2;
			break;
		}

		/* Restart timer with new interval */
		mod_timer(&led_states[led_id].animation_timer, jiffies + next_interval);
	}
	else if (led_states[led_id].mode == LED_MODE_STATIC && priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;

		/* For static mode, send command once */
		ret = send_led_command(priv, led_id, r, g, b, LED_MODE_STATIC, time_value);
		if (ret)
			return ret;
	}

	return count;
}

static DEVICE_ATTR(red, 0644, led_red_show, led_red_store);
static DEVICE_ATTR(green, 0644, led_green_show, led_green_store);
static DEVICE_ATTR(blue, 0644, led_blue_show, led_blue_store);
static DEVICE_ATTR(mode, 0644, led_mode_show, led_mode_store);
static DEVICE_ATTR(timing, 0644, led_timing_show, led_timing_store);

static struct attribute *led_rgb_attrs[] = {
	&dev_attr_red.attr,
	&dev_attr_green.attr,
	&dev_attr_blue.attr,
	&dev_attr_mode.attr,
	&dev_attr_timing.attr,
	NULL
};

static const struct attribute_group led_rgb_group = {
	.attrs = led_rgb_attrs,
};

/**
 * register_led_devices() - Register LED class devices
 * @priv: Private driver data
 *
 * Register multiple LED devices with RGB support.
 */
int register_led_devices(struct sam_protocol_data *priv)
{
	int i, ret;

	for (i = 0; i < MAX_LEDS; i++) {
		pamir_leds[i] = devm_kzalloc(&priv->serdev->dev, sizeof(struct led_classdev), GFP_KERNEL);
		if (!pamir_leds[i]) {
			ret = -ENOMEM;
			goto err_cleanup;
		}

		/* Set LED device name */
		pamir_leds[i]->name = devm_kasprintf(&priv->serdev->dev, GFP_KERNEL, "pamir:led%d", i);
		if (!pamir_leds[i]->name) {
			ret = -ENOMEM;
			goto err_cleanup;
		}

		pamir_leds[i]->brightness_set = sam_led_brightness_set;
		pamir_leds[i]->max_brightness = 255;
		pamir_leds[i]->brightness = 0;

		/* Disable default triggers for RGB LEDs */
		pamir_leds[i]->default_trigger = NULL;

		ret = devm_led_classdev_register(&priv->serdev->dev, pamir_leds[i]);
		if (ret) {
			dev_err(&priv->serdev->dev, "Failed to register LED %d: %d\n", i, ret);
			goto err_cleanup;
		}

		/* Add RGB sysfs attributes */
		ret = sysfs_create_group(&pamir_leds[i]->dev->kobj, &led_rgb_group);
		if (ret) {
			dev_err(&priv->serdev->dev, "Failed to create RGB sysfs for LED %d: %d\n", i, ret);
			/* Continue without RGB attributes */
		}

		/* Initialize LED state */
		led_states[i].r = 0;
		led_states[i].g = 0;
		led_states[i].b = 0;
		led_states[i].mode = LED_MODE_STATIC;
		led_states[i].brightness = 0;
		led_states[i].timing = LED_TIME_500MS; /* Default 500ms timing */
		led_states[i].animation_state = 0;
		led_states[i].led_id = i;

		timer_setup(&led_states[i].animation_timer, led_animation_timer_function, 0);
	}

	/* Initialize trigger timer */
	timer_setup(&rgb_trigger_timer, rgb_trigger_timer_function, 0);

	/* Register custom RGB LED triggers */
	register_rgb_led_triggers();

	return 0;

err_cleanup:
	for (i = 0; i < MAX_LEDS; i++) {
		if (pamir_leds[i]) {
			sysfs_remove_group(&pamir_leds[i]->dev->kobj, &led_rgb_group);
			pamir_leds[i] = NULL;
		}
	}
	return ret;
}

/**
 * unregister_led_devices() - Unregister LED class devices
 *
 * Clean up LED devices and RGB sysfs attributes.
 */
void unregister_led_devices(void)
{
	int i;

	unregister_rgb_led_triggers();

	sam_timer_shutdown_sync(&rgb_trigger_timer);

	for (i = 0; i < MAX_LEDS; i++)
		sam_timer_shutdown_sync(&led_states[i].animation_timer);

	for (i = 0; i < MAX_LEDS; i++) {
		if (pamir_leds[i]) {
			sysfs_remove_group(&pamir_leds[i]->dev->kobj, &led_rgb_group);
			pamir_leds[i] = NULL;
		}
	}
}

/**
 * register_rgb_led_triggers() - Register RGB LED triggers
 */
static void register_rgb_led_triggers(void)
{
	int ret;

	/* Register heartbeat RGB trigger */
	heartbeat_rgb_trigger = kzalloc(sizeof(struct led_trigger), GFP_KERNEL);
	if (heartbeat_rgb_trigger) {
		heartbeat_rgb_trigger->name = "heartbeat-rgb";
		heartbeat_rgb_trigger->activate = rgb_trigger_activate;
		heartbeat_rgb_trigger->deactivate = rgb_trigger_deactivate;
		ret = led_trigger_register(heartbeat_rgb_trigger);
		if (ret) {
			pr_err("pamir-sam: Failed to register heartbeat RGB trigger: %d\n", ret);
			kfree(heartbeat_rgb_trigger);
			heartbeat_rgb_trigger = NULL;
		}
	}

	/* Register breathing RGB trigger */
	breathing_rgb_trigger = kzalloc(sizeof(struct led_trigger), GFP_KERNEL);
	if (breathing_rgb_trigger) {
		breathing_rgb_trigger->name = "breathing-rgb";
		breathing_rgb_trigger->activate = rgb_trigger_activate;
		breathing_rgb_trigger->deactivate = rgb_trigger_deactivate;
		ret = led_trigger_register(breathing_rgb_trigger);
		if (ret) {
			pr_err("pamir-sam: Failed to register breathing RGB trigger: %d\n", ret);
			kfree(breathing_rgb_trigger);
			breathing_rgb_trigger = NULL;
		}
	}

	/* Register rainbow RGB trigger */
	rainbow_rgb_trigger = kzalloc(sizeof(struct led_trigger), GFP_KERNEL);
	if (rainbow_rgb_trigger) {
		rainbow_rgb_trigger->name = "rainbow-rgb";
		rainbow_rgb_trigger->activate = rgb_trigger_activate;
		rainbow_rgb_trigger->deactivate = rgb_trigger_deactivate;
		ret = led_trigger_register(rainbow_rgb_trigger);
		if (ret) {
			pr_err("pamir-sam: Failed to register rainbow RGB trigger: %d\n", ret);
			kfree(rainbow_rgb_trigger);
			rainbow_rgb_trigger = NULL;
		}
	}

	pr_debug("pamir-sam: RGB LED triggers registered\n");
}

/**
 * unregister_rgb_led_triggers() - Unregister RGB LED triggers
 */
static void unregister_rgb_led_triggers(void)
{
	if (heartbeat_rgb_trigger) {
		led_trigger_unregister(heartbeat_rgb_trigger);
		kfree(heartbeat_rgb_trigger);
		heartbeat_rgb_trigger = NULL;
	}

	if (breathing_rgb_trigger) {
		led_trigger_unregister(breathing_rgb_trigger);
		kfree(breathing_rgb_trigger);
		breathing_rgb_trigger = NULL;
	}

	if (rainbow_rgb_trigger) {
		led_trigger_unregister(rainbow_rgb_trigger);
		kfree(rainbow_rgb_trigger);
		rainbow_rgb_trigger = NULL;
	}
}
