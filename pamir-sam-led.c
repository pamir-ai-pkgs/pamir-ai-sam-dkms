// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) LED Handler
 *
 * LED control functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"
#include <linux/leds.h>
#include <linux/timer.h>

/* LED class devices for multiple LED support */
#define MAX_LEDS 16  /* supports 16 LEDs (0-15) */
struct led_classdev *pamir_leds[MAX_LEDS];

/* RGB LED state for each LED */
struct pamir_led_state {
	uint8_t r, g, b;
	uint8_t mode;
	uint8_t brightness;
	uint8_t timing;  /* Animation timing value (0-3: 100ms/200ms/500ms/1000ms) */
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
static int trigger_state = 0;
static int active_trigger_led = -1;

/**
 * RGB LED trigger timer function
 */
static void rgb_trigger_timer_function(struct timer_list *t)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t r, g, b;
	int ret;
	
	if (!priv || !priv->serdev || active_trigger_led < 0 || active_trigger_led >= MAX_LEDS)
		return;
	
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
		if (brightness > 10) brightness = 20 - brightness;
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
		return;
	}
	
	/* Send LED command to firmware */
	ret = send_led_command(priv, active_trigger_led, 
			      (r * 15) / 255, (g * 15) / 255, (b * 15) / 255, 
			      LED_MODE_STATIC, LED_TIME_100MS);
	if (ret) {
		dev_warn(&priv->serdev->dev, "Trigger LED update failed: %d\n", ret);
	}
	
	/* Update LED state */
	led_states[active_trigger_led].r = r;
	led_states[active_trigger_led].g = g;
	led_states[active_trigger_led].b = b;
	
	trigger_state++;
}

/**
 * Activate RGB LED trigger
 */
static int rgb_trigger_activate(struct led_classdev *led_cdev)
{
	/* Find LED ID */
	int led_id = -1;
	for (int i = 0; i < MAX_LEDS; i++) {
		if (pamir_leds[i] == led_cdev) {
			led_id = i;
			break;
		}
	}
	
	if (led_id < 0) {
		pr_err("pamir-sam: Could not find LED for trigger activation\n");
		return -EINVAL;
	}
	
	/* Stop any existing timer */
	del_timer_sync(&rgb_trigger_timer);
	
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
	/* Stop the trigger timer */
	del_timer_sync(&rgb_trigger_timer);
	
	/* Clear active LED */
	active_trigger_led = -1;
	trigger_state = 0;
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
	/* Extract LED ID from type_flags bits 3-0 */
	uint8_t led_id = packet->type_flags & LED_ID_MASK;
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

	/* Validate LED ID */
	if (led_id >= MAX_LEDS) {
		dev_warn(&priv->serdev->dev, "Invalid LED ID: %u\n", led_id);
		return;
	}

	/* Store LED state */
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
	char led_name[32];

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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
	
	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
	if (led_id >= MAX_LEDS)
		return -EINVAL;
		
	/* Parse mode string */
	if (sscanf(buf, "%15s", mode_str) != 1)
		return -EINVAL;
		
	if (strcmp(mode_str, "static") == 0) {
		mode = LED_MODE_STATIC;
	} else if (strcmp(mode_str, "blink") == 0) {
		mode = LED_MODE_BLINK;
	} else if (strcmp(mode_str, "fade") == 0) {
		mode = LED_MODE_FADE;
	} else if (strcmp(mode_str, "rainbow") == 0) {
		mode = LED_MODE_RAINBOW;
	} else {
		return -EINVAL;
	}
	
	/* Store new mode */
	led_states[led_id].mode = mode;
	
	/* Send LED command with current RGB values and mode */
	if (priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;
		uint8_t time_value = led_states[led_id].timing;
		
		ret = send_led_command(priv, led_id, r, g, b, mode, time_value);
		if (ret)
			return ret;
	}
	
	return count;
}

static ssize_t led_timing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t led_id = 0;
	
	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
	if (led_id >= MAX_LEDS)
		return -EINVAL;
		
	/* Convert timing value to milliseconds (2-bit: 0=100ms, 1=200ms, 2=500ms, 3=1000ms) */
	uint16_t timing_ms;
	switch (led_states[led_id].timing & 0x03) {
	case 0: timing_ms = 100; break;
	case 1: timing_ms = 200; break;
	case 2: timing_ms = 500; break;
	case 3: timing_ms = 1000; break;
	default: timing_ms = 100; break;
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
	int ret;
	
	/* Parse LED ID from device name */
	if (sscanf(dev_name(dev), "pamir:led%hhu", &led_id) != 1) {
		return -EINVAL;
	}
	
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
		
	/* Send LED command with current RGB values, mode, and new timing */
	if (priv && priv->serdev) {
		uint8_t r = (led_states[led_id].r * 15) / 255;
		uint8_t g = (led_states[led_id].g * 15) / 255;
		uint8_t b = (led_states[led_id].b * 15) / 255;
		uint8_t mode = led_states[led_id].mode;
		
		ret = send_led_command(priv, led_id, r, g, b, mode, time_value);
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
	
	/* Clean up triggers */
	unregister_rgb_led_triggers();
	
	/* Stop trigger timer */
	del_timer_sync(&rgb_trigger_timer);
	
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
	
	pr_info("pamir-sam: RGB LED triggers registered\n");
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
