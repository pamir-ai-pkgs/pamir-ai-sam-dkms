// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) LED Handler
 *
 * LED control functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/* LED class device for control */
struct led_classdev *pamir_led;

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
	uint8_t mode = packet->type_flags & LED_MODE_MASK;
	__maybe_unused uint8_t led_id = packet->type_flags & LED_ID_MASK;
	uint8_t r = (packet->data[0] >> 4) & 0x0F;
	uint8_t g = packet->data[0] & 0x0F;
	uint8_t b = (packet->data[1] >> 4) & 0x0F;
	uint8_t value = packet->data[1] & 0x0F;

	/* Scale RGB values to 0-255 range */
	r = (r * 255) / 15;
	g = (g * 255) / 15;
	b = (b * 255) / 15;

	dev_dbg(&priv->serdev->dev, "LED packet - Mode: 0x%02x, R: %u, G: %u, B: %u, Value: %u\n",
	     mode, r, g, b, value);

	/* If we have a registered LED class device, set its brightness */
	if (pamir_led) {
		/* For simplicity, convert RGB to brightness using average */
		int brightness = (r + g + b) / 3;

		led_set_brightness(pamir_led, brightness);
	}

	/* Forward LED status to user space via a sysfs attribute if needed */
}

/**
 * sam_led_brightness_set() - Set LED brightness from host
 * @led_cdev: LED class device
 * @brightness: Brightness value (0-255)
 *
 * Convert brightness to RGB LED command and send to RP2040.
 * Uses white color (equal R,G,B) scaled by brightness.
 */
void sam_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint8_t scaled_brightness;
	int ret;

	if (!led_cdev) {
		pr_err("pamir-sam: LED brightness_set: invalid LED device\n");
		return;
	}

	/* Verify this is our LED */
	if (strcmp(led_cdev->name, "pamir:status") != 0) {
		pr_warn("pamir-sam: LED brightness_set: unknown LED %s\n", led_cdev->name);
		return;
	}

	/* Check if driver is available */
	if (!priv || !priv->serdev) {
		pr_warn("pamir-sam: LED brightness_set: driver not available\n");
		return;
	}

	/* Convert brightness (0-255) to 4-bit value (0-15) */
	scaled_brightness = (brightness * 15) / 255;

	/* Use white color (equal R,G,B components) for brightness control */
	ret = send_led_command(priv, LED_MODE_STATIC, scaled_brightness, 
	                       scaled_brightness, scaled_brightness, 0);
	
	if (ret) {
		dev_warn(&priv->serdev->dev, "Failed to set LED brightness: %d\n", ret);
	} else {
		dev_dbg(&priv->serdev->dev, "LED brightness set to %u (scaled: %u)\n", 
			brightness, scaled_brightness);
	}
}
