// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Protocol Core
 *
 * Core protocol functionality for the SAM driver.
 *
 * Copyright (C) 2025 PamirAI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/**
 * calculate_crc8() - Calculate CRC8 checksum for data
 * @data: Data to calculate CRC8 for
 * @len: Length of data
 *
 * Calculate CRC8 checksum using polynomial 0x07 (x^8 + x^2 + x + 1)
 *
 * Return: Calculated CRC8 checksum
 */
static uint8_t calculate_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

/**
 * calculate_checksum() - Calculate CRC8 checksum for packet
 * @packet: Packet to calculate checksum for
 *
 * Calculate the CRC8 checksum of first 3 bytes in the packet
 *
 * Return: Calculated CRC8 checksum
 */
uint8_t calculate_checksum(const struct sam_protocol_packet *packet)
{
	return calculate_crc8((const uint8_t *)packet, 3);
}

/**
 * verify_checksum() - Verify packet checksum
 * @packet: Packet to verify
 *
 * Check if the packet's checksum matches the calculated checksum.
 *
 * Return: true if checksum is valid, false otherwise
 */
bool verify_checksum(const struct sam_protocol_packet *packet)
{
	return packet->checksum == calculate_checksum(packet);
}

/**
 * send_packet() - Send packet to device
 * @priv: Private driver data
 * @packet: Packet to send
 *
 * Prepare and send a packet, updating the checksum.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_packet(struct sam_protocol_data *priv,
		struct sam_protocol_packet *packet)
{
	int ret;

	/* Calculate checksum */
	packet->checksum = calculate_checksum(packet);

	debug_uart_packet(&priv->serdev->dev, packet, "TX");

	mutex_lock(&priv->tx_mutex);
	ret = serdev_device_write(priv->serdev, (const unsigned char *)packet,
			       PACKET_SIZE, MAX_SCHEDULE_TIMEOUT);
	mutex_unlock(&priv->tx_mutex);

	if (ret == PACKET_SIZE) {
		debug_uart_print(&priv->serdev->dev, "Packet sent successfully");
	} else {
		debug_uart_print(&priv->serdev->dev, "Packet send failed: %d", ret);
	}

	return (ret == PACKET_SIZE) ? 0 : -EIO;
}

/**
 * send_led_command() - Send LED control command
 * @priv: Private driver data
 * @led_id: LED ID (0-15)
 * @r: Red component (0-15)
 * @g: Green component (0-15)
 * @b: Blue component (0-15)
 * @mode: LED mode (0-3)
 * @timing: Timing value (0-3)
 *
 * Send a command to control the LED
 *
 * Return: 0 on success, negative error code on failure
 */
int send_led_command(struct sam_protocol_data *priv, uint8_t led_id,
		     uint8_t r, uint8_t g, uint8_t b, uint8_t mode, uint8_t timing)
{
	struct sam_protocol_packet packet;

	/* type_flags = [001][E][LED_ID] */
	packet.type_flags = TYPE_LED | (led_id & LED_ID_MASK);
	/* data[0] = [R][G] */
	packet.data[0] = ((r & 0x0F) << 4) | (g & 0x0F);
	/* data[1] = [B][MODE][TIMING] */
	packet.data[1] = ((b & 0x0F) << 4) | ((mode & LED_MODE_MASK) << LED_MODE_SHIFT) | (timing & LED_TIME_MASK);

	return send_packet(priv, &packet);
}

/**
 * send_system_command() - Send system control command
 * @priv: Private driver data
 * @action: System action (ping, reset, etc.)
 * @command: Command parameter
 * @subcommand: Subcommand parameter
 *
 * Send a system control command to the device.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_system_command(struct sam_protocol_data *priv, uint8_t action,
			uint8_t command, uint8_t subcommand)
{
	struct sam_protocol_packet packet;

	packet.type_flags = TYPE_SYSTEM | (action & 0x1F);
	packet.data[0] = command;
	packet.data[1] = subcommand;

	return send_packet(priv, &packet);
}

/**
 * process_packet() - Process a complete packet
 * @priv: Private driver data
 * @packet: Packet to process
 *
 * Process a received packet based on its type.
 */
void process_packet(struct sam_protocol_data *priv,
		    const struct sam_protocol_packet *packet)
{
	uint8_t type = packet->type_flags & TYPE_MASK;

	debug_uart_print(&priv->serdev->dev, "Processing packet type: 0x%02X", type);

	/* Update statistics */
	priv->packet_stats[type >> 5]++;

	/* Verify checksum first */
	if (!verify_checksum(packet)) {
		dev_warn(&priv->serdev->dev,
			"Invalid checksum: got 0x%02x, expected 0x%02x\n",
			packet->checksum, calculate_checksum(packet));
		debug_uart_print(&priv->serdev->dev, "Checksum validation failed");
		
		/* Trigger protocol recovery on checksum error */
		sam_protocol_recovery(priv);
		return;
	}

	debug_uart_print(&priv->serdev->dev, "Packet checksum valid");

	/* Process by type */
	switch (type) {
	case TYPE_BUTTON:
		debug_uart_print(&priv->serdev->dev, "Processing button packet");
		process_button_packet(priv, packet);
		break;

	case TYPE_LED:
		debug_uart_print(&priv->serdev->dev, "Processing LED packet");
		process_led_packet(priv, packet);
		break;

	case TYPE_POWER:
		debug_uart_print(&priv->serdev->dev, "Processing power packet");
		process_power_packet(priv, packet);
		break;

	case TYPE_DISPLAY:
		debug_uart_print(&priv->serdev->dev, "Processing display packet");
		process_display_packet(priv, packet);
		break;

	case TYPE_DEBUG_CODE:
		debug_uart_print(&priv->serdev->dev, "Processing debug code packet");
		process_debug_code_packet(priv, packet);
		break;

	case TYPE_DEBUG_TEXT:
		debug_uart_print(&priv->serdev->dev, "Processing debug text packet");
		process_debug_text_packet(priv, packet);
		break;

	case TYPE_SYSTEM:
		debug_uart_print(&priv->serdev->dev, "Processing system packet");
		process_system_packet(priv, packet);
		break;

	case TYPE_RESERVED:
		debug_uart_print(&priv->serdev->dev, "Processing extended packet");
		process_extended_packet(priv, packet);
		break;

	default:
		/* Should never happen due to mask */
		dev_warn(&priv->serdev->dev, "Unknown packet type: 0x%02x\n", type);
		debug_uart_print(&priv->serdev->dev, "Unknown packet type: 0x%02X", type);
		break;
	}

	/* Send acknowledgment if required */
	if (priv->config.ack_required && type != TYPE_BUTTON) {
		debug_uart_print(&priv->serdev->dev, "Sending ACK for packet type: 0x%02X", type);
		send_system_command(priv, SYSTEM_PING, 0x00, 0x00);
	}
	
	/* Reset recovery attempts on successful packet processing */
	if (priv->recovery_in_progress) {
		priv->recovery_in_progress = false;
		priv->recovery_attempts = 0;
		dev_info(&priv->serdev->dev, "Protocol recovery successful\n");
		debug_uart_print(&priv->serdev->dev, "Protocol recovery completed");
	}
}

/**
 * sam_protocol_flush_rx_buffer() - Flush receive buffer
 * @priv: Private driver data
 *
 * Clear the receive buffer to discard potentially corrupted data.
 */
void sam_protocol_flush_rx_buffer(struct sam_protocol_data *priv)
{
	if (!priv) {
		return;
	}

	priv->rx_pos = 0;
	memset(priv->rx_buf, 0, sizeof(priv->rx_buf));
	dev_dbg(&priv->serdev->dev, "RX buffer flushed\n");
}

/**
 * sam_protocol_recovery() - Perform protocol recovery
 * @priv: Private driver data
 *
 * Handle protocol recovery after detecting communication errors.
 * Implements exponential backoff and limits recovery attempts.
 */
void sam_protocol_recovery(struct sam_protocol_data *priv)
{
	unsigned long backoff_ms;
	int ret;

	if (!priv || !priv->serdev) {
		return;
	}

	/* Avoid recursive recovery attempts */
	if (priv->recovery_in_progress) {
		return;
	}

	priv->recovery_in_progress = true;
	priv->recovery_attempts++;

	dev_warn(&priv->serdev->dev, "Protocol recovery attempt %d/%d\n",
		 priv->recovery_attempts, MAX_RECOVERY_ATTEMPTS);

	/* Check if we've exceeded maximum recovery attempts */
	if (priv->recovery_attempts > MAX_RECOVERY_ATTEMPTS) {
		dev_err(&priv->serdev->dev, "Protocol recovery failed after %d attempts\n",
			MAX_RECOVERY_ATTEMPTS);
		priv->recovery_in_progress = false;
		priv->recovery_attempts = 0;
		return;
	}

	/* Flush receive buffer to clear corrupted data */
	sam_protocol_flush_rx_buffer(priv);

	/* Calculate exponential backoff delay */
	backoff_ms = RECOVERY_BACKOFF_MS * (1 << (priv->recovery_attempts - 1));
	backoff_ms = min(backoff_ms, (unsigned long)priv->config.recovery_timeout_ms);

	dev_info(&priv->serdev->dev, "Recovery backoff: %lu ms\n", backoff_ms);
	msleep(backoff_ms);

	/* Send ping to test communication */
	ret = send_system_command(priv, SYSTEM_PING, 0x00, 0x00);
	if (ret) {
		dev_warn(&priv->serdev->dev, "Recovery ping failed: %d\n", ret);
		/* Recovery will be marked as failed if no response is received */
		/* The recovery_in_progress flag will be cleared on next successful packet */
	} else {
		dev_info(&priv->serdev->dev, "Recovery ping sent successfully\n");
	}

	/* Note: recovery_in_progress will be cleared when next valid packet is received */
}
