// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) System Handler
 *
 * System command functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"
#include <linux/version.h>

/**
 * send_sam_version() - Send SAM driver version information
 * @priv: Private driver data
 *
 * Send SAM driver version information to the RP2040.
 *
 * Return: 0 on success, negative error code on failure
 */
static int send_sam_version(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;
	int ret;

	/* Send basic version information (major.minor) */
	packet.type_flags = TYPE_SYSTEM | SYSTEM_VERSION;
	packet.data[0] = PAMIR_SAM_VERSION_MAJOR;
	packet.data[1] = PAMIR_SAM_VERSION_MINOR;

	debug_uart_print(&priv->serdev->dev, "Sending SAM version: %d.%d.%d", 
			  PAMIR_SAM_VERSION_MAJOR, PAMIR_SAM_VERSION_MINOR, PAMIR_SAM_VERSION_PATCH);

	ret = send_packet(priv, &packet);
	if (ret) {
		dev_err(&priv->serdev->dev, "Failed to send version info: %d\n", ret);
		return ret;
	}

	/* Send extended version information (patch version) using TYPE_RESERVED */
	packet.type_flags = TYPE_RESERVED | 0x01; /* Extended version command */
	packet.data[0] = PAMIR_SAM_VERSION_PATCH;
	packet.data[1] = 0x00; /* Reserved */

	ret = send_packet(priv, &packet);
	if (ret) {
		dev_err(&priv->serdev->dev, "Failed to send extended version: %d\n", ret);
		return ret;
	}

	dev_info(&priv->serdev->dev, "SAM driver version sent: %d.%d.%d\n",
		 PAMIR_SAM_VERSION_MAJOR, PAMIR_SAM_VERSION_MINOR, PAMIR_SAM_VERSION_PATCH);
	debug_uart_print(&priv->serdev->dev, "Version exchange completed");
	
	return 0;
}

/**
 * send_display_release() - Send display release signal to RP2040
 * @priv: Private driver data
 *
 * Send display release signal to notify RP2040 that Pi has booted
 * and RP2040 should release eink control back to Pi.
 *
 * Return: 0 on success, negative error code on failure
 */
static int send_display_release(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;
	int ret;

	if (!priv || !priv->serdev) {
		return -ENODEV;
	}

	/* Send display release command */
	packet.type_flags = TYPE_DISPLAY | 0x07; /* Display release command */
	packet.data[0] = 0xFF; /* Release signal */
	packet.data[1] = 0x00; /* Reserved */

	debug_uart_print(&priv->serdev->dev, "Sending display release signal");

	ret = send_packet(priv, &packet);
	if (ret) {
		dev_err(&priv->serdev->dev, "Failed to send display release signal: %d\n", ret);
		return ret;
	}

	dev_info(&priv->serdev->dev, "Display release signal sent successfully\n");
	return 0;
}

/**
 * send_boot_notification() - Send boot notification to RP2040
 * @priv: Private driver data
 *
 * Send boot notification including version exchange and power state.
 * This implements the TRM-specified boot notification sequence.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_boot_notification(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;
	int ret;

	if (!priv || !priv->serdev) {
		return -ENODEV;
	}

	debug_uart_print(&priv->serdev->dev, "Starting boot notification sequence");

	/* Step 1: Send version information */
	ret = send_sam_version(priv);
	if (ret) {
		dev_err(&priv->serdev->dev, "Boot notification: version exchange failed\n");
		return ret;
	}

	/* Small delay to allow RP2040 to process version info */
	msleep(10);

	/* Step 2: Send power state (boot notification) */
	packet.type_flags = TYPE_POWER | POWER_CMD_SET;
	packet.data[0] = 0x01; /* Power state: Running */
	packet.data[1] = 0x00; /* Flags: Normal boot */

	debug_uart_print(&priv->serdev->dev, "Sending boot notification - power state: running");

	ret = send_packet(priv, &packet);
	if (ret) {
		dev_err(&priv->serdev->dev, "Boot notification: power state failed\n");
		return ret;
	}

	/* Step 3: Send display release signal */
	ret = send_display_release(priv);
	if (ret) {
		dev_err(&priv->serdev->dev, "Boot notification: display release failed\n");
		return ret;
	}

	dev_info(&priv->serdev->dev, "Boot notification completed successfully\n");
	debug_uart_print(&priv->serdev->dev, "Boot notification sequence completed");
	return 0;
}

/**
 * sam_reboot_notifier_call() - Reboot notifier callback
 * @nb: Notifier block
 * @action: Reboot action
 * @data: Callback data (unused)
 *
 * Send shutdown notification to RP2040 before system reboot/shutdown.
 *
 * Return: NOTIFY_DONE
 */
int sam_reboot_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct sam_protocol_data *priv = container_of(nb, struct sam_protocol_data, reboot_notifier);
	struct sam_protocol_packet packet;
	uint8_t shutdown_mode = 0;
	const char *action_name = "unknown";

	if (!priv || !priv->serdev) {
		return NOTIFY_DONE;
	}

	/* Determine shutdown mode based on reboot action */
	switch (action) {
	case SYS_HALT:
		shutdown_mode = 0x00; /* Normal shutdown */
		action_name = "halt";
		break;
	case SYS_POWER_OFF:
		shutdown_mode = 0x00; /* Normal shutdown */
		action_name = "power off";
		break;
	case SYS_RESTART:
		shutdown_mode = 0x02; /* Reboot */
		action_name = "restart";
		break;
	default:
		shutdown_mode = 0x00; /* Default to normal shutdown */
		action_name = "shutdown";
		break;
	}

	/* Send shutdown notification */
	packet.type_flags = TYPE_POWER | POWER_CMD_SHUTDOWN;
	packet.data[0] = shutdown_mode;
	packet.data[1] = 0x00; /* No reason code */

	if (send_packet(priv, &packet) == 0) {
		dev_info(&priv->serdev->dev, "Shutdown notification sent (%s)\n", action_name);
		/* Give RP2040 time to process the notification */
		msleep(50);
	} else {
		dev_warn(&priv->serdev->dev, "Failed to send shutdown notification\n");
	}

	return NOTIFY_DONE;
}

/**
 * process_system_packet() - Process system control packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle system control messages from the RP2040.
 */
void process_system_packet(struct sam_protocol_data *priv,
			   const struct sam_protocol_packet *packet)
{
	uint8_t action = packet->type_flags & 0x1F;
	uint8_t command = packet->data[0];
	uint8_t subcommand = packet->data[1];

	dev_dbg(&priv->serdev->dev, "System packet - Action: 0x%02x, Cmd: 0x%02x, Sub: 0x%02x\n",
	     action, command, subcommand);

	switch (action) {
	case SYSTEM_PING:
		/* Respond with ping acknowledgment */
		send_system_command(priv, SYSTEM_PING, 0, 0);
		break;

	case SYSTEM_RESET:
		/* RP2040 will reset itself - acknowledge */
		dev_info(&priv->serdev->dev, "RP2040 reset reported\n");
		break;

	case SYSTEM_VERSION:
		/* Send SAM driver version information */
		send_sam_version(priv);
		break;

	case SYSTEM_STATUS:
		/* Send system status */
		{
			struct sam_protocol_packet response;

			response.type_flags = TYPE_SYSTEM | SYSTEM_STATUS;
			response.data[0] = 0x01;  /* Status OK */
			response.data[1] = 0x00;  /* No error */
			send_packet(priv, &response);
		}
		break;

	case SYSTEM_CONFIG:
		/* Set/Get configuration */
		if (command == 0x00) {
			/* Get configuration */
			dev_dbg(&priv->serdev->dev, "RP2040 requested configuration\n");

			/* Send current configuration */
			struct sam_protocol_packet response;

			response.type_flags = TYPE_SYSTEM | SYSTEM_CONFIG;
			response.data[0] = priv->config.debug_level; /* Debug level */
			response.data[1] = priv->config.ack_required ? 0x01 : 0x00; /* ACK required */
			send_packet(priv, &response);
		} else if (command == 0x01) {
			/* Set configuration */
			dev_info(&priv->serdev->dev, "RP2040 set configuration: Debug=%u, ACK=%u\n",
		subcommand & 0x0F, (subcommand >> 4) & 0x01);

			/* Update configuration */
			priv->config.debug_level = subcommand & 0x0F;
			priv->config.ack_required = ((subcommand >> 4) & 0x01) != 0;
		}
		break;
	}
}

/**
 * process_extended_packet() - Process extended command packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle extended commands from the RP2040 (reserved for future use).
 */
void process_extended_packet(struct sam_protocol_data *priv,
			     const struct sam_protocol_packet *packet)
{
	uint8_t ext_cmd = packet->type_flags & 0x1F;
	uint8_t data1 = packet->data[0];
	uint8_t data2 = packet->data[1];

	dev_info(&priv->serdev->dev, "Extended command received: Cmd=0x%02x, Data=0x%02x 0x%02x\n",
	      ext_cmd, data1, data2);

	/* Extended commands are reserved for future use */
	/* Just log them for now and acknowledge receipt */
	if (priv->config.ack_required)
		send_system_command(priv, SYSTEM_PING, 0, 0);
}
