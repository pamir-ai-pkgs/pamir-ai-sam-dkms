// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Driver
 *
 * Driver for communication between host processor and RP2040 microcontroller
 * providing interface to sensor inputs and actuator outputs on CM5 device.
 *
 * Copyright (C) 2025 PamirAI Incorporated - http://www.pamir.ai/
 */
#include <linux/version.h>
#include "pamir-sam.h"

/* Global pointer for LED brightness control access with protection */
struct sam_protocol_data *g_sam_protocol_data;
DEFINE_MUTEX(g_sam_driver_mutex);
atomic_t g_sam_driver_refcount = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(g_sam_protocol_data);
EXPORT_SYMBOL_GPL(g_sam_driver_mutex);
EXPORT_SYMBOL_GPL(g_sam_driver_refcount);

/**
 * sam_protocol_receive_buf() - Process received UART data
 * @serdev: Serial device
 * @data: Received data buffer
 * @count: Number of bytes received
 *
 * State machine for packet processing.
 * Note: Mainline kernel 5.13+ changed return type from int to size_t, but
 * vendor kernels (Rockchip, Allwinner) kept the old int return type even
 * in 6.x series. Using 6.8 threshold to cover vendor kernel compatibility.
 *
 * Return: Number of bytes processed
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
static size_t sam_protocol_receive_buf(struct serdev_device *serdev,
				     const unsigned char *data, size_t count)
#else
static int sam_protocol_receive_buf(struct serdev_device *serdev,
				     const unsigned char *data, size_t count)
#endif
{
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);
	size_t i;

	if (!priv) {
		dev_err(&serdev->dev, "UART RX: priv is NULL!");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
		return count;
#else
		return (int)count;
#endif
	}

	if (!data || count == 0) {
		dev_warn(&serdev->dev, "UART RX: Invalid data or count=0");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
		return count;
#else
		return (int)count;
#endif
	}

	debug_uart_raw(&serdev->dev, data, count, "RX");
	debug_uart_print(&serdev->dev, "RX callback triggered: %zu bytes received", count);

	for (i = 0; i < count; i++) {
		/* Store byte in buffer with overflow protection */
		if (priv->rx_pos < RX_BUF_SIZE) {
			priv->rx_buf[priv->rx_pos++] = data[i];
			debug_uart_print(&serdev->dev, "RX byte[%zu]: 0x%02X, buffer pos: %zu",
					  i, data[i], priv->rx_pos);
		} else {
			dev_warn(&serdev->dev, "RX buffer overflow, triggering recovery");
			/* Trigger protocol recovery instead of simple reset */
			sam_protocol_recovery(priv);
			priv->rx_pos = 0;
			/* Store the byte that caused overflow for resync */
			priv->rx_buf[priv->rx_pos++] = data[i];
		}

		/* Process packet when complete */
		if (priv->rx_pos >= PACKET_SIZE) {
			debug_uart_print(&serdev->dev, "Buffer has %zu bytes, processing packet", priv->rx_pos);
			debug_uart_packet(&serdev->dev, priv->rx_buf, "RX");
			process_packet(priv, (const struct sam_protocol_packet *)priv->rx_buf);
			priv->rx_pos = 0;
		}
	}

	priv->last_receive_jiffies = jiffies;
	debug_uart_print(&serdev->dev, "RX callback completed, processed %zu bytes", count);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 13, 0)
	return count;
#else
	return (int)count;
#endif
}

/* UART operations */
static const struct serdev_device_ops sam_protocol_serdev_ops = {
	.receive_buf = sam_protocol_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/**
 * sam_protocol_load_config() - Load driver configuration from device tree
 * @node: Device tree node
 * @config: Configuration structure to populate
 *
 * Read device tree properties to configure driver behavior.
 */
static void sam_protocol_load_config(struct device_node *node,
				      struct sam_protocol_config *config)
{
	/* Set defaults */
	config->debug_level = 0;  /* Production: no debug output by default */
	config->ack_required = false;
	config->recovery_timeout_ms = 1000;
	config->power_poll_interval_ms = 1000;  /* Default 1 second polling */
	config->num_leds = 15;  /* Default LEDs if not specified */

	/* Override with device tree settings if present */
	of_property_read_u32(node, "debug-level", &config->debug_level);
	of_property_read_u32(node, "recovery-timeout-ms",
			  &config->recovery_timeout_ms);
	of_property_read_u32(node, "power-poll-interval-ms",
			  &config->power_poll_interval_ms);
	of_property_read_u32(node, "num-leds", &config->num_leds);

	config->ack_required = of_property_read_bool(node, "ack-required");

	/* Validate configuration values */
	if (config->debug_level > 3) {
		pr_warn("pamir-sam: Invalid debug level %u, using 3\n", config->debug_level);
		config->debug_level = 3;
	}

	if (config->recovery_timeout_ms < 100 || config->recovery_timeout_ms > 30000) {
		pr_warn("pamir-sam: Invalid recovery timeout %u ms, using 1000\n",
			config->recovery_timeout_ms);
		config->recovery_timeout_ms = 1000;
	}

	/* Power polling interval: 0 = disabled, otherwise must be >= 100ms */
	if (config->power_poll_interval_ms > 0 && config->power_poll_interval_ms < 100) {
		pr_warn("pamir-sam: Power poll interval too small (%u ms), using 1000\n",
			config->power_poll_interval_ms);
		config->power_poll_interval_ms = 1000;
	}

	/* Validate LED count: must be between 1 and 15 */
	if (config->num_leds < 1 || config->num_leds > 15) {
		pr_warn("pamir-sam: Invalid num-leds %u, using 15\n", config->num_leds);
		config->num_leds = 15;
	}
}

/**
 * sam_protocol_probe() - Probe function for SAM protocol driver
 * @serdev: Serial device
 *
 * Initialize hardware, set up input device, and register with the kernel.
 *
 * Return: 0 on success, negative error code on failure
 */
static int sam_protocol_probe(struct serdev_device *serdev)
{
	struct sam_protocol_data *priv;
	struct input_dev *input_dev;
	int ret;

	priv = devm_kzalloc(&serdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Load configuration from device tree */
	sam_protocol_load_config(serdev->dev.of_node, &priv->config);

	input_dev = devm_input_allocate_device(&serdev->dev);
	if (!input_dev) {
		dev_err(&serdev->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	priv->input_dev = input_dev;
	priv->rx_pos = 0;
	priv->last_receive_jiffies = jiffies;
	priv->serdev = serdev;
	priv->debug_head = 0;
	priv->debug_tail = 0;
	priv->boot_notification_sent = false;
	priv->recovery_in_progress = false;
	priv->recovery_attempts = 0;

	/* Initialize mutexes */
	mutex_init(&priv->tx_mutex);
	mutex_init(&priv->power_metrics_mutex);
	mutex_init(&priv->debug_mutex);

	/* Initialize power metrics */
	priv->power_metrics.metrics_valid = false;
	priv->power_metrics.last_update = 0;

	input_dev->name = "Pamir AI Signal Aggregation Module";
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_DOWN, input_dev->keybit);
	__set_bit(KEY_ENTER, input_dev->keybit);
	__set_bit(KEY_POWER, input_dev->keybit);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&serdev->dev, "Failed to register input device: %d\n",
	  ret);
		return ret;
	}

	/* Register LED class devices */
	ret = register_led_devices(priv);
	if (ret) {
		dev_err(&serdev->dev, "Failed to register LED devices: %d\n", ret);
		input_unregister_device(input_dev);
		return ret;
	}

	serdev_device_set_drvdata(serdev, priv);
	serdev_device_set_client_ops(serdev, &sam_protocol_serdev_ops);

	/* Set global pointer for LED brightness control access */
	/* Set global pointer with protection */
	mutex_lock(&g_sam_driver_mutex);
	g_sam_protocol_data = priv;
	atomic_set(&g_sam_driver_refcount, 1);
	mutex_unlock(&g_sam_driver_mutex);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "Failed to open serdev: %d\n", ret);
		goto err_cleanup_leds;
	}

	/* Configure UART parameters */
	ret = serdev_device_set_baudrate(serdev, 115200);
	if (ret != 115200)
		dev_warn(&serdev->dev, "Baudrate set to %d instead of requested 115200", ret);
	else
		debug_uart_print(&serdev->dev, "Baudrate successfully set to 115200");

	serdev_device_set_flow_control(serdev, false);

	debug_uart_print(&serdev->dev, "UART initialized - Baud: 115200, Flow control: disabled");
	debug_uart_print(&serdev->dev, "RX callback registered: %p", sam_protocol_receive_buf);
	debug_uart_print(&serdev->dev, "Waiting for RP2040 communication...");

	ret = setup_char_device(priv);
	if (ret < 0) {
		dev_err(&serdev->dev, "Failed to set up char device: %d\n",
	  ret);
		goto err_close_serdev;
	}

	/* Initialize work queue for deferred processing */
	priv->work_queue = create_singlethread_workqueue("sam_protocol_wq");
	if (!priv->work_queue) {
		dev_err(&serdev->dev, "Failed to create workqueue\n");
		cleanup_char_device(priv);
		serdev_device_close(serdev);
		input_unregister_device(input_dev);
		return -ENOMEM;
	}

	/* Initialize power polling work */
	INIT_DELAYED_WORK(&priv->power_poll_work, power_poll_work_handler);

	/* Set up reboot notifier for shutdown notifications */
	priv->reboot_notifier.notifier_call = sam_reboot_notifier_call;
	priv->reboot_notifier.priority = 0;
	ret = register_reboot_notifier(&priv->reboot_notifier);
	if (ret) {
		dev_warn(&serdev->dev, "Failed to register reboot notifier: %d\n", ret);
		/* Non-fatal, continue initialization */
	}

	/* Set up power metrics sysfs interface */
	ret = setup_power_metrics_sysfs(priv);
	if (ret) {
		dev_warn(&serdev->dev, "Failed to setup power metrics sysfs: %d\n", ret);
		/* Non-fatal, continue initialization */
	}

	/* Set up power supply interface */
	ret = setup_power_supply(priv);
	if (ret) {
		dev_warn(&serdev->dev, "Failed to setup power supply: %d\n", ret);
		/* Non-fatal, continue initialization */
	}

	/* Send boot notification with version exchange */
	ret = send_boot_notification(priv);
	if (ret == 0) {
		dev_dbg(&serdev->dev, "Boot notification sent successfully\n");
		priv->boot_notification_sent = true;
	} else {
		dev_warn(&serdev->dev, "Boot notification failed: %d\n", ret);
		/* Non-fatal, but mark as not sent for potential retry */
		priv->boot_notification_sent = false;
	}

	/* Start power metrics polling if enabled */
	if (priv->config.power_poll_interval_ms > 0) {
		schedule_delayed_work(&priv->power_poll_work,
				      msecs_to_jiffies(priv->config.power_poll_interval_ms));
		dev_dbg(&serdev->dev, "Power metrics polling started (interval: %u ms)\n",
			 priv->config.power_poll_interval_ms);
	}

	dev_dbg(&serdev->dev, "SAM protocol driver initialized and ready\n");
	debug_uart_print(&serdev->dev, "=== SAM Kernel Driver Initialized ===");
	debug_uart_print(&serdev->dev, "Driver ready for RP2040 communication");
	return 0;

	/* Error cleanup paths */
err_close_serdev:
	serdev_device_close(serdev);
err_cleanup_leds:
	unregister_led_devices();
	mutex_lock(&g_sam_driver_mutex);
	g_sam_protocol_data = NULL;
	atomic_set(&g_sam_driver_refcount, 0);
	mutex_unlock(&g_sam_driver_mutex);
	input_unregister_device(input_dev);
	return ret;
}

/**
 * sam_protocol_remove() - Remove function for SAM protocol driver
 * @serdev: Serial device
 *
 * Clean up resources when the driver is unloaded.
 */
static void sam_protocol_remove(struct serdev_device *serdev)
{
	struct sam_protocol_data *priv = serdev_device_get_drvdata(serdev);

	dev_dbg(&serdev->dev, "Removing SAM protocol driver\n");

	/* Cancel power polling work */
	cancel_delayed_work_sync(&priv->power_poll_work);

	/* Unregister reboot notifier */
	unregister_reboot_notifier(&priv->reboot_notifier);

	/* Clean up power metrics sysfs interface */
	cleanup_power_metrics_sysfs(priv);

	/* Clean up power supply interface */
	cleanup_power_supply(priv);

	if (priv->work_queue)
		destroy_workqueue(priv->work_queue);

	/* Send a system reset command to notify RP2040 of host shutdown */
	send_system_command(priv, SYSTEM_RESET, 0, 0);

	/* Clean up character device */
	cleanup_char_device(priv);

	/* Clean up LED devices */
	unregister_led_devices();

	serdev_device_close(serdev);
	input_unregister_device(priv->input_dev);

	/* Clear global pointer */
	/* Clear global pointer with protection */
	mutex_lock(&g_sam_driver_mutex);
	g_sam_protocol_data = NULL;
	atomic_set(&g_sam_driver_refcount, 0);
	mutex_unlock(&g_sam_driver_mutex);
}

#ifdef CONFIG_OF
static const struct of_device_id sam_protocol_of_match[] = {
	{ .compatible = "pamir-ai,sam" },
	{}
};

MODULE_DEVICE_TABLE(of, sam_protocol_of_match);
#endif

static struct serdev_device_driver sam_protocol_driver = {
	.probe = sam_protocol_probe,
	.remove = sam_protocol_remove,
	.driver = {
		.name = "pamir_sam_protocol",
		.of_match_table = of_match_ptr(sam_protocol_of_match),
	},
};

module_serdev_device_driver(sam_protocol_driver);

MODULE_ALIAS("serdev:pamir_sam_protocol");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("PamirAI Incorporated.");
MODULE_DESCRIPTION("PamirAI Signal Aggregation Module (SAM)");
