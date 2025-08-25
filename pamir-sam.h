/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pamir AI Signal Aggregation Module (SAM) Header
 *
 * Copyright (C) 2025 PamirAI Incorporated - http://www.pamir.ai/
 */
#ifndef _PAMIR_SAM_H
#define _PAMIR_SAM_H

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define DEVICE_NAME "pamir-sam"
#define MAX_DEVICES 1
#define RX_BUF_SIZE 64
#define TX_BUF_SIZE 256
#define DEBUG_QUEUE_SIZE 32

/* Debug configuration */
#define DEBUG_UART_ENABLED 0  /* Disable UART debugging for production */

/* Debug macros for UART operations */
#if DEBUG_UART_ENABLED
#define debug_uart_print(dev, fmt, args...) \
	dev_info(dev, "[UART] " fmt, ##args)
#define debug_uart_packet(dev, packet, direction) \
	do { \
		if (packet) { \
			dev_info(dev, "[UART-%s] [%02X %02X %02X %02X] len=4", \
				direction, \
				((uint8_t *)packet)[0], ((uint8_t *)packet)[1], \
				((uint8_t *)packet)[2], ((uint8_t *)packet)[3]); \
		} \
	} while (0)
#define debug_uart_raw(dev, data, len, direction) \
	do { \
		if (data && len > 0) { \
			char hex_str[256]; \
			int i, offset = 0; \
			for (i = 0; i < len && offset < sizeof(hex_str) - 3; i++) { \
				offset += snprintf(hex_str + offset, sizeof(hex_str) - offset, \
					"%02X ", ((uint8_t *)data)[i]); \
			} \
			hex_str[offset] = '\0'; \
			dev_info(dev, "[UART-RAW-%s] [%s] len=%zu", direction, hex_str, len); \
		} \
	} while (0)
#else
#define debug_uart_print(dev, fmt, args...) \
	do { (void)(dev); (void)(fmt); } while (0)
#define debug_uart_packet(dev, packet, direction) \
	do { (void)(dev); (void)(packet); (void)(direction); } while (0)
#define debug_uart_raw(dev, data, len, direction) \
	do { (void)(dev); (void)(data); (void)(len); (void)(direction); } while (0)
#endif

/* SAM Driver Versioning - Semantic Versioning */
#define PAMIR_SAM_VERSION_MAJOR 1
#define PAMIR_SAM_VERSION_MINOR 0
#define PAMIR_SAM_VERSION_PATCH 0

/* Error recovery constants */
#define MAX_RECOVERY_ATTEMPTS 3
#define RECOVERY_BACKOFF_MS 100
#define POWER_METRICS_TIMEOUT_MS 5000
#define BOOT_NOTIFICATION_RETRY_MS 1000

/* Protocol definitions */
#define PACKET_SIZE 4  /* Type+Flags (1B) + Data (2B) + Checksum (1B) */

/* Message types (3 most significant bits) */
#define TYPE_BUTTON     0x00  /* 0b000xxxxx */
#define TYPE_LED        0x20  /* 0b001xxxxx */
#define TYPE_POWER      0x40  /* 0b010xxxxx */
#define TYPE_DISPLAY    0x60  /* 0b011xxxxx */
#define TYPE_DEBUG_CODE 0x80  /* 0b100xxxxx */
#define TYPE_DEBUG_TEXT 0xA0  /* 0b101xxxxx */
#define TYPE_SYSTEM     0xC0  /* 0b110xxxxx */
#define TYPE_RESERVED   0xE0  /* 0b111xxxxx */
#define TYPE_MASK       0xE0  /* 0b11100000 */

/* Button event flags (5 least significant bits) */
#define BTN_UP_MASK     0x01
#define BTN_DOWN_MASK   0x02
#define BTN_SELECT_MASK 0x04
#define BTN_POWER_MASK  0x08

/* LED control flags */
#define LED_CMD_EXECUTE   0x10  /* Execute bit in type_flags */
#define LED_ID_MASK       0x0F  /* 4-bit LED ID (0-15) */
#define LED_MODE_STATIC   0x00  /* Static mode */
#define LED_MODE_BLINK    0x01  /* Blinking mode */
#define LED_MODE_FADE     0x02  /* Fade mode */
#define LED_MODE_RAINBOW  0x03  /* Rainbow mode */
#define LED_MODE_MASK     0x03  /* 2-bit mode mask in data[1] */
#define LED_MODE_SHIFT    2     /* Mode position in data[1] */
#define LED_TIME_MASK     0x03  /* 2-bit timing mask in data[1] */
#define LED_TIME_SHIFT    0     /* Timing position in data[1] */

/* LED timing values */
#define LED_TIME_100MS    0x00  /* 100ms intervals */
#define LED_TIME_200MS    0x01  /* 200ms intervals */
#define LED_TIME_500MS    0x02  /* 500ms intervals */
#define LED_TIME_1000MS   0x03  /* 1000ms intervals */

/* Power management commands */
/* Control commands (0x00-0x0F) */
#define POWER_CMD_QUERY           0x00  /* Query power status */
#define POWER_CMD_SET             0x01  /* Set power state */
#define POWER_CMD_SLEEP           0x02  /* Enter sleep mode */
#define POWER_CMD_SHUTDOWN        0x03  /* Shutdown system */

/* Reporting commands (0x10-0x1F) */
#define POWER_CMD_CURRENT         0x10  /* Current draw reporting */
#define POWER_CMD_BATTERY         0x11  /* Battery state reporting */
#define POWER_CMD_TEMP            0x12  /* Temperature reporting */
#define POWER_CMD_VOLTAGE         0x13  /* Voltage reporting */
#define POWER_CMD_REQUEST_METRICS 0x1F  /* Request all metrics */

#define POWER_CMD_MASK            0x1F  /* 5-bit command mask */
#define POWER_CMD_CONTROL_MASK    0x0F  /* Control command mask */
#define POWER_CMD_REPORT_MASK     0x10  /* Reporting command flag */

/* Debug text flags */
#define DEBUG_FIRST_CHUNK  0x10
#define DEBUG_CONTINUE     0x08
#define DEBUG_CHUNK_MASK   0x07

/* System control actions */
#define SYSTEM_PING        0x00
#define SYSTEM_RESET       0x01
#define SYSTEM_VERSION     0x02
#define SYSTEM_STATUS      0x03
#define SYSTEM_CONFIG      0x04

/**
 * struct sam_protocol_packet - SAM packet structure
 * @type_flags: Type (3 bits) and flags (5 bits)
 * @data: 16-bit payload (2 bytes)
 * @checksum: CRC8 checksum of all previous bytes
 *
 * This structure represents the ultra-optimized 4-byte packet format.
 */
struct sam_protocol_packet {
	uint8_t type_flags;
	uint8_t data[2];
	uint8_t checksum;
} __packed;

/**
 * struct debug_code_entry - Debug code entry
 * @category: Debug category (0-31)
 * @code: Debug code (0-255)
 * @param: Parameter value (0-255)
 * @timestamp: Time when the debug code was received
 *
 * Entries for the debug code circular buffer.
 */
struct debug_code_entry {
	uint8_t category;
	uint8_t code;
	uint8_t param;
	unsigned long timestamp;
};

/**
 * struct sam_power_metrics - Power metrics data
 * @current_ma: Current draw in milliamps (positive=charging, negative=discharging)
 * @battery_percent: Battery charge percentage (0-100)
 * @temperature_dc: Temperature in deci-celsius (0.1°C units)
 * @voltage_mv: Voltage in millivolts
 * @last_update: Timestamp of last successful update
 * @metrics_valid: Whether metrics are valid/fresh
 *
 * Power metrics received from RP2040 microcontroller.
 */
struct sam_power_metrics {
	int16_t current_ma;  /* Can be negative (discharging) */
	uint16_t battery_percent;
	int16_t temperature_dc;  /* Can be negative */
	uint16_t voltage_mv;
	unsigned long last_update;
	bool metrics_valid;
};

/**
 * struct sam_protocol_config - configuration for SAM protocol
 * @debug_level: Current debug level (0-3)
 * @ack_required: Whether commands require acknowledgment
 * @recovery_timeout_ms: Timeout for protocol recovery
 * @power_poll_interval_ms: Interval for power metrics polling (0 = disabled)
 *
 * This struct holds configurable parameters for the SAM protocol driver
 * that can be set through device tree properties.
 */
struct sam_protocol_config {
	unsigned int debug_level;
	bool ack_required;
	unsigned int recovery_timeout_ms;
	unsigned int power_poll_interval_ms;
};

/**
 * struct sam_protocol_data - private data for SAM protocol
 * @input_dev: pointer to input device
 * @rx_buf: Receive buffer for UART data
 * @rx_pos: Current position in receive buffer
 * @config: Protocol configuration
 * @last_receive_jiffies: Last time data was received
 * @serdev: pointer to serdev device
 * @cdev: character device for userspace communication
 * @dev_no: device number for char device
 * @tx_mutex: mutex for TX operations
 * @dev_class: device class for char device
 * @debug_codes: Circular buffer for debug codes
 * @debug_head: Head of debug code circular buffer
 * @debug_tail: Tail of debug code circular buffer
 * @debug_mutex: Mutex for debug code buffer access
 * @packet_stats: Statistics for received/processed packets
 * @rx_state: Current state of packet processing state machine
 * @work_queue: Workqueue for deferred processing
 * @power_metrics: Current power metrics data
 * @power_metrics_mutex: Mutex for power metrics access
 * @power_poll_work: Delayed work for power metrics polling
 * @reboot_notifier: Reboot notifier for shutdown notification
 * @boot_notification_sent: Whether boot notification was sent successfully
 * @recovery_in_progress: Whether protocol recovery is in progress
 * @recovery_attempts: Number of consecutive recovery attempts
 *
 * This struct holds the runtime state of the SAM protocol driver.
 */
struct sam_protocol_data {
	struct input_dev *input_dev;
	uint8_t rx_buf[RX_BUF_SIZE];
	size_t rx_pos;
	struct sam_protocol_config config;
	unsigned long last_receive_jiffies;

	/* UART device */
	struct serdev_device *serdev;
	struct cdev cdev;
	dev_t dev_no;
	struct mutex tx_mutex;
	struct class *dev_class;

	/* Debug tracking */
	struct debug_code_entry debug_codes[DEBUG_QUEUE_SIZE];
	int debug_head;
	int debug_tail;
	struct mutex debug_mutex;

	/* Protocol state */
	uint64_t packet_stats[8];  /* Stats per message type */
	int rx_state;
	struct workqueue_struct *work_queue;

	/* Power management */
	struct sam_power_metrics power_metrics;
	struct mutex power_metrics_mutex;
	struct delayed_work power_poll_work;
	struct notifier_block reboot_notifier;
	bool boot_notification_sent;

	/* Error recovery */
	bool recovery_in_progress;
	int recovery_attempts;
};

/* Function declarations */
/* Protocol functions */
uint8_t calculate_checksum(const struct sam_protocol_packet *packet);
bool verify_checksum(const struct sam_protocol_packet *packet);
int send_packet(struct sam_protocol_data *priv, struct sam_protocol_packet *packet);
void process_packet(struct sam_protocol_data *priv, const struct sam_protocol_packet *packet);

/* Command senders */
int send_led_command(struct sam_protocol_data *priv, uint8_t led_id,
		     uint8_t r, uint8_t g, uint8_t b, uint8_t mode, uint8_t timing);
int send_system_command(struct sam_protocol_data *priv, uint8_t action,
			uint8_t command, uint8_t subcommand);

/* Power management functions */
int send_boot_notification(struct sam_protocol_data *priv);
int send_power_metrics_request(struct sam_protocol_data *priv);
void power_poll_work_handler(struct work_struct *work);
int sam_reboot_notifier_call(struct notifier_block *nb, unsigned long action, void *data);

/* Error recovery functions */
void sam_protocol_recovery(struct sam_protocol_data *priv);
void sam_protocol_flush_rx_buffer(struct sam_protocol_data *priv);

/* Sysfs interface functions */
int setup_power_metrics_sysfs(struct sam_protocol_data *priv);
void cleanup_power_metrics_sysfs(struct sam_protocol_data *priv);

/* Power supply interface functions */
int setup_power_supply(struct sam_protocol_data *priv);
void cleanup_power_supply(struct sam_protocol_data *priv);

/* LED control functions */
void sam_led_brightness_set(struct led_classdev *led_cdev, enum led_brightness brightness);
int register_led_devices(struct sam_protocol_data *priv);
void unregister_led_devices(void);

/* Message handlers */
void process_button_packet(struct sam_protocol_data *priv,
			   const struct sam_protocol_packet *packet);
void process_led_packet(struct sam_protocol_data *priv,
			const struct sam_protocol_packet *packet);
void process_power_packet(struct sam_protocol_data *priv,
			  const struct sam_protocol_packet *packet);
void process_display_packet(struct sam_protocol_data *priv,
			    const struct sam_protocol_packet *packet);
void process_debug_code_packet(struct sam_protocol_data *priv,
			       const struct sam_protocol_packet *packet);
void process_debug_text_packet(struct sam_protocol_data *priv,
			       const struct sam_protocol_packet *packet);
void process_system_packet(struct sam_protocol_data *priv,
			   const struct sam_protocol_packet *packet);
void process_extended_packet(struct sam_protocol_data *priv,
			     const struct sam_protocol_packet *packet);

/* Character device interface */
int setup_char_device(struct sam_protocol_data *priv);
void cleanup_char_device(struct sam_protocol_data *priv);

/* Exported globals */
extern struct sam_protocol_data *g_sam_protocol_data;

#endif /* _PAMIR_SAM_H */
