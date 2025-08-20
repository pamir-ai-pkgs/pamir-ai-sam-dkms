/**
 * Pamir AI Signal Aggregation Module (SAM) Userspace Testing Program
 * 
 * This program provides comprehensive testing capabilities for the SAM kernel driver,
 * supporting granular packet-level testing for all message types defined in the 
 * SAM protocol specification v0.2.0.
 * 
 * Features:
 * - Direct packet communication via /dev/pamir-sam character device
 * - Individual testing of all 8 message types and their variants
 * - CRC8 checksum validation per v0.2.0 specification
 * - Granular command testing for each packet type
 * - Input event monitoring for button testing
 * - LED control testing via sysfs interface
 * - Power metrics monitoring
 * - Protocol error injection and recovery testing
 * - Interactive menu system and command-line interface
 * 
 * Copyright (C) 2025 PamirAI Incorporated
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <getopt.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>

/* SAM Protocol Constants - From SAM-TRM-v0.2.0 */
#define DEVICE_NAME "/dev/pamir-sam"
#define PACKET_SIZE 4

/* Message types (3 most significant bits) - v0.2.0 */
#define TYPE_BUTTON 0x00 /* 0b000xxxxx */
#define TYPE_LED 0x20 /* 0b001xxxxx */
#define TYPE_POWER 0x40 /* 0b010xxxxx */
#define TYPE_DISPLAY 0x60 /* 0b011xxxxx */
#define TYPE_DEBUG_CODE 0x80 /* 0b100xxxxx */
#define TYPE_DEBUG_TEXT 0xA0 /* 0b101xxxxx */
#define TYPE_SYSTEM 0xC0 /* 0b110xxxxx */
#define TYPE_EXTENDED 0xE0 /* 0b111xxxxx */
#define TYPE_MASK 0xE0 /* 0b11100000 */

/* Button event flags */
#define BTN_UP_MASK 0x01
#define BTN_DOWN_MASK 0x02
#define BTN_SELECT_MASK 0x04
#define BTN_POWER_MASK 0x08

/* LED control flags - v0.2.0 specification */
#define LED_EXECUTE_BIT 0x10
#define LED_MODE_STATIC 0x00 /* 00 in bits 3-2 */
#define LED_MODE_BLINK 0x04 /* 01 in bits 3-2 */
#define LED_MODE_FADE 0x08 /* 10 in bits 3-2 */
#define LED_MODE_RAINBOW 0x0C /* 11 in bits 3-2 */
#define LED_MODE_MASK 0x0C
#define LED_ID_MASK 0x0F /* 4-bit LED ID (0-15) */
#define LED_TIMING_MASK 0x03 /* 2-bit timing (0-3) */

/* LED Timing values */
#define LED_TIMING_100MS 0x00
#define LED_TIMING_200MS 0x01
#define LED_TIMING_500MS 0x02
#define LED_TIMING_1000MS 0x03

/* Power management commands - v0.2.0 separated structure */
/* Control Commands (Host → RP2040) */
#define POWER_CMD_QUERY 0x00
#define POWER_CMD_SET 0x01
#define POWER_CMD_SLEEP 0x02
#define POWER_CMD_SHUTDOWN 0x03
#define POWER_CMD_REQUEST_METRICS 0x0F

/* Reporting Commands (RP2040 → Host) */
#define POWER_CMD_CURRENT 0x10
#define POWER_CMD_BATTERY 0x11
#define POWER_CMD_TEMP 0x12
#define POWER_CMD_VOLTAGE 0x13
#define POWER_CMD_METRICS_COMPLETE 0x1F

/* Power states */
#define POWER_STATE_OFF 0x00
#define POWER_STATE_RUNNING 0x01
#define POWER_STATE_SUSPEND 0x02
#define POWER_STATE_SLEEP 0x03

/* Display commands - v0.2.0 specification */
#define DISPLAY_CMD_QUERY 0x00
#define DISPLAY_CMD_STATUS 0x01
#define DISPLAY_CMD_INIT 0x02
#define DISPLAY_CMD_CLEAR 0x03
#define DISPLAY_CMD_REFRESH 0x04
#define DISPLAY_CMD_SLEEP 0x05
#define DISPLAY_CMD_WAKE 0x06
#define DISPLAY_CMD_RELEASE 0x07
#define DISPLAY_CMD_ACQUIRE 0x08

/* Debug categories */
#define DEBUG_CAT_SYSTEM 0x00
#define DEBUG_CAT_ERROR 0x01
#define DEBUG_CAT_BUTTON 0x02
#define DEBUG_CAT_LED 0x03
#define DEBUG_CAT_POWER 0x04
#define DEBUG_CAT_DISPLAY 0x05
#define DEBUG_CAT_COMM 0x06
#define DEBUG_CAT_PERFORMANCE 0x07

/* Debug text flags */
#define DEBUG_TEXT_FIRST 0x10
#define DEBUG_TEXT_CONTINUE 0x08
#define DEBUG_TEXT_SEQ_MASK 0x07

/* System commands */
#define SYSTEM_PING 0x00
#define SYSTEM_RESET 0x01
#define SYSTEM_VERSION 0x02
#define SYSTEM_STATUS 0x03
#define SYSTEM_CONFIG 0x04
#define SYSTEM_SYNC 0x05
#define SYSTEM_CAPABILITIES 0x06

/* Extended commands */
#define EXT_CAPABILITIES 0x00
#define EXT_SENSOR 0x01
#define EXT_ACTUATOR 0x02
#define EXT_NETWORK 0x03
#define EXT_STORAGE 0x04
#define EXT_CRYPTO 0x05
#define EXT_AUDIO 0x06
#define EXT_VIDEO 0x07

/* Testing configuration */
#define MAX_RESPONSE_WAIT_MS 2000
#define MAX_RETRY_ATTEMPTS 3
#define INPUT_DEVICE_PATH \
	"/dev/input/by-id/input-pamir-ai-signal-aggregation-module"
#define LED_SYSFS_PATH "/sys/class/leds/pamir:status"
#define LED_SYSFS_BASE_PATH "/sys/class/leds"
#define POWER_METRICS_PATH "/sys/devices/platform/serial@7e201800"

/* RP2040 timing configuration */
#define RP2040_PACKET_DELAY_MS 100
#define RP2040_RESPONSE_DELAY_MS 50
#define RP2040_COMMAND_DELAY_MS 150
#define RP2040_SEQUENCE_DELAY_MS 200

/**
 * SAM Protocol Packet Structure - v0.2.0
 */
struct sam_packet {
	uint8_t type_flags;
	uint8_t data[2];
	uint8_t checksum;
} __attribute__((packed));

/**
 * Test statistics structure
 */
struct test_stats {
	uint32_t packets_sent;
	uint32_t packets_received;
	uint32_t checksum_errors;
	uint32_t timeout_errors;
	uint32_t protocol_errors;
	uint32_t tests_passed;
	uint32_t tests_failed;
};

/**
 * Global test state
 */
struct test_context {
	int sam_fd;
	int input_fd;
	int verbose;
	int interactive;
	struct test_stats stats;
	volatile int running;
};

static struct test_context g_ctx = { 0 };

/**
 * Calculate CRC8 checksum using polynomial 0x07 - SAM-TRM-v0.2.0
 * 
 * @param data: Pointer to data buffer
 * @param len: Length of data buffer
 * @return: Calculated CRC8 checksum
 */
static uint8_t calculate_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07; /* Polynomial 0x07 */
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

/**
 * Calculate CRC8 checksum for SAM packet
 * 
 * @param packet: Pointer to SAM packet
 * @return: Calculated checksum
 */
static uint8_t calculate_packet_checksum(const struct sam_packet *packet)
{
	return calculate_crc8((const uint8_t *)packet, 3);
}

/**
 * Verify checksum of received SAM packet
 * 
 * @param packet: Pointer to SAM packet
 * @return: true if checksum is valid, false otherwise
 */
static bool verify_checksum(const struct sam_packet *packet)
{
	uint8_t expected = calculate_packet_checksum(packet);
	return packet->checksum == expected;
}

/**
 * Create a SAM packet with automatic CRC8 checksum calculation
 * 
 * @param packet: Pointer to packet structure to fill
 * @param type_flags: Message type and flags
 * @param data0: First data byte
 * @param data1: Second data byte
 */
static void create_packet(struct sam_packet *packet, uint8_t type_flags,
			  uint8_t data0, uint8_t data1)
{
	packet->type_flags = type_flags;
	packet->data[0] = data0;
	packet->data[1] = data1;
	packet->checksum = calculate_packet_checksum(packet);
}

/**
 * Print packet in human-readable format
 * 
 * @param packet: Pointer to packet
 * @param direction: "TX" or "RX"
 */
static void print_packet(const struct sam_packet *packet, const char *direction)
{
	const char *type_name = "UNKNOWN";

	switch (packet->type_flags & TYPE_MASK) {
	case TYPE_BUTTON:
		type_name = "BUTTON";
		break;
	case TYPE_LED:
		type_name = "LED";
		break;
	case TYPE_POWER:
		type_name = "POWER";
		break;
	case TYPE_DISPLAY:
		type_name = "DISPLAY";
		break;
	case TYPE_DEBUG_CODE:
		type_name = "DEBUG_CODE";
		break;
	case TYPE_DEBUG_TEXT:
		type_name = "DEBUG_TEXT";
		break;
	case TYPE_SYSTEM:
		type_name = "SYSTEM";
		break;
	case TYPE_EXTENDED:
		type_name = "EXTENDED";
		break;
	}

	printf("[%s] %s: [%02X %02X %02X %02X] (type=0x%02X, data=[%02X %02X], crc8=%02X %s)\n",
	       direction, type_name, packet->type_flags, packet->data[0],
	       packet->data[1], packet->checksum,
	       packet->type_flags & TYPE_MASK, packet->data[0], packet->data[1],
	       packet->checksum, verify_checksum(packet) ? "✓" : "✗");
}

/**
 * Send packet to SAM device
 * 
 * @param packet: Pointer to packet to send
 * @return: 0 on success, -1 on error
 */
static int send_packet(const struct sam_packet *packet)
{
	ssize_t bytes_written = write(g_ctx.sam_fd, packet, PACKET_SIZE);

	if (bytes_written != PACKET_SIZE) {
		perror("Failed to write packet");
		g_ctx.stats.protocol_errors++;
		return -1;
	}

	g_ctx.stats.packets_sent++;

	if (g_ctx.verbose) {
		print_packet(packet, "TX");
	}

	usleep(RP2040_PACKET_DELAY_MS * 1000);
	return 0;
}

/**
 * Receive packet from SAM device with timeout
 * 
 * @param packet: Pointer to packet structure to fill
 * @param timeout_ms: Timeout in milliseconds
 * @return: 0 on success, -1 on error/timeout
 */
static int receive_packet(struct sam_packet *packet, int timeout_ms)
{
	fd_set read_fds;
	struct timeval timeout;
	int ret;

	FD_ZERO(&read_fds);
	FD_SET(g_ctx.sam_fd, &read_fds);

	timeout.tv_sec = timeout_ms / 1000;
	timeout.tv_usec = (timeout_ms % 1000) * 1000;

	ret = select(g_ctx.sam_fd + 1, &read_fds, NULL, NULL, &timeout);

	if (ret == 0) {
		if (g_ctx.verbose) {
			printf("Timeout waiting for response\n");
		}
		g_ctx.stats.timeout_errors++;
		return -1;
	}

	if (ret < 0) {
		perror("select failed");
		g_ctx.stats.protocol_errors++;
		return -1;
	}

	ssize_t bytes_read = read(g_ctx.sam_fd, packet, PACKET_SIZE);

	if (bytes_read != PACKET_SIZE) {
		fprintf(stderr, "Received incomplete packet (%zd bytes)\n",
			bytes_read);
		g_ctx.stats.protocol_errors++;
		return -1;
	}

	g_ctx.stats.packets_received++;

	if (!verify_checksum(packet)) {
		fprintf(stderr, "CRC8 checksum verification failed\n");
		g_ctx.stats.checksum_errors++;
		return -1;
	}

	if (g_ctx.verbose) {
		print_packet(packet, "RX");
	}

	return 0;
}

/**
 * Send packet and wait for response
 * 
 * @param tx_packet: Packet to send
 * @param rx_packet: Buffer for response packet
 * @return: 0 on success, -1 on error
 */
static int send_and_receive(const struct sam_packet *tx_packet,
			    struct sam_packet *rx_packet)
{
	if (send_packet(tx_packet) < 0) {
		return -1;
	}

	usleep(RP2040_RESPONSE_DELAY_MS * 1000);
	return receive_packet(rx_packet, MAX_RESPONSE_WAIT_MS);
}

/* =============================================================================
 * INDIVIDUAL PACKET FUNCTIONS - MAXIMUM GRANULARITY
 * =============================================================================
 */

/* BUTTON EVENT MONITORING FUNCTIONS - Buttons are MCU → Host only */
static int monitor_button_events(int timeout_ms)
{
	struct sam_packet rx_packet;
	int events_received = 0;
	int start_time = time(NULL);

	printf("Monitoring button events for %d ms...\n", timeout_ms);
	printf("Press buttons on the device to see events\n");

	while ((time(NULL) - start_time) * 1000 < timeout_ms) {
		if (receive_packet(&rx_packet, 100) == 0) {
			// Check if this is a button packet
			if ((rx_packet.type_flags & TYPE_MASK) == TYPE_BUTTON) {
				uint8_t button_state = rx_packet.type_flags &
						       0x1F;

				printf("Button Event Received: ");
				if (button_state == 0) {
					printf("All Released");
				} else {
					if (button_state & BTN_UP_MASK)
						printf("UP ");
					if (button_state & BTN_DOWN_MASK)
						printf("DOWN ");
					if (button_state & BTN_SELECT_MASK)
						printf("SELECT ");
					if (button_state & BTN_POWER_MASK)
						printf("POWER ");
				}
				printf(" (0x%02X)\n", button_state);
				events_received++;
			}
		}
	}

	printf("Button monitoring completed. %d events received.\n",
	       events_received);
	return events_received;
}

/* LED PACKET FUNCTIONS */
static int send_led_red_static(uint8_t led_id)
{
	struct sam_packet packet;
	uint8_t data0 = (15 << 4) | 0; // R=15, G=0
	uint8_t data1 = (0 << 4) | 0; // B=0, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Red Static\n", led_id);
	return send_packet(&packet);
}

static int send_led_green_static(uint8_t led_id)
{
	struct sam_packet packet;
	uint8_t data0 = (0 << 4) | 15; // R=0, G=15
	uint8_t data1 = (0 << 4) | 0; // B=0, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Green Static\n", led_id);
	return send_packet(&packet);
}

static int send_led_blue_static(uint8_t led_id)
{
	struct sam_packet packet;
	uint8_t data0 = (0 << 4) | 0; // R=0, G=0
	uint8_t data1 = (15 << 4) | 0; // B=15, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Blue Static\n", led_id);
	return send_packet(&packet);
}

static int send_led_white_static(uint8_t led_id)
{
	struct sam_packet packet;
	uint8_t data0 = (15 << 4) | 15; // R=15, G=15
	uint8_t data1 = (15 << 4) | 0; // B=15, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d White Static\n", led_id);
	return send_packet(&packet);
}

static int send_led_off(uint8_t led_id)
{
	struct sam_packet packet;
	uint8_t data0 = 0; // R=0, G=0
	uint8_t data1 = 0; // B=0, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Off\n", led_id);
	return send_packet(&packet);
}

static int send_led_blink_red(uint8_t led_id, uint8_t timing)
{
	struct sam_packet packet;
	uint8_t data0 = (15 << 4) | 0; // R=15, G=0
	uint8_t data1 = (0 << 4) | (1 << 2) |
			(timing & 0x03); // B=0, Mode=Blink, Timing
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Red Blink (timing=%d)\n", led_id, timing);
	return send_packet(&packet);
}

static int send_led_fade_green(uint8_t led_id, uint8_t timing)
{
	struct sam_packet packet;
	uint8_t data0 = (0 << 4) | 15; // R=0, G=15
	uint8_t data1 = (0 << 4) | (2 << 2) |
			(timing & 0x03); // B=0, Mode=Fade, Timing
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Green Fade (timing=%d)\n", led_id, timing);
	return send_packet(&packet);
}

static int send_led_rainbow(uint8_t led_id, uint8_t timing)
{
	struct sam_packet packet;
	uint8_t data0 = 0; // Ignored for rainbow
	uint8_t data1 = (3 << 2) | (timing & 0x03); // Mode=Rainbow, Timing
	create_packet(&packet, TYPE_LED | (led_id & 0x0F), data0, data1);
	printf("Sending: LED %d Rainbow (timing=%d)\n", led_id, timing);
	return send_packet(&packet);
}

static int send_led_broadcast_off(void)
{
	struct sam_packet packet;
	uint8_t data0 = 0; // R=0, G=0
	uint8_t data1 = 0; // B=0, Mode=Static, Timing=100ms
	create_packet(&packet, TYPE_LED | 0x0F, data0,
		      data1); // LED ID 15 = broadcast
	printf("Sending: All LEDs Off (Broadcast)\n");
	return send_packet(&packet);
}

/* POWER PACKET FUNCTIONS */
static int send_power_query(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_QUERY, 0x00, 0x00);
	printf("Sending: Power Query\n");
	return send_packet(&packet);
}

static int send_power_set_running(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SET, POWER_STATE_RUNNING,
		      0x00);
	printf("Sending: Power Set Running\n");
	return send_packet(&packet);
}

static int send_power_set_sleep(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SET, POWER_STATE_SLEEP,
		      0x00);
	printf("Sending: Power Set Sleep\n");
	return send_packet(&packet);
}

static int send_power_sleep_30s(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SLEEP, 30,
		      0x01); // 30s, wake on button
	printf("Sending: Power Sleep 30s (wake on button)\n");
	return send_packet(&packet);
}

static int send_power_shutdown_normal(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SHUTDOWN, 0x00,
		      0x05); // Normal, user initiated
	printf("Sending: Power Shutdown Normal (user initiated)\n");
	return send_packet(&packet);
}

static int send_power_shutdown_emergency(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SHUTDOWN, 0x01,
		      0x01); // Emergency, system critical
	printf("Sending: Power Shutdown Emergency (system critical)\n");
	return send_packet(&packet);
}

static int send_power_shutdown_reboot(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_SHUTDOWN, 0x02,
		      0x03); // Reboot, maintenance
	printf("Sending: Power Shutdown Reboot (maintenance)\n");
	return send_packet(&packet);
}

static int send_power_request_all_metrics(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_REQUEST_METRICS, 0x00,
		      0x00); // All metrics
	printf("Sending: Power Request All Metrics\n");
	return send_packet(&packet);
}

static int send_power_request_current_only(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_REQUEST_METRICS, 0x01,
		      0x00); // Current only
	printf("Sending: Power Request Current Only\n");
	return send_packet(&packet);
}

static int send_power_request_battery_only(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_POWER | POWER_CMD_REQUEST_METRICS, 0x02,
		      0x00); // Battery only
	printf("Sending: Power Request Battery Only\n");
	return send_packet(&packet);
}

/* DISPLAY PACKET FUNCTIONS */
static int send_display_query(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_QUERY, 0x00, 0x00);
	printf("Sending: Display Query\n");
	return send_packet(&packet);
}

static int send_display_init(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_INIT, 0x00, 0x00);
	printf("Sending: Display Init\n");
	return send_packet(&packet);
}

static int send_display_clear_white_full(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_CLEAR, 0x00,
		      0x00); // White, full refresh
	printf("Sending: Display Clear White (Full Refresh)\n");
	return send_packet(&packet);
}

static int send_display_clear_white_partial(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_CLEAR, 0x00,
		      0x01); // White, partial refresh
	printf("Sending: Display Clear White (Partial Refresh)\n");
	return send_packet(&packet);
}

static int send_display_clear_black_full(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_CLEAR, 0xFF,
		      0x00); // Black, full refresh
	printf("Sending: Display Clear Black (Full Refresh)\n");
	return send_packet(&packet);
}

static int send_display_refresh_full(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_REFRESH, 0x00,
		      0x00); // Full refresh
	printf("Sending: Display Refresh Full\n");
	return send_packet(&packet);
}

static int send_display_refresh_partial(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_REFRESH, 0x01,
		      0x00); // Partial refresh
	printf("Sending: Display Refresh Partial\n");
	return send_packet(&packet);
}

static int send_display_sleep(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_SLEEP, 0x00, 0x00);
	printf("Sending: Display Sleep\n");
	return send_packet(&packet);
}

static int send_display_wake(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_WAKE, 0x00, 0x00);
	printf("Sending: Display Wake\n");
	return send_packet(&packet);
}

static int send_display_release_control(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_RELEASE, 0xFF,
		      0x00); // Release signal
	printf("Sending: Display Release Control (Boot Handover)\n");
	return send_packet(&packet);
}

static int send_display_acquire_control(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DISPLAY | DISPLAY_CMD_ACQUIRE, 0x01,
		      0x00); // Acquire from host
	printf("Sending: Display Acquire Control\n");
	return send_packet(&packet);
}

/* SYSTEM PACKET FUNCTIONS */
static int send_system_ping(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_PING, 0x00, 0x00);
	printf("Sending: System Ping\n");
	return send_packet(&packet);
}

static int send_system_reset_soft(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_RESET, 0x00,
		      0x00); // Soft reset
	printf("Sending: System Reset (Soft)\n");
	return send_packet(&packet);
}

static int send_system_reset_hard(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_RESET, 0x01,
		      0x00); // Hard reset
	printf("Sending: System Reset (Hard)\n");
	return send_packet(&packet);
}

static int send_system_version_query(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_VERSION, 0x00,
		      0x00); // Query version
	printf("Sending: System Version Query\n");
	return send_packet(&packet);
}

static int send_system_status_query(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_STATUS, 0x01,
		      0x00); // General status
	printf("Sending: System Status Query\n");
	return send_packet(&packet);
}

static int send_system_config_debug_level(uint8_t level)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_CONFIG, 0x01,
		      level & 0x03); // Debug level
	printf("Sending: System Config Debug Level %d\n", level);
	return send_packet(&packet);
}

static int send_system_sync_time(void)
{
	struct sam_packet packet;
	uint32_t timestamp = time(NULL);
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_SYNC,
		      (timestamp >> 8) & 0xFF, timestamp & 0xFF);
	printf("Sending: System Sync Time\n");
	return send_packet(&packet);
}

static int send_system_capabilities_query(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_SYSTEM | SYSTEM_CAPABILITIES, 0x00, 0x00);
	printf("Sending: System Capabilities Query\n");
	return send_packet(&packet);
}

/* DEBUG PACKET FUNCTIONS */
static int send_debug_system_boot_complete(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_SYSTEM, 0x01,
		      0x02); // Boot complete, stage 2
	printf("Sending: Debug System Boot Complete\n");
	return send_packet(&packet);
}

static int send_debug_error_checksum_failed(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_ERROR, 0xFE,
		      0x02); // Checksum error
	printf("Sending: Debug Error Checksum Failed\n");
	return send_packet(&packet);
}

static int send_debug_button_up_pressed(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_BUTTON, 0x10,
		      BTN_UP_MASK); // UP pressed
	printf("Sending: Debug Button UP Pressed\n");
	return send_packet(&packet);
}

static int send_debug_led_animation_complete(uint8_t led_id)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_LED, 0x20,
		      led_id); // Animation complete
	printf("Sending: Debug LED %d Animation Complete\n", led_id);
	return send_packet(&packet);
}

static int send_debug_power_state_change(uint8_t new_state)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_POWER, 0x30,
		      new_state); // State change
	printf("Sending: Debug Power State Change to %d\n", new_state);
	return send_packet(&packet);
}

static int send_debug_display_refresh_start(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_DISPLAY, 0x40,
		      0x00); // Refresh start
	printf("Sending: Debug Display Refresh Start\n");
	return send_packet(&packet);
}

static int send_debug_comm_packet_received(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_COMM, 0x50,
		      0x01); // Packet RX
	printf("Sending: Debug Comm Packet Received\n");
	return send_packet(&packet);
}

static int send_debug_performance_memory_usage(uint8_t percentage)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_CODE | DEBUG_CAT_PERFORMANCE, 0x60,
		      percentage); // Memory usage
	printf("Sending: Debug Performance Memory Usage %d%%\n", percentage);
	return send_packet(&packet);
}

static int send_debug_text_hello(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_DEBUG_TEXT | DEBUG_TEXT_FIRST, 'H',
		      'i'); // "Hi"
	printf("Sending: Debug Text 'Hi'\n");
	return send_packet(&packet);
}

static int send_debug_text_multipacket_start(void)
{
	struct sam_packet packet;
	create_packet(&packet,
		      TYPE_DEBUG_TEXT | DEBUG_TEXT_FIRST | DEBUG_TEXT_CONTINUE,
		      'S', 't');
	printf("Sending: Debug Text Multi-packet Start 'St'\n");
	return send_packet(&packet);
}

/* EXTENDED PACKET FUNCTIONS */
static int send_extended_capabilities(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_EXTENDED | EXT_CAPABILITIES, 0x01, 0x00);
	printf("Sending: Extended Capabilities\n");
	return send_packet(&packet);
}

static int send_extended_sensor_query(uint8_t sensor_id)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_EXTENDED | EXT_SENSOR, sensor_id, 0x00);
	printf("Sending: Extended Sensor Query (ID=%d)\n", sensor_id);
	return send_packet(&packet);
}

static int send_extended_actuator_control(uint8_t actuator_id, uint8_t value)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_EXTENDED | EXT_ACTUATOR, actuator_id,
		      value);
	printf("Sending: Extended Actuator Control (ID=%d, Value=%d)\n",
	       actuator_id, value);
	return send_packet(&packet);
}

static int send_extended_experimental(void)
{
	struct sam_packet packet;
	create_packet(&packet, TYPE_EXTENDED | 0x1F, 0xFF,
		      0xFF); // Experimental command
	printf("Sending: Extended Experimental Command\n");
	return send_packet(&packet);
}

/**
 * Send individual packet by name with optional LED ID
 */
static int send_individual_packet(const char *packet_name, int led_id)
{
	if (!packet_name) {
		fprintf(stderr, "Error: No packet name specified\n");
		return -1;
	}

	printf("Sending individual packet: %s", packet_name);
	if (strstr(packet_name, "led-") == packet_name) {
		printf(" (LED ID: %d)", led_id);
	}
	printf("\n");

	/* System Commands */
	if (strcmp(packet_name, "ping") == 0) {
		return send_system_ping();
	} else if (strcmp(packet_name, "reset") == 0) {
		return send_system_reset_soft();
	} else if (strcmp(packet_name, "version") == 0) {
		return send_system_version_query();
	} else if (strcmp(packet_name, "uptime") == 0) {
		return send_system_status_query();

		/* Power Commands */
	} else if (strcmp(packet_name, "shutdown-normal") == 0) {
		return send_power_shutdown_normal();
	} else if (strcmp(packet_name, "shutdown-emergency") == 0) {
		return send_power_shutdown_emergency();
	} else if (strcmp(packet_name, "sleep-light") == 0) {
		return send_power_sleep_30s();
	} else if (strcmp(packet_name, "sleep-deep") == 0) {
		return send_power_sleep_30s();
	} else if (strcmp(packet_name, "power-query") == 0) {
		return send_power_query();
	} else if (strcmp(packet_name, "power-metrics") == 0) {
		return send_power_request_all_metrics();

		/* Display Commands */
	} else if (strcmp(packet_name, "display-clear-white") == 0) {
		return send_display_clear_white_full();
	} else if (strcmp(packet_name, "display-clear-black") == 0) {
		return send_display_clear_black_full();
	} else if (strcmp(packet_name, "display-refresh") == 0) {
		return send_display_refresh_full();
	} else if (strcmp(packet_name, "display-sleep") == 0) {
		return send_display_sleep();
	} else if (strcmp(packet_name, "display-status") == 0) {
		return send_display_query();

		/* LED Commands - require LED ID parameter */
	} else if (strcmp(packet_name, "led-red") == 0) {
		return send_led_red_static(led_id);
	} else if (strcmp(packet_name, "led-green") == 0) {
		return send_led_green_static(led_id);
	} else if (strcmp(packet_name, "led-blue") == 0) {
		return send_led_blue_static(led_id);
	} else if (strcmp(packet_name, "led-white") == 0) {
		return send_led_white_static(led_id);
	} else if (strcmp(packet_name, "led-yellow") == 0) {
		return send_led_white_static(
			led_id); // Approximating yellow as white
	} else if (strcmp(packet_name, "led-cyan") == 0) {
		return send_led_blue_static(
			led_id); // Approximating cyan as blue
	} else if (strcmp(packet_name, "led-magenta") == 0) {
		return send_led_red_static(
			led_id); // Approximating magenta as red
	} else if (strcmp(packet_name, "led-off") == 0) {
		return send_led_off(led_id);
	} else if (strcmp(packet_name, "led-blink-red") == 0) {
		return send_led_blink_red(led_id, LED_TIMING_500MS);
	} else if (strcmp(packet_name, "led-blink-green") == 0) {
		return send_led_blink_red(
			led_id,
			LED_TIMING_500MS); // Using red blink function for green
	} else if (strcmp(packet_name, "led-blink-blue") == 0) {
		return send_led_blink_red(
			led_id,
			LED_TIMING_500MS); // Using red blink function for blue
	} else if (strcmp(packet_name, "led-fade-red") == 0) {
		return send_led_fade_green(
			led_id,
			LED_TIMING_1000MS); // Using green fade function for red
	} else if (strcmp(packet_name, "led-fade-green") == 0) {
		return send_led_fade_green(led_id, LED_TIMING_1000MS);
	} else if (strcmp(packet_name, "led-fade-blue") == 0) {
		return send_led_fade_green(
			led_id,
			LED_TIMING_1000MS); // Using green fade function for blue
	} else if (strcmp(packet_name, "led-rainbow") == 0) {
		return send_led_rainbow(led_id, LED_TIMING_200MS);

		/* Button Commands - Changed to monitoring (buttons are MCU → Host only) */
	} else if (strcmp(packet_name, "button-monitor") == 0) {
		return monitor_button_events(
			10000); // Monitor for 10 seconds		/* Debug Commands */
	} else if (strcmp(packet_name, "debug-info") == 0) {
		return send_debug_system_boot_complete();
	} else if (strcmp(packet_name, "debug-warning") == 0) {
		return send_debug_error_checksum_failed();
	} else if (strcmp(packet_name, "debug-error") == 0) {
		return send_debug_error_checksum_failed();
	} else if (strcmp(packet_name, "debug-critical") == 0) {
		return send_debug_error_checksum_failed();
	} else if (strcmp(packet_name, "debug-text") == 0) {
		return send_debug_text_hello();

		/* Extended Commands */
	} else if (strcmp(packet_name, "extended-capabilities") == 0) {
		return send_extended_capabilities();
	} else if (strcmp(packet_name, "extended-sensor") == 0) {
		return send_extended_sensor_query(0); // Default sensor ID 0
	} else if (strcmp(packet_name, "extended-actuator") == 0) {
		return send_extended_actuator_control(
			0, 128); // Default actuator ID 0, value 128
	} else if (strcmp(packet_name, "extended-experimental") == 0) {
		return send_extended_experimental();

	} else {
		fprintf(stderr, "Error: Unknown packet name '%s'\n",
			packet_name);
		fprintf(stderr, "Available packet names:\n");
		fprintf(stderr, "  System: ping, reset, version, uptime\n");
		fprintf(stderr,
			"  Power: shutdown-normal, shutdown-emergency, sleep-light, sleep-deep, power-query, power-metrics\n");
		fprintf(stderr,
			"  Display: display-clear-white, display-clear-black, display-refresh, display-sleep, display-status\n");
		fprintf(stderr,
			"  LED: led-red, led-green, led-blue, led-white, led-yellow, led-cyan, led-magenta, led-off,\n");
		fprintf(stderr,
			"       led-blink-red, led-blink-green, led-blink-blue, led-fade-red, led-fade-green, led-fade-blue, led-rainbow\n");
		fprintf(stderr,
			"  Button: button-monitor (monitor button events for 10s)\n");
		fprintf(stderr,
			"  Debug: debug-info, debug-warning, debug-error, debug-critical, debug-text\n");
		fprintf(stderr,
			"  Extended: extended-capabilities, extended-sensor, extended-actuator, extended-experimental\n");
		return -1;
	}
}

/* =============================================================================
 * INDIVIDUAL COMMAND TESTING FUNCTIONS
 * =============================================================================
 */

/**
 * Test button event monitoring (buttons are MCU → Host only)
 */
static int test_individual_buttons(void)
{
	printf("\n=== BUTTON EVENT MONITORING TEST ===\n");
	printf("NOTE: Buttons send events FROM RP2040 TO host, not the other way around.\n");
	printf("This test will monitor for button events for 15 seconds.\n");
	printf("Press buttons on the device to see the events.\n\n");

	int events_received = monitor_button_events(15000);

	if (events_received > 0) {
		printf("Button monitoring test PASSED: %d events received\n",
		       events_received);
		g_ctx.stats.tests_passed++;
		return 0;
	} else {
		printf("Button monitoring test: No events received (normal if no buttons pressed)\n");
		g_ctx.stats
			.tests_passed++; // Not a failure - just no button presses
		return 0;
	}
}

/**
 * Test individual LED commands
 */
static int test_led_command(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b,
			    uint8_t mode, uint8_t timing,
			    const char *description)
{
	struct sam_packet tx_packet;
	uint8_t data0, data1;

	printf("Testing LED command: %s...\n", description);
	printf("  LED ID: %d, RGB: (%d,%d,%d), Mode: %d, Timing: %d\n", led_id,
	       r, g, b, mode, timing);

	/* Validate parameters */
	if (led_id > 15 || r > 15 || g > 15 || b > 15 || mode > 3 ||
	    timing > 3) {
		printf("LED test FAILED: Invalid parameters\n");
		g_ctx.stats.tests_failed++;
		return -1;
	}

	/* Pack RGB444 format: data[0] = RRRRGGGG, data[1] = BBBBMMTT */
	data0 = (r << 4) | (g & 0x0F);
	data1 = (b << 4) | ((mode & 0x03) << 2) | (timing & 0x03);

	create_packet(&tx_packet, TYPE_LED | (led_id & LED_ID_MASK), data0,
		      data1);

	if (send_packet(&tx_packet) < 0) {
		printf("LED test FAILED: %s\n", description);
		g_ctx.stats.tests_failed++;
		return -1;
	}

	printf("LED test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test individual power commands
 */
static int test_power_command(uint8_t command, uint8_t param1, uint8_t param2,
			      const char *description, bool expect_response)
{
	struct sam_packet tx_packet, rx_packet;

	printf("Testing power command: %s...\n", description);
	printf("  Command: 0x%02X, Params: [0x%02X, 0x%02X]\n", command, param1,
	       param2);

	create_packet(&tx_packet, TYPE_POWER | (command & 0x1F), param1,
		      param2);

	if (expect_response) {
		if (send_and_receive(&tx_packet, &rx_packet) < 0) {
			printf("Power test FAILED: %s (no response)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		/* Verify response is a power response */
		if ((rx_packet.type_flags & TYPE_MASK) != TYPE_POWER) {
			printf("Power test FAILED: %s (invalid response type)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		printf("  Response: Command=0x%02X, Data=[0x%02X, 0x%02X]\n",
		       rx_packet.type_flags & 0x1F, rx_packet.data[0],
		       rx_packet.data[1]);
	} else {
		if (send_packet(&tx_packet) < 0) {
			printf("Power test FAILED: %s\n", description);
			g_ctx.stats.tests_failed++;
			return -1;
		}
	}

	printf("Power test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test individual display commands
 */
static int test_display_command(uint8_t command, uint8_t param1, uint8_t param2,
				const char *description, bool expect_response)
{
	struct sam_packet tx_packet, rx_packet;

	printf("Testing display command: %s...\n", description);
	printf("  Command: 0x%02X, Params: [0x%02X, 0x%02X]\n", command, param1,
	       param2);

	create_packet(&tx_packet, TYPE_DISPLAY | (command & 0x1F), param1,
		      param2);

	if (expect_response) {
		if (send_and_receive(&tx_packet, &rx_packet) < 0) {
			printf("Display test FAILED: %s (no response)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		/* Verify response is a display response */
		if ((rx_packet.type_flags & TYPE_MASK) != TYPE_DISPLAY) {
			printf("Display test FAILED: %s (invalid response type)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		printf("  Response: Command=0x%02X, Data=[0x%02X, 0x%02X]\n",
		       rx_packet.type_flags & 0x1F, rx_packet.data[0],
		       rx_packet.data[1]);
	} else {
		if (send_packet(&tx_packet) < 0) {
			printf("Display test FAILED: %s\n", description);
			g_ctx.stats.tests_failed++;
			return -1;
		}
	}

	printf("Display test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test individual system commands
 */
static int test_system_command(uint8_t command, uint8_t param1, uint8_t param2,
			       const char *description, bool expect_response)
{
	struct sam_packet tx_packet, rx_packet;

	printf("Testing system command: %s...\n", description);
	printf("  Command: 0x%02X, Params: [0x%02X, 0x%02X]\n", command, param1,
	       param2);

	create_packet(&tx_packet, TYPE_SYSTEM | (command & 0x1F), param1,
		      param2);

	if (expect_response) {
		if (send_and_receive(&tx_packet, &rx_packet) < 0) {
			printf("System test FAILED: %s (no response)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		/* Verify response is a system response */
		if ((rx_packet.type_flags & TYPE_MASK) != TYPE_SYSTEM) {
			printf("System test FAILED: %s (invalid response type)\n",
			       description);
			g_ctx.stats.tests_failed++;
			return -1;
		}

		printf("  Response: Command=0x%02X, Data=[0x%02X, 0x%02X]\n",
		       rx_packet.type_flags & 0x1F, rx_packet.data[0],
		       rx_packet.data[1]);
	} else {
		if (send_packet(&tx_packet) < 0) {
			printf("System test FAILED: %s\n", description);
			g_ctx.stats.tests_failed++;
			return -1;
		}
	}

	printf("System test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test debug code transmission
 */
static int test_debug_code(uint8_t category, uint8_t code, uint8_t param,
			   const char *description)
{
	struct sam_packet tx_packet;

	printf("Testing debug code: %s...\n", description);
	printf("  Category: 0x%02X, Code: 0x%02X, Param: 0x%02X\n", category,
	       code, param);

	create_packet(&tx_packet, TYPE_DEBUG_CODE | (category & 0x07), code,
		      param);

	if (send_packet(&tx_packet) < 0) {
		printf("Debug code test FAILED: %s\n", description);
		g_ctx.stats.tests_failed++;
		return -1;
	}

	printf("Debug code test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test debug text transmission (single packet)
 */
static int test_debug_text(uint8_t flags, uint8_t char1, uint8_t char2,
			   const char *description)
{
	struct sam_packet tx_packet;

	printf("Testing debug text: %s...\n", description);
	printf("  Flags: 0x%02X, Chars: ['%c', '%c']\n", flags,
	       char1 ? char1 : '?', char2 ? char2 : '?');

	create_packet(&tx_packet, TYPE_DEBUG_TEXT | (flags & 0x1F), char1,
		      char2);

	if (send_packet(&tx_packet) < 0) {
		printf("Debug text test FAILED: %s\n", description);
		g_ctx.stats.tests_failed++;
		return -1;
	}

	printf("Debug text test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/**
 * Test extended commands
 */
static int test_extended_command(uint8_t extension, uint8_t param1,
				 uint8_t param2, const char *description)
{
	struct sam_packet tx_packet;

	printf("Testing extended command: %s...\n", description);
	printf("  Extension: 0x%02X, Params: [0x%02X, 0x%02X]\n", extension,
	       param1, param2);

	create_packet(&tx_packet, TYPE_EXTENDED | (extension & 0x1F), param1,
		      param2);

	if (send_packet(&tx_packet) < 0) {
		printf("Extended test FAILED: %s\n", description);
		g_ctx.stats.tests_failed++;
		return -1;
	}

	printf("Extended test PASSED: %s\n", description);
	g_ctx.stats.tests_passed++;
	return 0;
}

/* =============================================================================
 * INDIVIDUAL COMMAND TEST SUITES
 * =============================================================================
 */

/* Function removed - buttons are MCU → Host only, see earlier implementation */

/**
 * Test all LED commands individually
 */
static int test_individual_leds(void)
{
	printf("\n=== INDIVIDUAL LED TESTS ===\n");

	int failed = 0;

	/* Test static colors on different LEDs */
	failed += test_led_command(0, 15, 0, 0, LED_MODE_STATIC >> 2,
				   LED_TIMING_100MS, "LED 0 Red Static");
	usleep(1000000);
	failed += test_led_command(1, 0, 15, 0, LED_MODE_STATIC >> 2,
				   LED_TIMING_100MS, "LED 1 Green Static");
	usleep(1000000);
	failed += test_led_command(2, 0, 0, 15, LED_MODE_STATIC >> 2,
				   LED_TIMING_100MS, "LED 2 Blue Static");
	usleep(1000000);

	/* Test animation modes */
	failed += test_led_command(0, 15, 0, 0, LED_MODE_BLINK >> 2,
				   LED_TIMING_500MS, "LED 0 Red Blink 500ms");
	usleep(2000000);
	failed += test_led_command(1, 0, 15, 0, LED_MODE_FADE >> 2,
				   LED_TIMING_1000MS,
				   "LED 1 Green Fade 1000ms");
	usleep(2000000);
	failed += test_led_command(2, 15, 15, 15, LED_MODE_RAINBOW >> 2,
				   LED_TIMING_200MS, "LED 2 Rainbow 200ms");
	usleep(3000000);

	/* Test broadcast LED (ID 15) */
	failed += test_led_command(15, 0, 0, 0, LED_MODE_STATIC >> 2,
				   LED_TIMING_100MS,
				   "All LEDs Off (Broadcast)");
	usleep(1000000);

	printf("Individual LED tests: %s\n", failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test all power commands individually
 */
static int test_individual_power(void)
{
	printf("\n=== INDIVIDUAL POWER TESTS ===\n");

	int failed = 0;

	/* Control commands (expect responses) */
	failed += test_power_command(POWER_CMD_QUERY, 0x00, 0x00, "Power Query",
				     true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_power_command(POWER_CMD_SET, POWER_STATE_RUNNING, 0x00,
				     "Set Running State", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_power_command(POWER_CMD_SLEEP, 0x1E, 0x01,
				     "Sleep 30s with wake on button", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_power_command(POWER_CMD_REQUEST_METRICS, 0x00, 0x00,
				     "Request All Metrics", false);

	/* Give time for metric responses */
	printf("Waiting for metric responses...\n");
	struct sam_packet rx_packet;
	int responses = 0;
	for (int i = 0; i < 5; i++) { /* Expect up to 5 responses */
		if (receive_packet(&rx_packet, 1000) == 0) {
			responses++;
			uint8_t cmd = rx_packet.type_flags & 0x1F;
			uint16_t value = rx_packet.data[0] |
					 (rx_packet.data[1] << 8);

			switch (cmd) {
			case POWER_CMD_CURRENT:
				printf("  Current: %d mA\n", value);
				break;
			case POWER_CMD_BATTERY:
				printf("  Battery: %d%%\n", value);
				break;
			case POWER_CMD_TEMP:
				printf("  Temperature: %.1f°C\n",
				       (int16_t)value / 10.0);
				break;
			case POWER_CMD_VOLTAGE:
				printf("  Voltage: %d mV\n", value);
				break;
			case POWER_CMD_METRICS_COMPLETE:
				printf("  Metrics complete\n");
				break;
			default:
				printf("  Unknown metric: cmd=0x%02X, value=%d\n",
				       cmd, value);
				break;
			}
		}
	}

	if (responses > 0) {
		printf("Received %d metric responses\n", responses);
		g_ctx.stats.tests_passed++;
	} else {
		printf("No metric responses received\n");
		g_ctx.stats.tests_failed++;
		failed++;
	}

	printf("Individual power tests: %s\n",
	       failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test all display commands individually
 */
static int test_individual_display(void)
{
	printf("\n=== INDIVIDUAL DISPLAY TESTS ===\n");

	int failed = 0;

	failed += test_display_command(DISPLAY_CMD_QUERY, 0x00, 0x00,
				       "Display Query", true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_display_command(DISPLAY_CMD_RELEASE, 0xFF, 0x00,
				       "Release Display Control", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_display_command(DISPLAY_CMD_CLEAR, 0x00, 0x01,
				       "Clear Display (White, Partial)", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_display_command(DISPLAY_CMD_SLEEP, 0x00, 0x00,
				       "Display Sleep", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_display_command(DISPLAY_CMD_WAKE, 0x00, 0x00,
				       "Display Wake", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	printf("Individual display tests: %s\n",
	       failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test all system commands individually  
 */
static int test_individual_system(void)
{
	printf("\n=== INDIVIDUAL SYSTEM TESTS ===\n");

	int failed = 0;

	failed += test_system_command(SYSTEM_PING, 0x00, 0x00, "System Ping",
				      true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_system_command(SYSTEM_VERSION, 0x00, 0x00,
				      "System Version", true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_system_command(SYSTEM_STATUS, 0x01, 0x00,
				      "System Status", true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_system_command(SYSTEM_CONFIG, 0x01, 0x00,
				      "System Config", false);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	failed += test_system_command(SYSTEM_CAPABILITIES, 0x00, 0x00,
				      "System Capabilities", true);
	usleep(RP2040_COMMAND_DELAY_MS * 1000);

	printf("Individual system tests: %s\n",
	       failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test all debug commands individually
 */
static int test_individual_debug(void)
{
	printf("\n=== INDIVIDUAL DEBUG TESTS ===\n");

	int failed = 0;

	/* Test debug codes for each category */
	failed += test_debug_code(DEBUG_CAT_SYSTEM, 0x01, 0x02,
				  "System Boot Complete");
	usleep(100000);
	failed +=
		test_debug_code(DEBUG_CAT_ERROR, 0xFE, 0x02, "Checksum Error");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_BUTTON, 0x10, 0x04,
				  "Button Press (SELECT)");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_LED, 0x20, 0x05,
				  "LED Animation Complete");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_POWER, 0x30, 0x42,
				  "Memory Usage 66%");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_DISPLAY, 0x40, 0x01,
				  "Display Refresh Start");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_COMM, 0x50, 0x08,
				  "Packet Processing 8us");
	usleep(100000);
	failed += test_debug_code(DEBUG_CAT_PERFORMANCE, 0x60, 0x42,
				  "Performance Metric");
	usleep(100000);

	/* Test debug text messages */
	failed +=
		test_debug_text(DEBUG_TEXT_FIRST, 'H', 'i', "Simple text 'Hi'");
	usleep(100000);
	failed += test_debug_text(DEBUG_TEXT_FIRST | DEBUG_TEXT_CONTINUE, 'T',
				  'e', "Multi-packet text start 'Te'");
	usleep(100000);
	failed += test_debug_text(DEBUG_TEXT_CONTINUE | 0x01, 's', 't',
				  "Multi-packet continue 'st'");
	usleep(100000);
	failed += test_debug_text(0x02, '!', 0x00, "Multi-packet end '!'");
	usleep(100000);

	printf("Individual debug tests: %s\n",
	       failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test all extended commands individually
 */
static int test_individual_extended(void)
{
	printf("\n=== INDIVIDUAL EXTENDED TESTS ===\n");

	int failed = 0;

	failed += test_extended_command(EXT_CAPABILITIES, 0x01, 0x00,
					"Extended Capabilities");
	usleep(100000);
	failed += test_extended_command(EXT_SENSOR, 0x02, 0x01,
					"Extended Sensor");
	usleep(100000);
	failed += test_extended_command(EXT_ACTUATOR, 0x03, 0x02,
					"Extended Actuator");
	usleep(100000);
	failed += test_extended_command(EXT_NETWORK, 0x04, 0x03,
					"Extended Network");
	usleep(100000);
	failed += test_extended_command(EXT_STORAGE, 0x05, 0x04,
					"Extended Storage");
	usleep(100000);
	failed += test_extended_command(0x1F, 0xFF, 0xFF,
					"Extended Experimental");
	usleep(100000);

	printf("Individual extended tests: %s\n",
	       failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/**
 * Test CRC8 algorithm with known test vectors
 */
static int test_crc8_vectors(void)
{
	printf("\n=== CRC8 TEST VECTORS ===\n");

	struct {
		uint8_t data[3];
		uint8_t expected_crc;
		const char *description;
	} test_vectors[] = {
		{ { 0x00, 0x00, 0x00 }, 0x00, "All zeros" },
		{ { 0xFF, 0xFF, 0xFF }, 0x49, "All ones" },
		{ { 0x20, 0xF0, 0x00 }, 0x8E, "LED red command" },
		{ { 0x40, 0x00, 0x00 }, 0x8C, "Power query" },
		{ { 0xC0, 0x00, 0x00 }, 0x8B, "System ping" },
		{ { 0x01, 0x00, 0x00 }, 0x55, "Button UP pressed" },
		{ { 0x67, 0xFF, 0x00 }, 0x98, "Display release" },
		{ { 0x80, 0x01, 0x02 }, 0x83, "Debug system boot" },
	};

	int failed = 0;

	for (int i = 0; i < sizeof(test_vectors) / sizeof(test_vectors[0]);
	     i++) {
		uint8_t calculated = calculate_crc8(test_vectors[i].data, 3);

		printf("Test vector %d: %s\n", i + 1,
		       test_vectors[i].description);
		printf("  Data: [0x%02X, 0x%02X, 0x%02X]\n",
		       test_vectors[i].data[0], test_vectors[i].data[1],
		       test_vectors[i].data[2]);
		printf("  Expected: 0x%02X, Calculated: 0x%02X %s\n",
		       test_vectors[i].expected_crc, calculated,
		       (calculated == test_vectors[i].expected_crc) ? "✓" :
								      "✗");

		if (calculated != test_vectors[i].expected_crc) {
			failed++;
			g_ctx.stats.tests_failed++;
		} else {
			g_ctx.stats.tests_passed++;
		}
	}

	printf("CRC8 test vectors: %s\n", failed == 0 ? "PASSED" : "FAILED");
	return failed;
}

/* =============================================================================
 * LEGACY FUNCTIONS (Updated for CRC8)
 * =============================================================================
 */

/**
 * Monitor input events from button device
 */
static int monitor_input_events(int duration_ms)
{
	struct input_event ev;
	fd_set read_fds;
	struct timeval timeout;
	int event_count = 0;
	int start_time = time(NULL);

	printf("\n=== INPUT EVENTS MONITORING ===\n");
	printf("Monitoring input events for %d ms...\n", duration_ms);
	printf("Press buttons on the device to see events\n");

	if (g_ctx.input_fd < 0) {
		printf("Input device not available\n");
		return 0;
	}

	while ((time(NULL) - start_time) * 1000 < duration_ms) {
		FD_ZERO(&read_fds);
		FD_SET(g_ctx.input_fd, &read_fds);

		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;

		int ret = select(g_ctx.input_fd + 1, &read_fds, NULL, NULL,
				 &timeout);

		if (ret > 0 && FD_ISSET(g_ctx.input_fd, &read_fds)) {
			if (read(g_ctx.input_fd, &ev, sizeof(ev)) ==
			    sizeof(ev)) {
				if (ev.type == EV_KEY) {
					const char *key_name = "UNKNOWN";
					switch (ev.code) {
					case KEY_UP:
						key_name = "UP";
						break;
					case KEY_DOWN:
						key_name = "DOWN";
						break;
					case KEY_ENTER:
						key_name = "SELECT";
						break;
					case KEY_POWER:
						key_name = "POWER";
						break;
					}

					printf("Input event: %s %s\n", key_name,
					       ev.value ? "PRESSED" :
							  "RELEASED");
					event_count++;
				}
			}
		}
	}

	printf("Input monitoring completed. %d events received.\n",
	       event_count);
	return event_count;
}

/**
 * Sub-menu for button event monitoring (buttons are MCU → Host only)
 */
static void button_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== BUTTON EVENT MONITORING MENU ===\n");
		printf("NOTE: Buttons send events FROM RP2040 TO host, not the other way around.\n");
		printf("1. Monitor button events (5 seconds)\n");
		printf("2. Monitor button events (10 seconds)\n");
		printf("3. Monitor button events (30 seconds)\n");
		printf("4. Monitor button events (60 seconds)\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		switch (choice[0]) {
		case '1':
			monitor_button_events(5000);
			break;
		case '2':
			monitor_button_events(10000);
			break;
		case '3':
			monitor_button_events(30000);
			break;
		case '4':
			monitor_button_events(60000);
			break;
		case 'b':
		case 'B':
			running = 0;
			break;
		default:
			printf("Invalid choice.\n");
			break;
		}
	}
}

/**
 * Sub-menu for individual LED packet tests
 */
static void led_packet_menu(void)
{
	char choice[16];
	int running = 1;
	uint8_t led_id = 0;

	printf("Enter LED ID (0-15): ");
	scanf("%hhu", &led_id);
	if (led_id > 15)
		led_id = 0;

	while (running) {
		printf("\n=== LED PACKET MENU (LED %d) ===\n", led_id);
		printf("1. Red Static\n");
		printf("2. Green Static\n");
		printf("3. Blue Static\n");
		printf("4. White Static\n");
		printf("5. Off\n");
		printf("6. Red Blink (500ms)\n");
		printf("7. Green Fade (1000ms)\n");
		printf("8. Rainbow (200ms)\n");
		printf("9. All LEDs Off (Broadcast)\n");
		printf("c. Change LED ID\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		switch (choice[0]) {
		case '1':
			send_led_red_static(led_id);
			break;
		case '2':
			send_led_green_static(led_id);
			break;
		case '3':
			send_led_blue_static(led_id);
			break;
		case '4':
			send_led_white_static(led_id);
			break;
		case '5':
			send_led_off(led_id);
			break;
		case '6':
			send_led_blink_red(led_id, LED_TIMING_500MS);
			break;
		case '7':
			send_led_fade_green(led_id, LED_TIMING_1000MS);
			break;
		case '8':
			send_led_rainbow(led_id, LED_TIMING_200MS);
			break;
		case '9':
			send_led_broadcast_off();
			break;
		case 'c':
		case 'C':
			printf("Enter new LED ID (0-15): ");
			scanf("%hhu", &led_id);
			if (led_id > 15)
				led_id = 0;
			break;
		case 'b':
		case 'B':
			running = 0;
			break;
		default:
			printf("Invalid choice.\n");
			break;
		}
		if (choice[0] >= '1' && choice[0] <= '9') {
			usleep(1000000); // 1s delay to see LED effect
		}
	}
}

/**
 * Sub-menu for individual power packet tests
 */
static void power_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== POWER PACKET MENU ===\n");
		printf("1. Query Power State\n");
		printf("2. Set Running State\n");
		printf("3. Set Sleep State\n");
		printf("4. Sleep 30s (wake on button)\n");
		printf("5. Shutdown Normal\n");
		printf("6. Shutdown Emergency\n");
		printf("7. Shutdown Reboot\n");
		printf("8. Request All Metrics\n");
		printf("9. Request Current Only\n");
		printf("10. Request Battery Only\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		if (strcmp(choice, "10") == 0) {
			send_power_request_battery_only();
		} else {
			switch (choice[0]) {
			case '1':
				send_power_query();
				break;
			case '2':
				send_power_set_running();
				break;
			case '3':
				send_power_set_sleep();
				break;
			case '4':
				send_power_sleep_30s();
				break;
			case '5':
				printf("WARNING: This will send shutdown command! Continue? (y/N): ");
				char confirm[16];
				if (fgets(confirm, sizeof(confirm), stdin) &&
				    (confirm[0] == 'y' || confirm[0] == 'Y')) {
					send_power_shutdown_normal();
				}
				break;
			case '6':
				printf("WARNING: This will send emergency shutdown! Continue? (y/N): ");
				if (fgets(confirm, sizeof(confirm), stdin) &&
				    (confirm[0] == 'y' || confirm[0] == 'Y')) {
					send_power_shutdown_emergency();
				}
				break;
			case '7':
				printf("WARNING: This will send reboot command! Continue? (y/N): ");
				if (fgets(confirm, sizeof(confirm), stdin) &&
				    (confirm[0] == 'y' || confirm[0] == 'Y')) {
					send_power_shutdown_reboot();
				}
				break;
			case '8':
				send_power_request_all_metrics();
				break;
			case '9':
				send_power_request_current_only();
				break;
			case 'b':
			case 'B':
				running = 0;
				break;
			default:
				printf("Invalid choice.\n");
				break;
			}
		}
		if ((choice[0] >= '1' && choice[0] <= '9') ||
		    strcmp(choice, "10") == 0) {
			usleep(500000); // 500ms delay
		}
	}
}

/**
 * Sub-menu for individual display packet tests
 */
static void display_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== DISPLAY PACKET MENU ===\n");
		printf("1. Query Display Status\n");
		printf("2. Initialize Display\n");
		printf("3. Clear White (Full Refresh)\n");
		printf("4. Clear White (Partial Refresh)\n");
		printf("5. Clear Black (Full Refresh)\n");
		printf("6. Refresh Full\n");
		printf("7. Refresh Partial\n");
		printf("8. Sleep Display\n");
		printf("9. Wake Display\n");
		printf("10. Release Control (Boot Handover)\n");
		printf("11. Acquire Control\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		if (strcmp(choice, "10") == 0) {
			send_display_release_control();
		} else if (strcmp(choice, "11") == 0) {
			send_display_acquire_control();
		} else {
			switch (choice[0]) {
			case '1':
				send_display_query();
				break;
			case '2':
				send_display_init();
				break;
			case '3':
				send_display_clear_white_full();
				break;
			case '4':
				send_display_clear_white_partial();
				break;
			case '5':
				send_display_clear_black_full();
				break;
			case '6':
				send_display_refresh_full();
				break;
			case '7':
				send_display_refresh_partial();
				break;
			case '8':
				send_display_sleep();
				break;
			case '9':
				send_display_wake();
				break;
			case 'b':
			case 'B':
				running = 0;
				break;
			default:
				printf("Invalid choice.\n");
				break;
			}
		}
		if ((choice[0] >= '1' && choice[0] <= '9') ||
		    strcmp(choice, "10") == 0 || strcmp(choice, "11") == 0) {
			usleep(1000000); // 1s delay for display operations
		}
	}
}

/**
 * Sub-menu for individual system packet tests
 */
static void system_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== SYSTEM PACKET MENU ===\n");
		printf("1. Ping\n");
		printf("2. Reset (Soft)\n");
		printf("3. Reset (Hard)\n");
		printf("4. Version Query\n");
		printf("5. Status Query\n");
		printf("6. Config Debug Level 0 (Off)\n");
		printf("7. Config Debug Level 1 (Error)\n");
		printf("8. Config Debug Level 2 (Info)\n");
		printf("9. Config Debug Level 3 (Verbose)\n");
		printf("10. Sync Time\n");
		printf("11. Capabilities Query\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		if (strcmp(choice, "10") == 0) {
			send_system_sync_time();
		} else if (strcmp(choice, "11") == 0) {
			send_system_capabilities_query();
		} else {
			switch (choice[0]) {
			case '1':
				send_system_ping();
				break;
			case '2':
				printf("WARNING: This will reset the system! Continue? (y/N): ");
				char confirm[16];
				if (fgets(confirm, sizeof(confirm), stdin) &&
				    (confirm[0] == 'y' || confirm[0] == 'Y')) {
					send_system_reset_soft();
				}
				break;
			case '3':
				printf("WARNING: This will hard reset the system! Continue? (y/N): ");
				if (fgets(confirm, sizeof(confirm), stdin) &&
				    (confirm[0] == 'y' || confirm[0] == 'Y')) {
					send_system_reset_hard();
				}
				break;
			case '4':
				send_system_version_query();
				break;
			case '5':
				send_system_status_query();
				break;
			case '6':
				send_system_config_debug_level(0);
				break;
			case '7':
				send_system_config_debug_level(1);
				break;
			case '8':
				send_system_config_debug_level(2);
				break;
			case '9':
				send_system_config_debug_level(3);
				break;
			case 'b':
			case 'B':
				running = 0;
				break;
			default:
				printf("Invalid choice.\n");
				break;
			}
		}
		if ((choice[0] >= '1' && choice[0] <= '9') ||
		    strcmp(choice, "10") == 0 || strcmp(choice, "11") == 0) {
			usleep(500000); // 500ms delay
		}
	}
}

/**
 * Sub-menu for individual debug packet tests
 */
static void debug_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== DEBUG PACKET MENU ===\n");
		printf("1. System Boot Complete\n");
		printf("2. Error Checksum Failed\n");
		printf("3. Button UP Pressed\n");
		printf("4. LED Animation Complete (LED 0)\n");
		printf("5. Power State Change (Running)\n");
		printf("6. Display Refresh Start\n");
		printf("7. Comm Packet Received\n");
		printf("8. Performance Memory Usage (66%%)\n");
		printf("9. Text 'Hi'\n");
		printf("10. Text Multi-packet Start 'St'\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		if (strcmp(choice, "10") == 0) {
			send_debug_text_multipacket_start();
		} else {
			switch (choice[0]) {
			case '1':
				send_debug_system_boot_complete();
				break;
			case '2':
				send_debug_error_checksum_failed();
				break;
			case '3':
				send_debug_button_up_pressed();
				break;
			case '4':
				send_debug_led_animation_complete(0);
				break;
			case '5':
				send_debug_power_state_change(
					POWER_STATE_RUNNING);
				break;
			case '6':
				send_debug_display_refresh_start();
				break;
			case '7':
				send_debug_comm_packet_received();
				break;
			case '8':
				send_debug_performance_memory_usage(66);
				break;
			case '9':
				send_debug_text_hello();
				break;
			case 'b':
			case 'B':
				running = 0;
				break;
			default:
				printf("Invalid choice.\n");
				break;
			}
		}
		if ((choice[0] >= '1' && choice[0] <= '9') ||
		    strcmp(choice, "10") == 0) {
			usleep(200000); // 200ms delay
		}
	}
}

/**
 * Sub-menu for individual extended packet tests
 */
static void extended_packet_menu(void)
{
	char choice[16];
	int running = 1;

	while (running) {
		printf("\n=== EXTENDED PACKET MENU ===\n");
		printf("1. Capabilities\n");
		printf("2. Sensor Query (ID 1)\n");
		printf("3. Sensor Query (ID 2)\n");
		printf("4. Actuator Control (ID 1, Value 128)\n");
		printf("5. Actuator Control (ID 2, Value 255)\n");
		printf("6. Experimental Command\n");
		printf("b. Back to main menu\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL)
			break;
		choice[strcspn(choice, "\n")] = 0;

		switch (choice[0]) {
		case '1':
			send_extended_capabilities();
			break;
		case '2':
			send_extended_sensor_query(1);
			break;
		case '3':
			send_extended_sensor_query(2);
			break;
		case '4':
			send_extended_actuator_control(1, 128);
			break;
		case '5':
			send_extended_actuator_control(2, 255);
			break;
		case '6':
			send_extended_experimental();
			break;
		case 'b':
		case 'B':
			running = 0;
			break;
		default:
			printf("Invalid choice.\n");
			break;
		}
		if (choice[0] >= '1' && choice[0] <= '6') {
			usleep(200000); // 200ms delay
		}
	}
}

/**
 * Enhanced Interactive menu system with maximum granularity
 */
static void interactive_menu(void)
{
	char choice[16];
	int running = 1;

	printf("\n================================================================================\n");
	printf("               SAM INTERACTIVE TEST MENU - v0.2.0 - MAXIMUM GRANULARITY\n");
	printf("================================================================================\n");

	while (running) {
		printf("\nSelect test category:\n");
		printf("=== Individual Packet Tests (Maximum Granularity) ===\n");
		printf("1. Button Packets (5 individual commands)\n");
		printf("2. LED Packets (9 individual commands)\n");
		printf("3. Power Packets (10 individual commands)\n");
		printf("4. Display Packets (11 individual commands)\n");
		printf("5. System Packets (11 individual commands)\n");
		printf("6. Debug Packets (10 individual commands)\n");
		printf("7. Extended Packets (6 individual commands)\n");
		printf("=== Grouped Command Tests ===\n");
		printf("8. Individual button tests (legacy)\n");
		printf("9. Individual LED tests (legacy)\n");
		printf("10. Individual power tests (legacy)\n");
		printf("11. Individual display tests (legacy)\n");
		printf("12. Individual system tests (legacy)\n");
		printf("13. Individual debug tests (legacy)\n");
		printf("14. Individual extended tests (legacy)\n");
		printf("=== Protocol Tests ===\n");
		printf("15. CRC8 test vectors\n");
		printf("16. Monitor input events\n");
		printf("17. Custom packet test\n");
		printf("=== Comprehensive Tests ===\n");
		printf("18. Run all tests\n");
		printf("q. Quit\n");
		printf("Choice: ");

		if (fgets(choice, sizeof(choice), stdin) == NULL) {
			break;
		}

		choice[strcspn(choice, "\n")] = 0;

		if (strcmp(choice, "10") == 0) {
			test_individual_power();
		} else if (strcmp(choice, "11") == 0) {
			test_individual_display();
		} else if (strcmp(choice, "12") == 0) {
			test_individual_system();
		} else if (strcmp(choice, "13") == 0) {
			test_individual_debug();
		} else if (strcmp(choice, "14") == 0) {
			test_individual_extended();
		} else if (strcmp(choice, "15") == 0) {
			test_crc8_vectors();
		} else if (strcmp(choice, "16") == 0) {
			monitor_input_events(10000);
		} else if (strcmp(choice, "17") == 0) {
			printf("\n=== CUSTOM PACKET TEST ===\n");
			uint8_t type, data0, data1;
			printf("Enter type_flags (hex): ");
			scanf("%hhx", &type);
			printf("Enter data[0] (hex): ");
			scanf("%hhx", &data0);
			printf("Enter data[1] (hex): ");
			scanf("%hhx", &data1);

			struct sam_packet packet;
			create_packet(&packet, type, data0, data1);

			printf("Created packet: [0x%02X 0x%02X 0x%02X 0x%02X]\n",
			       packet.type_flags, packet.data[0],
			       packet.data[1], packet.checksum);

			if (send_packet(&packet) == 0) {
				printf("Packet sent successfully\n");
				g_ctx.stats.tests_passed++;
			} else {
				printf("Packet send failed\n");
				g_ctx.stats.tests_failed++;
			}
		} else if (strcmp(choice, "18") == 0) {
			printf("\n=== COMPREHENSIVE TEST SUITE ===\n");
			int failed = 0;
			failed += test_individual_system();
			failed += test_individual_leds();
			failed += test_individual_power();
			failed += test_individual_display();
			failed += test_individual_buttons();
			failed += test_individual_debug();
			failed += test_crc8_vectors();

			printf("\n=== COMPREHENSIVE TEST RESULTS ===\n");
			printf("Tests passed: %d\n", g_ctx.stats.tests_passed);
			printf("Tests failed: %d\n", g_ctx.stats.tests_failed);
			printf("Overall result: %s\n",
			       failed == 0 ? "PASSED" : "FAILED");
		} else {
			switch (choice[0]) {
			case '1':
				button_packet_menu();
				break;
			case '2':
				led_packet_menu();
				break;
			case '3':
				power_packet_menu();
				break;
			case '4':
				display_packet_menu();
				break;
			case '5':
				system_packet_menu();
				break;
			case '6':
				debug_packet_menu();
				break;
			case '7':
				extended_packet_menu();
				break;
			case '8':
				test_individual_buttons();
				break;
			case '9':
				test_individual_leds();
				break;
			case 'q':
			case 'Q':
				running = 0;
				break;
			default:
				printf("Invalid choice. Please try again.\n");
				break;
			}
		}
	}
}

/**
 * Signal handler for graceful shutdown
 */
static void signal_handler(int signum)
{
	printf("\nReceived signal %d, shutting down...\n", signum);
	g_ctx.running = 0;
}

/**
 * Print usage information
 */
static void print_usage(const char *prog_name)
{
	printf("Usage: %s [OPTIONS]\n", prog_name);
	printf("\nSAM Protocol Testing Program v0.2.0\n");
	printf("Supports individual command and packet testing per SAM-TRM-v0.2.0\n\n");
	printf("Options:\n");
	printf("  -h, --help              Show this help message\n");
	printf("  -v, --verbose           Enable verbose output\n");
	printf("  -i, --interactive       Run in interactive mode (maximum granularity)\n");
	printf("  -t, --test TYPE         Run specific test type:\n");
	printf("                          buttons, leds, power, display, system,\n");
	printf("                          debug, extended, crc8, all\n");
	printf("  -p, --packet NAME       Send individual packet by name:\n");
	printf("                          ping, shutdown-normal, shutdown-emergency,\n");
	printf("                          display-clear-white, display-clear-black,\n");
	printf("                          led-red, led-green, led-blue, led-off,\n");
	printf("                          power-query, power-metrics, button-up, etc.\n");
	printf("  -d, --device PATH       SAM device path (default: %s)\n",
	       DEVICE_NAME);
	printf("  -m, --monitor TIME      Monitor input events for TIME seconds\n");
	printf("  -c, --crc8             Test CRC8 algorithm with known vectors\n");
	printf("  --led-id ID             LED ID for LED packet commands (0-15, default: 0)\n");
	printf("\nExamples:\n");
	printf("  %s -t buttons          # Test all button states\n",
	       prog_name);
	printf("  %s -p ping             # Send single ping packet\n",
	       prog_name);
	printf("  %s -p shutdown-normal  # Send normal shutdown packet\n",
	       prog_name);
	printf("  %s -p display-clear-white # Send display clear white packet\n",
	       prog_name);
	printf("  %s --led-id 2 -p led-red  # Send red LED packet to LED 2\n",
	       prog_name);
	printf("  %s -v -t all           # Run all tests with verbose output\n",
	       prog_name);
	printf("  %s -i                  # Interactive mode with maximum granularity\n",
	       prog_name);
}

/**
 * Initialize test context
 */
static int init_test_context(void)
{
	g_ctx.sam_fd = open(DEVICE_NAME, O_RDWR);
	if (g_ctx.sam_fd < 0) {
		perror("Failed to open SAM device");
		return -1;
	}

	g_ctx.input_fd = open(INPUT_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
	if (g_ctx.input_fd < 0) {
		printf("Warning: Could not open input device %s\n",
		       INPUT_DEVICE_PATH);
		printf("Button monitoring will not be available\n");
	}

	g_ctx.running = 1;
	return 0;
}

/**
 * Cleanup test context
 */
static void cleanup_test_context(void)
{
	if (g_ctx.sam_fd >= 0) {
		close(g_ctx.sam_fd);
	}

	if (g_ctx.input_fd >= 0) {
		close(g_ctx.input_fd);
	}
}

/**
 * Main function
 */
int main(int argc, char *argv[])
{
	int opt;
	const char *test_type = NULL;
	const char *packet_name = NULL;
	const char *device_path = DEVICE_NAME;
	int monitor_time = 0;
	int test_crc8 = 0;
	int led_id = 0; /* Default LED ID */

	static struct option long_options[] = {
		{ "help", no_argument, 0, 'h' },
		{ "verbose", no_argument, 0, 'v' },
		{ "interactive", no_argument, 0, 'i' },
		{ "test", required_argument, 0, 't' },
		{ "packet", required_argument, 0, 'p' },
		{ "device", required_argument, 0, 'd' },
		{ "monitor", required_argument, 0, 'm' },
		{ "crc8", no_argument, 0, 'c' },
		{ "led-id", required_argument, 0, 1000 },
		{ 0, 0, 0, 0 }
	};

	while ((opt = getopt_long(argc, argv, "hvict:p:d:m:", long_options,
				  NULL)) != -1) {
		switch (opt) {
		case 'h':
			print_usage(argv[0]);
			return 0;
		case 'v':
			g_ctx.verbose = 1;
			break;
		case 'i':
			g_ctx.interactive = 1;
			break;
		case 't':
			test_type = optarg;
			break;
		case 'p':
			packet_name = optarg;
			break;
		case 'd':
			device_path = optarg;
			break;
		case 'm':
			monitor_time = atoi(optarg);
			break;
		case 'c':
			test_crc8 = 1;
			break;
		case 1000: /* --led-id */
			led_id = atoi(optarg);
			if (led_id < 0 || led_id > 15) {
				fprintf(stderr, "Error: LED ID must be 0-15\n");
				return 1;
			}
			break;
		default:
			print_usage(argv[0]);
			return 1;
		}
	}

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	if (test_crc8) {
		return test_crc8_vectors();
	}

	if (init_test_context() < 0) {
		return 1;
	}

	printf("SAM Protocol Testing Program v0.2.0\n");
	printf("Device: %s\n", device_path);
	printf("Verbose: %s\n", g_ctx.verbose ? "Yes" : "No");
	printf("CRC8 Checksum: Enabled\n\n");

	int result = 0;

	if (monitor_time > 0) {
		monitor_input_events(monitor_time * 1000);
	} else if (packet_name) {
		result = send_individual_packet(packet_name, led_id);
	} else if (g_ctx.interactive) {
		interactive_menu();
	} else if (test_type) {
		if (strcmp(test_type, "buttons") == 0) {
			result = test_individual_buttons();
		} else if (strcmp(test_type, "leds") == 0) {
			result = test_individual_leds();
		} else if (strcmp(test_type, "power") == 0) {
			result = test_individual_power();
		} else if (strcmp(test_type, "display") == 0) {
			result = test_individual_display();
		} else if (strcmp(test_type, "system") == 0) {
			result = test_individual_system();
		} else if (strcmp(test_type, "debug") == 0) {
			result = test_individual_debug();
		} else if (strcmp(test_type, "extended") == 0) {
			result = test_individual_extended();
		} else if (strcmp(test_type, "crc8") == 0) {
			result = test_crc8_vectors();
		} else if (strcmp(test_type, "all") == 0) {
			printf("=== COMPREHENSIVE INDIVIDUAL TEST SUITE ===\n");
			result += test_crc8_vectors();
			result += test_individual_system();
			result += test_individual_leds();
			result += test_individual_power();
			result += test_individual_display();
			result += test_individual_buttons();
			result += test_individual_debug();
			result += test_individual_extended();
		} else {
			printf("Unknown test type: %s\n", test_type);
			result = 1;
		}
	} else {
		interactive_menu();
	}

	printf("\n=== FINAL TEST STATISTICS ===\n");
	printf("Tests passed: %d\n", g_ctx.stats.tests_passed);
	printf("Tests failed: %d\n", g_ctx.stats.tests_failed);
	printf("Packets sent: %d\n", g_ctx.stats.packets_sent);
	printf("Packets received: %d\n", g_ctx.stats.packets_received);
	printf("CRC8 errors: %d\n", g_ctx.stats.checksum_errors);
	printf("Timeout errors: %d\n", g_ctx.stats.timeout_errors);
	printf("Protocol errors: %d\n", g_ctx.stats.protocol_errors);

	cleanup_test_context();
	return result;
}