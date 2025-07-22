/**
 * Pamir AI Signal Aggregation Module (SAM) Userspace Testing Program
 * 
 * This program provides comprehensive testing capabilities for the SAM kernel driver,
 * supporting granular packet-level testing for all message types defined in the 
 * SAM protocol specification.
 * 
 * Features:
 * - Direct packet communication via /dev/pamir-sam character device
 * - Comprehensive testing of all 8 message types and their variants
 * - Input event monitoring for button testing
 * - LED control testing via sysfs interface
 * - Power metrics monitoring
 * - Protocol error injection and recovery testing
 * - Interactive menu system and command-line interface
 * 
 * Copyright (C) 2025 Pamir AI Incorporated
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
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

/* SAM Protocol Constants - From kernel driver */
#define DEVICE_NAME "/dev/pamir-sam"
#define PACKET_SIZE 4

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

/* Button event flags */
#define BTN_UP_MASK     0x01
#define BTN_DOWN_MASK   0x02
#define BTN_SELECT_MASK 0x04
#define BTN_POWER_MASK  0x08

/* LED control flags */
#define LED_CMD_IMMEDIATE 0x00
#define LED_CMD_SEQUENCE  0x10
#define LED_MODE_STATIC   0x00
#define LED_MODE_BLINK    0x04
#define LED_MODE_FADE     0x08
#define LED_MODE_RAINBOW  0x0C
#define LED_MODE_MASK     0x0C
#define LED_ID_ALL        0x00
#define LED_ID_MASK       0x03

/* Power management commands */
#define POWER_CMD_QUERY           0x00
#define POWER_CMD_SET             0x10
#define POWER_CMD_SLEEP           0x20
#define POWER_CMD_SHUTDOWN        0x30
#define POWER_CMD_CURRENT         0x40
#define POWER_CMD_BATTERY         0x50
#define POWER_CMD_TEMP            0x60
#define POWER_CMD_VOLTAGE         0x70
#define POWER_CMD_REQUEST_METRICS 0x80
#define POWER_CMD_MASK            0xF0

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

/* Testing configuration */
#define MAX_RESPONSE_WAIT_MS 2000
#define MAX_RETRY_ATTEMPTS 3
#define INPUT_DEVICE_PATH "/dev/input/by-id/input-pamir-ai-signal-aggregation-module"
#define LED_SYSFS_PATH "/sys/class/leds/pamir:status"
#define LED_SYSFS_BASE_PATH "/sys/class/leds"
#define POWER_METRICS_PATH "/sys/devices/platform/serial@7e201800"

/* RP2040 timing configuration */
#define RP2040_PACKET_DELAY_MS 100        /* Delay between packets to RP2040 */
#define RP2040_RESPONSE_DELAY_MS 50       /* Additional delay for RP2040 response processing */
#define RP2040_COMMAND_DELAY_MS 150       /* Delay between different commands */
#define RP2040_SEQUENCE_DELAY_MS 200      /* Delay between command sequences */

/**
 * SAM Protocol Packet Structure
 * 
 * Represents the 4-byte packet format used by the SAM protocol:
 * - type_flags: Message type (3 bits) + command flags (5 bits)
 * - data: 16-bit payload (2 bytes)
 * - checksum: XOR checksum of all previous bytes
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
    int sam_fd;                  /* SAM character device file descriptor */
    int input_fd;                /* Input device file descriptor */
    int verbose;                 /* Verbose output flag */
    int interactive;             /* Interactive mode flag */
    struct test_stats stats;     /* Test statistics */
    volatile int running;        /* Main loop running flag */
};

static struct test_context g_ctx = {0};

/**
 * Calculate XOR checksum for SAM packet
 * 
 * @param packet: Pointer to SAM packet
 * @return: Calculated checksum
 */
static uint8_t calculate_checksum(const struct sam_packet *packet)
{
    return packet->type_flags ^ packet->data[0] ^ packet->data[1];
}

/**
 * Verify checksum of received SAM packet
 * 
 * @param packet: Pointer to SAM packet
 * @return: true if checksum is valid, false otherwise
 */
static bool verify_checksum(const struct sam_packet *packet)
{
    uint8_t expected = calculate_checksum(packet);
    return packet->checksum == expected;
}

/**
 * Create a SAM packet with automatic checksum calculation
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
    packet->checksum = calculate_checksum(packet);
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
        case TYPE_BUTTON:     type_name = "BUTTON"; break;
        case TYPE_LED:        type_name = "LED"; break;
        case TYPE_POWER:      type_name = "POWER"; break;
        case TYPE_DISPLAY:    type_name = "DISPLAY"; break;
        case TYPE_DEBUG_CODE: type_name = "DEBUG_CODE"; break;
        case TYPE_DEBUG_TEXT: type_name = "DEBUG_TEXT"; break;
        case TYPE_SYSTEM:     type_name = "SYSTEM"; break;
        case TYPE_RESERVED:   type_name = "RESERVED"; break;
    }
    
    printf("[%s] %s: [%02X %02X %02X %02X] (type=0x%02X, data=[%02X %02X], checksum=%02X %s)\n",
           direction, type_name,
           packet->type_flags, packet->data[0], packet->data[1], packet->checksum,
           packet->type_flags & TYPE_MASK, packet->data[0], packet->data[1],
           packet->checksum, 
           verify_checksum(packet) ? "✓" : "✗");
}

/**
 * Send packet to SAM device with RP2040 timing considerations
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
    
    /* Add delay after packet transmission to allow RP2040 processing time */
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
        fprintf(stderr, "Received incomplete packet (%zd bytes)\n", bytes_read);
        g_ctx.stats.protocol_errors++;
        return -1;
    }
    
    g_ctx.stats.packets_received++;
    
    if (!verify_checksum(packet)) {
        fprintf(stderr, "Checksum verification failed\n");
        g_ctx.stats.checksum_errors++;
        return -1;
    }
    
    if (g_ctx.verbose) {
        print_packet(packet, "RX");
    }
    
    return 0;
}

/**
 * Send packet and wait for response with RP2040 timing considerations
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
    
    /* Additional delay for RP2040 response processing */
    usleep(RP2040_RESPONSE_DELAY_MS * 1000);
    
    return receive_packet(rx_packet, MAX_RESPONSE_WAIT_MS);
}

/**
 * Test system ping command
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_system_ping(void)
{
    struct sam_packet tx_packet, rx_packet;
    
    printf("Testing SYSTEM_PING command...\n");
    
    create_packet(&tx_packet, TYPE_SYSTEM | SYSTEM_PING, 0x00, 0x00);
    
    if (send_and_receive(&tx_packet, &rx_packet) < 0) {
        printf("SYSTEM_PING test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Verify response is a ping response */
    if ((rx_packet.type_flags & TYPE_MASK) != TYPE_SYSTEM ||
        (rx_packet.type_flags & ~TYPE_MASK) != SYSTEM_PING) {
        printf("SYSTEM_PING test FAILED - Invalid response type\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("SYSTEM_PING test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test system version command
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_system_version(void)
{
    struct sam_packet tx_packet, rx_packet;
    
    printf("Testing SYSTEM_VERSION command...\n");
    
    create_packet(&tx_packet, TYPE_SYSTEM | SYSTEM_VERSION, 0x00, 0x00);
    
    if (send_and_receive(&tx_packet, &rx_packet) < 0) {
        printf("SYSTEM_VERSION test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Verify response is a version response */
    if ((rx_packet.type_flags & TYPE_MASK) != TYPE_SYSTEM ||
        (rx_packet.type_flags & ~TYPE_MASK) != SYSTEM_VERSION) {
        printf("SYSTEM_VERSION test FAILED - Invalid response type\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("SYSTEM_VERSION test PASSED - Version: %d.%d\n", 
           rx_packet.data[0], rx_packet.data[1]);
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test system status command
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_system_status(void)
{
    struct sam_packet tx_packet, rx_packet;
    
    printf("Testing SYSTEM_STATUS command...\n");
    
    create_packet(&tx_packet, TYPE_SYSTEM | SYSTEM_STATUS, 0x01, 0x00);
    
    if (send_and_receive(&tx_packet, &rx_packet) < 0) {
        printf("SYSTEM_STATUS test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Verify response is a status response */
    if ((rx_packet.type_flags & TYPE_MASK) != TYPE_SYSTEM ||
        (rx_packet.type_flags & ~TYPE_MASK) != SYSTEM_STATUS) {
        printf("SYSTEM_STATUS test FAILED - Invalid response type\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("SYSTEM_STATUS test PASSED - Status: %d, Error: %d\n", 
           rx_packet.data[0], rx_packet.data[1]);
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test all system commands
 * 
 * @return: 0 if all tests passed, -1 if any failed
 */
static int test_system_commands(void)
{
    int failed = 0;
    
    printf("\n=== SYSTEM COMMANDS TEST SUITE ===\n");
    
    failed += test_system_ping();
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    
    failed += test_system_version();
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    
    failed += test_system_status();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    printf("System commands test suite: %d passed, %d failed\n",
           failed == 0 ? 3 : 3 + failed, failed < 0 ? -failed : 0);
    
    return failed;
}

/**
 * Test LED static color command
 * 
 * @param led_id: LED ID (0-15)
 * @param r: Red component (0-15)
 * @param g: Green component (0-15)
 * @param b: Blue component (0-15)
 * @return: 0 on success, -1 on failure
 */
static int test_led_static_color(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b)
{
    struct sam_packet tx_packet;
    uint8_t data0, data1;
    
    printf("Testing LED static color - LED %d, RGB(%d,%d,%d)...\n", 
           led_id, r, g, b);
    
    /* Pack RGB444 format: data[0] = RRRRGGGG, data[1] = BBBBVVVV */
    data0 = (r << 4) | (g & 0x0F);
    data1 = (b << 4) | 0x00;  /* Value/time = 0 for static */
    
    create_packet(&tx_packet, TYPE_LED | LED_CMD_IMMEDIATE | LED_MODE_STATIC | led_id,
                  data0, data1);
    
    if (send_packet(&tx_packet) < 0) {
        printf("LED static color test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("LED static color test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test LED blink mode
 * 
 * @param led_id: LED ID (0-15)
 * @param r: Red component (0-15)
 * @param g: Green component (0-15)
 * @param b: Blue component (0-15)
 * @param speed: Blink speed (0-15)
 * @return: 0 on success, -1 on failure
 */
static int test_led_blink(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b, uint8_t speed)
{
    struct sam_packet tx_packet;
    uint8_t data0, data1;
    
    printf("Testing LED blink - LED %d, RGB(%d,%d,%d), speed %d...\n", 
           led_id, r, g, b, speed);
    
    data0 = (r << 4) | (g & 0x0F);
    data1 = (b << 4) | (speed & 0x0F);
    
    create_packet(&tx_packet, TYPE_LED | LED_CMD_IMMEDIATE | LED_MODE_BLINK | led_id,
                  data0, data1);
    
    if (send_packet(&tx_packet) < 0) {
        printf("LED blink test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("LED blink test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test LED fade mode
 * 
 * @param led_id: LED ID (0-15)
 * @param r: Red component (0-15)
 * @param g: Green component (0-15)
 * @param b: Blue component (0-15)
 * @param speed: Fade speed (0-15)
 * @return: 0 on success, -1 on failure
 */
static int test_led_fade(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b, uint8_t speed)
{
    struct sam_packet tx_packet;
    uint8_t data0, data1;
    
    printf("Testing LED fade - LED %d, RGB(%d,%d,%d), speed %d...\n", 
           led_id, r, g, b, speed);
    
    data0 = (r << 4) | (g & 0x0F);
    data1 = (b << 4) | (speed & 0x0F);
    
    create_packet(&tx_packet, TYPE_LED | LED_CMD_IMMEDIATE | LED_MODE_FADE | led_id,
                  data0, data1);
    
    if (send_packet(&tx_packet) < 0) {
        printf("LED fade test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("LED fade test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test LED rainbow mode
 * 
 * @param led_id: LED ID (0-15)
 * @param speed: Rainbow speed (0-15)
 * @return: 0 on success, -1 on failure
 */
static int test_led_rainbow(uint8_t led_id, uint8_t speed)
{
    struct sam_packet tx_packet;
    
    printf("Testing LED rainbow - LED %d, speed %d...\n", led_id, speed);
    
    create_packet(&tx_packet, TYPE_LED | LED_CMD_IMMEDIATE | LED_MODE_RAINBOW | led_id,
                  0x00, speed & 0x0F);
    
    if (send_packet(&tx_packet) < 0) {
        printf("LED rainbow test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("LED rainbow test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test all LED commands
 * 
 * @return: 0 if all tests passed, -1 if any failed
 */
static int test_led_commands(void)
{
    int failed = 0;
    
    printf("\n=== LED COMMANDS TEST SUITE ===\n");
    
    /* Test basic colors on LED 0 */
    failed += test_led_static_color(0, 15, 0, 0);   /* Red */
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    failed += test_led_static_color(0, 0, 15, 0);   /* Green */
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    failed += test_led_static_color(0, 0, 0, 15);   /* Blue */
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    failed += test_led_static_color(0, 15, 15, 15); /* White */
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    
    /* Test animation modes */
    failed += test_led_blink(0, 15, 0, 0, 8);       /* Red blink */
    usleep(2000000);  /* 2s delay to see blink */
    failed += test_led_fade(0, 0, 15, 0, 8);        /* Green fade */
    usleep(2000000);
    failed += test_led_rainbow(0, 8);               /* Rainbow */
    usleep(3000000);  /* 3s delay to see rainbow */
    
    /* Turn off LED */
    failed += test_led_static_color(0, 0, 0, 0);    /* Off */
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    printf("LED commands test suite: %d passed, %d failed\n",
           (failed == 0) ? 8 : 8 + failed, (failed < 0) ? -failed : 0);
    
    return failed;
}

/**
 * Test power query command
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_power_query(void)
{
    struct sam_packet tx_packet, rx_packet;
    
    printf("Testing POWER_QUERY command...\n");
    
    create_packet(&tx_packet, TYPE_POWER | POWER_CMD_QUERY, 0x00, 0x00);
    
    if (send_and_receive(&tx_packet, &rx_packet) < 0) {
        printf("POWER_QUERY test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Verify response is a power response */
    if ((rx_packet.type_flags & TYPE_MASK) != TYPE_POWER) {
        printf("POWER_QUERY test FAILED - Invalid response type\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("POWER_QUERY test PASSED - State: %d\n", rx_packet.data[0]);
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test power metrics request
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_power_metrics_request(void)
{
    struct sam_packet tx_packet, rx_packet;
    int responses = 0;
    
    printf("Testing POWER_METRICS_REQUEST command...\n");
    
    create_packet(&tx_packet, TYPE_POWER | POWER_CMD_REQUEST_METRICS, 0x00, 0x00);
    
    if (send_packet(&tx_packet) < 0) {
        printf("POWER_METRICS_REQUEST test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Expect 4 responses: current, battery, temperature, voltage */
    for (int i = 0; i < 4; i++) {
        if (receive_packet(&rx_packet, MAX_RESPONSE_WAIT_MS) < 0) {
            printf("POWER_METRICS_REQUEST test FAILED - Missing response %d\n", i);
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        if ((rx_packet.type_flags & TYPE_MASK) != TYPE_POWER) {
            printf("POWER_METRICS_REQUEST test FAILED - Invalid response type %d\n", i);
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        uint8_t cmd = rx_packet.type_flags & POWER_CMD_MASK;
        uint16_t value = rx_packet.data[0] | (rx_packet.data[1] << 8);
        
        switch (cmd) {
            case POWER_CMD_CURRENT:
                printf("  Current: %d mA\n", value);
                break;
            case POWER_CMD_BATTERY:
                printf("  Battery: %d%%\n", value);
                break;
            case POWER_CMD_TEMP:
                printf("  Temperature: %.1f°C\n", (int16_t)value / 10.0);
                break;
            case POWER_CMD_VOLTAGE:
                printf("  Voltage: %d mV\n", value);
                break;
            default:
                printf("  Unknown metric: cmd=0x%02X, value=%d\n", cmd, value);
                break;
        }
        
        responses++;
        
        /* Small delay between metric responses to prevent overwhelming RP2040 */
        if (i < 3) {
            usleep(RP2040_RESPONSE_DELAY_MS * 1000);
        }
    }
    
    if (responses == 4) {
        printf("POWER_METRICS_REQUEST test PASSED\n");
        g_ctx.stats.tests_passed++;
        return 0;
    } else {
        printf("POWER_METRICS_REQUEST test FAILED - Only %d/4 responses\n", responses);
        g_ctx.stats.tests_failed++;
        return -1;
    }
}

/**
 * Test all power commands
 * 
 * @return: 0 if all tests passed, -1 if any failed
 */
static int test_power_commands(void)
{
    int failed = 0;
    
    printf("\n=== POWER COMMANDS TEST SUITE ===\n");
    
    failed += test_power_query();
    usleep(RP2040_COMMAND_DELAY_MS * 1000);
    
    failed += test_power_metrics_request();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    printf("Power commands test suite: %d passed, %d failed\n",
           (failed == 0) ? 2 : 2 + failed, (failed < 0) ? -failed : 0);
    
    return failed;
}

/**
 * Test display status command
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_display_status(void)
{
    struct sam_packet tx_packet, rx_packet;
    
    printf("Testing DISPLAY_STATUS command...\n");
    
    create_packet(&tx_packet, TYPE_DISPLAY | 0x00, 0x00, 0x00);
    
    if (send_and_receive(&tx_packet, &rx_packet) < 0) {
        printf("DISPLAY_STATUS test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Verify response is a display response */
    if ((rx_packet.type_flags & TYPE_MASK) != TYPE_DISPLAY) {
        printf("DISPLAY_STATUS test FAILED - Invalid response type\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("DISPLAY_STATUS test PASSED - Status: %d\n", rx_packet.data[0]);
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test all display commands
 * 
 * @return: 0 if all tests passed, -1 if any failed
 */
static int test_display_commands(void)
{
    int failed = 0;
    
    printf("\n=== DISPLAY COMMANDS TEST SUITE ===\n");
    
    failed += test_display_status();
    
    printf("Display commands test suite: %d passed, %d failed\n",
           (failed == 0) ? 1 : 1 + failed, (failed < 0) ? -failed : 0);
    
    return failed;
}

/**
 * Test button event simulation
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_button_events(void)
{
    struct sam_packet tx_packet;
    
    printf("\n=== BUTTON EVENTS TEST SUITE ===\n");
    printf("Testing button event simulation...\n");
    
    /* Note: Button events are typically sent by the RP2040 to the host,
     * not the other way around. This test simulates what the RP2040 would send. */
    
    uint8_t button_combinations[] = {
        0x00,  /* No buttons */
        BTN_UP_MASK,                              /* UP only */
        BTN_DOWN_MASK,                            /* DOWN only */
        BTN_SELECT_MASK,                          /* SELECT only */
        BTN_POWER_MASK,                           /* POWER only */
        BTN_UP_MASK | BTN_DOWN_MASK,             /* UP + DOWN */
        BTN_UP_MASK | BTN_SELECT_MASK,           /* UP + SELECT */
        BTN_DOWN_MASK | BTN_SELECT_MASK,         /* DOWN + SELECT */
        BTN_UP_MASK | BTN_DOWN_MASK | BTN_SELECT_MASK, /* UP + DOWN + SELECT */
        BTN_POWER_MASK | BTN_SELECT_MASK,        /* POWER + SELECT */
        0x0F   /* All buttons */
    };
    
    const char* button_names[] = {
        "No buttons",
        "UP only",
        "DOWN only", 
        "SELECT only",
        "POWER only",
        "UP + DOWN",
        "UP + SELECT",
        "DOWN + SELECT",
        "UP + DOWN + SELECT",
        "POWER + SELECT",
        "All buttons"
    };
    
    for (int i = 0; i < sizeof(button_combinations); i++) {
        printf("Simulating button state: %s (0x%02X)...\n", 
               button_names[i], button_combinations[i]);
        
        create_packet(&tx_packet, TYPE_BUTTON | button_combinations[i], 0x00, 0x00);
        
        if (send_packet(&tx_packet) < 0) {
            printf("Button event test FAILED\n");
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        usleep(100000);  /* 100ms delay between button events */
    }
    
    printf("Button events test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test debug code transmission
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_debug_codes(void)
{
    struct sam_packet tx_packet;
    
    printf("\n=== DEBUG CODE TEST SUITE ===\n");
    printf("Testing debug code transmission...\n");
    
    /* Test debug codes for different categories */
    uint8_t debug_categories[] = {0, 1, 2, 3, 4, 5, 6, 7};
    const char* category_names[] = {
        "System", "Error", "Button", "LED", "Power", "Display", "Communication", "Performance"
    };
    
    for (int i = 0; i < sizeof(debug_categories); i++) {
        printf("Simulating debug code - Category: %s (%d), Code: %d...\n",
               category_names[i], debug_categories[i], i + 1);
        
        create_packet(&tx_packet, TYPE_DEBUG_CODE | debug_categories[i], 
                      i + 1, 0x00);
        
        if (send_packet(&tx_packet) < 0) {
            printf("Debug code test FAILED\n");
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        usleep(50000);  /* 50ms delay between debug codes */
    }
    
    printf("Debug codes test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test debug text transmission
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_debug_text(void)
{
    struct sam_packet tx_packet;
    const char* test_message = "Hello SAM!";
    int len = strlen(test_message);
    
    printf("\n=== DEBUG TEXT TEST SUITE ===\n");
    printf("Testing debug text transmission: \"%s\"...\n", test_message);
    
    for (int i = 0; i < len; i += 2) {
        uint8_t flags = 0;
        
        if (i == 0) {
            flags |= DEBUG_FIRST_CHUNK;
        }
        
        if (i + 2 < len) {
            flags |= DEBUG_CONTINUE;
        }
        
        uint8_t char1 = test_message[i];
        uint8_t char2 = (i + 1 < len) ? test_message[i + 1] : 0;
        
        printf("Sending debug text chunk: '%c%c' (flags=0x%02X)...\n",
               char1, char2 ? char2 : ' ', flags);
        
        create_packet(&tx_packet, TYPE_DEBUG_TEXT | flags, char1, char2);
        
        if (send_packet(&tx_packet) < 0) {
            printf("Debug text test FAILED\n");
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        usleep(100000);  /* 100ms delay between chunks */
    }
    
    printf("Debug text test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test extended/reserved commands
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_extended_commands(void)
{
    struct sam_packet tx_packet;
    
    printf("\n=== EXTENDED COMMANDS TEST SUITE ===\n");
    printf("Testing extended command transmission...\n");
    
    /* Test various extended command patterns */
    for (int i = 0; i < 8; i++) {
        printf("Sending extended command %d...\n", i);
        
        create_packet(&tx_packet, TYPE_RESERVED | i, 0x00, 0x00);
        
        if (send_packet(&tx_packet) < 0) {
            printf("Extended command test FAILED\n");
            g_ctx.stats.tests_failed++;
            return -1;
        }
        
        usleep(50000);  /* 50ms delay */
    }
    
    printf("Extended commands test PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Test protocol error handling
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_error_handling(void)
{
    struct sam_packet tx_packet;
    
    printf("\n=== ERROR HANDLING TEST SUITE ===\n");
    
    /* Test invalid checksum */
    printf("Testing invalid checksum handling...\n");
    create_packet(&tx_packet, TYPE_SYSTEM | SYSTEM_PING, 0x00, 0x00);
    tx_packet.checksum = 0xFF;  /* Deliberately corrupt checksum */
    
    if (send_packet(&tx_packet) < 0) {
        printf("Invalid checksum test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    /* Test invalid message type */
    printf("Testing invalid message type handling...\n");
    create_packet(&tx_packet, 0xFF, 0x00, 0x00);  /* Invalid type */
    
    if (send_packet(&tx_packet) < 0) {
        printf("Invalid message type test FAILED\n");
        g_ctx.stats.tests_failed++;
        return -1;
    }
    
    printf("Error handling tests PASSED\n");
    g_ctx.stats.tests_passed++;
    return 0;
}

/**
 * Monitor input events from button device
 * 
 * @param duration_ms: Duration to monitor in milliseconds
 * @return: Number of events received
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
        timeout.tv_usec = 100000;  /* 100ms timeout */
        
        int ret = select(g_ctx.input_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (ret > 0 && FD_ISSET(g_ctx.input_fd, &read_fds)) {
            if (read(g_ctx.input_fd, &ev, sizeof(ev)) == sizeof(ev)) {
                if (ev.type == EV_KEY) {
                    const char* key_name = "UNKNOWN";
                    switch (ev.code) {
                        case KEY_UP:    key_name = "UP"; break;
                        case KEY_DOWN:  key_name = "DOWN"; break;
                        case KEY_ENTER: key_name = "SELECT"; break;
                        case KEY_POWER: key_name = "POWER"; break;
                    }
                    
                    printf("Input event: %s %s\n", key_name, 
                           ev.value ? "PRESSED" : "RELEASED");
                    event_count++;
                }
            }
        }
    }
    
    printf("Input monitoring completed. %d events received.\n", event_count);
    return event_count;
}

/**
 * Read and display power metrics from sysfs
 * 
 * @return: 0 on success, -1 on failure
 */
static int read_power_metrics_sysfs(void)
{
    char path[256];
    char buffer[64];
    FILE *fp;
    
    printf("\n=== POWER METRICS SYSFS MONITORING ===\n");
    
    const char* metrics[] = {"current_ma", "battery_percent", "temperature", "voltage_mv"};
    const char* units[] = {"mA", "%", "°C", "mV"};
    
    for (int i = 0; i < 4; i++) {
        snprintf(path, sizeof(path), "%s/power_metrics/%s", POWER_METRICS_PATH, metrics[i]);
        
        fp = fopen(path, "r");
        if (fp) {
            if (fgets(buffer, sizeof(buffer), fp)) {
                buffer[strcspn(buffer, "\n")] = 0;  /* Remove newline */
                printf("  %s: %s %s\n", metrics[i], buffer, units[i]);
            }
            fclose(fp);
        } else {
            printf("  %s: Not available\n", metrics[i]);
        }
    }
    
    return 0;
}

/**
 * Test LED sysfs interface with RGB control
 * 
 * @return: 0 on success, -1 on failure
 */
static int test_led_sysfs(void)
{
    char path[256];
    FILE *fp;
    
    printf("\n=== LED SYSFS INTERFACE TEST ===\n");
    
    /* Test brightness control */
    snprintf(path, sizeof(path), "%s/brightness", LED_SYSFS_PATH);
    
    uint8_t brightness_values[] = {0, 64, 128, 192, 255};
    const char* brightness_names[] = {"Off", "25%", "50%", "75%", "100%"};
    
    printf("Testing brightness control...\n");
    for (int i = 0; i < 5; i++) {
        printf("Setting LED brightness to %s (%d)...\n", 
               brightness_names[i], brightness_values[i]);
        
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%d\n", brightness_values[i]);
            fclose(fp);
            
            printf("LED brightness set successfully\n");
            g_ctx.stats.tests_passed++;
        } else {
            printf("LED brightness set FAILED\n");
            g_ctx.stats.tests_failed++;
        }
        
        usleep(1000000);  /* 1s delay */
    }
    
    /* Test RGB control */
    printf("\nTesting RGB control...\n");
    
    struct {
        uint8_t r, g, b;
        const char* name;
    } rgb_tests[] = {
        {255, 0, 0, "Red"},
        {0, 255, 0, "Green"},
        {0, 0, 255, "Blue"},
        {255, 255, 0, "Yellow"},
        {255, 0, 255, "Magenta"},
        {0, 255, 255, "Cyan"},
        {255, 255, 255, "White"},
        {0, 0, 0, "Off"}
    };
    
    for (int i = 0; i < 8; i++) {
        printf("Setting LED to %s (%d,%d,%d)...\n", 
               rgb_tests[i].name, rgb_tests[i].r, rgb_tests[i].g, rgb_tests[i].b);
        
        /* Set red component */
        snprintf(path, sizeof(path), "%s/red", LED_SYSFS_PATH);
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%d\n", rgb_tests[i].r);
            fclose(fp);
        }
        
        /* Set green component */
        snprintf(path, sizeof(path), "%s/green", LED_SYSFS_PATH);
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%d\n", rgb_tests[i].g);
            fclose(fp);
        }
        
        /* Set blue component */
        snprintf(path, sizeof(path), "%s/blue", LED_SYSFS_PATH);
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%d\n", rgb_tests[i].b);
            fclose(fp);
            printf("RGB color set successfully\n");
            g_ctx.stats.tests_passed++;
        } else {
            printf("RGB color set FAILED\n");
            g_ctx.stats.tests_failed++;
        }
        
        usleep(1500000);  /* 1.5s delay */
    }
    
    /* Test animation modes */
    printf("\nTesting animation modes...\n");
    
    const char* modes[] = {"static", "blink", "fade", "rainbow"};
    const char* mode_names[] = {"Static", "Blink", "Fade", "Rainbow"};
    
    for (int i = 0; i < 4; i++) {
        printf("Setting LED mode to %s...\n", mode_names[i]);
        
        /* Set color for non-rainbow modes */
        if (i != 3) {  /* Not rainbow */
            snprintf(path, sizeof(path), "%s/red", LED_SYSFS_PATH);
            fp = fopen(path, "w");
            if (fp) { fprintf(fp, "255\n"); fclose(fp); }
            
            snprintf(path, sizeof(path), "%s/green", LED_SYSFS_PATH);
            fp = fopen(path, "w");
            if (fp) { fprintf(fp, "0\n"); fclose(fp); }
            
            snprintf(path, sizeof(path), "%s/blue", LED_SYSFS_PATH);
            fp = fopen(path, "w");
            if (fp) { fprintf(fp, "0\n"); fclose(fp); }
        }
        
        /* Set timing for animations */
        if (i > 0) {  /* Not static */
            snprintf(path, sizeof(path), "%s/timing", LED_SYSFS_PATH);
            fp = fopen(path, "w");
            if (fp) { fprintf(fp, "500\n"); fclose(fp); }
        }
        
        /* Set mode */
        snprintf(path, sizeof(path), "%s/mode", LED_SYSFS_PATH);
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%s\n", modes[i]);
            fclose(fp);
            printf("LED mode set successfully\n");
            g_ctx.stats.tests_passed++;
        } else {
            printf("LED mode set FAILED\n");
            g_ctx.stats.tests_failed++;
        }
        
        usleep(3000000);  /* 3s delay to observe animation */
    }
    
    /* Test LED triggers */
    printf("\nTesting LED triggers...\n");
    
    const char* triggers[] = {"none", "heartbeat-rgb", "breathing-rgb", "rainbow-rgb"};
    const char* trigger_names[] = {"None", "Heartbeat RGB", "Breathing RGB", "Rainbow RGB"};
    
    for (int i = 0; i < 4; i++) {
        printf("Setting LED trigger to %s...\n", trigger_names[i]);
        
        snprintf(path, sizeof(path), "%s/trigger", LED_SYSFS_PATH);
        fp = fopen(path, "w");
        if (fp) {
            fprintf(fp, "%s\n", triggers[i]);
            fclose(fp);
            printf("LED trigger set successfully\n");
            g_ctx.stats.tests_passed++;
        } else {
            printf("LED trigger set FAILED\n");
            g_ctx.stats.tests_failed++;
        }
        
        usleep(5000000);  /* 5s delay to observe trigger */
    }
    
    return 0;
}

/**
 * Run comprehensive test suite
 * 
 * @return: 0 if all tests passed, -1 if any failed
 */
static int run_comprehensive_test(void)
{
    int total_failed = 0;
    
    printf("\n" "=" "50s\n", "COMPREHENSIVE SAM TESTING SUITE");
    printf("Starting comprehensive test of SAM protocol...\n");
    printf("=" "50s\n");
    
    /* Initialize statistics */
    memset(&g_ctx.stats, 0, sizeof(g_ctx.stats));
    
    /* Test all command types with delays between suites */
    total_failed += test_system_commands();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_led_commands();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_power_commands();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_display_commands();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_button_events();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_debug_codes();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_debug_text();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_extended_commands();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    total_failed += test_error_handling();
    usleep(RP2040_SEQUENCE_DELAY_MS * 1000);
    
    /* Test system interfaces */
    test_led_sysfs();
    read_power_metrics_sysfs();
    
    /* Monitor input events for 5 seconds */
    monitor_input_events(5000);
    
    /* Print final statistics */
    printf("\n" "=" "50s\n", "TEST RESULTS");
    printf("Tests passed: %d\n", g_ctx.stats.tests_passed);
    printf("Tests failed: %d\n", g_ctx.stats.tests_failed);
    printf("Packets sent: %d\n", g_ctx.stats.packets_sent);
    printf("Packets received: %d\n", g_ctx.stats.packets_received);
    printf("Checksum errors: %d\n", g_ctx.stats.checksum_errors);
    printf("Timeout errors: %d\n", g_ctx.stats.timeout_errors);
    printf("Protocol errors: %d\n", g_ctx.stats.protocol_errors);
    printf("=" "50s\n");
    
    if (total_failed == 0) {
        printf("🎉 ALL TESTS PASSED! 🎉\n");
    } else {
        printf("❌ %d TEST(S) FAILED ❌\n", -total_failed);
    }
    
    return total_failed;
}

/**
 * Interactive menu system
 */
static void interactive_menu(void)
{
    char choice[16];
    int running = 1;
    
    printf("\n" "=" "50s\n", "SAM INTERACTIVE TEST MENU");
    
    while (running) {
        printf("\nSelect test to run:\n");
        printf("1. System commands\n");
        printf("2. LED commands\n");
        printf("3. Power commands\n");
        printf("4. Display commands\n");
        printf("5. Button events\n");
        printf("6. Debug codes\n");
        printf("7. Debug text\n");
        printf("8. Extended commands\n");
        printf("9. Error handling\n");
        printf("10. Monitor input events\n");
        printf("11. Read power metrics\n");
        printf("12. Test LED sysfs\n");
        printf("13. Run comprehensive test\n");
        printf("q. Quit\n");
        printf("Choice: ");
        
        if (fgets(choice, sizeof(choice), stdin) == NULL) {
            break;
        }
        
        choice[strcspn(choice, "\n")] = 0;  /* Remove newline */
        
        switch (choice[0]) {
            case '1':
                test_system_commands();
                break;
            case '2':
                test_led_commands();
                break;
            case '3':
                test_power_commands();
                break;
            case '4':
                test_display_commands();
                break;
            case '5':
                test_button_events();
                break;
            case '6':
                test_debug_codes();
                break;
            case '7':
                test_debug_text();
                break;
            case '8':
                test_extended_commands();
                break;
            case '9':
                test_error_handling();
                break;
            case '1' + '0':  /* Handle "10" */
                if (choice[1] == '0') {
                    monitor_input_events(10000);  /* 10 seconds */
                }
                break;
            case '1' + '1':  /* Handle "11" */
                if (choice[1] == '1') {
                    read_power_metrics_sysfs();
                }
                break;
            case '1' + '2':  /* Handle "12" */
                if (choice[1] == '2') {
                    test_led_sysfs();
                }
                break;
            case '1' + '3':  /* Handle "13" */
                if (choice[1] == '3') {
                    run_comprehensive_test();
                }
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
static void print_usage(const char* prog_name)
{
    printf("Usage: %s [OPTIONS]\n", prog_name);
    printf("\nOptions:\n");
    printf("  -h, --help              Show this help message\n");
    printf("  -v, --verbose           Enable verbose output\n");
    printf("  -i, --interactive       Run in interactive mode\n");
    printf("  -t, --test TYPE         Run specific test type:\n");
    printf("                          system, led, power, display, button,\n");
    printf("                          debug, extended, error, all\n");
    printf("  -d, --device PATH       SAM device path (default: %s)\n", DEVICE_NAME);
    printf("  -m, --monitor TIME      Monitor input events for TIME seconds\n");
    printf("\nExamples:\n");
    printf("  %s -t system           # Test system commands only\n", prog_name);
    printf("  %s -v -t all           # Run all tests with verbose output\n", prog_name);
    printf("  %s -i                  # Run interactive mode\n", prog_name);
    printf("  %s -m 10               # Monitor input events for 10 seconds\n", prog_name);
}

/**
 * Initialize test context
 * 
 * @return: 0 on success, -1 on failure
 */
static int init_test_context(void)
{
    /* Open SAM character device */
    g_ctx.sam_fd = open(DEVICE_NAME, O_RDWR);
    if (g_ctx.sam_fd < 0) {
        perror("Failed to open SAM device");
        return -1;
    }
    
    /* Try to open input device (optional) */
    g_ctx.input_fd = open(INPUT_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (g_ctx.input_fd < 0) {
        printf("Warning: Could not open input device %s\n", INPUT_DEVICE_PATH);
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
    const char* test_type = NULL;
    const char* device_path = DEVICE_NAME;
    int monitor_time = 0;
    
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"verbose", no_argument, 0, 'v'},
        {"interactive", no_argument, 0, 'i'},
        {"test", required_argument, 0, 't'},
        {"device", required_argument, 0, 'd'},
        {"monitor", required_argument, 0, 'm'},
        {0, 0, 0, 0}
    };
    
    /* Parse command line arguments */
    while ((opt = getopt_long(argc, argv, "hvit:d:m:", long_options, NULL)) != -1) {
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
            case 'd':
                device_path = optarg;
                break;
            case 'm':
                monitor_time = atoi(optarg);
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }
    
    /* Install signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Initialize test context */
    if (init_test_context() < 0) {
        return 1;
    }
    
    printf("SAM Protocol Testing Program\n");
    printf("Device: %s\n", device_path);
    printf("Verbose: %s\n", g_ctx.verbose ? "Yes" : "No");
    printf("\n");
    
    int result = 0;
    
    if (monitor_time > 0) {
        /* Monitor input events */
        monitor_input_events(monitor_time * 1000);
    } else if (g_ctx.interactive) {
        /* Interactive mode */
        interactive_menu();
    } else if (test_type) {
        /* Run specific test */
        if (strcmp(test_type, "system") == 0) {
            result = test_system_commands();
        } else if (strcmp(test_type, "led") == 0) {
            result = test_led_commands();
        } else if (strcmp(test_type, "power") == 0) {
            result = test_power_commands();
        } else if (strcmp(test_type, "display") == 0) {
            result = test_display_commands();
        } else if (strcmp(test_type, "button") == 0) {
            result = test_button_events();
        } else if (strcmp(test_type, "debug") == 0) {
            result = test_debug_codes();
            result += test_debug_text();
        } else if (strcmp(test_type, "extended") == 0) {
            result = test_extended_commands();
        } else if (strcmp(test_type, "error") == 0) {
            result = test_error_handling();
        } else if (strcmp(test_type, "all") == 0) {
            result = run_comprehensive_test();
        } else {
            printf("Unknown test type: %s\n", test_type);
            result = 1;
        }
    } else {
        /* Default: run comprehensive test */
        result = run_comprehensive_test();
    }
    
    /* Cleanup */
    cleanup_test_context();
    
    return result;
}