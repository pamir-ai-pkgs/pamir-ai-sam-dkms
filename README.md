# Pamir AI Signal Aggregation Module (SAM) DKMS Driver

This package contains the DKMS (Dynamic Kernel Module Support) driver for the Pamir AI Signal Aggregation Module (SAM). The SAM driver enables bidirectional communication between a Linux host system (Raspberry Pi CM5) and an RP2040 microcontroller via UART, providing comprehensive hardware control and status reporting functionality.

## Overview

The Pamir AI SAM driver implements a highly optimized 4-byte packet protocol designed for embedded systems with limited resources. It provides a modular architecture with clearly defined components that interact through standardized interfaces, enabling reliable communication for hardware control and status monitoring.

## Features

### Hardware Interfaces
- **Button Input**: Physical button events via Linux input subsystem with hardware debouncing
- **LED Control**: RGB LED control with multiple animation modes (static, blink, fade, rainbow)
- **Power Management**: Complete power state coordination with automatic boot/shutdown notifications
- **E-ink Display**: Display control and status reporting with refresh management
- **Debug Interface**: Comprehensive diagnostic codes and text messages with circular buffering

### Protocol Support
- **4-byte Packet Format**: Ultra-optimized communication protocol with minimal overhead
- **Message Types**: 8 different message types covering all hardware functions
- **Error Detection**: XOR checksum validation with automatic recovery mechanisms
- **UART Communication**: 115200 baud, 8N1 format via UART2 (GPIO4/5)
- **Timing Constraints**: Sub-millisecond packet processing with configurable timeouts

### Linux Integration
- **Input Device**: Button events via `/dev/input/event*` with standard key codes
- **LED Class Device**: LED control via `/sys/class/leds/pamir:status/brightness`
- **Character Device**: Raw protocol access via `/dev/pamir-sam` for debugging
- **Power Metrics**: Sysfs interface for current, battery, temperature, and voltage monitoring
- **Debug Interface**: Kernel logging with configurable verbosity levels

## Installation

### Prerequisites

- Linux kernel headers for your current kernel
- DKMS package installed
- Device Tree Compiler (dtc)
- Root privileges
- Raspberry Pi CM5 with BCM2712 ARM64 architecture

### Install DKMS Module

```bash
sudo ./install.sh
```

### Enable the SAM Module

1. Add the device tree overlay to `/boot/config.txt`:
   ```
   dtoverlay=pamir-ai-sam
   ```

2. Configure UART2 in `/boot/config.txt` (if not already done):
   ```
   enable_uart=1
   dtoverlay=uart2
   ```

3. Reboot your system

## Usage

### Button Events

Monitor button events:
```bash
# Find the input device
cat /proc/bus/input/devices | grep -A 5 "Pamir AI SAM"

# Monitor events
sudo evtest /dev/input/event<N>
```

Button mappings:
- UP button → KEY_UP
- DOWN button → KEY_DOWN
- SELECT button → KEY_ENTER
- POWER button → KEY_POWER

### LED Control

Control LED via sysfs:
```bash
# Set LED brightness (0-255)
echo 128 | sudo tee /sys/class/leds/pamir:status/brightness

# Get current brightness
cat /sys/class/leds/pamir:status/brightness
```

Send LED commands via character device:
```bash
# Set LED to red (static mode)
echo -e "\x20\xF0\x00\xD0" | sudo tee /dev/pamir-sam

# Set LED to blink blue
echo -e "\x24\x00\xF8\xDC" | sudo tee /dev/pamir-sam
```

### Power Management

The driver automatically manages power states and provides access to power metrics:

```bash
# Send ping command to test communication
echo -e "\xC0\x00\x00\xC0" | sudo tee /dev/pamir-sam

# Query power status
echo -e "\x40\x00\x00\x40" | sudo tee /dev/pamir-sam

# Read power metrics via sysfs
cat /sys/devices/platform/serial@*/power_metrics/current_ma
cat /sys/devices/platform/serial@*/power_metrics/battery_percent
cat /sys/devices/platform/serial@*/power_metrics/temperature
cat /sys/devices/platform/serial@*/power_metrics/voltage_mv
```

Power management features:
- **Automatic Boot Notification**: Sent during driver initialization
- **Automatic Shutdown Notification**: Sent before system shutdown via reboot notifier
- **Power Metrics Polling**: Configurable interval for current, battery, temperature, and voltage
- **Sleep Mode Support**: Coordinated sleep states between host and microcontroller

### Debug Interface

Monitor debug messages and configure logging levels:
```bash
# Watch system logs for debug information
sudo journalctl -f | grep pamir-sam

# Enable verbose debugging
sudo rmmod pamir-ai-sam
sudo modprobe pamir-ai-sam debug=3

# View debug levels:
# 0 = SAM_DEBUG_OFF (no debug messages)
# 1 = SAM_DEBUG_ERROR (error messages only)
# 2 = SAM_DEBUG_INFO (basic info + errors)
# 3 = SAM_DEBUG_VERBOSE (detailed protocol information)
```

Debug features:
- **Debug Codes**: Compact diagnostic codes for predefined events
- **Debug Text**: Detailed text messages with multi-packet support
- **Protocol Monitoring**: Detailed packet-level debugging with checksum validation
- **Error Recovery**: Automatic recovery from communication errors with logging

## Configuration

### Device Tree Properties

The SAM module can be configured via device tree overlay parameters:

```dts
&serial1 {
    status = "okay";
    current-speed = <115200>;

    pamir_sam: pamir-sam {
        compatible = "pamir-ai,sam";
        debug-level = <1>;                /* Optional: 0=off, 1=error, 2=info, 3=verbose */
        ack-required = <0>;               /* Optional: Whether commands require ACK */
        recovery-timeout-ms = <1000>;     /* Optional: Recovery timeout in milliseconds */
        power-poll-interval-ms = <1000>;  /* Optional: Poll power metrics every 1000ms (0 to disable) */
    };
};
```

Available properties:
- **debug-level**: Controls logging verbosity (0-3)
- **ack-required**: Enable command acknowledgment requirement
- **recovery-timeout-ms**: Timeout for protocol recovery after errors
- **power-poll-interval-ms**: Interval for polling power metrics from microcontroller

### UART Configuration

The SAM protocol uses UART2 (GPIO4/5):
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Protocol Specification

### Packet Structure (4 bytes)

| Field      | Size    | Description                                    |
|------------|---------|------------------------------------------------|
| type_flags | 1 byte  | Message type (3 MSB) + flags (5 LSB)         |
| data       | 2 bytes | Payload data                                   |
| checksum   | 1 byte  | XOR checksum of all previous bytes            |

### Message Types

| Type | Value | Description              |
|------|-------|--------------------------|
| 0    | 0x00  | Button events            |
| 1    | 0x20  | LED control              |
| 2    | 0x40  | Power management         |
| 3    | 0x60  | Display control          |
| 4    | 0x80  | Debug codes              |
| 5    | 0xA0  | Debug text               |
| 6    | 0xC0  | System commands          |
| 7    | 0xE0  | Extended commands        |

### LED Control Modes

| Mode     | Value | Description              |
|----------|-------|--------------------------|
| Static   | 0x00  | Solid color              |
| Blink    | 0x04  | Blinking animation       |
| Fade     | 0x08  | Fading animation         |
| Rainbow  | 0x0C  | Rainbow color cycle      |

## Troubleshooting

### Module Not Loading

Check dmesg for errors:
```bash
dmesg | grep -i pamir
```

### UART Communication Issues

1. Verify UART configuration:
   ```bash
   sudo systemctl status serial-getty@ttyAMA2.service
   ```

2. Check GPIO configuration:
   ```bash
   sudo raspi-gpio get 4,5
   ```

3. Test UART communication:
   ```bash
   # Send a ping command
   echo -e "\xC0\x00\x00\xC0" | sudo tee /dev/pamir-sam
   
   # Check for response in dmesg
   dmesg | tail -10
   ```

### Button Events Not Working

1. Check input device registration:
   ```bash
   ls -la /dev/input/event*
   cat /proc/bus/input/devices
   ```

2. Verify button state:
   ```bash
   sudo evtest /dev/input/event<N>
   ```

### LED Control Issues

1. Check LED class device:
   ```bash
   ls -la /sys/class/leds/pamir:status/
   ```

2. Verify LED hardware:
   ```bash
   # Test LED brightness
   echo 255 | sudo tee /sys/class/leds/pamir:status/brightness
   echo 0 | sudo tee /sys/class/leds/pamir:status/brightness
   ```

## Implementation Status

| Component          | Status    | Feature Completeness | Notes                                           |
|-------------------|-----------|---------------------|-------------------------------------------------|
| Protocol Core      | Complete  | 100%                | Core packet processing functionality fully implemented and tested |
| Input Handler       | Complete  | 100%                | Button events fully supported with Linux input subsystem integration |
| LED Handler         | Partial   | 75%                 | Basic control implemented; LED brightness control from host is pending |
| Power Manager       | Complete  | 100%                | Boot/shutdown notifications and poll-based power metrics fully implemented |
| Display Controller  | Minimal   | 25%                 | Status reporting only; active display control pending |
| Debug Interface     | Complete  | 100%                | Both debug codes and text messages fully supported |
| System Commands     | Complete  | 100%                | All core system commands implemented, including versioning |
| Character Device    | Complete  | 100%                | Userspace interface fully functional with proper error handling |

## Architecture and Performance

### Communication Performance
- **UART Speed**: 115200 baud (≈11.5 KB/s theoretical maximum)
- **Packet Processing**: Sub-millisecond per packet processing time
- **Memory Usage**: 64-byte RX buffer, 256-byte TX buffer, small driver footprint
- **Throughput**: Practical maximum ~5,000 packets/second with processing overhead

### Error Handling and Recovery
- **Checksum Validation**: XOR checksum for packet integrity
- **Automatic Recovery**: Protocol resynchronization on communication errors
- **Timeout Management**: Configurable timeouts for acknowledgments and recovery
- **Watchdog Protection**: Hardware watchdog on RP2040, software watchdog in driver

### Timing Characteristics
- **Packet Transmission**: ~348 μs for complete 4-byte packet
- **Response Latency**: <10 ms for time-critical responses
- **Recovery Timeout**: 1000 ms default (configurable via device tree)
- **Power Polling**: Configurable interval (default 1000 ms)

## Required Kernel Modifications

For complete functionality, some additional kernel modifications may be needed:

### LED Control Enhancement
- Implement host-side LED brightness control callback
- Add RGB color space conversion utilities

### Display Control Enhancement  
- Implement framebuffer or DRM driver for active display control
- Add ioctl commands for display refresh modes and content updates

### Extended Protocol Support
- Define and implement extended commands for advanced functionality
- Add support for future protocol expansions

## Uninstallation

```bash
sudo dkms remove pamir-ai-sam/1.0.0 --all
```

## Technical Details

### File Structure

- `pamir-sam-main.c` - Main driver and serdev interface
- `pamir-sam-protocol.c` - Protocol handling and packet processing
- `pamir-sam-input.c` - Input device handling
- `pamir-sam-led.c` - LED control implementation
- `pamir-sam-power.c` - Power management functions
- `pamir-sam-display.c` - Display control functions
- `pamir-sam-debug.c` - Debug interface and logging
- `pamir-sam-system.c` - System command handling
- `pamir-sam-chardev.c` - Character device interface
- `pamir-sam.h` - Header with definitions and prototypes

### Memory Usage

- RX Buffer: 64 bytes
- TX Buffer: 256 bytes
- Debug Queue: 32 entries
- Packet Statistics: 8 counters (64-bit each)

### Performance Characteristics

- UART Speed: 115200 baud (≈11.5 KB/s)
- Packet Processing: < 1ms per packet
- Button Debouncing: Hardware-level
- LED Update Rate: Up to 60 Hz

## License

GPL v2

## Author

Pamir AI Incorporated - http://www.pamir.ai/