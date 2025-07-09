# Pamir AI SAM (Sensor and Actuator Module) DKMS Module

This package contains the DKMS (Dynamic Kernel Module Support) module for the Pamir AI SAM (Sensor and Actuator Module) driver for Raspberry Pi CM5.

## Overview

The Pamir AI SAM driver implements a 4-byte packet protocol for communication between the Linux host and the RP2040 microcontroller via UART. It provides comprehensive hardware control and status reporting functionality.

## Features

### Hardware Interfaces
- **Button Input**: Physical button events via Linux input subsystem
- **LED Control**: RGB LED control with various animation modes
- **Power Management**: Power state coordination with RP2040
- **E-ink Display**: Display control and status reporting
- **Debug Interface**: Diagnostic codes and text messages

### Protocol Support
- **4-byte Packet Format**: Ultra-optimized communication protocol
- **Message Types**: 8 different message types for various functions
- **Error Detection**: XOR checksum validation
- **UART Communication**: 115200 baud, 8N1 format

### Linux Integration
- **Input Device**: Button events via `/dev/input/event*`
- **LED Class Device**: LED control via `/sys/class/leds/pamir:status/`
- **Character Device**: Raw protocol access via `/dev/pamir-sam`
- **Debug Interface**: Diagnostic information and logging

## Installation

### Prerequisites

- Linux kernel headers for your current kernel
- DKMS package installed
- Root privileges

### Install DKMS Module

```bash
sudo ./install.sh
```

### Enable the SAM Module

1. Add the device tree overlay to `/boot/config.txt`:
   ```
   dtoverlay=pamir-ai-sam
   ```

2. Reboot your system

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

Send power commands:
```bash
# Send ping command
echo -e "\xC0\x00\x00\xC0" | sudo tee /dev/pamir-sam

# Query power status
echo -e "\x40\x00\x00\x40" | sudo tee /dev/pamir-sam
```

### Debug Interface

Monitor debug messages:
```bash
# Watch system logs for debug information
sudo journalctl -f | grep pamir-sam
```

## Configuration

### Device Tree Properties

The SAM module can be configured via device tree overlay parameters:

```dts
&pamir_ai_sam {
    /* Debug level (0-3) */
    debug-level = <1>;
    
    /* Recovery timeout in milliseconds */
    recovery-timeout-ms = <1000>;
    
    /* Require acknowledgment for commands */
    ack-required;
    
    /* Enable/disable device */
    status = "okay";
};
```

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

| Component          | Status    | Notes                                           |
|-------------------|-----------|------------------------------------------------|
| Protocol Core      | Complete  | Full packet processing                         |
| Input Handler       | Complete  | Button events fully supported                  |
| LED Handler         | Partial   | Basic control implemented                      |
| Power Manager       | Minimal   | Boot/shutdown notifications need integration   |
| Display Controller  | Minimal   | Status reporting only                          |
| Debug Interface     | Complete  | Full diagnostic support                        |
| System Commands     | Complete  | All core commands implemented                  |
| Character Device    | Complete  | Full userspace interface                       |

## Required Kernel Modifications

For complete functionality, some kernel modifications are needed:

### LED Control
- Implement `brightness_set` callback in LED class device
- Add code to translate brightness values to RGB colors

### Power Management
- Add hooks to Linux power management subsystem
- Implement battery reporting system
- Add shutdown notification support

### Display Control
- Implement framebuffer or DRM driver
- Add ioctl commands for display refresh modes
- Connect to Linux graphics subsystem

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