# Pamir AI Signal Aggregation Module (SAM) DKMS Driver

This package contains the DKMS (Dynamic Kernel Module Support) driver for the Pamir AI Signal Aggregation Module (SAM). The SAM driver enables bidirectional communication between a Linux host system (Raspberry Pi CM5, Rockchip RK3566/Radxa Zero 3W) and an RP2040 microcontroller via UART, providing comprehensive hardware control and status reporting functionality.

## Overview

The Pamir AI SAM driver implements a highly optimized 4-byte packet protocol designed for embedded systems with limited resources. It provides a modular architecture with clearly defined components that interact through standardized interfaces, enabling reliable communication for hardware control and status monitoring.

## Recent Updates

### Version 1.0.0 Enhancements

- **Multiple LED Support**: Consistent LED naming scheme with `pamir:led0`, `pamir:led1`, etc.
- **Power Supply Integration**: Standard Linux power supply interface at `/sys/class/power_supply/pamir_battery/`
- **Display Release Signal**: Automatic eink control handover from RP2040 to Pi during boot
- **Enhanced Protocol Testing**: Comprehensive test suite with RGB LED validation
- **Improved RGB LED Control**: Full multi-LED support with independent control

## Features

### Hardware Interfaces
- **Button Input**: Physical button events via Linux input subsystem with hardware debouncing
- **RGB LED Control**: Full RGB LED control with multiple animation modes and custom triggers
- **Power Management**: Complete power state coordination with automatic boot/shutdown notifications
- **E-ink Display**: Display control and status reporting with refresh management
- **Debug Interface**: Comprehensive diagnostic codes and text messages with circular buffering

### Protocol Support
- **4-byte Packet Format**: Ultra-optimized communication protocol with minimal overhead
- **Message Types**: 8 different message types covering all hardware functions
- **Error Detection**: XOR checksum validation with automatic recovery mechanisms
- **UART Communication**: 115200 baud, 8N1 format
  - Raspberry Pi CM5: UART2 (GPIO4/5)
  - Rockchip RK3566: UART3 (GPIO1_A0/A1, pins 3/5)
- **Timing Constraints**: Sub-millisecond packet processing with configurable timeouts

### Linux Integration
- **Input Device**: Button events via `/dev/input/event*` with standard key codes
- **LED Class Device**: Multiple RGB LED control via `/sys/class/leds/pamir:led0/`, `/sys/class/leds/pamir:led1/`, etc.
- **LED Triggers**: Custom RGB triggers (`heartbeat-rgb`, `breathing-rgb`, `rainbow-rgb`)
- **Character Device**: Raw protocol access via `/dev/pamir-sam` for debugging
- **Power Metrics**: Sysfs interface for current, battery, temperature, and voltage monitoring
- **Power Supply**: Standard Linux power supply interface at `/sys/class/power_supply/pamir_battery/`
- **Display Control**: Boot-time display release signal for eink handover from RP2040 to Pi
- **Debug Interface**: Kernel logging with configurable verbosity levels

## Installation

### Prerequisites

- Linux kernel headers for your current kernel
- DKMS package installed
- Device Tree Compiler (dtc)
- Root privileges
- Supported hardware:
  - Raspberry Pi CM5 with BCM2712 ARM64 architecture
  - Rockchip RK3566 boards (Radxa Zero 3W)
  - Other compatible ARM64 platforms

### Install DKMS Module

```bash
sudo ./install.sh
```

### Enable the SAM Module

#### For Raspberry Pi CM5:

1. Add the device tree overlay to `/boot/firmware/config.txt`:
   ```
   dtoverlay=pamir-ai-sam
   ```

2. Configure UART2 in `/boot/firmware/config.txt` (if not already done):
   ```
   enable_uart=1
   dtoverlay=uart2
   ```

3. Reboot your system

#### For Rockchip RK3566 (Radxa Zero 3W with Armbian):

1. Add the overlay to `/boot/armbianEnv.txt`:
   ```
   overlays=pamir-ai-sam-rk3566
   ```
   Or append to existing overlays line if present.

2. The UART3 interface on pins 3 (RX) and 5 (TX) will be automatically configured.

3. Reboot your system

**Note**: The installation script automatically detects your platform and configures the appropriate overlay.

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

### RGB LED Control

The SAM driver provides comprehensive RGB LED control with multiple interfaces. LEDs are numbered starting from 0:

#### RGB Color Control
Control individual RGB components:
```bash
# Set RGB color (0-255 for each component) for LED 0
echo 255 | sudo tee /sys/class/leds/pamir:led0/red    # Red component
echo 128 | sudo tee /sys/class/leds/pamir:led0/green  # Green component
echo 0 | sudo tee /sys/class/leds/pamir:led0/blue     # Blue component

# Control additional LEDs (led1, led2, etc.)
echo 0 | sudo tee /sys/class/leds/pamir:led1/red
echo 255 | sudo tee /sys/class/leds/pamir:led1/green
echo 0 | sudo tee /sys/class/leds/pamir:led1/blue

# Read current RGB values
cat /sys/class/leds/pamir:led0/red
cat /sys/class/leds/pamir:led0/green
cat /sys/class/leds/pamir:led0/blue
```

#### Animation Modes
Control LED animation patterns:
```bash
# Set animation mode for LED 0
echo "static" | sudo tee /sys/class/leds/pamir:led0/mode    # Solid color
echo "blink" | sudo tee /sys/class/leds/pamir:led0/mode     # Blinking
echo "fade" | sudo tee /sys/class/leds/pamir:led0/mode      # Fading
echo "rainbow" | sudo tee /sys/class/leds/pamir:led0/mode   # Rainbow cycle

# Set animation timing (100, 200, 500, or 1000 milliseconds)
echo 500 | sudo tee /sys/class/leds/pamir:led0/timing

# Control multiple LEDs independently
echo "blink" | sudo tee /sys/class/leds/pamir:led1/mode
echo "fade" | sudo tee /sys/class/leds/pamir:led2/mode

# Read current mode and timing
cat /sys/class/leds/pamir:led0/mode
cat /sys/class/leds/pamir:led0/timing
```

#### LED Triggers
Use standard Linux LED triggers with RGB support:
```bash
# List available triggers for LED 0
cat /sys/class/leds/pamir:led0/trigger

# Set RGB triggers for LED 0
echo "heartbeat-rgb" | sudo tee /sys/class/leds/pamir:led0/trigger  # Red heartbeat
echo "breathing-rgb" | sudo tee /sys/class/leds/pamir:led0/trigger  # Blue breathing
echo "rainbow-rgb" | sudo tee /sys/class/leds/pamir:led0/trigger    # Rainbow cycling

# Apply different triggers to different LEDs
echo "heartbeat-rgb" | sudo tee /sys/class/leds/pamir:led0/trigger
echo "breathing-rgb" | sudo tee /sys/class/leds/pamir:led1/trigger
echo "rainbow-rgb" | sudo tee /sys/class/leds/pamir:led2/trigger

# Disable triggers
echo "none" | sudo tee /sys/class/leds/pamir:led0/trigger
```

#### Brightness Control
Overall brightness control (preserves RGB ratios):
```bash
# Set overall brightness (0-255) for LED 0
echo 128 | sudo tee /sys/class/leds/pamir:led0/brightness

# Control brightness for multiple LEDs
echo 64 | sudo tee /sys/class/leds/pamir:led1/brightness
echo 192 | sudo tee /sys/class/leds/pamir:led2/brightness

# Get current brightness
cat /sys/class/leds/pamir:led0/brightness
```

#### Raw Protocol Commands
Send LED commands via character device:
```bash
# Set LED to red (static mode)
echo -e "\x20\xF0\x00\xD0" | sudo tee /dev/pamir-sam

# Set LED to blink blue
echo -e "\x24\x00\xF8\xDC" | sudo tee /dev/pamir-sam
```

### Power Management

The driver automatically manages power states and provides access to power metrics through multiple interfaces:

#### Power Supply Interface (Standard Linux)
```bash
# Access power metrics via standard power supply interface
cat /sys/class/power_supply/pamir_battery/status
cat /sys/class/power_supply/pamir_battery/capacity
cat /sys/class/power_supply/pamir_battery/voltage_now
cat /sys/class/power_supply/pamir_battery/current_now
cat /sys/class/power_supply/pamir_battery/temp
cat /sys/class/power_supply/pamir_battery/technology
```

#### Raw Protocol Commands
```bash
# Send ping command to test communication
echo -e "\xC0\x00\x00\xC0" | sudo tee /dev/pamir-sam

# Query power status
echo -e "\x40\x00\x00\x40" | sudo tee /dev/pamir-sam
```

Power management features:
- **Automatic Boot Notification**: Sent during driver initialization
- **Display Release Signal**: Automatically signals RP2040 to release eink control to Pi during boot
- **Automatic Shutdown Notification**: Sent before system shutdown via reboot notifier
- **Power Metrics Polling**: Configurable interval for current, battery, temperature, and voltage
- **Standard Power Supply Interface**: Integration with Linux power supply subsystem
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

### Display Control

The driver provides automatic eink display handover functionality:

```bash
# Monitor display-related messages
sudo dmesg | grep -i display

# Display release signal is sent automatically during boot
# This tells the RP2040 to:
# 1. Call eink.de_init() to release GPIO control
# 2. Call einkMux.low() to route eink to Pi
```

Display control features:
- **Automatic Boot Release**: Signals RP2040 to release eink control during driver initialization
- **Multiplexer Control**: Coordinates eink access between RP2040 and Pi
- **Boot Animation Loop**: RP2040 shows animation until Pi takes control
- **GPIO Release**: Proper GPIO handover with high-impedance state

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

### RGB LED Control Issues

1. Check LED class devices and RGB attributes:
   ```bash
   ls -la /sys/class/leds/pamir:led*/
   # Should show multiple LEDs: pamir:led0, pamir:led1, etc.
   # Each should have: red, green, blue, mode, timing, trigger, brightness
   ```

2. Verify RGB LED hardware:
   ```bash
   # Test RGB components for LED 0
   echo 255 | sudo tee /sys/class/leds/pamir:led0/red
   echo 255 | sudo tee /sys/class/leds/pamir:led0/green
   echo 255 | sudo tee /sys/class/leds/pamir:led0/blue
   
   # Test animation modes
   echo "blink" | sudo tee /sys/class/leds/pamir:led0/mode
   echo "500" | sudo tee /sys/class/leds/pamir:led0/timing
   
   # Test additional LEDs
   echo 255 | sudo tee /sys/class/leds/pamir:led1/red
   echo "fade" | sudo tee /sys/class/leds/pamir:led1/mode
   ```

3. Test RGB LED triggers:
   ```bash
   # List available triggers for LED 0
   cat /sys/class/leds/pamir:led0/trigger
   # Should show: none heartbeat-rgb breathing-rgb rainbow-rgb
   
   # Test trigger activation
   echo "heartbeat-rgb" | sudo tee /sys/class/leds/pamir:led0/trigger
   
   # Test different triggers on different LEDs
   echo "breathing-rgb" | sudo tee /sys/class/leds/pamir:led1/trigger
   echo "rainbow-rgb" | sudo tee /sys/class/leds/pamir:led2/trigger
   
   # Disable triggers
   echo "none" | sudo tee /sys/class/leds/pamir:led0/trigger
   ```

### Power Supply Issues

1. Check power supply interface:
   ```bash
   ls -la /sys/class/power_supply/pamir_battery/
   # Should show: status, capacity, voltage_now, current_now, temp, technology, present
   ```

2. Test power supply readings:
   ```bash
   # Check if power supply is registered
   cat /sys/class/power_supply/pamir_battery/present
   # Should return: 1
   
   # Check battery status
   cat /sys/class/power_supply/pamir_battery/status
   # Should return: Discharging
   
   # Check if metrics are updating
   cat /sys/class/power_supply/pamir_battery/capacity
   cat /sys/class/power_supply/pamir_battery/voltage_now
   cat /sys/class/power_supply/pamir_battery/current_now
   ```


## Implementation Status

| Component          | Status    | Feature Completeness | Notes                                           |
|-------------------|-----------|---------------------|-------------------------------------------------|
| Protocol Core      | Complete  | 100%                | Core packet processing functionality fully implemented and tested |
| Input Handler       | Complete  | 100%                | Button events fully supported with Linux input subsystem integration |
| RGB LED Handler     | Complete  | 100%                | Full RGB control, multiple LED support, animation modes, and custom triggers |
| Power Manager       | Complete  | 100%                | Boot/shutdown notifications, power metrics, and standard power supply interface |
| Display Controller  | Partial   | 40%                 | Boot-time display release signal implemented; active display control pending |
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

## Advanced Features

### RGB LED Control Features
- **Full RGB Color Control**: Individual R, G, B component control (0-255 range)
- **Animation Modes**: Static, blink, fade, and rainbow animation patterns
- **Custom Triggers**: RGB-aware LED triggers for system integration
- **Timing Control**: Configurable animation timing (100ms, 200ms, 500ms, 1000ms)
- **Standard LED Interface**: Maintains standard LED brightness interface

### RGB LED Triggers
- **`heartbeat-rgb`**: Red LED with heartbeat pattern (double-pulse)
- **`breathing-rgb`**: Blue LED with smooth breathing animation
- **`rainbow-rgb`**: Full spectrum rainbow color cycling
- **Timer-based**: Kernel timer implementation for smooth animations

### Sysfs Interface Structure
The SAM driver provides a comprehensive sysfs interface for RGB LED control:

```
/sys/class/leds/pamir:led0/          # LED 0 (primary LED)
├── brightness          # Overall brightness (0-255)
├── red                 # Red component (0-255)
├── green               # Green component (0-255)
├── blue                # Blue component (0-255)
├── mode                # Animation mode (static/blink/fade/rainbow)
├── timing              # Animation timing (100/200/500/1000ms)
├── trigger             # LED trigger (none/heartbeat-rgb/breathing-rgb/rainbow-rgb)
├── max_brightness      # Maximum brightness value (255)
└── uevent              # Device events

/sys/class/leds/pamir:led1/          # LED 1 (additional LED)
├── brightness          # Overall brightness (0-255)
├── red                 # Red component (0-255)
├── green               # Green component (0-255)
├── blue                # Blue component (0-255)
├── mode                # Animation mode (static/blink/fade/rainbow)
├── timing              # Animation timing (100/200/500/1000ms)
├── trigger             # LED trigger (none/heartbeat-rgb/breathing-rgb/rainbow-rgb)
├── max_brightness      # Maximum brightness value (255)
└── uevent              # Device events

# Additional LEDs (led2, led3, etc.) follow same structure
```

#### Power Supply Interface Structure
```
/sys/class/power_supply/pamir_battery/
├── status              # Battery status (Discharging)
├── capacity            # Battery percentage (0-100)
├── voltage_now         # Voltage in microvolts
├── current_now         # Current in microamps
├── temp                # Temperature in 0.1°C units
├── technology          # Battery technology (Li-ion)
├── present             # Battery present (1)
└── uevent              # Device events
```

### Display Control Enhancement  
- Implement framebuffer or DRM driver for active display control
- Add ioctl commands for display refresh modes and content updates

### Extended Protocol Support
- Define and implement extended commands for advanced functionality
- Add support for future protocol expansions

## Testing and Validation

### SAM Test Utility
The driver includes a comprehensive test utility (`sam_test.c`) that provides:

- **Protocol Testing**: Complete validation of all 8 message types
- **RGB LED Testing**: Full RGB color control, animation modes, and trigger testing
- **Input Event Monitoring**: Real-time button event monitoring
- **Power Metrics Testing**: Power management command validation
- **Interactive Mode**: Menu-driven testing interface
- **Performance Metrics**: Packet timing, error rates, and statistics

#### Running Tests
```bash
# Compile the test utility
gcc -o sam_test sam_test.c

# Run comprehensive tests
sudo ./sam_test

# Run interactive mode
sudo ./sam_test -i

# Test specific components
sudo ./sam_test -t led      # Test LED functionality
sudo ./sam_test -t system   # Test system commands
sudo ./sam_test -t power    # Test power management
```

### Python Test Scripts
Additional Python test scripts are available in the firmware directory:
- `test_sam_protocol.py`: Comprehensive protocol validation
- `test_sam_basic.py`: Quick functionality tests

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