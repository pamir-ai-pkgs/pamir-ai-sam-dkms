#!/bin/bash

# Pamir AI SAM DKMS Installation Script
# This script installs the Pamir AI SAM (Signal Aggregation Module) driver as a DKMS module
# Supports Raspberry Pi (BCM), Rockchip, and Armbian platforms

set -e

PACKAGE_NAME="pamir-ai-sam"
PACKAGE_VERSION="1.0.0"
DKMS_SOURCE_DIR="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

# Source platform detection functions
if [ -f "./platform-detect.sh" ]; then
	source "./platform-detect.sh"
	PLATFORM=$(detect_platform)
	echo "Detected platform: $PLATFORM"
else
	echo "Warning: platform-detect.sh not found, assuming Raspberry Pi"
	PLATFORM="rpi"
fi

echo "Installing Pamir AI SAM DKMS module for $PLATFORM..."

# Create DKMS source directory
mkdir -p "${DKMS_SOURCE_DIR}"

# Copy all source files
cp -r * "${DKMS_SOURCE_DIR}/"

# Remove the install script from the source directory
rm -f "${DKMS_SOURCE_DIR}/install.sh"

# Add to DKMS
echo "Adding to DKMS..."
dkms add -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"

# Build the module
echo "Building DKMS module..."
dkms build -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"

# Install the module
echo "Installing DKMS module..."
dkms install -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"

echo "Pamir AI SAM DKMS installation completed successfully!"

# Configure module auto-loading for Armbian/Rockchip platforms
if [[ "$PLATFORM" == "armbian" || "$PLATFORM" == "rockchip" ]]; then
    echo "Configuring module auto-loading for $PLATFORM..."
    # Create modules-load.d configuration for systemd
    echo "pamir-ai-sam" > /etc/modules-load.d/pamir-ai-sam.conf
    echo "Module auto-loading configured in /etc/modules-load.d/pamir-ai-sam.conf"
fi

# Get platform-specific paths and names
OVERLAY_DIR=$(get_overlay_dir "$PLATFORM")
CONFIG_FILE=$(get_config_file "$PLATFORM")
OVERLAY_NAME=$(get_overlay_name "$PLATFORM")
DTS_FILE=$(get_dts_file "$PLATFORM")
DTS_SOURCE="${DKMS_SOURCE_DIR}/${DTS_FILE}"
DTBO_DEST="${OVERLAY_DIR}${OVERLAY_NAME}.dtbo"

# Compile and copy device tree overlay
if [ -f "$DTS_SOURCE" ]; then
    echo "Compiling device tree overlay for $PLATFORM..."

    # Check if device-tree-compiler is available
    if ! command -v dtc &> /dev/null; then
        echo "Error: device-tree-compiler (dtc) not found. Please install it:"
        echo "  sudo apt-get install device-tree-compiler"
        exit 1
    fi

    # Compile the overlay with include paths
    KERNEL_HEADERS="/lib/modules/$(uname -r)/build"
    INCLUDE_PATHS="-i ${KERNEL_HEADERS}/include -i ${KERNEL_HEADERS}/arch/arm64/boot/dts/overlays"

    # Create overlay directory if it doesn't exist
    mkdir -p "$OVERLAY_DIR"

    if dtc -@ -I dts -O dtb ${INCLUDE_PATHS} -o "${DTBO_DEST}" "${DTS_SOURCE}"; then
        echo "Device tree overlay compiled successfully"
        echo "Device tree overlay installed to ${DTBO_DEST}"
    else
        echo "Error: Failed to compile device tree overlay"
        # Try without include paths as fallback
        echo "Attempting compilation without include paths..."
        if dtc -@ -I dts -O dtb -o "${DTBO_DEST}" "${DTS_SOURCE}"; then
            echo "Device tree overlay compiled successfully (without includes)"
            echo "Device tree overlay installed to ${DTBO_DEST}"
        else
            echo "Error: Failed to compile device tree overlay"
            exit 1
        fi
    fi
else
    echo "Warning: Device tree overlay source not found at $DTS_SOURCE"
    echo "You may need to manually compile and copy the overlay"
fi

echo ""
echo "To enable the SAM module:"

case "$PLATFORM" in
    armbian|rockchip)
        echo "1. Add 'overlays=${OVERLAY_NAME}' to ${CONFIG_FILE}"
        echo "   Or if overlays line exists, add '${OVERLAY_NAME}' to the list"
        echo "2. Reboot your system"
        ;;
    rpi)
        echo "1. Add 'dtoverlay=${OVERLAY_NAME}' to ${CONFIG_FILE}"
        echo "2. Reboot your system"
        ;;
    *)
        echo "1. Configure your bootloader to load the overlay ${OVERLAY_NAME}"
        echo "2. Reboot your system"
        ;;
esac

echo ""
echo "The SAM module provides:"
echo "  - Button input events via /dev/input/event*"
echo "  - LED control via /sys/class/leds/pamir:status/"
echo "  - Character device interface via /dev/pamir-sam"
echo ""
echo "To remove the module:"
echo "  sudo dkms remove ${PACKAGE_NAME}/${PACKAGE_VERSION} --all"
echo ""
echo "Platform-specific information:"
echo "  Platform: $PLATFORM"
echo "  Overlay: ${OVERLAY_NAME}.dtbo"
echo "  Config: ${CONFIG_FILE}"
echo "  Location: ${OVERLAY_DIR}"