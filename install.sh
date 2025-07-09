#!/bin/bash

# Pamir AI SAM DKMS Installation Script
# This script installs the Pamir AI SAM (Sensor and Actuator Module) driver as a DKMS module

set -e

PACKAGE_NAME="pamir-ai-sam"
PACKAGE_VERSION="1.0.0"
DKMS_SOURCE_DIR="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

echo "Installing Pamir AI SAM DKMS module..."

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

# Compile and copy device tree overlay
DTS_SOURCE="${DKMS_SOURCE_DIR}/pamir-ai-sam-overlay.dts"
DTBO_DEST="/boot/firmware/overlays/"
DTBO_FILE="pamir-ai-sam.dtbo"

if [ -f "$DTS_SOURCE" ]; then
    echo "Compiling device tree overlay..."
    
    # Check if device-tree-compiler is available
    if ! command -v dtc &> /dev/null; then
        echo "Error: device-tree-compiler (dtc) not found. Please install it:"
        echo "  sudo apt-get install device-tree-compiler"
        exit 1
    fi
    
    # Compile the overlay with include paths
    KERNEL_HEADERS="/lib/modules/$(uname -r)/build"
    INCLUDE_PATHS="-i ${KERNEL_HEADERS}/include -i ${KERNEL_HEADERS}/arch/arm64/boot/dts/overlays"
    
    if dtc -@ -I dts -O dtb ${INCLUDE_PATHS} -o "${DTBO_FILE}" "${DTS_SOURCE}"; then
        echo "Device tree overlay compiled successfully"
        
        # Copy to destination
        mkdir -p "$DTBO_DEST"
        cp "${DTBO_FILE}" "$DTBO_DEST"
        echo "Device tree overlay copied to ${DTBO_DEST}${DTBO_FILE}"
        
        # Clean up temporary file
        rm -f "${DTBO_FILE}"
    else
        echo "Error: Failed to compile device tree overlay"
        exit 1
    fi
else
    echo "Warning: Device tree overlay source not found at $DTS_SOURCE"
    echo "You may need to manually compile and copy pamir-ai-sam-overlay.dts"
fi

echo ""
echo "To enable the SAM module:"
echo "1. Add 'dtoverlay=pamir-ai-sam' to /boot/config.txt"
echo "2. Reboot your system"
echo ""
echo "The SAM module provides:"
echo "  - Button input events via /dev/input/event*"
echo "  - LED control via /sys/class/leds/pamir:status/"
echo "  - Character device interface via /dev/pamir-sam"
echo ""
echo "To remove the module:"
echo "  sudo dkms remove ${PACKAGE_NAME}/${PACKAGE_VERSION} --all"
