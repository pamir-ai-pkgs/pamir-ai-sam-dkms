#!/bin/bash

# Platform detection helper script for Pamir AI SAM DKMS
# Supports Raspberry Pi (BCM), Rockchip, and Armbian platforms

detect_platform() {
    # Check for Armbian first (most specific)
    if [ -f /etc/armbian-release ]; then
        echo "armbian"
        return
    fi

    # Check device tree for platform identification
    if [ -f /proc/device-tree/compatible ]; then
        # Read compatible string properly (null-terminated strings)
        local compat=$(tr '\0' '\n' < /proc/device-tree/compatible 2>/dev/null)
        if echo "$compat" | grep -q "brcm,bcm2712"; then
            echo "rpi"
        elif echo "$compat" | grep -q "rockchip,rk35"; then
            # More generic check for rk35xx series
            echo "rockchip"
        else
            echo "unknown"
        fi
    else
        echo "unknown"
    fi
}

get_overlay_dir() {
    local platform="${1:-$(detect_platform)}"

    case "$platform" in
        armbian|rockchip)
            # Armbian and generic Rockchip use this path
            echo "/boot/dtb/rockchip/overlay/"
            ;;
        rpi)
            # Raspberry Pi OS uses this path
            echo "/boot/firmware/overlays/"
            ;;
        *)
            # Fallback for unknown platforms
            echo "/boot/overlays/"
            ;;
    esac
}

get_config_file() {
    local platform="${1:-$(detect_platform)}"

    case "$platform" in
        armbian|rockchip)
            # Armbian uses armbianEnv.txt
            echo "/boot/armbianEnv.txt"
            ;;
        rpi)
            # Raspberry Pi OS uses config.txt
            echo "/boot/firmware/config.txt"
            ;;
        *)
            # Fallback
            echo "/boot/config.txt"
            ;;
    esac
}

get_overlay_name() {
    local platform="${1:-$(detect_platform)}"

    case "$platform" in
        armbian|rockchip)
            # Rockchip-specific overlay name
            echo "pamir-ai-sam-rk3566"
            ;;
        rpi)
            # Raspberry Pi overlay name
            echo "pamir-ai-sam"
            ;;
        *)
            # Default overlay name
            echo "pamir-ai-sam"
            ;;
    esac
}

get_dts_file() {
    local platform="${1:-$(detect_platform)}"

    case "$platform" in
        armbian|rockchip)
            # Rockchip-specific DTS file
            echo "pamir-ai-sam-rk3566-overlay.dts"
            ;;
        rpi)
            # Raspberry Pi DTS file
            echo "pamir-ai-sam-overlay.dts"
            ;;
        *)
            # Default to RPi overlay
            echo "pamir-ai-sam-overlay.dts"
            ;;
    esac
}

# Function to print platform information (for debugging)
print_platform_info() {
    local platform=$(detect_platform)

    echo "Detected Platform: $platform"
    echo "Overlay Directory: $(get_overlay_dir "$platform")"
    echo "Config File: $(get_config_file "$platform")"
    echo "Overlay Name: $(get_overlay_name "$platform")"
    echo "DTS File: $(get_dts_file "$platform")"
}

# If script is run directly (not sourced), print platform info
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    print_platform_info
fi