#!/bin/bash

# Platform detection helper script for Pamir AI SAM DKMS
# Supports Raspberry Pi (BCM), Rockchip, and Armbian platforms
#
# Environment variable override:
#   DISTILLER_PLATFORM=rpi|rockchip|armbian|unknown

# Validate if the provided platform is supported
validate_platform() {
	local platform="$1"
	case "$platform" in
	rpi | rockchip | armbian | armsom-rk3576 | unknown)
		return 0
		;;
	*)
		return 1
		;;
	esac
}

# Detect if running on Armbian by checking kernel naming patterns
# This is useful during Armbian build when /etc/armbian-release doesn't exist yet
is_armbian_kernel() {
	# Check running kernel first
	if uname -r 2>/dev/null | grep -qE '(vendor|current|edge|legacy)-(rk35xx|rockchip64)'; then
		return 0
	fi

	# Check installed kernels in /lib/modules (useful during build/chroot)
	if ls /lib/modules/ 2>/dev/null | grep -qE '(vendor|current|edge|legacy)-(rk35xx|rockchip64)'; then
		return 0
	fi

	return 1
}

detect_platform() {
	local platform="unknown"

	# Environment variable override
	if [ -n "$DISTILLER_PLATFORM" ] && validate_platform "$DISTILLER_PLATFORM"; then
		[ -t 2 ] && echo "Platform overridden by DISTILLER_PLATFORM: $DISTILLER_PLATFORM" >&2
		echo "$DISTILLER_PLATFORM"
		return
	elif [ -n "$DISTILLER_PLATFORM" ]; then
		echo "Warning: Invalid DISTILLER_PLATFORM value: $DISTILLER_PLATFORM" >&2
	fi

	# Armbian detection
	if [ -f /etc/armbian-release ] || [ -f /boot/armbianEnv.txt ] || is_armbian_kernel; then
		echo "armbian"
		return
	fi

	# Device tree compatibility checks
	if [ -f /proc/device-tree/compatible ]; then
		local compat
		compat=$(tr '\0' '\n' </proc/device-tree/compatible 2>/dev/null)
		if echo "$compat" | grep -q "brcm,bcm2712"; then
			echo "rpi"
			return
		elif echo "$compat" | grep -q -e "armsom,cm5-io" -e "rockchip,rk3576"; then
			echo "armsom-rk3576"
			return
		elif echo "$compat" | grep -q "rockchip,rk35"; then
			echo "rockchip"
			return
		fi
	fi

	echo "$platform"
}

get_overlay_dir() {
	local platform="${1:-$(detect_platform)}"

	case "$platform" in
	armbian | rockchip | armsom-rk3576)
		echo "/boot/dtb/rockchip/overlay/"
		;;
	*)
		echo "/boot/firmware/overlays/"
		;;
	esac
}

get_config_file() {
	local platform="${1:-$(detect_platform)}"

	case "$platform" in
	armbian | rockchip | armsom-rk3576)
		echo "/boot/armbianEnv.txt"
		;;
	*)
		echo "/boot/firmware/config.txt"
		;;
	esac
}

get_overlay_name() {
	local platform="${1:-$(detect_platform)}"

	case "$platform" in
	armbian | rockchip)
		echo "pamir-ai-sam-rk3566"
		;;
	armsom-rk3576)
		echo "pamir-ai-sam-rk3576"
		;;
	*)
		echo "pamir-ai-sam"
		;;
	esac
}

get_dts_file() {
	local platform="${1:-$(detect_platform)}"

	case "$platform" in
	armbian | rockchip)
		echo "pamir-ai-sam-rk3566-overlay.dts"
		;;
	armsom-rk3576)
		echo "pamir-ai-sam-rk3576-overlay.dts"
		;;
	*)
		echo "pamir-ai-sam-overlay.dts"
		;;
	esac
}

print_platform_info() {
	local platform=$(detect_platform)
	local detection_method="auto-detected"

	# Check if platform was overridden
	if [ -n "$DISTILLER_PLATFORM" ] && validate_platform "$DISTILLER_PLATFORM"; then
		detection_method="overridden by DISTILLER_PLATFORM"
	fi

	echo "Platform: $platform ($detection_method)"
	if [ -n "$DISTILLER_PLATFORM" ]; then
		echo "DISTILLER_PLATFORM env var: $DISTILLER_PLATFORM"
	fi
	echo "Overlay Directory: $(get_overlay_dir "$platform")"
	echo "Config File: $(get_config_file "$platform")"
	echo "Overlay Name: $(get_overlay_name "$platform")"
	echo "DTS File: $(get_dts_file "$platform")"
}

# If script is run directly (not sourced), print platform info
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
	print_platform_info
fi
