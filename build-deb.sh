#!/bin/bash

# Pamir AI SAM DKMS Debian Package Builder
# This script builds a Debian package for the Pamir AI sam DKMS modules

set -e

# Configuration
PACKAGE_NAME="pamir-ai-sam-dkms"
PACKAGE_VERSION="1.0.0"
DEBIAN_REVISION="1"
FULL_VERSION="${PACKAGE_VERSION}-${DEBIAN_REVISION}"
BUILD_DIR="dist"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on a Debian-based system
check_system() {
    if ! command -v dpkg-buildpackage &> /dev/null; then
        print_error "dpkg-buildpackage not found. Please install dpkg-dev:"
        print_error "  sudo apt-get install dpkg-dev"
        exit 1
    fi
    
    if ! command -v dh &> /dev/null; then
        print_error "dh not found. Please install debhelper:"
        print_error "  sudo apt-get install debhelper"
        exit 1
    fi
}

# Check build dependencies
check_dependencies() {
    print_info "Checking build dependencies..."
    
    local missing_deps=()
    
    # Required packages for building
    local required_packages=(
        "build-essential"
        "debhelper"
        "dkms"
        "device-tree-compiler"
        "fakeroot"
        "dpkg-dev"
    )
    
    for package in "${required_packages[@]}"; do
        if ! dpkg -l | grep -q "^ii  ${package}"; then
            missing_deps+=("${package}")
        fi
    done
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing build dependencies: ${missing_deps[*]}"
        print_error "Please install them with:"
        print_error "  sudo apt-get install ${missing_deps[*]}"
        exit 1
    fi
    
    print_success "All build dependencies are satisfied"
}

# Clean previous builds
clean_build() {
    print_info "Cleaning previous builds..."
    
    # Remove build directory
    if [ -d "${BUILD_DIR}" ]; then
        rm -rf "${BUILD_DIR}"
    fi
    
    # Remove any generated files
    rm -f ../*.deb ../*.changes ../*.buildinfo ../*.dsc ../*.tar.* ../*.upload
    
    print_success "Build environment cleaned"
}

# Prepare source directory
prepare_source() {
    print_info "Preparing source directory..."
    
    # Create build directory
    mkdir -p "${BUILD_DIR}"
    
    # Copy source files to build directory
    local source_dir="${BUILD_DIR}/${PACKAGE_NAME}-${PACKAGE_VERSION}"
    mkdir -p "${source_dir}"
    
    # Copy source files
    cp -r "${SCRIPT_DIR}"/*.c "${source_dir}/"
    cp -r "${SCRIPT_DIR}"/*.h "${source_dir}/"
    cp -r "${SCRIPT_DIR}"/Makefile "${source_dir}/"
    cp -r "${SCRIPT_DIR}"/dkms.conf "${source_dir}/"
    cp -r "${SCRIPT_DIR}"/*.dts "${source_dir}/"
    cp -r "${SCRIPT_DIR}"/README.md "${source_dir}/"
    
    # Copy debian packaging files
    cp -r "${SCRIPT_DIR}"/debian "${source_dir}/"
    
    # Fix changelog date
    sed -i "s/\$(date -R)/$(date -R)/" "${source_dir}/debian/changelog"
    
    print_success "Source directory prepared at ${source_dir}"
}

# Build the package
build_package() {
    print_info "Building Debian package..."
    
    local source_dir="${BUILD_DIR}/${PACKAGE_NAME}-${PACKAGE_VERSION}"
    
    # Change to source directory
    cd "${source_dir}"
    
    # Build the package
    print_info "Running dpkg-buildpackage..."
    if CC="aarch64-linux-gnu-gcc" dpkg-buildpackage -us -uc -b -aarm64; then
        print_success "Package built successfully"
    else
        print_error "Package build failed"
        exit 1
    fi
    
    # Return to original directory
    cd "${SCRIPT_DIR}"
}

# Display results
show_results() {
    print_info "Build completed successfully!"
    print_info "Generated files:"
    
    # List generated files
    for file in "${BUILD_DIR}"/*.deb "${BUILD_DIR}"/*.changes "${BUILD_DIR}"/*.buildinfo; do
        if [ -f "$file" ]; then
            local filename=$(basename "$file")
            local size=$(du -h "$file" | cut -f1)
            print_success "  ${filename} (${size})"
        fi
    done
    
    print_info ""
    print_info "To install the package:"
    print_info "  sudo dpkg -i ${BUILD_DIR}/${PACKAGE_NAME}_${FULL_VERSION}_all.deb"
    print_info ""
    print_info "To install dependencies if needed:"
    print_info "  sudo apt-get install -f"
    print_info ""
    print_info "To test the package:"
    print_info "  sudo dpkg -i ${BUILD_DIR}/${PACKAGE_NAME}_${FULL_VERSION}_all.deb"
    print_info "  dkms status"
    print_info "  ls -la /boot/firmware/overlays/pamir-ai-sam.dtbo"
}

# Main execution
main() {
    print_info "Starting Debian package build for ${PACKAGE_NAME} ${FULL_VERSION}"
    print_info "Script directory: ${SCRIPT_DIR}"
    
    # Check system and dependencies
    check_system
    check_dependencies
    
    # Build process
    clean_build
    prepare_source
    build_package
    show_results
    
    print_success "Debian package build completed successfully!"
}

# Handle script arguments
case "${1:-}" in
    clean)
        clean_build
        print_success "Clean completed"
        ;;
    check)
        check_system
        check_dependencies
        print_success "System check completed"
        ;;
    help|--help|-h)
        echo "Usage: $0 [clean|check|help]"
        echo ""
        echo "Commands:"
        echo "  clean    Clean build directory and generated files"
        echo "  check    Check system and build dependencies"
        echo "  help     Show this help message"
        echo ""
        echo "Default: Build the Debian package"
        ;;
    *)
        main
        ;;
esac
