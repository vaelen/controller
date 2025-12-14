#!/bin/bash
#
# Deploy RTEMS application to TFTP server for BeagleBone Black
#
# Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
# SPDX-License-Identifier: MIT

set -e

TFTP_DIR="/srv/tftp"
BUILD_DIR="build/arm-rtems7-beagleboneblack"
EXE_NAME="controller.exe"
BIN_NAME="controller.bin"
IMG_NAME="controller.img"
EXE_PATH="$BUILD_DIR/$EXE_NAME"
BIN_PATH="$TFTP_DIR/$BIN_NAME"
IMG_PATH="$TFTP_DIR/$IMG_NAME"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    echo "Usage: $0 [-b|--build] [-h|--help]"
    echo ""
    echo "Options:"
    echo "  -b, --build    Build the project before deploying"
    echo "  -h, --help     Show this help message"
    echo ""
    echo "Deploys $EXE_NAME to $TFTP_DIR for TFTP boot on BeagleBone Black."
}

build_project() {
    echo -e "${YELLOW}Building project...${NC}"
    ./waf
    echo -e "${GREEN}Build complete.${NC}"
}

DO_BUILD=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--build)
            DO_BUILD=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            exit 1
            ;;
    esac
done

# Build if requested
if [ "$DO_BUILD" = true ]; then
    build_project
fi

# Check that executable exists
if [ ! -f "$EXE_PATH" ]; then
    echo -e "${RED}Error: $EXE_PATH not found.${NC}"
    echo "Run './waf' to build, or use '$0 --build' to build and deploy."
    exit 1
fi

# Check that TFTP directory exists
if [ ! -d "$TFTP_DIR" ]; then
    echo -e "${RED}Error: TFTP directory $TFTP_DIR does not exist.${NC}"
    echo "Please set up the TFTP server first. See README.md for instructions."
    exit 1
fi

# Convert ELF to binary
echo -e "${YELLOW}Converting ELF to binary...${NC}"
arm-rtems7-objcopy -O binary "$EXE_PATH" "$BIN_PATH"

# Compress and create U-Boot image
echo -e "${YELLOW}Creating U-Boot image...${NC}"
gzip -9 -f -k "$BIN_PATH"
mkimage -A arm -O linux -T kernel -a 0x80000000 -e 0x80000000 -n RTEMS -d "${BIN_PATH}.gz" "$IMG_PATH"
rm -f "${BIN_PATH}.gz"

# Show file info
SIZE=$(stat --printf="%s" "$IMG_PATH")
SIZE_KB=$((SIZE / 1024))
echo -e "${GREEN}Deployed:${NC} $IMG_PATH (${SIZE_KB} KB)"
echo ""
echo "Reset or power cycle the BeagleBone Black to boot the new firmware."
