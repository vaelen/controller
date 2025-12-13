#!/bin/bash

# Serial port mapping for Realview PBX-A9:
#   ttyS0 -> Console (stdio)
#   ttyS1 -> GPS (/dev/ttyUSB0)
#   ttyS2 -> Rotator (/dev/ttyACM1)
#   ttyS3 -> Radio (/dev/ttyACM0)

GPS_DEV="/dev/ttyUSB0"
ROTATOR_DEV="/dev/ttyACM1"
RADIO_DEV="/dev/ttyACM0"

# Temporary directory for PTY links
PTY_DIR=$(mktemp -d)
GPS_PTY="$PTY_DIR/gps"
ROTATOR_PTY="$PTY_DIR/rotator"
RADIO_PTY="$PTY_DIR/radio"

# Track socat PIDs for cleanup
SOCAT_PIDS=()

cleanup() {
    echo ""
    echo "Cleaning up..."
    for pid in "${SOCAT_PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    rm -rf "$PTY_DIR"
}
trap cleanup EXIT

# Use socat to bridge real serial devices to PTYs
# QEMU's serial passthrough has issues with real devices, but works fine with PTYs
echo "Starting socat bridges..."

if [ -e "$GPS_DEV" ]; then
    socat -d -d "PTY,raw,echo=0,link=$GPS_PTY" "$GPS_DEV,raw,echo=0,b9600" 2>/dev/null &
    SOCAT_PIDS+=($!)
    sleep 0.5
    if [ -e "$GPS_PTY" ]; then
        echo "  GPS: $GPS_DEV <-> $GPS_PTY"
    else
        echo "  GPS: failed to create PTY bridge"
        GPS_PTY=""
    fi
else
    echo "  GPS ($GPS_DEV): not found"
    GPS_PTY=""
fi

if [ -e "$ROTATOR_DEV" ]; then
    socat -d -d "PTY,raw,echo=0,link=$ROTATOR_PTY" "$ROTATOR_DEV,raw,echo=0,b9600" 2>/dev/null &
    SOCAT_PIDS+=($!)
    sleep 0.5
    if [ -e "$ROTATOR_PTY" ]; then
        echo "  Rotator: $ROTATOR_DEV <-> $ROTATOR_PTY"
    else
        echo "  Rotator: failed to create PTY bridge"
        ROTATOR_PTY=""
    fi
else
    echo "  Rotator ($ROTATOR_DEV): not found"
    ROTATOR_PTY=""
fi

if [ -e "$RADIO_DEV" ]; then
    socat -d -d "PTY,raw,echo=0,link=$RADIO_PTY" "$RADIO_DEV,raw,echo=0,b9600" 2>/dev/null &
    SOCAT_PIDS+=($!)
    sleep 0.5
    if [ -e "$RADIO_PTY" ]; then
        echo "  Radio: $RADIO_DEV <-> $RADIO_PTY"
    else
        echo "  Radio: failed to create PTY bridge"
        RADIO_PTY=""
    fi
else
    echo "  Radio ($RADIO_DEV): not found"
    RADIO_PTY=""
fi

echo ""

# Build QEMU arguments
QEMU_ARGS=(
    -M realview-pbx-a9
    -m 256M
    -no-reboot
    -chardev stdio,id=con0,mux=on,signal=off
    -serial chardev:con0
    -mon chardev=con0
)

if [ -n "$GPS_PTY" ] && [ -e "$GPS_PTY" ]; then
    QEMU_ARGS+=(-chardev "serial,id=gps,path=$GPS_PTY" -serial chardev:gps)
else
    QEMU_ARGS+=(-chardev null,id=gps -serial chardev:gps)
fi

if [ -n "$ROTATOR_PTY" ] && [ -e "$ROTATOR_PTY" ]; then
    QEMU_ARGS+=(-chardev "serial,id=rotator,path=$ROTATOR_PTY" -serial chardev:rotator)
else
    QEMU_ARGS+=(-chardev null,id=rotator -serial chardev:rotator)
fi

if [ -n "$RADIO_PTY" ] && [ -e "$RADIO_PTY" ]; then
    QEMU_ARGS+=(-chardev "serial,id=radio,path=$RADIO_PTY" -serial chardev:radio)
else
    QEMU_ARGS+=(-chardev null,id=radio -serial chardev:radio)
fi

QEMU_ARGS+=(
    -nographic
    -kernel build/arm-rtems7-realview_pbx_a9_qemu/sattrack_controller.exe
)

echo "Starting QEMU..."
qemu-system-arm "${QEMU_ARGS[@]}"
