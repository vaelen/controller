#!/bin/bash

# Test script using PTYs instead of real serial devices.
# QEMU will print the PTY paths (e.g., "/dev/pts/X") that you can
# connect to with: screen /dev/pts/X 9600
#
# Serial port mapping for Realview PBX-A9:
#   ttyS0 -> Console (stdio)
#   ttyS1 -> GPS (pty, printed to stderr)
#   ttyS2 -> Rotator (pty, printed to stderr)
#   ttyS3 -> Radio (pty, printed to stderr)

echo "Starting QEMU with PTY serial ports..."
echo "Connect to the PTY paths printed below with: screen /dev/pts/X 9600"
echo ""

qemu-system-arm -M realview-pbx-a9 -m 256M -no-reboot \
    -chardev stdio,id=con0,mux=on,signal=off \
    -serial chardev:con0 \
    -mon chardev=con0 \
    -chardev pty,id=gps \
    -serial chardev:gps \
    -chardev pty,id=rotator \
    -serial chardev:rotator \
    -chardev pty,id=radio \
    -serial chardev:radio \
    -nographic \
    -kernel build/arm-rtems7-realview_pbx_a9_qemu/sattrack_controller.exe
