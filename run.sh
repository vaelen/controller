#!/bin/bash

# Serial port mapping for Realview PBX-A9:
#   ttyS0 -> Console (stdio)
#   ttyS1 -> GPS (/dev/ttyUSB0)
#   ttyS2 -> Rotator (/dev/ttyACM0)
#   ttyS3 -> Radio (/dev/ttyACM1)

qemu-system-arm -M realview-pbx-a9 -m 256M -no-reboot \
    -serial mon:stdio \
    -serial /dev/ttyUSB0 \
    -serial /dev/ttyACM1 \
    -serial /dev/ttyACM0 \
    -nographic \
    -kernel build/arm-rtems7-realview_pbx_a9_qemu/sattrack_controller.exe
