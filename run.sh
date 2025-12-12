#!/bin/bash

# Serial port mapping:
#   ttyS0 -> Console (stdio)
#   ttyS1 -> GPS (/dev/ttyUSB0)
#   ttyS2 -> Rotator (/dev/ttyUSB1)
#   ttyS3 -> Radio (/dev/ttyUSB2)

qemu-system-arm -M xilinx-zynq-a9 -m 256M -no-reboot \
    -serial mon:stdio \
    -serial /dev/ttyUSB0 \
    -serial /dev/ttyUSB1 \
    -serial /dev/ttyUSB2 \
    -nographic \
    -kernel build/arm-rtems7-xilinx_zynq_a9_qemu/sattrack_controller.exe