#!/bin/bash

qemu-system-arm -M xilinx-zynq-a9 -m 256M -no-reboot -serial null \
    -serial mon:stdio -nographic \
    -kernel build/arm-rtems7-xilinx_zynq_a9_qemu/sattrack_controller.exe