#!/bin/bash

echo "=== Running SGP4 Tests ==="
qemu-system-arm -M xilinx-zynq-a9 -m 256M -no-reboot -serial null \
    -serial mon:stdio -nographic \
    -kernel build/arm-rtems7-xilinx_zynq_a9_qemu/test_sgp4.exe
SGP4_RESULT=$?

echo ""
echo "=== Running Satellite Tests ==="
qemu-system-arm -M xilinx-zynq-a9 -m 256M -no-reboot -serial null \
    -serial mon:stdio -nographic \
    -kernel build/arm-rtems7-xilinx_zynq_a9_qemu/test_satellite.exe
SAT_RESULT=$?

echo ""
echo "=== Running Coordinate Tests ==="
qemu-system-arm -M xilinx-zynq-a9 -m 256M -no-reboot -serial null \
    -serial mon:stdio -nographic \
    -kernel build/arm-rtems7-xilinx_zynq_a9_qemu/test_coordinates.exe
COORD_RESULT=$?

echo ""
echo "=== All Tests Complete ==="
if [ $SGP4_RESULT -eq 0 ] && [ $SAT_RESULT -eq 0 ] && [ $COORD_RESULT -eq 0 ]; then
    echo "All tests PASSED"
    exit 0
else
    echo "Some tests FAILED"
    exit 1
fi
