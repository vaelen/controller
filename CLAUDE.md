# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This is an RTEMS application using the Waf build system with the rtems_waf submodule.

```bash
# Initialize submodule (first time only)
git submodule init
git submodule update rtems_waf

# Configure (requires RTEMS 7 toolchain installed at $HOME/rtems/7)
./waf configure --prefix=$HOME/rtems/7

# Build
./waf

# Clean
./waf clean
```

The executable is built at `build/arm-rtems7-xilinx_zynq_a9_qemu/sattrack_controller.exe`.

## Architecture Overview

This is a satellite tracking controller for RTEMS embedded systems. The codebase is designed to run without heap allocation and without exceptions (for RTEMS compatibility).

### Core Libraries

**sgp4.hpp / sgp4.cpp** - SGP4/SDP4 satellite propagation algorithm
- Based on Vallado reference implementation from CelesTrak
- Namespace: `sgp4`
- Key types: `State`, `Elements`, `Result`, `ErrorCode`
- Main API: `initialize()` returns `ErrorCode`, `propagate()` returns `Result` with error field
- Already heap-free; uses fixed-size structs only

**satellite.hpp / satellite.cpp** - High-level satellite tracking
- Namespace: `sattrack`
- `Satellite` class: TLE parsing, SGP4 wrapper, position/velocity computation
- Coordinate transforms: ECI ↔ ECEF ↔ Geodetic ↔ ENU
- Pass prediction: `findNextPass()` for satellite visibility calculations
- Uses fixed-size char arrays instead of std::string (MAX_SATELLITE_NAME_LEN, MAX_DESIGNATOR_LEN)
- TLE database I/O functions still use heap allocation (acceptable for file operations)

### RTEMS Entry Points

**init.cpp** - RTEMS configuration (confdefs.h)
**controller.cpp** - Main application with `extern "C" rtems_task Init()`

## Key Constraints

- **No exceptions**: All error handling uses `ErrorCode` enum and `printf()` for diagnostics
- **No heap allocation** in core library: Uses fixed-size buffers (`TLEString`, char arrays)
- **C++17 with `-fno-exceptions -fno-rtti`**
- RTEMS Init function must have `extern "C"` linkage

## Coordinate Systems

- **ECI/TEME**: True Equator Mean Equinox frame (SGP4 output)
- **ECEF**: Earth-Centered Earth-Fixed
- **Geodetic**: lat/lon/alt (WGS84 ellipsoid)
- **ENU**: East-North-Up local tangent plane

All angles in the library are in **radians** unless explicitly noted (e.g., orbital elements from TLE are in degrees internally).
