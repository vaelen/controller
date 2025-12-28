# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## RTEMS Installation

The RTEMS 7 toolchain, kernel, and libbsd are installed at:
- **Toolchain/executables**: `$HOME/rtems/7` (e.g., `~/rtems/7/bin/arm-rtems7-gcc`)
- **Source code**: `$HOME/rtems/src`
- **rtems-libbsd**: Custom fork from https://github.com/vaelen/rtems-libbsd (branch: `7-freebsd-14-beaglebone-black`)

**Note:** This project requires the custom rtems-libbsd fork for BeagleBone Black SD card and networking support. Use `buildset/bbb.ini` when building rtems-libbsd.

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

The executable is built at `build/arm-rtems7-<bsp>/controller.exe` (e.g., `build/arm-rtems7-beagleboneblack/controller.exe`).

## Running

### BeagleBone Black (Hardware)

```bash
./deploy.sh --build
```

Builds the project and deploys to the TFTP server at `/srv/tftp/controller.img`. Reset the BeagleBone Black to boot via TFTP. See README.md for TFTP setup instructions.

### QEMU Emulation

```bash
./run-emulator.sh
```

Runs the controller in QEMU with simulated serial ports for GPS, rotator, and radio.

## Architecture Overview

This is a satellite tracking controller for RTEMS embedded systems. The codebase is written in C99 and designed to run without heap allocation and without exceptions.

### Source Files

| File           | Description                                        |
|----------------|----------------------------------------------------|
| `controller.c` | Main application, task implementations             |
| `init.c`       | RTEMS configuration (confdefs.h)                   |
| `sgp4.c/h`     | SGP4/SDP4 satellite propagation algorithm          |
| `nmea.c/h`     | NMEA 0183 GPS parser                               |
| `date.h`       | Date/time utilities (third-party, Howard Hinnant)  |

### Tasks

| Task            | Priority | Function                                     |
|-----------------|----------|----------------------------------------------|
| Controller      |       10 | Main coordination, pass scheduling           |
| Pass Executor   |       15 | Controls rotator and radio during passes     |
| Antenna         |       20 | Polls antenna rotator position via UART      |
| GPS             |       30 | Reads NMEA sentences, extracts position/time |
| Pass Calculator |       40 | Computes upcoming satellite passes           |
| TLE Updater     |       50 | Updates satellite TLE database               |

## Key Constraints

- **C99**: All code must be C99 compatible
- **No exceptions**: All error handling uses error codes and the logging system for diagnostics
- **No heap allocation** in core code: Uses fixed-size buffers and structs
- **Non-blocking I/O**: Serial reads use termios with `VMIN=0, VTIME=0`

## Logging

The logging system uses a semaphore (mutex) to protect `printf` from concurrent access. Each task can call the logging macros directly, and the semaphore ensures output is not interleaved.

```c
LOG_DEBUG("TAG", "Debug message: %d", value);
LOG_INFO("TAG", "Informational message");
LOG_WARN("TAG", "Warning message");
LOG_ERROR("TAG", "Error message: %s", error_str);
```

Output format: `[HH:MM:SS] LEVEL [TAG    ] message`

Log levels: DEBUG < INFO < WARN < ERROR. Default level is INFO.

## Copyright Header Requirement

All new source files must include the following copyright header:

```c
/*
 * [File description]
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */
```

Do not modify third-party files (e.g., `date.h`).

## Coordinate Systems

- **ECI/TEME**: True Equator Mean Equinox frame (SGP4 output)
- **ECEF**: Earth-Centered Earth-Fixed
- **Geodetic**: lat/lon/alt (WGS84 ellipsoid)
- **ENU**: East-North-Up local tangent plane

All angles in the library are in **radians** unless explicitly noted (e.g., orbital elements from TLE are in degrees internally).

## Serial Port Configuration

| Port         | Device  | Function      | Baud   |
|--------------|---------|---------------|--------|
| `/dev/ttyS0` | Console | Log output    | 115200 |
| `/dev/ttyS1` | GPS     | NMEA input    | 9600   |
| `/dev/ttyS2` | Rotator | Az/El control | 9600   |
| `/dev/ttyS3` | Radio   | Frequency/PTT | 9600   |