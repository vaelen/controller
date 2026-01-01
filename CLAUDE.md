# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## RTEMS Installation

The RTEMS 7 toolchain, kernel, and libbsd are installed at:

- **Toolchain/executables**: `$HOME/rtems/7` (e.g., `~/rtems/7/bin/arm-rtems7-gcc`)
- **Source code**: `$HOME/rtems/src`
- **rtems-libbsd**: Custom fork from [vaelen/rtems-libbsd](https://github.com/vaelen/rtems-libbsd) (branch: `7-freebsd-14-beaglebone-black`)

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

| File                 | Description                                       |
|----------------------|---------------------------------------------------|
| `controller.c`       | Main application, all task implementations        |
| `init.c`             | RTEMS configuration (confdefs.h)                  |
| `sgp4.c/h`           | SGP4/SDP4 satellite propagation algorithm         |
| `nmea.c/h`           | NMEA 0183 GPS parser                              |
| `config.c/h`         | Configuration system (INI file parsing)           |
| `log.c/h`            | Thread-safe logging system                        |
| `priority_queue.c/h` | Pass priority queue for scheduling                |
| `https_client.c/h`   | HTTPS client for TLE download                     |
| `date.h`             | Date/time utilities (third-party, Howard Hinnant) |

### Tasks

All tasks are defined in `controller.c`. Lower priority number = higher priority.

| Task            | Priority | Function                                                      |
|-----------------|----------|---------------------------------------------------------------|
| Pass Executor   |       10 | State machine for satellite tracking during passes            |
| Controller      |       20 | Main coordination, pass scheduling, message routing           |
| Rotator Command |       25 | Sends GS-232A commands to antenna rotator via UART            |
| Rotator Status  |       30 | Parses rotator position responses from UART                   |
| Radio Status    |       35 | Parses radio CAT protocol responses                           |
| Radio Frequency |       36 | Sends CAT query commands to radio                             |
| Antenna         |       40 | Polls antenna rotator position every 2 seconds                |
| GPS             |       50 | Reads NMEA sentences, extracts position/time                  |
| Pass Calculator |       60 | Computes upcoming satellite passes using SGP4                 |
| TLE Updater     |       70 | Downloads TLE data via HTTPS, updates satellite database      |
| Status          |       80 | Periodic system status logging (every 30 seconds default)     |

### Message Queues

| Queue Name             | Route                         | Message Type                 |
|------------------------|-------------------------------|------------------------------|
| `g_gps_queue`          | GPS -> Controller             | `gps_message_t`              |
| `g_antenna_queue`      | Rotator Status -> Controller  | `antenna_position_message_t` |
| `g_tle_queue`          | TLE Updater -> Controller     | `tle_update_message_t`       |
| `g_pass_queue`         | Pass Calculator -> Controller | `pass_message_t`             |
| `g_radio_queue`        | Radio Status -> Controller    | `radio_status_message_t`     |
| `g_ctrl_cmd_queue`     | External -> Controller        | `ctrl_cmd_message_t`         |
| `g_executor_cmd_queue` | Controller -> Pass Executor   | `executor_command_message_t` |
| `g_rotator_cmd_queue`  | Pass Executor -> Rotator Cmd  | `rotator_command_message_t`  |

### Semaphores (Mutexes)

| Mutex                 | Protects                                    |
|-----------------------|---------------------------------------------|
| `g_uart1_mutex`       | Rotator serial port access                  |
| `g_uart3_mutex`       | Radio serial port access                    |
| `g_state_mutex`       | Global state (observer, time, antenna pos)  |
| `g_tle_database_mutex`| Satellite TLE database                      |
| `g_pass_queue_mutex`  | Pass priority queue and scheduled pass      |

## Pass Execution System

### Pass Executor State Machine

The pass executor (`pass_executor_task`) uses a state machine:

1. **EXEC_STATE_IDLE** - Blocking wait for START_PASS command
2. **EXEC_STATE_PREPOSITIONING** - Antenna moving to AOS position
3. **EXEC_STATE_WAITING_AOS** - Antenna positioned, waiting for AOS time
4. **EXEC_STATE_TRACKING** - Active tracking with 100ms update loop
5. **EXEC_STATE_COMPLETING** - Pass ended, parking antenna

### Tracking Loop (100ms default)

During TRACKING state, each iteration:

1. Gets current GPS time and observer location
2. Propagates satellite position using SGP4
3. Calculates look angles (azimuth, elevation, range)
4. Calculates Doppler shift from satellite velocity
5. If position changed >= threshold, sends rotator command
6. If Doppler changed >= threshold, logs (radio commands stubbed)

### Doppler Calculation

```c
// Range rate from ECEF velocity dot unit range vector
double range_rate = dot(sat_vel_ecef, unit_range_vector);  // km/s

// Doppler factor (multiply nominal frequency)
double doppler_factor = 1.0 - (range_rate / 299792.458);
```

Positive range_rate = satellite moving away = lower frequency.

### GS-232A Rotator Commands

The rotator command task sends:

- `W<aaa> <eee>\r` - Move to position (e.g., "W180 045")
- `S\r` - Stop movement
- `C2\r` - Query position (sent by antenna location task)

## Key Constraints

- **C99**: All code must be C99 compatible
- **No exceptions**: All error handling uses error codes and the logging system for diagnostics
- **No heap allocation** in core code: Uses fixed-size buffers and structs
- **Non-blocking I/O**: Serial reads use termios with `VMIN=0, VTIME=0`
- **MAX_SATELLITES**: Fixed at 64 satellites in TLE database

## Configuration Parameters

Key tracking parameters in `[pass]` section of config.ini:

| Parameter                | Default | Description                              |
|--------------------------|---------|------------------------------------------|
| `tracking_poll_ms`       | 100     | Tracking loop interval (10-1000 ms)      |
| `rotator_threshold`      | 1.0     | Min degrees change to send command       |
| `doppler_threshold`      | 1.0     | Min kHz change to update radio           |
| `preposition_margin`     | 30      | Seconds before AOS to finish positioning |
| `prep_time`              | 300     | Seconds before AOS to start preparation  |

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

All angles in the library are in **radians** unless explicitly noted (e.g., config values are in degrees, orbital elements from TLE are in degrees internally).

## Serial Port Configuration

| Port         | Device  | Function        | Baud   |
|--------------|---------|-----------------|--------|
| `/dev/ttyS0` | Console | Log output      | 115200 |
| `/dev/ttyS1` | GPS     | NMEA input      | 9600   |
| `/dev/ttyS2` | Rotator | GS-232A control | 9600   |
| `/dev/ttyS3` | Radio   | Yaesu CAT       | 38400  |

## Key Data Structures

### Pass Information

```c
typedef struct pass_info {
    int norad_id;               // Satellite NORAD catalog ID
    double aos_jd;              // Acquisition of signal (Julian Date)
    double los_jd;              // Loss of signal (Julian Date)
    double max_elevation_rad;   // Maximum elevation angle (radians)
    double aos_azimuth_rad;     // Azimuth at AOS (radians)
    double los_azimuth_rad;     // Azimuth at LOS (radians)
} pass_info_t;
```

### Executor Tracking State

```c
typedef struct executor_tracking_state {
    executor_state_t state;           // Current state machine state
    pass_info_t current_pass;         // Pass being executed
    sgp4_state_t sat_state;           // Copied from TLE database at start
    char sat_name[SGP4_TLE_NAME_LEN]; // For display
    double last_cmd_azimuth_rad;      // Last commanded position
    double last_cmd_elevation_rad;
    double current_azimuth_rad;       // Current calculated position
    double current_elevation_rad;
    double current_range_km;
    double last_doppler_factor;       // For threshold comparison
    double range_rate_km_s;           // Radial velocity
} executor_tracking_state_t;
```

## Helper Functions for Tracking

In `controller.c`, key helper functions:

- `get_satellite_state()` - Copy SGP4 state from TLE database (requires mutex)
- `calculate_doppler_factor()` - Compute Doppler from ECEF velocity vectors
- `calculate_tracking_data()` - Full position + Doppler calculation at given time
- `calculate_look_angles_at_time()` - Position-only calculation (used by pass calculator)
