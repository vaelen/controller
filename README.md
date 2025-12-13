# Satellite Groundstation Controller

An RTEMS-based satellite groundstation controller that coordinates GPS positioning, TLE database management, pass prediction, and antenna control for automated satellite tracking.

## Overview

This controller runs on embedded systems using RTEMS (Real-Time Executive for Multiprocessor Systems) and provides:

- **GPS Integration**: Parses NMEA sentences (GGA/RMC) from a GPS receiver for precise location and time
- **TLE Database**: Manages Two-Line Element sets for satellite orbit data
- **Pass Prediction**: Calculates upcoming satellite passes using SGP4/SDP4 propagation
- **Antenna Control**: Commands antenna rotator for automated satellite tracking

## Architecture

The system uses multiple RTEMS tasks communicating via message queues:

| Task            | Priority | Function                                |
|-----------------|----------|-----------------------------------------|
| Controller      |       10 | Main coordination, pass scheduling      |
| Pass Executor   |       15 | Controls rotator and radio during passes|
| Antenna         |       20 | Polls antenna rotator position          |
| GPS             |       30 | Reads NMEA data, extracts position/time |
| Pass Calculator |       40 | Computes upcoming satellite passes      |
| TLE Updater     |       50 | Updates satellite TLE database          |

Tasks are synchronized using binary semaphores with priority inheritance for shared resources (UART, TLE database, state).

## Building

### Prerequisites

- RTEMS 7 toolchain installed at `$HOME/rtems/7`
- ARM cross-compiler (`arm-rtems7-gcc`)

### Build Commands

```bash
# Initialize submodule (first time only)
git submodule init
git submodule update rtems_waf

# Configure
./waf configure --prefix=$HOME/rtems/7

# Build
./waf

# Clean
./waf clean
```

The executable is built at `build/arm-rtems7-xilinx_zynq_a9_qemu/sattrack_controller.exe`.

## Running

### QEMU Emulation

```bash
./run.sh
```

This runs the controller in QEMU with:

- `/dev/ttyS0` -> stdio (console/log output)
- `/dev/ttyS1` -> `/dev/ttyUSB0` (GPS receiver)
- `/dev/ttyS2` -> `/dev/ttyUSB1` (rotator)
- `/dev/ttyS3` -> `/dev/ttyUSB2` (radio)

### Hardware Requirements

- GPS receiver outputting NMEA 0183 at 9600 baud (8N1)
- Antenna rotator with serial control interface
- Radio with serial control interface

An emulator for these devices can be created using a Pi Pico, ESP32, or STM32 microcontroller using the [serial-device-emulator](https://github.com/vaelen/serial-device-emulator) firmware.

## Example Log Output

```txt
*** GROUNDSTATION CONTROLLER ***
[??:??:??] INFO  [LOG    ] Logging initialized
[??:??:??] INFO  [INIT   ] Semaphores created successfully
[??:??:??] INFO  [INIT   ] Message queues created successfully
[??:??:??] INFO  [INIT   ] Tasks created successfully
[??:??:??] INFO  [INIT   ] All tasks started successfully
[??:??:??] INFO  [INIT   ] Initialization complete. Tasks running.
[??:??:??] INFO  [CTRL   ] Controller task started
[??:??:??] INFO  [GPS    ] Task started
[??:??:??] INFO  [GPS    ] Serial port /dev/ttyS1 opened successfully
[??:??:??] INFO  [ROTATOR] Status task started
[??:??:??] INFO  [EXEC   ] Pass executor task started
[??:??:??] INFO  [ANTENNA] Location task started
[??:??:??] INFO  [PASS   ] Calculator task started
[??:??:??] INFO  [TLE    ] Updater task started
[03:31:43] INFO  [CTRL   ] GPS: lat=35.587078 deg lon=139.490148 deg alt=0.08 km
[03:31:43] INFO  [CTRL   ] GPS Time: 2025-12-13 03:31:43 UTC
```

## Configuration

Key constants in `controller.c`:

```c
#define GPS_DEVICE_PATH       "/dev/ttyS1"
#define GPS_BAUD_RATE         B9600
#define GPS_FLOW_CONTROL      false

#define ROTATOR_DEVICE_PATH   "/dev/ttyS2"
#define ROTATOR_BAUD_RATE     B9600
#define ROTATOR_FLOW_CONTROL  false

#define RADIO_DEVICE_PATH     "/dev/ttyS3"
#define RADIO_BAUD_RATE       B9600
#define RADIO_FLOW_CONTROL    false

#define MAX_SATELLITES        32
```

## File Structure

```
sattrack_controller/
├── controller.c    # Main application, task implementations
├── init.c          # RTEMS configuration
├── sgp4.c/h        # SGP4/SDP4 satellite propagation
├── nmea.c/h        # NMEA 0183 parser
├── date.h          # Date/time utilities (third-party)
├── wscript         # Waf build configuration
├── run.sh          # QEMU launch script
└── docs/
    └── NMEA.md     # NMEA sentence format reference
```

## Libraries

### SGP4/SDP4

Implementation of the SGP4/SDP4 satellite propagation algorithms based on the Vallado reference implementation. Supports:
- TLE parsing
- Satellite position/velocity propagation
- Coordinate transforms (ECI, ECEF, Geodetic, ENU)
- Look angle calculations

### NMEA Parser

Parses NMEA 0183 sentences from GPS receivers:
- Supports all GNSS talker IDs (GP, GN, GA, GB, GL, GQ)
- GGA sentences for position and altitude
- RMC sentences for date and time
- Checksum validation
- Non-blocking buffered reads

## License

MIT License - See individual source files for details.

## Author

Andrew C. Young <andrew@vaelen.org>
