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

- RTEMS 7 toolchain and BSP installed at `$HOME/rtems/7`
- Python 3.6+ (for RTEMS Source Builder)
- Standard build tools (gcc, g++, make, etc.)

### Installing RTEMS 7

The application requires the RTEMS 7 toolchain and a Board Support Package (BSP). Follow these steps to build and install them.

#### 1. Install Host Dependencies

On Debian/Ubuntu:

```bash
sudo apt-get install build-essential gcc g++ gdb git python3 python3-dev \
    python3-pip unzip pax bison flex texinfo libncurses5-dev zlib1g-dev \
    libexpat1-dev libpython3-dev
```

#### 2. Clone RTEMS Source Builder

```bash
mkdir -p $HOME/rtems/src
cd $HOME/rtems/src
git clone https://gitlab.rtems.org/rtems/tools/rtems-source-builder.git rsb
cd rsb
git checkout 7
```

#### 3. Build the ARM Toolchain

This builds the cross-compiler, binutils, GDB, and other tools:

```bash
cd $HOME/rtems/src/rsb/rtems
../source-builder/sb-set-builder --prefix=$HOME/rtems/7 7/rtems-arm
```

This takes approximately 25-30 minutes and requires about 10GB of temporary disk space.

#### 4. Clone and Build the RTEMS Kernel

```bash
cd $HOME/rtems/src
git clone https://gitlab.rtems.org/rtems/rtos/rtems.git
cd rtems
git checkout 7

# Add the toolchain to your PATH
export PATH=$HOME/rtems/7/bin:$PATH
```

#### 5. Build a Board Support Package (BSP)

Choose and build a BSP for your target hardware:

**For BeagleBone Black:**

```bash
cd $HOME/rtems/src/rtems
./waf bspdefaults --rtems-bsps=arm/beagleboneblack > config.ini
./waf configure --prefix=$HOME/rtems/7
./waf
./waf install
```

**For QEMU (Xilinx Zynq A9):**

```bash
cd $HOME/rtems/src/rtems
./waf bspdefaults --rtems-bsps=arm/xilinx_zynq_a9_qemu > config.ini
./waf configure --prefix=$HOME/rtems/7
./waf
./waf install
```

You can build multiple BSPs by repeating step 5 with different BSP names.

#### 6. Verify Installation

```bash
ls $HOME/rtems/7/arm-rtems7/
```

You should see directories for each installed BSP (e.g., `beagleboneblack`, `xilinx_zynq_a9_qemu`).

### Build Commands

```bash
# Initialize submodule (first time only)
git submodule init
git submodule update rtems_waf

# Configure for BeagleBone Black
./waf configure --prefix=$HOME/rtems/7 --rtems-bsps=arm/beagleboneblack

# Or configure for QEMU
./waf configure --prefix=$HOME/rtems/7 --rtems-bsps=arm/xilinx_zynq_a9_qemu

# Build
./waf

# Clean
./waf clean
```

The executable is built at `build/arm-rtems7-<bsp>/sattrack_controller.exe` (e.g., `build/arm-rtems7-beagleboneblack/sattrack_controller.exe`).

## Running

### BeagleBone Black (TFTP Boot)

The recommended development workflow uses TFTP boot for fast iteration:

1. **Set up the TFTP server** (one-time):

   ```bash
   # Install TFTP server
   sudo apt install tftpd-hpa

   # Create TFTP directory with proper permissions
   sudo mkdir -p /srv/tftp
   sudo chown $USER:$USER /srv/tftp
   sudo chmod 755 /srv/tftp

   # Edit /etc/default/tftpd-hpa and set:
   #   TFTP_DIRECTORY="/srv/tftp"
   #   TFTP_OPTIONS="--secure --create"

   # Restart service
   sudo systemctl restart tftpd-hpa
   sudo systemctl enable tftpd-hpa
   ```

2. **Prepare SD card for U-Boot** (one-time):

   Create a small FAT32 partition on an SD card and add a `uEnv.txt` file:

   ```ini
   bootdelay=3
   ipaddr=192.168.68.15
   serverip=192.168.68.23
   netmask=255.255.252.0
   uenvcmd=run boot
   boot=tftp 0x80800000 controller.img; ext4load mmc 1:1 0x88000000 /boot/dtbs/4.19.94-ti-r42/am335x-boneblack.dtb; bootm 0x80800000 - 0x88000000
   ```

   Adjust `ipaddr` and `serverip` for your network. Insert the SD card into the BBB.

3. **Deploy and run**:

   ```bash
   # Build and deploy to TFTP server
   ./deploy.sh --build

   # Or just deploy (if already built)
   ./deploy.sh
   ```

   Power cycle or reset the BeagleBone Black. It will automatically:
   - Load the firmware via TFTP
   - Execute the RTEMS application

#### Network Configuration

| Device      | IP Address      | Netmask       |
|-------------|-----------------|---------------|
| Workstation | 192.168.68.23   | 255.255.252.0 |
| BBB         | 192.168.68.15   | 255.255.252.0 |

#### Troubleshooting TFTP Boot

- **"TFTP error: 'Permission denied'"**: Check TFTP directory permissions
- **"TFTP error: 'File not found'"**: Run `./deploy.sh` to copy the executable
- **No network response**: Verify Ethernet cable and IP addresses
- **U-Boot doesn't read uEnv.txt**: Ensure SD card has FAT32 partition and is inserted

### QEMU Emulation

```bash
./run-emulator.sh
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

```text
sattrack_controller/
├── controller.c      # Main application, task implementations
├── init.c            # RTEMS configuration
├── sgp4.c/h          # SGP4/SDP4 satellite propagation
├── nmea.c/h          # NMEA 0183 parser
├── date.h            # Date/time utilities (third-party)
├── wscript           # Waf build configuration
├── deploy.sh         # Deploy to TFTP server for BBB
├── run-emulator.sh   # QEMU launch script
└── docs/
    └── NMEA.md       # NMEA sentence format reference
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
