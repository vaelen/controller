# Satellite Groundstation Controller

An RTEMS-based satellite groundstation controller that coordinates GPS positioning, TLE database management, pass prediction, and antenna control for automated satellite tracking.

## Overview

This controller runs on embedded systems using RTEMS (Real-Time Executive for Multiprocessor Systems) and provides:

- **GPS Integration**: Parses NMEA sentences (GGA/RMC) from a GPS receiver for precise location and time
- **TLE Database**: Manages Two-Line Element sets for satellite orbit data with automatic HTTPS updates
- **Pass Prediction**: Calculates upcoming satellite passes using SGP4/SDP4 propagation
- **Antenna Control**: Commands antenna rotator for automated satellite tracking using GS-232A protocol
- **Doppler Tracking**: Real-time Doppler shift calculation for radio frequency correction

## Architecture

The system uses multiple RTEMS tasks communicating via message queues:

| Task            | Priority | Function                                              |
|-----------------|----------|-------------------------------------------------------|
| Pass Executor   |       10 | Controls rotator and radio during satellite passes    |
| Controller      |       20 | Main coordination, pass scheduling                    |
| Rotator Command |       25 | Sends GS-232A commands to antenna rotator             |
| Rotator Status  |       30 | Parses rotator position responses                     |
| Radio Status    |       35 | Parses radio CAT protocol responses                   |
| Radio Frequency |       36 | Sends CAT query commands to radio                     |
| Antenna         |       40 | Polls antenna rotator position periodically           |
| GPS             |       50 | Reads NMEA data, extracts position/time               |
| Pass Calculator |       60 | Computes upcoming satellite passes                    |
| TLE Updater     |       70 | Downloads and updates satellite TLE database          |
| Status          |       80 | Periodic system status logging                        |

Tasks are synchronized using binary semaphores with priority inheritance for shared resources (UART, TLE database, state).

## System Operation

### TLE Download and Database Management

The TLE Updater task automatically downloads Two-Line Element data from a configurable URL (default: Celestrak amateur radio satellites). The process:

1. Waits for GPS time to be valid (required for HTTPS certificate validation)
2. Downloads TLE data via HTTPS
3. Filters satellites by configured NORAD IDs
4. Parses and validates TLE data, initializes SGP4 state for each satellite
5. Stores in fixed-size database (up to 64 satellites)
6. Repeats at configurable interval (default: 6 hours)

### Pass Calculation

The Pass Calculator task predicts upcoming satellite passes:

1. Scans the prediction window (default: 60 minutes ahead) in 60-second steps
2. For each satellite, uses SGP4 to propagate position and calculate look angles
3. Detects visibility transitions using binary search for 1-second precision
4. Finds maximum elevation using ternary search
5. Filters passes below minimum elevation threshold
6. Stores passes in a priority queue (sorted by elevation for overlapping passes)
7. Sends discovered passes to the Controller task

### Pass Execution

When a pass is scheduled, the Pass Executor task controls tracking:

**State Machine:**

1. **IDLE** - Waiting for pass command from Controller
2. **PREPOSITIONING** - Moving antenna to AOS (Acquisition of Signal) position
3. **WAITING_AOS** - Antenna positioned, waiting for satellite to rise
4. **TRACKING** - Active real-time satellite tracking
5. **COMPLETING** - Pass ended, returning antenna to park position

**During Tracking:**

- Polls satellite position at configurable interval (default: 100ms)
- Calculates azimuth, elevation, and Doppler shift using SGP4
- Sends rotator commands when position changes exceed threshold (default: 1 degree)
- Calculates Doppler correction when shift exceeds threshold (default: 1 kHz)
- Radio frequency commands are stubbed for future implementation

### Rotator Control

The Rotator Command task receives position commands and sends GS-232A protocol commands:

- `W<aaa> <eee>` - Move to azimuth/elevation (e.g., "W180 045")
- `S` - Stop movement
- `C2` - Query current position (sent by Antenna Location task)

Position responses are parsed by the Rotator Status task.

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

```bash
cd $HOME/rtems/src/rtems
./waf bspdefaults --rtems-bsps=arm/beagleboneblack > config.ini
./waf configure --prefix=$HOME/rtems/7
./waf
./waf install
```

#### 6. Clone and Build rtems-libbsd

This project uses rtems-libbsd for networking and SD card support. A [custom fork](https://github.com/vaelen/rtems-libbsd/tree/7-freebsd-14-beaglebone-black) is required for BeagleBone Black that fixes driver issues in the official rtems-libbsd release.

**Clone the custom fork:**

```bash
cd $HOME/rtems/src
git clone -b 7-freebsd-14-beaglebone-black https://github.com/vaelen/rtems-libbsd.git
cd rtems-libbsd
git submodule init
git submodule update rtems_waf
```

**Build and install:**

```bash
./waf configure --prefix=$HOME/rtems/7 \
    --rtems-bsps=arm/beagleboneblack \
    --buildset=buildset/bbb.ini
./waf
./waf install
```

The `buildset/bbb.ini` configuration enables SD card and networking support while disabling components that are not needed or cause issues on BeagleBone Black.

For technical details about the BeagleBone Black fixes, see [sdcard.md](sdcard.md) and [networking.md](networking.md).

#### 7. Verify Installation

```bash
ls $HOME/rtems/7/arm-rtems7/
```

You should see a directory for the installed BSP (e.g., `beagleboneblack`).

### Build Commands

```bash
# Initialize submodule (first time only)
git submodule init
git submodule update rtems_waf

# Configure for BeagleBone Black
./waf configure --prefix=$HOME/rtems/7 --rtems-bsps=arm/beagleboneblack

# Build
./waf

# Clean
./waf clean
```

The executable is built at `build/arm-rtems7-<bsp>/controller.exe` (e.g., `build/arm-rtems7-beagleboneblack/controller.exe`).

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

### Hardware Requirements

- GPS receiver outputting NMEA 0183 at 9600 baud (8N1)
- Antenna rotator with GS-232A serial control interface
- Radio with serial control interface (Yaesu CAT protocol)

An emulator for these devices can be created using a Pi Pico, ESP32, or STM32 microcontroller using the [serial-device-emulator](https://github.com/vaelen/serial-device-emulator) firmware.

### BeagleBone Black Pin Assignments

The following table shows the physical P9 header pins used for each UART:

| Device  | UART  | Function | Pin   | Ball Name      | Mux Mode |
|---------|-------|----------|-------|----------------|----------|
| Console | UART0 | -        | J1    | (debug header) | -        |
| GPS     | UART1 | RXD      | P9.26 | uart1_rxd      | 0        |
| GPS     | UART1 | TXD      | P9.24 | uart1_txd      | 0        |
| Rotator | UART2 | RXD      | P9.22 | spi0_sclk      | 1        |
| Rotator | UART2 | TXD      | P9.21 | spi0_d0        | 1        |
| Radio   | UART4 | RXD      | P9.11 | gpmc_wait0     | 6        |
| Radio   | UART4 | TXD      | P9.13 | gpmc_wpn       | 6        |

**Notes:**

- UART0 is the debug console, directly accessible via the J1 serial header
- UART3 is not used (pins conflict with other functions)
- UART4 is mapped to `/dev/ttyS3` in software
- Pin muxing is configured in `console-config.c`

## Example Log Output

```txt
*** GROUNDSTATION CONTROLLER ***
[??:??:??] INFO  [LOG    ] Logging initialized
[??:??:??] INFO  [INIT   ] Semaphores created successfully
[??:??:??] INFO  [INIT   ] Message queues created successfully
[??:??:??] INFO  [INIT   ] Tasks created successfully
[??:??:??] INFO  [INIT   ] All tasks started successfully
[??:??:??] INFO  [EXEC   ] Pass executor task started
[??:??:??] INFO  [ROTCMD ] Rotator command task started
[??:??:??] INFO  [CTRL   ] Controller task started
[??:??:??] INFO  [ROTATOR] Status task started
[??:??:??] INFO  [RADIO  ] Status task started
[??:??:??] INFO  [ANTENNA] Location task started
[??:??:??] INFO  [GPS    ] Task started
[??:??:??] INFO  [PASS   ] Calculator task started
[??:??:??] INFO  [TLE    ] Updater task started
[??:??:??] INFO  [STATUS ] Status task started
[09:29:42] INFO  [CTRL   ] GPS: lat=35.5872 lon=139.4901 alt=0.05 km
[09:29:42] INFO  [CTRL   ] GPS Time: 2025-12-18 09:29:42 UTC
[09:30:11] INFO  [STATUS ] === System Status ===
[09:30:11] INFO  [STATUS ] GPS Time: 2025-12-18 09:30:11 UTC
[09:30:11] INFO  [STATUS ] GPS Pos: 35.5872, 139.4901, 52m
[09:30:11] INFO  [STATUS ] Antenna: az=0.0 el=0.0 deg
[09:30:11] INFO  [STATUS ] Radio: VFO-A active, mode=USB, preamp=IPO
[09:30:11] INFO  [STATUS ] TLE: 12 satellites loaded
[09:30:11] INFO  [STATUS ] Passes: 3 upcoming
[09:30:11] INFO  [STATUS ]   Satellite             NORAD       AOS       LOS  AOS Az  LOS Az   MaxEl
[09:30:11] INFO  [STATUS ]   ISS (ZARYA)           25544  09:45:12  09:56:34   225.3°   45.2°   67.8°
[09:35:00] INFO  [EXEC   ] Starting pass for NORAD 25544
[09:35:00] INFO  [EXEC   ]   AOS: 09:45:12, az=225.3 deg
[09:35:00] INFO  [EXEC   ]   Max elevation: 67.8 deg
[09:35:00] INFO  [EXEC   ] Prepositioning antenna to az=225.3 el=0.0
[09:44:42] INFO  [EXEC   ] Preposition complete, waiting for AOS (30 sec)
[09:45:12] INFO  [EXEC   ] AOS - Beginning tracking of ISS (ZARYA)
[09:56:34] INFO  [EXEC   ] LOS - Pass complete for ISS (ZARYA)
[09:56:34] INFO  [EXEC   ] Pass completed, returning to idle
```

## Configuration

The controller reads configuration from an INI-style file on the SD card. If no configuration file exists, a default one is created automatically on first boot.

### Configuration File Location

```text
/mnt/sd/config.ini
```

### Example Configuration

```ini
; Satellite Tracking Controller Configuration

[serial]
gps_device = /dev/ttyS1
gps_baud = 9600
gps_flow_control = false
rotator_device = /dev/ttyS2
rotator_baud = 9600
rotator_flow_control = false
radio_device = /dev/ttyS3
radio_baud = 38400
radio_flow_control = false

[observer]
latitude = 35.5872
longitude = 139.4901
altitude = 52.0

[files]
tle_path = /mnt/sd/sats.tle

[system]
log_level = INFO
status_interval = 30

[network]
enabled = true
interface = cpsw0
ipv4_enabled = true
ipv4_dhcp = true
ipv4_address = 192.168.1.100
ipv4_netmask = 255.255.255.0
ipv4_gateway = 192.168.1.1
ipv6_enabled = true
ipv6_dhcp = true
ipv6_slaac = true

[tle]
url = https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle
satellites = 25544, 43017, 43770
update_interval = 6

[pass]
prediction_window = 60
min_elevation = 5.0
min_schedule_elevation = 15.0
prep_time = 300
calc_interval = 300
status_display_count = 5
tracking_poll_ms = 100
rotator_threshold = 1.0
doppler_threshold = 1.0
preposition_margin = 30
```

### Configuration Sections

#### [serial] - Serial Port Configuration

| Key                    | Description                     | Default      |
|------------------------|---------------------------------|--------------|
| `gps_device`           | GPS serial port device path     | `/dev/ttyS1` |
| `gps_baud`             | GPS baud rate                   | `9600`       |
| `gps_flow_control`     | Enable hardware flow control    | `false`      |
| `rotator_device`       | Rotator serial port device path | `/dev/ttyS2` |
| `rotator_baud`         | Rotator baud rate               | `9600`       |
| `rotator_flow_control` | Enable hardware flow control    | `false`      |
| `radio_device`         | Radio serial port device path   | `/dev/ttyS3` |
| `radio_baud`           | Radio baud rate                 | `38400`      |
| `radio_flow_control`   | Enable hardware flow control    | `false`      |

#### [observer] - Observer Location

| Key         | Description                               | Default |
|-------------|-------------------------------------------|---------|
| `latitude`  | Observer latitude in degrees (N positive) | `0.0`   |
| `longitude` | Observer longitude in degrees (E positive)| `0.0`   |
| `altitude`  | Observer altitude in meters               | `0.0`   |

**Note:** GPS location always overrides these settings when a GPS receiver is connected.

#### [files] - File Paths

| Key        | Description                | Default             |
|------------|----------------------------|---------------------|
| `tle_path` | Path to TLE database file  | `/mnt/sd/sats.tle`  |

#### [system] - System Settings

| Key              | Description                              | Default |
|------------------|------------------------------------------|---------|
| `log_level`      | Logging level: DEBUG, INFO, WARN, ERROR  | `INFO`  |
| `status_interval`| Status output interval in seconds        | `30`    |

#### [network] - Network Configuration

| Key              | Description                          | Default      |
|------------------|--------------------------------------|--------------|
| `enabled`        | Enable network interface             | `true`       |
| `interface`      | Network interface name               | `cpsw0`      |
| `ipv4_enabled`   | Enable IPv4                          | `true`       |
| `ipv4_dhcp`      | Use DHCP for IPv4                    | `true`       |
| `ipv4_address`   | Static IPv4 address (if DHCP off)    | (empty)      |
| `ipv4_netmask`   | Subnet mask (if DHCP off)            | (empty)      |
| `ipv4_gateway`   | Default gateway (if DHCP off)        | (empty)      |
| `ipv6_enabled`   | Enable IPv6                          | `true`       |
| `ipv6_dhcp`      | Use DHCPv6                           | `true`       |
| `ipv6_slaac`     | Use SLAAC for IPv6                   | `true`       |
| `ipv6_address`   | Static IPv6 address                  | (empty)      |
| `ipv6_prefix_len`| IPv6 prefix length                   | `64`         |
| `ipv6_gateway`   | IPv6 gateway                         | (empty)      |

#### [tle] - TLE Update Configuration

| Key                | Description                                | Default                                    |
|--------------------|--------------------------------------------|--------------------------------------------|
| `url`              | URL for TLE download (HTTPS)               | Celestrak amateur radio satellites         |
| `satellites`       | Comma-separated list of NORAD IDs to track | (empty = all satellites in file)           |
| `update_interval`  | Hours between TLE updates                  | `6`                                        |

Default URL: `https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle`

#### [pass] - Pass Prediction and Tracking Configuration

| Key                      | Description                                          | Default |
|--------------------------|------------------------------------------------------|---------|
| `prediction_window`      | How far ahead to predict passes (minutes)            | `60`    |
| `min_elevation`          | Minimum elevation for pass detection (degrees)       | `5.0`   |
| `min_schedule_elevation` | Minimum max-elevation to schedule a pass (degrees)   | `15.0`  |
| `prep_time`              | Time before AOS to start preparation (seconds)       | `300`   |
| `calc_interval`          | How often to recalculate passes (seconds)            | `300`   |
| `status_display_count`   | Number of upcoming passes to show in status          | `5`     |
| `tracking_poll_ms`       | Tracking loop update interval (milliseconds)         | `100`   |
| `rotator_threshold`      | Minimum position change to command rotator (degrees) | `1.0`   |
| `doppler_threshold`      | Minimum Doppler change to update radio (kHz)         | `1.0`   |
| `preposition_margin`     | Time before AOS to complete prepositioning (seconds) | `30`    |

### Notes

- Comments start with `;` or `#`
- Boolean values accept: `true`/`false`, `yes`/`no`, `1`/`0`, `on`/`off`
- GPS location always overrides `[observer]` settings when a GPS receiver is connected
- Changes to the config file require a reboot to take effect
- Supported baud rates: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200

### SD Card Setup

The SD card must be formatted with a FAT filesystem (FAT16 or FAT32). The controller automatically:

1. Mounts the SD card at `/mnt/sd`
2. Loads `config.ini` if present
3. Creates a default `config.ini` if none exists

See [sdcard.md](sdcard.md) for technical details on SD card support.

## File Structure

```text
sattrack_controller/
├── controller.c      # Main application, task implementations
├── init.c            # RTEMS configuration
├── sgp4.c/h          # SGP4/SDP4 satellite propagation
├── nmea.c/h          # NMEA 0183 parser
├── config.c/h        # Configuration system
├── log.c/h           # Logging system
├── priority_queue.c/h # Pass priority queue
├── https_client.c/h  # HTTPS client for TLE download
├── date.h            # Date/time utilities (third-party)
├── wscript           # Waf build configuration
├── deploy.sh         # Deploy to TFTP server for BBB
└── docs/
    └── NMEA.md       # NMEA sentence format reference
```

## Libraries

### SGP4/SDP4

Implementation of the SGP4/SDP4 satellite propagation algorithms based on the Vallado reference implementation. Supports:

- TLE parsing with checksum validation
- Satellite position/velocity propagation
- Coordinate transforms (ECI, ECEF, Geodetic, ENU)
- Look angle calculations (azimuth, elevation, range)
- Greenwich Sidereal Time calculation

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
