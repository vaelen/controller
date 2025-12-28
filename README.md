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

### Hardware Requirements

- GPS receiver outputting NMEA 0183 at 9600 baud (8N1)
- Antenna rotator with serial control interface
- Radio with serial control interface

An emulator for these devices can be created using a Pi Pico, ESP32, or STM32 microcontroller using the [serial-device-emulator](https://github.com/vaelen/serial-device-emulator) firmware.

### BeagleBone Black Pin Assignments

The following table shows the physical P9 header pins used for each UART:

| Device  | UART  | Function | Pin   | Ball Name      | Mux Mode |
|---------|-------|----------|-------|----------------|----------|
| Console | UART0 | —        | J1    | (debug header) | —        |
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
[??:??:??] INFO  [INIT   ] Initialization complete. Tasks running.
[??:??:??] INFO  [EXEC   ] Pass executor task started
[??:??:??] INFO  [CTRL   ] Controller task started
[??:??:??] INFO  [ROTATOR] Status task started
[??:??:??] INFO  [RADIO  ] Status task started
[??:??:??] INFO  [RADIO  ] Frequency task started
[??:??:??] INFO  [ANTENNA] Location task started
[??:??:??] INFO  [GPS    ] Task started
[??:??:??] INFO  [GPS    ] Serial port /dev/ttyS1 opened successfully
[??:??:??] INFO  [PASS   ] Calculator task started
[??:??:??] INFO  [TLE    ] Updater task started
[??:??:??] INFO  [STATUS ] Status task started
[??:??:??] INFO  [CTRL   ] Antenna: az=0.0 el=0.0 deg
[??:??:??] INFO  [CTRL   ] Radio: connected
[??:??:??] INFO  [CTRL   ] Radio: VFO-A=14.074000 MHz VFO-B=7.074000 MHz
[09:29:42] INFO  [CTRL   ] GPS: lat=35.587182 deg lon=139.490050 deg alt=0.05 km
[09:29:42] INFO  [CTRL   ] GPS Time: 2025-12-18 09:29:42 UTC
[09:30:11] INFO  [STATUS ] === System Status ===
[09:30:11] INFO  [STATUS ] GPS Time: 2025-12-18 09:30:11 UTC
[09:30:11] INFO  [STATUS ] GPS Pos: 35.587182, 139.490050, 52m
[09:30:11] INFO  [STATUS ] Antenna: az=0.0 el=0.0 deg
[09:30:11] INFO  [STATUS ] Radio: VFO-A active, mode=???, preamp=IPO
[09:30:11] INFO  [STATUS ] Radio: VFO-A=14.074000 MHz, VFO-B=7.074000 MHz
[09:30:11] INFO  [STATUS ] TLE: 0 satellites loaded
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
latitude = 35.587182
longitude = 139.490050
altitude = 52.0

[files]
tle_path = /mnt/sd/tle.txt

[system]
log_level = INFO
status_interval = 30

[network]
interface = cpsw0
use_dhcp = true
ip_address = 192.168.1.100
netmask = 255.255.255.0
gateway = 192.168.1.1
```

### Configuration Sections

| Section      | Key                      | Description                                | Default           |
|--------------|--------------------------|------------------------------------------- |-------------------|
| `[serial]`   | `gps_device`             | GPS serial port device path                | `/dev/ttyS1`      |
|              | `gps_baud`               | GPS baud rate                              | `9600`            |
|              | `gps_flow_control`       | Enable hardware flow control               | `false`           |
|              | `rotator_device`         | Rotator serial port device path            | `/dev/ttyS2`      |
|              | `rotator_baud`           | Rotator baud rate                          | `9600`            |
|              | `rotator_flow_control`   | Enable hardware flow control               | `false`           |
|              | `radio_device`           | Radio serial port device path              | `/dev/ttyS3`      |
|              | `radio_baud`             | Radio baud rate                            | `38400`           |
|              | `radio_flow_control`     | Enable hardware flow control               | `false`           |
| `[observer]` | `latitude`               | Observer latitude in degrees (N positive)  | `0.0`             |
|              | `longitude`              | Observer longitude in degrees (E positive) | `0.0`             |
|              | `altitude`               | Observer altitude in meters                | `0.0`             |
| `[files]`    | `tle_path`               | Path to TLE database file                  | `/mnt/sd/tle.txt` |
| `[system]`   | `log_level`              | Logging level: DEBUG, INFO, WARN, ERROR    | `INFO`            |
|              | `status_interval`        | Status output interval in seconds          | `30`              |
| `[network]`  | `interface`              | Network interface name                     | `cpsw0`           |
|              | `use_dhcp`               | Use DHCP for network configuration         | `true`            |
|              | `ip_address`             | Static IP address (if DHCP disabled)       | `192.168.1.100`   |
|              | `netmask`                | Subnet mask (if DHCP disabled)             | `255.255.255.0`   |
|              | `gateway`                | Default gateway (if DHCP disabled)         | `192.168.1.1`     |

### Notes

- Comments start with `;` or `#`
- Boolean values accept: `true`/`false`, `yes`/`no`, `1`/`0`, `on`/`off`
- GPS location always overrides `[observer]` settings when a GPS receiver is connected
- Changes to the config file require a reboot to take effect
- Supported baud rates: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
- Static IP settings (`ip_address`, `netmask`, `gateway`) are only used when `use_dhcp` is `false`

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
├── date.h            # Date/time utilities (third-party)
├── wscript           # Waf build configuration
├── deploy.sh         # Deploy to TFTP server for BBB
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
