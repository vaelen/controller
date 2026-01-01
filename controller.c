/*
 * Satellite Tracking Controller for RTEMS
 *
 * This controller coordinates satellite tracking operations using multiple
 * RTEMS tasks communicating via message queues and protected by semaphores.
 *
 * Tasks:
 *   - GPS Task: Receives GPS data (location + time) from UART
 *   - Antenna Location Task: Polls antenna rotator position periodically
 *   - TLE Updater Task: Updates TLE database periodically
 *   - Pass Calculator Task: Calculates upcoming satellite passes
 *   - Pass Executor Task: Controls rotator and radio during satellite passes
 *   - Controller Task: Coordinates passes and commands antenna
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <bsp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "sgp4.h"
#include "nmea.h"
#include "log.h"
#include "config.h"
#include "https_client.h"

#include <sys/stat.h>
#include <rtems/dosfs.h>
#include <rtems/libio.h>

/* libbsd for SD card support */
#include <machine/rtems-bsd-commands.h>
#include <rtems/bsd/bsd.h>
#include <rtems/bdbuf.h>
#include <rtems/media.h>
#include <rtems/rtemsmmcsd.h>
#include <rtems/bdpart.h>

/* Networking support */
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <rtems/dhcpcd.h>

// ============================================================================
// Configuration Constants
// ============================================================================

#define MAX_SATELLITES CONFIG_MAX_NORAD_IDS

// TLE file paths
#define TLE_FILE_PATH       "/mnt/sd/sats.tle"
#define TLE_TEMP_FILE_PATH  "/mnt/sd/sats.tle.tmp"

// ============================================================================
// Message Types
// ============================================================================

typedef enum message_type {
    MSG_GPS_UPDATE,
    MSG_TLE_DATABASE_UPDATED,
    MSG_PASS_CALCULATED,
    MSG_ANTENNA_POSITION,
    MSG_COMMAND_ANTENNA,
    MSG_RADIO_STATUS,
    MSG_CONFIG_RELOAD
} message_type_t;

// Radio operating mode enum (Yaesu FT-991A CAT protocol)
typedef enum radio_mode {
    RADIO_MODE_LSB      = 0x1,
    RADIO_MODE_USB      = 0x2,
    RADIO_MODE_CW       = 0x3,
    RADIO_MODE_FM       = 0x4,
    RADIO_MODE_AM       = 0x5,
    RADIO_MODE_RTTY_LSB = 0x6,
    RADIO_MODE_CW_R     = 0x7,
    RADIO_MODE_DATA_LSB = 0x8,
    RADIO_MODE_RTTY_USB = 0x9,
    RADIO_MODE_DATA_FM  = 0xA,
    RADIO_MODE_FM_N     = 0xB,
    RADIO_MODE_DATA_USB = 0xC,
    RADIO_MODE_AM_N     = 0xD,
    RADIO_MODE_C4FM     = 0xE
} radio_mode_t;

// Radio pre-amp setting
typedef enum radio_preamp {
    RADIO_PREAMP_IPO  = 0,
    RADIO_PREAMP_AMP1 = 1,
    RADIO_PREAMP_AMP2 = 2
} radio_preamp_t;

// Bitmask for which fields are valid in a radio status message
#define RADIO_FIELD_VFO_A_FREQ  (1 << 0)
#define RADIO_FIELD_VFO_B_FREQ  (1 << 1)
#define RADIO_FIELD_AF_GAIN     (1 << 2)
#define RADIO_FIELD_RF_GAIN     (1 << 3)
#define RADIO_FIELD_MIC_GAIN    (1 << 4)
#define RADIO_FIELD_MODE        (1 << 5)
#define RADIO_FIELD_PREAMP      (1 << 6)
#define RADIO_FIELD_ACTIVE_VFO  (1 << 7)

// Radio status message (partial updates supported via valid_fields bitmask)
typedef struct radio_status_message {
    message_type_t type;
    uint8_t valid_fields;     // Bitmask of RADIO_FIELD_* indicating which fields are valid
    uint64_t vfo_a_freq_hz;
    uint64_t vfo_b_freq_hz;
    uint8_t af_gain;          // 0-255
    uint8_t rf_gain;          // 0-255
    uint8_t mic_gain;         // 0-100
    radio_mode_t mode;
    radio_preamp_t preamp;
    uint8_t active_vfo;       // 0=VFO-A, 1=VFO-B
} radio_status_message_t;

// Pass information is now defined in priority_queue.h
// (pass_info_t no longer includes plan steps - those are generated on-the-fly by executor)
#include "priority_queue.h"

// GPS position and time update message
typedef struct gps_message {
    message_type_t type;
    sgp4_geodetic_t location;
    time_t utc_time;
} gps_message_t;

// Antenna current position message
typedef struct antenna_position_message {
    message_type_t type;
    double azimuth;    // radians
    double elevation;  // radians
} antenna_position_message_t;

// Antenna command message
typedef struct antenna_command_message {
    message_type_t type;
    double target_azimuth;    // radians
    double target_elevation;  // radians
} antenna_command_message_t;

// Pass notification message
typedef struct pass_message {
    message_type_t type;
    pass_info_t pass;
} pass_message_t;

// TLE update notification (simple flag)
typedef struct tle_update_message {
    message_type_t type;
} tle_update_message_t;

// Union of all message types for queue sizing
typedef union controller_message {
    message_type_t type;
    gps_message_t gps;
    antenna_position_message_t antenna_pos;
    antenna_command_message_t antenna_cmd;
    pass_message_t pass;
    tle_update_message_t tle_update;
} controller_message_t;

// Controller command message (for config reload, etc.)
typedef struct ctrl_cmd_message {
    message_type_t type;
} ctrl_cmd_message_t;

// ============================================================================
// Pass Executor Command Types
// ============================================================================

// Commands sent from controller to pass executor
typedef enum executor_command_type {
    EXEC_CMD_START_PASS,       // Begin tracking a satellite pass
    EXEC_CMD_ABORT_PASS        // Abort the current pass
} executor_command_type_t;

// Message from controller to pass executor
typedef struct executor_command_message {
    executor_command_type_t command;
    pass_info_t pass;          // Only valid for EXEC_CMD_START_PASS
} executor_command_message_t;

// ============================================================================
// Pass Scheduling State
// ============================================================================

// Currently scheduled pass (waiting to execute or in-progress)
typedef struct scheduled_pass {
    bool active;               // True if a pass is scheduled
    pass_info_t pass;          // The scheduled pass
    bool prep_sent;            // True if start command sent to executor
} scheduled_pass_t;

// ============================================================================
// Satellite entry for TLE database
// ============================================================================

typedef struct satellite_entry {
    bool valid;
    sgp4_tle_t tle;
    sgp4_state_t state;
} satellite_entry_t;

// ============================================================================
// Shared State (protected by mutexes)
// ============================================================================

typedef struct controller_state {
    // Observer location from GPS
    sgp4_geodetic_t observer_location;
    bool location_valid;

    // Current time from GPS
    time_t current_time;
    bool time_valid;

    // TLE database (fixed-size array instead of std::map)
    satellite_entry_t satellites[MAX_SATELLITES];
    int satellite_count;

    // Current antenna position
    double antenna_azimuth;    // radians
    double antenna_elevation;  // radians
    bool antenna_position_valid;

    // Radio state
    uint64_t radio_vfo_a_freq_hz;
    uint64_t radio_vfo_b_freq_hz;
    uint8_t radio_af_gain;
    uint8_t radio_rf_gain;
    uint8_t radio_mic_gain;
    radio_mode_t radio_mode;
    radio_preamp_t radio_preamp;
    uint8_t radio_active_vfo;
    bool radio_status_valid;
} controller_state_t;

static controller_state_t g_state;

// ============================================================================
// IPC Object IDs
// ============================================================================

// Message queues
static rtems_id g_gps_queue;
static rtems_id g_antenna_queue;
static rtems_id g_tle_queue;
static rtems_id g_pass_queue;

// Semaphores (mutexes)
static rtems_id g_uart1_mutex;
static rtems_id g_tle_database_mutex;
static rtems_id g_state_mutex;
static rtems_id g_pass_queue_mutex;

// Task IDs
static rtems_id g_gps_task_id;
static rtems_id g_antenna_task_id;
static rtems_id g_tle_task_id;
static rtems_id g_pass_task_id;
static rtems_id g_pass_executor_task_id;
static rtems_id g_controller_task_id;
static rtems_id g_rotator_status_task_id;

// Rotator file descriptor (shared by antenna_location_task and rotator_status_task)
static int g_rotator_fd = -1;

// Radio IPC objects
static rtems_id g_radio_queue;
static rtems_id g_uart3_mutex;
static rtems_id g_radio_status_task_id;
static rtems_id g_radio_freq_task_id;
static int g_radio_fd = -1;

// Controller command queue (for config reload, etc.)
static rtems_id g_ctrl_cmd_queue;

// Pass executor command queue (controller -> executor)
static rtems_id g_executor_cmd_queue;

// Pass priority queue and scheduling state
static pass_priority_queue_t g_upcoming_passes;
static scheduled_pass_t g_scheduled_pass;

// Status task
static rtems_id g_status_task_id;

// ============================================================================
// Task Priorities and Stack Sizes
// ============================================================================

#define PRIORITY_PASS_EXECUTOR  10
#define PRIORITY_CONTROLLER     20
#define PRIORITY_ROTATOR_STATUS 30
#define PRIORITY_RADIO_STATUS   35
#define PRIORITY_RADIO_FREQ     36
#define PRIORITY_ANTENNA        40
#define PRIORITY_GPS            50
#define PRIORITY_PASS           60
#define PRIORITY_TLE            70
#define PRIORITY_STATUS         80

#define TASK_STACK_SIZE     (8 * 1024)
#define TLE_TASK_STACK_SIZE (32 * 1024)  /* OpenSSL needs more stack */

// ============================================================================
// Serial Port Initialization
// ============================================================================

/*
 * Initialize a serial port with the specified settings.
 *
 * @param path          Device path (e.g., "/dev/ttyS0")
 * @param baud          Baud rate (e.g., B9600, B115200)
 * @param flags         Open flags (O_RDONLY, O_WRONLY, or O_RDWR)
 * @param flow_control  Enable CTS/RTS hardware flow control
 * @return              File descriptor on success, -1 on failure
 */
static int serial_init(const char *path, speed_t baud, int flags, bool flow_control) {
    int fd = open(path, flags | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        LOG_ERROR("SERIAL", "Failed to open serial port %s, Error: %s", path, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        LOG_ERROR("SERIAL", "Failed to get attributes for serial port %s, Error: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    /* Set baud rate */
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    /* 8N1 mode */
    tty.c_cflag &= ~PARENB;        /* No parity */
    tty.c_cflag &= ~CSTOPB;        /* 1 stop bit */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            /* 8 data bits */

    /* Hardware flow control */
    if (flow_control) {
        tty.c_cflag |= CRTSCTS;
    } else {
        tty.c_cflag &= ~CRTSCTS;
    }

    /* Ignore modem control lines */
    tty.c_cflag |= CLOCAL;

    /* Enable receiver if reading (O_RDONLY is 0, so check access mode) */
    int access_mode = flags & O_ACCMODE;
    if (access_mode == O_RDONLY || access_mode == O_RDWR) {
        tty.c_cflag |= CREAD;

        /* Raw input mode */
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        /* Disable input processing that could alter data */
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* No software flow control */
        tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);  /* Don't translate CR/LF */
        tty.c_iflag &= ~(ISTRIP | BRKINT);        /* Don't strip 8th bit, ignore break */

        /* Non-blocking: return immediately even with no data */
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
    }

    /* Raw output mode */
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR("SERIAL", "Failed to set attributes for serial port %s, Error: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Initialize a serial port from configuration.
 * Returns file descriptor on success, -1 on failure.
 */
static int serial_init_from_config(const serial_config_t *cfg) {
    return serial_init(cfg->device_path, cfg->baud_rate,
                       O_RDWR, cfg->flow_control);
}

#include <dirent.h>

/*
 * List contents of a directory for debugging purposes.
 */
static void list_directory(const char *path) {
    DIR *dir = opendir(path);
    if (dir == NULL) {
        LOG_INFO("INIT", "Cannot open %s: %s", path, strerror(errno));
        return;
    }

    LOG_INFO("INIT", "Contents of %s:", path);
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;  /* Skip . and .. */
        LOG_INFO("INIT", "  %s", entry->d_name);
    }
    closedir(dir);
}

/*
 * Initialize libbsd and wait for SD card device.
 * Returns true on success, false on failure.
 */
static bool libbsd_init(void) {
    rtems_status_code sc;

    LOG_INFO("INIT", "Initializing block device buffer...");

    /* Initialize block device buffer - required for disk I/O */
    sc = rtems_bdbuf_init();
    if (sc != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to initialize bdbuf: %d", sc);
        return false;
    }

    LOG_INFO("INIT", "Initializing media manager...");

    /* Initialize media manager - handles partition discovery */
    sc = rtems_media_initialize();
    if (sc != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to initialize media: %d", sc);
        return false;
    }

    /* Start media server for automatic partition handling */
    sc = rtems_media_server_initialize(
        200,                      /* Priority */
        32 * 1024,               /* Stack size */
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES
    );
    if (sc != RTEMS_SUCCESSFUL) {
        LOG_WARN("INIT", "Failed to start media server: %d (continuing anyway)", sc);
    }

    /*
     * Configure mmcsd driver to use media server for device attach.
     * This enables automatic partition discovery (creates /dev/mmcsd0s1, etc.)
     * Must be called BEFORE rtems_bsd_initialize().
     */
    LOG_INFO("INIT", "Configuring mmcsd for media server attach...");
    rtems_mmcsd_use_media_server();

    LOG_INFO("INIT", "Initializing libbsd...");

    /* Initialize the libbsd library */
    sc = rtems_bsd_initialize();
    if (sc != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to initialize libbsd: %d", sc);
        return false;
    }

    /*
     * Wait for the SD card device to appear (up to 5 seconds).
     * The device name depends on how the driver attaches:
     * - Static attach: /dev/mmcsd0, /dev/mmcsd0s1
     * - Media server: /dev/mmcsd-0, /dev/mmcsd-0s1
     *
     * The media server processes disk attach asynchronously, then
     * tries to mount. If mount fails, it runs partition inquiry.
     * We need to wait for partition devices to appear.
     */
    const char *raw_devices[] = {
        "/dev/mmcsd0",     /* Static attach */
        "/dev/mmcsd-0",    /* Media server */
    };
    const char *partition_devices[] = {
        "/dev/mmcsd01",    /* Static attach, partition 1 */
        "/dev/mmcsd-01",   /* Media server, partition 1 (bdpart naming) */
        "/dev/mmcsd0s1",   /* Static attach, slice 1 */
        "/dev/mmcsd-0s1",  /* Media server, slice 1 */
    };
    int timeout_ms = 5000;
    int elapsed = 0;
    bool raw_found = false;

    LOG_INFO("INIT", "Waiting for SD card device...");

    /* First, wait for the raw device to appear */
    while (elapsed < timeout_ms && !raw_found) {
        for (int i = 0; i < 2; i++) {
            struct stat st;
            if (stat(raw_devices[i], &st) == 0) {
                LOG_INFO("INIT", "Raw SD card device found: %s", raw_devices[i]);
                raw_found = true;
                break;
            }
        }
        if (!raw_found) {
            rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
            elapsed += 100;
        }
    }

    if (!raw_found) {
        LOG_WARN("INIT", "SD card device not found after %d ms", timeout_ms);
        list_directory("/dev");
        return false;
    }

    /*
     * Raw device found. Try to register partitions manually.
     * The media server may not have completed partition discovery,
     * so we call rtems_bdpart_register_from_disk directly.
     */
    const char *disk_path = NULL;
    for (int i = 0; i < 2; i++) {
        struct stat st;
        if (stat(raw_devices[i], &st) == 0) {
            disk_path = raw_devices[i];
            break;
        }
    }

    if (disk_path != NULL) {
        LOG_INFO("INIT", "Registering partitions from %s...", disk_path);
        rtems_status_code psc = rtems_bdpart_register_from_disk(disk_path);
        if (psc == RTEMS_SUCCESSFUL) {
            LOG_INFO("INIT", "Partition registration successful");
        } else {
            LOG_WARN("INIT", "Partition registration failed: %d", psc);
        }
    }

    /* Wait briefly for partition devices to appear */
    LOG_INFO("INIT", "Waiting for partition devices...");
    const int num_partition_devs = sizeof(partition_devices) / sizeof(partition_devices[0]);
    int partition_timeout = 1000;
    elapsed = 0;

    while (elapsed < partition_timeout) {
        for (int i = 0; i < num_partition_devs; i++) {
            struct stat st;
            if (stat(partition_devices[i], &st) == 0) {
                LOG_INFO("INIT", "Partition device found: %s", partition_devices[i]);
                return true;
            }
        }
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
        elapsed += 100;
    }

    /* No partition found - maybe it's a superfloppy format (no partition table) */
    LOG_INFO("INIT", "No partition device found, will try raw device");
    list_directory("/dev");
    return true;  /* Return true anyway, mount_sd_card will try both */
}

/*
 * Mount the SD card filesystem.
 * Creates mount point and mounts FAT filesystem.
 * Returns true on success, false on failure.
 */
static bool mount_sd_card(void) {
    int rv;

    /* Create mount point directories */
    rv = mkdir("/mnt", 0755);
    if (rv != 0 && errno != EEXIST) {
        LOG_ERROR("INIT", "Failed to create /mnt: %s", strerror(errno));
        return false;
    }

    rv = mkdir(CONFIG_SD_MOUNT, 0755);
    if (rv != 0 && errno != EEXIST) {
        LOG_ERROR("INIT", "Failed to create %s: %s", CONFIG_SD_MOUNT, strerror(errno));
        return false;
    }

    /*
     * Try to mount in order of preference:
     * - Partitions first (most common for SD cards with partition table)
     * - Raw device last (for superfloppy format without partition table)
     * Naming depends on attach mode (static vs media server)
     */
    const char *devices[] = {
        "/dev/mmcsd01",    /* Static attach, partition 1 (bdpart) */
        "/dev/mmcsd-01",   /* Media server, partition 1 (bdpart) */
        "/dev/mmcsd0s1",   /* Static attach, slice 1 */
        "/dev/mmcsd-0s1",  /* Media server, slice 1 */
        "/dev/mmcsd0",     /* Static attach, raw */
        "/dev/mmcsd-0",    /* Media server, raw */
    };
    const int num_devices = sizeof(devices) / sizeof(devices[0]);

    for (int i = 0; i < num_devices; i++) {
        struct stat st;
        if (stat(devices[i], &st) != 0) {
            LOG_DEBUG("INIT", "Device %s not found", devices[i]);
            continue;
        }

        LOG_DEBUG("INIT", "Trying to mount %s...", devices[i]);

        rv = mount(
            devices[i],
            CONFIG_SD_MOUNT,
            RTEMS_FILESYSTEM_TYPE_DOSFS,
            RTEMS_FILESYSTEM_READ_WRITE,
            NULL
        );

        if (rv == 0) {
            LOG_INFO("INIT", "SD card mounted at %s (device: %s)",
                     CONFIG_SD_MOUNT, devices[i]);
            return true;
        }

        LOG_DEBUG("INIT", "Mount %s failed: %s", devices[i], strerror(errno));
    }

    LOG_WARN("INIT", "Failed to mount SD card: %s", strerror(errno));
    return false;
}

// ============================================================================
// Network Initialization
// ============================================================================

/*
 * Wait for network interface to appear.
 * Returns true if interface is found within timeout.
 */
static bool network_wait_for_interface(const char *ifname, int timeout_sec)
{
    for (int i = 0; i < timeout_sec; i++) {
        struct ifaddrs *ifap;
        if (getifaddrs(&ifap) == 0) {
            for (struct ifaddrs *ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
                if (strcmp(ifa->ifa_name, ifname) == 0) {
                    freeifaddrs(ifap);
                    LOG_INFO("NET", "Interface %s detected", ifname);
                    return true;
                }
            }
            freeifaddrs(ifap);
        }
        rtems_task_wake_after(rtems_clock_get_ticks_per_second());
    }
    return false;
}

/*
 * Configure static IPv4 address.
 */
static void network_configure_ipv4_static(const network_config_t *cfg)
{
    char *ifcfg_argv[] = {
        "ifconfig",
        (char *)cfg->interface,
        "inet",
        (char *)cfg->ipv4.address,
        "netmask",
        (char *)cfg->ipv4.netmask,
        NULL
    };

    int exit_code = rtems_bsd_command_ifconfig(6, ifcfg_argv);
    if (exit_code != 0) {
        LOG_ERROR("NET", "Failed to configure IPv4 address: %d", exit_code);
        return;
    }

    LOG_INFO("NET", "IPv4: %s/%s", cfg->ipv4.address, cfg->ipv4.netmask);

    /* Add default gateway if specified */
    if (cfg->ipv4.gateway[0] != '\0') {
        char *route_gw_argv[] = {
            "route",
            "add",
            "-host",
            (char *)cfg->ipv4.gateway,
            "-iface",
            (char *)cfg->interface,
            NULL
        };
        char *route_default_argv[] = {
            "route",
            "add",
            "default",
            (char *)cfg->ipv4.gateway,
            NULL
        };

        rtems_bsd_command_route(6, route_gw_argv);
        rtems_bsd_command_route(4, route_default_argv);
        LOG_INFO("NET", "IPv4 gateway: %s", cfg->ipv4.gateway);
    }
}

/*
 * Configure static IPv6 address.
 */
static void network_configure_ipv6_static(const network_config_t *cfg)
{
    char prefix_str[8];
    snprintf(prefix_str, sizeof(prefix_str), "%d", cfg->ipv6.prefix_len);

    char *ifcfg_argv[] = {
        "ifconfig",
        (char *)cfg->interface,
        "inet6",
        (char *)cfg->ipv6.address,
        "prefixlen",
        prefix_str,
        NULL
    };

    int exit_code = rtems_bsd_command_ifconfig(6, ifcfg_argv);
    if (exit_code != 0) {
        LOG_ERROR("NET", "Failed to configure IPv6 address: %d", exit_code);
        return;
    }

    LOG_INFO("NET", "IPv6: %s/%d", cfg->ipv6.address, cfg->ipv6.prefix_len);

    /* Add default gateway if specified */
    if (cfg->ipv6.gateway[0] != '\0') {
        char *route_default_argv[] = {
            "route",
            "add",
            "-inet6",
            "default",
            (char *)cfg->ipv6.gateway,
            NULL
        };

        rtems_bsd_command_route(5, route_default_argv);
        LOG_INFO("NET", "IPv6 gateway: %s", cfg->ipv6.gateway);
    }
}

/*
 * Create /etc/resolv.conf with fallback DNS servers.
 * dhcpcd may override this with DHCP-provided servers, but if DNS
 * information isn't provided by DHCP, these fallbacks will be used.
 */
static void network_create_resolv_conf(void)
{
    /* Create /etc directory if it doesn't exist */
    mkdir("/etc", 0755);

    int fd = open("/etc/resolv.conf", O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd < 0) {
        LOG_ERROR("NET", "Cannot create /etc/resolv.conf: %s", strerror(errno));
        return;
    }

    /* Write fallback DNS servers */
    const char *resolv_conf =
        "# Fallback DNS servers for satellite tracking controller\n"
        "# These may be overwritten by dhcpcd if DHCP provides DNS servers\n"
        "nameserver 1.1.1.1\n"
        "nameserver 8.8.8.8\n"
        "nameserver 2606:4700:4700::1111\n"
        "nameserver 2001:4860:4860::8888\n";

    write(fd, resolv_conf, strlen(resolv_conf));
    close(fd);

    LOG_DEBUG("NET", "Created /etc/resolv.conf with fallback DNS servers");
}

/*
 * Start dhcpcd daemon for DHCP/SLAAC.
 */
static void network_start_dhcpcd(const network_config_t *cfg)
{
    rtems_status_code sc;

    /* Create /etc directory if it doesn't exist */
    mkdir("/etc", 0755);

    /* Create dhcpcd configuration file */
    int fd = open("/etc/dhcpcd.conf", O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd < 0) {
        LOG_ERROR("NET", "Cannot create /etc/dhcpcd.conf: %s", strerror(errno));
        return;
    }

    /* Write configuration based on settings */
    const char *header = "# Auto-generated dhcpcd configuration\n";
    write(fd, header, strlen(header));

    /* Client ID for DHCP */
    const char *clientid = "clientid sattrack-controller\n";
    write(fd, clientid, strlen(clientid));

    /* Interface-specific configuration */
    char iface_line[64];
    snprintf(iface_line, sizeof(iface_line), "interface %s\n", cfg->interface);
    write(fd, iface_line, strlen(iface_line));

    /* Control IPv4 DHCP */
    if (!cfg->ipv4.enabled || !cfg->ipv4.dhcp) {
        const char *nodhcp = "nodhcp\n";
        write(fd, nodhcp, strlen(nodhcp));
    }

    /* Control IPv6 DHCPv6 */
    if (!cfg->ipv6.enabled || !cfg->ipv6.dhcp) {
        const char *nodhcp6 = "nodhcp6\n";
        write(fd, nodhcp6, strlen(nodhcp6));
    }

    /* Control SLAAC - dhcpcd handles SLAAC by default when IPv6 is enabled */
    if (!cfg->ipv6.enabled || !cfg->ipv6.slaac) {
        const char *noipv6rs = "noipv6rs\n";
        write(fd, noipv6rs, strlen(noipv6rs));
    }

    close(fd);

    /* Start dhcpcd */
    LOG_INFO("NET", "Starting dhcpcd...");
    sc = rtems_dhcpcd_start(NULL);
    if (sc != RTEMS_SUCCESSFUL) {
        LOG_ERROR("NET", "Failed to start dhcpcd: %d", sc);
    }
}

/*
 * Get current network interface addresses for status display.
 * Populates ipv4_addr and ipv6_addr buffers with current addresses.
 */
static void network_get_addresses(const char *ifname,
                                   char *ipv4_addr, size_t ipv4_len,
                                   char *ipv6_addr, size_t ipv6_len)
{
    struct ifaddrs *ifap, *ifa;

    ipv4_addr[0] = '\0';
    ipv6_addr[0] = '\0';

    if (getifaddrs(&ifap) != 0) {
        return;
    }

    for (ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
        if (strcmp(ifa->ifa_name, ifname) != 0) {
            continue;
        }

        if (ifa->ifa_addr == NULL) {
            continue;
        }

        if (ifa->ifa_addr->sa_family == AF_INET && ipv4_addr[0] == '\0') {
            struct sockaddr_in *sin = (struct sockaddr_in *)ifa->ifa_addr;
            inet_ntop(AF_INET, &sin->sin_addr, ipv4_addr, ipv4_len);
        }
        else if (ifa->ifa_addr->sa_family == AF_INET6 && ipv6_addr[0] == '\0') {
            struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)ifa->ifa_addr;
            /* Skip link-local addresses for display */
            if (!IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr)) {
                inet_ntop(AF_INET6, &sin6->sin6_addr, ipv6_addr, ipv6_len);
            }
        }
    }

    freeifaddrs(ifap);
}

/*
 * Initialize networking based on configuration.
 * Must be called after rtems_bsd_initialize() and before tasks start.
 * Returns true on success, false on failure.
 */
static bool network_init(const network_config_t *cfg)
{
    int exit_code;

    if (!cfg->enabled) {
        LOG_INFO("NET", "Networking disabled in configuration");
        return true;
    }

    LOG_INFO("NET", "Initializing network interface %s...", cfg->interface);

    /* Create resolv.conf with fallback DNS servers */
    network_create_resolv_conf();

    /* Configure loopback interface first */
    exit_code = rtems_bsd_ifconfig_lo0();
    if (exit_code != 0) {
        LOG_WARN("NET", "Failed to configure loopback interface: %d", exit_code);
    }

    /* Wait for the Ethernet interface to appear */
    if (!network_wait_for_interface(cfg->interface, 10)) {
        LOG_ERROR("NET", "Network interface %s not found", cfg->interface);
        return false;
    }

    /* Bring up the interface */
    char *ifup_argv[] = {
        "ifconfig",
        (char *)cfg->interface,
        "up",
        NULL
    };
    rtems_bsd_command_ifconfig(3, ifup_argv);

    /* Configure IPv4 */
    if (cfg->ipv4.enabled) {
        if (cfg->ipv4.dhcp) {
            LOG_INFO("NET", "IPv4: Using DHCP");
        } else if (cfg->ipv4.address[0] != '\0') {
            network_configure_ipv4_static(cfg);
        }
    }

    /* Configure IPv6 */
    if (cfg->ipv6.enabled) {
        if (cfg->ipv6.slaac || cfg->ipv6.dhcp) {
            LOG_INFO("NET", "IPv6: Using %s%s%s",
                     cfg->ipv6.slaac ? "SLAAC" : "",
                     (cfg->ipv6.slaac && cfg->ipv6.dhcp) ? "+" : "",
                     cfg->ipv6.dhcp ? "DHCPv6" : "");
        } else if (cfg->ipv6.address[0] != '\0') {
            network_configure_ipv6_static(cfg);
        }
    }

    /* Start dhcpcd if DHCP is enabled for either protocol */
    if ((cfg->ipv4.enabled && cfg->ipv4.dhcp) ||
        (cfg->ipv6.enabled && (cfg->ipv6.dhcp || cfg->ipv6.slaac))) {
        network_start_dhcpcd(cfg);
    }

    return true;
}

// ============================================================================
// Task Entry Points
// ============================================================================

/*
 * GPS Task
 *
 * Receives data from a GPS receiver via UART.
 * Parses NMEA sentences (GGA for position, RMC for date/time).
 * Sends updates to the controller via message queue.
 */
rtems_task gps_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("GPS", "Task started");

    /* Get current configuration */
    config_t cfg;
    config_get_copy(&cfg);

    int fd = serial_init_from_config(&cfg.gps);
    if (fd < 0) {
        LOG_ERROR("GPS", "Failed to open GPS port %s", cfg.gps.device_path);
        rtems_task_exit();
    }

    LOG_INFO("GPS", "Serial port %s opened successfully", cfg.gps.device_path);

    nmea_buffer_t nmea_buf;
    nmea_buffer_init(&nmea_buf);

    /* Track latest valid data from each sentence type */
    nmea_gga_t last_gga = {0};
    nmea_rmc_t last_rmc = {0};

    char read_buf[64];
    char line[NMEA_BUFFER_SIZE];

    while (true) {
        /* Non-blocking read from serial */
        ssize_t n;
        n = read(fd, read_buf, sizeof(read_buf) - 1);
        while (n > 0) {
            read_buf[n] = '\0';
            nmea_buffer_add(&nmea_buf, read_buf);
            n = read(fd, read_buf, sizeof(read_buf) - 1);
        }

        /* Process complete lines from buffer */
        while (nmea_buffer_get_line(&nmea_buf, line, sizeof(line))) {
            /* Strip trailing newline/carriage return */
            while(line[strlen(line)-1] == '\n' || line[strlen(line)-1] == '\r') {
                line[strlen(line)-1] = '\0';
            }
            // LOG_DEBUG("GPS", "NMEA: %s", line);

            /* Validate checksum before parsing */
            if (!nmea_validate_checksum(line)) {
                continue;
            }

            if (nmea_is_gga(line)) {
                LOG_DEBUG("GPS", "NMEA GGA: %s", line);
                nmea_parse_gga(line, &last_gga);
            } else if (nmea_is_rmc(line)) {
                LOG_DEBUG("GPS", "NMEA RMC: %s", line);
                nmea_parse_rmc(line, &last_rmc);
            }

            /* Send message if we have valid position and date/time */
            if (last_gga.valid && last_gga.fix_quality > 0 &&
                last_rmc.valid && last_rmc.date.valid) {

                gps_message_t msg;
                msg.type = MSG_GPS_UPDATE;

                /* Convert degrees to radians, meters to km */
                msg.location.lat_rad = last_gga.lat_deg * SGP4_DEG_TO_RAD;
                msg.location.lon_rad = last_gga.lon_deg * SGP4_DEG_TO_RAD;
                msg.location.alt_km = last_gga.alt_m / 1000.0;

                /* Build time_t from RMC date + GGA time (UTC) */
                struct tm tm_time;
                memset(&tm_time, 0, sizeof(tm_time));
                tm_time.tm_year = last_rmc.date.year - 1900;
                tm_time.tm_mon = last_rmc.date.month - 1;
                tm_time.tm_mday = last_rmc.date.day;
                tm_time.tm_hour = last_gga.time.hour;
                tm_time.tm_min = last_gga.time.minute;
                tm_time.tm_sec = (int)last_gga.time.second;
                tm_time.tm_isdst = 0;

                /* Use mktime (assumes UTC in RTEMS without timezone config) */
                msg.utc_time = mktime(&tm_time);

                rtems_message_queue_send(g_gps_queue, &msg, sizeof(msg));
            }
        }

        /* Yield to other tasks (poll at ~10 Hz) */
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
    }
}

/*
 * Antenna Location Task
 *
 * Sends periodic position query commands to the antenna rotator.
 * Uses GS-232A protocol: "C2\r" requests azimuth and elevation.
 * The rotator_status_task handles parsing responses.
 */
rtems_task antenna_location_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("ANTENNA", "Location task started");

    while (true) {

        // Acquire mutex before UART access
        rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        // Send C2 query command to rotator
        if (g_rotator_fd >= 0) {
            const char *cmd = "C2\r";
            write(g_rotator_fd, cmd, 3);
            fsync(g_rotator_fd);
            LOG_DEBUG("ROTATOR", "Sent command: %s", cmd);
        }

        rtems_semaphore_release(g_uart1_mutex);

        // Poll every 2 seconds
        rtems_task_wake_after(2 * rtems_clock_get_ticks_per_second());
    }
}

/*
 * Rotator Status Task
 *
 * Listens for status responses from the rotator (GS-232A protocol).
 * Parses "+0aaa+0eee\r" format (azimuth first, then elevation).
 * Sends antenna_position_message_t to controller via message queue.
 */
rtems_task rotator_status_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("ROTATOR", "Status task started");

    char buf[32];
    int buf_pos = 0;

    while (true) {
        // Acquire mutex before UART access
        rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        // Non-blocking read from rotator
        if (g_rotator_fd >= 0) {
            char read_buf[16];
            ssize_t total_read = 0;
            ssize_t n = read(g_rotator_fd, read_buf, sizeof(read_buf) - 1);
            while (n > 0) {
                total_read += n;
                // Append to buffer
                for (ssize_t i = 0; i < n && buf_pos < (int)sizeof(buf) - 1; i++) {
                    buf[buf_pos++] = read_buf[i];
                }
                buf[buf_pos] = '\0';
                n = read(g_rotator_fd, read_buf, sizeof(read_buf) - 1);
            }
            if (total_read > 0)
                LOG_DEBUG("ROTATOR", "Read %zd bytes from rotator", total_read);
        }

        rtems_semaphore_release(g_uart1_mutex);

        if (buf_pos == 0) {
            // No data received, yield and continue
            rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
            continue;
        }

        // Check for complete response (ends with CR)
        char *cr = strchr(buf, '\r');
        if (cr != NULL) {
            *cr = '\0';  // Null-terminate at CR

            // Parse GS-232A format: +0aaa+0eee or +0aaa +0eee
            // Example: +0180+0045 or +0180 +0045 = 180 deg az, 45 deg el
            int az_deg = 0, el_deg = 0;
            if (sscanf(buf, "+0%3d +0%3d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "+0%3d+0%3d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "%d %d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "%d%d", &az_deg, &el_deg) == 2) {

                // Convert degrees to radians and send message
                antenna_position_message_t msg;
                msg.type = MSG_ANTENNA_POSITION;
                msg.azimuth = (double)az_deg * SGP4_DEG_TO_RAD;
                msg.elevation = (double)el_deg * SGP4_DEG_TO_RAD;

                rtems_message_queue_send(g_antenna_queue, &msg, sizeof(msg));
                LOG_DEBUG("ROTATOR", "Position: az=%d el=%d deg", az_deg, el_deg);
            }

            // Reset buffer (shift remaining data if any)
            int remaining = buf_pos - (int)(cr - buf) - 1;
            if (remaining > 0) {
                memmove(buf, cr + 1, remaining);
                buf_pos = remaining;
            } else {
                buf_pos = 0;
            }
            buf[buf_pos] = '\0';
        }

        // Yield to other tasks (poll at ~20 Hz)
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
    }
}

/*
 * Radio Frequency Task
 *
 * Sends periodic CAT query commands to the radio to request status.
 * Uses Yaesu FT-991A CAT protocol.
 * The radio_status_task handles parsing responses.
 */
rtems_task radio_frequency_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("RADIO", "Frequency task started");

    // CAT query commands
    static const char *queries[] = {
        "FA;",   // VFO-A frequency
        "FB;",   // VFO-B frequency
        "AG0;",  // AF gain
        "RG0;",  // RF gain
        "MG;",   // Mic gain
        "MD0;",  // Mode
        "PA0;",  // Pre-amp
        "FT;"    // Active VFO
    };
    static const int num_queries = sizeof(queries) / sizeof(queries[0]);

    while (true) {
        rtems_semaphore_obtain(g_uart3_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (g_radio_fd >= 0) {
            for (int i = 0; i < num_queries; i++) {
                write(g_radio_fd, queries[i], strlen(queries[i]));
            }
            fsync(g_radio_fd);
            LOG_DEBUG("RADIO", "Sent %d query commands", num_queries);
        }

        rtems_semaphore_release(g_uart3_mutex);

        // Poll every 2 seconds
        rtems_task_wake_after(2 * rtems_clock_get_ticks_per_second());
    }
}

/*
 * Radio Status Task
 *
 * Listens for CAT responses from the radio (Yaesu FT-991A).
 * Parses responses and sends radio_status_message_t to controller.
 * Supports partial updates via valid_fields bitmask.
 */
rtems_task radio_status_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("RADIO", "Status task started");

    char buf[128];
    int buf_pos = 0;

    while (true) {
        rtems_semaphore_obtain(g_uart3_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (g_radio_fd >= 0) {
            ssize_t total_read = 0;
            char read_buf[64];
            ssize_t n = read(g_radio_fd, read_buf, sizeof(read_buf) - 1);
            while (n > 0) {
                total_read += n;
                for (ssize_t i = 0; i < n && buf_pos < (int)sizeof(buf) - 1; i++) {
                    buf[buf_pos++] = read_buf[i];
                }
                buf[buf_pos] = '\0';
                n = read(g_radio_fd, read_buf, sizeof(read_buf) - 1);
            }
            if (total_read > 0)
                LOG_DEBUG("RADIO", "Read %zd bytes from radio", total_read);
        }

        rtems_semaphore_release(g_uart3_mutex);

        // Parse complete responses (terminated by ';')
        // Accumulate fields into a message with valid_fields bitmask
        radio_status_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.type = MSG_RADIO_STATUS;
        msg.valid_fields = 0;

        char *semi;
        while ((semi = strchr(buf, ';')) != NULL) {
            *semi = '\0';
            char *cmd = buf;

            // Parse based on command prefix and set valid_fields bit
            unsigned long freq;
            unsigned int val;
            char mode_char;

            if (sscanf(cmd, "FA%9lu", &freq) == 1) {
                msg.vfo_a_freq_hz = freq;
                msg.valid_fields |= RADIO_FIELD_VFO_A_FREQ;
            } else if (sscanf(cmd, "FB%9lu", &freq) == 1) {
                msg.vfo_b_freq_hz = freq;
                msg.valid_fields |= RADIO_FIELD_VFO_B_FREQ;
            } else if (sscanf(cmd, "AG0%3u", &val) == 1) {
                msg.af_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_AF_GAIN;
            } else if (sscanf(cmd, "RG0%3u", &val) == 1) {
                msg.rf_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_RF_GAIN;
            } else if (sscanf(cmd, "MG%3u", &val) == 1) {
                msg.mic_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_MIC_GAIN;
            } else if (sscanf(cmd, "MD0%c", &mode_char) == 1) {
                if (mode_char >= '1' && mode_char <= '9')
                    msg.mode = (radio_mode_t)(mode_char - '0');
                else if (mode_char >= 'A' && mode_char <= 'E')
                    msg.mode = (radio_mode_t)(mode_char - 'A' + 10);
                msg.valid_fields |= RADIO_FIELD_MODE;
            } else if (sscanf(cmd, "PA0%1u", &val) == 1) {
                msg.preamp = (radio_preamp_t)val;
                msg.valid_fields |= RADIO_FIELD_PREAMP;
            } else if (sscanf(cmd, "FT%1u", &val) == 1) {
                msg.active_vfo = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_ACTIVE_VFO;
            }

            // Shift buffer
            int remaining = buf_pos - (int)(semi - buf) - 1;
            if (remaining > 0) {
                memmove(buf, semi + 1, remaining);
                buf_pos = remaining;
            } else {
                buf_pos = 0;
            }
            buf[buf_pos] = '\0';
        }

        // Send message if any fields were parsed
        if (msg.valid_fields != 0) {
            rtems_message_queue_send(g_radio_queue, &msg, sizeof(msg));
            LOG_DEBUG("RADIO", "Parsed fields=0x%02X", msg.valid_fields);
        }

        // Yield to other tasks (poll at ~20 Hz)
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
    }
}

// ============================================================================
// TLE Stream Parser for Download Filtering
// ============================================================================

/*
 * Parser states for streaming TLE data.
 */
typedef enum tle_parser_state {
    TLE_STATE_NAME = 0,   /* Expecting name line (line 0) */
    TLE_STATE_LINE1 = 1,  /* Expecting TLE line 1 */
    TLE_STATE_LINE2 = 2   /* Expecting TLE line 2 */
} tle_parser_state_t;

/*
 * TLE stream parser context for filtering during download.
 */
typedef struct tle_stream_parser {
    char line_buf[256];              /* Buffer for accumulating current line */
    int line_len;                    /* Current line length */
    char name_line[SGP4_TLE_NAME_LEN]; /* Satellite name */
    char line1[SGP4_TLE_LINE_LEN];   /* TLE line 1 */
    char line2[SGP4_TLE_LINE_LEN];   /* TLE line 2 */
    tle_parser_state_t state;        /* Current parser state */
    const int *norad_ids;            /* NORAD IDs to filter (NULL = accept all) */
    int norad_count;                 /* Number of NORAD IDs in filter */
    FILE *output_file;               /* File to write filtered TLE data */
    int total_parsed;                /* Total TLE entries parsed */
    int matches_found;               /* Number of matching satellites */
    int max_matches;                 /* Maximum matches to accept */
} tle_stream_parser_t;

/*
 * Initialize TLE stream parser.
 */
static void tle_parser_init(tle_stream_parser_t *parser, const int *norad_ids,
                            int norad_count, FILE *output_file, int max_matches)
{
    memset(parser, 0, sizeof(*parser));
    parser->norad_ids = norad_ids;
    parser->norad_count = norad_count;
    parser->output_file = output_file;
    parser->max_matches = max_matches;
    parser->state = TLE_STATE_NAME;
}

/*
 * Check if NORAD ID is in the filter list.
 * Returns true if the ID should be accepted.
 */
static bool tle_parser_should_accept(const tle_stream_parser_t *parser, int norad_id)
{
    /* If no filter list, accept based on max_matches limit */
    if (parser->norad_ids == NULL || parser->norad_count == 0) {
        return parser->matches_found < parser->max_matches;
    }

    /* Check if ID is in the filter list */
    for (int i = 0; i < parser->norad_count; i++) {
        if (parser->norad_ids[i] == norad_id) {
            return true;
        }
    }
    return false;
}

/*
 * Extract NORAD ID from TLE line 1.
 * Line 1 format: "1 NNNNN..." where NNNNN is the 5-digit NORAD ID at columns 3-7.
 */
static int tle_parser_extract_norad_id(const char *line1)
{
    if (strlen(line1) < 7) {
        return -1;
    }
    if (line1[0] != '1') {
        return -1;
    }

    /* Extract NORAD ID from columns 3-7 (0-indexed: 2-6) */
    char id_str[6];
    memcpy(id_str, &line1[2], 5);
    id_str[5] = '\0';

    return atoi(id_str);
}

/*
 * Process a complete TLE entry (name, line1, line2).
 * Returns true if the entry was accepted and written.
 */
static bool tle_parser_process_entry(tle_stream_parser_t *parser)
{
    parser->total_parsed++;

    /* Extract NORAD ID from line 1 */
    int norad_id = tle_parser_extract_norad_id(parser->line1);
    if (norad_id < 0) {
        LOG_DEBUG("TLE", "Invalid TLE line 1: %s", parser->line1);
        return false;
    }

    /* Check if we should accept this satellite */
    if (!tle_parser_should_accept(parser, norad_id)) {
        return false;
    }

    /* Write to output file */
    if (parser->output_file != NULL) {
        fprintf(parser->output_file, "%s\n%s\n%s\n",
                parser->name_line, parser->line1, parser->line2);
    }

    parser->matches_found++;
    LOG_DEBUG("TLE", "Accepted satellite: %s (NORAD %d), %d/%d",
              parser->name_line, norad_id, parser->matches_found, parser->max_matches);

    return true;
}

/*
 * Process a single line from the TLE stream.
 * Called for each complete line received.
 */
static void tle_parser_process_line(tle_stream_parser_t *parser, const char *line)
{
    /* Skip empty lines */
    size_t len = strlen(line);
    if (len == 0) {
        return;
    }

    /* Determine line type based on first character */
    if (line[0] == '1' && len >= 69) {
        /* This is TLE line 1 */
        if (parser->state == TLE_STATE_NAME) {
            /* No name line before line 1, use empty name */
            parser->name_line[0] = '\0';
        }
        strncpy(parser->line1, line, SGP4_TLE_LINE_LEN - 1);
        parser->line1[SGP4_TLE_LINE_LEN - 1] = '\0';
        parser->state = TLE_STATE_LINE2;
    }
    else if (line[0] == '2' && len >= 69) {
        /* This is TLE line 2 */
        if (parser->state == TLE_STATE_LINE2) {
            strncpy(parser->line2, line, SGP4_TLE_LINE_LEN - 1);
            parser->line2[SGP4_TLE_LINE_LEN - 1] = '\0';

            /* Process complete TLE entry */
            tle_parser_process_entry(parser);
        }
        parser->state = TLE_STATE_NAME;
    }
    else {
        /* This is a name line */
        strncpy(parser->name_line, line, SGP4_TLE_NAME_LEN - 1);
        parser->name_line[SGP4_TLE_NAME_LEN - 1] = '\0';

        /* Trim trailing whitespace from name */
        int name_len = strlen(parser->name_line);
        while (name_len > 0 && (parser->name_line[name_len - 1] == ' ' ||
                                 parser->name_line[name_len - 1] == '\r' ||
                                 parser->name_line[name_len - 1] == '\n')) {
            parser->name_line[--name_len] = '\0';
        }

        parser->state = TLE_STATE_LINE1;
    }
}

/*
 * Feed data chunk to parser.
 * Handles partial lines across chunks.
 */
static void tle_parser_feed(tle_stream_parser_t *parser, const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = data[i];

        if (c == '\n') {
            /* End of line - process it */
            parser->line_buf[parser->line_len] = '\0';

            /* Strip trailing CR if present */
            if (parser->line_len > 0 && parser->line_buf[parser->line_len - 1] == '\r') {
                parser->line_buf[parser->line_len - 1] = '\0';
            }

            tle_parser_process_line(parser, parser->line_buf);

            parser->line_len = 0;

            /* Stop if we've reached max matches */
            if (parser->matches_found >= parser->max_matches) {
                return;
            }
        }
        else if (parser->line_len < (int)sizeof(parser->line_buf) - 1) {
            /* Add character to line buffer */
            parser->line_buf[parser->line_len++] = c;
        }
    }
}

/*
 * HTTPS callback for TLE download.
 * Parses incoming data and filters satellites.
 */
static bool tle_download_callback(const char *data, size_t len, void *user_data)
{
    tle_stream_parser_t *parser = (tle_stream_parser_t *)user_data;

    tle_parser_feed(parser, data, len);

    /* Continue reading unless we've found enough matches */
    return parser->matches_found < parser->max_matches;
}

// ============================================================================
// TLE Database Loading
// ============================================================================

/*
 * Load TLE database from file into global state.
 * Must be called with g_tle_database_mutex held.
 */
static int load_tle_database(const char *filepath)
{
    FILE *file = fopen(filepath, "r");
    if (file == NULL) {
        LOG_DEBUG("TLE", "Cannot open TLE file: %s", filepath);
        return -1;
    }

    int count = 0;
    sgp4_tle_t tle;
    sgp4_error_t err;

    while ((err = sgp4_read_tle_stream(file, &tle)) == SGP4_SUCCESS && count < MAX_SATELLITES) {
        /* Convert TLE to SGP4 elements and initialize state */
        sgp4_elements_t elements;
        err = sgp4_tle_to_elements(&tle, &elements);
        if (err != SGP4_SUCCESS) {
            LOG_WARN("TLE", "Failed to convert TLE for %s: %d", tle.name, err);
            continue;
        }

        sgp4_state_t state;
        err = sgp4_init(&state, &elements);
        if (err != SGP4_SUCCESS) {
            LOG_WARN("TLE", "Failed to init SGP4 for %s: %d", tle.name, err);
            continue;
        }

        /* Store in database */
        g_state.satellites[count].valid = true;
        g_state.satellites[count].tle = tle;
        g_state.satellites[count].state = state;
        count++;

        LOG_DEBUG("TLE", "Loaded: %s (NORAD %d)", tle.name, tle.norad_id);
    }

    fclose(file);

    g_state.satellite_count = count;
    LOG_INFO("TLE", "Loaded %d satellites from %s", count, filepath);

    return count;
}

/*
 * Find satellite name by NORAD ID.
 * Must be called with g_tle_database_mutex held.
 * Returns pointer to name string if found, NULL otherwise.
 */
static const char *find_satellite_name(int norad_id)
{
    for (int i = 0; i < g_state.satellite_count; i++) {
        if (g_state.satellites[i].valid &&
            g_state.satellites[i].tle.norad_id == norad_id) {
            return g_state.satellites[i].tle.name;
        }
    }
    return NULL;
}

/*
 * Check if network is available (has a valid IP address).
 */
static bool network_is_available(const char *ifname)
{
    char ipv4_addr[CONFIG_IPV4_ADDR_MAX];
    char ipv6_addr[CONFIG_IPV6_ADDR_MAX];

    network_get_addresses(ifname, ipv4_addr, sizeof(ipv4_addr),
                          ipv6_addr, sizeof(ipv6_addr));

    return (ipv4_addr[0] != '\0' || ipv6_addr[0] != '\0');
}

/*
 * Download TLE data from URL and save filtered results to file.
 * Returns number of satellites saved, or -1 on error.
 */
static int download_tle_data(const char *url, const int *norad_ids, int norad_count,
                             const char *output_path, int max_satellites)
{
    /* Open output file */
    FILE *output = fopen(output_path, "w");
    if (output == NULL) {
        LOG_ERROR("TLE", "Failed to create temp file: %s", output_path);
        return -1;
    }

    /* Initialize parser - static to reduce stack usage */
    static tle_stream_parser_t parser;
    tle_parser_init(&parser, norad_ids, norad_count, output, max_satellites);

    /* Download and parse */
    LOG_INFO("TLE", "Downloading TLE from: %s", url);

    https_error_t err = https_get_stream(url, 60, tle_download_callback, &parser);

    fclose(output);

    if (err != HTTPS_SUCCESS) {
        LOG_ERROR("TLE", "Download failed: %s", https_error_string(err));
        unlink(output_path);
        return -1;
    }

    LOG_INFO("TLE", "Download complete: parsed %d, matched %d satellites",
             parser.total_parsed, parser.matches_found);

    return parser.matches_found;
}

/*
 * TLE Updater Task
 *
 * Downloads TLE data from configured URL, filters by configured satellites,
 * saves to SD card, and loads into memory.
 * Notifies controller when database is updated.
 */
rtems_task tle_updater_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("TLE", "Updater task started");

    /* Get initial configuration - static to reduce stack usage */
    static config_t cfg;
    config_get_copy(&cfg);

    /* Try to load existing TLE from file on startup */
    rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    int loaded = load_tle_database(TLE_FILE_PATH);
    rtems_semaphore_release(g_tle_database_mutex);

    if (loaded > 0) {
        /* Notify controller that TLE database is ready */
        tle_update_message_t msg;
        msg.type = MSG_TLE_DATABASE_UPDATED;
        rtems_message_queue_send(g_tle_queue, &msg, sizeof(msg));
    }

    /* Main update loop */
    while (true) {
        /* Refresh configuration */
        config_get_copy(&cfg);

        /* Check if networking is enabled */
        if (!cfg.network.enabled) {
            LOG_DEBUG("TLE", "Network disabled, skipping update");
            rtems_task_wake_after(cfg.tle_update_interval_hours * 60 * 60 *
                                  rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Wait for network to be available */
        int retries = 0;
        while (!network_is_available(cfg.network.interface) && retries < 12) {
            LOG_DEBUG("TLE", "Waiting for network...");
            rtems_task_wake_after(5 * rtems_clock_get_ticks_per_second());
            retries++;
        }

        if (!network_is_available(cfg.network.interface)) {
            LOG_WARN("TLE", "Network not available after 60s, will retry later");
            rtems_task_wake_after(5 * 60 * rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Download TLE data to temp file */
        int downloaded = download_tle_data(
            cfg.tle_url,
            cfg.satellite_count > 0 ? cfg.satellite_norad_ids : NULL,
            cfg.satellite_count,
            TLE_TEMP_FILE_PATH,
            MAX_SATELLITES
        );

        if (downloaded > 0) {
            /* Remove existing file first (required for FAT filesystems) */
            unlink(TLE_FILE_PATH);

            /* Rename temp file to final location */
            if (rename(TLE_TEMP_FILE_PATH, TLE_FILE_PATH) != 0) {
                LOG_ERROR("TLE", "Failed to rename temp file: %s", strerror(errno));
                unlink(TLE_TEMP_FILE_PATH);
            } else {
                /* Load new TLE data into memory */
                rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                load_tle_database(TLE_FILE_PATH);
                rtems_semaphore_release(g_tle_database_mutex);

                /* Notify controller */
                tle_update_message_t msg;
                msg.type = MSG_TLE_DATABASE_UPDATED;
                rtems_message_queue_send(g_tle_queue, &msg, sizeof(msg));
            }
        } else if (downloaded == 0) {
            LOG_WARN("TLE", "No matching satellites found in download");
            unlink(TLE_TEMP_FILE_PATH);
        } else {
            /* Download failed, keep existing data */
            LOG_WARN("TLE", "Download failed, keeping existing TLE data");
        }

        /* Wait for next update interval */
        rtems_task_wake_after(cfg.tle_update_interval_hours * 60 * 60 *
                              rtems_clock_get_ticks_per_second());
    }
}

/*
 * Helper function to convert radio mode enum to string.
 */
static const char *radio_mode_to_string(radio_mode_t mode) {
    switch (mode) {
        case RADIO_MODE_LSB:      return "LSB";
        case RADIO_MODE_USB:      return "USB";
        case RADIO_MODE_CW:       return "CW";
        case RADIO_MODE_FM:       return "FM";
        case RADIO_MODE_AM:       return "AM";
        case RADIO_MODE_RTTY_LSB: return "RTTY-L";
        case RADIO_MODE_CW_R:     return "CW-R";
        case RADIO_MODE_DATA_LSB: return "DATA-L";
        case RADIO_MODE_RTTY_USB: return "RTTY-U";
        case RADIO_MODE_DATA_FM:  return "DATA-FM";
        case RADIO_MODE_FM_N:     return "FM-N";
        case RADIO_MODE_DATA_USB: return "DATA-U";
        case RADIO_MODE_AM_N:     return "AM-N";
        case RADIO_MODE_C4FM:     return "C4FM";
        default:                  return "???";
    }
}

/*
 * Helper function to convert radio preamp enum to string.
 */
static const char *radio_preamp_to_string(radio_preamp_t preamp) {
    switch (preamp) {
        case RADIO_PREAMP_IPO:  return "IPO";
        case RADIO_PREAMP_AMP1: return "AMP1";
        case RADIO_PREAMP_AMP2: return "AMP2";
        default:                return "???";
    }
}

/*
 * Status Task
 *
 * Periodically logs system status including GPS, antenna, radio, and TLE info.
 * Runs every 30 seconds.
 */
rtems_task status_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("STATUS", "Status task started");

    while (true) {
        // Wait 30 seconds between status reports
        rtems_task_wake_after(30 * rtems_clock_get_ticks_per_second());

        // Take a snapshot of the current state
        rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        bool location_valid = g_state.location_valid;
        bool time_valid = g_state.time_valid;
        bool antenna_valid = g_state.antenna_position_valid;
        bool radio_valid = g_state.radio_status_valid;
        int sat_count = g_state.satellite_count;

        sgp4_geodetic_t location = g_state.observer_location;
        time_t current_time = g_state.current_time;
        double ant_az = g_state.antenna_azimuth;
        double ant_el = g_state.antenna_elevation;

        uint64_t vfo_a = g_state.radio_vfo_a_freq_hz;
        uint64_t vfo_b = g_state.radio_vfo_b_freq_hz;
        radio_mode_t mode = g_state.radio_mode;
        radio_preamp_t preamp = g_state.radio_preamp;
        uint8_t active_vfo = g_state.radio_active_vfo;

        rtems_semaphore_release(g_state_mutex);

        // Log status header
        LOG_INFO("STATUS", "=== System Status ===");

        // GPS status
        if (time_valid) {
            struct tm *tm_info = gmtime(&current_time);
            LOG_INFO("STATUS", "GPS Time: %04d-%02d-%02d %02d:%02d:%02d UTC",
                     tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                     tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
        } else {
            LOG_INFO("STATUS", "GPS Time: not available");
        }

        if (location_valid) {
            LOG_INFO("STATUS", "GPS Pos: %.4f, %.4f, %.0fm",
                     location.lat_rad * SGP4_RAD_TO_DEG,
                     location.lon_rad * SGP4_RAD_TO_DEG,
                     location.alt_km * 1000.0);
        } else {
            LOG_INFO("STATUS", "GPS Pos: not available");
        }

        // Antenna status
        if (antenna_valid) {
            LOG_INFO("STATUS", "Antenna: az=%.1f el=%.1f deg",
                     ant_az * SGP4_RAD_TO_DEG,
                     ant_el * SGP4_RAD_TO_DEG);
        } else {
            LOG_INFO("STATUS", "Antenna: not available");
        }

        // Radio status
        if (radio_valid) {
            LOG_INFO("STATUS", "Radio: VFO-%c active, mode=%s, preamp=%s",
                     active_vfo == 0 ? 'A' : 'B',
                     radio_mode_to_string(mode),
                     radio_preamp_to_string(preamp));
            LOG_INFO("STATUS", "Radio: VFO-A=%.6f MHz, VFO-B=%.6f MHz",
                     vfo_a / 1e6, vfo_b / 1e6);
        } else {
            LOG_INFO("STATUS", "Radio: not connected");
        }

        // TLE database status
        LOG_INFO("STATUS", "TLE: %d satellites loaded", sat_count);

        // Upcoming passes status
        {
            config_t pass_cfg;
            config_get_copy(&pass_cfg);
            int display_count = pass_cfg.pass.status_display_count;

            /* Take a snapshot of pass scheduling state */
            rtems_semaphore_obtain(g_pass_queue_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

            int queue_count = pass_queue_count(&g_upcoming_passes);
            int total_passes = queue_count + (g_scheduled_pass.active ? 1 : 0);

            /* Copy passes for display (scheduled pass + queue contents) */
            pass_info_t display_passes[20];
            int num_display = 0;

            /* First add the scheduled pass if active */
            if (g_scheduled_pass.active && num_display < display_count) {
                display_passes[num_display++] = g_scheduled_pass.pass;
            }

            /* Then add passes from queue (they're in priority order) */
            for (int i = 0; i < queue_count && num_display < display_count; i++) {
                if (i < PASS_QUEUE_CAPACITY) {
                    display_passes[num_display++] = g_upcoming_passes.passes[i];
                }
            }

            rtems_semaphore_release(g_pass_queue_mutex);

            LOG_INFO("STATUS", "Passes: %d upcoming", total_passes);

            if (num_display > 0) {
                /* Get satellite names from TLE database */
                rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

                /* Print table header */
                LOG_INFO("STATUS", "  %-20s %5s  %8s  %8s  %6s  %6s  %6s",
                         "Satellite", "NORAD", "AOS", "LOS", "AOS Az", "LOS Az", "MaxEl");
                LOG_INFO("STATUS", "  %-20s %5s  %8s  %8s  %6s  %6s  %6s",
                         "--------------------", "-----", "--------", "--------",
                         "------", "------", "------");

                for (int i = 0; i < num_display; i++) {
                    pass_info_t *p = &display_passes[i];

                    /* Get satellite name */
                    const char *name = find_satellite_name(p->norad_id);
                    char name_buf[21];
                    if (name) {
                        strncpy(name_buf, name, 20);
                        name_buf[20] = '\0';
                    } else {
                        snprintf(name_buf, sizeof(name_buf), "Unknown");
                    }

                    /* Convert times to HH:MM:SS */
                    double aos_unix = sgp4_jd_to_unix(p->aos_jd);
                    double los_unix = sgp4_jd_to_unix(p->los_jd);
                    struct tm *aos_tm = gmtime((time_t *)&aos_unix);
                    int aos_h = aos_tm->tm_hour;
                    int aos_m = aos_tm->tm_min;
                    int aos_s = aos_tm->tm_sec;
                    struct tm *los_tm = gmtime((time_t *)&los_unix);
                    int los_h = los_tm->tm_hour;
                    int los_m = los_tm->tm_min;
                    int los_s = los_tm->tm_sec;

                    LOG_INFO("STATUS", "  %-20s %5d  %02d:%02d:%02d  %02d:%02d:%02d  %5.1f°  %5.1f°  %5.1f°",
                             name_buf,
                             p->norad_id,
                             aos_h, aos_m, aos_s,
                             los_h, los_m, los_s,
                             p->aos_azimuth_rad * SGP4_RAD_TO_DEG,
                             p->los_azimuth_rad * SGP4_RAD_TO_DEG,
                             p->max_elevation_rad * SGP4_RAD_TO_DEG);
                }

                rtems_semaphore_release(g_tle_database_mutex);
            }
        }

        // Network status
        {
            config_t net_cfg;
            config_get_copy(&net_cfg);

            if (net_cfg.network.enabled) {
                char ipv4_addr[CONFIG_IPV4_ADDR_MAX];
                char ipv6_addr[CONFIG_IPV6_ADDR_MAX];

                network_get_addresses(net_cfg.network.interface,
                                      ipv4_addr, sizeof(ipv4_addr),
                                      ipv6_addr, sizeof(ipv6_addr));

                if (ipv4_addr[0] != '\0' || ipv6_addr[0] != '\0') {
                    LOG_INFO("STATUS", "Network: %s", net_cfg.network.interface);
                    if (ipv4_addr[0] != '\0') {
                        LOG_INFO("STATUS", "  IPv4: %s", ipv4_addr);
                    }
                    if (ipv6_addr[0] != '\0') {
                        LOG_INFO("STATUS", "  IPv6: %s", ipv6_addr);
                    }
                } else {
                    LOG_INFO("STATUS", "Network: %s (no address)", net_cfg.network.interface);
                }
            } else {
                LOG_INFO("STATUS", "Network: disabled");
            }
        }
    }
}

// ============================================================================
// Pass Calculator Helper Functions
// ============================================================================

/* Time step for coarse pass scanning (seconds) */
#define PASS_SCAN_STEP_SEC 60

/* Buffer added to prediction window (minutes) */
#define PASS_WINDOW_BUFFER_MIN 5

/* Binary search precision (seconds) */
#define PASS_REFINE_PRECISION_SEC 1.0

/*
 * Calculate look angles at a specific time.
 * Returns true if calculation was successful.
 */
static bool calculate_look_angles_at_time(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double jd,
    sgp4_look_angles_t *angles)
{
    /* Calculate minutes since TLE epoch */
    double epoch_jd = sat_state->jdsatepoch + sat_state->jdsatepochF;
    double tsince_min = (jd - epoch_jd) * 1440.0;  /* days to minutes */

    /* Propagate satellite position */
    sgp4_result_t result;
    sgp4_error_t err = sgp4_propagate(sat_state, tsince_min, &result);
    if (err != SGP4_SUCCESS) {
        return false;
    }

    /* Convert ECI to ECEF */
    double gst = sgp4_gstime(jd);
    sgp4_vec3_t eci = { result.r[0], result.r[1], result.r[2] };
    sgp4_vec3_t ecef;
    sgp4_eci_to_ecef(&eci, gst, &ecef);

    /* Calculate look angles */
    sgp4_look_angles(&ecef, observer, angles);

    return true;
}

/*
 * Refine a visibility transition (AOS or LOS) using binary search.
 *
 * @param sat_state      Satellite SGP4 state
 * @param observer       Observer location
 * @param t_before       Time when satellite was NOT visible
 * @param t_after        Time when satellite WAS visible (for AOS)
 * @param min_el_rad     Minimum elevation threshold
 * @param looking_for_aos True if looking for AOS, false for LOS
 * @return               Refined transition time as Julian Date
 */
static double refine_transition(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double t_before,
    double t_after,
    double min_el_rad,
    bool looking_for_aos)
{
    double precision_jd = PASS_REFINE_PRECISION_SEC / 86400.0;

    while ((t_after - t_before) > precision_jd) {
        double t_mid = (t_before + t_after) / 2.0;

        sgp4_look_angles_t angles;
        if (!calculate_look_angles_at_time(sat_state, observer, t_mid, &angles)) {
            break;  /* Propagation error - return best estimate */
        }

        bool is_visible = angles.elevation_rad >= min_el_rad;

        if (looking_for_aos) {
            if (is_visible) {
                t_after = t_mid;
            } else {
                t_before = t_mid;
            }
        } else {  /* looking for LOS */
            if (is_visible) {
                t_before = t_mid;
            } else {
                t_after = t_mid;
            }
        }
    }

    return (t_before + t_after) / 2.0;
}

/*
 * Find maximum elevation during a pass using ternary search.
 *
 * @param sat_state  Satellite SGP4 state
 * @param observer   Observer location
 * @param aos_jd     Start of pass (AOS)
 * @param los_jd     End of pass (LOS)
 * @param max_el_out Output: maximum elevation in radians
 * @return           True if successful
 */
static bool find_max_elevation(
    const sgp4_state_t *sat_state,
    const sgp4_geodetic_t *observer,
    double aos_jd,
    double los_jd,
    double *max_el_out)
{
    double low = aos_jd;
    double high = los_jd;
    double precision_jd = 1.0 / 86400.0;  /* 1 second */

    while ((high - low) > precision_jd) {
        double mid1 = low + (high - low) / 3.0;
        double mid2 = high - (high - low) / 3.0;

        sgp4_look_angles_t angles1, angles2;
        if (!calculate_look_angles_at_time(sat_state, observer, mid1, &angles1) ||
            !calculate_look_angles_at_time(sat_state, observer, mid2, &angles2)) {
            return false;
        }

        if (angles1.elevation_rad < angles2.elevation_rad) {
            low = mid1;
        } else {
            high = mid2;
        }
    }

    /* Get final elevation at peak */
    sgp4_look_angles_t angles;
    if (!calculate_look_angles_at_time(sat_state, observer, (low + high) / 2.0, &angles)) {
        return false;
    }

    *max_el_out = angles.elevation_rad;
    return true;
}

/*
 * Find the next satellite pass within the given time window.
 *
 * @param sat_state    Satellite SGP4 state
 * @param norad_id     Satellite NORAD ID
 * @param observer     Observer location
 * @param start_jd     Start of search window (Julian Date)
 * @param end_jd       End of search window (Julian Date)
 * @param min_el_rad   Minimum elevation for visibility
 * @param pass_out     Output: found pass information
 * @return             True if a pass was found
 */
static bool find_next_pass(
    const sgp4_state_t *sat_state,
    int norad_id,
    const sgp4_geodetic_t *observer,
    double start_jd,
    double end_jd,
    double min_el_rad,
    pass_info_t *pass_out)
{
    double step_jd = PASS_SCAN_STEP_SEC / 86400.0;

    /* Check if satellite is currently visible (skip in-progress passes) */
    sgp4_look_angles_t initial_angles;
    if (calculate_look_angles_at_time(sat_state, observer, start_jd, &initial_angles)) {
        if (initial_angles.elevation_rad >= min_el_rad) {
            /* Satellite is already visible - skip to find next pass */
            /* Scan forward to find when it sets */
            double t = start_jd;
            bool was_visible = true;
            while (t < end_jd && was_visible) {
                t += step_jd;
                sgp4_look_angles_t angles;
                if (calculate_look_angles_at_time(sat_state, observer, t, &angles)) {
                    was_visible = angles.elevation_rad >= min_el_rad;
                } else {
                    break;
                }
            }
            /* Now start searching from when it set */
            start_jd = t;
        }
    }

    /* Coarse scan for AOS */
    bool prev_visible = false;
    double aos_approx = 0.0;
    double los_approx = 0.0;
    bool found_aos = false;

    for (double t = start_jd; t < end_jd; t += step_jd) {
        sgp4_look_angles_t angles;
        if (!calculate_look_angles_at_time(sat_state, observer, t, &angles)) {
            continue;  /* Propagation failed - skip this time point */
        }

        bool curr_visible = angles.elevation_rad >= min_el_rad;

        if (!prev_visible && curr_visible) {
            /* Found approximate AOS */
            aos_approx = t;
            found_aos = true;
        } else if (prev_visible && !curr_visible && found_aos) {
            /* Found approximate LOS */
            los_approx = t;

            /* Refine AOS */
            double aos_refined = refine_transition(
                sat_state, observer,
                aos_approx - step_jd, aos_approx,
                min_el_rad, true);

            /* Refine LOS */
            double los_refined = refine_transition(
                sat_state, observer,
                los_approx - step_jd, los_approx,
                min_el_rad, false);

            /* Find max elevation */
            double max_el;
            if (!find_max_elevation(sat_state, observer, aos_refined, los_refined, &max_el)) {
                max_el = 0.1;  /* Default if calculation fails */
            }

            /* Get AOS and LOS azimuths */
            sgp4_look_angles_t aos_angles, los_angles;
            calculate_look_angles_at_time(sat_state, observer, aos_refined, &aos_angles);
            calculate_look_angles_at_time(sat_state, observer, los_refined, &los_angles);

            /* Populate pass info */
            pass_out->norad_id = norad_id;
            pass_out->aos_jd = aos_refined;
            pass_out->los_jd = los_refined;
            pass_out->max_elevation_rad = max_el;
            pass_out->aos_azimuth_rad = aos_angles.azimuth_rad;
            pass_out->los_azimuth_rad = los_angles.azimuth_rad;

            return true;
        }

        prev_visible = curr_visible;
    }

    /* Check if pass extends beyond window */
    if (found_aos && prev_visible) {
        /* Pass started but didn't end within window - refine AOS and use window end */
        double aos_refined = refine_transition(
            sat_state, observer,
            aos_approx - step_jd, aos_approx,
            min_el_rad, true);

        double max_el;
        if (!find_max_elevation(sat_state, observer, aos_refined, end_jd, &max_el)) {
            max_el = 0.1;
        }

        sgp4_look_angles_t aos_angles, los_angles;
        calculate_look_angles_at_time(sat_state, observer, aos_refined, &aos_angles);
        calculate_look_angles_at_time(sat_state, observer, end_jd, &los_angles);

        pass_out->norad_id = norad_id;
        pass_out->aos_jd = aos_refined;
        pass_out->los_jd = end_jd;  /* Truncated to window */
        pass_out->max_elevation_rad = max_el;
        pass_out->aos_azimuth_rad = aos_angles.azimuth_rad;
        pass_out->los_azimuth_rad = los_angles.azimuth_rad;

        return true;
    }

    return false;
}

/*
 * Pass Calculator Task
 *
 * Periodically calculates upcoming satellite passes.
 * Uses the TLE database and observer location.
 * Sends pass predictions to controller.
 */
rtems_task pass_calculator_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("PASS", "Calculator task started");

    /* Wait for initial GPS fix and TLE load */
    rtems_task_wake_after(10 * rtems_clock_get_ticks_per_second());

    while (true) {
        /* Get configuration */
        config_t cfg;
        config_get_copy(&cfg);

        int prediction_window_min = cfg.pass.prediction_window_min;
        double min_el_rad = cfg.pass.min_elevation_deg * SGP4_DEG_TO_RAD;
        int calc_interval_sec = cfg.pass.calc_interval_sec;

        /* Get current state snapshot */
        rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        bool location_valid = g_state.location_valid;
        bool time_valid = g_state.time_valid;
        sgp4_geodetic_t observer = g_state.observer_location;
        time_t current_time = g_state.current_time;
        rtems_semaphore_release(g_state_mutex);

        if (!location_valid || !time_valid) {
            LOG_DEBUG("PASS", "Waiting for GPS fix...");
            rtems_task_wake_after(10 * rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Calculate time window */
        double start_jd = sgp4_unix_to_jd((double)current_time);
        double end_jd = start_jd + (prediction_window_min + PASS_WINDOW_BUFFER_MIN) / 1440.0;

        int passes_found = 0;

        /* Iterate over all satellites */
        rtems_semaphore_obtain(g_tle_database_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        int sat_count = g_state.satellite_count;

        for (int i = 0; i < sat_count && i < MAX_SATELLITES; i++) {
            if (!g_state.satellites[i].valid) {
                continue;
            }

            pass_info_t pass;
            if (find_next_pass(&g_state.satellites[i].state,
                               g_state.satellites[i].tle.norad_id,
                               &observer,
                               start_jd, end_jd,
                               min_el_rad,
                               &pass)) {

                /* Send pass to controller */
                pass_message_t msg;
                msg.type = MSG_PASS_CALCULATED;
                msg.pass = pass;

                rtems_status_code status = rtems_message_queue_send(
                    g_pass_queue, &msg, sizeof(msg));

                if (status == RTEMS_SUCCESSFUL) {
                    passes_found++;
                    LOG_DEBUG("PASS", "Found pass: NORAD %d, max_el=%.1f deg",
                              pass.norad_id,
                              pass.max_elevation_rad * SGP4_RAD_TO_DEG);
                } else if (status == RTEMS_TOO_MANY) {
                    LOG_WARN("PASS", "Pass queue full, dropping pass for NORAD %d",
                             pass.norad_id);
                }
            }
        }

        rtems_semaphore_release(g_tle_database_mutex);

        if (passes_found > 0) {
            LOG_INFO("PASS", "Found %d passes in next %d minutes",
                     passes_found, prediction_window_min);
        }

        /* Sleep until next calculation cycle */
        rtems_task_wake_after(calc_interval_sec * rtems_clock_get_ticks_per_second());
    }
}

/*
 * Pass Executor Task
 *
 * Executes satellite passes by controlling the rotator and radio.
 * Receives commands from the controller via g_executor_cmd_queue.
 * Communicates with the rotator via UART (ttyS2) and radio via UART (ttyS3).
 *
 * NOTE: This is a stub implementation. Full tracking will be added in a future update.
 */
rtems_task pass_executor_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("EXEC", "Pass executor task started");

    while (true) {
        /* Wait for command from controller */
        executor_command_message_t cmd;
        size_t size;

        rtems_status_code status = rtems_message_queue_receive(
            g_executor_cmd_queue, &cmd, &size,
            RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (status != RTEMS_SUCCESSFUL) {
            LOG_ERROR("EXEC", "Failed to receive command: %d", status);
            rtems_task_wake_after(rtems_clock_get_ticks_per_second());
            continue;
        }

        switch (cmd.command) {
            case EXEC_CMD_START_PASS: {
                /* Log pass details */
                double aos_unix = sgp4_jd_to_unix(cmd.pass.aos_jd);
                double los_unix = sgp4_jd_to_unix(cmd.pass.los_jd);
                struct tm *aos_tm = gmtime((time_t *)&aos_unix);
                struct tm *los_tm = gmtime((time_t *)&los_unix);

                LOG_INFO("EXEC", "Received START_PASS for NORAD %d", cmd.pass.norad_id);
                LOG_INFO("EXEC", "  AOS: %02d:%02d:%02d, az=%.1f deg",
                         aos_tm->tm_hour, aos_tm->tm_min, aos_tm->tm_sec,
                         cmd.pass.aos_azimuth_rad * SGP4_RAD_TO_DEG);
                LOG_INFO("EXEC", "  LOS: %02d:%02d:%02d, az=%.1f deg",
                         los_tm->tm_hour, los_tm->tm_min, los_tm->tm_sec,
                         cmd.pass.los_azimuth_rad * SGP4_RAD_TO_DEG);
                LOG_INFO("EXEC", "  Max elevation: %.1f deg",
                         cmd.pass.max_elevation_rad * SGP4_RAD_TO_DEG);

                /*
                 * TODO (future update): Implement actual pass tracking:
                 * 1. Open rotator serial port
                 * 2. Open radio serial port
                 * 3. Pre-position antenna to AOS azimuth/elevation
                 * 4. During pass:
                 *    - Calculate current satellite position using SGP4
                 *    - Send rotator commands to track satellite
                 *    - Apply Doppler correction to radio frequency
                 *    - Handle data reception/transmission
                 * 5. Close serial ports when pass completes
                 * 6. Notify controller of completion
                 */
                break;
            }

            case EXEC_CMD_ABORT_PASS:
                LOG_WARN("EXEC", "Received ABORT_PASS command");
                /* TODO: Stop tracking, return to safe position */
                break;

            default:
                LOG_WARN("EXEC", "Unknown command type: %d", cmd.command);
                break;
        }
    }
}

/*
 * Controller Task
 *
 * Main coordination task that:
 * - Receives messages from all other tasks
 * - Tracks upcoming passes
 * - Commands antenna to track satellites during passes
 * - Shares UART1 access with antenna location task
 */
rtems_task controller_task(rtems_task_argument arg) {
    (void)arg;

    LOG_INFO("CTRL", "Controller task started");

    while (true) {
        // Check GPS queue for updates
        {
            gps_message_t gps_msg;
            size_t gps_size;
            while (rtems_message_queue_receive(g_gps_queue, &gps_msg, &gps_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {
       
                // Update shared state
                rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

                bool log_location = !g_state.location_valid ||
                    g_state.observer_location.lat_rad != gps_msg.location.lat_rad ||
                    g_state.observer_location.lon_rad != gps_msg.location.lon_rad ||
                    g_state.observer_location.alt_km != gps_msg.location.alt_km;

                bool log_time = !g_state.time_valid;

                g_state.observer_location = gps_msg.location;
                g_state.location_valid = true;
                g_state.current_time = gps_msg.utc_time;
                g_state.time_valid = true;
                rtems_semaphore_release(g_state_mutex);

                // Update log timestamp
                log_set_time(gps_msg.utc_time);

                if (log_location) {
                    LOG_INFO("CTRL", "GPS: lat=%.4f deg lon=%.4f deg alt=%.2f km",
                             gps_msg.location.lat_rad * SGP4_RAD_TO_DEG,
                             gps_msg.location.lon_rad * SGP4_RAD_TO_DEG,
                             gps_msg.location.alt_km);
                }

                if (log_time) {
                    struct tm *tm_utc = gmtime(&gps_msg.utc_time);
                    LOG_INFO("CTRL", "GPS Time: %04d-%02d-%02d %02d:%02d:%02d UTC",
                             tm_utc->tm_year + 1900,
                             tm_utc->tm_mon + 1,
                             tm_utc->tm_mday,
                             tm_utc->tm_hour,
                             tm_utc->tm_min,
                             tm_utc->tm_sec);
                }
            }
        }

        // Check antenna queue for position updates
        {
            antenna_position_message_t ant_msg;
            size_t ant_size;
            while (rtems_message_queue_receive(g_antenna_queue, &ant_msg, &ant_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {                
                // Update shared state
                rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                if (!g_state.antenna_position_valid || 
                    !(g_state.antenna_azimuth == ant_msg.azimuth) ||
                    !(g_state.antenna_elevation == ant_msg.elevation)) {
                                LOG_INFO("CTRL", "Antenna: az=%.1f el=%.1f deg",
                         ant_msg.azimuth * SGP4_RAD_TO_DEG,
                         ant_msg.elevation * SGP4_RAD_TO_DEG);
                    g_state.antenna_azimuth = ant_msg.azimuth;
                    g_state.antenna_elevation = ant_msg.elevation;
                    g_state.antenna_position_valid = true;
                }
                rtems_semaphore_release(g_state_mutex);
            }
        }

        // Check TLE queue for database update notifications
        {
            tle_update_message_t tle_msg;
            size_t tle_size;
            while (rtems_message_queue_receive(g_tle_queue, &tle_msg, &tle_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {
                LOG_INFO("CTRL", "TLE database updated");
                // TODO: Trigger pass recalculation if needed
            }
        }

        // Check pass queue for upcoming pass predictions and add to priority queue
        {
            /* Get min schedule elevation from config */
            config_t pass_filter_cfg;
            config_get_copy(&pass_filter_cfg);
            double min_sched_el_rad = pass_filter_cfg.pass.min_schedule_elevation_deg * SGP4_DEG_TO_RAD;

            pass_message_t pass_msg;
            size_t pass_size;
            while (rtems_message_queue_receive(g_pass_queue, &pass_msg, &pass_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {

                /* Filter out passes with max elevation below threshold */
                if (pass_msg.pass.max_elevation_rad < min_sched_el_rad) {
                    LOG_DEBUG("CTRL", "Skipping pass NORAD %d: max_el=%.1f° < min=%.1f°",
                             pass_msg.pass.norad_id,
                             pass_msg.pass.max_elevation_rad * SGP4_RAD_TO_DEG,
                             pass_filter_cfg.pass.min_schedule_elevation_deg);
                    continue;
                }

                rtems_semaphore_obtain(g_pass_queue_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

                /* Remove any existing pass for this satellite (updated prediction) */
                pass_queue_remove_by_norad(&g_upcoming_passes, pass_msg.pass.norad_id);

                /* Insert into priority queue */
                if (pass_queue_insert(&g_upcoming_passes, &pass_msg.pass) == 0) {
                    LOG_DEBUG("CTRL", "Queued pass: NORAD %d, max_el=%.1f deg",
                             pass_msg.pass.norad_id,
                             pass_msg.pass.max_elevation_rad * SGP4_RAD_TO_DEG);
                } else {
                    LOG_WARN("CTRL", "Pass queue full, dropping NORAD %d",
                             pass_msg.pass.norad_id);
                }

                rtems_semaphore_release(g_pass_queue_mutex);
            }
        }

        // Check radio queue for status updates
        {
            radio_status_message_t radio_msg;
            size_t radio_size;
            while (rtems_message_queue_receive(g_radio_queue, &radio_msg, &radio_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {
                rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

                bool freq_changed = false;

                // Only update fields that are valid in this message
                if (radio_msg.valid_fields & RADIO_FIELD_VFO_A_FREQ) {
                    if (g_state.radio_vfo_a_freq_hz != radio_msg.vfo_a_freq_hz) {
                        g_state.radio_vfo_a_freq_hz = radio_msg.vfo_a_freq_hz;
                        freq_changed = true;
                    }
                }
                if (radio_msg.valid_fields & RADIO_FIELD_VFO_B_FREQ) {
                    if (g_state.radio_vfo_b_freq_hz != radio_msg.vfo_b_freq_hz) {
                        g_state.radio_vfo_b_freq_hz = radio_msg.vfo_b_freq_hz;
                        freq_changed = true;
                    }
                }
                if (radio_msg.valid_fields & RADIO_FIELD_AF_GAIN) {
                    g_state.radio_af_gain = radio_msg.af_gain;
                }
                if (radio_msg.valid_fields & RADIO_FIELD_RF_GAIN) {
                    g_state.radio_rf_gain = radio_msg.rf_gain;
                }
                if (radio_msg.valid_fields & RADIO_FIELD_MIC_GAIN) {
                    g_state.radio_mic_gain = radio_msg.mic_gain;
                }
                if (radio_msg.valid_fields & RADIO_FIELD_MODE) {
                    g_state.radio_mode = radio_msg.mode;
                }
                if (radio_msg.valid_fields & RADIO_FIELD_PREAMP) {
                    g_state.radio_preamp = radio_msg.preamp;
                }
                if (radio_msg.valid_fields & RADIO_FIELD_ACTIVE_VFO) {
                    g_state.radio_active_vfo = radio_msg.active_vfo;
                }

                // Mark valid once we've received at least one update
                if (!g_state.radio_status_valid) {
                    g_state.radio_status_valid = true;
                    LOG_INFO("CTRL", "Radio: connected");
                }

                if (freq_changed) {
                    LOG_INFO("CTRL", "Radio: VFO-A=%.6f MHz VFO-B=%.6f MHz",
                             g_state.radio_vfo_a_freq_hz / 1e6,
                             g_state.radio_vfo_b_freq_hz / 1e6);
                }

                rtems_semaphore_release(g_state_mutex);
            }
        }

        // Check controller command queue (config reload, etc.)
        {
            ctrl_cmd_message_t cmd_msg;
            size_t cmd_size;
            while (rtems_message_queue_receive(g_ctrl_cmd_queue, &cmd_msg, &cmd_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {
                switch (cmd_msg.type) {
                    case MSG_CONFIG_RELOAD:
                        LOG_INFO("CTRL", "Config reload requested");
                        if (config_reload() == CONFIG_SUCCESS) {
                            LOG_INFO("CTRL", "Configuration reloaded successfully");
                        } else {
                            LOG_ERROR("CTRL", "Configuration reload failed");
                        }
                        break;
                    default:
                        LOG_WARN("CTRL", "Unknown command type: %d", cmd_msg.type);
                        break;
                }
            }
        }

        // ================================================================
        // Pass Scheduling Logic
        // ================================================================

        /* Get current time and configuration */
        rtems_semaphore_obtain(g_state_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        time_t current_time_t = g_state.current_time;
        bool have_time = g_state.time_valid;
        rtems_semaphore_release(g_state_mutex);

        if (have_time) {
            double current_jd = sgp4_unix_to_jd((double)current_time_t);

            /* Get prep time from config */
            config_t ctrl_cfg;
            config_get_copy(&ctrl_cfg);
            double prep_time_jd = ctrl_cfg.pass.prep_time_sec / 86400.0;
            double replacement_buffer_jd = (ctrl_cfg.pass.prep_time_sec + 60) / 86400.0;

            /* Lock pass queue mutex for all pass scheduling operations */
            rtems_semaphore_obtain(g_pass_queue_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

            /* Prune expired passes from queue */
            int pruned = pass_queue_prune_expired(&g_upcoming_passes, current_jd);
            if (pruned > 0) {
                LOG_DEBUG("CTRL", "Pruned %d expired passes", pruned);
            }

            /* Check if we have a scheduled pass that has ended */
            if (g_scheduled_pass.active && current_jd > g_scheduled_pass.pass.los_jd) {
                LOG_INFO("CTRL", "Pass for NORAD %d completed",
                         g_scheduled_pass.pass.norad_id);
                g_scheduled_pass.active = false;
                g_scheduled_pass.prep_sent = false;
            }

            /* If no pass scheduled, try to schedule one */
            if (!g_scheduled_pass.active && !pass_queue_is_empty(&g_upcoming_passes)) {
                pass_info_t next_pass;
                if (pass_queue_extract(&g_upcoming_passes, &next_pass) == 0) {
                    g_scheduled_pass.active = true;
                    g_scheduled_pass.pass = next_pass;
                    g_scheduled_pass.prep_sent = false;

                    double aos_unix = sgp4_jd_to_unix(next_pass.aos_jd);
                    struct tm *aos_tm = gmtime((time_t *)&aos_unix);
                    LOG_INFO("CTRL", "Scheduled pass: NORAD %d at %02d:%02d:%02d, max_el=%.1f deg",
                             next_pass.norad_id,
                             aos_tm->tm_hour, aos_tm->tm_min, aos_tm->tm_sec,
                             next_pass.max_elevation_rad * SGP4_RAD_TO_DEG);
                }
            }

            /* Dynamic pass replacement: check if a better overlapping pass arrived */
            if (g_scheduled_pass.active && !g_scheduled_pass.prep_sent) {
                /* Only consider replacement if more than 6 minutes until pass */
                double time_until_aos = g_scheduled_pass.pass.aos_jd - current_jd;
                if (time_until_aos > replacement_buffer_jd) {
                    /* Check if queue has a higher-priority overlapping pass */
                    const pass_info_t *candidate = pass_queue_peek(&g_upcoming_passes);
                    if (candidate != NULL &&
                        pass_overlaps(candidate, &g_scheduled_pass.pass) &&
                        candidate->max_elevation_rad > g_scheduled_pass.pass.max_elevation_rad) {

                        LOG_INFO("CTRL", "Replacing NORAD %d (el=%.1f) with NORAD %d (el=%.1f)",
                                 g_scheduled_pass.pass.norad_id,
                                 g_scheduled_pass.pass.max_elevation_rad * SGP4_RAD_TO_DEG,
                                 candidate->norad_id,
                                 candidate->max_elevation_rad * SGP4_RAD_TO_DEG);

                        /* Extract the better pass and schedule it */
                        pass_info_t better_pass;
                        pass_queue_extract(&g_upcoming_passes, &better_pass);
                        g_scheduled_pass.pass = better_pass;
                        /* Old pass is discarded (not re-queued since it was replaced) */
                    }
                }
            }

            /* Check if it's time to send prep command to executor */
            if (g_scheduled_pass.active && !g_scheduled_pass.prep_sent) {
                double time_until_aos = g_scheduled_pass.pass.aos_jd - current_jd;
                if (time_until_aos <= prep_time_jd) {
                    /* Send start command to executor */
                    executor_command_message_t exec_cmd;
                    exec_cmd.command = EXEC_CMD_START_PASS;
                    exec_cmd.pass = g_scheduled_pass.pass;

                    rtems_status_code exec_status = rtems_message_queue_send(
                        g_executor_cmd_queue, &exec_cmd, sizeof(exec_cmd));

                    if (exec_status == RTEMS_SUCCESSFUL) {
                        g_scheduled_pass.prep_sent = true;
                        LOG_INFO("CTRL", "Sent START_PASS to executor for NORAD %d",
                                 g_scheduled_pass.pass.norad_id);
                    } else {
                        LOG_ERROR("CTRL", "Failed to send command to executor: %d",
                                  exec_status);
                    }
                }
            }

            rtems_semaphore_release(g_pass_queue_mutex);
        }

        // Main control loop at 10 Hz
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() / 10);
    }
}

// ============================================================================
// IPC and Task Initialization
// ============================================================================

static rtems_status_code create_message_queues(void) {
    rtems_status_code status;

    // GPS to Controller queue
    status = rtems_message_queue_create(
        rtems_build_name('G', 'P', 'S', 'Q'),
        4,  // max messages
        sizeof(gps_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_gps_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create GPS queue: %d", status);
        return status;
    }

    // Antenna to Controller queue
    status = rtems_message_queue_create(
        rtems_build_name('A', 'N', 'T', 'Q'),
        4,
        sizeof(antenna_position_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_antenna_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create antenna queue: %d", status);
        return status;
    }

    // TLE update notification queue
    status = rtems_message_queue_create(
        rtems_build_name('T', 'L', 'E', 'Q'),
        2,
        sizeof(tle_update_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_tle_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create TLE queue: %d", status);
        return status;
    }

    // Pass prediction queue (increased to handle all satellites)
    status = rtems_message_queue_create(
        rtems_build_name('P', 'A', 'S', 'Q'),
        PASS_QUEUE_CAPACITY,  // matches priority queue capacity
        sizeof(pass_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_pass_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create pass queue: %d", status);
        return status;
    }

    // Radio status queue
    status = rtems_message_queue_create(
        rtems_build_name('R', 'A', 'D', 'Q'),
        4,
        sizeof(radio_status_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_radio_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create radio queue: %d", status);
        return status;
    }

    // Controller command queue (for config reload, etc.)
    status = rtems_message_queue_create(
        rtems_build_name('C', 'M', 'D', 'Q'),
        4,
        sizeof(ctrl_cmd_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_ctrl_cmd_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create controller command queue: %d", status);
        return status;
    }

    // Pass executor command queue (controller -> executor)
    status = rtems_message_queue_create(
        rtems_build_name('E', 'X', 'E', 'C'),
        4,
        sizeof(executor_command_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_executor_cmd_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create executor command queue: %d", status);
        return status;
    }

    // Initialize pass priority queue
    pass_queue_init(&g_upcoming_passes);
    memset(&g_scheduled_pass, 0, sizeof(g_scheduled_pass));

    LOG_INFO("INIT", "Message queues created successfully");
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code create_semaphores(void) {
    rtems_status_code status;

    // UART1 mutex (for antenna rotator communication)
    status = rtems_semaphore_create(
        rtems_build_name('U', 'A', 'R', '1'),
        1,  // initially available
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_uart1_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create UART1 mutex: %d", status);
        return status;
    }

    // TLE database mutex
    status = rtems_semaphore_create(
        rtems_build_name('T', 'L', 'E', 'M'),
        1,
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_tle_database_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create TLE database mutex: %d", status);
        return status;
    }

    // State mutex
    status = rtems_semaphore_create(
        rtems_build_name('S', 'T', 'A', 'T'),
        1,
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_state_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create state mutex: %d", status);
        return status;
    }

    // UART3 mutex (for radio communication)
    status = rtems_semaphore_create(
        rtems_build_name('U', 'A', 'R', '3'),
        1,
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_uart3_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create UART3 mutex: %d", status);
        return status;
    }

    // Pass queue mutex (for pass scheduling data)
    status = rtems_semaphore_create(
        rtems_build_name('P', 'A', 'S', 'Q'),
        1,
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_pass_queue_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create pass queue mutex: %d", status);
        return status;
    }

    LOG_INFO("INIT", "Semaphores created successfully");
    return RTEMS_SUCCESSFUL;
}

static rtems_status_code create_and_start_tasks(void) {
    rtems_status_code status;

    // Create GPS task
    status = rtems_task_create(
        rtems_build_name('G', 'P', 'S', ' '),
        PRIORITY_GPS,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_gps_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create GPS task: %d", status);
        return status;
    }

    // Create Antenna Location task
    status = rtems_task_create(
        rtems_build_name('A', 'N', 'T', ' '),
        PRIORITY_ANTENNA,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_antenna_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create antenna task: %d", status);
        return status;
    }

    // Get configuration for serial port initialization
    config_t cfg;
    config_get_copy(&cfg);

    // Open rotator serial port (shared by antenna and rotator status tasks)
    g_rotator_fd = serial_init_from_config(&cfg.rotator);
    if (g_rotator_fd < 0) {
        LOG_WARN("INIT", "Failed to open rotator port %s", cfg.rotator.device_path);
    }

    // Create Rotator Status task
    status = rtems_task_create(
        rtems_build_name('R', 'O', 'T', 'S'),
        PRIORITY_ROTATOR_STATUS,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_rotator_status_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create rotator status task: %d", status);
        return status;
    }

    // Open radio serial port
    g_radio_fd = serial_init_from_config(&cfg.radio);
    if (g_radio_fd < 0) {
        LOG_WARN("INIT", "Failed to open radio port %s", cfg.radio.device_path);
    }

    // Create Radio Frequency task
    status = rtems_task_create(
        rtems_build_name('R', 'F', 'R', 'Q'),
        PRIORITY_RADIO_FREQ,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_radio_freq_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create radio freq task: %d", status);
        return status;
    }

    // Create Radio Status task
    status = rtems_task_create(
        rtems_build_name('R', 'A', 'D', 'S'),
        PRIORITY_RADIO_STATUS,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_radio_status_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create radio status task: %d", status);
        return status;
    }

    // Create TLE Updater task (larger stack for OpenSSL)
    status = rtems_task_create(
        rtems_build_name('T', 'L', 'E', ' '),
        PRIORITY_TLE,
        TLE_TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_tle_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create TLE task: %d", status);
        return status;
    }

    // Create Pass Calculator task
    status = rtems_task_create(
        rtems_build_name('P', 'A', 'S', ' '),
        PRIORITY_PASS,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_pass_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create pass calculator task: %d", status);
        return status;
    }

    // Create Pass Executor task
    status = rtems_task_create(
        rtems_build_name('E', 'X', 'E', 'C'),
        PRIORITY_PASS_EXECUTOR,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_pass_executor_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create pass executor task: %d", status);
        return status;
    }

    // Create Controller task
    status = rtems_task_create(
        rtems_build_name('C', 'T', 'R', 'L'),
        PRIORITY_CONTROLLER,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_controller_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create controller task: %d", status);
        return status;
    }

    // Create Status task
    status = rtems_task_create(
        rtems_build_name('S', 'T', 'A', 'T'),
        PRIORITY_STATUS,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_status_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create status task: %d", status);
        return status;
    }

    LOG_INFO("INIT", "Tasks created successfully");

    // Start all tasks
    status = rtems_task_start(g_gps_task_id, gps_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_antenna_task_id, antenna_location_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_rotator_status_task_id, rotator_status_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_radio_freq_task_id, radio_frequency_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_radio_status_task_id, radio_status_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_tle_task_id, tle_updater_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_task_id, pass_calculator_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_executor_task_id, pass_executor_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_controller_task_id, controller_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_status_task_id, status_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    LOG_INFO("INIT", "All tasks started successfully");
    return RTEMS_SUCCESSFUL;
}

// ============================================================================
// Init Task (entry point)
// ============================================================================

rtems_task Init(rtems_task_argument ignored) {
    (void)ignored;
    rtems_status_code status;

    printf("\n*** GROUNDSTATION CONTROLLER ***\n");

    // Initialize shared state
    memset(&g_state, 0, sizeof(g_state));
    g_state.location_valid = false;
    g_state.time_valid = false;
    g_state.antenna_position_valid = false;
    g_state.satellite_count = 0;
    g_state.radio_status_valid = false;

    // Initialize logging
    status = log_init();
    if (status != RTEMS_SUCCESSFUL) {
        printf("FATAL: Failed to initialize logging\n");
        exit(1);
    }

    // Initialize libbsd (required for SD card driver)
    if (!libbsd_init()) {
        LOG_WARN("INIT", "libbsd init failed, SD card not available");
    } else if (!mount_sd_card()) {
        LOG_WARN("INIT", "SD card mount failed, using default configuration");
    }

    status = config_system_init(NULL);
    if (status != RTEMS_SUCCESSFUL) {
        LOG_WARN("INIT", "Config init failed, using defaults");
    }

    // Initialize networking
    {
        config_t cfg;
        config_get_copy(&cfg);
        if (!network_init(&cfg.network)) {
            LOG_WARN("INIT", "Network initialization failed");
        }
    }

    // Create IPC objects
    status = create_semaphores();
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create semaphores");
        exit(1);
    }

    status = create_message_queues();
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create message queues");
        exit(1);
    }

    // Create and start all tasks
    status = create_and_start_tasks();
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create/start tasks");
        exit(1);
    }

    LOG_INFO("INIT", "Initialization complete. Tasks running.");

    // Init task can now exit - other tasks will continue running
    // In a real application, you might want to keep this task alive
    // for monitoring or to handle shutdown requests
    rtems_task_exit();
}
