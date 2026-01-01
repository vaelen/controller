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
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
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
#include <math.h>
#include <dirent.h>

#include "sgp4.h"
#include "nmea.h"
#include "log.h"
#include "config.h"
#include "https_client.h"

#include "shared.h"
#include "serial.h"
#include "gps.h"
#include "rotator.h"
#include "radio.h"
#include "pass.h"

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
// Global Variable Definitions (exported via shared.h)
// ============================================================================

// Global state structures
controller_state_t g_state;
executor_tracking_state_t g_executor_state;
pass_priority_queue_t g_upcoming_passes;
scheduled_pass_t g_scheduled_pass;

// Message queues
rtems_id g_gps_queue;
rtems_id g_antenna_queue;
rtems_id g_tle_queue;
rtems_id g_pass_queue;
rtems_id g_radio_queue;
rtems_id g_ctrl_cmd_queue;
rtems_id g_executor_cmd_queue;
rtems_id g_rotator_cmd_queue;

// Semaphores (mutexes)
rtems_id g_uart1_mutex;
rtems_id g_uart3_mutex;
rtems_id g_tle_database_mutex;
rtems_id g_state_mutex;
rtems_id g_pass_queue_mutex;

// Shared file descriptors
int g_rotator_fd = -1;
int g_radio_fd = -1;

// Task IDs (local to controller.c)
static rtems_id g_gps_task_id;
static rtems_id g_antenna_task_id;
static rtems_id g_tle_task_id;
static rtems_id g_pass_task_id;
static rtems_id g_pass_executor_task_id;
static rtems_id g_controller_task_id;
static rtems_id g_rotator_status_task_id;
static rtems_id g_radio_status_task_id;
static rtems_id g_radio_freq_task_id;
static rtems_id g_rotator_cmd_task_id;
static rtems_id g_status_task_id;

// ============================================================================
// Utility Functions
// ============================================================================

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

// ============================================================================
// SD Card Initialization
// ============================================================================

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
        char *route_argv[] = {
            "route",
            "add",
            "-inet6",
            "default",
            (char *)cfg->ipv6.gateway,
            NULL
        };

        rtems_bsd_command_route(5, route_argv);
        LOG_INFO("NET", "IPv6 gateway: %s", cfg->ipv6.gateway);
    }
}

/*
 * Start dhcpcd daemon for DHCP/SLAAC configuration.
 */
static void network_start_dhcpcd(const network_config_t *cfg)
{
    /* Build dhcpcd options based on configuration */
    const char *argv[8];
    int argc = 0;

    argv[argc++] = "dhcpcd";

    /* Interface to configure */
    argv[argc++] = cfg->interface;

    /* If only IPv4, disable IPv6 in dhcpcd */
    if (!cfg->ipv6.enabled || (!cfg->ipv6.dhcp && !cfg->ipv6.slaac)) {
        argv[argc++] = "-4";  /* IPv4 only */
    }

    /* If only IPv6, disable IPv4 in dhcpcd */
    if (!cfg->ipv4.enabled || !cfg->ipv4.dhcp) {
        argv[argc++] = "-6";  /* IPv6 only */
    }

    argv[argc] = NULL;

    LOG_INFO("NET", "Starting dhcpcd...");
    rtems_dhcpcd_start(NULL);

    /* dhcpcd runs in background, wait briefly for it to get an address */
    rtems_task_wake_after(3 * rtems_clock_get_ticks_per_second());
}

/*
 * Get current IP addresses for an interface.
 * Fills ipv4_addr and ipv6_addr buffers with addresses (empty string if none).
 */
void network_get_addresses(const char *ifname,
                           char *ipv4_addr, size_t ipv4_size,
                           char *ipv6_addr, size_t ipv6_size)
{
    ipv4_addr[0] = '\0';
    ipv6_addr[0] = '\0';

    struct ifaddrs *ifap;
    if (getifaddrs(&ifap) != 0) {
        return;
    }

    for (struct ifaddrs *ifa = ifap; ifa != NULL; ifa = ifa->ifa_next) {
        if (strcmp(ifa->ifa_name, ifname) != 0 || ifa->ifa_addr == NULL) {
            continue;
        }

        if (ifa->ifa_addr->sa_family == AF_INET && ipv4_addr[0] == '\0') {
            struct sockaddr_in *sin = (struct sockaddr_in *)ifa->ifa_addr;
            inet_ntop(AF_INET, &sin->sin_addr, ipv4_addr, ipv4_size);
        }
        else if (ifa->ifa_addr->sa_family == AF_INET6 && ipv6_addr[0] == '\0') {
            struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)ifa->ifa_addr;
            /* Skip link-local addresses (fe80::) for display */
            if (!IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr)) {
                inet_ntop(AF_INET6, &sin6->sin6_addr, ipv6_addr, ipv6_size);
            }
        }
    }

    freeifaddrs(ifap);
}

/*
 * Initialize networking based on configuration.
 * Returns true on success, false on failure.
 */
bool network_init(const network_config_t *cfg)
{
    if (!cfg->enabled) {
        LOG_INFO("NET", "Networking disabled in configuration");
        return true;
    }

    LOG_INFO("NET", "Initializing network on %s", cfg->interface);

    /* Wait for the interface to appear (up to 10 seconds) */
    if (!network_wait_for_interface(cfg->interface, 10)) {
        LOG_ERROR("NET", "Interface %s not found", cfg->interface);
        return false;
    }

    /* Bring up the interface */
    char *ifcfg_argv[] = { "ifconfig", (char *)cfg->interface, "up", NULL };
    rtems_bsd_command_ifconfig(3, ifcfg_argv);

    /* Configure static addresses if specified */
    if (cfg->ipv4.enabled && !cfg->ipv4.dhcp) {
        if (cfg->ipv4.address[0] != '\0') {
            network_configure_ipv4_static(cfg);
        }
    }

    if (cfg->ipv6.enabled && !cfg->ipv6.dhcp && !cfg->ipv6.slaac) {
        if (cfg->ipv6.address[0] != '\0') {
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
// Status Task
// ============================================================================

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

        // Tracking status
        {
            executor_state_t exec_state = g_executor_state.state;

            if (exec_state == EXEC_STATE_TRACKING) {
                LOG_INFO("STATUS", "TRACKING: %s (NORAD %d)",
                         g_executor_state.sat_name,
                         g_executor_state.current_pass.norad_id);
            } else if (exec_state == EXEC_STATE_PREPOSITIONING ||
                       exec_state == EXEC_STATE_WAITING_AOS) {
                LOG_INFO("STATUS", "PREPARING: %s (NORAD %d)",
                         g_executor_state.sat_name,
                         g_executor_state.current_pass.norad_id);
            }
        }

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
                    LOG_INFO("STATUS", "Network: no address");
                }
            } else {
                LOG_INFO("STATUS", "Network: disabled");
            }
        }

        LOG_INFO("STATUS", "=====================");
    }
}

// ============================================================================
// Controller Task
// ============================================================================

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

    // Rotator command queue (executor -> rotator command task)
    status = rtems_message_queue_create(
        rtems_build_name('R', 'O', 'T', 'C'),
        8,  // Allow buffering of commands during fast tracking
        sizeof(rotator_command_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_rotator_cmd_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create rotator command queue: %d", status);
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

    // Create Rotator Command task
    status = rtems_task_create(
        rtems_build_name('R', 'O', 'T', 'C'),
        PRIORITY_ROTATOR_CMD,
        TASK_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_rotator_cmd_task_id
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create rotator command task: %d", status);
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

    status = rtems_task_start(g_rotator_cmd_task_id, rotator_command_task, 0);
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

    // Initialize executor state
    memset(&g_executor_state, 0, sizeof(g_executor_state));

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
