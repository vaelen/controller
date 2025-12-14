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

// ============================================================================
// Configuration Constants
// ============================================================================

#define MAX_SATELLITES 32

// ============================================================================
// Serial Device Configuration
// ============================================================================

#define GPS_DEVICE_PATH       "/dev/ttyS1"
#define GPS_BAUD_RATE         B9600
#define GPS_FLOW_CONTROL      false

#define ROTATOR_DEVICE_PATH   "/dev/ttyS2"
#define ROTATOR_BAUD_RATE     B9600
#define ROTATOR_FLOW_CONTROL  false

#define RADIO_DEVICE_PATH     "/dev/ttyS3"
#define RADIO_BAUD_RATE       B9600
#define RADIO_FLOW_CONTROL    false

// ============================================================================
// Message Types
// ============================================================================

typedef enum message_type {
    MSG_GPS_UPDATE,
    MSG_TLE_DATABASE_UPDATED,
    MSG_PASS_CALCULATED,
    MSG_ANTENNA_POSITION,
    MSG_COMMAND_ANTENNA
} message_type_t;

// Pass information structure
typedef struct pass_info {
    int norad_id;
    double aos_jd;           // Acquisition of signal (Julian Date)
    double los_jd;           // Loss of signal (Julian Date)
    double max_elevation_rad;
    double aos_azimuth_rad;
    double los_azimuth_rad;
} pass_info_t;

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

// ============================================================================
// Task Priorities and Stack Sizes
// ============================================================================

#define PRIORITY_PASS_EXECUTOR  10
#define PRIORITY_CONTROLLER     20
#define PRIORITY_ROTATOR_STATUS 30
#define PRIORITY_ANTENNA        40
#define PRIORITY_GPS            50
#define PRIORITY_PASS           60
#define PRIORITY_TLE            70

#define TASK_STACK_SIZE     (8 * 1024)

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
    int fd = open(path, flags | O_NOCTTY);
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
 * Initialize the GPS serial port for NMEA input.
 * Returns file descriptor on success, -1 on failure.
 */
static int gps_init(void) {
    return serial_init(GPS_DEVICE_PATH, GPS_BAUD_RATE,
                       O_RDONLY, GPS_FLOW_CONTROL);
}

/*
 * Initialize the rotator serial port for GS-232A communication.
 * Returns file descriptor on success, -1 on failure.
 */
static int rotator_init(void) {
    return serial_init(ROTATOR_DEVICE_PATH, ROTATOR_BAUD_RATE,
                       O_RDWR, ROTATOR_FLOW_CONTROL);
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

    int fd = gps_init();
    if (fd < 0) {
        LOG_ERROR("GPS", "Failed to open GPS port %s", GPS_DEVICE_PATH);
        rtems_task_exit();
    }

    LOG_INFO("GPS", "Serial port %s opened successfully", GPS_DEVICE_PATH);

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
            // For some reason, non-blocking read always reads only 1 byte at a time
            // even if more bytes are available. Keep reading until no more data.
            n = read(fd, read_buf, sizeof(read_buf) - 1);
        }

        /* Process complete lines from buffer */
        while (nmea_buffer_get_line(&nmea_buf, line, sizeof(line))) {
            LOG_DEBUG("GPS", "NMEA: %s", line);

            /* Validate checksum before parsing */
            if (!nmea_validate_checksum(line)) {
                continue;
            }

            if (nmea_is_gga(line)) {
                nmea_parse_gga(line, &last_gga);
            } else if (nmea_is_rmc(line)) {
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
            ssize_t n = read(g_rotator_fd, read_buf, sizeof(read_buf) - 1);
            if (n > 0) {
                // Append to buffer
                for (ssize_t i = 0; i < n && buf_pos < (int)sizeof(buf) - 1; i++) {
                    buf[buf_pos++] = read_buf[i];
                }
                buf[buf_pos] = '\0';
            }
        }

        rtems_semaphore_release(g_uart1_mutex);

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
 * TLE Updater Task
 *
 * Periodically updates the TLE database.
 * May fetch from network, read from file, or receive via other means.
 * Notifies controller when database is updated.
 */
rtems_task tle_updater_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("TLE", "Updater task started");

    while (true) {
        // TODO: Fetch TLE data from source (network, file, etc.)
        // TODO: Acquire g_tle_database_mutex
        // TODO: Update g_state.satellites with new TLE data
        // TODO: Release g_tle_database_mutex
        // TODO: Send tle_update_message_t to g_tle_queue to notify controller

        // Stub: simulate periodic TLE update (every 4 hours)
        rtems_task_wake_after(4 * 60 * 60 * rtems_clock_get_ticks_per_second());
    }
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

    while (true) {
        // TODO: Acquire g_state_mutex to read observer locationz
        // TODO: Acquire g_tle_database_mutex to read TLE database
        // TODO: For each satellite in database:
        //       - Propagate satellite position
        //       - Calculate look angles
        //       - If pass found within window, send pass_message_t to g_pass_queue
        // TODO: Release mutexes

        // Stub: simulate periodic pass calculation (every hour)
        rtems_task_wake_after(60 * 60 * rtems_clock_get_ticks_per_second());
    }
}

/*
 * Pass Executor Task
 *
 * Executes satellite passes by controlling the rotator and radio.
 * Started by the controller when a pass begins.
 * Communicates with the rotator via UART (ttyS2) and radio via UART (ttyS3).
 */
rtems_task pass_executor_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("EXEC", "Pass executor task started");

    while (true) {
        // TODO: Wait for pass execution command from controller
        // TODO: Open rotator serial port (ROTATOR_DEVICE_PATH)
        // TODO: Open radio serial port (RADIO_DEVICE_PATH)
        // TODO: During pass:
        //       - Calculate current satellite position
        //       - Send rotator commands to track satellite
        //       - Configure radio frequency (Doppler correction)
        //       - Handle data reception/transmission
        // TODO: Close serial ports when pass completes
        // TODO: Notify controller of pass completion

        // Stub: wait for pass execution trigger
        rtems_task_wake_after(rtems_clock_get_ticks_per_second());
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
                    LOG_INFO("CTRL", "GPS: lat=%.6f deg lon=%.6f deg alt=%.2f km",
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
                if (!(g_state.antenna_azimuth == ant_msg.azimuth) ||
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

        // Check pass queue for upcoming pass predictions
        {
            pass_message_t pass_msg;
            size_t pass_size;
            while (rtems_message_queue_receive(g_pass_queue, &pass_msg, &pass_size,
                                               RTEMS_NO_WAIT, 0) == RTEMS_SUCCESSFUL) {
                LOG_INFO("CTRL", "Pass: NORAD %d, max_el=%.1f deg",
                         pass_msg.pass.norad_id,
                         pass_msg.pass.max_elevation_rad * SGP4_RAD_TO_DEG);
                // TODO: Schedule pass execution
            }
        }

        // TODO: If a pass is currently active:
        //       - Calculate current look angles to satellite
        //       - Acquire g_uart1_mutex
        //       - Send antenna command via UART1
        //       - Release g_uart1_mutex

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

    // Pass prediction queue
    status = rtems_message_queue_create(
        rtems_build_name('P', 'A', 'S', 'Q'),
        16,  // multiple passes can be queued
        sizeof(pass_message_t),
        RTEMS_DEFAULT_ATTRIBUTES,
        &g_pass_queue
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("INIT", "Failed to create pass queue: %d", status);
        return status;
    }

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

    // Open rotator serial port (shared by antenna and rotator status tasks)
    g_rotator_fd = rotator_init();
    if (g_rotator_fd < 0) {
        LOG_WARN("INIT", "Failed to open rotator port %s", ROTATOR_DEVICE_PATH);
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

    // Create TLE Updater task
    status = rtems_task_create(
        rtems_build_name('T', 'L', 'E', ' '),
        PRIORITY_TLE,
        TASK_STACK_SIZE,
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

    LOG_INFO("INIT", "Tasks created successfully");

    // Start all tasks
    status = rtems_task_start(g_gps_task_id, gps_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_antenna_task_id, antenna_location_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_rotator_status_task_id, rotator_status_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_tle_task_id, tle_updater_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_task_id, pass_calculator_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_executor_task_id, pass_executor_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_controller_task_id, controller_task, 0);
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

    // Initialize logging
    status = log_init();
    if (status != RTEMS_SUCCESSFUL) {
        printf("FATAL: Failed to initialize logging\n");
        exit(1);
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
