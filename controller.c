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
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "sgp4.h"
#include "nmea.h"

// ============================================================================
// Configuration Constants
// ============================================================================

#define MAX_SATELLITES 32

// Serial Device Configuration
#define GPS_DEVICE_PATH       "/dev/ttyS1"
#define GPS_BAUD_RATE         B9600

#define ROTATOR_DEVICE_PATH   "/dev/ttyS2"
#define ROTATOR_BAUD_RATE     B9600

#define RADIO_DEVICE_PATH     "/dev/ttyS3"
#define RADIO_BAUD_RATE       B9600

// ============================================================================
// Message Types
// ============================================================================

typedef enum {
    MSG_GPS_UPDATE,
    MSG_TLE_DATABASE_UPDATED,
    MSG_PASS_CALCULATED,
    MSG_ANTENNA_POSITION,
    MSG_COMMAND_ANTENNA
} message_type_t;

// Pass information structure
typedef struct {
    int norad_id;
    double aos_jd;           // Acquisition of signal (Julian Date)
    double los_jd;           // Loss of signal (Julian Date)
    double max_elevation_rad;
    double aos_azimuth_rad;
    double los_azimuth_rad;
} pass_info_t;

// GPS position and time update message
typedef struct {
    message_type_t type;
    sgp4_geodetic_t location;
    time_t utc_time;
} gps_message_t;

// Antenna current position message
typedef struct {
    message_type_t type;
    double azimuth;    // radians
    double elevation;  // radians
} antenna_position_message_t;

// Antenna command message
typedef struct {
    message_type_t type;
    double target_azimuth;    // radians
    double target_elevation;  // radians
} antenna_command_message_t;

// Pass notification message
typedef struct {
    message_type_t type;
    pass_info_t pass;
} pass_message_t;

// TLE update notification (simple flag)
typedef struct {
    message_type_t type;
} tle_update_message_t;

// Union of all message types for queue sizing
typedef union {
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

typedef struct {
    bool valid;
    sgp4_tle_t tle;
    sgp4_state_t state;
} satellite_entry_t;

// ============================================================================
// Shared State (protected by mutexes)
// ============================================================================

typedef struct {
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

// ============================================================================
// Task Priorities and Stack Sizes
// ============================================================================

#define PRIORITY_CONTROLLER     10
#define PRIORITY_PASS_EXECUTOR  15
#define PRIORITY_ANTENNA        20
#define PRIORITY_GPS            30
#define PRIORITY_PASS           40
#define PRIORITY_TLE            50

#define TASK_STACK_SIZE     (8 * 1024)

// ============================================================================
// Serial Port Initialization
// ============================================================================

/*
 * Initialize GPS serial port for non-blocking reads.
 * Returns file descriptor on success, -1 on failure.
 */
static int gps_init_serial(void) {
    int fd = open(GPS_DEVICE_PATH, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("[GPS] Failed to open %s\n", GPS_DEVICE_PATH);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        printf("[GPS] Failed to get terminal attributes\n");
        close(fd);
        return -1;
    }

    /* Set baud rate */
    cfsetispeed(&tty, GPS_BAUD_RATE);
    cfsetospeed(&tty, GPS_BAUD_RATE);

    /* 8N1 mode */
    tty.c_cflag &= ~PARENB;        /* No parity */
    tty.c_cflag &= ~CSTOPB;        /* 1 stop bit */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            /* 8 data bits */

    /* No hardware flow control */
    tty.c_cflag &= ~CRTSCTS;

    /* Enable receiver, ignore modem control lines */
    tty.c_cflag |= CREAD | CLOCAL;

    /* Raw input mode */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* No software flow control */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Raw output mode */
    tty.c_oflag &= ~OPOST;

    /* Non-blocking: return immediately even with no data */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("[GPS] Failed to set terminal attributes\n");
        close(fd);
        return -1;
    }

    return fd;
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
    printf("[GPS] Task started\n");

    int fd = gps_init_serial();
    if (fd < 0) {
        printf("[GPS] Serial init failed, task exiting\n");
        rtems_task_exit();
    }

    printf("[GPS] Serial port %s opened successfully\n", GPS_DEVICE_PATH);

    nmea_buffer_t nmea_buf;
    nmea_buffer_init(&nmea_buf);

    /* Track latest valid data from each sentence type */
    nmea_gga_t last_gga = {0};
    nmea_rmc_t last_rmc = {0};

    char read_buf[64];
    char line[NMEA_BUFFER_SIZE];

    while (true) {
        /* Non-blocking read from serial */
        ssize_t n = read(fd, read_buf, sizeof(read_buf) - 1);
        if (n > 0) {
            read_buf[n] = '\0';
            nmea_buffer_add(&nmea_buf, read_buf, (int)n);
        }

        /* Process complete lines from buffer */
        while (nmea_buffer_get_line(&nmea_buf, line, sizeof(line))) {
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
 * Communicates with the antenna rotator via UART to read current position.
 * Polls the rotator periodically (every few seconds).
 * Reports position to controller via message queue.
 */
rtems_task antenna_location_task(rtems_task_argument arg) {
    (void)arg;
    printf("[ANTENNA] Location task started\n");

    while (true) {
        // TODO: Acquire g_uart1_mutex before UART access
        // TODO: Send query command to rotator via UART1
        // TODO: Read response with current az/el
        // TODO: Release g_uart1_mutex
        // TODO: Parse response to get azimuth and elevation
        // TODO: Send antenna_position_message_t to g_antenna_queue

        // Stub: simulate periodic position read
        rtems_task_wake_after(2 * rtems_clock_get_ticks_per_second());
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
    printf("[TLE] Updater task started\n");

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
    printf("[PASS] Calculator task started\n");

    while (true) {
        // TODO: Acquire g_state_mutex to read observer location
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
    printf("[EXEC] Pass executor task started\n");

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
    printf("[CTRL] Controller task started\n");

    while (true) {
        // TODO: Check g_gps_queue for GPS updates (non-blocking)
        //       - Update observer location and time in g_state
        // TODO: Check g_antenna_queue for position updates (non-blocking)
        //       - Update antenna position in g_state
        // TODO: Check g_tle_queue for TLE database updates (non-blocking)
        //       - Trigger pass recalculation if needed
        // TODO: Check g_pass_queue for upcoming passes (non-blocking)
        //       - Schedule pass execution

        // TODO: If a pass is currently active:
        //       - Calculate current look angles to satellite
        //       - Acquire g_uart1_mutex
        //       - Send antenna command via UART1
        //       - Release g_uart1_mutex

        // Stub: simulate main control loop at 10 Hz
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
        printf("Failed to create GPS queue: %d\n", status);
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
        printf("Failed to create antenna queue: %d\n", status);
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
        printf("Failed to create TLE queue: %d\n", status);
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
        printf("Failed to create pass queue: %d\n", status);
        return status;
    }

    printf("Message queues created successfully\n");
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
        printf("Failed to create UART1 mutex: %d\n", status);
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
        printf("Failed to create TLE database mutex: %d\n", status);
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
        printf("Failed to create state mutex: %d\n", status);
        return status;
    }

    printf("Semaphores created successfully\n");
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
        printf("Failed to create GPS task: %d\n", status);
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
        printf("Failed to create antenna task: %d\n", status);
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
        printf("Failed to create TLE task: %d\n", status);
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
        printf("Failed to create pass calculator task: %d\n", status);
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
        printf("Failed to create pass executor task: %d\n", status);
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
        printf("Failed to create controller task: %d\n", status);
        return status;
    }

    printf("Tasks created successfully\n");

    // Start all tasks
    status = rtems_task_start(g_gps_task_id, gps_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_antenna_task_id, antenna_location_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_tle_task_id, tle_updater_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_task_id, pass_calculator_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_pass_executor_task_id, pass_executor_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    status = rtems_task_start(g_controller_task_id, controller_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    printf("All tasks started successfully\n");
    return RTEMS_SUCCESSFUL;
}

// ============================================================================
// Init Task (entry point)
// ============================================================================

rtems_task Init(rtems_task_argument ignored) {
    (void)ignored;
    rtems_status_code status;

    printf("\n*** SATELLITE TRACKING CONTROLLER ***\n");
    printf("Initializing...\n");

    // Initialize shared state
    memset(&g_state, 0, sizeof(g_state));
    g_state.location_valid = false;
    g_state.time_valid = false;
    g_state.antenna_position_valid = false;
    g_state.satellite_count = 0;

    // Create IPC objects
    status = create_message_queues();
    if (status != RTEMS_SUCCESSFUL) {
        printf("Failed to create message queues\n");
        exit(1);
    }

    status = create_semaphores();
    if (status != RTEMS_SUCCESSFUL) {
        printf("Failed to create semaphores\n");
        exit(1);
    }

    // Create and start all tasks
    status = create_and_start_tasks();
    if (status != RTEMS_SUCCESSFUL) {
        printf("Failed to create/start tasks\n");
        exit(1);
    }

    printf("Initialization complete. Tasks running.\n");
    printf("*** CONTROLLER ACTIVE ***\n\n");

    // Init task can now exit - other tasks will continue running
    // In a real application, you might want to keep this task alive
    // for monitoring or to handle shutdown requests
    rtems_task_exit();
}
