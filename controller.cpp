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
 *   - Controller Task: Coordinates passes and commands antenna
 */

#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <map>

#include "satellite.hpp"

// ============================================================================
// Message Types
// ============================================================================

enum class MessageType : uint8_t {
    GPS_UPDATE,
    TLE_DATABASE_UPDATED,
    PASS_CALCULATED,
    ANTENNA_POSITION,
    COMMAND_ANTENNA
};

// GPS position and time update message
struct GpsMessage {
    MessageType type;
    sattrack::Geodetic location;
    time_t utcTime;
};

// Antenna current position message
struct AntennaPositionMessage {
    MessageType type;
    double azimuth;    // radians
    double elevation;  // radians
};

// Antenna command message
struct AntennaCommandMessage {
    MessageType type;
    double targetAzimuth;    // radians
    double targetElevation;  // radians
};

// Pass notification message
struct PassMessage {
    MessageType type;
    sattrack::PassInfo pass;
};

// TLE update notification (simple flag)
struct TleUpdateMessage {
    MessageType type;
};

// Union of all message types for queue sizing
union ControllerMessage {
    MessageType type;
    GpsMessage gps;
    AntennaPositionMessage antennaPos;
    AntennaCommandMessage antennaCmd;
    PassMessage pass;
    TleUpdateMessage tleUpdate;
};

// ============================================================================
// Shared State (protected by mutexes)
// ============================================================================

struct ControllerState {
    // Observer location from GPS
    sattrack::Geodetic observerLocation;
    bool locationValid;

    // Current time from GPS
    sattrack::time_point currentTime;
    bool timeValid;

    // TLE database
    std::map<int, sattrack::Satellite> tleDatabase;

    // Current antenna position
    double antennaAzimuth;    // radians
    double antennaElevation;  // radians
    bool antennaPositionValid;
};

static ControllerState g_state;

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
static rtems_id g_controller_task_id;

// ============================================================================
// Task Priorities and Stack Sizes
// ============================================================================

constexpr rtems_task_priority PRIORITY_CONTROLLER = 10;
constexpr rtems_task_priority PRIORITY_ANTENNA = 20;
constexpr rtems_task_priority PRIORITY_GPS = 30;
constexpr rtems_task_priority PRIORITY_PASS = 40;
constexpr rtems_task_priority PRIORITY_TLE = 50;

constexpr size_t TASK_STACK_SIZE = 8 * 1024;

// ============================================================================
// Task Entry Points (extern "C" for RTEMS)
// ============================================================================

extern "C" {

/*
 * GPS Task
 *
 * Receives data from a GPS receiver via UART.
 * Parses NMEA sentences to extract position and time.
 * Sends updates to the controller via message queue.
 */
rtems_task gps_task(rtems_task_argument arg) {
    (void)arg;
    printf("[GPS] Task started\n");

    while (true) {
        // TODO: Initialize UART0 for GPS receiver
        // TODO: Read UART data
        // TODO: Parse NMEA sentences (GGA for position, RMC for time)
        // TODO: Extract latitude, longitude, altitude, and UTC time
        // TODO: Send GpsMessage to g_gps_queue

        // Stub: simulate periodic GPS update
        rtems_task_wake_after(rtems_clock_get_ticks_per_second());
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
        // TODO: Send AntennaPositionMessage to g_antenna_queue

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
        // TODO: Update g_state.tleDatabase with new TLE data
        // TODO: Release g_tle_database_mutex
        // TODO: Send TleUpdateMessage to g_tle_queue to notify controller

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
        //       - Call sattrack::findNextPass() with observer location
        //       - If pass found within window, send PassMessage to g_pass_queue
        // TODO: Release mutexes

        // Stub: simulate periodic pass calculation (every hour)
        rtems_task_wake_after(60 * 60 * rtems_clock_get_ticks_per_second());
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

} // extern "C"

// ============================================================================
// IPC and Task Initialization
// ============================================================================

static rtems_status_code create_message_queues() {
    rtems_status_code status;

    // GPS to Controller queue
    status = rtems_message_queue_create(
        rtems_build_name('G', 'P', 'S', 'Q'),
        4,  // max messages
        sizeof(GpsMessage),
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
        sizeof(AntennaPositionMessage),
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
        sizeof(TleUpdateMessage),
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
        sizeof(PassMessage),
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

static rtems_status_code create_semaphores() {
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

static rtems_status_code create_and_start_tasks() {
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

    status = rtems_task_start(g_controller_task_id, controller_task, 0);
    if (status != RTEMS_SUCCESSFUL) return status;

    printf("All tasks started successfully\n");
    return RTEMS_SUCCESSFUL;
}

// ============================================================================
// Init Task (entry point)
// ============================================================================

extern "C" rtems_task Init(rtems_task_argument ignored) {
    (void)ignored;
    rtems_status_code status;

    printf("\n*** SATELLITE TRACKING CONTROLLER ***\n");
    printf("Initializing...\n");

    // Initialize shared state
    memset(&g_state, 0, sizeof(g_state));
    g_state.locationValid = false;
    g_state.timeValid = false;
    g_state.antennaPositionValid = false;

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
