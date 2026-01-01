/*
 * Shared types and declarations for the Satellite Tracking Controller
 *
 * This header contains message types, enums, and extern declarations
 * shared across multiple modules in the controller system.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef SHARED_H
#define SHARED_H

#include <rtems.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "sgp4.h"
#include "priority_queue.h"
#include "config.h"

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
// Rotator Command Types
// ============================================================================

// Commands sent from pass executor to rotator command task
typedef enum rotator_command_type {
    ROT_CMD_GOTO,              // Move to specified az/el
    ROT_CMD_STOP,              // Stop movement
    ROT_CMD_PARK               // Return to park position (az=0, el=0)
} rotator_command_type_t;

// Message from pass executor to rotator command task
typedef struct rotator_command_message {
    rotator_command_type_t command;
    double target_azimuth_rad;    // For ROT_CMD_GOTO
    double target_elevation_rad;  // For ROT_CMD_GOTO
} rotator_command_message_t;

// ============================================================================
// Pass Executor State
// ============================================================================

typedef enum executor_state {
    EXEC_STATE_IDLE,           // No pass in progress
    EXEC_STATE_PREPOSITIONING, // Moving antenna to AOS position
    EXEC_STATE_WAITING_AOS,    // Antenna positioned, waiting for AOS
    EXEC_STATE_TRACKING,       // Active satellite tracking
    EXEC_STATE_COMPLETING      // Pass ending, cleanup
} executor_state_t;

typedef struct executor_tracking_state {
    // Current state machine position
    executor_state_t state;

    // Pass being executed
    pass_info_t current_pass;

    // Satellite SGP4 state (copied from TLE database at pass start)
    sgp4_state_t sat_state;
    bool sat_state_valid;

    // Satellite name for display
    char sat_name[SGP4_TLE_NAME_LEN];

    // Last commanded rotator position (radians)
    double last_cmd_azimuth_rad;
    double last_cmd_elevation_rad;
    bool rotator_commanded;

    // Current calculated satellite position (radians)
    double current_azimuth_rad;
    double current_elevation_rad;
    double current_range_km;

    // Doppler tracking state
    double last_doppler_factor;    // Last applied doppler factor (ratio)
    bool doppler_valid;

    // Velocity components for doppler calculation (km/s)
    double range_rate_km_s;        // Radial velocity toward/away from observer
} executor_tracking_state_t;

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

// ============================================================================
// Task Priorities and Stack Sizes
// ============================================================================

#define PRIORITY_PASS_EXECUTOR  10
#define PRIORITY_CONTROLLER     20
#define PRIORITY_ROTATOR_CMD    25
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
// Extern Declarations for IPC Objects
// ============================================================================

// Message queues
extern rtems_id g_gps_queue;
extern rtems_id g_antenna_queue;
extern rtems_id g_tle_queue;
extern rtems_id g_pass_queue;
extern rtems_id g_radio_queue;
extern rtems_id g_ctrl_cmd_queue;
extern rtems_id g_executor_cmd_queue;
extern rtems_id g_rotator_cmd_queue;

// Semaphores (mutexes)
extern rtems_id g_uart1_mutex;
extern rtems_id g_uart3_mutex;
extern rtems_id g_state_mutex;
extern rtems_id g_tle_database_mutex;
extern rtems_id g_pass_queue_mutex;

// Shared file descriptors
extern int g_rotator_fd;
extern int g_radio_fd;

// Global state
extern controller_state_t g_state;
extern executor_tracking_state_t g_executor_state;
extern pass_priority_queue_t g_upcoming_passes;
extern scheduled_pass_t g_scheduled_pass;

// ============================================================================
// Network Utility Functions (defined in controller.c)
// ============================================================================

/*
 * Get current IP addresses for an interface.
 * Fills ipv4_addr and ipv6_addr buffers with addresses (empty string if none).
 */
void network_get_addresses(const char *ifname,
                           char *ipv4_addr, size_t ipv4_size,
                           char *ipv6_addr, size_t ipv6_size);

#endif /* SHARED_H */
