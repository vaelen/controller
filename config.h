/*
 * Configuration system for RTEMS satellite tracking controller.
 *
 * Provides INI-style configuration file parsing and runtime reload support.
 * Configuration is stored on SD card at /mnt/sd/config.ini.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <rtems.h>
#include <stdbool.h>
#include <termios.h>

// ============================================================================
// Configuration Constants
// ============================================================================

#define CONFIG_PATH_MAX     64
#define CONFIG_LINE_MAX     128
#define CONFIG_KEY_MAX      32
#define CONFIG_VALUE_MAX    64

#define CONFIG_DEFAULT_PATH "/mnt/sd/config.ini"
#define CONFIG_SD_MOUNT     "/mnt/sd"

// ============================================================================
// Error Codes
// ============================================================================

typedef enum config_error {
    CONFIG_SUCCESS = 0,
    CONFIG_ERROR_NULL_POINTER = -1,
    CONFIG_ERROR_FILE_OPEN = -2,
    CONFIG_ERROR_FILE_READ = -3,
    CONFIG_ERROR_FILE_WRITE = -4,
    CONFIG_ERROR_PARSE = -5,
    CONFIG_ERROR_INVALID_VALUE = -6,
    CONFIG_ERROR_NOT_INITIALIZED = -7
} config_error_t;

// ============================================================================
// Configuration Structures
// ============================================================================

/*
 * Serial port configuration.
 */
typedef struct serial_config {
    char device_path[CONFIG_PATH_MAX];
    speed_t baud_rate;
    bool flow_control;
} serial_config_t;

/*
 * Observer location configuration.
 */
typedef struct observer_config {
    double latitude_deg;    /* Degrees, + = North, - = South */
    double longitude_deg;   /* Degrees, + = East, - = West */
    double altitude_m;      /* Meters above sea level */
    bool is_set;            /* True if explicitly configured */
} observer_config_t;

/*
 * Complete configuration structure.
 */
typedef struct config {
    /* Serial ports */
    serial_config_t gps;
    serial_config_t rotator;
    serial_config_t radio;

    /* Observer location */
    observer_config_t observer;

    /* File paths */
    char tle_path[CONFIG_PATH_MAX];

    /* System settings */
    int log_level;              /* LOG_LEVEL_DEBUG through LOG_LEVEL_ERROR */
    int status_interval_sec;    /* Seconds between status reports */

    /* Metadata */
    bool loaded;                /* True if config was loaded from file */
    char source_path[CONFIG_PATH_MAX];
} config_t;

// ============================================================================
// Configuration Functions
// ============================================================================

/*
 * Initialize configuration to default values.
 * Must be called before load or use.
 *
 * @param cfg  Configuration structure to initialize
 */
void config_init_defaults(config_t *cfg);

/*
 * Load configuration from INI file.
 * Missing values retain their defaults from config_init_defaults().
 *
 * @param cfg   Configuration structure to populate
 * @param path  Path to INI file (NULL = use CONFIG_DEFAULT_PATH)
 * @return      CONFIG_SUCCESS or error code
 */
config_error_t config_load(config_t *cfg, const char *path);

/*
 * Save configuration to INI file.
 *
 * @param cfg   Configuration to save
 * @param path  Path to INI file (NULL = use cfg->source_path or default)
 * @return      CONFIG_SUCCESS or error code
 */
config_error_t config_save(const config_t *cfg, const char *path);

/*
 * Initialize the configuration subsystem.
 * Creates the mutex, loads initial config from file.
 * Must be called from Init task before other tasks start.
 *
 * @param path  Path to config file (NULL = use default)
 * @return      RTEMS_SUCCESSFUL or error code
 */
rtems_status_code config_system_init(const char *path);

/*
 * Get a thread-safe copy of the current configuration.
 * Acquires the config mutex briefly to copy the config.
 *
 * @param cfg_out  Destination for configuration copy
 * @return         CONFIG_SUCCESS or error code
 */
config_error_t config_get_copy(config_t *cfg_out);

/*
 * Reload configuration from file at runtime.
 * Acquires the config mutex, reloads from file, releases mutex.
 *
 * @return CONFIG_SUCCESS or error code
 */
config_error_t config_reload(void);

/*
 * Convert baud rate integer to termios speed_t constant.
 * Supported: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
 *
 * @param baud  Baud rate as integer
 * @return      speed_t constant, or B9600 if not recognized
 */
speed_t config_baud_to_speed(int baud);

/*
 * Convert termios speed_t constant to baud rate integer.
 *
 * @param speed  termios speed constant
 * @return       Baud rate as integer
 */
int config_speed_to_baud(speed_t speed);

#endif /* CONFIG_H */
