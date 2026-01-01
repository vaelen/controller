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

#define CONFIG_PATH_MAX         64
#define CONFIG_LINE_MAX         128
#define CONFIG_KEY_MAX          32
#define CONFIG_VALUE_MAX        64
#define CONFIG_URL_MAX          256
#define CONFIG_SATELLITES_MAX   512
#define CONFIG_MAX_NORAD_IDS    64

#define CONFIG_DEFAULT_PATH "/mnt/sd/config.ini"
#define CONFIG_SD_MOUNT     "/mnt/sd"

#define CONFIG_IPV4_ADDR_MAX    16   /* "255.255.255.255\0" */
#define CONFIG_IPV6_ADDR_MAX    46   /* Full IPv6 address with scope */
#define CONFIG_IFACE_NAME_MAX   16   /* Interface name e.g., "cpsw0" */

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
 * IPv4 network configuration.
 */
typedef struct ipv4_config {
    bool enabled;
    bool dhcp;
    char address[CONFIG_IPV4_ADDR_MAX];
    char netmask[CONFIG_IPV4_ADDR_MAX];
    char gateway[CONFIG_IPV4_ADDR_MAX];
} ipv4_config_t;

/*
 * IPv6 network configuration.
 */
typedef struct ipv6_config {
    bool enabled;
    bool dhcp;       /* DHCPv6 */
    bool slaac;      /* Stateless Address Autoconfiguration */
    char address[CONFIG_IPV6_ADDR_MAX];
    int prefix_len;
    char gateway[CONFIG_IPV6_ADDR_MAX];
} ipv6_config_t;

/*
 * Complete network configuration.
 */
typedef struct network_config {
    bool enabled;
    char interface[CONFIG_IFACE_NAME_MAX];
    ipv4_config_t ipv4;
    ipv6_config_t ipv6;
} network_config_t;

/*
 * Pass prediction and scheduling configuration.
 */
typedef struct pass_config {
    int prediction_window_min;     /* How far ahead to predict (default: 60) */
    double min_elevation_deg;      /* Minimum elevation for pass detection (default: 5.0) */
    double min_schedule_elevation_deg; /* Min max-elevation to schedule pass (default: 15.0) */
    int prep_time_sec;             /* Time before AOS to start prep (default: 300) */
    int calc_interval_sec;         /* How often to recalculate (default: 300) */
    int status_display_count;      /* Number of passes to show in status (default: 5) */

    /* Pass executor / tracking configuration */
    int tracking_poll_ms;          /* Position update interval in ms (default: 100) */
    double rotator_threshold_deg;  /* Min change to command rotator (default: 1.0) */
    double doppler_threshold_khz;  /* Min change to update radio freq (default: 1.0) */
    int preposition_margin_sec;    /* Time before AOS to finish preposition (default: 30) */
} pass_config_t;

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

    /* Network configuration */
    network_config_t network;

    /* File paths */
    char tle_path[CONFIG_PATH_MAX];

    /* TLE update configuration */
    char tle_url[CONFIG_URL_MAX];
    char satellites_str[CONFIG_SATELLITES_MAX];
    int satellite_norad_ids[CONFIG_MAX_NORAD_IDS];
    int satellite_count;
    int tle_update_interval_hours;

    /* Pass prediction configuration */
    pass_config_t pass;

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
