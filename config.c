/*
 * Configuration system implementation for RTEMS satellite tracking controller.
 *
 * Parses INI-style configuration files with minimal memory allocation.
 * Uses fixed-size buffers and follows existing codebase patterns.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "config.h"
#include "log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ============================================================================
// Module State
// ============================================================================

static config_t g_config;
static rtems_id g_config_mutex;
static bool g_initialized = false;

// ============================================================================
// Internal Helper Functions
// ============================================================================

/*
 * Trim leading and trailing whitespace in-place.
 * Returns pointer to trimmed string (may be offset from input).
 */
static char *trim(char *str)
{
    if (!str) {
        return NULL;
    }

    /* Trim leading whitespace */
    while (isspace((unsigned char)*str)) {
        str++;
    }

    if (*str == '\0') {
        return str;
    }

    /* Trim trailing whitespace */
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) {
        end--;
    }
    end[1] = '\0';

    return str;
}

/*
 * Case-insensitive string comparison.
 */
static int strcasecmp_local(const char *s1, const char *s2)
{
    while (*s1 && *s2) {
        int c1 = tolower((unsigned char)*s1);
        int c2 = tolower((unsigned char)*s2);
        if (c1 != c2) {
            return c1 - c2;
        }
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

/*
 * Parse boolean value from string.
 * Recognizes: true/false, yes/no, 1/0, on/off
 */
static bool parse_bool(const char *value, bool default_val)
{
    if (!value || !*value) {
        return default_val;
    }

    if (strcasecmp_local(value, "true") == 0 ||
        strcasecmp_local(value, "yes") == 0 ||
        strcasecmp_local(value, "1") == 0 ||
        strcasecmp_local(value, "on") == 0) {
        return true;
    }

    if (strcasecmp_local(value, "false") == 0 ||
        strcasecmp_local(value, "no") == 0 ||
        strcasecmp_local(value, "0") == 0 ||
        strcasecmp_local(value, "off") == 0) {
        return false;
    }

    return default_val;
}

// ============================================================================
// Baud Rate Conversion
// ============================================================================

speed_t config_baud_to_speed(int baud)
{
    switch (baud) {
        case 1200:   return B1200;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:
            LOG_WARN("CONFIG", "Unknown baud rate %d, using 9600", baud);
            return B9600;
    }
}

int config_speed_to_baud(speed_t speed)
{
    switch (speed) {
        case B1200:   return 1200;
        case B2400:   return 2400;
        case B4800:   return 4800;
        case B9600:   return 9600;
        case B19200:  return 19200;
        case B38400:  return 38400;
        case B57600:  return 57600;
        case B115200: return 115200;
        default:      return 9600;
    }
}

// ============================================================================
// Section Parsing
// ============================================================================

/* Section identifiers */
typedef enum {
    SECTION_NONE = 0,
    SECTION_SERIAL,
    SECTION_OBSERVER,
    SECTION_FILES,
    SECTION_SYSTEM,
    SECTION_NETWORK
} config_section_t;

/*
 * Identify section from header string.
 */
static config_section_t identify_section(const char *name)
{
    if (strcasecmp_local(name, "serial") == 0) {
        return SECTION_SERIAL;
    }
    if (strcasecmp_local(name, "observer") == 0) {
        return SECTION_OBSERVER;
    }
    if (strcasecmp_local(name, "files") == 0) {
        return SECTION_FILES;
    }
    if (strcasecmp_local(name, "system") == 0) {
        return SECTION_SYSTEM;
    }
    if (strcasecmp_local(name, "network") == 0) {
        return SECTION_NETWORK;
    }
    return SECTION_NONE;
}

/*
 * Process a key=value pair for the [serial] section.
 */
static void process_serial_key(config_t *cfg, const char *key, const char *value)
{
    if (strcasecmp_local(key, "gps_device") == 0) {
        strncpy(cfg->gps.device_path, value, CONFIG_PATH_MAX - 1);
        cfg->gps.device_path[CONFIG_PATH_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "gps_baud") == 0) {
        cfg->gps.baud_rate = config_baud_to_speed(atoi(value));
    } else if (strcasecmp_local(key, "gps_flow_control") == 0) {
        cfg->gps.flow_control = parse_bool(value, false);
    } else if (strcasecmp_local(key, "rotator_device") == 0) {
        strncpy(cfg->rotator.device_path, value, CONFIG_PATH_MAX - 1);
        cfg->rotator.device_path[CONFIG_PATH_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "rotator_baud") == 0) {
        cfg->rotator.baud_rate = config_baud_to_speed(atoi(value));
    } else if (strcasecmp_local(key, "rotator_flow_control") == 0) {
        cfg->rotator.flow_control = parse_bool(value, false);
    } else if (strcasecmp_local(key, "radio_device") == 0) {
        strncpy(cfg->radio.device_path, value, CONFIG_PATH_MAX - 1);
        cfg->radio.device_path[CONFIG_PATH_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "radio_baud") == 0) {
        cfg->radio.baud_rate = config_baud_to_speed(atoi(value));
    } else if (strcasecmp_local(key, "radio_flow_control") == 0) {
        cfg->radio.flow_control = parse_bool(value, false);
    }
}

/*
 * Process a key=value pair for the [observer] section.
 */
static void process_observer_key(config_t *cfg, const char *key, const char *value)
{
    if (strcasecmp_local(key, "latitude") == 0) {
        cfg->observer.latitude_deg = atof(value);
        cfg->observer.is_set = true;
    } else if (strcasecmp_local(key, "longitude") == 0) {
        cfg->observer.longitude_deg = atof(value);
        cfg->observer.is_set = true;
    } else if (strcasecmp_local(key, "altitude") == 0) {
        cfg->observer.altitude_m = atof(value);
    }
}

/*
 * Process a key=value pair for the [files] section.
 */
static void process_files_key(config_t *cfg, const char *key, const char *value)
{
    if (strcasecmp_local(key, "tle_path") == 0) {
        strncpy(cfg->tle_path, value, CONFIG_PATH_MAX - 1);
        cfg->tle_path[CONFIG_PATH_MAX - 1] = '\0';
    }
}

/*
 * Process a key=value pair for the [system] section.
 */
static void process_system_key(config_t *cfg, const char *key, const char *value)
{
    if (strcasecmp_local(key, "log_level") == 0) {
        if (strcasecmp_local(value, "DEBUG") == 0) {
            cfg->log_level = LOG_LEVEL_DEBUG;
        } else if (strcasecmp_local(value, "INFO") == 0) {
            cfg->log_level = LOG_LEVEL_INFO;
        } else if (strcasecmp_local(value, "WARN") == 0) {
            cfg->log_level = LOG_LEVEL_WARN;
        } else if (strcasecmp_local(value, "ERROR") == 0) {
            cfg->log_level = LOG_LEVEL_ERROR;
        }
    } else if (strcasecmp_local(key, "status_interval") == 0) {
        cfg->status_interval_sec = atoi(value);
        if (cfg->status_interval_sec < 1) {
            cfg->status_interval_sec = 30;
        }
    }
}

/*
 * Process a key=value pair for the [network] section.
 */
static void process_network_key(config_t *cfg, const char *key, const char *value)
{
    if (strcasecmp_local(key, "enabled") == 0) {
        cfg->network.enabled = parse_bool(value, true);
    } else if (strcasecmp_local(key, "interface") == 0) {
        strncpy(cfg->network.interface, value, CONFIG_IFACE_NAME_MAX - 1);
        cfg->network.interface[CONFIG_IFACE_NAME_MAX - 1] = '\0';
    }
    /* IPv4 settings */
    else if (strcasecmp_local(key, "ipv4_enabled") == 0) {
        cfg->network.ipv4.enabled = parse_bool(value, true);
    } else if (strcasecmp_local(key, "ipv4_dhcp") == 0) {
        cfg->network.ipv4.dhcp = parse_bool(value, true);
    } else if (strcasecmp_local(key, "ipv4_address") == 0) {
        strncpy(cfg->network.ipv4.address, value, CONFIG_IPV4_ADDR_MAX - 1);
        cfg->network.ipv4.address[CONFIG_IPV4_ADDR_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "ipv4_netmask") == 0) {
        strncpy(cfg->network.ipv4.netmask, value, CONFIG_IPV4_ADDR_MAX - 1);
        cfg->network.ipv4.netmask[CONFIG_IPV4_ADDR_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "ipv4_gateway") == 0) {
        strncpy(cfg->network.ipv4.gateway, value, CONFIG_IPV4_ADDR_MAX - 1);
        cfg->network.ipv4.gateway[CONFIG_IPV4_ADDR_MAX - 1] = '\0';
    }
    /* IPv6 settings */
    else if (strcasecmp_local(key, "ipv6_enabled") == 0) {
        cfg->network.ipv6.enabled = parse_bool(value, true);
    } else if (strcasecmp_local(key, "ipv6_dhcp") == 0) {
        cfg->network.ipv6.dhcp = parse_bool(value, true);
    } else if (strcasecmp_local(key, "ipv6_slaac") == 0) {
        cfg->network.ipv6.slaac = parse_bool(value, true);
    } else if (strcasecmp_local(key, "ipv6_address") == 0) {
        strncpy(cfg->network.ipv6.address, value, CONFIG_IPV6_ADDR_MAX - 1);
        cfg->network.ipv6.address[CONFIG_IPV6_ADDR_MAX - 1] = '\0';
    } else if (strcasecmp_local(key, "ipv6_prefix_len") == 0) {
        cfg->network.ipv6.prefix_len = atoi(value);
        if (cfg->network.ipv6.prefix_len < 1 || cfg->network.ipv6.prefix_len > 128) {
            cfg->network.ipv6.prefix_len = 64;
        }
    } else if (strcasecmp_local(key, "ipv6_gateway") == 0) {
        strncpy(cfg->network.ipv6.gateway, value, CONFIG_IPV6_ADDR_MAX - 1);
        cfg->network.ipv6.gateway[CONFIG_IPV6_ADDR_MAX - 1] = '\0';
    }
}

// ============================================================================
// Public Functions
// ============================================================================

void config_init_defaults(config_t *cfg)
{
    if (!cfg) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    /* GPS defaults */
    strncpy(cfg->gps.device_path, "/dev/ttyS1", CONFIG_PATH_MAX - 1);
    cfg->gps.baud_rate = B9600;
    cfg->gps.flow_control = false;

    /* Rotator defaults */
    strncpy(cfg->rotator.device_path, "/dev/ttyS2", CONFIG_PATH_MAX - 1);
    cfg->rotator.baud_rate = B9600;
    cfg->rotator.flow_control = false;

    /* Radio defaults */
    strncpy(cfg->radio.device_path, "/dev/ttyS3", CONFIG_PATH_MAX - 1);
    cfg->radio.baud_rate = B38400;
    cfg->radio.flow_control = false;

    /* Observer - not set by default (use GPS location) */
    cfg->observer.latitude_deg = 0.0;
    cfg->observer.longitude_deg = 0.0;
    cfg->observer.altitude_m = 0.0;
    cfg->observer.is_set = false;

    /* File paths */
    strncpy(cfg->tle_path, "/mnt/sd/tle.txt", CONFIG_PATH_MAX - 1);

    /* System settings */
    cfg->log_level = LOG_LEVEL_INFO;
    cfg->status_interval_sec = 30;

    /* Network defaults */
    cfg->network.enabled = true;
    strncpy(cfg->network.interface, "cpsw0", CONFIG_IFACE_NAME_MAX - 1);
    cfg->network.interface[CONFIG_IFACE_NAME_MAX - 1] = '\0';
    cfg->network.ipv4.enabled = true;
    cfg->network.ipv4.dhcp = true;
    cfg->network.ipv4.address[0] = '\0';
    cfg->network.ipv4.netmask[0] = '\0';
    cfg->network.ipv4.gateway[0] = '\0';
    cfg->network.ipv6.enabled = true;
    cfg->network.ipv6.dhcp = true;
    cfg->network.ipv6.slaac = true;
    cfg->network.ipv6.address[0] = '\0';
    cfg->network.ipv6.prefix_len = 64;
    cfg->network.ipv6.gateway[0] = '\0';

    cfg->loaded = false;
    cfg->source_path[0] = '\0';
}

config_error_t config_load(config_t *cfg, const char *path)
{
    if (!cfg) {
        return CONFIG_ERROR_NULL_POINTER;
    }

    const char *file_path = path ? path : CONFIG_DEFAULT_PATH;

    FILE *f = fopen(file_path, "r");
    if (!f) {
        LOG_WARN("CONFIG", "Cannot open config file: %s", file_path);
        return CONFIG_ERROR_FILE_OPEN;
    }

    LOG_INFO("CONFIG", "Loading configuration from %s", file_path);

    char line[CONFIG_LINE_MAX];
    config_section_t current_section = SECTION_NONE;
    int line_num = 0;

    while (fgets(line, sizeof(line), f)) {
        line_num++;

        /* Trim the line */
        char *trimmed = trim(line);

        /* Skip empty lines and comments */
        if (*trimmed == '\0' || *trimmed == ';' || *trimmed == '#') {
            continue;
        }

        /* Check for section header */
        if (*trimmed == '[') {
            char *end = strchr(trimmed, ']');
            if (end) {
                *end = '\0';
                current_section = identify_section(trimmed + 1);
                LOG_DEBUG("CONFIG", "Section: [%s]", trimmed + 1);
            }
            continue;
        }

        /* Parse key=value */
        char *equals = strchr(trimmed, '=');
        if (!equals) {
            LOG_WARN("CONFIG", "Line %d: Invalid format (no '=')", line_num);
            continue;
        }

        *equals = '\0';
        char *key = trim(trimmed);
        char *value = trim(equals + 1);

        /* Dispatch to section handler */
        switch (current_section) {
            case SECTION_SERIAL:
                process_serial_key(cfg, key, value);
                break;
            case SECTION_OBSERVER:
                process_observer_key(cfg, key, value);
                break;
            case SECTION_FILES:
                process_files_key(cfg, key, value);
                break;
            case SECTION_SYSTEM:
                process_system_key(cfg, key, value);
                break;
            case SECTION_NETWORK:
                process_network_key(cfg, key, value);
                break;
            default:
                LOG_DEBUG("CONFIG", "Line %d: Key '%s' outside section",
                          line_num, key);
                break;
        }
    }

    fclose(f);

    cfg->loaded = true;
    strncpy(cfg->source_path, file_path, CONFIG_PATH_MAX - 1);
    cfg->source_path[CONFIG_PATH_MAX - 1] = '\0';

    LOG_INFO("CONFIG", "Configuration loaded successfully");
    return CONFIG_SUCCESS;
}

config_error_t config_save(const config_t *cfg, const char *path)
{
    if (!cfg) {
        return CONFIG_ERROR_NULL_POINTER;
    }

    const char *file_path = path ? path :
        (cfg->source_path[0] ? cfg->source_path : CONFIG_DEFAULT_PATH);

    FILE *f = fopen(file_path, "w");
    if (!f) {
        LOG_ERROR("CONFIG", "Cannot open config file for writing: %s", file_path);
        return CONFIG_ERROR_FILE_OPEN;
    }

    LOG_INFO("CONFIG", "Saving configuration to %s", file_path);

    /* Write header */
    fprintf(f, "; Satellite Tracking Controller Configuration\n");
    fprintf(f, "; Auto-generated - manual edits may be overwritten\n\n");

    /* [serial] section */
    fprintf(f, "[serial]\n");
    fprintf(f, "gps_device = %s\n", cfg->gps.device_path);
    fprintf(f, "gps_baud = %d\n", config_speed_to_baud(cfg->gps.baud_rate));
    fprintf(f, "gps_flow_control = %s\n", cfg->gps.flow_control ? "true" : "false");
    fprintf(f, "rotator_device = %s\n", cfg->rotator.device_path);
    fprintf(f, "rotator_baud = %d\n", config_speed_to_baud(cfg->rotator.baud_rate));
    fprintf(f, "rotator_flow_control = %s\n", cfg->rotator.flow_control ? "true" : "false");
    fprintf(f, "radio_device = %s\n", cfg->radio.device_path);
    fprintf(f, "radio_baud = %d\n", config_speed_to_baud(cfg->radio.baud_rate));
    fprintf(f, "radio_flow_control = %s\n\n", cfg->radio.flow_control ? "true" : "false");

    /* [observer] section */
    fprintf(f, "[observer]\n");
    fprintf(f, "latitude = %.6f\n", cfg->observer.latitude_deg);
    fprintf(f, "longitude = %.6f\n", cfg->observer.longitude_deg);
    fprintf(f, "altitude = %.1f\n\n", cfg->observer.altitude_m);

    /* [files] section */
    fprintf(f, "[files]\n");
    fprintf(f, "tle_path = %s\n\n", cfg->tle_path);

    /* [system] section */
    fprintf(f, "[system]\n");
    const char *level_str = "INFO";
    switch (cfg->log_level) {
        case LOG_LEVEL_DEBUG: level_str = "DEBUG"; break;
        case LOG_LEVEL_INFO:  level_str = "INFO";  break;
        case LOG_LEVEL_WARN:  level_str = "WARN";  break;
        case LOG_LEVEL_ERROR: level_str = "ERROR"; break;
    }
    fprintf(f, "log_level = %s\n", level_str);
    fprintf(f, "status_interval = %d\n\n", cfg->status_interval_sec);

    /* [network] section */
    fprintf(f, "[network]\n");
    fprintf(f, "enabled = %s\n", cfg->network.enabled ? "true" : "false");
    fprintf(f, "interface = %s\n", cfg->network.interface);
    fprintf(f, "ipv4_enabled = %s\n", cfg->network.ipv4.enabled ? "true" : "false");
    fprintf(f, "ipv4_dhcp = %s\n", cfg->network.ipv4.dhcp ? "true" : "false");
    fprintf(f, "ipv4_address = %s\n", cfg->network.ipv4.address);
    fprintf(f, "ipv4_netmask = %s\n", cfg->network.ipv4.netmask);
    fprintf(f, "ipv4_gateway = %s\n", cfg->network.ipv4.gateway);
    fprintf(f, "ipv6_enabled = %s\n", cfg->network.ipv6.enabled ? "true" : "false");
    fprintf(f, "ipv6_dhcp = %s\n", cfg->network.ipv6.dhcp ? "true" : "false");
    fprintf(f, "ipv6_slaac = %s\n", cfg->network.ipv6.slaac ? "true" : "false");
    fprintf(f, "ipv6_address = %s\n", cfg->network.ipv6.address);
    fprintf(f, "ipv6_prefix_len = %d\n", cfg->network.ipv6.prefix_len);
    fprintf(f, "ipv6_gateway = %s\n", cfg->network.ipv6.gateway);

    fclose(f);

    LOG_INFO("CONFIG", "Configuration saved successfully");
    return CONFIG_SUCCESS;
}

rtems_status_code config_system_init(const char *path)
{
    rtems_status_code status;

    /* Create config mutex */
    status = rtems_semaphore_create(
        rtems_build_name('C', 'O', 'N', 'F'),
        1,
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_config_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        LOG_ERROR("CONFIG", "Failed to create config mutex: %d", status);
        return status;
    }

    /* Initialize defaults */
    config_init_defaults(&g_config);

    /* Attempt to load config (non-fatal if file missing) */
    config_error_t err = config_load(&g_config, path);
    if (err == CONFIG_ERROR_FILE_OPEN) {
        LOG_WARN("CONFIG", "Config file not found, using defaults");
        /* Save default configuration to SD card for future editing */
        const char *save_path = path ? path : CONFIG_DEFAULT_PATH;
        config_error_t save_err = config_save(&g_config, save_path);
        if (save_err == CONFIG_SUCCESS) {
            LOG_INFO("CONFIG", "Default configuration saved to %s", save_path);
            strncpy(g_config.source_path, save_path, CONFIG_PATH_MAX - 1);
            g_config.source_path[CONFIG_PATH_MAX - 1] = '\0';
        } else {
            LOG_WARN("CONFIG", "Could not save default config: %d", save_err);
        }
    } else if (err != CONFIG_SUCCESS) {
        LOG_WARN("CONFIG", "Config load error %d, using defaults", err);
    }

    /* Apply log level from config */
    log_set_level(g_config.log_level);

    g_initialized = true;
    return RTEMS_SUCCESSFUL;
}

config_error_t config_get_copy(config_t *cfg_out)
{
    if (!cfg_out) {
        return CONFIG_ERROR_NULL_POINTER;
    }

    if (!g_initialized) {
        /* Not initialized yet, return defaults */
        config_init_defaults(cfg_out);
        return CONFIG_SUCCESS;
    }

    rtems_semaphore_obtain(g_config_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    *cfg_out = g_config;
    rtems_semaphore_release(g_config_mutex);

    return CONFIG_SUCCESS;
}

config_error_t config_reload(void)
{
    if (!g_initialized) {
        return CONFIG_ERROR_NOT_INITIALIZED;
    }

    LOG_INFO("CONFIG", "Reloading configuration...");

    /* Load into temporary config first */
    config_t new_config;
    config_init_defaults(&new_config);

    const char *path = g_config.source_path[0] ? g_config.source_path : NULL;
    config_error_t err = config_load(&new_config, path);

    if (err != CONFIG_SUCCESS) {
        LOG_ERROR("CONFIG", "Reload failed: %d", err);
        return err;
    }

    /* Update global config atomically */
    rtems_semaphore_obtain(g_config_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    g_config = new_config;
    rtems_semaphore_release(g_config_mutex);

    /* Apply new log level */
    log_set_level(g_config.log_level);

    /* TODO: Notify other components of config reload */

    LOG_INFO("CONFIG", "Configuration reloaded successfully");
    return CONFIG_SUCCESS;
}
