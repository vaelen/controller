/*
 * Logging utilities for RTEMS satellite tracking controller.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef LOG_H
#define LOG_H

#include <rtems.h>
#include <time.h>

// ============================================================================
// Logging Configuration
// ============================================================================

#define LOG_MSG_MAX_LEN     128

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_ERROR = 3
} log_level_t;

// ============================================================================
// Logging Functions
// ============================================================================

/*
 * Initialize the logging subsystem.
 *
 * Creates the mutex used to protect printf from concurrent access.
 * Must be called before any other logging functions.
 *
 * @return RTEMS_SUCCESSFUL on success, error code on failure
 */
rtems_status_code log_init(void);

/*
 * Set the current time for log timestamps.
 *
 * Call this whenever the system time is updated (e.g., from GPS).
 * If never called, log timestamps will show "??:??:??".
 *
 * @param utc_time  Current UTC time
 */
void log_set_time(time_t utc_time);

/*
 * Set the minimum log level.
 *
 * Messages below this level will not be printed.
 *
 * @param level  Minimum log level (DEBUG, INFO, WARN, ERROR)
 */
void log_set_level(log_level_t level);

/*
 * Log a formatted message to the console.
 *
 * Uses a semaphore to protect printf from concurrent access.
 * If the log level is below the minimum, the message is not printed.
 *
 * @param level   Log level (DEBUG, INFO, WARN, ERROR)
 * @param tag     Task identifier (e.g., "GPS", "CTRL")
 * @param fmt     printf-style format string
 * @param ...     Format arguments
 */
void log_write(log_level_t level, const char *tag, const char *fmt, ...);

// ============================================================================
// Logging Macros
// ============================================================================

#define LOG_DEBUG(tag, fmt, ...)  log_write(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOG_INFO(tag, fmt, ...)   log_write(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOG_WARN(tag, fmt, ...)   log_write(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOG_ERROR(tag, fmt, ...)  log_write(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)

#endif /* LOG_H */
