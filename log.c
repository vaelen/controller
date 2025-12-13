/*
 * Logging utilities for RTEMS satellite tracking controller.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "log.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// ============================================================================
// Module State
// ============================================================================

static log_level_t g_log_level = LOG_LEVEL_INFO;
static rtems_id g_log_mutex;
static time_t g_log_time = 0;

// ============================================================================
// Logging Functions
// ============================================================================

rtems_status_code log_init(void) {
    rtems_status_code status;

    status = rtems_semaphore_create(
        rtems_build_name('L', 'O', 'G', 'M'),
        1,  /* initially available */
        RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
        0,
        &g_log_mutex
    );
    if (status != RTEMS_SUCCESSFUL) {
        printf("FATAL: Failed to create log mutex: %d\n", status);
        return status;
    }

    LOG_INFO("LOG", "Logging initialized");
    return RTEMS_SUCCESSFUL;
}

void log_set_time(time_t utc_time) {
    g_log_time = utc_time;
}

void log_set_level(log_level_t level) {
    g_log_level = level;
}

void log_write(log_level_t level, const char *tag, const char *fmt, ...) {
    /* Early exit if below threshold */
    if (level < g_log_level) {
        return;
    }

    /* Get level string */
    const char *level_str;
    switch (level) {
        case LOG_LEVEL_DEBUG: level_str = "DEBUG"; break;
        case LOG_LEVEL_INFO:  level_str = "INFO";  break;
        case LOG_LEVEL_WARN:  level_str = "WARN";  break;
        case LOG_LEVEL_ERROR: level_str = "ERROR"; break;
        default:              level_str = "?";     break;
    }

    /* Format timestamp (HH:MM:SS if valid, or "??:??:??" if not) */
    char time_str[12];
    if (g_log_time > 0) {
        struct tm *tm = gmtime(&g_log_time);
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 tm->tm_hour, tm->tm_min, tm->tm_sec);
    } else {
        strcpy(time_str, "??:??:??");
    }

    /* Format user message to fixed buffer */
    char message[LOG_MSG_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    /* Acquire semaphore and print */
    rtems_semaphore_obtain(g_log_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    printf("[%s] %-5s [%-7s] %s\n", time_str, level_str, tag, message);
    rtems_semaphore_release(g_log_mutex);
}
