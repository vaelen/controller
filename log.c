/*
 * Logging utilities for RTEMS satellite tracking controller.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "log.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

// ============================================================================
// Module State
// ============================================================================

static log_level_t g_log_level = LOG_LEVEL_INFO;
static rtems_id g_log_mutex;
static bool g_clock_valid = false;

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
    struct tm *tm = gmtime(&utc_time);
    if (tm == NULL) {
        return;
    }

    int year = tm->tm_year + 1900;
    if (year < 2020 || year > 2100) {
        return;
    }

    rtems_time_of_day tod;
    tod.year   = (uint32_t)year;
    tod.month  = (uint32_t)(tm->tm_mon + 1);
    tod.day    = (uint32_t)tm->tm_mday;
    tod.hour   = (uint32_t)tm->tm_hour;
    tod.minute = (uint32_t)tm->tm_min;
    tod.second = (uint32_t)tm->tm_sec;
    tod.ticks  = 0;

    if (rtems_clock_set(&tod) == RTEMS_SUCCESSFUL) {
        g_clock_valid = true;
    }
}

bool log_clock_is_valid(void) {
    return g_clock_valid;
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

    /* Format timestamp (HH:MM:SS if clock valid, or "??:??:??" if not) */
    char time_str[12];
    if (g_clock_valid) {
        rtems_time_of_day tod;
        if (rtems_clock_get_tod(&tod) == RTEMS_SUCCESSFUL) {
            snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u",
                     (unsigned)tod.hour, (unsigned)tod.minute, (unsigned)tod.second);
        } else {
            strcpy(time_str, "??:??:??");
        }
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
