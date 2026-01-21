/*
 * DS3231MZ RTC Driver Implementation
 *
 * Communicates with the DS3231MZ I2C RTC chip via RTEMS libbsd.
 * Provides time read/write, GPS synchronization, and alarm functionality.
 *
 * NOTE: This driver requires I2C support in libbsd which is not included
 * by default. When RTC_I2C_AVAILABLE is not defined, stub implementations
 * are provided that return RTC_ERROR_I2C_OPEN.
 *
 * To enable I2C support:
 * 1. Rebuild libbsd with I2C drivers enabled
 * 2. Define RTC_I2C_AVAILABLE when compiling this file
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include "rtc.h"
#include "log.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>

#define LOG_TAG "RTC"

// ============================================================================
// Module State
// ============================================================================

typedef struct rtc_state {
    bool initialized;
    int i2c_fd;
    bool oscillator_was_stopped;
    time_t last_sync_time;
    rtc_time_source_t time_source;
} rtc_state_t;

static rtc_state_t g_rtc = {0};

// ============================================================================
// BCD Conversion Helpers
// ============================================================================

static inline uint8_t bcd_to_bin(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static inline uint8_t bin_to_bcd(uint8_t bin) {
    return ((bin / 10) << 4) | (bin % 10);
}

// ============================================================================
// I2C Support Check
// ============================================================================

/*
 * Check if I2C support is available at compile time.
 * To enable, rebuild libbsd with I2C support and define RTC_I2C_AVAILABLE.
 */
#ifdef RTC_I2C_AVAILABLE

#include <dev/iicbus/iic.h>

// ============================================================================
// Low-Level I2C Functions (with I2C support)
// ============================================================================

rtc_error_t rtc_read_register(uint8_t reg, uint8_t *value_out) {
    return rtc_read_registers(reg, value_out, 1);
}

rtc_error_t rtc_write_register(uint8_t reg, uint8_t value) {
    return rtc_write_registers(reg, &value, 1);
}

rtc_error_t rtc_read_registers(uint8_t start_reg, uint8_t *data, size_t len) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }
    if (data == NULL) {
        return RTC_ERROR_NULL_POINTER;
    }

    struct iic_msg msgs[2];
    struct iic_rdwr_data rdwr;

    /* Message 1: Write register address */
    msgs[0].slave = RTC_I2C_ADDRESS << 1;
    msgs[0].flags = IIC_M_WR;
    msgs[0].len = 1;
    msgs[0].buf = &start_reg;

    /* Message 2: Read data */
    msgs[1].slave = RTC_I2C_ADDRESS << 1;
    msgs[1].flags = IIC_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;

    rdwr.msgs = msgs;
    rdwr.nmsgs = 2;

    if (ioctl(g_rtc.i2c_fd, I2CRDWR, &rdwr) < 0) {
        LOG_ERROR(LOG_TAG, "I2C read failed at reg 0x%02X: %s", start_reg, strerror(errno));
        return RTC_ERROR_I2C_READ;
    }

    return RTC_SUCCESS;
}

rtc_error_t rtc_write_registers(uint8_t start_reg, const uint8_t *data, size_t len) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }
    if (data == NULL && len > 0) {
        return RTC_ERROR_NULL_POINTER;
    }

    /* Build write buffer: register address + data */
    uint8_t buf[17];  /* Max 16 bytes data + 1 address */
    if (len + 1 > sizeof(buf)) {
        LOG_ERROR(LOG_TAG, "Write too large: %zu bytes", len);
        return RTC_ERROR_I2C_WRITE;
    }
    buf[0] = start_reg;
    if (len > 0) {
        memcpy(&buf[1], data, len);
    }

    struct iic_msg msg;
    struct iic_rdwr_data rdwr;

    msg.slave = RTC_I2C_ADDRESS << 1;
    msg.flags = IIC_M_WR;
    msg.len = len + 1;
    msg.buf = buf;

    rdwr.msgs = &msg;
    rdwr.nmsgs = 1;

    if (ioctl(g_rtc.i2c_fd, I2CRDWR, &rdwr) < 0) {
        LOG_ERROR(LOG_TAG, "I2C write failed at reg 0x%02X: %s", start_reg, strerror(errno));
        return RTC_ERROR_I2C_WRITE;
    }

    return RTC_SUCCESS;
}

// ============================================================================
// Initialization Functions (with I2C support)
// ============================================================================

rtc_error_t rtc_init(const char *device_path) {
    if (device_path == NULL) {
        return RTC_ERROR_NULL_POINTER;
    }

    if (g_rtc.initialized) {
        LOG_WARN(LOG_TAG, "Already initialized");
        return RTC_SUCCESS;
    }

    g_rtc.i2c_fd = open(device_path, O_RDWR);
    if (g_rtc.i2c_fd < 0) {
        LOG_ERROR(LOG_TAG, "Failed to open I2C device %s: %s", device_path, strerror(errno));
        return RTC_ERROR_I2C_OPEN;
    }

    g_rtc.initialized = true;
    g_rtc.last_sync_time = 0;
    g_rtc.time_source = RTC_TIME_SOURCE_RTC;

    /* Check oscillator status */
    uint8_t status;
    rtc_error_t err = rtc_read_register(RTC_REG_STATUS, &status);
    if (err != RTC_SUCCESS) {
        LOG_ERROR(LOG_TAG, "Failed to read status register");
        close(g_rtc.i2c_fd);
        g_rtc.initialized = false;
        return err;
    }

    g_rtc.oscillator_was_stopped = (status & RTC_STATUS_OSF) != 0;
    if (g_rtc.oscillator_was_stopped) {
        LOG_WARN(LOG_TAG, "Oscillator was stopped - time may be invalid");
    }

    /* Configure control register:
     * - EOSC = 0 (oscillator runs on battery)
     * - INTCN = 1 (INT pin is alarm output, not square wave)
     * - A1IE = 0, A2IE = 0 (alarms disabled until set)
     */
    uint8_t ctrl = RTC_CTRL_INTCN;
    err = rtc_write_register(RTC_REG_CONTROL, ctrl);
    if (err != RTC_SUCCESS) {
        LOG_ERROR(LOG_TAG, "Failed to configure control register");
        close(g_rtc.i2c_fd);
        g_rtc.initialized = false;
        return err;
    }

    /* Read and log initial time */
    time_t initial_time;
    if (rtc_read_time(&initial_time) == RTC_SUCCESS) {
        struct tm *tm = gmtime(&initial_time);
        if (tm != NULL) {
            LOG_INFO(LOG_TAG, "Initialized: %04d-%02d-%02d %02d:%02d:%02d UTC (OSF=%s)",
                     tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                     tm->tm_hour, tm->tm_min, tm->tm_sec,
                     g_rtc.oscillator_was_stopped ? "SET" : "clear");
        }
    }

    return RTC_SUCCESS;
}

#else /* !RTC_I2C_AVAILABLE */

// ============================================================================
// Stub Implementations (without I2C support)
// ============================================================================

rtc_error_t rtc_read_register(uint8_t reg, uint8_t *value_out) {
    (void)reg;
    (void)value_out;
    return RTC_ERROR_I2C_OPEN;
}

rtc_error_t rtc_write_register(uint8_t reg, uint8_t value) {
    (void)reg;
    (void)value;
    return RTC_ERROR_I2C_OPEN;
}

rtc_error_t rtc_read_registers(uint8_t start_reg, uint8_t *data, size_t len) {
    (void)start_reg;
    (void)data;
    (void)len;
    return RTC_ERROR_I2C_OPEN;
}

rtc_error_t rtc_write_registers(uint8_t start_reg, const uint8_t *data, size_t len) {
    (void)start_reg;
    (void)data;
    (void)len;
    return RTC_ERROR_I2C_OPEN;
}

rtc_error_t rtc_init(const char *device_path) {
    (void)device_path;
    LOG_WARN(LOG_TAG, "I2C support not available - RTC disabled");
    LOG_WARN(LOG_TAG, "Rebuild libbsd with I2C and define RTC_I2C_AVAILABLE");
    return RTC_ERROR_I2C_OPEN;
}

#endif /* RTC_I2C_AVAILABLE */

// ============================================================================
// Common Functions (shared between I2C and stub implementations)
// ============================================================================

void rtc_shutdown(void) {
    if (g_rtc.initialized) {
        close(g_rtc.i2c_fd);
        g_rtc.initialized = false;
        LOG_INFO(LOG_TAG, "Shutdown");
    }
}

bool rtc_is_valid(void) {
    return g_rtc.initialized && !g_rtc.oscillator_was_stopped;
}

bool rtc_oscillator_was_stopped(void) {
    return g_rtc.oscillator_was_stopped;
}

// ============================================================================
// Time Functions
// ============================================================================

rtc_error_t rtc_read_time(time_t *time_out) {
    if (time_out == NULL) {
        return RTC_ERROR_NULL_POINTER;
    }
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    uint8_t regs[7];
    rtc_error_t err = rtc_read_registers(RTC_REG_SECONDS, regs, 7);
    if (err != RTC_SUCCESS) {
        return err;
    }

    struct tm tm_time;
    memset(&tm_time, 0, sizeof(tm_time));

    tm_time.tm_sec = bcd_to_bin(regs[0] & 0x7F);
    tm_time.tm_min = bcd_to_bin(regs[1] & 0x7F);

    /* Handle 12/24 hour mode */
    if (regs[2] & 0x40) {
        /* 12-hour mode */
        uint8_t hour = bcd_to_bin(regs[2] & 0x1F);
        if (regs[2] & 0x20) {
            /* PM */
            tm_time.tm_hour = (hour == 12) ? 12 : hour + 12;
        } else {
            /* AM */
            tm_time.tm_hour = (hour == 12) ? 0 : hour;
        }
    } else {
        /* 24-hour mode */
        tm_time.tm_hour = bcd_to_bin(regs[2] & 0x3F);
    }

    tm_time.tm_mday = bcd_to_bin(regs[4] & 0x3F);
    tm_time.tm_mon = bcd_to_bin(regs[5] & 0x1F) - 1;  /* struct tm uses 0-based month */

    uint8_t year = bcd_to_bin(regs[6]);
    /* Century bit is in month register bit 7 */
    if (regs[5] & 0x80) {
        tm_time.tm_year = year + 100;  /* Years since 1900, so 100 = 2000 */
    } else {
        tm_time.tm_year = year;        /* 1900s (unlikely but handled) */
    }

    tm_time.tm_isdst = 0;

    /* Convert to time_t - mktime assumes local time, but RTEMS defaults to UTC */
    *time_out = mktime(&tm_time);
    return RTC_SUCCESS;
}

rtc_error_t rtc_write_time(time_t time) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    struct tm *tm = gmtime(&time);
    if (tm == NULL) {
        LOG_ERROR(LOG_TAG, "gmtime failed for time %ld", (long)time);
        return RTC_ERROR_INVALID_TIME;
    }

    /* Validate year range (2000-2099 for century bit handling) */
    if (tm->tm_year < 100 || tm->tm_year > 199) {
        LOG_ERROR(LOG_TAG, "Year out of range: %d (must be 2000-2099)", tm->tm_year + 1900);
        return RTC_ERROR_INVALID_TIME;
    }

    uint8_t regs[7];
    regs[0] = bin_to_bcd(tm->tm_sec);
    regs[1] = bin_to_bcd(tm->tm_min);
    regs[2] = bin_to_bcd(tm->tm_hour);  /* 24-hour mode (bit 6 = 0) */
    regs[3] = tm->tm_wday + 1;          /* Day of week: DS3231 uses 1-7, struct tm uses 0-6 */
    regs[4] = bin_to_bcd(tm->tm_mday);
    regs[5] = bin_to_bcd(tm->tm_mon + 1);  /* Month: DS3231 uses 1-12, struct tm uses 0-11 */

    /* Set century bit for years >= 2000 */
    if (tm->tm_year >= 100) {
        regs[5] |= 0x80;  /* Century bit */
        regs[6] = bin_to_bcd(tm->tm_year - 100);
    } else {
        regs[6] = bin_to_bcd(tm->tm_year);
    }

    rtc_error_t err = rtc_write_registers(RTC_REG_SECONDS, regs, 7);
    if (err != RTC_SUCCESS) {
        return err;
    }

    /* Clear oscillator stop flag if set */
    uint8_t status;
    err = rtc_read_register(RTC_REG_STATUS, &status);
    if (err == RTC_SUCCESS && (status & RTC_STATUS_OSF)) {
        status &= ~RTC_STATUS_OSF;
        rtc_write_register(RTC_REG_STATUS, status);
        g_rtc.oscillator_was_stopped = false;
        LOG_INFO(LOG_TAG, "Cleared oscillator stop flag");
    }

    LOG_INFO(LOG_TAG, "Time set to %04d-%02d-%02d %02d:%02d:%02d UTC",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);

    return RTC_SUCCESS;
}

rtc_error_t rtc_sync_from_gps(time_t gps_time, int max_drift_sec, bool force) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    time_t rtc_time;
    rtc_error_t err = rtc_read_time(&rtc_time);
    if (err != RTC_SUCCESS && !force) {
        return err;
    }

    /* Calculate drift */
    int drift = (int)(gps_time - rtc_time);
    if (drift < 0) {
        drift = -drift;
    }

    /* Sync if forced, drift exceeds threshold, or oscillator was stopped */
    if (force || drift > max_drift_sec || g_rtc.oscillator_was_stopped) {
        LOG_INFO(LOG_TAG, "Syncing from GPS (drift: %d sec, force: %s, OSF: %s)",
                 drift, force ? "yes" : "no",
                 g_rtc.oscillator_was_stopped ? "yes" : "no");

        err = rtc_write_time(gps_time);
        if (err == RTC_SUCCESS) {
            g_rtc.last_sync_time = gps_time;
            g_rtc.time_source = RTC_TIME_SOURCE_GPS;
        }
        return err;
    }

    LOG_DEBUG(LOG_TAG, "Drift within threshold (%d sec <= %d sec), skipping sync",
              drift, max_drift_sec);
    return RTC_SUCCESS;
}

time_t rtc_get_last_sync_time(void) {
    return g_rtc.last_sync_time;
}

rtc_time_source_t rtc_get_time_source(void) {
    return g_rtc.time_source;
}

// ============================================================================
// Alarm Functions
// ============================================================================

rtc_error_t rtc_set_alarm1(time_t alarm_time) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    struct tm *tm = gmtime(&alarm_time);
    if (tm == NULL) {
        LOG_ERROR(LOG_TAG, "gmtime failed for alarm time %ld", (long)alarm_time);
        return RTC_ERROR_INVALID_TIME;
    }

    /*
     * Configure Alarm 1 for date/hour/minute/second match.
     * A1M4=0, A1M3=0, A1M2=0, A1M1=0, DY/DT=0 (match on date of month)
     */
    uint8_t regs[4];
    regs[0] = bin_to_bcd(tm->tm_sec);           /* A1M1 = 0 (match seconds) */
    regs[1] = bin_to_bcd(tm->tm_min);           /* A1M2 = 0 (match minutes) */
    regs[2] = bin_to_bcd(tm->tm_hour);          /* A1M3 = 0 (match hours, 24hr) */
    regs[3] = bin_to_bcd(tm->tm_mday);          /* A1M4 = 0, DY/DT = 0 (match date) */

    rtc_error_t err = rtc_write_registers(RTC_REG_ALARM1_SECONDS, regs, 4);
    if (err != RTC_SUCCESS) {
        return err;
    }

    /* Clear any existing alarm flag */
    uint8_t status;
    err = rtc_read_register(RTC_REG_STATUS, &status);
    if (err == RTC_SUCCESS && (status & RTC_STATUS_A1F)) {
        status &= ~RTC_STATUS_A1F;
        rtc_write_register(RTC_REG_STATUS, status);
    }

    /* Enable Alarm 1 interrupt */
    uint8_t ctrl;
    err = rtc_read_register(RTC_REG_CONTROL, &ctrl);
    if (err != RTC_SUCCESS) {
        return err;
    }

    ctrl |= RTC_CTRL_A1IE | RTC_CTRL_INTCN;
    err = rtc_write_register(RTC_REG_CONTROL, ctrl);
    if (err != RTC_SUCCESS) {
        return err;
    }

    LOG_INFO(LOG_TAG, "Alarm 1 set for %04d-%02d-%02d %02d:%02d:%02d UTC",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);

    return RTC_SUCCESS;
}

rtc_error_t rtc_clear_alarm1(bool disable) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    /* Clear alarm flag */
    uint8_t status;
    rtc_error_t err = rtc_read_register(RTC_REG_STATUS, &status);
    if (err != RTC_SUCCESS) {
        return err;
    }

    if (status & RTC_STATUS_A1F) {
        status &= ~RTC_STATUS_A1F;
        err = rtc_write_register(RTC_REG_STATUS, status);
        if (err != RTC_SUCCESS) {
            return err;
        }
        LOG_DEBUG(LOG_TAG, "Alarm 1 flag cleared");
    }

    /* Optionally disable alarm interrupt */
    if (disable) {
        uint8_t ctrl;
        err = rtc_read_register(RTC_REG_CONTROL, &ctrl);
        if (err != RTC_SUCCESS) {
            return err;
        }

        ctrl &= ~RTC_CTRL_A1IE;
        err = rtc_write_register(RTC_REG_CONTROL, ctrl);
        if (err != RTC_SUCCESS) {
            return err;
        }
        LOG_DEBUG(LOG_TAG, "Alarm 1 disabled");
    }

    return RTC_SUCCESS;
}

rtc_error_t rtc_check_alarm1(bool *triggered_out) {
    if (triggered_out == NULL) {
        return RTC_ERROR_NULL_POINTER;
    }
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    uint8_t status;
    rtc_error_t err = rtc_read_register(RTC_REG_STATUS, &status);
    if (err != RTC_SUCCESS) {
        return err;
    }

    *triggered_out = (status & RTC_STATUS_A1F) != 0;
    return RTC_SUCCESS;
}

rtc_error_t rtc_get_alarm1(time_t *alarm_time_out, bool *enabled_out) {
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    /* Read alarm registers */
    uint8_t regs[4];
    rtc_error_t err = rtc_read_registers(RTC_REG_ALARM1_SECONDS, regs, 4);
    if (err != RTC_SUCCESS) {
        return err;
    }

    /* Check if enabled */
    if (enabled_out != NULL) {
        uint8_t ctrl;
        err = rtc_read_register(RTC_REG_CONTROL, &ctrl);
        if (err != RTC_SUCCESS) {
            return err;
        }
        *enabled_out = (ctrl & RTC_CTRL_A1IE) != 0;
    }

    /* Build time (note: alarm only stores partial date info) */
    if (alarm_time_out != NULL) {
        /* Read current time to get year/month context */
        time_t current;
        err = rtc_read_time(&current);
        if (err != RTC_SUCCESS) {
            return err;
        }

        struct tm *tm_now = gmtime(&current);
        if (tm_now == NULL) {
            return RTC_ERROR_INVALID_TIME;
        }

        struct tm tm_alarm;
        memset(&tm_alarm, 0, sizeof(tm_alarm));
        tm_alarm.tm_sec = bcd_to_bin(regs[0] & 0x7F);
        tm_alarm.tm_min = bcd_to_bin(regs[1] & 0x7F);
        tm_alarm.tm_hour = bcd_to_bin(regs[2] & 0x3F);
        tm_alarm.tm_mday = bcd_to_bin(regs[3] & 0x3F);
        tm_alarm.tm_mon = tm_now->tm_mon;
        tm_alarm.tm_year = tm_now->tm_year;
        tm_alarm.tm_isdst = 0;

        *alarm_time_out = mktime(&tm_alarm);
    }

    return RTC_SUCCESS;
}

// ============================================================================
// Temperature Functions
// ============================================================================

rtc_error_t rtc_read_temperature(double *temp_out) {
    if (temp_out == NULL) {
        return RTC_ERROR_NULL_POINTER;
    }
    if (!g_rtc.initialized) {
        return RTC_ERROR_NOT_INITIALIZED;
    }

    uint8_t regs[2];
    rtc_error_t err = rtc_read_registers(RTC_REG_TEMP_MSB, regs, 2);
    if (err != RTC_SUCCESS) {
        return err;
    }

    /* Temperature is in two's complement format:
     * MSB = integer part (signed)
     * LSB bits 7-6 = fractional part (0.25C resolution)
     */
    int8_t msb = (int8_t)regs[0];
    uint8_t lsb = (regs[1] >> 6) & 0x03;

    *temp_out = (double)msb + (double)lsb * 0.25;

    return RTC_SUCCESS;
}
