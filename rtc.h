/*
 * DS3231MZ RTC Driver for Satellite Tracking Controller
 *
 * Provides real-time clock support using the DS3231MZ I2C RTC chip.
 * The RTC maintains time during power loss and provides alarm functionality
 * for scheduling satellite pass wake-ups.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef RTC_H
#define RTC_H

#include <rtems.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

// ============================================================================
// DS3231MZ Constants
// ============================================================================

/* DS3231MZ I2C address (7-bit) */
#define RTC_I2C_ADDRESS             0x68

/* DS3231MZ Register Addresses */
#define RTC_REG_SECONDS             0x00
#define RTC_REG_MINUTES             0x01
#define RTC_REG_HOURS               0x02
#define RTC_REG_DAY                 0x03
#define RTC_REG_DATE                0x04
#define RTC_REG_MONTH               0x05
#define RTC_REG_YEAR                0x06
#define RTC_REG_ALARM1_SECONDS      0x07
#define RTC_REG_ALARM1_MINUTES      0x08
#define RTC_REG_ALARM1_HOURS        0x09
#define RTC_REG_ALARM1_DAY_DATE     0x0A
#define RTC_REG_ALARM2_MINUTES      0x0B
#define RTC_REG_ALARM2_HOURS        0x0C
#define RTC_REG_ALARM2_DAY_DATE     0x0D
#define RTC_REG_CONTROL             0x0E
#define RTC_REG_STATUS              0x0F
#define RTC_REG_AGING_OFFSET        0x10
#define RTC_REG_TEMP_MSB            0x11
#define RTC_REG_TEMP_LSB            0x12

/* Control Register Bits (0x0E) */
#define RTC_CTRL_EOSC               (1 << 7)  /* Enable Oscillator (0=run, 1=stop on battery) */
#define RTC_CTRL_BBSQW              (1 << 6)  /* Battery-Backed Square Wave */
#define RTC_CTRL_CONV               (1 << 5)  /* Convert Temperature */
#define RTC_CTRL_RS2                (1 << 4)  /* Rate Select 2 */
#define RTC_CTRL_RS1                (1 << 3)  /* Rate Select 1 */
#define RTC_CTRL_INTCN              (1 << 2)  /* Interrupt Control (1=alarm, 0=sqw) */
#define RTC_CTRL_A2IE               (1 << 1)  /* Alarm 2 Interrupt Enable */
#define RTC_CTRL_A1IE               (1 << 0)  /* Alarm 1 Interrupt Enable */

/* Status Register Bits (0x0F) */
#define RTC_STATUS_OSF              (1 << 7)  /* Oscillator Stop Flag */
#define RTC_STATUS_EN32KHZ          (1 << 3)  /* Enable 32kHz Output */
#define RTC_STATUS_BSY              (1 << 2)  /* Busy (temperature conversion) */
#define RTC_STATUS_A2F              (1 << 1)  /* Alarm 2 Flag */
#define RTC_STATUS_A1F              (1 << 0)  /* Alarm 1 Flag */

/* Alarm Match Modes (A1M4:A1M1 / A2M4:A2M2 bits) */
#define RTC_ALARM_MASK_BIT          (1 << 7)  /* Mask bit in alarm registers */

// ============================================================================
// Error Codes
// ============================================================================

typedef enum rtc_error {
    RTC_SUCCESS = 0,
    RTC_ERROR_NULL_POINTER = -1,
    RTC_ERROR_I2C_OPEN = -2,
    RTC_ERROR_I2C_READ = -3,
    RTC_ERROR_I2C_WRITE = -4,
    RTC_ERROR_NOT_INITIALIZED = -5,
    RTC_ERROR_INVALID_TIME = -6,
    RTC_ERROR_OSCILLATOR_STOPPED = -7
} rtc_error_t;

// ============================================================================
// Time Source Indicator
// ============================================================================

typedef enum rtc_time_source {
    RTC_TIME_SOURCE_UNKNOWN = 0,
    RTC_TIME_SOURCE_RTC = 1,
    RTC_TIME_SOURCE_GPS = 2
} rtc_time_source_t;

// ============================================================================
// Initialization Functions
// ============================================================================

/*
 * Initialize the RTC driver.
 *
 * Opens the I2C device, checks the oscillator status, and configures
 * the control register for alarm interrupts.
 *
 * @param device_path  Path to I2C device (e.g., "/dev/iic2")
 * @return             RTC_SUCCESS or error code
 */
rtc_error_t rtc_init(const char *device_path);

/*
 * Shutdown the RTC driver.
 *
 * Closes the I2C file descriptor.
 */
void rtc_shutdown(void);

/*
 * Check if RTC is initialized and oscillator is valid.
 *
 * @return  true if RTC is ready for use
 */
bool rtc_is_valid(void);

/*
 * Check if oscillator was stopped (time may be invalid).
 *
 * @return  true if oscillator stop flag was set at init
 */
bool rtc_oscillator_was_stopped(void);

// ============================================================================
// Time Functions
// ============================================================================

/*
 * Read current time from RTC.
 *
 * @param time_out  Output: time as Unix timestamp (UTC)
 * @return          RTC_SUCCESS or error code
 */
rtc_error_t rtc_read_time(time_t *time_out);

/*
 * Write time to RTC.
 *
 * Also clears the oscillator stop flag if set.
 *
 * @param time  Time to set as Unix timestamp (UTC)
 * @return      RTC_SUCCESS or error code
 */
rtc_error_t rtc_write_time(time_t time);

/*
 * Synchronize RTC with GPS time.
 *
 * Only writes to RTC if drift exceeds threshold or force is true.
 * Updates internal tracking of last sync time.
 *
 * @param gps_time       Current GPS time (Unix timestamp, UTC)
 * @param max_drift_sec  Maximum drift before sync (seconds)
 * @param force          Force sync even if within threshold
 * @return               RTC_SUCCESS or error code
 */
rtc_error_t rtc_sync_from_gps(time_t gps_time, int max_drift_sec, bool force);

/*
 * Get the last GPS sync time.
 *
 * @return  Unix timestamp of last GPS sync, or 0 if never synced
 */
time_t rtc_get_last_sync_time(void);

/*
 * Get the current time source.
 *
 * @return  RTC_TIME_SOURCE_RTC, RTC_TIME_SOURCE_GPS, or UNKNOWN
 */
rtc_time_source_t rtc_get_time_source(void);

// ============================================================================
// Alarm Functions
// ============================================================================

/*
 * Set Alarm 1 (second-resolution, used for pass wake-up).
 *
 * Configures Alarm 1 to trigger at the specified time with
 * day/hour/minute/second matching. Enables the alarm interrupt.
 *
 * @param alarm_time  Time for alarm (Unix timestamp, UTC)
 * @return            RTC_SUCCESS or error code
 */
rtc_error_t rtc_set_alarm1(time_t alarm_time);

/*
 * Clear Alarm 1 flag and optionally disable the alarm.
 *
 * @param disable  If true, also disables the alarm interrupt
 * @return         RTC_SUCCESS or error code
 */
rtc_error_t rtc_clear_alarm1(bool disable);

/*
 * Check if Alarm 1 has triggered.
 *
 * @param triggered_out  Output: true if alarm flag is set
 * @return               RTC_SUCCESS or error code
 */
rtc_error_t rtc_check_alarm1(bool *triggered_out);

/*
 * Get the currently set Alarm 1 time.
 *
 * @param alarm_time_out  Output: alarm time (may be partial if not all fields match)
 * @param enabled_out     Output: true if alarm is enabled
 * @return                RTC_SUCCESS or error code
 */
rtc_error_t rtc_get_alarm1(time_t *alarm_time_out, bool *enabled_out);

// ============================================================================
// Temperature Functions
// ============================================================================

/*
 * Read temperature from RTC (0.25C resolution).
 *
 * The DS3231MZ has an internal temperature sensor used for
 * temperature compensation. This provides ambient temperature.
 *
 * @param temp_out  Output: temperature in degrees Celsius
 * @return          RTC_SUCCESS or error code
 */
rtc_error_t rtc_read_temperature(double *temp_out);

// ============================================================================
// Low-Level Register Functions
// ============================================================================

/*
 * Read a single register from the RTC.
 *
 * @param reg       Register address
 * @param value_out Output: register value
 * @return          RTC_SUCCESS or error code
 */
rtc_error_t rtc_read_register(uint8_t reg, uint8_t *value_out);

/*
 * Write a single register to the RTC.
 *
 * @param reg    Register address
 * @param value  Value to write
 * @return       RTC_SUCCESS or error code
 */
rtc_error_t rtc_write_register(uint8_t reg, uint8_t value);

/*
 * Read multiple consecutive registers.
 *
 * @param start_reg  Starting register address
 * @param data       Output buffer
 * @param len        Number of bytes to read
 * @return           RTC_SUCCESS or error code
 */
rtc_error_t rtc_read_registers(uint8_t start_reg, uint8_t *data, size_t len);

/*
 * Write multiple consecutive registers.
 *
 * @param start_reg  Starting register address
 * @param data       Data buffer to write
 * @param len        Number of bytes to write
 * @return           RTC_SUCCESS or error code
 */
rtc_error_t rtc_write_registers(uint8_t start_reg, const uint8_t *data, size_t len);

#endif /* RTC_H */
