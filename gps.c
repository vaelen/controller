/*
 * GPS Task
 *
 * Receives NMEA data from a GPS receiver via UART and sends
 * position/time updates to the controller via message queue.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"
#include "shared.h"
#include "serial.h"
#include "nmea.h"
#include "log.h"

rtems_task gps_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("GPS", "Task started");

    /* Get current configuration */
    config_t cfg;
    config_get_copy(&cfg);

    int fd = serial_init_from_config(&cfg.gps);
    if (fd < 0) {
        LOG_ERROR("GPS", "Failed to open GPS port %s", cfg.gps.device_path);
        rtems_task_exit();
    }

    LOG_INFO("GPS", "Serial port %s opened successfully", cfg.gps.device_path);

    nmea_buffer_t nmea_buf;
    nmea_buffer_init(&nmea_buf);

    /* Track latest valid data from each sentence type */
    nmea_gga_t last_gga = {0};
    nmea_rmc_t last_rmc = {0};

    char read_buf[64];
    char line[NMEA_BUFFER_SIZE];

    while (true) {
        /* Non-blocking read from serial */
        ssize_t n;
        n = read(fd, read_buf, sizeof(read_buf) - 1);
        while (n > 0) {
            read_buf[n] = '\0';
            nmea_buffer_add(&nmea_buf, read_buf);
            n = read(fd, read_buf, sizeof(read_buf) - 1);
        }

        /* Process complete lines from buffer */
        while (nmea_buffer_get_line(&nmea_buf, line, sizeof(line))) {
            /* Strip trailing newline/carriage return */
            while(line[strlen(line)-1] == '\n' || line[strlen(line)-1] == '\r') {
                line[strlen(line)-1] = '\0';
            }
            // LOG_DEBUG("GPS", "NMEA: %s", line);

            /* Validate checksum before parsing */
            if (!nmea_validate_checksum(line)) {
                continue;
            }

            if (nmea_is_gga(line)) {
                LOG_DEBUG("GPS", "NMEA GGA: %s", line);
                nmea_parse_gga(line, &last_gga);
            } else if (nmea_is_rmc(line)) {
                LOG_DEBUG("GPS", "NMEA RMC: %s", line);
                nmea_parse_rmc(line, &last_rmc);
            }

            /* Send message if we have valid position and date/time */
            if (last_gga.valid && last_gga.fix_quality > 0 &&
                last_rmc.valid && last_rmc.date.valid) {

                gps_message_t msg;
                msg.type = MSG_GPS_UPDATE;

                /* Convert degrees to radians, meters to km */
                msg.location.lat_rad = last_gga.lat_deg * SGP4_DEG_TO_RAD;
                msg.location.lon_rad = last_gga.lon_deg * SGP4_DEG_TO_RAD;
                msg.location.alt_km = last_gga.alt_m / 1000.0;

                /* Build time_t from RMC date + GGA time (UTC) */
                struct tm tm_time;
                memset(&tm_time, 0, sizeof(tm_time));
                tm_time.tm_year = last_rmc.date.year - 1900;
                tm_time.tm_mon = last_rmc.date.month - 1;
                tm_time.tm_mday = last_rmc.date.day;
                tm_time.tm_hour = last_gga.time.hour;
                tm_time.tm_min = last_gga.time.minute;
                tm_time.tm_sec = (int)last_gga.time.second;
                tm_time.tm_isdst = 0;

                /* Use mktime (assumes UTC in RTEMS without timezone config) */
                msg.utc_time = mktime(&tm_time);

                rtems_message_queue_send(g_gps_queue, &msg, sizeof(msg));
            }
        }

        /* Yield to other tasks (poll at ~10 Hz) */
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(100));
    }
}
