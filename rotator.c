/*
 * Rotator Control Tasks
 *
 * Tasks for controlling the antenna rotator via GS-232A protocol.
 * Includes position querying, status parsing, and command sending.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "rotator.h"
#include "shared.h"
#include "log.h"

rtems_task antenna_location_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("ANTENNA", "Location task started");

    while (true) {

        // Acquire mutex before UART access
        rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        // Send C2 query command to rotator
        if (g_rotator_fd >= 0) {
            const char *cmd = "C2\r";
            write(g_rotator_fd, cmd, 3);
            fsync(g_rotator_fd);
            LOG_DEBUG("ROTATOR", "Sent command: %s", cmd);
        }

        rtems_semaphore_release(g_uart1_mutex);

        // Poll every 2 seconds
        rtems_task_wake_after(2 * rtems_clock_get_ticks_per_second());
    }
}

rtems_task rotator_status_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("ROTATOR", "Status task started");

    char buf[32];
    int buf_pos = 0;

    while (true) {
        // Acquire mutex before UART access
        rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        // Non-blocking read from rotator
        if (g_rotator_fd >= 0) {
            char read_buf[16];
            ssize_t total_read = 0;
            ssize_t n = read(g_rotator_fd, read_buf, sizeof(read_buf) - 1);
            while (n > 0) {
                total_read += n;
                // Append to buffer
                for (ssize_t i = 0; i < n && buf_pos < (int)sizeof(buf) - 1; i++) {
                    buf[buf_pos++] = read_buf[i];
                }
                buf[buf_pos] = '\0';
                n = read(g_rotator_fd, read_buf, sizeof(read_buf) - 1);
            }
            if (total_read > 0)
                LOG_DEBUG("ROTATOR", "Read %zd bytes from rotator", total_read);
        }

        rtems_semaphore_release(g_uart1_mutex);

        if (buf_pos == 0) {
            // No data received, yield and continue
            rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
            continue;
        }

        // Check for complete response (ends with CR)
        char *cr = strchr(buf, '\r');
        if (cr != NULL) {
            *cr = '\0';  // Null-terminate at CR

            // Parse GS-232A format: +0aaa+0eee or +0aaa +0eee
            // Example: +0180+0045 or +0180 +0045 = 180 deg az, 45 deg el
            int az_deg = 0, el_deg = 0;
            if (sscanf(buf, "+0%3d +0%3d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "+0%3d+0%3d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "%d %d", &az_deg, &el_deg) == 2 ||
                sscanf(buf, "%d%d", &az_deg, &el_deg) == 2) {

                // Convert degrees to radians and send message
                antenna_position_message_t msg;
                msg.type = MSG_ANTENNA_POSITION;
                msg.azimuth = (double)az_deg * SGP4_DEG_TO_RAD;
                msg.elevation = (double)el_deg * SGP4_DEG_TO_RAD;

                rtems_message_queue_send(g_antenna_queue, &msg, sizeof(msg));
                LOG_DEBUG("ROTATOR", "Position: az=%d el=%d deg", az_deg, el_deg);
            }

            // Reset buffer (shift remaining data if any)
            int remaining = buf_pos - (int)(cr - buf) - 1;
            if (remaining > 0) {
                memmove(buf, cr + 1, remaining);
                buf_pos = remaining;
            } else {
                buf_pos = 0;
            }
            buf[buf_pos] = '\0';
        }

        // Yield to other tasks (poll at ~20 Hz)
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
    }
}

rtems_task rotator_command_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("ROTCMD", "Rotator command task started");

    /* Command buffer for GS-232A protocol */
    char cmd_buf[32];

    while (true) {
        /* Wait for command */
        rotator_command_message_t cmd;
        size_t size;

        rtems_status_code status = rtems_message_queue_receive(
            g_rotator_cmd_queue, &cmd, &size,
            RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (status != RTEMS_SUCCESSFUL) {
            LOG_ERROR("ROTCMD", "Failed to receive command: %d", status);
            rtems_task_wake_after(rtems_clock_get_ticks_per_second());
            continue;
        }

        /* Check if rotator is available */
        if (g_rotator_fd < 0) {
            LOG_WARN("ROTCMD", "Rotator not connected");
            continue;
        }

        /* Process command */
        switch (cmd.command) {
            case ROT_CMD_GOTO: {
                /* Convert radians to integer degrees for GS-232A */
                int az_deg = (int)(cmd.target_azimuth_rad * SGP4_RAD_TO_DEG + 0.5);
                int el_deg = (int)(cmd.target_elevation_rad * SGP4_RAD_TO_DEG + 0.5);

                /* Clamp values to valid ranges */
                if (az_deg < 0) az_deg += 360;
                if (az_deg >= 360) az_deg -= 360;
                if (el_deg < 0) el_deg = 0;
                if (el_deg > 90) el_deg = 90;

                /* Format GS-232A command: W<az> <el>\r */
                /* W command moves to absolute position */
                int len = snprintf(cmd_buf, sizeof(cmd_buf), "W%03d %03d\r",
                                   az_deg, el_deg);

                /* Send to rotator */
                rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                write(g_rotator_fd, cmd_buf, len);
                fsync(g_rotator_fd);
                rtems_semaphore_release(g_uart1_mutex);

                LOG_DEBUG("ROTCMD", "Sent: W%03d %03d", az_deg, el_deg);
                break;
            }

            case ROT_CMD_STOP: {
                /* GS-232A stop command: S\r */
                rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                write(g_rotator_fd, "S\r", 2);
                fsync(g_rotator_fd);
                rtems_semaphore_release(g_uart1_mutex);

                LOG_DEBUG("ROTCMD", "Sent: S (stop)");
                break;
            }

            case ROT_CMD_PARK: {
                /* Park at az=0, el=0 */
                rtems_semaphore_obtain(g_uart1_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
                write(g_rotator_fd, "W000 000\r", 9);
                fsync(g_rotator_fd);
                rtems_semaphore_release(g_uart1_mutex);

                LOG_DEBUG("ROTCMD", "Sent: W000 000 (park)");
                break;
            }

            default:
                LOG_WARN("ROTCMD", "Unknown command: %d", cmd.command);
                break;
        }

        /* Small delay between commands to avoid flooding rotator */
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() / 20);  /* 50ms */
    }
}
