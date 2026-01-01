/*
 * Radio Control Tasks
 *
 * Tasks for controlling the radio via Yaesu CAT protocol.
 * Includes status querying, response parsing, and helper functions.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "radio.h"
#include "shared.h"
#include "log.h"

rtems_task radio_frequency_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("RADIO", "Frequency task started");

    // CAT query commands
    static const char *queries[] = {
        "FA;",   // VFO-A frequency
        "FB;",   // VFO-B frequency
        "AG0;",  // AF gain
        "RG0;",  // RF gain
        "MG;",   // Mic gain
        "MD0;",  // Mode
        "PA0;",  // Pre-amp
        "FT;"    // Active VFO
    };
    static const int num_queries = sizeof(queries) / sizeof(queries[0]);

    while (true) {
        rtems_semaphore_obtain(g_uart3_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (g_radio_fd >= 0) {
            for (int i = 0; i < num_queries; i++) {
                write(g_radio_fd, queries[i], strlen(queries[i]));
            }
            fsync(g_radio_fd);
            LOG_DEBUG("RADIO", "Sent %d query commands", num_queries);
        }

        rtems_semaphore_release(g_uart3_mutex);

        // Poll every 2 seconds
        rtems_task_wake_after(2 * rtems_clock_get_ticks_per_second());
    }
}

rtems_task radio_status_task(rtems_task_argument arg) {
    (void)arg;
    LOG_INFO("RADIO", "Status task started");

    char buf[128];
    int buf_pos = 0;

    while (true) {
        rtems_semaphore_obtain(g_uart3_mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

        if (g_radio_fd >= 0) {
            ssize_t total_read = 0;
            char read_buf[64];
            ssize_t n = read(g_radio_fd, read_buf, sizeof(read_buf) - 1);
            while (n > 0) {
                total_read += n;
                for (ssize_t i = 0; i < n && buf_pos < (int)sizeof(buf) - 1; i++) {
                    buf[buf_pos++] = read_buf[i];
                }
                buf[buf_pos] = '\0';
                n = read(g_radio_fd, read_buf, sizeof(read_buf) - 1);
            }
            if (total_read > 0)
                LOG_DEBUG("RADIO", "Read %zd bytes from radio", total_read);
        }

        rtems_semaphore_release(g_uart3_mutex);

        // Parse complete responses (terminated by ';')
        // Accumulate fields into a message with valid_fields bitmask
        radio_status_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.type = MSG_RADIO_STATUS;
        msg.valid_fields = 0;

        char *semi;
        while ((semi = strchr(buf, ';')) != NULL) {
            *semi = '\0';
            char *cmd = buf;

            // Parse based on command prefix and set valid_fields bit
            unsigned long freq;
            unsigned int val;
            char mode_char;

            if (sscanf(cmd, "FA%9lu", &freq) == 1) {
                msg.vfo_a_freq_hz = freq;
                msg.valid_fields |= RADIO_FIELD_VFO_A_FREQ;
            } else if (sscanf(cmd, "FB%9lu", &freq) == 1) {
                msg.vfo_b_freq_hz = freq;
                msg.valid_fields |= RADIO_FIELD_VFO_B_FREQ;
            } else if (sscanf(cmd, "AG0%3u", &val) == 1) {
                msg.af_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_AF_GAIN;
            } else if (sscanf(cmd, "RG0%3u", &val) == 1) {
                msg.rf_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_RF_GAIN;
            } else if (sscanf(cmd, "MG%3u", &val) == 1) {
                msg.mic_gain = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_MIC_GAIN;
            } else if (sscanf(cmd, "MD0%c", &mode_char) == 1) {
                if (mode_char >= '1' && mode_char <= '9')
                    msg.mode = (radio_mode_t)(mode_char - '0');
                else if (mode_char >= 'A' && mode_char <= 'E')
                    msg.mode = (radio_mode_t)(mode_char - 'A' + 10);
                msg.valid_fields |= RADIO_FIELD_MODE;
            } else if (sscanf(cmd, "PA0%1u", &val) == 1) {
                msg.preamp = (radio_preamp_t)val;
                msg.valid_fields |= RADIO_FIELD_PREAMP;
            } else if (sscanf(cmd, "FT%1u", &val) == 1) {
                msg.active_vfo = (uint8_t)val;
                msg.valid_fields |= RADIO_FIELD_ACTIVE_VFO;
            }

            // Shift buffer
            int remaining = buf_pos - (int)(semi - buf) - 1;
            if (remaining > 0) {
                memmove(buf, semi + 1, remaining);
                buf_pos = remaining;
            } else {
                buf_pos = 0;
            }
            buf[buf_pos] = '\0';
        }

        // Send message if any fields were parsed
        if (msg.valid_fields != 0) {
            rtems_message_queue_send(g_radio_queue, &msg, sizeof(msg));
            LOG_DEBUG("RADIO", "Parsed fields=0x%02X", msg.valid_fields);
        }

        // Yield to other tasks (poll at ~20 Hz)
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(50));
    }
}

const char *radio_mode_to_string(radio_mode_t mode) {
    switch (mode) {
        case RADIO_MODE_LSB:      return "LSB";
        case RADIO_MODE_USB:      return "USB";
        case RADIO_MODE_CW:       return "CW";
        case RADIO_MODE_FM:       return "FM";
        case RADIO_MODE_AM:       return "AM";
        case RADIO_MODE_RTTY_LSB: return "RTTY-L";
        case RADIO_MODE_CW_R:     return "CW-R";
        case RADIO_MODE_DATA_LSB: return "DATA-L";
        case RADIO_MODE_RTTY_USB: return "RTTY-U";
        case RADIO_MODE_DATA_FM:  return "DATA-FM";
        case RADIO_MODE_FM_N:     return "FM-N";
        case RADIO_MODE_DATA_USB: return "DATA-U";
        case RADIO_MODE_AM_N:     return "AM-N";
        case RADIO_MODE_C4FM:     return "C4FM";
        default:                  return "???";
    }
}

const char *radio_preamp_to_string(radio_preamp_t preamp) {
    switch (preamp) {
        case RADIO_PREAMP_IPO:  return "IPO";
        case RADIO_PREAMP_AMP1: return "AMP1";
        case RADIO_PREAMP_AMP2: return "AMP2";
        default:                return "???";
    }
}
