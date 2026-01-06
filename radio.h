/*
 * Radio Control Tasks
 *
 * Tasks for controlling the radio via Yaesu CAT protocol.
 * Includes status querying, response parsing, and helper functions.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef RADIO_H
#define RADIO_H

#include <rtems.h>

#include "shared.h"

/*
 * Radio Frequency Task
 *
 * Sends periodic CAT query commands to the radio to request status.
 * Uses Yaesu FT-991A CAT protocol.
 * The radio_status_task handles parsing responses.
 *
 * @param arg  Unused task argument
 */
rtems_task radio_frequency_task(rtems_task_argument arg);

/*
 * Radio Status Task
 *
 * Listens for CAT responses from the radio (Yaesu FT-991A).
 * Parses responses and sends radio_status_message_t to controller.
 * Supports partial updates via valid_fields bitmask.
 *
 * @param arg  Unused task argument
 */
rtems_task radio_status_task(rtems_task_argument arg);

/*
 * Convert radio mode enum to human-readable string.
 *
 * @param mode  Radio operating mode
 * @return      String representation (e.g., "USB", "FM")
 */
const char *radio_mode_to_string(radio_mode_t mode);

/*
 * Convert radio preamp enum to human-readable string.
 *
 * @param preamp  Preamp setting
 * @return        String representation (e.g., "IPO", "AMP1")
 */
const char *radio_preamp_to_string(radio_preamp_t preamp);

/*
 * Radio Command Task
 *
 * Receives frequency/mode commands from pass executor and sends
 * Yaesu CAT protocol commands to the radio.
 *
 * @param arg  Unused task argument
 */
rtems_task radio_command_task(rtems_task_argument arg);

/*
 * Convert signal mode to radio mode using config mappings.
 *
 * @param cfg          Configuration with mode mappings
 * @param signal_mode  Signal modulation mode
 * @return             Corresponding radio mode
 */
radio_mode_t signal_mode_to_radio_mode(const config_t *cfg, signal_mode_t signal_mode);

/*
 * Convert signal mode enum to human-readable string.
 *
 * @param mode  Signal modulation mode
 * @return      String representation (e.g., "AFSK", "GMSK")
 */
const char *signal_mode_to_string(signal_mode_t mode);

/*
 * Convert encoding type enum to human-readable string.
 *
 * @param encoding  Encoding type
 * @return          String representation (e.g., "AX.25", "CCSDS")
 */
const char *encoding_to_string(encoding_type_t encoding);

#endif /* RADIO_H */
