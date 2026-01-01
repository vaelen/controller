/*
 * Rotator Control Tasks
 *
 * Tasks for controlling the antenna rotator via GS-232A protocol.
 * Includes position querying, status parsing, and command sending.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef ROTATOR_H
#define ROTATOR_H

#include <rtems.h>

/*
 * Antenna Location Task
 *
 * Sends periodic position query commands to the antenna rotator.
 * Uses GS-232A protocol: "C2\r" requests azimuth and elevation.
 * The rotator_status_task handles parsing responses.
 *
 * @param arg  Unused task argument
 */
rtems_task antenna_location_task(rtems_task_argument arg);

/*
 * Rotator Status Task
 *
 * Listens for status responses from the rotator (GS-232A protocol).
 * Parses "+0aaa+0eee\r" format (azimuth first, then elevation).
 * Sends antenna_position_message_t to controller via message queue.
 *
 * @param arg  Unused task argument
 */
rtems_task rotator_status_task(rtems_task_argument arg);

/*
 * Rotator Command Task
 *
 * Receives rotator commands from the pass executor and sends
 * GS-232A protocol commands to the rotator via UART.
 *
 * @param arg  Unused task argument
 */
rtems_task rotator_command_task(rtems_task_argument arg);

#endif /* ROTATOR_H */
