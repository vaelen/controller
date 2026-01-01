/*
 * Pass Management Tasks
 *
 * Tasks for TLE database management, pass prediction, and pass execution.
 * Includes the TLE updater, pass calculator, and pass executor tasks.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef PASS_H
#define PASS_H

#include <rtems.h>
#include <stdbool.h>
#include <stddef.h>

#include "sgp4.h"

/*
 * TLE Updater Task
 *
 * Downloads TLE data from configured URL, filters by configured satellites,
 * saves to SD card, and loads into memory.
 * Notifies controller when database is updated.
 *
 * @param arg  Unused task argument
 */
rtems_task tle_updater_task(rtems_task_argument arg);

/*
 * Pass Calculator Task
 *
 * Periodically calculates upcoming satellite passes.
 * Uses the TLE database and observer location.
 * Sends pass predictions to controller.
 *
 * @param arg  Unused task argument
 */
rtems_task pass_calculator_task(rtems_task_argument arg);

/*
 * Pass Executor Task
 *
 * Executes satellite passes by controlling the rotator and radio.
 * Receives commands from the controller via g_executor_cmd_queue.
 * Maintains real-time tracking state and sends commands via g_rotator_cmd_queue.
 *
 * @param arg  Unused task argument
 */
rtems_task pass_executor_task(rtems_task_argument arg);

/*
 * Find satellite name by NORAD ID.
 * Must be called with g_tle_database_mutex held.
 *
 * @param norad_id  NORAD catalog ID
 * @return          Pointer to name string if found, NULL otherwise
 */
const char *find_satellite_name(int norad_id);

/*
 * Look up satellite in TLE database and copy its SGP4 state.
 * Must be called with g_tle_database_mutex held.
 *
 * @param norad_id   NORAD catalog ID
 * @param state_out  Output: SGP4 state
 * @param name_out   Output: satellite name (can be NULL)
 * @param name_size  Size of name buffer
 * @return           True if found, false otherwise
 */
bool get_satellite_state(int norad_id, sgp4_state_t *state_out,
                         char *name_out, size_t name_size);

#endif /* PASS_H */
