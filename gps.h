/*
 * GPS Task
 *
 * Receives NMEA data from a GPS receiver via UART and sends
 * position/time updates to the controller via message queue.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef GPS_H
#define GPS_H

#include <rtems.h>

/*
 * GPS Task Entry Point
 *
 * Receives data from a GPS receiver via UART.
 * Parses NMEA sentences (GGA for position, RMC for date/time).
 * Sends updates to the controller via message queue.
 *
 * @param arg  Unused task argument
 */
rtems_task gps_task(rtems_task_argument arg);

#endif /* GPS_H */
