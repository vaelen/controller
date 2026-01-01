/*
 * Serial Port Initialization Utilities
 *
 * Provides functions for configuring serial ports with termios settings
 * for use with GPS, rotator, and radio UART devices.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>
#include <termios.h>

#include "config.h"

/*
 * Initialize a serial port with the specified settings.
 *
 * Configures the port for 8N1 mode with raw input/output.
 * Non-blocking reads are enabled (VMIN=0, VTIME=0).
 *
 * @param path          Device path (e.g., "/dev/ttyS0")
 * @param baud          Baud rate constant (e.g., B9600, B115200)
 * @param flags         Open flags (O_RDONLY, O_WRONLY, or O_RDWR)
 * @param flow_control  Enable CTS/RTS hardware flow control
 * @return              File descriptor on success, -1 on failure
 */
int serial_init(const char *path, speed_t baud, int flags, bool flow_control);

/*
 * Initialize a serial port from configuration structure.
 *
 * Opens the port for read/write access using settings from the config.
 *
 * @param cfg  Serial configuration from config file
 * @return     File descriptor on success, -1 on failure
 */
int serial_init_from_config(const serial_config_t *cfg);

#endif /* SERIAL_H */
