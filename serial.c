/*
 * Serial Port Initialization Utilities
 *
 * Provides functions for configuring serial ports with termios settings
 * for use with GPS, rotator, and radio UART devices.
 *
 * Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "serial.h"
#include "log.h"

int serial_init(const char *path, speed_t baud, int flags, bool flow_control) {
    int fd = open(path, flags | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        LOG_ERROR("SERIAL", "Failed to open serial port %s, Error: %s", path, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        LOG_ERROR("SERIAL", "Failed to get attributes for serial port %s, Error: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    /* Set baud rate */
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    /* 8N1 mode */
    tty.c_cflag &= ~PARENB;        /* No parity */
    tty.c_cflag &= ~CSTOPB;        /* 1 stop bit */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            /* 8 data bits */

    /* Hardware flow control */
    if (flow_control) {
        tty.c_cflag |= CRTSCTS;
    } else {
        tty.c_cflag &= ~CRTSCTS;
    }

    /* Ignore modem control lines */
    tty.c_cflag |= CLOCAL;

    /* Enable receiver if reading (O_RDONLY is 0, so check access mode) */
    int access_mode = flags & O_ACCMODE;
    if (access_mode == O_RDONLY || access_mode == O_RDWR) {
        tty.c_cflag |= CREAD;

        /* Raw input mode */
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        /* Disable input processing that could alter data */
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* No software flow control */
        tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);  /* Don't translate CR/LF */
        tty.c_iflag &= ~(ISTRIP | BRKINT);        /* Don't strip 8th bit, ignore break */

        /* Non-blocking: return immediately even with no data */
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
    }

    /* Raw output mode */
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR("SERIAL", "Failed to set attributes for serial port %s, Error: %s", path, strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

int serial_init_from_config(const serial_config_t *cfg) {
    return serial_init(cfg->device_path, cfg->baud_rate,
                       O_RDWR, cfg->flow_control);
}
