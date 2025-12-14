/*
 * RTEMS Configuration for Satellite Tracking Controller
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <rtems.h>
#include <bsp.h>

/* Forward declaration */
rtems_task Init(rtems_task_argument argument);

/* Clock driver - required for task scheduling */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

/* Console driver */
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

/* Task configuration: Init (1) + GPS + Antenna + TLE + Pass + PassExec + Controller (6) + system tasks */
#define CONFIGURE_MAXIMUM_TASKS 16

/* Message queue configuration (5 app queues + 1 log queue) */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 6
#define CONFIGURE_MESSAGE_BUFFER_MEMORY (12 * 1024)

/* Semaphore configuration for mutexes */
#define CONFIGURE_MAXIMUM_SEMAPHORES 5

/* File descriptor configuration: stdin/stdout/stderr (3) + GPS + Rotator + Radio + spare */
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 16

/* Init task configuration */
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_STACK_SIZE (16 * 1024)

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
