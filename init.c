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

/* Task configuration: Init (1) + GPS + Antenna + RotatorStatus + RadioFreq + RadioStatus + TLE + Pass + PassExec + Controller + Status (10) + system tasks */
#define CONFIGURE_MAXIMUM_TASKS 20

/* Message queue configuration (6 app queues + 1 log queue + radio queue) */
/* pass_message_t is ~2.7KB with 64-step plan, pass queue holds 4 messages = ~11KB */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 8
#define CONFIGURE_MESSAGE_BUFFER_MEMORY (48 * 1024)

/* Semaphore configuration for mutexes (UART1, UART3, TLE, State, Log, spare) */
#define CONFIGURE_MAXIMUM_SEMAPHORES 8

/* File descriptor configuration: stdin/stdout/stderr (3) + GPS + Rotator + Radio + spare */
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 16

/* Init task configuration */
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_STACK_SIZE (16 * 1024)

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
