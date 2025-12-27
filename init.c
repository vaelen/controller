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

/* Task configuration: Init (1) + GPS + Antenna + RotatorStatus + RadioFreq + RadioStatus + TLE + Pass + PassExec + Controller + Status (10) + libbsd tasks */
#define CONFIGURE_MAXIMUM_TASKS 32
#define CONFIGURE_UNIFIED_WORK_AREAS
#define CONFIGURE_UNLIMITED_OBJECTS

/* User extensions for libbsd thread support */
#define CONFIGURE_MAXIMUM_USER_EXTENSIONS 8

/* Message queue configuration (6 app queues + 1 log queue + radio queue + ctrl queue) */
/* pass_message_t is ~2.7KB with 64-step plan, pass queue holds 4 messages = ~11KB */
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 16
#define CONFIGURE_MESSAGE_BUFFER_MEMORY (64 * 1024)

/* Semaphore configuration for mutexes (UART1, UART3, TLE, State, Log, Config + libbsd) */
#define CONFIGURE_MAXIMUM_SEMAPHORES 32

/* File descriptor configuration: stdin/stdout/stderr (3) + GPS + Rotator + Radio + SD card + config files + libbsd */
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 64

/* Filesystem support for SD card */
#define CONFIGURE_FILESYSTEM_DOSFS
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK

/* Init task configuration */
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)

#define CONFIGURE_INIT

#include <rtems/confdefs.h>

/* libbsd configuration for BeagleBone Black */
#include <machine/rtems-bsd-config.h>

/*
 * Driver configuration for SD card support on BeagleBone Black.
 *
 * The sdhci_ti driver has been patched for RTEMS to skip clock
 * operations that require the full device tree hierarchy.
 * The driver attaches directly to simplebus and uses the default
 * 96MHz clock rate for AM335x.
 */
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
SYSINIT_DRIVER_REFERENCE(ti_scm, simplebus);
SYSINIT_DRIVER_REFERENCE(sdhci_ti, simplebus);
SYSINIT_DRIVER_REFERENCE(mmcsd, mmc);
