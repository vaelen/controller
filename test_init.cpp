/*
 * RTEMS Test Configuration
 * Common configuration for all sattrack test executables
 */
#include <rtems.h>
#include <bsp.h>

/* Forward declaration */
extern "C" rtems_task Init(rtems_task_argument argument);

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS 4
#define CONFIGURE_INIT_TASK_STACK_SIZE (64 * 1024)

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT

#include <rtems/confdefs.h>
