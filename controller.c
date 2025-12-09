/*
 * Hello World for RTEMS
 */
#include <rtems.h>
#include <stdlib.h>
#include <stdio.h>

rtems_task Init(rtems_task_argument ignored)
{
    printf("\n*** HELLO WORLD ***\n");
    printf("Hello from RTEMS on Zynq QEMU!\n");
    printf("*** END OF HELLO WORLD ***\n");
    
    exit(0);
}
