/*
 * BeagleBone Black UART configuration
 *
 * Configures pin muxing, clocks, and registers UART0 (console), UART1 (GPS),
 * UART2 (rotator), and UART4 (radio) with the RTEMS console driver.
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <bsp.h>

#ifdef IS_AM335X

#include <rtems/sysinit.h>
#include <libchip/serial.h>
#include <libchip/ns16550.h>
#include <rtems/bspIo.h>
#include <bsp/irq.h>

/* ============================================================================
 * Pin Mux Configuration
 * ============================================================================ */

/* Control Module pad configuration base address */
#define CM_BASE       0x44E10000

/* Pin mux register offsets for UARTs */
#define CM_UART1_RXD  (CM_BASE + 0x980)  /* P9.26 - conf_uart1_rxd */
#define CM_UART1_TXD  (CM_BASE + 0x984)  /* P9.24 - conf_uart1_txd */
#define CM_UART2_RXD  (CM_BASE + 0x950)  /* P9.22 - conf_spi0_sclk */
#define CM_UART2_TXD  (CM_BASE + 0x954)  /* P9.21 - conf_spi0_d0 */
#define CM_UART4_RXD  (CM_BASE + 0x870)  /* P9.11 - conf_gpmc_wait0 */
#define CM_UART4_TXD  (CM_BASE + 0x874)  /* P9.13 - conf_gpmc_wpn */

/*
 * Pin mux configuration bits:
 *   Bits 2:0 - Mode select
 *   Bit 3    - Pull type (0=pulldown, 1=pullup)
 *   Bit 4    - Pull enable (0=enabled, 1=disabled)
 *   Bit 5    - Receiver enable (0=disabled, 1=enabled)
 *   Bit 6    - Slew rate (0=fast, 1=slow)
 */
#define RXACTIVE      (1 << 5)  /* Receiver enabled */
#define PULLUP        (1 << 4)  /* Pullup enabled (active low) */

/* UART1: Mode 0 */
#define PIN_UART1_RX  (0x00 | RXACTIVE | PULLUP)  /* Mode 0, RX enable, pullup */
#define PIN_UART1_TX  (0x00)                       /* Mode 0 */

/* UART2: Mode 1 on SPI0 pins */
#define PIN_UART2_RX  (0x01 | RXACTIVE | PULLUP)  /* Mode 1, RX enable, pullup */
#define PIN_UART2_TX  (0x01)                       /* Mode 1 */

/* UART4: Mode 6 on GPMC pins */
#define PIN_UART4_RX  (0x06 | RXACTIVE | PULLUP)  /* Mode 6, RX enable, pullup */
#define PIN_UART4_TX  (0x06)                       /* Mode 6 */

static void configure_uart_pins(void) {
    volatile uint32_t *reg;

    /* UART1 - Mode 0 */
    reg = (uint32_t *)CM_UART1_RXD;
    *reg = PIN_UART1_RX;
    reg = (uint32_t *)CM_UART1_TXD;
    *reg = PIN_UART1_TX;

    /* UART2 - Mode 1 */
    reg = (uint32_t *)CM_UART2_RXD;
    *reg = PIN_UART2_RX;
    reg = (uint32_t *)CM_UART2_TXD;
    *reg = PIN_UART2_TX;

    /* UART4 - Mode 6 */
    reg = (uint32_t *)CM_UART4_RXD;
    *reg = PIN_UART4_RX;
    reg = (uint32_t *)CM_UART4_TXD;
    *reg = PIN_UART4_TX;
}

/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

/* Clock Module Peripheral base address */
#define CM_PER_BASE           0x44E00000
#define CM_PER_UART1_CLKCTRL  (CM_PER_BASE + 0x6C)
#define CM_PER_UART2_CLKCTRL  (CM_PER_BASE + 0x70)
#define CM_PER_UART4_CLKCTRL  (CM_PER_BASE + 0x78)

/* Clock control values */
#define MODULEMODE_ENABLE     0x2

static void enable_uart_clocks(void) {
    volatile uint32_t *reg;

    reg = (uint32_t *)CM_PER_UART1_CLKCTRL;
    *reg = MODULEMODE_ENABLE;

    reg = (uint32_t *)CM_PER_UART2_CLKCTRL;
    *reg = MODULEMODE_ENABLE;

    reg = (uint32_t *)CM_PER_UART4_CLKCTRL;
    *reg = MODULEMODE_ENABLE;
}

/*
 * Initialize UART hardware before the console driver runs.
 * Called automatically via RTEMS_SYSINIT_ITEM.
 */
static void uart_hardware_init(void) {
    enable_uart_clocks();
    configure_uart_pins();
}

RTEMS_SYSINIT_ITEM(
    uart_hardware_init,
    RTEMS_SYSINIT_BSP_PRE_DRIVERS,
    RTEMS_SYSINIT_ORDER_MIDDLE
);

/* ============================================================================
 * Console Driver Configuration
 * ============================================================================ */

/* AM335x UART base addresses */
#define AM335X_UART0_BASE   0x44E09000  /* Console */
#define AM335X_UART1_BASE   0x48022000  /* GPS - /dev/ttyS1 */
#define AM335X_UART2_BASE   0x48024000  /* Rotator - /dev/ttyS2 */
#define AM335X_UART4_BASE   0x481A8000  /* Radio - /dev/ttyS3 */

/* AM335x UART IRQ numbers */
#define AM335X_UART0_IRQ    72
#define AM335X_UART1_IRQ    73
#define AM335X_UART2_IRQ    74
#define AM335X_UART4_IRQ    45

/* Console baud rate - must match BSP */
#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200
#endif

static uint8_t beagle_uart_get_register(uintptr_t addr, uint8_t i)
{
    volatile uint32_t *reg_r = (volatile uint32_t *)addr + i;
    return (uint8_t)*reg_r;
}

static void beagle_uart_set_register(uintptr_t addr, uint8_t i, uint8_t val)
{
    volatile uint32_t *reg = (volatile uint32_t *)addr;
    reg[i] = val;
}

/*
 * Console configuration table
 *
 * Device mapping:
 *   /dev/ttyS0 - UART0 (Console, 115200 baud)
 *   /dev/ttyS1 - UART1 (GPS, 9600 baud)
 *   /dev/ttyS2 - UART2 (Rotator, 9600 baud)
 *   /dev/ttyS3 - UART4 (Radio, 9600 baud)
 */
console_tbl Console_Configuration_Ports[] = {
    /* UART0 - Console */
    {
        .sDeviceName = "/dev/ttyS0",
        .deviceType = SERIAL_NS16550,
        .pDeviceFns = &ns16550_fns,
        .ulMargin = 16,
        .ulHysteresis = 8,
        .pDeviceParams = (void *)CONSOLE_BAUD,
        .ulCtrlPort1 = AM335X_UART0_BASE,
        .ulDataPort = AM335X_UART0_BASE,
        .ulIntVector = AM335X_UART0_IRQ,
        .getRegister = beagle_uart_get_register,
        .setRegister = beagle_uart_set_register,
        .ulClock = UART_CLOCK,
    },
    /* UART1 - GPS */
    {
        .sDeviceName = "/dev/ttyS1",
        .deviceType = SERIAL_NS16550,
        .pDeviceFns = &ns16550_fns,
        .ulMargin = 16,
        .ulHysteresis = 8,
        .pDeviceParams = (void *)9600,
        .ulCtrlPort1 = AM335X_UART1_BASE,
        .ulDataPort = AM335X_UART1_BASE,
        .ulIntVector = AM335X_UART1_IRQ,
        .getRegister = beagle_uart_get_register,
        .setRegister = beagle_uart_set_register,
        .ulClock = UART_CLOCK,
    },
    /* UART2 - Rotator */
    {
        .sDeviceName = "/dev/ttyS2",
        .deviceType = SERIAL_NS16550,
        .pDeviceFns = &ns16550_fns,
        .ulMargin = 16,
        .ulHysteresis = 8,
        .pDeviceParams = (void *)9600,
        .ulCtrlPort1 = AM335X_UART2_BASE,
        .ulDataPort = AM335X_UART2_BASE,
        .ulIntVector = AM335X_UART2_IRQ,
        .getRegister = beagle_uart_get_register,
        .setRegister = beagle_uart_set_register,
        .ulClock = UART_CLOCK,
    },
    /* UART4 - Radio (mapped to /dev/ttyS3) */
    {
        .sDeviceName = "/dev/ttyS3",
        .deviceType = SERIAL_NS16550,
        .pDeviceFns = &ns16550_fns,
        .ulMargin = 16,
        .ulHysteresis = 8,
        .pDeviceParams = (void *)9600,
        .ulCtrlPort1 = AM335X_UART4_BASE,
        .ulDataPort = AM335X_UART4_BASE,
        .ulIntVector = AM335X_UART4_IRQ,
        .getRegister = beagle_uart_get_register,
        .setRegister = beagle_uart_set_register,
        .ulClock = UART_CLOCK,
    },
};

unsigned long Console_Configuration_Count =
    sizeof(Console_Configuration_Ports) / sizeof(Console_Configuration_Ports[0]);

#endif /* IS_AM335X */
