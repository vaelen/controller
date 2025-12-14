/*
 * Code specific to the BeagleBone Black platform
 *
 * Copyright (c) 2025 Andrew C. Young <andrew@vaelen.org>
 * SPDX-License-Identifier: MIT
 */

#include <bsp.h>
#include <bsp/bbb-gpio.h>

#define CM_BASE       0x44E10000
#define CM_UART1_RXD  (CM_BASE + 0x980)  /* P9.26 */
#define CM_UART1_TXD  (CM_BASE + 0x984)  /* P9.24 */
#define CM_UART2_RXD  (CM_BASE + 0x950)  /* P9.22 */
#define CM_UART2_TXD  (CM_BASE + 0x954)  /* P9.21 */
#define CM_UART4_RXD  (CM_BASE + 0x870)  /* P9.11 */
#define CM_UART4_TXD  (CM_BASE + 0x874)  /* P9.13 */

/* Mode 0 = UART, with receiver enabled, pullup */
#define PIN_UART_RX   (0x00 | (1 << 5) | (1 << 4))  /* Mode 0, RX enable, pullup */
#define PIN_UART_TX   (0x00)                        /* Mode 0 */

static void configure_uart_pins(void) {
    volatile uint32_t *reg;
    
    /* UART1 */
    reg = (uint32_t *)CM_UART1_RXD;
    *reg = PIN_UART_RX;
    reg = (uint32_t *)CM_UART1_TXD;
    *reg = PIN_UART_TX;
    
    /* UART2 */
    reg = (uint32_t *)CM_UART2_RXD;
    *reg = PIN_UART_RX;
    reg = (uint32_t *)CM_UART2_TXD;
    *reg = PIN_UART_TX;
    
    /* UART4 */
    reg = (uint32_t *)CM_UART4_RXD;
    *reg = PIN_UART_RX;
    reg = (uint32_t *)CM_UART4_TXD;
    *reg = PIN_UART_TX;
}

#define CM_PER_BASE        0x44E00000
#define CM_PER_UART1_CLKCTRL  (CM_PER_BASE + 0x6C)
#define CM_PER_UART2_CLKCTRL  (CM_PER_BASE + 0x70)
#define CM_PER_UART4_CLKCTRL  (CM_PER_BASE + 0x78)

static void enable_uart_clocks(void) {
    volatile uint32_t *reg;
    
    reg = (uint32_t *)CM_PER_UART1_CLKCTRL;
    *reg = 0x2;  /* Enable */
    
    reg = (uint32_t *)CM_PER_UART2_CLKCTRL;
    *reg = 0x2;
    
    reg = (uint32_t *)CM_PER_UART4_CLKCTRL;
    *reg = 0x2;
}