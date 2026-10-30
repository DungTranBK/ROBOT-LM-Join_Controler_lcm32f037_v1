/**
 * @file bsp_clock.c
 * @brief [BSP] Peripheral clock gating, ported 1:1 from code_da_doc.bin 0x08004128
 */

#include "system/lcm32_regs.h"
#include "bsp/bsp_clock.h"

void bsp_clock_init(void) {
    /* AHB: GPIOA | GPIOB | DMA | GPIOF */
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR0, 0, 0x00200103UL);
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR0, 0, (1UL << 26));
    /* APB1: ADC | SSP0(SPI1) | UART0 | FLASH | TIM1 */
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR0, 0, 0x00093200UL);
    /* ANACTRL (OPA / comparator) */
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR2, 0, 0x20UL);

    /* UART0 kernel clock: ClkEnR1 stays 0 (golden dump 0x48007004 = 0) -> UART0 runs from
     * SYSCLK 96 MHz; bsp_bus_uart.c derives IBRD = 6 for 1 Mbaud. */
}
