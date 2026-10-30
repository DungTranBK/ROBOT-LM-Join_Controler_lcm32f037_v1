/**
 * @file bsp_systick.c
 * @brief [BSP] SysTick 1 ms time base
 *        Matching disasm 0x080032B0 (ISR) and 0x08003E94 (LOAD = 0x000176FF @ 96 MHz)
 */

#include "system/lcm32_regs.h"
#include "bsp/bsp_clock.h"
#include "bsp/bsp_systick.h"

static volatile uint32_t s_ms_counter = 0; /* 0x20000068 */
static volatile uint8_t  s_1ms_flag   = 0; /* 0x20000078 */

/**
 * @brief SysTick ISR: 1ms period @ 96MHz
 *        Matching disasm 0x080032B0
 */
void SysTick_Handler(void) {
    s_ms_counter++;
    s_1ms_flag = 1;
}

void bsp_systick_init_1ms(void) {
    SysTick->LOAD = BSP_CORE_CLOCK_HZ / 1000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0); /* CLKSOURCE=AHB, TICKINT=1, ENABLE=1 */
}

uint32_t bsp_systick_get_ms(void) {
    return s_ms_counter;
}

uint8_t bsp_systick_take_1ms_flag(void) {
    if (s_1ms_flag) {
        s_1ms_flag = 0;
        return 1;
    }
    return 0;
}
