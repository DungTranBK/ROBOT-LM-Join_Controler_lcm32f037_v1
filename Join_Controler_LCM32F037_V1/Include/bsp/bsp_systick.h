/**
 * @file bsp_systick.h
 * @brief [BSP] SysTick 1 ms time base (SysTick_Handler lives in bsp_systick.c)
 */

#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     bsp_systick_init_1ms(void);        /* LOAD = BSP_CORE_CLOCK_HZ/1000 - 1, IRQ on (0x08003E94) */
uint32_t bsp_systick_get_ms(void);          /* milliseconds since bsp_systick_init_1ms() */
uint8_t  bsp_systick_take_1ms_flag(void);   /* returns 1 once per elapsed tick and clears it */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYSTICK_H */
