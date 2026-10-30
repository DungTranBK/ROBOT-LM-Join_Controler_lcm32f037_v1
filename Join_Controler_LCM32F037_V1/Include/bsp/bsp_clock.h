/**
 * @file bsp_clock.h
 * @brief [BSP] Peripheral clock gating (core PLL itself is set up in the System layer)
 */

#ifndef __BSP_CLOCK_H
#define __BSP_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_CORE_CLOCK_HZ     96000000UL    /* SYSCLK after LM_SystemInit() */

void bsp_clock_init(void);      /* peripheral clock gating (0x08004128) */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CLOCK_H */
