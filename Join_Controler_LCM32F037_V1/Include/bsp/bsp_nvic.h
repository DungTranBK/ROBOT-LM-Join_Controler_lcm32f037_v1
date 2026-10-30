/**
 * @file bsp_nvic.h
 * @brief [BSP] Interrupt priorities and enables for the motor controller
 */

#ifndef __BSP_NVIC_H
#define __BSP_NVIC_H

#ifdef __cplusplus
extern "C" {
#endif

/** 0x08003F10 : SysTick prio 0, TIM1_CC prio 1, DMAC_CH1_2 prio 2, TIM2 prio 2 */
void bsp_nvic_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_NVIC_H */
