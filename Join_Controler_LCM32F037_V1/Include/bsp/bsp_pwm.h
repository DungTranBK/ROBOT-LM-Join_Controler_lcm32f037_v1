/**
 * @file bsp_pwm.h
 * @brief [BSP] TIM1 three-phase complementary center-aligned PWM + ADC trigger (CC4)
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param period       auto-reload value (counts per half carrier)
 * @param dead_time    BDTR dead-time generator value
 * @param adc_trigger  CC4 compare value that raises the control-loop interrupt
 */
void bsp_pwm_init(uint16_t period, uint8_t dead_time, uint16_t adc_trigger);   /* 0x080047BC */

/** Write the compare registers of phases U, V, W (CCR1..CCR3). */
void bsp_pwm_set_compare(uint32_t ch1, uint32_t ch2, uint32_t ch3);

/** Enable all three-phase PWM outputs (set TIM1 MOE bit). */
void bsp_pwm_output_enable(void);

/** Disable all three-phase PWM outputs (clear TIM1 MOE bit). */
void bsp_pwm_output_disable(void);

/** Acknowledge the CC4 interrupt (TIM1->SR). */
void bsp_pwm_clear_cc4_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
