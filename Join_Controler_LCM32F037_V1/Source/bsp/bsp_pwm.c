/**
 * @file bsp_pwm.c
 * @brief [BSP] TIM1 PWM driver, ported 1:1 from code_da_doc.bin 0x080047BC
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_tim.h"

#include "bsp/bsp_pwm.h"

/* 0x080047BC : TIM1 center aligned PWM, CC4 triggers the control ISR */
void bsp_pwm_init(uint16_t period, uint8_t dead_time, uint16_t adc_trigger) {
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;
    TIM_BDTRInitTypeDef     bd;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler   = 1;
    tb.TIM_Period      = period;
    tb.TIM_CounterMode = TIM_CounterMode_CenterAligned2;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode       = TIM_OCMode_PWM1;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Enable;
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    oc.TIM_Pulse        = adc_trigger;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC4Init(TIM1, &oc);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_BDTRStructInit(&bd);
    bd.TIM_OSSIState = TIM_OSSIState_Enable;
    bd.TIM_DeadTime  = dead_time;
    TIM_BDTRConfig(TIM1, &bd);

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    /* PWM outputs start disabled; motor_ctrl enables them on the first
     * active==1 tick (torque_enable=1 + calib_done=1). */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

void bsp_pwm_set_compare(uint32_t ch1, uint32_t ch2, uint32_t ch3) {
    TIM1->CCR1 = ch1;
    TIM1->CCR2 = ch2;
    TIM1->CCR3 = ch3;
}

void bsp_pwm_output_enable(void) {
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void bsp_pwm_output_disable(void) {
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

#define BSP_PWM_SR_CC4IF          0x10UL   /* capture/compare 4 interrupt flag */

void bsp_pwm_clear_cc4_flag(void) {
    TIM1->SR &= ~BSP_PWM_SR_CC4IF;
}
