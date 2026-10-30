/**
 * @file app_isr.c
 * @brief [Application layer] Time-critical interrupt glue that wires BSP drivers to middleware
 *          0x0800330C TIM1_CC_Handler (IRQ14, 20 kHz FOC loop)
 *
 *        The handler only sequences calls: all hardware access is in the BSP layer and all
 *        control maths is in middleware/motor_ctrl.
 */

#include "bsp/bsp_adc.h"
#include "bsp/bsp_mt6701.h"
#include "bsp/bsp_pwm.h"

#include "mid/motor_ctrl.h"

void TIM1_CC_Handler(void) {
    bsp_adc_start_conversion();
    motor_ctrl_fast_loop(bsp_mt6701_read_raw());
    bsp_pwm_clear_cc4_flag();
}
