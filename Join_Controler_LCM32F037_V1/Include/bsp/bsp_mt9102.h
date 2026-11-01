/**
 * @file bsp_mt9102.h
 * @brief [BSP] Dual MT9102 linear Hall sensor driver - analog output via ADC
 *
 * Two MT9102 ICs are placed 90 electrical degrees apart on the rotor:
 *   Hall_A  ->  proportional to  sin(theta_e)
 *   Hall_B  ->  proportional to  cos(theta_e)
 *
 * Each IC outputs a ratiometric analog voltage:
 *   V_out = Vcc/2  +  sensitivity * B
 *
 * The raw ADC value (12-bit, 0..4095) maps to a signed magnetic flux:
 *   signal = ADC_raw - ADC_MID   (ADC_MID = 2048 at Vcc/2)
 *
 * atan2(sin_signal, cos_signal) gives the electrical angle, which is then
 * converted to the 0..16383 fixed-point scale used by the rest of the FOC.
 *
 * NOTE: Used when MOTOR_USE_HALL_MT9102 == 1 (JOINT_TYPE_NECK).
 */

#ifndef __BSP_MT9102_H
#define __BSP_MT9102_H

#include <stdint.h>
#include "mid/foc_config.h"

#if MOTOR_USE_HALL_MT9102

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the MT9102 driver.
 *        Call once at startup AFTER bsp_adc_init().
 */
void    bsp_mt9102_init(void);

/**
 * @brief Read the current angle from the two Hall sensors.
 *
 * @return Angle in the same 14-bit fixed-point scale (0..16383)
 *         as bsp_mt6701_read_raw(), so motor_ctrl_fast_loop requires NO changes.
 */
int32_t bsp_mt9102_read_raw(void);

void    bsp_mt9102_recalibrate(void);

/**
 * @brief Configure baseline calibration parameters (from Flash or calibration routine).
 */
void    bsp_mt9102_set_calibration(uint16_t off_sin, uint16_t off_cos, uint16_t gain_num, uint16_t gain_den);

/**
 * @brief Retrieve active runtime calibration parameters.
 */
void    bsp_mt9102_get_calibration(uint16_t *off_sin, uint16_t *off_cos, uint16_t *gain_num, uint16_t *gain_den);

#if DEBUG_HALL_ANGLE_LOG
void    bsp_mt9102_log_poll(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_USE_HALL_MT9102 */

#endif /* __BSP_MT9102_H */

