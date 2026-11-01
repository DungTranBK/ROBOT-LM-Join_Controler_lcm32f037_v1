/**
 * @file motor_ctrl.h
 * @brief [Middleware] FOC motor control (position/speed/current loops +
 * calibration) Matching code_da_doc.bin: 0x080025B0 angle/speed  0x08002708 FOC
 * step  0x08003B64 clarke_park 0x08003C04 inv_park_svpwm
 *
 *        Hardware is reached only through the BSP layer (bsp_adc / bsp_pwm).
 */

#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "mid/foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Calibration state machine (original byte @0x2000006F) */
#define CAL_STATE_IDLE 0       /* normal run                                  */
#define CAL_STATE_ZERO_TOTAL 1 /* PWM off, average total-current ADC */
#define CAL_STATE_ZERO_PHASE 2 /* low duty, average U/V phase ADC */
#if MOTOR_USE_HALL_MT9102
#define CAL_STATE_HALL_SWEEP 3 /* open-loop 360 deg sweep, auto-cal Hall Min/Max */
#define CAL_STATE_ALIGN 4      /* ramp Vd, lock rotor, store angle offset     */
#else
#define CAL_STATE_ALIGN 3      /* ramp Vd, lock rotor, store angle offset     */
#endif

/* Control Modes (register 0x2D control_mode):
 *   2 = Multi-turn Position mode (Goal Position 0x1E..0x1F, speed limit
 * 0x20..0x21) 3 = MIT / Impedance mode (default on startup): tau_ref =
 * Kp*(goal_position - p_cur) + Kd*(moving_speed - v_cur) + tau_ff using unified
 * 10-byte register block 0x1A..0x23 [kp, kd, goal_position, moving_speed,
 * tau_ff].
 */
#define VEL_MODE_POSITION 2
#define VEL_MODE_IMPEDANCE 3

/** Copy angle/ADC offsets from the loaded control table. */
void motor_ctrl_load_calibration(void);

/** Fast loop, call once per PWM carrier interrupt (20 kHz) with the raw 14-bit
 * rotor angle. */
void motor_ctrl_fast_loop(int32_t raw_angle);

/* Unified target setters written by the bus master */
void motor_ctrl_set_speed_cmd(int16_t speed);
void motor_ctrl_set_goal_position(int16_t position);
void motor_ctrl_set_kp(uint16_t kp);
void motor_ctrl_set_kd(uint16_t kd);
void motor_ctrl_set_tau_ff(int16_t tau_ff);
bool motor_ctrl_start_calibration(void);

/* Measurements published to the monitor */
int16_t
motor_ctrl_get_position(void); /* signed 16-bit multi-turn output position */
int32_t motor_ctrl_get_speed(void); /* rpm */

#if FEATURE_UART_STREAM_TELEMETRY
int16_t motor_ctrl_get_iu(void);
int16_t motor_ctrl_get_iv(void);
int16_t motor_ctrl_get_iw(void);
int32_t motor_ctrl_get_elec_angle(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_H */
