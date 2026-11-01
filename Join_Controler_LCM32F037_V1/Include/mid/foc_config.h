/**
 * @file foc_config.h
 * @brief [Middleware] FOC carrier/PWM parameters and per-joint motor physical
 * configuration (HIP vs KNEE vs NECK) shared by the control code and
 * application.
 */

#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. Joint Type Selector
 *    - JOINT_TYPE_HIP  (0): EC4012 Hip Actuator  (10 PP, i = 7.5:1, MT6701 SPI,
 * normal U/V)
 *    - JOINT_TYPE_KNEE (1): Knee Joint Actuator  (16 PP, i = 9.0:1, MT6701 SPI,
 * invert U/V)
 *    - JOINT_TYPE_NECK (2): Neck Joint Actuator  ( 7 PP, i = 1.0:1, Dual MT9102
 * Hall ADC, normal U/V)
 * ============================================================================
 */
#define JOINT_TYPE_HIP 0
#define JOINT_TYPE_KNEE 1
#define JOINT_TYPE_NECK 2

#ifndef MOTOR_JOINT_TYPE
#define MOTOR_JOINT_TYPE JOINT_TYPE_NECK
#endif

/* ============================================================================
 * 2. PWM & Current-Loop Carrier Parameters (20 kHz center-aligned)
 * ============================================================================
 */
#define FOC_PWM_ARR 0x04AF /* 1199                                         */
#define FOC_PWM_DEAD_TIME 0x2F
#define FOC_PWM_ADC_TRIG (FOC_PWM_ARR - 0x46)
#define FOC_PWM_DUTY_MIN 0x46
#define FOC_PWM_DUTY_MID                                                       \
  0x257 /* 599                                          */

/* ============================================================================
 * 3. Per-Joint Physical Parameters, MIT Impedance Limits & Rotor Conversions
 * ============================================================================
 */
#if (MOTOR_JOINT_TYPE == JOINT_TYPE_HIP)
/* ----------------------------------------------------------------------------
 * [JOINT_TYPE_HIP] EC4012 Geared Hip Actuator:
 *
 * 1. Physical parameters:
 *    - Pole pairs         : 10 (20 magnet poles)
 *    - Phase U/V wiring   : Normal (FOC_INVERT_PHASE_UV = 0)
 *    - Gearbox ratio      : i = 15 / 2 = 7.5 : 1  (i^2 = 56.25)
 *    - Max output torque  : tau_out_max = Kt_rotor * I_max * i ~ 1.023 N.m
 *    - Rotor inertia      : J_rotor ~ 0.9e-5 kg.m^2 (40x12 mm outrunner bell)
 *    - Output-shaft J_out : J_out = J_rotor * i^2 ~ 0.5e-3 kg.m^2 (unloaded)
 *
 * 2. Calculation of MOTOR_KP_MAX (Torque saturation limit at output shaft):
 *    Full peak torque (~1.0 N.m) is reached at deflection delta_theta = 0.05
 * rad (~2.86 deg): Kp_max = tau_out_max / delta_theta = 1.0 / 0.05 = 20.000
 * N.m/rad
 *      => Register value (x1000) = 20000U
 *
 * 3. Calculation of MOTOR_KD_MAX (Second-order damping & encoder quantization
 * limit):
 *    - From closed-loop damping ratio (zeta ~ 0.6 at Kp_max = 20.0 N.m/rad):
 *        Kd_max = 2 * zeta * sqrt(J_out * Kp_max)
 *               = 2 * 0.6  * sqrt(0.5e-3 * 20.0) = 1.2 * 0.1 = 0.120 N.m.s/rad
 *    - Verification against discrete velocity quantization (0.5 ms window,
 *      1 encoder count = 7.324 rotor RPM, rotor gain = Kd_SI * 4.189 Iq/RPM):
 *        Kd_rotor_max = 0.120 * 4.189 = 0.503 Iq/RPM (10x factory POS_HOLD_KD =
 * 0.05) Higher values amplify 1-LSB velocity steps into audible high-frequency
 * buzz.
 *      => Register value (x1000) = 120U
 *
 * 4. Output-shaft SI -> Rotor fixed-point conversion (i = 7.5:1, tau_max =
 *    1.0 N.m = 300 Iq):
 *    - 1 N.m at output shaft   = 300 Iq_counts (IQ_LIMIT)
 *    - 1 rad at output shaft   = 7.5 * 1024 / (2*pi) = 1222.31 rotor_counts
 *    - 1 rad/s at output shaft = 7.5 * 60 / (2*pi)   = 71.6197 rotor_RPM
 *    => Kp_rotor = Kp_SI * (300 / 1222.31) = Kp_SI * 2454 / 10000
 *    => Kd_rotor = Kd_SI * (300 / 71.6197) = Kd_SI * 4189 / 1000
 *    => tau_iq   = tau_mNm * 300 / 1000
 * ----------------------------------------------------------------------------
 */
#define MOTOR_POLE_PAIRS 10 /* EC4012 BLDC: 10 pole pairs (20 magnet poles) */
#define FOC_INVERT_PHASE_UV 0
#define MOTOR_GEAR_RATIO_NUM 15 /* EC4012 gearbox ratio = 15 / 2 = 7.5 : 1 */
#define MOTOR_GEAR_RATIO_DEN 2
#define MOTOR_USE_HALL_MT9102 0 /* 0 = MT6701 SPI encoder */

/* Per-motor maximum MIT Impedance gains at output shaft (read-only RAM
 * 0x32..0x35) */
#define MOTOR_KP_MAX                                                           \
  20000U /* Max Kp: 20.000 N.m/rad   (1 LSB = 0.001 N.m/rad)   */
#define MOTOR_KD_MAX                                                           \
  120U /* Max Kd:  0.120 N.m.s/rad (1 LSB = 0.001 N.m.s/rad) */
#define MOTOR_DEFAULT_KP 10000U /* 10.000 N.m/rad at output shaft */
#define MOTOR_DEFAULT_KD 36U    /* 0.036 N.m.s/rad at output shaft */

#define SI_KP_TO_ROTOR_NUM 2454U
#define SI_KP_TO_ROTOR_DEN 10000U
#define SI_KD_TO_ROTOR_NUM 4189U
#define SI_KD_TO_ROTOR_DEN 1000U
#define SI_TAU_TO_IQ_NUM 300 /* = IQ_LIMIT */
#define SI_TAU_TO_IQ_DEN 1000

#elif (MOTOR_JOINT_TYPE == JOINT_TYPE_KNEE)
/* ----------------------------------------------------------------------------
 * [JOINT_TYPE_KNEE] Geared Knee Actuator:
 *
 * 1. Physical parameters:
 *    - Pole pairs         : 16 (32 magnet poles)
 *    - Phase U/V wiring   : Swapped U <-> V (FOC_INVERT_PHASE_UV = 1)
 *    - Gearbox ratio      : i = 9 / 1 = 9.0 : 1  (i^2 = 81)
 *    - Max output torque  : tau_out_max = Kt_rotor * I_max * i ~ 1.80 N.m
 *                           (Kt_out ~ 0.90 N.m/A at I_max ~ 2.0 A / IQ_LIMIT =
 * 300)
 *    - Rotor inertia      : J_rotor ~ 1.4e-5 kg.m^2 (32-pole outrunner rotor)
 *    - Output-shaft J_out : J_out = J_rotor * i^2 ~ 1.13e-3 kg.m^2 (unloaded)
 *
 * 2. Calculation of MOTOR_KP_MAX (Torque saturation limit at output shaft):
 *    Full peak torque (~1.80 N.m) is reached at deflection delta_theta =
 *    0.045 rad (~2.58 deg):
 *      Kp_max = tau_out_max / delta_theta = 1.80 / 0.045 = 40.000 N.m/rad
 *      (corresponds to rotor gain 4.545 Iq/rotor_count)
 *      => Register value (x1000) = 40000U
 *
 * 3. Calculation of MOTOR_KD_MAX (Second-order damping & encoder quantization
 * limit):
 *    - From closed-loop damping ratio (zeta ~ 0.6 at Kp_max = 40.0 N.m/rad):
 *        Kd_max = 2 * zeta * sqrt(J_out * Kp_max)
 *               = 2 * 0.6  * sqrt(1.13e-3 * 40.0) = 1.2 * 0.213 = 0.255
 * N.m.s/rad
 *    - Verification against discrete velocity quantization (0.5 ms window,
 *      1 encoder count = 7.324 rotor RPM, rotor gain = Kd_SI * 1.939 Iq/RPM):
 *        Kd_rotor_max = 0.260 * 1.939 = 0.504 Iq/RPM (10x factory POS_HOLD_KD
 * = 0.05)
 *      => Register value (x1000) = 260U
 *
 * 4. Output-shaft SI -> Rotor fixed-point conversion (i = 9.0:1, tau_max =
 *    1.80 N.m = 300 Iq):
 *    - 1 N.m at output shaft   = 300 / 1.80          = 166.67 Iq_counts
 *    - 1 rad at output shaft   = 9.0 * 1024 / (2*pi) = 1466.77 rotor_counts
 *    - 1 rad/s at output shaft = 9.0 * 60 / (2*pi)   = 85.9437 rotor_RPM
 *    => Kp_rotor = Kp_SI * (166.67 / 1466.77) = Kp_SI * 1136 / 10000
 *    => Kd_rotor = Kd_SI * (166.67 / 85.9437) = Kd_SI * 1939 / 1000
 *    => tau_iq   = tau_mNm * 167 / 1000
 * ----------------------------------------------------------------------------
 */
#define MOTOR_POLE_PAIRS 16 /* 16 pole pairs (32 magnet poles) */
#define FOC_INVERT_PHASE_UV 1
#define MOTOR_GEAR_RATIO_NUM 9 /* Knee gearbox ratio = 9 / 1 = 9.0 : 1 */
#define MOTOR_GEAR_RATIO_DEN 1
#define MOTOR_USE_HALL_MT9102 0 /* 0 = MT6701 SPI encoder */

#define MOTOR_KP_MAX 40000U     /* Max Kp: 40.000 N.m/rad */
#define MOTOR_KD_MAX 260U       /* Max Kd:  0.260 N.m.s/rad */
#define MOTOR_DEFAULT_KP 20000U /* 20.000 N.m/rad at output shaft */
#define MOTOR_DEFAULT_KD 80U    /* 0.080 N.m.s/rad at output shaft */

#define SI_KP_TO_ROTOR_NUM 1136U
#define SI_KP_TO_ROTOR_DEN 10000U
#define SI_KD_TO_ROTOR_NUM 1939U
#define SI_KD_TO_ROTOR_DEN 1000U
#define SI_TAU_TO_IQ_NUM 167
#define SI_TAU_TO_IQ_DEN 1000

#elif (MOTOR_JOINT_TYPE == JOINT_TYPE_NECK)
/* ----------------------------------------------------------------------------
 * [JOINT_TYPE_NECK] Direct-Drive Neck Actuator (DJI BLDC + Dual MT9102 Hall):
 *
 * 1. Physical parameters:
 *    - Pole pairs         : 7 (14 magnet poles)
 *    - Phase U/V wiring   : Normal (FOC_INVERT_PHASE_UV = 0)
 *    - Gearbox ratio      : i = 1 / 1 = 1.0 : 1 (Direct-drive, no gearbox)
 *    - Angle sensor       : Dual MT9102 linear Hall sensors (ADC CH3/CH4)
 *
 * 2. Direct-drive SI -> Rotor fixed-point conversion (i = 1.0:1):
 *    - 1 rad at output shaft   = 1.0 * 1024 / (2*pi) = 162.9746 rotor_counts
 *    - 1 rad/s at output shaft = 1.0 * 60 / (2*pi)   = 9.5493 rotor_RPM
 *    - Normalized torque scale (300 Iq_counts full scale):
 *        Kp_rotor = Kp_SI * (300 / 162.9746) = Kp_SI * 18408 / 10000
 *        Kd_rotor = Kd_SI * (300 / 9.5493)   = Kd_SI * 31416 / 1000
 *        tau_iq   = tau_mNm * 300 / 1000
 * ----------------------------------------------------------------------------
 */
#define MOTOR_POLE_PAIRS 7 /* Neck BLDC: 7 pole pairs */
#define FOC_INVERT_PHASE_UV 0
#define MOTOR_GEAR_RATIO_NUM 1 /* Direct-drive: ratio = 1 / 1 = 1.0 : 1 */
#define MOTOR_GEAR_RATIO_DEN 1
#define MOTOR_USE_HALL_MT9102 1 /* 1 = Dual MT9102 linear Hall via ADC */

#define MOTOR_KP_MAX 3000U     /* Max Kp: 3.000 N.m/rad (direct drive) */
#define MOTOR_KD_MAX 100U      /* Max Kd: 0.100 N.m.s/rad (direct drive) */
#define MOTOR_DEFAULT_KP 1500U /* Default Kp: 1.500 N.m/rad (~0.92 Iq/count) */
#define MOTOR_DEFAULT_KD 64U   /* Default Kd: 0.064 N.m.s/rad (~0.36 Iq/RPM) */

#define SI_KP_TO_ROTOR_NUM 9600U
#define SI_KP_TO_ROTOR_DEN 10000U
#define SI_KD_TO_ROTOR_NUM 5625U
#define SI_KD_TO_ROTOR_DEN 1000U
#define SI_TAU_TO_IQ_NUM 300 /* = IQ_LIMIT */
#define SI_TAU_TO_IQ_DEN 1000

#else
#error                                                                         \
    "Invalid MOTOR_JOINT_TYPE: must be JOINT_TYPE_HIP, JOINT_TYPE_KNEE, or JOINT_TYPE_NECK"
#endif

#define MOTOR_HAS_GEARBOX (MOTOR_GEAR_RATIO_NUM != MOTOR_GEAR_RATIO_DEN)

/* ============================================================================
 * 4. Feature & Debug Toggles
 * ============================================================================
 */
#define FEATURE_UART_STREAM_TELEMETRY                                          \
  0 /* 1 = Enable, 0 = Disable real-time Iu, Iw, Iv telemetry streaming over   \
       UART */

/* Debug: print a one-line summary over UART0 every 500 ms.
 *   0 = disabled (production), 1 = enabled (debug build only).
 * Format: "DBG pos=XXXX spd=SSSSS cur=CCCCC V=VVV T=TT fl=FF\r\n" */
#define DEBUG_UART_MONITOR 0

/* Debug: print raw Hall ADC + reconstructed angle every 250 ms (NECK only).
 *   0 = disabled (production), 1 = enabled (bring-up / tuning). */
#define DEBUG_HALL_ANGLE_LOG 0

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CONFIG_H */
