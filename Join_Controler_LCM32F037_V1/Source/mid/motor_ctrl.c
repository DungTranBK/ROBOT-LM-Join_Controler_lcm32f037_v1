/**
 * @file motor_ctrl.c
 * @brief [Middleware] FOC BLDC motor control, ported from code_da_doc.bin
 * (LCM32F037, Cortex-M0)
 *
 *   0x080025B0 angle_speed()       0x08002708 foc_step()
 *   0x0800330C TIM1_CC ISR body (motor_ctrl_fast_loop, 20 kHz)
 *
 * Hardware independent: ADC samples and PWM outputs are accessed through
 * bsp_adc / bsp_pwm, the rotor angle is handed in by the caller (application
 * ISR), transforms live in foc_math.
 */

#include "bsp/bsp_adc.h"
#include "bsp/bsp_pwm.h"

#include "mid/control_table.h"
#include "mid/foc_config.h"
#if MOTOR_USE_HALL_MT9102
#include "bsp/bsp_mt9102.h"
#endif
#include "mid/foc_math.h"
#include "mid/motor_ctrl.h"
#include "mid/trig_table.h"

/* ------------------------------------------------------------------------------------------
 *  Constants taken from the binary
 * ----------------------------------------------------------------------------------------
 */
#define ELEC_ANGLE_MAX 0xF56F /* 62831 = 2*pi*10000 */
#define ELEC_RATIO_NUM 0xEFA  /* 3834  */
#define ELEC_ALIGN_STEP 10    /* electrical angle step per tick during align  */
#define SPEED_FILTER_CALLS 10
#define SPEED_INTEG_LIMIT                                                      \
  1000000 /* 0x000F4240                                   */
#define CURRENT_INTEG_LIMIT                                                    \
  0x1FBD0 /* 129,999                                      */
#define IQ_LIMIT 300
#define VEL_KI_DIV 3000 /* RAM initial value @0x20000010                */

/* --- EXTENSION: Friction Punch Kickstart --- */
#define PUNCH_MAX 50 /* Maximum allowable punch kick current clamp */
#define PUNCH_KICK_TICKS                                                       \
  500 /* Kickstart pulse duration: 500 ticks @ 20 kHz = 25 ms */

/* Rotor encoder: 14-bit raw angle, 10-bit internal loop position */
#define ROTOR_COUNTS_PER_REV 1024
#define ENC_WRAP_DETECT 12000 /* |delta| above this = encoder wrapped */
#define POS_WRAP_DETECT                                                        \
  (ROTOR_COUNTS_PER_REV / 2) /* 512: position wrapped    */
#define SPEED_RPM_PER_COUNT                                                    \
  7.32421875f /* 120000.0f / 16384.0f: sum(10 deltas) -> rpm @ 20 kHz */
#define ENC_SPEED_DEADBAND_RAW                                                 \
  1 /* 14-bit raw delta stationary deadband (+/-1 count / 0.5 ms) */
#define SPEED_EMA_PREV_WEIGHT 7 /* EMA filter weight for previous speed */
#define SPEED_EMA_DIV 8         /* EMA filter divisor (7 + 1 = 8) */
#define SPEED_ZERO_SNAP_RPM                                                    \
  2 /* Snap filtered speed to 0 within +/-2 RPM when stationary */

/* Current / voltage PI (Id ref = 0) */
#if MOTOR_USE_HALL_MT9102
#define CURRENT_KP_GAIN 120
#define CURRENT_KI_GAIN 5
#define CURRENT_D_INTEG_LIMIT 20000

/* Position hold: Iq = pos_err * KP + pos_integ - speed * KD */
#define POS_HOLD_KP 7.20f
#define POS_HOLD_KD 1.50f
#else
#define CURRENT_KI_GAIN 30

/* Position hold: Iq = pos_err * KP - speed * KD */
#define POS_HOLD_KP 2.5f
#define POS_HOLD_KD 0.05
#endif

/* Rotor alignment (calibration) */
#define ALIGN_RAMP_STEP 0.04f /* Vd ramp per tick */
#if MOTOR_USE_HALL_MT9102
#define ALIGN_RAMP_MAX 300.0f /* ~1.1A holding current for crisp alignment */
#define ALIGN_TICKS 16000     /* 1.6s at 10 kHz ISR */
#define ALIGN_AVG_SAMPLES                                                      \
  256 /* average last 256 samples to cancel ADC noise                          \
       */
#define CAL_SWEEP_PRELOCK_TICKS                                                \
  4096 /* 0.41s smooth pre-lock at elec=0 before sweeping */
#define CAL_SWEEP_ROTATE_TICKS                                                 \
  28096 /* 2.81s trapezoidal 1-full-turn (360 deg) sweep */
#define CAL_SWEEP_TICKS                                                        \
  (CAL_SWEEP_PRELOCK_TICKS + CAL_SWEEP_ROTATE_TICKS) /* 3.22s total */
#define CAL_SWEEP_RAMP_TICKS                                                   \
  4096 /* 0.41s acceleration / deceleration ramp (>> 12) */
#define CAL_SWEEP_STEP_Q8                                                      \
  4691 /* (7 * 62831 * 256) / 24000 = 1.000 mechanical turn */
#define CAL_SWEEP_PRELOCK_VOLT 180 /* initial rotor capture voltage */
#define CAL_SWEEP_VOLT 100         /* low-flux sweep voltage (~0.33A) */
#else
#define ALIGN_RAMP_MAX 400.0f
#define ALIGN_TICKS 30000 /* ticks before the angle offset is latched     */
#endif

/* Offset calibration: ticks counted at 20 kHz */
#define CAL_SETTLE_TICKS 3200 /* discard samples while the signal settles */
#define CAL_TOTAL_TICKS 4224  /* settle + 1024 averaged samples  */
#define CAL_PHASE_DUTY 0x64   /* low duty applied while sampling phase offset */

/* ------------------------------------------------------------------------------------------
 *  State (original RAM map in comments)
 * ----------------------------------------------------------------------------------------
 */
static volatile uint8_t s_calib_state = CAL_STATE_IDLE; /* 0x2000006F */
static volatile int16_t s_moving_speed_cmd = 0;         /* 0x2000006C */
static volatile int16_t s_goal_position_cmd =
    0; /* EXTENSION: latched Goal Position */

/* --- EXTENSION: Impedance / MIT-Cheetah mode shadow registers (Fixed-Point
 * int16) --- */
/* Written by motor_ctrl_set_impedance_cmd() from protocol context (converted
 * from Master output-shaft SI units [x0.001 N.m/rad, x0.001 N.m.s/rad, mN.m]
 * into rotor-side fixed-point gains), read by foc_step() at 20 kHz ISR. */
static volatile uint16_t s_imp_kp =
    ((uint32_t)DEFAULT_KP * SI_KP_TO_ROTOR_NUM + (SI_KP_TO_ROTOR_DEN / 2U)) /
    SI_KP_TO_ROTOR_DEN; /* rotor stiffness gain [x0.001 Iq/rotor_count] */
static volatile uint16_t s_imp_kd =
    ((uint32_t)DEFAULT_KD * SI_KD_TO_ROTOR_NUM + (SI_KD_TO_ROTOR_DEN / 2U)) /
    SI_KD_TO_ROTOR_DEN; /* rotor damping gain   [x0.001 Iq/rotor_RPM]   */
static volatile int16_t s_imp_tau_ff =
    0; /* rotor feedforward    [Iq counts, signed]      */

/* --- State variables for direction reversal --- */
static int32_t s_prev_target_speed =
    0; /* Target speed from previous tick for reversal edge detection */
#if !MOTOR_USE_HALL_MT9102
static int16_t s_punch_kick_timer =
    0; /* Countdown timer for decaying punch pulse (500 -> 0 ticks) */
static int16_t s_punch_kick_dir =
    0; /* Direction of punch kick (+1: forward, -1: reverse) */
#endif

static int32_t s_raw_angle;    /* 0x20000074 */
static int32_t s_angle_off;    /* 0x20000070 (low 16 bit loaded from table) */
static uint16_t s_pos10;       /* 0x200000B8 */
static int32_t s_multiturn;    /* 0x200000A4 (1024 counts/rotor_rev) */
static int32_t s_multiturn_14; /* Full 14-bit multi-turn rotor angle (16384
                                  counts/rotor_rev) */
static int32_t s_rev_cnt;      /* 0x200000B0 */
static int32_t s_speed;        /* 0x200000B4 */
static int16_t s_adc_v;        /* 0x200000C4 (ADC[1] >> 2, 10-bit scale) */
static int16_t s_adc_u;        /* 0x200000C6 (ADC[0] >> 2, 10-bit scale) */
static int16_t s_off_v;        /* 0x200000C0 (10-bit phase V zero offset) */
static int16_t s_off_u;        /* 0x200000C2 (10-bit phase U zero offset) */
#if MOTOR_USE_HALL_MT9102
static int32_t s_off_v_acc_q8; /* Sub-LSB 10-bit idle auto-zero tracker (Q8) */
static int32_t s_off_u_acc_q8; /* Sub-LSB 10-bit idle auto-zero tracker (Q8) */
#else
static int16_t s_adc_v_12;     /* Full 12-bit phase V ADC sample (0..4095) */
static int16_t s_adc_u_12;     /* Full 12-bit phase U ADC sample (0..4095) */
static int16_t s_off_v_12;     /* Full 12-bit phase V zero offset (Q2) */
static int16_t s_off_u_12;     /* Full 12-bit phase U zero offset (Q2) */
static int32_t s_off_v_acc_q8; /* Sub-LSB 12-bit idle auto-zero tracker (Q8) */
static int32_t s_off_u_acc_q8; /* Sub-LSB 12-bit idle auto-zero tracker (Q8) */
static int32_t s_id_filt_q2;   /* 20 kHz EMA-filtered Id in 12-bit Q2 scale */
static int32_t s_iq_filt_q2;   /* 20 kHz EMA-filtered Iq in 12-bit Q2 scale */
#endif

#if FEATURE_UART_STREAM_TELEMETRY
static volatile int16_t s_cur_iu;
static volatile int16_t s_cur_iv;
static volatile int16_t s_cur_iw;
#endif

static int16_t f_spd_err;    /* F+0x02 */
static int16_t f_iq_ref;     /* F+0x00 / F+0x04 */
static int16_t f_spd_cnt;    /* F+0x06 */
static int16_t f_pos_err;    /* F+0x08 */
static uint16_t f_pos_prev;  /* F+0x0A */
static int16_t f_turns;      /* F+0x0E */
static int32_t f_elec;       /* F+0x14 electrical angle, 1/10000 rad */
static int32_t f_integ_d;    /* F+0x20 */
static int32_t f_integ_q;    /* F+0x24 */
static int32_t f_vq;         /* F+0x28 */
static int32_t f_vd;         /* F+0x2C */
static int32_t f_integ_v;    /* F+0x30 velocity integrator */
static int32_t f_id, f_iq;   /* F+0x40 / F+0x44 */
static int32_t f_delta_acc;  /* F+0x48 */
static int32_t f_raw_prev;   /* F+0x4C */
static int32_t f_delta;      /* F+0x50 */
static int32_t f_duty[3];    /* F+0x54..0x5C */
static int32_t f_pos_target; /* F+0x60 */

static float s_align_ramp;   /* 0x200000BC */
static uint16_t s_align_cnt; /* 0x20000094 */
static uint16_t s_cal_cnt;   /* 0x20000096 */
static int32_t s_cal_sum_v;  /* 0x20000098 */
static int32_t s_cal_sum_u;  /* 0x2000009C */
static int32_t s_cal_sum_t;  /* 0x200000A0 */
#if MOTOR_USE_HALL_MT9102
static int32_t s_sweep_elec;
static int32_t s_sweep_elec_q8;
static uint16_t s_min_sin = 4095;
static uint16_t s_max_sin = 0;
static uint16_t s_min_cos = 4095;
static uint16_t s_max_cos = 0;
static int32_t s_align_sum_raw;
static int32_t s_align_ref_raw;
static int32_t s_align_pos_plus;
static int32_t s_pos14_filt_q8;
static int32_t s_pos14_hist[16];
static uint8_t s_spd_hist_idx;
static uint8_t s_spd_hist_init;
static int32_t s_speed_q4;
static uint16_t s_still_ticks;
static int32_t s_hold_perr_14;
static int32_t s_hold_raw_angle;
#endif

/* ------------------------------------------------------------------------------------------
 *  0x080025B0 : multi-turn position + speed (RPM = sum(10 deltas) * 7.3242 @ 20
 * kHz)
 * ----------------------------------------------------------------------------------------
 */
static void angle_speed(void) {
  int32_t raw = s_raw_angle;
  int32_t delta = raw - f_raw_prev;
  int16_t pos = (int16_t)(raw >> 4);
  int16_t diff;

  f_delta = delta;
  s_pos10 = (uint16_t)pos;
  diff = (int16_t)(pos - f_pos_prev);

  if (diff > POS_WRAP_DETECT) {
    f_turns = (int16_t)(f_turns - 1);
  } else if (diff < -POS_WRAP_DETECT) {
    f_turns = (int16_t)(f_turns + 1);
  }
  f_pos_prev = (uint16_t)pos;
  s_multiturn = ((int32_t)f_turns << 10) + pos;
  s_multiturn_14 = ((int32_t)f_turns << 14) + raw;

  if (delta > ENC_WRAP_DETECT) {
    s_rev_cnt--;
    f_delta = delta - 16383;
  } else if (delta < -ENC_WRAP_DETECT) {
    s_rev_cnt++;
    f_delta = delta + 16383;
  }

  f_delta_acc += f_delta;
  f_raw_prev = raw;

  f_spd_cnt = (int16_t)(f_spd_cnt + 1);
#if MOTOR_USE_HALL_MT9102
  if (!s_spd_hist_init) {
    s_pos14_filt_q8 = s_multiturn_14 << 8;
    for (int i = 0; i < 16; i++) {
      s_pos14_hist[i] = s_pos14_filt_q8;
    }
    s_spd_hist_idx = 0;
    s_spd_hist_init = 1;
  } else {
    s_pos14_filt_q8 += ((s_multiturn_14 << 8) - s_pos14_filt_q8) >> 3;
  }
  if (f_spd_cnt == SPEED_FILTER_CALLS) {
    f_spd_cnt = 0;
    int32_t delta_4ms_q8 = s_pos14_filt_q8 - s_pos14_hist[s_spd_hist_idx];
    s_pos14_hist[s_spd_hist_idx] = s_pos14_filt_q8;
    s_spd_hist_idx = (uint8_t)((s_spd_hist_idx + 1U) & 0x07U);

    int32_t acc_q8 =
        (delta_4ms_q8 >= -320 && delta_4ms_q8 <= 320) ? 0 : delta_4ms_q8;
    int32_t raw_spd_q4 = (acc_q8 * 3750) >> 16;
    s_speed_q4 = (s_speed_q4 + raw_spd_q4) >> 1;
    if (acc_q8 == 0 && s_speed_q4 >= -16 && s_speed_q4 <= 16) {
      s_speed_q4 = 0;
    }
    if (s_speed_q4 >= -32 && s_speed_q4 <= 32) {
      s_speed = 0;
    } else {
      s_speed = (s_speed_q4 >= 0) ? ((s_speed_q4 + 8) >> 4)
                                  : -(((-s_speed_q4) + 8) >> 4);
    }
    if (delta_4ms_q8 >= -512 && delta_4ms_q8 <= 512) {
      if (s_still_ticks < 10000U) {
        s_still_ticks++;
      }
    } else {
      s_still_ticks = 0;
    }
    f_delta_acc = 0;
  }
#else
  if (f_spd_cnt == SPEED_FILTER_CALLS) {
    /* Suppress 14-bit magnetic encoder thermal LSB toggle only when stationary;
     * preserve every single count during active slow rotation. */
    int32_t acc = (s_speed == 0 && f_delta_acc >= -ENC_SPEED_DEADBAND_RAW &&
                   f_delta_acc <= ENC_SPEED_DEADBAND_RAW)
                      ? 0
                      : f_delta_acc;
    int32_t raw_spd = (int32_t)((float)acc * SPEED_RPM_PER_COUNT);
    f_spd_cnt = 0;
    /* First-order EMA low-pass filter (7/8 weight, tau ~ 3.5 ms) */
    s_speed = (s_speed * SPEED_EMA_PREV_WEIGHT + raw_spd) / SPEED_EMA_DIV;
    if (raw_spd == 0 && s_speed >= -SPEED_ZERO_SNAP_RPM &&
        s_speed <= SPEED_ZERO_SNAP_RPM) {
      s_speed = 0;
    }
    f_delta_acc = 0;
  }
#endif
}

/* ------------------------------------------------------------------------------------------
 *  EXTENSION (not in the original firmware) : position (joint) mode, Velocity
 * Mode Control = 2 Goal Position 0..1023 (1024 counts per revolution) clamped
 * to Min/Max Angle Limit; Moving Speed is the speed limit.
 * ----------------------------------------------------------------------------------------
 */
#define POS_MODE_DEFAULT_SPEED                                                 \
  200 /* rpm limit when Moving Speed = 0                   */
#define POS_MODE_MAX_SPEED                                                     \
  1000 /* rpm                                               */
#define POS_MODE_MIN_SPEED                                                     \
  8 /* rpm, above the 7.3 rpm speed resolution           */

/* Deadband & settling thresholds for Position / MIT Impedance control:
 * - POS_DEADBAND_KP_COUNTS: +/-1 count rotor LSB filter during active motion
 * - POS_DEADBAND_KD_DEG_ROTOR: rotor angle window (deg) for settled hold lock
 * - POS_DEADBAND_KD_COUNTS: converted to 10-bit rotor counts (1 deg => 3
 *   counts)
 * - IMP_HOLD_SETTLE_TICKS: stationary ticks (2000 @ 20 kHz = 100 ms) inside
 *   +/-POS_DEADBAND_KD_COUNTS before latching settled hold (p_err=0, v_err=0)
 */
#define POS_DEADBAND_KP_COUNTS 1
#define POS_DEADBAND_KD_DEG_ROTOR 1
#define POS_DEADBAND_KD_COUNTS                                                 \
  ((ROTOR_COUNTS_PER_REV * POS_DEADBAND_KD_DEG_ROTOR + 180) / 360)
#define IMP_HOLD_SETTLE_TICKS 2000
#define POS_ERR_CLAMP_MAX 32767
#define IMP_GAIN_SCALE_DIV 1000

/* Mode 3 (MIT Impedance) 14-bit resolution & smooth trajectory parameters:
 * - POS_DEADBAND_KP_COUNTS_14 : +/-4 counts_14 (0.088 deg rotor) stationary
 * deadzone
 * - POS_DEADBAND_KD_COUNTS_14 : +/-16 counts_14 (0.35 deg rotor) stationary Kd
 * lock window
 * - POS_ERR_14_CLAMP_MAX      : +/-65535 counts_14 (+/-4 rotor turns)
 * anti-overflow clamp
 * - IMP_TRAJ_DEFAULT_ROTOR_RPM: 600 RPM rotor default cruise limit when
 * moving_speed == 0
 * - IMP_TRAJ_MIN_ROTOR_RPM    : 7 RPM rotor (< 1 RPM output shaft) minimum
 * speed limit
 * - IMP_TRAJ_MAX_ROTOR_RPM    : 2400 RPM rotor maximum trajectory speed limit
 * - IMP_TRAJ_ACCEL_Q8         : 1 Q8_count/tick^2 (~5700 RPM/s smooth
 * acceleration ramp)
 * - IMP_TRAJ_DECEL_SHIFT      : >> 8 exponential filter time constant (256
 * ticks = 12.8 ms)
 * - IMP_MOTION_KD_MIN_ROTOR   : 400 (0.400 Iq/RPM) active trajectory damping
 * anti-cogging floor
 * - IMP_FRIC_INTEG_DIV        : 800 velocity/lag integrator divisor during
 * active trajectory
 * - IMP_FRIC_INTEG_MAX_IQ     : 80 Iq counts (~260 mA) bounded stiction/cogging
 * compensator limit
 * - IMP_MOTION_TAIL_TICKS     : 3000 ticks (150 ms @ 20 kHz) landing window
 * after trajectory stops
 */
#if MOTOR_USE_HALL_MT9102
#define POS_DEADBAND_KP_COUNTS_14 0
#define POS_DEADBAND_KD_COUNTS_14                                              \
  32 /* +/-0.70 deg settled hold Kd mute window */
#define POS_INTEG_FREEZE_COUNTS_14                                             \
  32 /* +/-0.70 deg integrator freeze window                                   \
      */
#define POS_ERR_14_CLAMP_MAX                                                   \
  720 /* Max active spring lead (~15.8 deg => 216 Iq @ Kp=1500) */
#define IMP_TRAJ_DEFAULT_ROTOR_RPM 90
#define IMP_TRAJ_MIN_ROTOR_RPM 5
#define IMP_TRAJ_MAX_ROTOR_RPM 300
#define IMP_TRAJ_ACCEL_Q8 3
#define IMP_TRAJ_DECEL_SHIFT 10
#define IMP_MOTION_KD_MIN_ROTOR 1500
#define IMP_SETTLE_KD_MIN_ROTOR 1500
#define IMP_FRIC_INTEG_DIV 8000
#define IMP_FRIC_INTEG_MAX_IQ 45
#define IMP_FRIC_INTEG_LIMIT (IMP_FRIC_INTEG_MAX_IQ * IMP_FRIC_INTEG_DIV)
#define IMP_MOTION_TAIL_TICKS 1000
#else
#define POS_DEADBAND_KP_COUNTS_14 4
#define POS_DEADBAND_KD_COUNTS_14 16
#define POS_ERR_14_CLAMP_MAX 65535
#define IMP_TRAJ_DEFAULT_ROTOR_RPM 600
#define IMP_TRAJ_MIN_ROTOR_RPM 7
#define IMP_TRAJ_MAX_ROTOR_RPM 2400
#define IMP_TRAJ_ACCEL_Q8 1
#define IMP_TRAJ_DECEL_SHIFT 8
#define IMP_MOTION_KD_MIN_ROTOR 400
#define IMP_FRIC_INTEG_DIV 800
#define IMP_FRIC_INTEG_MAX_IQ 80
#define IMP_FRIC_INTEG_LIMIT (IMP_FRIC_INTEG_MAX_IQ * IMP_FRIC_INTEG_DIV)
#define IMP_MOTION_TAIL_TICKS 3000
#endif

/* Output-shaft SI unit -> Rotor fixed-point conversion (N = 7.5:1 gearbox):
 *   - 1 N.m at output shaft   = 300 Iq_counts (IQ_LIMIT)
 *   - 1 rad at output shaft   = 7.5 * 1024 / (2*pi) = 1222.31 rotor_counts
 *   - 1 rad/s at output shaft = 7.5 * 60 / (2*pi)   = 71.6197 rotor_RPM
 *   => Kp_rotor = Kp_SI * (300 / 1222.31) = Kp_SI * 2454 / 10000
 *   => Kd_rotor = Kd_SI * (300 / 71.6197) = Kd_SI * 4189 / 1000
 *   => tau_iq   = tau_mNm * 300 / 1000
 */
/* SI_*_TO_ROTOR_* and SI_TAU_TO_IQ_* are now defined per-joint in foc_config.h
 */

static inline uint16_t si_kp_to_rotor(uint16_t kp_si) {
  if (kp_si > MOTOR_KP_MAX) {
    kp_si = MOTOR_KP_MAX;
  }
  uint32_t v =
      ((uint32_t)kp_si * SI_KP_TO_ROTOR_NUM + (SI_KP_TO_ROTOR_DEN / 2U)) /
      SI_KP_TO_ROTOR_DEN;
  return (v > 65535U) ? 65535U : (uint16_t)v;
}

static inline uint16_t si_kd_to_rotor(uint16_t kd_si) {
  if (kd_si > MOTOR_KD_MAX) {
    kd_si = MOTOR_KD_MAX;
  }
  uint32_t v =
      ((uint32_t)kd_si * SI_KD_TO_ROTOR_NUM + (SI_KD_TO_ROTOR_DEN / 2U)) /
      SI_KD_TO_ROTOR_DEN;
  return (v > 65535U) ? 65535U : (uint16_t)v;
}

static inline int16_t si_tau_to_iq(int16_t tau_mNm) {
  int32_t v = ((int32_t)tau_mNm * SI_TAU_TO_IQ_NUM) / SI_TAU_TO_IQ_DEN;
  if (v > IQ_LIMIT)
    return IQ_LIMIT;
  if (v < -IQ_LIMIT)
    return -IQ_LIMIT;
  return (int16_t)v;
}

#if MOTOR_USE_HALL_MT9102
#define POS_HOLD_ENTER                                                         \
  16                     /* |err| <= 16 counts (5.6 deg): enter 14-bit hold    \
                          */
#define POS_HOLD_EXIT 32 /* |err| >  32 counts (11.2 deg): leave hold */
static int32_t s_pos_integ;
#else
#define POS_HOLD_ENTER 2 /* |err| <= 2 counts: enter hold */
#define POS_HOLD_EXIT 6  /* |err| >  6 counts: leave hold (hysteresis) */
#endif

static uint8_t s_pos_hold;
static uint8_t s_prev_mode;
static uint8_t s_prev_active; /* Edge detection for PWM output enable/disable */
static uint8_t s_imp_hold;
static uint16_t s_imp_settle_cnt;
static uint8_t s_imp_traj_init;
static int32_t s_imp_pos_q8; /* Smoothed 14-bit rotor position setpoint (Q8) */
static int32_t s_imp_vel_q8; /* Smoothed trajectory velocity (Q8 counts/tick) */
static int32_t
    s_imp_last_vlim;        /* Last active rotor RPM limit during smooth move */
static int32_t s_imp_integ; /* Bounded stiction & cogging torque integrator */
static uint16_t
    s_imp_motion_tail; /* Landing countdown ticks after trajectory stops */
static int8_t
    s_imp_prev_dir; /* Trajectory direction (+1, -1, 0) for reversal reset */

/* Gearbox conversion:
 * - Output shaft register: 16384 counts/rev (14-bit)
 * - Internal rotor loop (s_multiturn): 1024 counts/rotor_rev
 * - Full 14-bit rotor tracker (s_multiturn_14): 16384 counts/rotor_rev
 * - Gearbox ratio: MOTOR_GEAR_RATIO_NUM / MOTOR_GEAR_RATIO_DEN (15 / 2 = 7.5)
 */
#define OUT_TO_ROTOR_DEN                                                       \
  (MOTOR_GEAR_RATIO_DEN * (POSITION_COUNTS_PER_REV / ROTOR_COUNTS_PER_REV))

static inline int32_t pos_out_to_rotor(int32_t out_counts) {
  if (out_counts >= 0) {
    return (out_counts * MOTOR_GEAR_RATIO_NUM + (OUT_TO_ROTOR_DEN / 2)) /
           OUT_TO_ROTOR_DEN;
  } else {
    return (out_counts * MOTOR_GEAR_RATIO_NUM - (OUT_TO_ROTOR_DEN / 2)) /
           OUT_TO_ROTOR_DEN;
  }
}

static inline int32_t pos_out_to_rotor_14(int32_t out_counts) {
  if (out_counts >= 0) {
    return (out_counts * MOTOR_GEAR_RATIO_NUM + (MOTOR_GEAR_RATIO_DEN / 2)) /
           MOTOR_GEAR_RATIO_DEN;
  } else {
    return (out_counts * MOTOR_GEAR_RATIO_NUM - (MOTOR_GEAR_RATIO_DEN / 2)) /
           MOTOR_GEAR_RATIO_DEN;
  }
}

static inline int32_t pos_rotor_to_out(int32_t rotor_counts_14) {
  if (rotor_counts_14 >= 0) {
    return (rotor_counts_14 * MOTOR_GEAR_RATIO_DEN +
            (MOTOR_GEAR_RATIO_NUM / 2)) /
           MOTOR_GEAR_RATIO_NUM;
  } else {
    return (rotor_counts_14 * MOTOR_GEAR_RATIO_DEN -
            (MOTOR_GEAR_RATIO_NUM / 2)) /
           MOTOR_GEAR_RATIO_NUM;
  }
}

/* Returns 1 to hold the position (f_pos_target is set), else 0 with *speed =
 * rpm command. */
static int pos_mode_plan(int32_t *speed) {
  int32_t goal = (int32_t)s_goal_position_cmd;
  int32_t pmin = (int32_t)g_control_table.min_angle_limit;
  int32_t pmax = (int32_t)g_control_table.max_angle_limit;
  int32_t err, aerr, v, vlim;

  if (pmin == 0 && pmax == POSITION_SINGLE_TURN_MAX) {
    pmin = POSITION_MIN;
    pmax = POSITION_MAX;
  }
  if (pmax > POSITION_MAX) {
    pmax = POSITION_MAX;
  }
  if (pmin < POSITION_MIN) {
    pmin = POSITION_MIN;
  }
  if (goal > POSITION_MAX) {
    goal = POSITION_MAX;
  }
  if (goal < POSITION_MIN) {
    goal = POSITION_MIN;
  }
  if (pmin <= pmax) {
    if (goal > pmax || goal < pmin) {
      if (!(g_motor_status_flags & ERR_FLAG_ANGLE)) {
        g_motor_status_flags |= ERR_FLAG_ANGLE; /* goal outside the limits */
      }
      goal = (goal > pmax) ? pmax : pmin;
    } else if (g_motor_status_flags & ERR_FLAG_ANGLE) {
      g_motor_status_flags &= (uint8_t)~ERR_FLAG_ANGLE;
    }
  }

  err = pos_out_to_rotor(goal) - s_multiturn;
  aerr = (err < 0) ? -err : err;

  if (aerr <= POS_HOLD_ENTER) {
    s_pos_hold = 1;
  } else if (aerr > POS_HOLD_EXIT) {
    s_pos_hold = 0;
  }
  if (s_pos_hold) {
    f_pos_target = s_multiturn + err;
    f_integ_v = 0;
    g_control_table.moving_flag = 0;
    return 1;
  }

  vlim = s_moving_speed_cmd;
  if (vlim < 0) {
    vlim = -vlim;
  }
  if (vlim == 0) {
    vlim = POS_MODE_DEFAULT_SPEED;
  }
  if (vlim > POS_MODE_MAX_SPEED) {
    vlim = POS_MODE_MAX_SPEED;
  }
#if MOTOR_USE_HALL_MT9102
  if (s_moving_speed_cmd == 0) {
    vlim = 60;
  }
  v = (aerr + 1) >> 1; /* 0.5 rpm per count_10 (8 RPM at POS_HOLD_ENTER=16) */
  if (v > vlim) {
    v = vlim;
  }
  if (v < POS_MODE_MIN_SPEED) {
    v = POS_MODE_MIN_SPEED;
  }
#else
  v = (aerr * 3) >> 1; /* 1.5 rpm per count of error */
  if (v > vlim) {
    v = vlim;
  }
  if (v < POS_MODE_MIN_SPEED) {
    v = POS_MODE_MIN_SPEED;
  }
#endif
  *speed = (err < 0) ? -v : v;
  g_control_table.moving_flag = 1;
  return 0;
}

/* ------------------------------------------------------------------------------------------
 *  0x08002708 : field oriented control step (every TIM1 CC4 interrupt)
 * ----------------------------------------------------------------------------------------
 */
static void foc_step(void) {
  int32_t x;
  int32_t iu, iv, iw;
  int32_t tc, ts;
  int active;

  /* Electrical angle calculation (Rotor flux angle for Park / Inv-Park) */
  if (s_calib_state != CAL_STATE_ALIGN) {
#if MOTOR_USE_HALL_MT9102
    int32_t raw_comm = s_imp_hold ? s_hold_raw_angle : s_raw_angle;
    int32_t diff = (raw_comm - s_angle_off) & 0x3FFF;
    x = (diff * MOTOR_POLE_PAIRS) & 0x3FFF;
#else
    int32_t diff = (s_raw_angle - s_angle_off) & 0x3FFF;
    x = (diff * MOTOR_POLE_PAIRS) % 16384;
#endif
    /* scale 14-bit counts to sin/cos table 0..62831 scale (ratio 3834 / 1000
     * ~ 3.834) */
    x = (x * ELEC_RATIO_NUM) / 1000;
    if (x > ELEC_ANGLE_MAX) {
      x -= ELEC_ANGLE_MAX;
    }
    f_elec = x;
  } else {
    /* Calibration: ramp virtual electrical angle to pull and lock rotor at
     * angle 0 */
    x = f_elec + ELEC_ALIGN_STEP;
    f_elec = (x > ELEC_ANGLE_MAX) ? 0 : x;
  }

  /* sin/cos of the electrical angle, shared by Park and inverse Park */
  tc = trig_cos(f_elec);
  ts = trig_sin(f_elec);

#if MOTOR_USE_HALL_MT9102
  /* phase currents: V = ADC[1], U = ADC[0], W = -(U+V) (10-bit scale, matching
   * LCM32F037_BLDC_Keil) */
  s_adc_v = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) >> 2);
  s_adc_u = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) >> 2);
  iv = (int32_t)(uint16_t)s_adc_v - s_off_v;
  iu = (int32_t)(uint16_t)s_adc_u - s_off_u;
  iw = -(iv + iu);
#if FEATURE_UART_STREAM_TELEMETRY
  s_cur_iu = (int16_t)iu;
  s_cur_iv = (int16_t)iv;
  s_cur_iw = (int16_t)iw;
#endif
  foc_clarke_park(tc, ts, iw, iu, iv, &f_id, &f_iq);
#else
  /* phase currents: V = ADC[1], U = ADC[0], W = -(U+V) in full 12-bit (Q2) */
#if FOC_INVERT_PHASE_UV
  /* KNEE motor: phase U and V are physically swapped */
  s_adc_v_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) & 0x0FFFU);
  s_adc_u_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) & 0x0FFFU);
#else
  s_adc_v_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) & 0x0FFFU);
  s_adc_u_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) & 0x0FFFU);
#endif
  s_adc_v = (int16_t)(s_adc_v_12 >> 2);
  s_adc_u = (int16_t)(s_adc_u_12 >> 2);

  /* Full 12-bit zero-offset subtraction before Clarke/Park eliminates 2-bit
   * pre-Park truncation step and 16x/rev stationary DC offset ripple */
  iv = (int32_t)s_adc_v_12 - (int32_t)s_off_v_12;
  iu = (int32_t)s_adc_u_12 - (int32_t)s_off_u_12;
  iw = -(iv + iu);
#if FEATURE_UART_STREAM_TELEMETRY
  s_cur_iu = (int16_t)(iu >> 2);
  s_cur_iv = (int16_t)(iv >> 2);
  s_cur_iw = (int16_t)(iw >> 2);
#endif
  {
    int32_t id_raw_q2, iq_raw_q2;
    foc_clarke_park(tc, ts, iw, iu, iv, &id_raw_q2, &iq_raw_q2);
    /* Fast 20 kHz EMA low-pass filter (3/4 prev + 1/4 new, fc ~ 1.1 kHz)
     * suppresses PWM switching spikes while preserving current loop phase
     * margin */
    s_id_filt_q2 = (s_id_filt_q2 * 3 + id_raw_q2) >> 2;
    s_iq_filt_q2 = (s_iq_filt_q2 * 3 + iq_raw_q2) >> 2;
    f_id = (s_id_filt_q2 >= 0) ? ((s_id_filt_q2 + 2) >> 2)
                               : -(((-s_id_filt_q2) + 2) >> 2);
    f_iq = (s_iq_filt_q2 >= 0) ? ((s_iq_filt_q2 + 2) >> 2)
                               : -(((-s_iq_filt_q2) + 2) >> 2);
  }
#endif

  active =
      (g_control_table.torque_enable == 1) && (g_control_table.calib_done == 1);

  /* PWM outputs must stay on during calibration (zero-offset sampling and
   * rotor alignment both inject current via PWM), even though active == 0
   * while calib_done == 0. */
  {
    int pwm_needed = active || (s_calib_state != CAL_STATE_IDLE);
    int prev_needed = s_prev_active; /* s_prev_active tracks pwm_needed */

    /* Rising / falling edge: toggle TIM1 MOE to physically cut gate-driver
     * signals. This eliminates any residual drive current felt when torque is
     * disabled (stiff feel at power-on or after torque-off command). The CC4
     * interrupt continues to run so ADC, encoder and auto-zero tracking remain
     * fully functional while PWM outputs are off. */
    if (pwm_needed && !prev_needed) {
      bsp_pwm_output_enable();
    } else if (!pwm_needed && prev_needed) {
      bsp_pwm_output_disable();
    }
    s_prev_active = (uint8_t)pwm_needed;
  }

#if MOTOR_USE_HALL_MT9102
  /* When motor is idle (torque disabled, stationary, not calibrating),
   * track 10-bit op-amp zero offset (bounded within +/-24 counts of EEPROM
   * offset) to eliminate switching-transient bias and thermal drift. */
  if (!active && s_calib_state == CAL_STATE_IDLE && s_speed == 0 &&
      g_control_table.calib_done == 1) {
    int32_t base_v = (int32_t)g_control_table.offset_v;
    int32_t base_u = (int32_t)g_control_table.offset_u;
    if (s_off_v_acc_q8 == 0) {
      s_off_v_acc_q8 = (int32_t)s_adc_v << 8;
    }
    if (s_off_u_acc_q8 == 0) {
      s_off_u_acc_q8 = (int32_t)s_adc_u << 8;
    }
    s_off_v_acc_q8 += (((int32_t)s_adc_v << 8) - s_off_v_acc_q8) >> 6;
    s_off_u_acc_q8 += (((int32_t)s_adc_u << 8) - s_off_u_acc_q8) >> 6;
    int16_t trim_v = (int16_t)((s_off_v_acc_q8 + 128) >> 8);
    int16_t trim_u = (int16_t)((s_off_u_acc_q8 + 128) >> 8);
    if (trim_v > base_v + 24)
      trim_v = (int16_t)(base_v + 24);
    else if (trim_v < base_v - 24)
      trim_v = (int16_t)(base_v - 24);
    if (trim_u > base_u + 24)
      trim_u = (int16_t)(base_u + 24);
    else if (trim_u < base_u - 24)
      trim_u = (int16_t)(base_u - 24);
    s_off_v = trim_v;
    s_off_u = trim_u;
  }
#else
  /* When motor is idle (torque disabled, stationary, not calibrating),
   * gently track 12-bit op-amp zero offset (bounded within +/-8 counts_12 of
   * EEPROM offset) to cancel thermal drift and 10-bit EEPROM rounding error.
   * ADC samples are valid even with PWM outputs off. */
  if (!active && s_calib_state == CAL_STATE_IDLE && s_speed == 0 &&
      g_control_table.calib_done == 1) {
    int32_t base_v_12 = (int32_t)s_off_v << 2;
    int32_t base_u_12 = (int32_t)s_off_u << 2;
    if (s_off_v_acc_q8 == 0) {
      s_off_v_acc_q8 = base_v_12 << 8;
    }
    if (s_off_u_acc_q8 == 0) {
      s_off_u_acc_q8 = base_u_12 << 8;
    }
    s_off_v_acc_q8 += (((int32_t)s_adc_v_12 << 8) - s_off_v_acc_q8) >> 8;
    s_off_u_acc_q8 += (((int32_t)s_adc_u_12 << 8) - s_off_u_acc_q8) >> 8;
    int16_t trim_v_12 = (int16_t)((s_off_v_acc_q8 + 128) >> 8);
    int16_t trim_u_12 = (int16_t)((s_off_u_acc_q8 + 128) >> 8);
    if (trim_v_12 > base_v_12 + 8)
      trim_v_12 = (int16_t)(base_v_12 + 8);
    else if (trim_v_12 < base_v_12 - 8)
      trim_v_12 = (int16_t)(base_v_12 - 8);
    if (trim_u_12 > base_u_12 + 8)
      trim_u_12 = (int16_t)(base_u_12 + 8);
    else if (trim_u_12 < base_u_12 - 8)
      trim_u_12 = (int16_t)(base_u_12 - 8);
    s_off_v_12 = trim_v_12;
    s_off_u_12 = trim_u_12;
  }
#endif

  {
    uint8_t mode = g_control_table.control_mode;

    if (mode != VEL_MODE_POSITION && s_prev_mode == VEL_MODE_POSITION) {
      g_control_table.moving_flag = 0;
      g_motor_status_flags &= (uint8_t)~ERR_FLAG_ANGLE;
    }
    if (mode == VEL_MODE_IMPEDANCE && s_prev_mode != VEL_MODE_IMPEDANCE) {
      s_imp_traj_init = 0;
    }
    s_prev_mode = mode;
  }

  if (!active) {
    f_integ_d = 0;
    f_integ_q = 0;
    f_integ_v = 0;
    f_vd = 0;
    f_vq = 0;
    f_iq_ref = 0; /* Ensure current reference is zeroed immediately */
    s_prev_target_speed = 0;
#if MOTOR_USE_HALL_MT9102
    s_pos_integ = 0;
#else
    s_id_filt_q2 = 0;
    s_iq_filt_q2 = 0;
    s_punch_kick_timer = 0;
    s_punch_kick_dir = 0;
#endif
    f_pos_target = s_multiturn;
    s_imp_hold = 0;
    s_imp_settle_cnt = 0;
    s_imp_traj_init = 0;
    s_imp_integ = 0;
    s_imp_motion_tail = 0;
    s_imp_prev_dir = 0;
    /* s_imp_tau_ff is a protocol shadow register (written from host) and is NOT
     * cleared here so the host value is preserved across enable/disable cycles.
     * It takes effect only when active==1, so it cannot cause residual current
     * while torque is disabled. The current PI above (f_vd/f_vq = 0) ensures
     * no drive voltage is applied regardless of f_iq_ref. */
  } else if (g_control_table.control_mode == VEL_MODE_IMPEDANCE) {
    /* -----------------------------------------------------------------------
     * IMPEDANCE / MIT-CHEETAH CONTROL MODE (14-Bit + Smooth Trajectory Filter)
     *   tau_ref = Kp*(p_set - p_cur) + Kd*(v_set - v_cur) + tau_fric + tau_ff
     *
     * 1. Full 14-bit rotor position resolution (16384 counts/rotor_rev) gives
     *    16x finer stiffness micro-stepping than 10-bit.
     * 2. Smooth Q8 trajectory generator (like ODrive TRAP_TRAJ + POS_FILTER):
     *    ramps acceleration smoothly, clamps cruise velocity to moving_speed,
     *    and filters incoming host steps (tau = 12.8 ms) with kinematically
     *    consistent velocity feedforward v_traj_rpm.
     * 3. Bounded stiction & cogging compensator (s_imp_integ) + active motion
     *    damping floor eliminates PD stick-slip ("khuc khuc") at low RPM while
     *    decaying to zero at rest to preserve pure MIT spring-damper
     * compliance.
     * -----------------------------------------------------------------------
     */
    int32_t p_err_14 = 0;
    int32_t v_err = 0;
    uint16_t kd_eff = s_imp_kd;

#if MOTOR_USE_HALL_MT9102
    /* Direct-drive Neck Joint: 20 kHz smooth trajectory + viscous damping +
     * non-hunting cogging/gravity compensator */
    int32_t vcmd_out = (int32_t)s_moving_speed_cmd;
    int32_t v_err_q4 = 0;

    if (s_imp_kp > 0) {
      int32_t target_14 = pos_out_to_rotor_14((int32_t)s_goal_position_cmd);
      int32_t target_q8 = target_14 << 8;

      if (!s_imp_traj_init) {
        s_imp_pos_q8 = s_multiturn_14 << 8;
        s_imp_vel_q8 = 0;
        s_imp_last_vlim = IMP_TRAJ_DEFAULT_ROTOR_RPM;
        s_imp_integ = 0;
        s_imp_motion_tail = 0;
        s_imp_prev_dir = 0;
        s_imp_traj_init = 1;
      }

      int32_t dist_q8 = target_q8 - s_imp_pos_q8;
      int32_t adist_q8 = (dist_q8 >= 0) ? dist_q8 : -dist_q8;
      int32_t vlim_rpm = (vcmd_out >= 0) ? vcmd_out : -vcmd_out;

      if (vlim_rpm != 0) {
        if (vlim_rpm < IMP_TRAJ_MIN_ROTOR_RPM) {
          vlim_rpm = IMP_TRAJ_MIN_ROTOR_RPM;
        } else if (vlim_rpm > IMP_TRAJ_MAX_ROTOR_RPM) {
          vlim_rpm = IMP_TRAJ_MAX_ROTOR_RPM;
        }
        s_imp_last_vlim = vlim_rpm;
      } else {
        if (adist_q8 > (128 << 8) && s_imp_vel_q8 == 0) {
          vlim_rpm = IMP_TRAJ_DEFAULT_ROTOR_RPM;
          s_imp_last_vlim = vlim_rpm;
        } else {
          vlim_rpm = s_imp_last_vlim;
        }
      }

      int32_t vmax_q8 = (vlim_rpm * 895) >> 8;
      int32_t vdes_mag_q8 = adist_q8 >> IMP_TRAJ_DECEL_SHIFT;
      if (vdes_mag_q8 < 16 && adist_q8 > 0) {
        vdes_mag_q8 = (adist_q8 > 16) ? 16 : adist_q8;
      }
      if (vdes_mag_q8 > vmax_q8) {
        vdes_mag_q8 = vmax_q8;
      }
      int32_t vdes_q8 = (dist_q8 >= 0) ? vdes_mag_q8 : -vdes_mag_q8;

      if (s_imp_vel_q8 < vdes_q8) {
        s_imp_vel_q8 += IMP_TRAJ_ACCEL_Q8;
        if (s_imp_vel_q8 > vdes_q8) {
          s_imp_vel_q8 = vdes_q8;
        }
      } else if (s_imp_vel_q8 > vdes_q8) {
        s_imp_vel_q8 -= IMP_TRAJ_ACCEL_Q8;
        if (s_imp_vel_q8 < vdes_q8) {
          s_imp_vel_q8 = vdes_q8;
        }
      }

      s_imp_pos_q8 += s_imp_vel_q8;
      if ((dist_q8 > 0 && s_imp_pos_q8 >= target_q8) ||
          (dist_q8 < 0 && s_imp_pos_q8 <= target_q8) || dist_q8 == 0) {
        s_imp_pos_q8 = target_q8;
        s_imp_vel_q8 = 0;
      }

      int32_t p_set_14 = (s_imp_pos_q8 + 128) >> 8;
      p_err_14 = p_set_14 - s_multiturn_14;
      if (p_err_14 > POS_ERR_14_CLAMP_MAX) {
        p_err_14 = POS_ERR_14_CLAMP_MAX;
      } else if (p_err_14 < -POS_ERR_14_CLAMP_MAX) {
        p_err_14 = -POS_ERR_14_CLAMP_MAX;
      }
      v_err_q4 = -s_speed_q4;

      int32_t goal_err_14 = target_14 - s_multiturn_14;
      /* Safe glide-path overspeed braking outside +/-180 counts_14 (4.0 deg)
       * with +96 Q4 (6 RPM) floor so full Kp stiffness is preserved on final
       * cogging slopes. */
      if (goal_err_14 > 180) {
        int32_t v_safe_q4 = ((goal_err_14 * 3) >> 2) + 96;
        if (s_speed_q4 > v_safe_q4) {
          int32_t over_q4 = s_speed_q4 - v_safe_q4;
          p_err_14 -= over_q4;
          if (p_err_14 < 0) {
            p_err_14 = 0;
          }
          v_err_q4 -= over_q4;
        }
      } else if (goal_err_14 < -180) {
        int32_t v_safe_q4 = -((((-goal_err_14) * 3) >> 2) + 96);
        if (s_speed_q4 < v_safe_q4) {
          int32_t over_q4 = v_safe_q4 - s_speed_q4;
          p_err_14 += over_q4;
          if (p_err_14 > 0) {
            p_err_14 = 0;
          }
          v_err_q4 += over_q4;
        }
      }
      /* Latch settled hold after 40 ms (800 ticks @ 20 kHz) of true stillness
       * (s_still_ticks >= 8U) inside +/-POS_DEADBAND_KD_COUNTS_14 so turnaround
       * peaks never false-trigger hold, and 0.1 deg Hall toggles at rest are
       * completely suppressed. */
      if (s_imp_vel_q8 == 0 && s_still_ticks >= 8U) {
        if (!s_imp_hold) {
          if (goal_err_14 >= -POS_DEADBAND_KD_COUNTS_14 &&
              goal_err_14 <= POS_DEADBAND_KD_COUNTS_14) {
            if (s_imp_settle_cnt < 800U) {
              s_imp_settle_cnt++;
            } else {
              s_imp_hold = 1;
              s_hold_perr_14 = goal_err_14;
              s_hold_raw_angle = s_raw_angle;
            }
          } else {
            s_imp_settle_cnt = 0;
          }
        } else {
          int32_t drift = goal_err_14 - s_hold_perr_14;
          if (goal_err_14 < -(POS_DEADBAND_KD_COUNTS_14 + 6) ||
              goal_err_14 > (POS_DEADBAND_KD_COUNTS_14 + 6) || drift < -6 ||
              drift > 6) {
            s_imp_hold = 0;
            s_imp_settle_cnt = 0;
          }
        }
      } else {
        s_imp_settle_cnt = 0;
        s_imp_hold = 0;
      }

      if (s_imp_hold && s_imp_vel_q8 == 0) {
        p_err_14 = s_hold_perr_14;
        v_err_q4 = 0;
      }
      if (s_imp_kp >= 500) {
        uint16_t kd_min = (s_imp_vel_q8 == 0) ? IMP_SETTLE_KD_MIN_ROTOR
                                              : IMP_MOTION_KD_MIN_ROTOR;
        if (kd_eff < kd_min) {
          kd_eff = kd_min;
        }
      }
      if (s_imp_vel_q8 != 0 ||
          (goal_err_14 > 0 && s_imp_integ < 0) ||
          (goal_err_14 < 0 && s_imp_integ > 0)) {
        s_imp_integ = 0;
      } else if (s_still_ticks == 0) {
        s_imp_integ -= (s_imp_integ >> 5);
      } else if (!s_imp_hold && s_still_ticks >= 9U &&
                 (goal_err_14 > POS_INTEG_FREEZE_COUNTS_14 ||
                  goal_err_14 < -POS_INTEG_FREEZE_COUNTS_14)) {
        int32_t step_i;
        if (goal_err_14 > 44 || goal_err_14 < -44) {
          step_i = goal_err_14 >> 2;
          if (step_i > 18)
            step_i = 18;
          else if (step_i < -18)
            step_i = -18;
        } else {
          step_i = goal_err_14 >> 3;
          if (step_i == 0) {
            step_i = (goal_err_14 > 0) ? 1 : -1;
          }
          if (step_i > 6)
            step_i = 6;
          else if (step_i < -6)
            step_i = -6;
        }
        s_imp_integ += step_i;
        if (s_imp_integ > IMP_FRIC_INTEG_LIMIT) {
          s_imp_integ = IMP_FRIC_INTEG_LIMIT;
        } else if (s_imp_integ < -IMP_FRIC_INTEG_LIMIT) {
          s_imp_integ = -IMP_FRIC_INTEG_LIMIT;
        }
      }

      if (p_err_14 > POS_ERR_14_CLAMP_MAX) {
        p_err_14 = POS_ERR_14_CLAMP_MAX;
      } else if (p_err_14 < -POS_ERR_14_CLAMP_MAX) {
        p_err_14 = -POS_ERR_14_CLAMP_MAX;
      }
    } else {
      /* Pure damping / torque mode (Kp == 0) */
      s_imp_pos_q8 = s_multiturn_14 << 8;
      s_imp_vel_q8 = 0;
      s_imp_integ = 0;
      s_imp_traj_init = 1;
      s_imp_hold = 0;
      s_imp_settle_cnt = 0;
      v_err_q4 = (vcmd_out << 4) - s_speed_q4;
    }
#else
    if (s_imp_kp > 0) {
      int32_t target_14 = pos_out_to_rotor_14((int32_t)s_goal_position_cmd);
      int32_t target_q8 = target_14 << 8;

      if (!s_imp_traj_init) {
        s_imp_pos_q8 = s_multiturn_14 << 8;
        s_imp_vel_q8 = 0;
        s_imp_last_vlim = IMP_TRAJ_DEFAULT_ROTOR_RPM;
        s_imp_integ = 0;
        s_imp_motion_tail = 0;
        s_imp_prev_dir = 0;
        s_imp_traj_init = 1;
      }

      int32_t dist_q8 = target_q8 - s_imp_pos_q8;
      int32_t adist_q8 = (dist_q8 >= 0) ? dist_q8 : -dist_q8;
      int32_t vcmd_out = (int32_t)s_moving_speed_cmd;
      int32_t vlim_rpm = (vcmd_out >= 0) ? vcmd_out : -vcmd_out;

      if (vlim_rpm != 0) {
        /* Scale output-shaft RPM command to rotor RPM with rounding */
        vlim_rpm =
            (vlim_rpm * MOTOR_GEAR_RATIO_NUM + (MOTOR_GEAR_RATIO_DEN / 2)) /
            MOTOR_GEAR_RATIO_DEN;
        if (vlim_rpm < IMP_TRAJ_MIN_ROTOR_RPM) {
          vlim_rpm = IMP_TRAJ_MIN_ROTOR_RPM;
        } else if (vlim_rpm > IMP_TRAJ_MAX_ROTOR_RPM) {
          vlim_rpm = IMP_TRAJ_MAX_ROTOR_RPM;
        }
        s_imp_last_vlim = vlim_rpm;
      } else {
        /* If host commands a large step (> 256 rotor counts_14) with v_cmd ==
         * 0, use default 600 RPM; otherwise preserve last smooth vlim to finish
         * settling without a sudden 600 RPM yank at the end of a smooth stream.
         */
        if (adist_q8 > (256 << 8) && s_imp_vel_q8 == 0) {
          vlim_rpm = IMP_TRAJ_DEFAULT_ROTOR_RPM;
          s_imp_last_vlim = vlim_rpm;
        } else {
          vlim_rpm = s_imp_last_vlim;
        }
      }

      /* 1 rotor RPM = 3.49525 Q8_counts/tick at 20 kHz (~895/256) */
      int32_t vmax_q8 = (vlim_rpm * 895) >> 8;

      /* Exponential smoothing & approach filter (tau = 256 ticks = 12.8 ms)
       * smoothly bridges 10..20 ms host streaming packets without stopping
       * mid-stream */
      int32_t vdes_mag_q8 = adist_q8 >> IMP_TRAJ_DECEL_SHIFT;
      if (vdes_mag_q8 == 0 && adist_q8 > 0) {
        vdes_mag_q8 =
            (adist_q8 > IMP_TRAJ_ACCEL_Q8) ? IMP_TRAJ_ACCEL_Q8 : adist_q8;
      }
      if (vdes_mag_q8 > vmax_q8) {
        vdes_mag_q8 = vmax_q8;
      }
      int32_t vdes_q8 = (dist_q8 >= 0) ? vdes_mag_q8 : -vdes_mag_q8;

      /* Acceleration-limited velocity update */
      if (s_imp_vel_q8 < vdes_q8) {
        s_imp_vel_q8 += IMP_TRAJ_ACCEL_Q8;
        if (s_imp_vel_q8 > vdes_q8) {
          s_imp_vel_q8 = vdes_q8;
        }
      } else if (s_imp_vel_q8 > vdes_q8) {
        s_imp_vel_q8 -= IMP_TRAJ_ACCEL_Q8;
        if (s_imp_vel_q8 < vdes_q8) {
          s_imp_vel_q8 = vdes_q8;
        }
      }

      /* Advance smooth position setpoint without overshooting target_q8 */
      s_imp_pos_q8 += s_imp_vel_q8;
      if ((dist_q8 > 0 && s_imp_pos_q8 >= target_q8) ||
          (dist_q8 < 0 && s_imp_pos_q8 <= target_q8) || dist_q8 == 0) {
        s_imp_pos_q8 = target_q8;
        s_imp_vel_q8 = 0;
      }

      int32_t p_set_14 = (s_imp_pos_q8 + 128) >> 8;
      int32_t v_traj_rpm = (s_imp_vel_q8 * 73) >> 8;
      p_err_14 = p_set_14 - s_multiturn_14;
      v_err = v_traj_rpm - s_speed;

      /* Active trajectory anti-stick-slip compensator:
       * Reset integrator on trajectory direction reversal, accumulate velocity
       * and position lag during active trajectory motion, and bleed to zero at
       * rest. */
      if (s_imp_vel_q8 != 0) {
        int8_t cur_dir = (s_imp_vel_q8 > 0) ? 1 : -1;
        if (s_imp_prev_dir != 0 && cur_dir != s_imp_prev_dir) {
          s_imp_integ = 0;
        }
        s_imp_prev_dir = cur_dir;
        s_imp_motion_tail = IMP_MOTION_TAIL_TICKS;
      } else if (s_imp_motion_tail > 0) {
        s_imp_motion_tail--;
      }

      if (s_imp_vel_q8 != 0 ||
          (s_imp_motion_tail > 0 && (p_err_14 > POS_DEADBAND_KD_COUNTS_14 ||
                                     p_err_14 < -POS_DEADBAND_KD_COUNTS_14))) {
        int32_t err_i = v_err + (p_err_14 >> 3);
        s_imp_integ += err_i;
        if (s_imp_integ > IMP_FRIC_INTEG_LIMIT) {
          s_imp_integ = IMP_FRIC_INTEG_LIMIT;
        } else if (s_imp_integ < -IMP_FRIC_INTEG_LIMIT) {
          s_imp_integ = -IMP_FRIC_INTEG_LIMIT;
        }
        if (kd_eff < IMP_MOTION_KD_MIN_ROTOR) {
          kd_eff = IMP_MOTION_KD_MIN_ROTOR;
        }
      } else {
        /* Smoothly bleed friction integrator to zero once settled at target */
        if (s_imp_integ > 0) {
          s_imp_integ = (s_imp_integ * 255) >> 8;
        } else if (s_imp_integ < 0) {
          s_imp_integ = -(((-s_imp_integ) * 255) >> 8);
        }
        if (s_imp_motion_tail == 0) {
          s_imp_prev_dir = 0;
        }
      }

      /* Settled hold: once trajectory is complete and rotor rests inside
       * +/-0.35 deg for 100 ms, mute Kd velocity noise while keeping smooth Kp
       */
      if (s_imp_vel_q8 == 0 && vcmd_out == 0 &&
          p_err_14 >= -POS_DEADBAND_KD_COUNTS_14 &&
          p_err_14 <= POS_DEADBAND_KD_COUNTS_14) {
        if (!s_imp_hold) {
          if (++s_imp_settle_cnt >= IMP_HOLD_SETTLE_TICKS) {
            s_imp_hold = 1;
          }
        }
      } else {
        s_imp_hold = 0;
        s_imp_settle_cnt = 0;
      }

      if (s_imp_hold) {
        v_err = 0;
      }

      /* Apply continuous position deadzone only when trajectory is stationary
       * so active slow rotation has zero stiffness deadband gap. */
      if (s_imp_vel_q8 == 0 && vcmd_out == 0 && s_imp_motion_tail == 0) {
        if (p_err_14 > POS_DEADBAND_KP_COUNTS_14) {
          p_err_14 -= POS_DEADBAND_KP_COUNTS_14;
        } else if (p_err_14 < -POS_DEADBAND_KP_COUNTS_14) {
          p_err_14 += POS_DEADBAND_KP_COUNTS_14;
        } else {
          p_err_14 = 0;
        }
      }

      if (p_err_14 > POS_ERR_14_CLAMP_MAX) {
        p_err_14 = POS_ERR_14_CLAMP_MAX;
      } else if (p_err_14 < -POS_ERR_14_CLAMP_MAX) {
        p_err_14 = -POS_ERR_14_CLAMP_MAX;
      }
    } else {
      /* Pure damping / torque mode (Kp == 0): track current rotor angle */
      s_imp_pos_q8 = s_multiturn_14 << 8;
      s_imp_vel_q8 = 0;
      s_imp_integ = 0;
      s_imp_motion_tail = 0;
      s_imp_prev_dir = 0;
      s_imp_traj_init = 1;
      s_imp_hold = 0;
      s_imp_settle_cnt = 0;
      v_err = ((int32_t)s_moving_speed_cmd * MOTOR_GEAR_RATIO_NUM) /
                  MOTOR_GEAR_RATIO_DEN -
              s_speed;
    }
#endif

    /* Combine Kp*(p_err_14 / 16) + Kd_eff*v_err + tau_fric + tau_ff */
    int32_t kp_term = ((int32_t)s_imp_kp * p_err_14) >> 4;
#if MOTOR_USE_HALL_MT9102
    (void)v_err;
    int32_t kd_term = ((int32_t)kd_eff * v_err_q4) >> 4;
    int32_t tau =
        (kp_term * 5 + kd_term + (s_imp_integ / 8)) / IMP_GAIN_SCALE_DIV +
        (int32_t)s_imp_tau_ff;
#else
    int32_t kd_term = (int32_t)kd_eff * v_err;
    int32_t i_term = s_imp_integ / IMP_FRIC_INTEG_DIV;
    int32_t tau = (kp_term + kd_term) / IMP_GAIN_SCALE_DIV + i_term +
                  (int32_t)s_imp_tau_ff;
#endif

    /* clamp to Iq limit */
    if (tau > IQ_LIMIT)
      tau = IQ_LIMIT;
    if (tau < -IQ_LIMIT)
      tau = -IQ_LIMIT;

    f_iq_ref = (int16_t)tau;
    f_integ_v = 0; /* velocity integrator not used in this mode */
    s_prev_target_speed = 0;
#if MOTOR_USE_HALL_MT9102
    s_pos_integ = 0;
    if (s_imp_vel_q8 == 0 && f_iq_ref == 0) {
      f_integ_d = 0;
      f_integ_q = 0;
      f_vd = 0;
      f_vq = 0;
    } else if (!s_imp_hold) {
      int32_t err_d = -f_id;
      int32_t err_q = (int32_t)f_iq_ref - f_iq;
      int32_t vd, vq;

      f_integ_d += CURRENT_KI_GAIN * err_d;
      f_integ_q += CURRENT_KI_GAIN * err_q;
      if (f_integ_d > CURRENT_D_INTEG_LIMIT)
        f_integ_d = CURRENT_D_INTEG_LIMIT;
      else if (f_integ_d < -CURRENT_D_INTEG_LIMIT)
        f_integ_d = -CURRENT_D_INTEG_LIMIT;
      if (f_integ_q >= CURRENT_INTEG_LIMIT)
        f_integ_q = CURRENT_INTEG_LIMIT;
      else if (f_integ_q <= -CURRENT_INTEG_LIMIT)
        f_integ_q = -CURRENT_INTEG_LIMIT;

      vd = (err_d * CURRENT_KP_GAIN + f_integ_d) / 100;
      vq = (err_q * CURRENT_KP_GAIN + f_integ_q) / 100;
      f_vd = (vd >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vd <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vd;
      f_vq = (vq >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vq <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vq;
    }
#else
    s_prev_target_speed = 0;
    s_punch_kick_timer = 0;
    s_punch_kick_dir = 0;

    /* current PI in 12-bit Q2 resolution (Id = 0 target) */
    {
      int32_t err_d_q2 = -s_id_filt_q2;
      int32_t err_q_q2 = ((int32_t)f_iq_ref << 2) - s_iq_filt_q2;
      int32_t vd, vq;

      f_integ_d += (CURRENT_KI_GAIN * err_d_q2) >> 2;
      f_integ_q += (CURRENT_KI_GAIN * err_q_q2) >> 2;
      if (f_integ_d > CURRENT_INTEG_LIMIT)
        f_integ_d = CURRENT_INTEG_LIMIT;
      else if (f_integ_d < -CURRENT_INTEG_LIMIT)
        f_integ_d = -CURRENT_INTEG_LIMIT;
      if (f_integ_q >= CURRENT_INTEG_LIMIT)
        f_integ_q = CURRENT_INTEG_LIMIT;
      else if (f_integ_q <= -CURRENT_INTEG_LIMIT)
        f_integ_q = -CURRENT_INTEG_LIMIT;

      vd = (((err_d_q2 * 401) >> 2) + f_integ_d) / 100;
      vq = (((err_q_q2 * 401) >> 2) + f_integ_q) / 100;
      f_vd = (vd >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vd <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vd;
      f_vq = (vq >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vq <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vq;
    }
#endif
  } else {
#if MOTOR_USE_HALL_MT9102
    /* Position Mode (VEL_MODE_POSITION = 2) */
    int32_t target_speed = s_moving_speed_cmd;
    int hold = pos_mode_plan(&target_speed);

    if (!hold) {
      /* closed loop speed */
      int16_t err;

      s_pos_integ = 0;
      s_imp_hold = 0;
      s_imp_settle_cnt = 0;
      f_pos_target = s_multiturn;
      err = (int16_t)(target_speed - s_speed);
      f_spd_err = err;
      if ((target_speed > 0 && s_prev_target_speed <= 0) ||
          (target_speed < 0 && s_prev_target_speed >= 0)) {
        f_integ_v = 0;
      }
      s_prev_target_speed = target_speed;
      if (target_speed > 0 && err < 0 && f_integ_v > 0) {
        f_integ_v += (int32_t)err * 4;
        if (f_integ_v < 0) {
          f_integ_v = 0;
        }
      } else if (target_speed < 0 && err > 0 && f_integ_v < 0) {
        f_integ_v += (int32_t)err * 4;
        if (f_integ_v > 0) {
          f_integ_v = 0;
        }
      } else {
        f_integ_v += err;
      }
      if (f_integ_v > SPEED_INTEG_LIMIT) {
        f_integ_v = SPEED_INTEG_LIMIT;
      } else if (f_integ_v < -SPEED_INTEG_LIMIT) {
        f_integ_v = -SPEED_INTEG_LIMIT;
      }
      f_iq_ref = (int16_t)(f_integ_v / 600 + err * 2);
      s_pos_integ = 0;
    } else {
      /* 14-bit position hold: tight PD + stationary-only cogging compensator */
      int32_t target_14 = pos_out_to_rotor_14((int32_t)s_goal_position_cmd);
      int32_t perr_14 = target_14 - s_multiturn_14;
      double d;

      f_pos_err = (int16_t)(perr_14 >> 4);
      s_prev_target_speed = 0;

      if ((perr_14 > 0 && s_pos_integ < 0) ||
          (perr_14 < 0 && s_pos_integ > 0)) {
        s_pos_integ = 0;
      } else if (s_still_ticks == 0) {
        s_pos_integ -= (s_pos_integ >> 5);
      }
      if (!s_imp_hold && s_still_ticks >= 9U &&
          (perr_14 > POS_INTEG_FREEZE_COUNTS_14 ||
           perr_14 < -POS_INTEG_FREEZE_COUNTS_14)) {
        int32_t step_p;
        if (perr_14 > 44 || perr_14 < -44) {
          step_p = perr_14 >> 2;
          if (step_p > 18)
            step_p = 18;
          else if (step_p < -18)
            step_p = -18;
        } else {
          step_p = perr_14 >> 3;
          if (step_p == 0) {
            step_p = (perr_14 > 0) ? 1 : -1;
          }
          if (step_p > 6)
            step_p = 6;
          else if (step_p < -6)
            step_p = -6;
        }
        s_pos_integ += step_p;
        if (s_pos_integ > IMP_FRIC_INTEG_LIMIT) {
          s_pos_integ = IMP_FRIC_INTEG_LIMIT;
        } else if (s_pos_integ < -IMP_FRIC_INTEG_LIMIT) {
          s_pos_integ = -IMP_FRIC_INTEG_LIMIT;
        }
      }

      if (s_still_ticks >= 8U) {
        if (!s_imp_hold) {
          if (perr_14 >= -POS_DEADBAND_KD_COUNTS_14 &&
              perr_14 <= POS_DEADBAND_KD_COUNTS_14) {
            if (s_imp_settle_cnt < 800U) {
              s_imp_settle_cnt++;
            } else {
              s_imp_hold = 1;
              s_hold_perr_14 = perr_14;
              s_hold_raw_angle = s_raw_angle;
            }
          } else {
            s_imp_settle_cnt = 0;
          }
        } else {
          int32_t drift = perr_14 - s_hold_perr_14;
          if (perr_14 < -(POS_DEADBAND_KD_COUNTS_14 + 6) ||
              perr_14 > (POS_DEADBAND_KD_COUNTS_14 + 6) || drift < -6 ||
              drift > 6) {
            s_imp_hold = 0;
            s_imp_settle_cnt = 0;
          }
        }
      } else {
        s_imp_settle_cnt = 0;
        s_imp_hold = 0;
      }

      int32_t spd_q4_eff = s_imp_hold ? 0 : s_speed_q4;
      int32_t perr_eff_14 = s_imp_hold ? s_hold_perr_14 : perr_14;
      if (!s_imp_hold) {
        if ((perr_eff_14 > 16 && spd_q4_eff > 0) ||
            (perr_eff_14 < -16 && spd_q4_eff < 0)) {
          perr_eff_14 /= 5;
        }
      }
      d = (double)((float)perr_eff_14 * (POS_HOLD_KP * 0.0625f) +
                   (float)s_pos_integ * 0.000125f) -
          (double)spd_q4_eff * (double)(POS_HOLD_KD * 0.0625f);
      f_iq_ref = (int16_t)(int32_t)d;
    }

    if (f_iq_ref > IQ_LIMIT) {
      f_iq_ref = IQ_LIMIT;
    } else if (f_iq_ref < -IQ_LIMIT) {
      f_iq_ref = -IQ_LIMIT;
    }

    /* current PI (Id ref = 0) */
    if (hold && (f_iq_ref == 0)) {
      f_integ_d = 0;
      f_integ_q = 0;
      f_vd = 0;
      f_vq = 0;
    } else if (!(hold && s_imp_hold)) {
      int32_t err_d = -f_id;
      int32_t err_q = (int32_t)f_iq_ref - f_iq;
      int32_t vd, vq;

      f_integ_d += CURRENT_KI_GAIN * err_d;
      f_integ_q += CURRENT_KI_GAIN * err_q;
      if (f_integ_d > CURRENT_D_INTEG_LIMIT)
        f_integ_d = CURRENT_D_INTEG_LIMIT;
      else if (f_integ_d < -CURRENT_D_INTEG_LIMIT)
        f_integ_d = -CURRENT_D_INTEG_LIMIT;
      if (f_integ_q >= CURRENT_INTEG_LIMIT)
        f_integ_q = CURRENT_INTEG_LIMIT;
      else if (f_integ_q <= -CURRENT_INTEG_LIMIT)
        f_integ_q = -CURRENT_INTEG_LIMIT;

      vd = (err_d * CURRENT_KP_GAIN + f_integ_d) / 100;
      vq = (err_q * CURRENT_KP_GAIN + f_integ_q) / 100;
      f_vd = vd;
      f_vq = vq;

      if (vd >= FOC_PWM_ARR)
        f_vd = FOC_PWM_ARR;
      else if (vd <= -FOC_PWM_ARR)
        f_vd = -FOC_PWM_ARR;
      if (vq >= FOC_PWM_ARR)
        f_vq = FOC_PWM_ARR;
      else if (vq <= -FOC_PWM_ARR)
        f_vq = -FOC_PWM_ARR;
    }
#else
    /* Position Mode (VEL_MODE_POSITION = 2) */
    int32_t target_speed = s_moving_speed_cmd;
    int hold = pos_mode_plan(&target_speed);

    if (!hold) {
      /* Closed-loop speed control */
      int16_t err;
      int16_t punch_val = 0;

      f_pos_target = s_multiturn;
      err = (int16_t)(target_speed - s_speed);
      f_spd_err = err;

      /* --- 2. INTEGRATOR DISCHARGE & KICKSTART TRIGGER ON DIRECTION REVERSAL
       * ---
       * - Reset f_integ_v = 0: Clears opposite integral torque from previous
       * direction.
       * - Arm punch timer s_punch_kick_timer = 500 (25 ms) along the new
       * direction. */
      if ((target_speed > 0 && s_prev_target_speed <= 0) ||
          (target_speed < 0 && s_prev_target_speed >= 0)) {
        f_integ_v = 0;
        s_punch_kick_timer = PUNCH_KICK_TICKS;
        s_punch_kick_dir = (target_speed > 0) ? 1 : -1;
      }
      s_prev_target_speed = target_speed;

      /* --- 3. TRANSIENT DECAYING PUNCH KICKSTART (Low-speed stiction break)
       * --- Injects a temporary kick torque decaying linearly from punch (~32
       * counts) to 0 over 25 ms to immediately overcome static breakaway
       * friction at low speed. Decays to zero once in motion so closed-loop PID
       * regulates cleanly without stepping jitter. */
      if (s_punch_kick_timer > 0) {
        int16_t pmax = (int16_t)g_control_table.punch;
        if (pmax > PUNCH_MAX) {
          pmax = PUNCH_MAX;
        }
        punch_val =
            (int16_t)(((int32_t)pmax * s_punch_kick_timer) / PUNCH_KICK_TICKS) *
            s_punch_kick_dir;
        s_punch_kick_timer--;
      }

      f_integ_v += err;
      if (f_integ_v > SPEED_INTEG_LIMIT) {
        f_integ_v = SPEED_INTEG_LIMIT;
      } else if (f_integ_v < -SPEED_INTEG_LIMIT) {
        f_integ_v = -SPEED_INTEG_LIMIT;
      }
      f_iq_ref = (int16_t)(f_integ_v / VEL_KI_DIV + err / 2 + punch_val);
    } else {
      /* Stop / Position Hold: Clear velocity integrator and kick state */
      f_integ_v = 0;
      s_prev_target_speed = 0;
      s_punch_kick_timer = 0;
      s_punch_kick_dir = 0;

      /* hold position (deadband +/-POS_DEADBAND_KP_COUNTS) */
      int16_t perr = (int16_t)(f_pos_target - s_multiturn);
      f_pos_err = perr;
      if (perr >= -POS_DEADBAND_KP_COUNTS && perr <= POS_DEADBAND_KP_COUNTS) {
        f_iq_ref = 0;
      } else {
        double d =
            (double)((float)perr * POS_HOLD_KP) - (double)s_speed * POS_HOLD_KD;
        f_iq_ref = (int16_t)(int32_t)d;
      }
    }

    if (f_iq_ref > IQ_LIMIT) {
      f_iq_ref = IQ_LIMIT;
    } else if (f_iq_ref < -IQ_LIMIT) {
      f_iq_ref = -IQ_LIMIT;
    }

    /* current PI in 12-bit Q2 resolution (Id ref = 0) */
    {
      int32_t err_d_q2 = -s_id_filt_q2;
      int32_t err_q_q2 = ((int32_t)f_iq_ref << 2) - s_iq_filt_q2;
      int32_t vd, vq;

      f_integ_d += (CURRENT_KI_GAIN * err_d_q2) >> 2;
      f_integ_q += (CURRENT_KI_GAIN * err_q_q2) >> 2;
      if (f_integ_d > CURRENT_INTEG_LIMIT)
        f_integ_d = CURRENT_INTEG_LIMIT;
      else if (f_integ_d < -CURRENT_INTEG_LIMIT)
        f_integ_d = -CURRENT_INTEG_LIMIT;
      if (f_integ_q >= CURRENT_INTEG_LIMIT)
        f_integ_q = CURRENT_INTEG_LIMIT;
      else if (f_integ_q <= -CURRENT_INTEG_LIMIT)
        f_integ_q = -CURRENT_INTEG_LIMIT;

      vd = (((err_d_q2 * 401) >> 2) + f_integ_d) / 100;
      vq = (((err_q_q2 * 401) >> 2) + f_integ_q) / 100;
      f_vd = vd;
      f_vq = vq;

      if (vd >= FOC_PWM_ARR)
        f_vd = FOC_PWM_ARR;
      else if (vd <= -FOC_PWM_ARR)
        f_vd = -FOC_PWM_ARR;
      if (vq >= FOC_PWM_ARR)
        f_vq = FOC_PWM_ARR;
      else if (vq <= -FOC_PWM_ARR)
        f_vq = -FOC_PWM_ARR;
    }
#endif
  }

  /* ---- voltage vector -> PWM ---- */
#if MOTOR_USE_HALL_MT9102
  if (s_calib_state == CAL_STATE_HALL_SWEEP) {
    /* Stage 1 (0 .. CAL_SWEEP_PRELOCK_TICKS): smooth Vd ramp + damped pre-lock
     * at elec = 0 so the rotor is synchronized before rotation starts.
     * Stage 2 (CAL_SWEEP_PRELOCK_TICKS+1 .. CAL_SWEEP_TICKS): trapezoidal
     * velocity profile over 7 electrical cycles (exact 1.000 mechanical turn),
     * accelerating from 0 and decelerating smoothly back to 0 at elec = 0. */
    int32_t vd_sweep = CAL_SWEEP_VOLT;
    int32_t vq_sweep = 0;
    if (s_cal_cnt < CAL_SWEEP_PRELOCK_TICKS) {
      s_sweep_elec = 0;
      s_sweep_elec_q8 = 0;
      if (s_cal_cnt < 2048U) {
        vd_sweep = ((int32_t)s_cal_cnt * CAL_SWEEP_PRELOCK_VOLT) >> 11;
      } else {
        vd_sweep = CAL_SWEEP_PRELOCK_VOLT -
                   (((int32_t)(s_cal_cnt - 2048U) *
                     (CAL_SWEEP_PRELOCK_VOLT - CAL_SWEEP_VOLT)) >>
                    11);
      }
      vq_sweep = -(s_speed_q4 * 2);
      if (vq_sweep > 60)
        vq_sweep = 60;
      else if (vq_sweep < -60)
        vq_sweep = -60;
    } else {
      int32_t rot_tick = (int32_t)s_cal_cnt - CAL_SWEEP_PRELOCK_TICKS + 1;
      int32_t step_q8 = CAL_SWEEP_STEP_Q8;
      if (rot_tick <= CAL_SWEEP_RAMP_TICKS) {
        step_q8 = (rot_tick * CAL_SWEEP_STEP_Q8) >> 12;
      } else if (rot_tick > (CAL_SWEEP_ROTATE_TICKS - CAL_SWEEP_RAMP_TICKS)) {
        int32_t rem = CAL_SWEEP_ROTATE_TICKS - rot_tick;
        if (rem < 0)
          rem = 0;
        step_q8 = (rem * CAL_SWEEP_STEP_Q8) >> 12;
      }
      s_sweep_elec_q8 += step_q8;
      if (s_sweep_elec_q8 >= (ELEC_ANGLE_MAX << 8)) {
        s_sweep_elec_q8 -= (ELEC_ANGLE_MAX << 8);
      }
      s_sweep_elec = s_sweep_elec_q8 >> 8;
    }
    foc_inv_park_svpwm(trig_cos(s_sweep_elec), trig_sin(s_sweep_elec),
                       vd_sweep, vq_sweep, f_duty);
  } else if (s_calib_state == CAL_STATE_ALIGN) {
    /* Two-sided symmetric rotor alignment at angle 0 (Vd = 300):
     * - First half (1 .. 8000): nudge +Vq (+100 -> 0), settle at upper
     *   stiction edge theta_plus = theta_pole + delta, average last 256 ticks.
     * - Second half (8001 .. 16000): nudge -Vq (-100 -> 0), settle at lower
     *   stiction edge theta_minus = theta_pole - delta, average last 256 ticks.
     * - Final angle = circular midpoint of (theta_plus, theta_minus), canceling
     *   bidirectional Coulomb static friction hysteresis identically to zero.
     */
    uint16_t half_ticks = ALIGN_TICKS / 2U;
    uint16_t phase_cnt =
        (s_align_cnt < half_ticks) ? s_align_cnt : (s_align_cnt - half_ticks);
    int32_t vq_mag = 0;
    int32_t vq_align = 0;

    s_align_ramp = ALIGN_RAMP_MAX;
    if (phase_cnt < 2048U) {
      vq_mag = 100;
    } else if (phase_cnt < 4096U) {
      vq_mag = ((int32_t)(4096U - phase_cnt) * 100) >> 11;
    }
    if (vq_mag > 0) {
      vq_align = (s_align_cnt < half_ticks) ? vq_mag : -vq_mag;
    } else if (phase_cnt < 6500U) {
      vq_align = -(s_speed_q4 * 2);
      if (vq_align > 60)
        vq_align = 60;
      else if (vq_align < -60)
        vq_align = -60;
    }
    foc_inv_park_svpwm(trig_cos(0), trig_sin(0), (int32_t)s_align_ramp,
                       vq_align, f_duty);

    s_align_cnt++;
    if (s_align_cnt > (half_ticks - ALIGN_AVG_SAMPLES) &&
        s_align_cnt <= half_ticks) {
      if (s_align_cnt == (half_ticks - ALIGN_AVG_SAMPLES + 1U)) {
        s_align_ref_raw = s_raw_angle;
        s_align_sum_raw = 0;
      }
      int32_t diff = (s_raw_angle - s_align_ref_raw) & 0x3FFF;
      if (diff > 8192) {
        diff -= 16384;
      }
      s_align_sum_raw += diff;
      if (s_align_cnt == half_ticks) {
        int32_t avg_diff = (s_align_sum_raw >= 0)
                               ? ((s_align_sum_raw + (ALIGN_AVG_SAMPLES / 2)) /
                                  ALIGN_AVG_SAMPLES)
                               : ((s_align_sum_raw - (ALIGN_AVG_SAMPLES / 2)) /
                                  ALIGN_AVG_SAMPLES);
        s_align_pos_plus = (s_align_ref_raw + avg_diff) & 0x3FFF;
        s_align_sum_raw = 0;
      }
    } else if (s_align_cnt > (ALIGN_TICKS - ALIGN_AVG_SAMPLES)) {
      if (s_align_cnt == (ALIGN_TICKS - ALIGN_AVG_SAMPLES + 1U)) {
        s_align_ref_raw = s_raw_angle;
        s_align_sum_raw = 0;
      }
      int32_t diff = (s_raw_angle - s_align_ref_raw) & 0x3FFF;
      if (diff > 8192) {
        diff -= 16384;
      }
      s_align_sum_raw += diff;
    }

    if (s_align_cnt >= ALIGN_TICKS) {
      int32_t avg_diff = (s_align_sum_raw >= 0)
                             ? ((s_align_sum_raw + (ALIGN_AVG_SAMPLES / 2)) /
                                ALIGN_AVG_SAMPLES)
                             : ((s_align_sum_raw - (ALIGN_AVG_SAMPLES / 2)) /
                                ALIGN_AVG_SAMPLES);
      int32_t angle_minus = (s_align_ref_raw + avg_diff) & 0x3FFF;
      int32_t diff_pm = (angle_minus - s_align_pos_plus) & 0x3FFF;
      if (diff_pm > 8192) {
        diff_pm -= 16384;
      }
      int32_t final_angle = (s_align_pos_plus + (diff_pm / 2)) & 0x3FFF;

      s_align_ramp = 0.0f;
      s_align_cnt = 0;
      s_align_sum_raw = 0;
      s_angle_off = final_angle;
      g_control_table.angle_offset = (uint16_t)final_angle;
      f_raw_prev = s_raw_angle;
      f_pos_prev = s_pos10;
      f_turns = (s_raw_angle > 8192) ? -1 : 0;
      s_rev_cnt = 0;
      s_multiturn = ((int32_t)f_turns << 10) + (int32_t)s_pos10;
      s_multiturn_14 = ((int32_t)f_turns << 14) + s_raw_angle;
      f_pos_target = s_multiturn;
      s_spd_hist_init = 0;
      s_speed_q4 = 0;
      s_still_ticks = 0;
      {
        int16_t cur_out = (int16_t)pos_rotor_to_out(s_multiturn_14);
        s_goal_position_cmd = cur_out;
        g_control_table.goal_position = cur_out;
        g_control_table.current_position = cur_out;
      }
      s_imp_traj_init = 0;
      s_calib_state = CAL_STATE_IDLE;
      g_control_table.calib_done = 1;
      g_eeprom_dirty_flag = 1;
    }
  } else {
    foc_inv_park_svpwm(tc, ts, f_vd, f_vq, f_duty);
  }
#else
  if (s_calib_state == CAL_STATE_ALIGN) {
    /* rotor alignment: Vd ramp (+0.04 per tick up to 400) at angle 0 */
    s_align_ramp += ALIGN_RAMP_STEP;
    if (s_align_ramp > ALIGN_RAMP_MAX) {
      s_align_ramp = ALIGN_RAMP_MAX;
    }
    foc_inv_park_svpwm(trig_cos(0), trig_sin(0), (int32_t)s_align_ramp, 0,
                       f_duty);

    s_align_cnt++;
    if (s_align_cnt > ALIGN_TICKS) {
      s_align_ramp = 0.0f;
      s_align_cnt = 0;
      s_angle_off = s_raw_angle;
      g_control_table.angle_offset = (uint16_t)s_raw_angle;
      s_calib_state = CAL_STATE_IDLE;
      g_control_table.calib_done = 1;
      g_eeprom_dirty_flag = 1;
    }
  } else {
    foc_inv_park_svpwm(tc, ts, f_vd, f_vq, f_duty);
  }
#endif

  switch (s_calib_state) {
  case CAL_STATE_ZERO_TOTAL:
    s_cal_cnt++;
    if (s_cal_cnt > CAL_SETTLE_TICKS) {
      s_cal_sum_t += bsp_adc_read(BSP_ADC_TOTAL_CURRENT);
    }
    if (s_cal_cnt >= CAL_TOTAL_TICKS) {
      g_control_table.offset_total = (int16_t)(s_cal_sum_t >> 10);
      s_cal_sum_t = 0;
      s_cal_cnt = 0;
      s_calib_state = CAL_STATE_ZERO_PHASE;
    }
    bsp_pwm_set_compare(0, 0, 0);
    break;

  case CAL_STATE_ZERO_PHASE:
    s_cal_cnt++;
#if MOTOR_USE_HALL_MT9102
    if (s_cal_cnt > CAL_SETTLE_TICKS) {
      s_cal_sum_v += s_adc_v;
      s_cal_sum_u += s_adc_u;
    }
    if (s_cal_cnt >= CAL_TOTAL_TICKS) {
      s_off_v = (int16_t)((s_cal_sum_v + 512) >> 10);
      s_off_u = (int16_t)((s_cal_sum_u + 512) >> 10);
      s_off_v_acc_q8 = (int32_t)s_off_v << 8;
      s_off_u_acc_q8 = (int32_t)s_off_u << 8;
      g_control_table.offset_v = s_off_v;
      g_control_table.offset_u = s_off_u;
      s_cal_sum_v = 0;
      s_cal_sum_u = 0;
      s_cal_cnt = 0;
      s_sweep_elec = 0;
      s_sweep_elec_q8 = 0;
      s_min_sin = 4095;
      s_max_sin = 0;
      s_min_cos = 4095;
      s_max_cos = 0;
      s_calib_state = CAL_STATE_HALL_SWEEP;
    }
    bsp_pwm_set_compare(CAL_PHASE_DUTY, CAL_PHASE_DUTY, CAL_PHASE_DUTY);
#else
    if (s_cal_cnt > CAL_SETTLE_TICKS) {
      s_cal_sum_v += s_adc_v_12;
      s_cal_sum_u += s_adc_u_12;
    }
    if (s_cal_cnt >= CAL_TOTAL_TICKS) {
      s_off_v_12 = (int16_t)((s_cal_sum_v + 512) >> 10);
      s_off_u_12 = (int16_t)((s_cal_sum_u + 512) >> 10);
      s_off_v = (int16_t)((s_off_v_12 + 2) >> 2);
      s_off_u = (int16_t)((s_off_u_12 + 2) >> 2);
      s_off_v_acc_q8 = (int32_t)s_off_v_12 << 8;
      s_off_u_acc_q8 = (int32_t)s_off_u_12 << 8;
      g_control_table.offset_v = s_off_v;
      g_control_table.offset_u = s_off_u;
      s_cal_sum_v = 0;
      s_cal_sum_u = 0;
      s_cal_cnt = 0;
      s_calib_state = CAL_STATE_ALIGN;
    }
    bsp_pwm_set_compare(CAL_PHASE_DUTY, CAL_PHASE_DUTY, CAL_PHASE_DUTY);
#endif
    break;

#if MOTOR_USE_HALL_MT9102
  case CAL_STATE_HALL_SWEEP:
    s_cal_cnt++;
    if (s_cal_cnt > CAL_SWEEP_PRELOCK_TICKS) {
      uint16_t ha = bsp_adc_read(BSP_ADC_HALL_SIN);
      uint16_t hb = bsp_adc_read(BSP_ADC_HALL_COS);
      if (ha < s_min_sin)
        s_min_sin = ha;
      if (ha > s_max_sin)
        s_max_sin = ha;
      if (hb < s_min_cos)
        s_min_cos = hb;
      if (hb > s_max_cos)
        s_max_cos = hb;
    }
    if (s_cal_cnt >= CAL_SWEEP_TICKS) {
      uint16_t off_sin = (s_min_sin + s_max_sin) >> 1;
      uint16_t off_cos = (s_min_cos + s_max_cos) >> 1;
      uint16_t span_sin = s_max_sin - s_min_sin;
      uint16_t span_cos = s_max_cos - s_min_cos;
      if (span_sin > 2500 && span_cos > 2500 && off_sin > 1600 &&
          off_sin < 2400 && off_cos > 1600 && off_cos < 2400) {
        g_control_table.hall_offset_sin = off_sin;
        g_control_table.hall_offset_cos = off_cos;
        g_control_table.hall_gain_sin_num = span_cos;
        g_control_table.hall_gain_sin_den = span_sin;
        g_control_table.hall_calib_done = 1;
        bsp_mt9102_set_calibration(off_sin, off_cos, span_cos, span_sin);
      }
      s_cal_cnt = 0;
      s_align_ramp = (float)CAL_SWEEP_VOLT;
      s_calib_state = CAL_STATE_ALIGN;
    }
#if FOC_INVERT_PHASE_UV
    bsp_pwm_set_compare(FOC_PWM_ARR - f_duty[0], FOC_PWM_ARR - f_duty[2],
                        FOC_PWM_ARR - f_duty[1]);
#else
    bsp_pwm_set_compare(FOC_PWM_ARR - f_duty[0], FOC_PWM_ARR - f_duty[1],
                        FOC_PWM_ARR - f_duty[2]);
#endif
    break;
#endif

  default:
#if FOC_INVERT_PHASE_UV
    bsp_pwm_set_compare(FOC_PWM_ARR - f_duty[0], FOC_PWM_ARR - f_duty[2],
                        FOC_PWM_ARR - f_duty[1]);
#else
    bsp_pwm_set_compare(FOC_PWM_ARR - f_duty[0], FOC_PWM_ARR - f_duty[1],
                        FOC_PWM_ARR - f_duty[2]);
#endif
    break;
  }
}

/* ------------------------------------------------------------------------------------------
 *  Public interface
 * ----------------------------------------------------------------------------------------
 */

/* 0x0800330C body: read angle -> angle/speed -> FOC step */
void motor_ctrl_fast_loop(int32_t raw_angle) {
  s_raw_angle = raw_angle;
  angle_speed();
  foc_step();
}

/* Called after the control table has been loaded from flash */
void motor_ctrl_load_calibration(void) {
  s_off_v = g_control_table.offset_v;
  s_off_u = g_control_table.offset_u;
#if !MOTOR_USE_HALL_MT9102
  s_off_v_12 = (int16_t)(s_off_v << 2);
  s_off_u_12 = (int16_t)(s_off_u << 2);
  s_off_v_acc_q8 = (int32_t)s_off_v_12 << 8;
  s_off_u_acc_q8 = (int32_t)s_off_u_12 << 8;
#endif
  s_angle_off = (int32_t)g_control_table.angle_offset;
#if MOTOR_USE_HALL_MT9102
  if (g_control_table.hall_calib_done == 1) {
    bsp_mt9102_set_calibration(
        g_control_table.hall_offset_sin, g_control_table.hall_offset_cos,
        g_control_table.hall_gain_sin_num, g_control_table.hall_gain_sin_den);
  }
  if (g_control_table.angle_offset != 0 ||
      g_control_table.hall_calib_done == 1) {
    g_control_table.calib_done = 1;
  } else {
    g_control_table.calib_done = 0;
  }
#endif
  s_imp_kp = si_kp_to_rotor(g_control_table.kp);
  s_imp_kd = si_kd_to_rotor(g_control_table.kd);
  s_imp_tau_ff = si_tau_to_iq(g_control_table.tau_ff);
}

void motor_ctrl_set_speed_cmd(int16_t speed) { s_moving_speed_cmd = speed; }

void motor_ctrl_set_goal_position(int16_t position) {
  s_goal_position_cmd = position;
}

void motor_ctrl_set_kp(uint16_t kp) { s_imp_kp = si_kp_to_rotor(kp); }

void motor_ctrl_set_kd(uint16_t kd) { s_imp_kd = si_kd_to_rotor(kd); }

void motor_ctrl_set_tau_ff(int16_t tau_ff) {
  s_imp_tau_ff = si_tau_to_iq(tau_ff);
}

bool motor_ctrl_start_calibration(void) {
  if (s_calib_state != CAL_STATE_IDLE) {
    return false;
  }
  g_control_table.calib_done = 0;
#if MOTOR_USE_HALL_MT9102
  g_control_table.torque_enable = 0;
  s_cal_cnt = 0;
  s_cal_sum_t = 0;
  s_cal_sum_v = 0;
  s_cal_sum_u = 0;
  s_align_cnt = 0;
  s_align_ramp = 0.0f;
  s_sweep_elec = 0;
  s_sweep_elec_q8 = 0;
  s_min_sin = 4095;
  s_max_sin = 0;
  s_min_cos = 4095;
  s_max_cos = 0;
  s_align_sum_raw = 0;
  s_align_ref_raw = 0;
  s_align_pos_plus = 0;
  /* Ensure baseline Hall calibration is valid while measuring pre-lock/sweep */
  if (g_control_table.hall_offset_sin < 1600 ||
      g_control_table.hall_offset_sin > 2400 ||
      g_control_table.hall_offset_cos < 1600 ||
      g_control_table.hall_offset_cos > 2400 ||
      g_control_table.hall_gain_sin_num < 2500 ||
      g_control_table.hall_gain_sin_den < 2500) {
    g_control_table.hall_offset_sin = 2006U;
    g_control_table.hall_offset_cos = 1997U;
    g_control_table.hall_gain_sin_num = 3447U;
    g_control_table.hall_gain_sin_den = 3859U;
    bsp_mt9102_set_calibration(2006U, 1997U, 3447U, 3859U);
  }
#endif
  s_calib_state = CAL_STATE_ZERO_TOTAL;
  return true;
}

int16_t motor_ctrl_get_position(void) {
  int32_t out_pos = pos_rotor_to_out(s_multiturn_14);
  if (out_pos > POSITION_MAX)
    return POSITION_MAX;
  if (out_pos < POSITION_MIN)
    return POSITION_MIN;
  return (int16_t)out_pos;
}

int32_t motor_ctrl_get_speed(void) { return s_speed; }

#if FEATURE_UART_STREAM_TELEMETRY
int16_t motor_ctrl_get_iu(void) { return s_cur_iu; }
int16_t motor_ctrl_get_iv(void) { return s_cur_iv; }
int16_t motor_ctrl_get_iw(void) { return s_cur_iw; }
int32_t motor_ctrl_get_elec_angle(void) { return f_elec; }
#endif
