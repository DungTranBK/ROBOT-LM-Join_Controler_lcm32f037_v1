/**
 * @file motor_ctrl.c
 * @brief [Middleware] FOC BLDC motor control loop (LCM32F037, Cortex-M0)
 *
 *   - angle_speed(): Multi-turn rotor position tracking and speed estimation
 *   - foc_step(): Field-oriented current/velocity/position/impedance control
 *   - motor_ctrl_fast_loop(): High-frequency 20 kHz FOC control loop entry point
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
 *  Control loop constants
 * ----------------------------------------------------------------------------------------
 */
#define ELEC_ANGLE_MAX 0xF56F /* 62831 = 2*pi*10000 */
#define ELEC_RATIO_NUM 0xEFA  /* 3834  */
#define ELEC_ALIGN_STEP 10    /* electrical angle step per tick during align  */
#define SPEED_FILTER_CALLS 10
#define SPEED_INTEG_LIMIT                                                      \
  1000000 /* Velocity PI integrator saturation limit      */
#define CURRENT_INTEG_LIMIT                                                    \
  0x1FBD0 /* Current PI integrator saturation limit       */
#define IQ_LIMIT 300
#define VEL_KI_DIV 3000 /* Velocity integral gain divisor               */

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
 *  Internal state structures
 * ----------------------------------------------------------------------------------------
 */

/**
 * @brief Host command and MIT-Cheetah impedance shadow registers.
 *        Written from protocol context (converted from output-shaft SI units
 *        into rotor-side fixed-point gains) and read by foc_step() at 20 kHz.
 */
typedef struct {
  volatile int16_t moving_speed;  /* Commanded moving speed (RPM) */
  volatile int16_t goal_position; /* Latched Goal Position (output counts) */
  volatile uint16_t imp_kp;       /* Rotor stiffness gain [x0.001 Iq/rotor_count] */
  volatile uint16_t imp_kd;       /* Rotor damping gain   [x0.001 Iq/rotor_RPM]   */
  volatile int16_t imp_tau_ff;    /* Rotor feedforward    [Iq counts, signed]      */
} foc_cmd_t;

/**
 * @brief Multi-turn rotor position tracking and velocity estimation state.
 */
typedef struct {
  int32_t raw_angle;    /* Raw 14-bit sensor angle (0..16383) */
  int32_t angle_off;    /* Electrical zero angle offset from calibration */
  uint16_t pos10;       /* Single-turn 10-bit rotor position (0..1023) */
  uint16_t pos_prev;    /* Previous 10-bit rotor position for wrap detection */
  int16_t turns;        /* Accumulated rotor revolution count (10-bit wrap) */
  int16_t spd_cnt;      /* Speed decimation sample counter */
  int32_t multiturn;    /* Multi-turn rotor position (1024 counts/rotor_rev) */
  int32_t multiturn_14; /* Full 14-bit multi-turn rotor angle (16384 counts/rev) */
  int32_t rev_cnt;      /* Full revolution counter (14-bit wrap) */
  int32_t raw_prev;     /* Previous 14-bit raw angle sample */
  int32_t delta;        /* Per-tick 14-bit raw angle delta */
  int32_t delta_acc;    /* Accumulated raw angle delta over speed window */
  int32_t speed;        /* Estimated rotor speed (RPM) */
#if MOTOR_USE_HALL_MT9102
  int32_t pos14_filt_q8;  /* Low-pass filtered 14-bit position (Q8) */
  int32_t pos14_hist[16]; /* Ring buffer of filtered Q8 positions for 4 ms velocity */
  uint8_t spd_hist_idx;   /* Current index into pos14_hist[] ring buffer */
  uint8_t spd_hist_init;  /* Non-zero once velocity history ring buffer is primed */
  int32_t speed_q4;       /* High-resolution rotor speed in Q4 RPM (1/16 RPM LSB) */
  uint16_t still_ticks;   /* Consecutive 0.5 ms windows of true rotor stillness */
#endif
} foc_encoder_t;

/**
 * @brief Phase current ADC sampling, idle auto-zero offset tracker, electrical
 *        angle, and DQ-frame current PI regulator state.
 */
typedef struct {
  int16_t adc_v;        /* Phase V current ADC sample (10-bit scale) */
  int16_t adc_u;        /* Phase U current ADC sample (10-bit scale) */
  int16_t off_v;        /* 10-bit phase V zero-current offset */
  int16_t off_u;        /* 10-bit phase U zero-current offset */
  int32_t off_v_acc_q8; /* Sub-LSB idle auto-zero offset accumulator (Q8) */
  int32_t off_u_acc_q8; /* Sub-LSB idle auto-zero offset accumulator (Q8) */
#if !MOTOR_USE_HALL_MT9102
  int16_t adc_v_12;     /* Full 12-bit phase V ADC sample (0..4095) */
  int16_t adc_u_12;     /* Full 12-bit phase U ADC sample (0..4095) */
  int16_t off_v_12;     /* Full 12-bit phase V zero offset (Q2) */
  int16_t off_u_12;     /* Full 12-bit phase U zero offset (Q2) */
  int32_t id_filt_q2;   /* 20 kHz EMA-filtered Id in 12-bit Q2 scale */
  int32_t iq_filt_q2;   /* 20 kHz EMA-filtered Iq in 12-bit Q2 scale */
#endif
#if FEATURE_UART_STREAM_TELEMETRY
  volatile int16_t cur_iu; /* Telemetry phase U current */
  volatile int16_t cur_iv; /* Telemetry phase V current */
  volatile int16_t cur_iw; /* Telemetry phase W current */
#endif
  int32_t elec;    /* Electrical angle in 1/10000 rad (0..62831) */
  int16_t iq_ref;  /* Quadrature-axis current reference (Iq) */
  int32_t id;      /* Measured D-axis current */
  int32_t iq;      /* Measured Q-axis current */
  int32_t integ_d; /* D-axis current PI integrator accumulator */
  int32_t integ_q; /* Q-axis current PI integrator accumulator */
  int32_t vd;      /* D-axis voltage command */
  int32_t vq;      /* Q-axis voltage command */
  int32_t duty[3]; /* Three-phase SVPWM compare duty values */
} foc_current_t;

/**
 * @brief Outer-loop Position, Velocity, and MIT Impedance trajectory/hold state.
 */
typedef struct {
  uint8_t pos_hold;          /* Non-zero when Mode 2 is in position-hold window */
  uint8_t prev_mode;         /* Previous control_mode for transition edge detection */
  uint8_t prev_active;       /* Previous PWM output active state for MOE edge toggle */
  uint8_t imp_hold;          /* Non-zero when settled hold lock is latched */
  uint16_t imp_settle_cnt;   /* Stationary settling tick counter before hold lock */
  uint8_t imp_traj_init;     /* Non-zero when Mode 3 Q8 trajectory generator is primed */
  int8_t imp_prev_dir;       /* Trajectory direction (+1, -1, 0) for reversal reset */
  uint16_t imp_motion_tail;  /* Landing countdown ticks after trajectory stops */
  int16_t spd_err;           /* Velocity tracking error (target_speed - speed) */
  int16_t pos_err;           /* Position tracking error (10-bit scale) */
  int32_t pos_target;        /* Target rotor position in 10-bit multi-turn counts */
  int32_t integ_v;           /* Velocity PI integrator accumulator */
  int32_t prev_target_speed; /* Previous tick target speed for reversal detection */
  int32_t imp_pos_q8;        /* Smoothed 14-bit rotor position setpoint (Q8) */
  int32_t imp_vel_q8;        /* Smoothed trajectory velocity (Q8 counts/tick) */
  int32_t imp_last_vlim;     /* Last active rotor RPM limit during smooth move */
  int32_t imp_integ;         /* Bounded stiction & cogging torque integrator */
#if MOTOR_USE_HALL_MT9102
  int32_t pos_integ;         /* Stationary cogging integrator in Mode 2 hold */
  int32_t hold_perr_14;      /* Latched 14-bit position error during settled hold */
  int32_t hold_raw_angle;    /* Latched 14-bit electrical commutation angle in hold */
#else
  int16_t punch_kick_timer;  /* Countdown timer for decaying punch pulse (500 -> 0) */
  int16_t punch_kick_dir;    /* Direction of punch kick (+1: forward, -1: reverse) */
#endif
} foc_ctrl_t;

/**
 * @brief Calibration state machine (current zero offsets, Hall 360-deg sweep,
 *        and rotor pole alignment).
 */
typedef struct {
  volatile uint8_t state;  /* Active calibration state (CAL_STATE_*) */
  uint16_t cal_cnt;        /* Tick counter for offset / Hall sweep phases */
  uint16_t align_cnt;      /* Tick counter for rotor alignment phase */
  float align_ramp;        /* Voltage ramp level during rotor alignment */
  int32_t sum_v;           /* Accumulator for phase V zero-offset calibration */
  int32_t sum_u;           /* Accumulator for phase U zero-offset calibration */
  int32_t sum_t;           /* Accumulator for total shunt zero-offset calibration */
#if MOTOR_USE_HALL_MT9102
  int32_t sweep_elec;      /* Open-loop electrical angle during Hall sweep */
  int32_t sweep_elec_q8;   /* Q8 fractional electrical angle during Hall sweep */
  uint16_t min_sin;        /* Minimum raw SIN ADC value observed during sweep */
  uint16_t max_sin;        /* Maximum raw SIN ADC value observed during sweep */
  uint16_t min_cos;        /* Minimum raw COS ADC value observed during sweep */
  uint16_t max_cos;        /* Maximum raw COS ADC value observed during sweep */
  int32_t align_sum_raw;   /* Circular angle difference accumulator during align */
  int32_t align_ref_raw;   /* Reference raw angle for circular averaging */
  int32_t align_pos_plus;  /* Latched +Vq stiction edge angle during two-sided align */
#endif
} foc_calib_t;

static foc_cmd_t s_cmd = {
    .moving_speed = 0,
    .goal_position = 0,
    .imp_kp =
        ((uint32_t)DEFAULT_KP * SI_KP_TO_ROTOR_NUM + (SI_KP_TO_ROTOR_DEN / 2U)) /
        SI_KP_TO_ROTOR_DEN,
    .imp_kd =
        ((uint32_t)DEFAULT_KD * SI_KD_TO_ROTOR_NUM + (SI_KD_TO_ROTOR_DEN / 2U)) /
        SI_KD_TO_ROTOR_DEN,
    .imp_tau_ff = 0,
};

static foc_encoder_t s_enc;
static foc_current_t s_foc;
static foc_ctrl_t s_ctrl;

static foc_calib_t s_cal = {
    .state = CAL_STATE_IDLE,
#if MOTOR_USE_HALL_MT9102
    .min_sin = 4095,
    .max_sin = 0,
    .min_cos = 4095,
    .max_cos = 0,
#endif
};

/**
 * @func    angle_speed
 * @brief   Track multi-turn rotor position and estimate rotor speed in RPM at 20 kHz.
 * @param   None
 * @retval  None
 */
static void angle_speed(void) {
  int32_t raw = s_enc.raw_angle;
  int32_t delta = raw - s_enc.raw_prev;
  int16_t pos = (int16_t)(raw >> 4);
  int16_t diff;

  s_enc.delta = delta;
  s_enc.pos10 = (uint16_t)pos;
  diff = (int16_t)(pos - s_enc.pos_prev);

  if (diff > POS_WRAP_DETECT) {
    s_enc.turns = (int16_t)(s_enc.turns - 1);
  } else if (diff < -POS_WRAP_DETECT) {
    s_enc.turns = (int16_t)(s_enc.turns + 1);
  }
  s_enc.pos_prev = (uint16_t)pos;
  s_enc.multiturn = ((int32_t)s_enc.turns << 10) + pos;
  s_enc.multiturn_14 = ((int32_t)s_enc.turns << 14) + raw;

  if (delta > ENC_WRAP_DETECT) {
    s_enc.rev_cnt--;
    s_enc.delta = delta - 16383;
  } else if (delta < -ENC_WRAP_DETECT) {
    s_enc.rev_cnt++;
    s_enc.delta = delta + 16383;
  }

  s_enc.delta_acc += s_enc.delta;
  s_enc.raw_prev = raw;

  s_enc.spd_cnt = (int16_t)(s_enc.spd_cnt + 1);
#if MOTOR_USE_HALL_MT9102
  if (!s_enc.spd_hist_init) {
    s_enc.pos14_filt_q8 = s_enc.multiturn_14 << 8;
    for (int i = 0; i < 16; i++) {
      s_enc.pos14_hist[i] = s_enc.pos14_filt_q8;
    }
    s_enc.spd_hist_idx = 0;
    s_enc.spd_hist_init = 1;
  } else {
    s_enc.pos14_filt_q8 += ((s_enc.multiturn_14 << 8) - s_enc.pos14_filt_q8) >> 3;
  }
  if (s_enc.spd_cnt == SPEED_FILTER_CALLS) {
    s_enc.spd_cnt = 0;
    int32_t delta_4ms_q8 =
        s_enc.pos14_filt_q8 - s_enc.pos14_hist[s_enc.spd_hist_idx];
    s_enc.pos14_hist[s_enc.spd_hist_idx] = s_enc.pos14_filt_q8;
    s_enc.spd_hist_idx = (uint8_t)((s_enc.spd_hist_idx + 1U) & 0x07U);

    int32_t acc_q8 =
        (delta_4ms_q8 >= -320 && delta_4ms_q8 <= 320) ? 0 : delta_4ms_q8;
    int32_t raw_spd_q4 = (acc_q8 * 3750) >> 16;
    s_enc.speed_q4 = (s_enc.speed_q4 + raw_spd_q4) >> 1;
    if (acc_q8 == 0 && s_enc.speed_q4 >= -16 && s_enc.speed_q4 <= 16) {
      s_enc.speed_q4 = 0;
    }
    if (s_enc.speed_q4 >= -32 && s_enc.speed_q4 <= 32) {
      s_enc.speed = 0;
    } else {
      s_enc.speed = (s_enc.speed_q4 >= 0) ? ((s_enc.speed_q4 + 8) >> 4)
                                          : -(((-s_enc.speed_q4) + 8) >> 4);
    }
    if (delta_4ms_q8 >= -512 && delta_4ms_q8 <= 512) {
      if (s_enc.still_ticks < 10000U) {
        s_enc.still_ticks++;
      }
    } else {
      s_enc.still_ticks = 0;
    }
    s_enc.delta_acc = 0;
  }
#else
  if (s_enc.spd_cnt == SPEED_FILTER_CALLS) {
    /* Suppress 14-bit magnetic encoder thermal LSB toggle only when stationary;
     * preserve every single count during active slow rotation. */
    int32_t acc =
        (s_enc.speed == 0 && s_enc.delta_acc >= -ENC_SPEED_DEADBAND_RAW &&
         s_enc.delta_acc <= ENC_SPEED_DEADBAND_RAW)
            ? 0
            : s_enc.delta_acc;
    int32_t raw_spd = (int32_t)((float)acc * SPEED_RPM_PER_COUNT);
    s_enc.spd_cnt = 0;
    /* First-order EMA low-pass filter (7/8 weight, tau ~ 3.5 ms) */
    s_enc.speed =
        (s_enc.speed * SPEED_EMA_PREV_WEIGHT + raw_spd) / SPEED_EMA_DIV;
    if (raw_spd == 0 && s_enc.speed >= -SPEED_ZERO_SNAP_RPM &&
        s_enc.speed <= SPEED_ZERO_SNAP_RPM) {
      s_enc.speed = 0;
    }
    s_enc.delta_acc = 0;
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

/**
 * @func    si_kp_to_rotor
 * @brief   Convert output-shaft SI stiffness gain Kp (x0.001 N.m/rad) to rotor fixed-point gain.
 * @param   kp_si: Stiffness gain in output-shaft SI register units
 * @retval  Rotor-side fixed-point stiffness gain [x0.001 Iq/rotor_count]
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

/**
 * @func    si_kd_to_rotor
 * @brief   Convert output-shaft SI damping gain Kd (x0.001 N.m.s/rad) to rotor fixed-point gain.
 * @param   kd_si: Damping gain in output-shaft SI register units
 * @retval  Rotor-side fixed-point damping gain [x0.001 Iq/rotor_RPM]
 */
static inline uint16_t si_kd_to_rotor(uint16_t kd_si) {
  if (kd_si > MOTOR_KD_MAX) {
    kd_si = MOTOR_KD_MAX;
  }
  uint32_t v =
      ((uint32_t)kd_si * SI_KD_TO_ROTOR_NUM + (SI_KD_TO_ROTOR_DEN / 2U)) /
      SI_KD_TO_ROTOR_DEN;
  return (v > 65535U) ? 65535U : (uint16_t)v;
}

/**
 * @func    si_tau_to_iq
 * @brief   Convert output-shaft feedforward torque (mN.m) to clamped Iq current reference counts.
 * @param   tau_mNm: Feedforward torque in mN.m
 * @retval  Signed Iq current reference clamped to [-IQ_LIMIT, +IQ_LIMIT]
 */
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
#else
#define POS_HOLD_ENTER 2 /* |err| <= 2 counts: enter hold */
#define POS_HOLD_EXIT 6  /* |err| >  6 counts: leave hold (hysteresis) */
#endif

/* Gearbox conversion:
 * - Output shaft register: 16384 counts/rev (14-bit)
 * - Internal rotor loop (s_enc.multiturn): 1024 counts/rotor_rev
 * - Full 14-bit rotor tracker (s_enc.multiturn_14): 16384 counts/rotor_rev
 * - Gearbox ratio: MOTOR_GEAR_RATIO_NUM / MOTOR_GEAR_RATIO_DEN (15 / 2 = 7.5)
 */
#define OUT_TO_ROTOR_DEN                                                       \
  (MOTOR_GEAR_RATIO_DEN * (POSITION_COUNTS_PER_REV / ROTOR_COUNTS_PER_REV))

/**
 * @func    pos_out_to_rotor
 * @brief   Convert 14-bit output-shaft position counts to 10-bit rotor multi-turn counts.
 * @param   out_counts: Output-shaft position in 14-bit counts (16384 counts/rev)
 * @retval  Rotor position in 10-bit counts (1024 counts/rotor_rev)
 */
static inline int32_t pos_out_to_rotor(int32_t out_counts) {
  if (out_counts >= 0) {
    return (out_counts * MOTOR_GEAR_RATIO_NUM + (OUT_TO_ROTOR_DEN / 2)) /
           OUT_TO_ROTOR_DEN;
  } else {
    return (out_counts * MOTOR_GEAR_RATIO_NUM - (OUT_TO_ROTOR_DEN / 2)) /
           OUT_TO_ROTOR_DEN;
  }
}

/**
 * @func    pos_out_to_rotor_14
 * @brief   Convert 14-bit output-shaft position counts to 14-bit rotor multi-turn counts.
 * @param   out_counts: Output-shaft position in 14-bit counts (16384 counts/rev)
 * @retval  Rotor position in 14-bit counts (16384 counts/rotor_rev)
 */
static inline int32_t pos_out_to_rotor_14(int32_t out_counts) {
  if (out_counts >= 0) {
    return (out_counts * MOTOR_GEAR_RATIO_NUM + (MOTOR_GEAR_RATIO_DEN / 2)) /
           MOTOR_GEAR_RATIO_DEN;
  } else {
    return (out_counts * MOTOR_GEAR_RATIO_NUM - (MOTOR_GEAR_RATIO_DEN / 2)) /
           MOTOR_GEAR_RATIO_DEN;
  }
}

/**
 * @func    pos_rotor_to_out
 * @brief   Convert 14-bit rotor multi-turn counts to 14-bit output-shaft position counts.
 * @param   rotor_counts_14: Rotor position in 14-bit counts (16384 counts/rotor_rev)
 * @retval  Output-shaft position in 14-bit counts (16384 counts/rev)
 */
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

/**
 * @func    pos_mode_plan
 * @brief   Plan trajectory velocity or position-hold state for Mode 2 (Position Control).
 * @param   speed: Pointer to receive the planned rotor speed command in RPM when not holding
 * @retval  1 if inside position-hold window (s_ctrl.pos_target latched), 0 if moving via speed loop
 */
static int pos_mode_plan(int32_t *speed) {
  int32_t goal = (int32_t)s_cmd.goal_position;
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

  err = pos_out_to_rotor(goal) - s_enc.multiturn;
  aerr = (err < 0) ? -err : err;

  if (aerr <= POS_HOLD_ENTER) {
    s_ctrl.pos_hold = 1;
  } else if (aerr > POS_HOLD_EXIT) {
    s_ctrl.pos_hold = 0;
  }
  if (s_ctrl.pos_hold) {
    s_ctrl.pos_target = s_enc.multiturn + err;
    s_ctrl.integ_v = 0;
    g_control_table.moving_flag = 0;
    return 1;
  }

  vlim = s_cmd.moving_speed;
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
  if (s_cmd.moving_speed == 0) {
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

/**
 * @func    foc_update_elec_angle
 * @brief   Compute electrical commutation angle (0..62831) from rotor position
 *          or ramp virtual angle during rotor alignment calibration.
 * @param   None
 * @retval  None
 */
static void foc_update_elec_angle(void) {
  int32_t x;

  if (s_cal.state != CAL_STATE_ALIGN) {
#if MOTOR_USE_HALL_MT9102
    int32_t raw_comm =
        s_ctrl.imp_hold ? s_ctrl.hold_raw_angle : s_enc.raw_angle;
    int32_t diff = (raw_comm - s_enc.angle_off) & 0x3FFF;
    x = (diff * MOTOR_POLE_PAIRS) & 0x3FFF;
#else
    int32_t diff = (s_enc.raw_angle - s_enc.angle_off) & 0x3FFF;
    x = (diff * MOTOR_POLE_PAIRS) % 16384;
#endif
    /* Scale 14-bit counts to sin/cos table 0..62831 range (ratio 3834 / 1000) */
    x = (x * ELEC_RATIO_NUM) / 1000;
    if (x > ELEC_ANGLE_MAX) {
      x -= ELEC_ANGLE_MAX;
    }
    s_foc.elec = x;
  } else {
    /* Calibration: ramp virtual electrical angle to pull and lock rotor */
    x = s_foc.elec + ELEC_ALIGN_STEP;
    s_foc.elec = (x > ELEC_ANGLE_MAX) ? 0 : x;
  }
}

/**
 * @func    foc_sample_phase_currents
 * @brief   Read phase current ADC channels, subtract zero offsets, and run
 *          Clarke/Park transformation to obtain synchronous d/q currents.
 * @param   tc  Cosine of current electrical angle
 * @param   ts  Sine of current electrical angle
 * @retval  None
 */
static void foc_sample_phase_currents(int32_t tc, int32_t ts) {
  int32_t iu, iv, iw;

#if MOTOR_USE_HALL_MT9102
  /* Phase currents: V = ADC[1], U = ADC[0], W = -(U+V) in 10-bit scale */
  s_foc.adc_v = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) >> 2);
  s_foc.adc_u = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) >> 2);
  iv = (int32_t)(uint16_t)s_foc.adc_v - s_foc.off_v;
  iu = (int32_t)(uint16_t)s_foc.adc_u - s_foc.off_u;
  iw = -(iv + iu);
#if FEATURE_UART_STREAM_TELEMETRY
  s_foc.cur_iu = (int16_t)iu;
  s_foc.cur_iv = (int16_t)iv;
  s_foc.cur_iw = (int16_t)iw;
#endif
  foc_clarke_park(tc, ts, iw, iu, iv, &s_foc.id, &s_foc.iq);
#else
  /* Phase currents: V = ADC[1], U = ADC[0], W = -(U+V) in full 12-bit (Q2) */
#if FOC_INVERT_PHASE_UV
  /* KNEE motor: phase U and V are physically swapped */
  s_foc.adc_v_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) & 0x0FFFU);
  s_foc.adc_u_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) & 0x0FFFU);
#else
  s_foc.adc_v_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_V) & 0x0FFFU);
  s_foc.adc_u_12 = (int16_t)(bsp_adc_read(BSP_ADC_PHASE_U) & 0x0FFFU);
#endif
  s_foc.adc_v = (int16_t)(s_foc.adc_v_12 >> 2);
  s_foc.adc_u = (int16_t)(s_foc.adc_u_12 >> 2);

  /* Full 12-bit zero-offset subtraction before Clarke/Park */
  iv = (int32_t)s_foc.adc_v_12 - (int32_t)s_foc.off_v_12;
  iu = (int32_t)s_foc.adc_u_12 - (int32_t)s_foc.off_u_12;
  iw = -(iv + iu);
#if FEATURE_UART_STREAM_TELEMETRY
  s_foc.cur_iu = (int16_t)(iu >> 2);
  s_foc.cur_iv = (int16_t)(iv >> 2);
  s_foc.cur_iw = (int16_t)(iw >> 2);
#endif
  {
    int32_t id_raw_q2, iq_raw_q2;
    foc_clarke_park(tc, ts, iw, iu, iv, &id_raw_q2, &iq_raw_q2);
    /* Fast 20 kHz EMA low-pass filter (3/4 prev + 1/4 new, fc ~ 1.1 kHz) */
    s_foc.id_filt_q2 = (s_foc.id_filt_q2 * 3 + id_raw_q2) >> 2;
    s_foc.iq_filt_q2 = (s_foc.iq_filt_q2 * 3 + iq_raw_q2) >> 2;
    s_foc.id = (s_foc.id_filt_q2 >= 0) ? ((s_foc.id_filt_q2 + 2) >> 2)
                                       : -(((-s_foc.id_filt_q2) + 2) >> 2);
    s_foc.iq = (s_foc.iq_filt_q2 >= 0) ? ((s_foc.iq_filt_q2 + 2) >> 2)
                                       : -(((-s_foc.iq_filt_q2) + 2) >> 2);
  }
#endif
}

/**
 * @func    foc_update_gate_and_idle_offset
 * @brief   Manage TIM1 PWM gate driver enable/disable transitions, track op-amp
 *          zero offsets while idle, and handle control mode switches.
 * @param   active  Non-zero when torque is enabled and calibration is complete
 * @retval  None
 */
static void foc_update_gate_and_idle_offset(int active) {
  int pwm_needed = active || (s_cal.state != CAL_STATE_IDLE);
  int prev_needed = s_ctrl.prev_active;

  if (pwm_needed && !prev_needed) {
    bsp_pwm_output_enable();
  } else if (!pwm_needed && prev_needed) {
    bsp_pwm_output_disable();
  }
  s_ctrl.prev_active = (uint8_t)pwm_needed;

#if MOTOR_USE_HALL_MT9102
  /* Track 10-bit op-amp zero offset within +/-24 counts of EEPROM base */
  if (!active && s_cal.state == CAL_STATE_IDLE && s_enc.speed == 0 &&
      g_control_table.calib_done == 1) {
    int32_t base_v = (int32_t)g_control_table.offset_v;
    int32_t base_u = (int32_t)g_control_table.offset_u;
    if (s_foc.off_v_acc_q8 == 0) {
      s_foc.off_v_acc_q8 = (int32_t)s_foc.adc_v << 8;
    }
    if (s_foc.off_u_acc_q8 == 0) {
      s_foc.off_u_acc_q8 = (int32_t)s_foc.adc_u << 8;
    }
    s_foc.off_v_acc_q8 +=
        (((int32_t)s_foc.adc_v << 8) - s_foc.off_v_acc_q8) >> 6;
    s_foc.off_u_acc_q8 +=
        (((int32_t)s_foc.adc_u << 8) - s_foc.off_u_acc_q8) >> 6;
    int16_t trim_v = (int16_t)((s_foc.off_v_acc_q8 + 128) >> 8);
    int16_t trim_u = (int16_t)((s_foc.off_u_acc_q8 + 128) >> 8);
    if (trim_v > base_v + 24)
      trim_v = (int16_t)(base_v + 24);
    else if (trim_v < base_v - 24)
      trim_v = (int16_t)(base_v - 24);
    if (trim_u > base_u + 24)
      trim_u = (int16_t)(base_u + 24);
    else if (trim_u < base_u - 24)
      trim_u = (int16_t)(base_u - 24);
    s_foc.off_v = trim_v;
    s_foc.off_u = trim_u;
  }
#else
  /* Track 12-bit op-amp zero offset within +/-8 counts_12 of EEPROM base */
  if (!active && s_cal.state == CAL_STATE_IDLE && s_enc.speed == 0 &&
      g_control_table.calib_done == 1) {
    int32_t base_v_12 = (int32_t)s_foc.off_v << 2;
    int32_t base_u_12 = (int32_t)s_foc.off_u << 2;
    if (s_foc.off_v_acc_q8 == 0) {
      s_foc.off_v_acc_q8 = base_v_12 << 8;
    }
    if (s_foc.off_u_acc_q8 == 0) {
      s_foc.off_u_acc_q8 = base_u_12 << 8;
    }
    s_foc.off_v_acc_q8 +=
        (((int32_t)s_foc.adc_v_12 << 8) - s_foc.off_v_acc_q8) >> 8;
    s_foc.off_u_acc_q8 +=
        (((int32_t)s_foc.adc_u_12 << 8) - s_foc.off_u_acc_q8) >> 8;
    int16_t trim_v_12 = (int16_t)((s_foc.off_v_acc_q8 + 128) >> 8);
    int16_t trim_u_12 = (int16_t)((s_foc.off_u_acc_q8 + 128) >> 8);
    if (trim_v_12 > base_v_12 + 8)
      trim_v_12 = (int16_t)(base_v_12 + 8);
    else if (trim_v_12 < base_v_12 - 8)
      trim_v_12 = (int16_t)(base_v_12 - 8);
    if (trim_u_12 > base_u_12 + 8)
      trim_u_12 = (int16_t)(base_u_12 + 8);
    else if (trim_u_12 < base_u_12 - 8)
      trim_u_12 = (int16_t)(base_u_12 - 8);
    s_foc.off_v_12 = trim_v_12;
    s_foc.off_u_12 = trim_u_12;
  }
#endif

  {
    uint8_t mode = g_control_table.control_mode;
    if (mode != VEL_MODE_POSITION && s_ctrl.prev_mode == VEL_MODE_POSITION) {
      g_control_table.moving_flag = 0;
      g_motor_status_flags &= (uint8_t)~ERR_FLAG_ANGLE;
    }
    if (mode == VEL_MODE_IMPEDANCE && s_ctrl.prev_mode != VEL_MODE_IMPEDANCE) {
      s_ctrl.imp_traj_init = 0;
    }
    s_ctrl.prev_mode = mode;
  }
}

/**
 * @func    foc_reset_inactive_state
 * @brief   Clear all control loop integrators, voltage outputs, and trajectory
 *          states when torque is disabled.
 * @param   None
 * @retval  None
 */
static void foc_reset_inactive_state(void) {
  s_foc.integ_d = 0;
  s_foc.integ_q = 0;
  s_ctrl.integ_v = 0;
  s_foc.vd = 0;
  s_foc.vq = 0;
  s_foc.iq_ref = 0;
  s_ctrl.prev_target_speed = 0;
#if MOTOR_USE_HALL_MT9102
  s_ctrl.pos_integ = 0;
#else
  s_foc.id_filt_q2 = 0;
  s_foc.iq_filt_q2 = 0;
  s_ctrl.punch_kick_timer = 0;
  s_ctrl.punch_kick_dir = 0;
#endif
  s_ctrl.pos_target = s_enc.multiturn;
  s_ctrl.imp_hold = 0;
  s_ctrl.imp_settle_cnt = 0;
  s_ctrl.imp_traj_init = 0;
  s_ctrl.imp_integ = 0;
  s_ctrl.imp_motion_tail = 0;
  s_ctrl.imp_prev_dir = 0;
}

/**
 * @func    foc_run_current_pi
 * @brief   Execute synchronous d/q current PI regulator (Id_ref = 0) and clamp
 *          output voltages vd/vq to +/-FOC_PWM_ARR.
 * @param   zero_output  Non-zero to immediately zero integrators and vd/vq
 * @param   freeze_pi    Non-zero to hold current integrators and vd/vq frozen
 * @retval  None
 */
static void foc_run_current_pi(int zero_output, int freeze_pi) {
#if MOTOR_USE_HALL_MT9102
  if (zero_output) {
    s_foc.integ_d = 0;
    s_foc.integ_q = 0;
    s_foc.vd = 0;
    s_foc.vq = 0;
  } else if (!freeze_pi) {
    int32_t err_d = -s_foc.id;
    int32_t err_q = (int32_t)s_foc.iq_ref - s_foc.iq;
    int32_t vd, vq;

    s_foc.integ_d += CURRENT_KI_GAIN * err_d;
    s_foc.integ_q += CURRENT_KI_GAIN * err_q;
    if (s_foc.integ_d > CURRENT_D_INTEG_LIMIT)
      s_foc.integ_d = CURRENT_D_INTEG_LIMIT;
    else if (s_foc.integ_d < -CURRENT_D_INTEG_LIMIT)
      s_foc.integ_d = -CURRENT_D_INTEG_LIMIT;
    if (s_foc.integ_q >= CURRENT_INTEG_LIMIT)
      s_foc.integ_q = CURRENT_INTEG_LIMIT;
    else if (s_foc.integ_q <= -CURRENT_INTEG_LIMIT)
      s_foc.integ_q = -CURRENT_INTEG_LIMIT;

    vd = (err_d * CURRENT_KP_GAIN + s_foc.integ_d) / 100;
    vq = (err_q * CURRENT_KP_GAIN + s_foc.integ_q) / 100;
    s_foc.vd = (vd >= FOC_PWM_ARR)    ? FOC_PWM_ARR
               : (vd <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                      : vd;
    s_foc.vq = (vq >= FOC_PWM_ARR)    ? FOC_PWM_ARR
               : (vq <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                      : vq;
  }
#else
  int32_t err_d_q2 = -s_foc.id_filt_q2;
  int32_t err_q_q2 = ((int32_t)s_foc.iq_ref << 2) - s_foc.iq_filt_q2;
  int32_t vd, vq;
  (void)zero_output;
  (void)freeze_pi;

  s_foc.integ_d += (CURRENT_KI_GAIN * err_d_q2) >> 2;
  s_foc.integ_q += (CURRENT_KI_GAIN * err_q_q2) >> 2;
  if (s_foc.integ_d > CURRENT_INTEG_LIMIT)
    s_foc.integ_d = CURRENT_INTEG_LIMIT;
  else if (s_foc.integ_d < -CURRENT_INTEG_LIMIT)
    s_foc.integ_d = -CURRENT_INTEG_LIMIT;
  if (s_foc.integ_q >= CURRENT_INTEG_LIMIT)
    s_foc.integ_q = CURRENT_INTEG_LIMIT;
  else if (s_foc.integ_q <= -CURRENT_INTEG_LIMIT)
    s_foc.integ_q = -CURRENT_INTEG_LIMIT;

  vd = (((err_d_q2 * 401) >> 2) + s_foc.integ_d) / 100;
  vq = (((err_q_q2 * 401) >> 2) + s_foc.integ_q) / 100;
  s_foc.vd = (vd >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vd <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vd;
  s_foc.vq = (vq >= FOC_PWM_ARR)    ? FOC_PWM_ARR
             : (vq <= -FOC_PWM_ARR) ? -FOC_PWM_ARR
                                    : vq;
#endif
}

#if MOTOR_USE_HALL_MT9102
/**
 * @func    foc_update_settled_hold
 * @brief   Update the 40 ms settled hold lock state machine for the direct-drive
 *          neck joint to freeze electrical angle and PI voltage at rest.
 * @param   err_14    Current 14-bit position error relative to target
 * @param   can_hold  Non-zero when trajectory is stationary and rotor is still
 * @retval  None
 */
static void foc_update_settled_hold(int32_t err_14, int can_hold) {
  if (can_hold) {
    if (!s_ctrl.imp_hold) {
      if (err_14 >= -POS_DEADBAND_KD_COUNTS_14 &&
          err_14 <= POS_DEADBAND_KD_COUNTS_14) {
        if (s_ctrl.imp_settle_cnt < 800U) {
          s_ctrl.imp_settle_cnt++;
        } else {
          s_ctrl.imp_hold = 1;
          s_ctrl.hold_perr_14 = err_14;
          s_ctrl.hold_raw_angle = s_enc.raw_angle;
        }
      } else {
        s_ctrl.imp_settle_cnt = 0;
      }
    } else {
      int32_t drift = err_14 - s_ctrl.hold_perr_14;
      if (err_14 < -(POS_DEADBAND_KD_COUNTS_14 + 6) ||
          err_14 > (POS_DEADBAND_KD_COUNTS_14 + 6) || drift < -6 || drift > 6) {
        s_ctrl.imp_hold = 0;
        s_ctrl.imp_settle_cnt = 0;
      }
    }
  } else {
    s_ctrl.imp_settle_cnt = 0;
    s_ctrl.imp_hold = 0;
  }
}

/**
 * @func    foc_step_stationary_integ
 * @brief   Accumulate bounded anti-cogging integral step when rotor stops short
 *          outside the settling deadband.
 * @param   integ   Pointer to the integrator accumulator
 * @param   err_14  Current 14-bit position error relative to target
 * @retval  None
 */
static void foc_step_stationary_integ(int32_t *integ, int32_t err_14) {
  int32_t step;

  if (err_14 > 44 || err_14 < -44) {
    step = err_14 >> 2;
    if (step > 18)
      step = 18;
    else if (step < -18)
      step = -18;
  } else {
    step = err_14 >> 3;
    if (step == 0) {
      step = (err_14 > 0) ? 1 : -1;
    }
    if (step > 6)
      step = 6;
    else if (step < -6)
      step = -6;
  }
  *integ += step;
  if (*integ > IMP_FRIC_INTEG_LIMIT) {
    *integ = IMP_FRIC_INTEG_LIMIT;
  } else if (*integ < -IMP_FRIC_INTEG_LIMIT) {
    *integ = -IMP_FRIC_INTEG_LIMIT;
  }
}
#endif

/**
 * @func    foc_step_imp_trajectory
 * @brief   Advance the smooth Q8 position and velocity trajectory generator
 *          toward target_14 with acceleration and cruise velocity limits.
 * @param   target_14  Target rotor angle in 14-bit counts
 * @param   vcmd_out   Host velocity command in output-shaft RPM
 * @retval  Smoothed 14-bit position setpoint (p_set_14)
 */
static int32_t foc_step_imp_trajectory(int32_t target_14, int32_t vcmd_out) {
  int32_t target_q8 = target_14 << 8;

  if (!s_ctrl.imp_traj_init) {
    s_ctrl.imp_pos_q8 = s_enc.multiturn_14 << 8;
    s_ctrl.imp_vel_q8 = 0;
    s_ctrl.imp_last_vlim = IMP_TRAJ_DEFAULT_ROTOR_RPM;
    s_ctrl.imp_integ = 0;
    s_ctrl.imp_motion_tail = 0;
    s_ctrl.imp_prev_dir = 0;
    s_ctrl.imp_traj_init = 1;
  }

  int32_t dist_q8 = target_q8 - s_ctrl.imp_pos_q8;
  int32_t adist_q8 = (dist_q8 >= 0) ? dist_q8 : -dist_q8;
  int32_t vlim_rpm = (vcmd_out >= 0) ? vcmd_out : -vcmd_out;

  if (vlim_rpm != 0) {
#if !MOTOR_USE_HALL_MT9102
    vlim_rpm =
        (vlim_rpm * MOTOR_GEAR_RATIO_NUM + (MOTOR_GEAR_RATIO_DEN / 2)) /
        MOTOR_GEAR_RATIO_DEN;
#endif
    if (vlim_rpm < IMP_TRAJ_MIN_ROTOR_RPM) {
      vlim_rpm = IMP_TRAJ_MIN_ROTOR_RPM;
    } else if (vlim_rpm > IMP_TRAJ_MAX_ROTOR_RPM) {
      vlim_rpm = IMP_TRAJ_MAX_ROTOR_RPM;
    }
    s_ctrl.imp_last_vlim = vlim_rpm;
  } else {
#if MOTOR_USE_HALL_MT9102
    int32_t step_thresh_q8 = 128 << 8;
#else
    int32_t step_thresh_q8 = 256 << 8;
#endif
    if (adist_q8 > step_thresh_q8 && s_ctrl.imp_vel_q8 == 0) {
      vlim_rpm = IMP_TRAJ_DEFAULT_ROTOR_RPM;
      s_ctrl.imp_last_vlim = vlim_rpm;
    } else {
      vlim_rpm = s_ctrl.imp_last_vlim;
    }
  }

  int32_t vmax_q8 = (vlim_rpm * 895) >> 8;
  int32_t vdes_mag_q8 = adist_q8 >> IMP_TRAJ_DECEL_SHIFT;
#if MOTOR_USE_HALL_MT9102
  if (vdes_mag_q8 < 16 && adist_q8 > 0) {
    vdes_mag_q8 = (adist_q8 > 16) ? 16 : adist_q8;
  }
#else
  if (vdes_mag_q8 == 0 && adist_q8 > 0) {
    vdes_mag_q8 =
        (adist_q8 > IMP_TRAJ_ACCEL_Q8) ? IMP_TRAJ_ACCEL_Q8 : adist_q8;
  }
#endif
  if (vdes_mag_q8 > vmax_q8) {
    vdes_mag_q8 = vmax_q8;
  }
  int32_t vdes_q8 = (dist_q8 >= 0) ? vdes_mag_q8 : -vdes_mag_q8;

  if (s_ctrl.imp_vel_q8 < vdes_q8) {
    s_ctrl.imp_vel_q8 += IMP_TRAJ_ACCEL_Q8;
    if (s_ctrl.imp_vel_q8 > vdes_q8) {
      s_ctrl.imp_vel_q8 = vdes_q8;
    }
  } else if (s_ctrl.imp_vel_q8 > vdes_q8) {
    s_ctrl.imp_vel_q8 -= IMP_TRAJ_ACCEL_Q8;
    if (s_ctrl.imp_vel_q8 < vdes_q8) {
      s_ctrl.imp_vel_q8 = vdes_q8;
    }
  }

  s_ctrl.imp_pos_q8 += s_ctrl.imp_vel_q8;
  if ((dist_q8 > 0 && s_ctrl.imp_pos_q8 >= target_q8) ||
      (dist_q8 < 0 && s_ctrl.imp_pos_q8 <= target_q8) || dist_q8 == 0) {
    s_ctrl.imp_pos_q8 = target_q8;
    s_ctrl.imp_vel_q8 = 0;
  }

  return (s_ctrl.imp_pos_q8 + 128) >> 8;
}

/**
 * @func    foc_ctrl_impedance_mode
 * @brief   Execute MIT Impedance control mode (trajectory generation, glide-path
 *          damping, cogging compensation, and current PI regulation).
 * @param   None
 * @retval  None
 */
static void foc_ctrl_impedance_mode(void) {
  int32_t p_err_14 = 0;
  int32_t v_err = 0;
  int32_t vcmd_out = (int32_t)s_cmd.moving_speed;
  uint16_t kd_eff = s_cmd.imp_kd;

#if MOTOR_USE_HALL_MT9102
  int32_t v_err_q4 = 0;

  if (s_cmd.imp_kp > 0) {
    int32_t target_14 = pos_out_to_rotor_14((int32_t)s_cmd.goal_position);
    int32_t p_set_14 = foc_step_imp_trajectory(target_14, vcmd_out);

    p_err_14 = p_set_14 - s_enc.multiturn_14;
    if (p_err_14 > POS_ERR_14_CLAMP_MAX) {
      p_err_14 = POS_ERR_14_CLAMP_MAX;
    } else if (p_err_14 < -POS_ERR_14_CLAMP_MAX) {
      p_err_14 = -POS_ERR_14_CLAMP_MAX;
    }
    v_err_q4 = -s_enc.speed_q4;

    int32_t goal_err_14 = target_14 - s_enc.multiturn_14;
    /* Safe glide-path overspeed braking outside +/-180 counts_14 (4.0 deg) */
    if (goal_err_14 > 180) {
      int32_t v_safe_q4 = ((goal_err_14 * 3) >> 2) + 96;
      if (s_enc.speed_q4 > v_safe_q4) {
        int32_t over_q4 = s_enc.speed_q4 - v_safe_q4;
        p_err_14 -= over_q4;
        if (p_err_14 < 0) {
          p_err_14 = 0;
        }
        v_err_q4 -= over_q4;
      }
    } else if (goal_err_14 < -180) {
      int32_t v_safe_q4 = -((((-goal_err_14) * 3) >> 2) + 96);
      if (s_enc.speed_q4 < v_safe_q4) {
        int32_t over_q4 = v_safe_q4 - s_enc.speed_q4;
        p_err_14 += over_q4;
        if (p_err_14 > 0) {
          p_err_14 = 0;
        }
        v_err_q4 += over_q4;
      }
    }

    foc_update_settled_hold(goal_err_14,
                            s_ctrl.imp_vel_q8 == 0 && s_enc.still_ticks >= 8U);

    if (s_ctrl.imp_hold && s_ctrl.imp_vel_q8 == 0) {
      p_err_14 = s_ctrl.hold_perr_14;
      v_err_q4 = 0;
    }
    if (s_cmd.imp_kp >= 500) {
      uint16_t kd_min = (s_ctrl.imp_vel_q8 == 0) ? IMP_SETTLE_KD_MIN_ROTOR
                                                 : IMP_MOTION_KD_MIN_ROTOR;
      if (kd_eff < kd_min) {
        kd_eff = kd_min;
      }
    }
    if (s_ctrl.imp_vel_q8 != 0 || (goal_err_14 > 0 && s_ctrl.imp_integ < 0) ||
        (goal_err_14 < 0 && s_ctrl.imp_integ > 0)) {
      s_ctrl.imp_integ = 0;
    } else if (s_enc.still_ticks == 0) {
      s_ctrl.imp_integ -= (s_ctrl.imp_integ >> 5);
    } else if (!s_ctrl.imp_hold && s_enc.still_ticks >= 9U &&
               (goal_err_14 > POS_INTEG_FREEZE_COUNTS_14 ||
                goal_err_14 < -POS_INTEG_FREEZE_COUNTS_14)) {
      foc_step_stationary_integ(&s_ctrl.imp_integ, goal_err_14);
    }

    if (p_err_14 > POS_ERR_14_CLAMP_MAX) {
      p_err_14 = POS_ERR_14_CLAMP_MAX;
    } else if (p_err_14 < -POS_ERR_14_CLAMP_MAX) {
      p_err_14 = -POS_ERR_14_CLAMP_MAX;
    }
  } else {
    /* Pure damping / torque mode (Kp == 0) */
    s_ctrl.imp_pos_q8 = s_enc.multiturn_14 << 8;
    s_ctrl.imp_vel_q8 = 0;
    s_ctrl.imp_integ = 0;
    s_ctrl.imp_traj_init = 1;
    s_ctrl.imp_hold = 0;
    s_ctrl.imp_settle_cnt = 0;
    v_err_q4 = (vcmd_out << 4) - s_enc.speed_q4;
  }
#else
  if (s_cmd.imp_kp > 0) {
    int32_t target_14 = pos_out_to_rotor_14((int32_t)s_cmd.goal_position);
    int32_t p_set_14 = foc_step_imp_trajectory(target_14, vcmd_out);
    int32_t v_traj_rpm = (s_ctrl.imp_vel_q8 * 73) >> 8;

    p_err_14 = p_set_14 - s_enc.multiturn_14;
    v_err = v_traj_rpm - s_enc.speed;

    if (s_ctrl.imp_vel_q8 != 0) {
      int8_t cur_dir = (s_ctrl.imp_vel_q8 > 0) ? 1 : -1;
      if (s_ctrl.imp_prev_dir != 0 && cur_dir != s_ctrl.imp_prev_dir) {
        s_ctrl.imp_integ = 0;
      }
      s_ctrl.imp_prev_dir = cur_dir;
      s_ctrl.imp_motion_tail = IMP_MOTION_TAIL_TICKS;
    } else if (s_ctrl.imp_motion_tail > 0) {
      s_ctrl.imp_motion_tail--;
    }

    if (s_ctrl.imp_vel_q8 != 0 ||
        (s_ctrl.imp_motion_tail > 0 &&
         (p_err_14 > POS_DEADBAND_KD_COUNTS_14 ||
          p_err_14 < -POS_DEADBAND_KD_COUNTS_14))) {
      int32_t err_i = v_err + (p_err_14 >> 3);
      s_ctrl.imp_integ += err_i;
      if (s_ctrl.imp_integ > IMP_FRIC_INTEG_LIMIT) {
        s_ctrl.imp_integ = IMP_FRIC_INTEG_LIMIT;
      } else if (s_ctrl.imp_integ < -IMP_FRIC_INTEG_LIMIT) {
        s_ctrl.imp_integ = -IMP_FRIC_INTEG_LIMIT;
      }
      if (kd_eff < IMP_MOTION_KD_MIN_ROTOR) {
        kd_eff = IMP_MOTION_KD_MIN_ROTOR;
      }
    } else {
      if (s_ctrl.imp_integ > 0) {
        s_ctrl.imp_integ = (s_ctrl.imp_integ * 255) >> 8;
      } else if (s_ctrl.imp_integ < 0) {
        s_ctrl.imp_integ = -(((-s_ctrl.imp_integ) * 255) >> 8);
      }
      if (s_ctrl.imp_motion_tail == 0) {
        s_ctrl.imp_prev_dir = 0;
      }
    }

    if (s_ctrl.imp_vel_q8 == 0 && vcmd_out == 0 &&
        p_err_14 >= -POS_DEADBAND_KD_COUNTS_14 &&
        p_err_14 <= POS_DEADBAND_KD_COUNTS_14) {
      if (!s_ctrl.imp_hold) {
        if (++s_ctrl.imp_settle_cnt >= IMP_HOLD_SETTLE_TICKS) {
          s_ctrl.imp_hold = 1;
        }
      }
    } else {
      s_ctrl.imp_hold = 0;
      s_ctrl.imp_settle_cnt = 0;
    }

    if (s_ctrl.imp_hold) {
      v_err = 0;
    }

    if (s_ctrl.imp_vel_q8 == 0 && vcmd_out == 0 &&
        s_ctrl.imp_motion_tail == 0) {
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
    s_ctrl.imp_pos_q8 = s_enc.multiturn_14 << 8;
    s_ctrl.imp_vel_q8 = 0;
    s_ctrl.imp_integ = 0;
    s_ctrl.imp_motion_tail = 0;
    s_ctrl.imp_prev_dir = 0;
    s_ctrl.imp_traj_init = 1;
    s_ctrl.imp_hold = 0;
    s_ctrl.imp_settle_cnt = 0;
    v_err = (vcmd_out * MOTOR_GEAR_RATIO_NUM) / MOTOR_GEAR_RATIO_DEN -
            s_enc.speed;
  }
#endif

  int32_t kp_term = ((int32_t)s_cmd.imp_kp * p_err_14) >> 4;
#if MOTOR_USE_HALL_MT9102
  (void)v_err;
  int32_t kd_term = ((int32_t)kd_eff * v_err_q4) >> 4;
  int32_t tau =
      (kp_term * 5 + kd_term + (s_ctrl.imp_integ / 8)) / IMP_GAIN_SCALE_DIV +
      (int32_t)s_cmd.imp_tau_ff;
#else
  int32_t kd_term = (int32_t)kd_eff * v_err;
  int32_t i_term = s_ctrl.imp_integ / IMP_FRIC_INTEG_DIV;
  int32_t tau = (kp_term + kd_term) / IMP_GAIN_SCALE_DIV + i_term +
                (int32_t)s_cmd.imp_tau_ff;
#endif

  if (tau > IQ_LIMIT)
    tau = IQ_LIMIT;
  if (tau < -IQ_LIMIT)
    tau = -IQ_LIMIT;

  s_foc.iq_ref = (int16_t)tau;
  s_ctrl.integ_v = 0;
  s_ctrl.prev_target_speed = 0;
#if MOTOR_USE_HALL_MT9102
  s_ctrl.pos_integ = 0;
  foc_run_current_pi(s_ctrl.imp_vel_q8 == 0 && s_foc.iq_ref == 0,
                     s_ctrl.imp_hold);
#else
  s_ctrl.punch_kick_timer = 0;
  s_ctrl.punch_kick_dir = 0;
  foc_run_current_pi(0, 0);
#endif
}

/**
 * @func    foc_ctrl_position_mode
 * @brief   Execute Position/Velocity control mode (speed ramp planning, closed-loop
 *          speed PI or 14-bit/10-bit position hold PD, and current PI regulation).
 * @param   None
 * @retval  None
 */
static void foc_ctrl_position_mode(void) {
  int32_t target_speed = s_cmd.moving_speed;
  int hold = pos_mode_plan(&target_speed);

#if MOTOR_USE_HALL_MT9102
  if (!hold) {
    int16_t err;

    s_ctrl.pos_integ = 0;
    s_ctrl.imp_hold = 0;
    s_ctrl.imp_settle_cnt = 0;
    s_ctrl.pos_target = s_enc.multiturn;
    err = (int16_t)(target_speed - s_enc.speed);
    s_ctrl.spd_err = err;
    if ((target_speed > 0 && s_ctrl.prev_target_speed <= 0) ||
        (target_speed < 0 && s_ctrl.prev_target_speed >= 0)) {
      s_ctrl.integ_v = 0;
    }
    s_ctrl.prev_target_speed = target_speed;
    if (target_speed > 0 && err < 0 && s_ctrl.integ_v > 0) {
      s_ctrl.integ_v += (int32_t)err * 4;
      if (s_ctrl.integ_v < 0) {
        s_ctrl.integ_v = 0;
      }
    } else if (target_speed < 0 && err > 0 && s_ctrl.integ_v < 0) {
      s_ctrl.integ_v += (int32_t)err * 4;
      if (s_ctrl.integ_v > 0) {
        s_ctrl.integ_v = 0;
      }
    } else {
      s_ctrl.integ_v += err;
    }
    if (s_ctrl.integ_v > SPEED_INTEG_LIMIT) {
      s_ctrl.integ_v = SPEED_INTEG_LIMIT;
    } else if (s_ctrl.integ_v < -SPEED_INTEG_LIMIT) {
      s_ctrl.integ_v = -SPEED_INTEG_LIMIT;
    }
    s_foc.iq_ref = (int16_t)(s_ctrl.integ_v / 600 + err * 2);
    s_ctrl.pos_integ = 0;
  } else {
    int32_t target_14 = pos_out_to_rotor_14((int32_t)s_cmd.goal_position);
    int32_t perr_14 = target_14 - s_enc.multiturn_14;
    double d;

    s_ctrl.pos_err = (int16_t)(perr_14 >> 4);
    s_ctrl.prev_target_speed = 0;

    if ((perr_14 > 0 && s_ctrl.pos_integ < 0) ||
        (perr_14 < 0 && s_ctrl.pos_integ > 0)) {
      s_ctrl.pos_integ = 0;
    } else if (s_enc.still_ticks == 0) {
      s_ctrl.pos_integ -= (s_ctrl.pos_integ >> 5);
    }
    if (!s_ctrl.imp_hold && s_enc.still_ticks >= 9U &&
        (perr_14 > POS_INTEG_FREEZE_COUNTS_14 ||
         perr_14 < -POS_INTEG_FREEZE_COUNTS_14)) {
      foc_step_stationary_integ(&s_ctrl.pos_integ, perr_14);
    }

    foc_update_settled_hold(perr_14, s_enc.still_ticks >= 8U);

    int32_t spd_q4_eff = s_ctrl.imp_hold ? 0 : s_enc.speed_q4;
    int32_t perr_eff_14 = s_ctrl.imp_hold ? s_ctrl.hold_perr_14 : perr_14;
    if (!s_ctrl.imp_hold) {
      if ((perr_eff_14 > 16 && spd_q4_eff > 0) ||
          (perr_eff_14 < -16 && spd_q4_eff < 0)) {
        perr_eff_14 /= 5;
      }
    }
    d = (double)((float)perr_eff_14 * (POS_HOLD_KP * 0.0625f) +
                 (float)s_ctrl.pos_integ * 0.000125f) -
        (double)spd_q4_eff * (double)(POS_HOLD_KD * 0.0625f);
    s_foc.iq_ref = (int16_t)(int32_t)d;
  }

  if (s_foc.iq_ref > IQ_LIMIT) {
    s_foc.iq_ref = IQ_LIMIT;
  } else if (s_foc.iq_ref < -IQ_LIMIT) {
    s_foc.iq_ref = -IQ_LIMIT;
  }
  foc_run_current_pi(hold && (s_foc.iq_ref == 0), hold && s_ctrl.imp_hold);
#else
  if (!hold) {
    int16_t err;
    int16_t punch_val = 0;

    s_ctrl.pos_target = s_enc.multiturn;
    err = (int16_t)(target_speed - s_enc.speed);
    s_ctrl.spd_err = err;

    if ((target_speed > 0 && s_ctrl.prev_target_speed <= 0) ||
        (target_speed < 0 && s_ctrl.prev_target_speed >= 0)) {
      s_ctrl.integ_v = 0;
      s_ctrl.punch_kick_timer = PUNCH_KICK_TICKS;
      s_ctrl.punch_kick_dir = (target_speed > 0) ? 1 : -1;
    }
    s_ctrl.prev_target_speed = target_speed;

    if (s_ctrl.punch_kick_timer > 0) {
      int16_t pmax = (int16_t)g_control_table.punch;
      if (pmax > PUNCH_MAX) {
        pmax = PUNCH_MAX;
      }
      punch_val = (int16_t)(((int32_t)pmax * s_ctrl.punch_kick_timer) /
                            PUNCH_KICK_TICKS) *
                  s_ctrl.punch_kick_dir;
      s_ctrl.punch_kick_timer--;
    }

    s_ctrl.integ_v += err;
    if (s_ctrl.integ_v > SPEED_INTEG_LIMIT) {
      s_ctrl.integ_v = SPEED_INTEG_LIMIT;
    } else if (s_ctrl.integ_v < -SPEED_INTEG_LIMIT) {
      s_ctrl.integ_v = -SPEED_INTEG_LIMIT;
    }
    s_foc.iq_ref =
        (int16_t)(s_ctrl.integ_v / VEL_KI_DIV + err / 2 + punch_val);
  } else {
    s_ctrl.integ_v = 0;
    s_ctrl.prev_target_speed = 0;
    s_ctrl.punch_kick_timer = 0;
    s_ctrl.punch_kick_dir = 0;

    int16_t perr = (int16_t)(s_ctrl.pos_target - s_enc.multiturn);
    s_ctrl.pos_err = perr;
    if (perr >= -POS_DEADBAND_KP_COUNTS && perr <= POS_DEADBAND_KP_COUNTS) {
      s_foc.iq_ref = 0;
    } else {
      double d = (double)((float)perr * POS_HOLD_KP) -
                 (double)s_enc.speed * POS_HOLD_KD;
      s_foc.iq_ref = (int16_t)(int32_t)d;
    }
  }

  if (s_foc.iq_ref > IQ_LIMIT) {
    s_foc.iq_ref = IQ_LIMIT;
  } else if (s_foc.iq_ref < -IQ_LIMIT) {
    s_foc.iq_ref = -IQ_LIMIT;
  }
  foc_run_current_pi(0, 0);
#endif
}

/**
 * @func    foc_write_pwm_duty
 * @brief   Write active SVPWM duty cycles to TIM1 compare registers with
 *          optional U/V phase swap.
 * @param   None
 * @retval  None
 */
static void foc_write_pwm_duty(void) {
#if FOC_INVERT_PHASE_UV
  bsp_pwm_set_compare(FOC_PWM_ARR - s_foc.duty[0], FOC_PWM_ARR - s_foc.duty[2],
                      FOC_PWM_ARR - s_foc.duty[1]);
#else
  bsp_pwm_set_compare(FOC_PWM_ARR - s_foc.duty[0], FOC_PWM_ARR - s_foc.duty[1],
                      FOC_PWM_ARR - s_foc.duty[2]);
#endif
}

#if MOTOR_USE_HALL_MT9102
/**
 * @func    foc_cal_hall_sweep_svpwm
 * @brief   Generate open-loop rotating voltage vector for MT9102 Hall sensor
 *          min/max auto-calibration sweep.
 * @param   None
 * @retval  None
 */
static void foc_cal_hall_sweep_svpwm(void) {
  int32_t vd_sweep = CAL_SWEEP_VOLT;
  int32_t vq_sweep = 0;

  if (s_cal.cal_cnt < CAL_SWEEP_PRELOCK_TICKS) {
    s_cal.sweep_elec = 0;
    s_cal.sweep_elec_q8 = 0;
    if (s_cal.cal_cnt < 2048U) {
      vd_sweep = ((int32_t)s_cal.cal_cnt * CAL_SWEEP_PRELOCK_VOLT) >> 11;
    } else {
      vd_sweep = CAL_SWEEP_PRELOCK_VOLT -
                 (((int32_t)(s_cal.cal_cnt - 2048U) *
                   (CAL_SWEEP_PRELOCK_VOLT - CAL_SWEEP_VOLT)) >>
                  11);
    }
    vq_sweep = -(s_enc.speed_q4 * 2);
    if (vq_sweep > 60)
      vq_sweep = 60;
    else if (vq_sweep < -60)
      vq_sweep = -60;
  } else {
    int32_t rot_tick = (int32_t)s_cal.cal_cnt - CAL_SWEEP_PRELOCK_TICKS + 1;
    int32_t step_q8 = CAL_SWEEP_STEP_Q8;
    if (rot_tick <= CAL_SWEEP_RAMP_TICKS) {
      step_q8 = (rot_tick * CAL_SWEEP_STEP_Q8) >> 12;
    } else if (rot_tick > (CAL_SWEEP_ROTATE_TICKS - CAL_SWEEP_RAMP_TICKS)) {
      int32_t rem = CAL_SWEEP_ROTATE_TICKS - rot_tick;
      if (rem < 0)
        rem = 0;
      step_q8 = (rem * CAL_SWEEP_STEP_Q8) >> 12;
    }
    s_cal.sweep_elec_q8 += step_q8;
    if (s_cal.sweep_elec_q8 >= (ELEC_ANGLE_MAX << 8)) {
      s_cal.sweep_elec_q8 -= (ELEC_ANGLE_MAX << 8);
    }
    s_cal.sweep_elec = s_cal.sweep_elec_q8 >> 8;
  }
  foc_inv_park_svpwm(trig_cos(s_cal.sweep_elec), trig_sin(s_cal.sweep_elec),
                     vd_sweep, vq_sweep, s_foc.duty);
}
#endif

/**
 * @func    foc_cal_align_svpwm
 * @brief   Execute rotor pole alignment at electrical angle 0 and store the
 *          calibrated mechanical zero angle offset.
 * @param   None
 * @retval  None
 */
static void foc_cal_align_svpwm(void) {
#if MOTOR_USE_HALL_MT9102
  uint16_t half_ticks = ALIGN_TICKS / 2U;
  uint16_t phase_cnt = (s_cal.align_cnt < half_ticks)
                           ? s_cal.align_cnt
                           : (s_cal.align_cnt - half_ticks);
  int32_t vq_mag = 0;
  int32_t vq_align = 0;

  s_cal.align_ramp = ALIGN_RAMP_MAX;
  if (phase_cnt < 2048U) {
    vq_mag = 100;
  } else if (phase_cnt < 4096U) {
    vq_mag = ((int32_t)(4096U - phase_cnt) * 100) >> 11;
  }
  if (vq_mag > 0) {
    vq_align = (s_cal.align_cnt < half_ticks) ? vq_mag : -vq_mag;
  } else if (phase_cnt < 6500U) {
    vq_align = -(s_enc.speed_q4 * 2);
    if (vq_align > 60)
      vq_align = 60;
    else if (vq_align < -60)
      vq_align = -60;
  }
  foc_inv_park_svpwm(trig_cos(0), trig_sin(0), (int32_t)s_cal.align_ramp,
                     vq_align, s_foc.duty);

  s_cal.align_cnt++;
  if (s_cal.align_cnt > (half_ticks - ALIGN_AVG_SAMPLES) &&
      s_cal.align_cnt <= half_ticks) {
    if (s_cal.align_cnt == (half_ticks - ALIGN_AVG_SAMPLES + 1U)) {
      s_cal.align_ref_raw = s_enc.raw_angle;
      s_cal.align_sum_raw = 0;
    }
    int32_t diff = (s_enc.raw_angle - s_cal.align_ref_raw) & 0x3FFF;
    if (diff > 8192) {
      diff -= 16384;
    }
    s_cal.align_sum_raw += diff;
    if (s_cal.align_cnt == half_ticks) {
      int32_t avg_diff =
          (s_cal.align_sum_raw >= 0)
              ? ((s_cal.align_sum_raw + (ALIGN_AVG_SAMPLES / 2)) /
                 ALIGN_AVG_SAMPLES)
              : ((s_cal.align_sum_raw - (ALIGN_AVG_SAMPLES / 2)) /
                 ALIGN_AVG_SAMPLES);
      s_cal.align_pos_plus = (s_cal.align_ref_raw + avg_diff) & 0x3FFF;
      s_cal.align_sum_raw = 0;
    }
  } else if (s_cal.align_cnt > (ALIGN_TICKS - ALIGN_AVG_SAMPLES)) {
    if (s_cal.align_cnt == (ALIGN_TICKS - ALIGN_AVG_SAMPLES + 1U)) {
      s_cal.align_ref_raw = s_enc.raw_angle;
      s_cal.align_sum_raw = 0;
    }
    int32_t diff = (s_enc.raw_angle - s_cal.align_ref_raw) & 0x3FFF;
    if (diff > 8192) {
      diff -= 16384;
    }
    s_cal.align_sum_raw += diff;
  }

  if (s_cal.align_cnt >= ALIGN_TICKS) {
    int32_t avg_diff =
        (s_cal.align_sum_raw >= 0)
            ? ((s_cal.align_sum_raw + (ALIGN_AVG_SAMPLES / 2)) /
               ALIGN_AVG_SAMPLES)
            : ((s_cal.align_sum_raw - (ALIGN_AVG_SAMPLES / 2)) /
               ALIGN_AVG_SAMPLES);
    int32_t angle_minus = (s_cal.align_ref_raw + avg_diff) & 0x3FFF;
    int32_t diff_pm = (angle_minus - s_cal.align_pos_plus) & 0x3FFF;
    if (diff_pm > 8192) {
      diff_pm -= 16384;
    }
    int32_t final_angle = (s_cal.align_pos_plus + (diff_pm / 2)) & 0x3FFF;

    s_cal.align_ramp = 0.0f;
    s_cal.align_cnt = 0;
    s_cal.align_sum_raw = 0;
    s_enc.angle_off = final_angle;
    g_control_table.angle_offset = (uint16_t)final_angle;
    s_enc.raw_prev = s_enc.raw_angle;
    s_enc.pos_prev = s_enc.pos10;
    s_enc.turns = (s_enc.raw_angle > 8192) ? -1 : 0;
    s_enc.rev_cnt = 0;
    s_enc.multiturn = ((int32_t)s_enc.turns << 10) + (int32_t)s_enc.pos10;
    s_enc.multiturn_14 = ((int32_t)s_enc.turns << 14) + s_enc.raw_angle;
    s_ctrl.pos_target = s_enc.multiturn;
    s_enc.spd_hist_init = 0;
    s_enc.speed_q4 = 0;
    s_enc.still_ticks = 0;
    {
      int16_t cur_out = (int16_t)pos_rotor_to_out(s_enc.multiturn_14);
      s_cmd.goal_position = cur_out;
      g_control_table.goal_position = cur_out;
      g_control_table.current_position = cur_out;
    }
    s_ctrl.imp_traj_init = 0;
    s_cal.state = CAL_STATE_IDLE;
    g_control_table.calib_done = 1;
    g_eeprom_dirty_flag = 1;
  }
#else
  s_cal.align_ramp += ALIGN_RAMP_STEP;
  if (s_cal.align_ramp > ALIGN_RAMP_MAX) {
    s_cal.align_ramp = ALIGN_RAMP_MAX;
  }
  foc_inv_park_svpwm(trig_cos(0), trig_sin(0), (int32_t)s_cal.align_ramp, 0,
                     s_foc.duty);

  s_cal.align_cnt++;
  if (s_cal.align_cnt > ALIGN_TICKS) {
    s_cal.align_ramp = 0.0f;
    s_cal.align_cnt = 0;
    s_enc.angle_off = s_enc.raw_angle;
    g_control_table.angle_offset = (uint16_t)s_enc.raw_angle;
    s_cal.state = CAL_STATE_IDLE;
    g_control_table.calib_done = 1;
    g_eeprom_dirty_flag = 1;
  }
#endif
}

/**
 * @func    foc_apply_calib_and_pwm
 * @brief   Compute SVPWM duty cycles (for normal operation or calibration
 *          states) and write compare values to TIM1 PWM registers.
 * @param   tc  Cosine of current electrical angle
 * @param   ts  Sine of current electrical angle
 * @retval  None
 */
static void foc_apply_calib_and_pwm(int32_t tc, int32_t ts) {
#if MOTOR_USE_HALL_MT9102
  if (s_cal.state == CAL_STATE_HALL_SWEEP) {
    foc_cal_hall_sweep_svpwm();
  } else if (s_cal.state == CAL_STATE_ALIGN) {
    foc_cal_align_svpwm();
  } else {
    foc_inv_park_svpwm(tc, ts, s_foc.vd, s_foc.vq, s_foc.duty);
  }
#else
  if (s_cal.state == CAL_STATE_ALIGN) {
    foc_cal_align_svpwm();
  } else {
    foc_inv_park_svpwm(tc, ts, s_foc.vd, s_foc.vq, s_foc.duty);
  }
#endif

  switch (s_cal.state) {
  case CAL_STATE_ZERO_TOTAL:
    s_cal.cal_cnt++;
    if (s_cal.cal_cnt > CAL_SETTLE_TICKS) {
      s_cal.sum_t += bsp_adc_read(BSP_ADC_TOTAL_CURRENT);
    }
    if (s_cal.cal_cnt >= CAL_TOTAL_TICKS) {
      g_control_table.offset_total = (int16_t)(s_cal.sum_t >> 10);
      s_cal.sum_t = 0;
      s_cal.cal_cnt = 0;
      s_cal.state = CAL_STATE_ZERO_PHASE;
    }
    bsp_pwm_set_compare(0, 0, 0);
    break;

  case CAL_STATE_ZERO_PHASE:
    s_cal.cal_cnt++;
#if MOTOR_USE_HALL_MT9102
    if (s_cal.cal_cnt > CAL_SETTLE_TICKS) {
      s_cal.sum_v += s_foc.adc_v;
      s_cal.sum_u += s_foc.adc_u;
    }
    if (s_cal.cal_cnt >= CAL_TOTAL_TICKS) {
      s_foc.off_v = (int16_t)((s_cal.sum_v + 512) >> 10);
      s_foc.off_u = (int16_t)((s_cal.sum_u + 512) >> 10);
      s_foc.off_v_acc_q8 = (int32_t)s_foc.off_v << 8;
      s_foc.off_u_acc_q8 = (int32_t)s_foc.off_u << 8;
      g_control_table.offset_v = s_foc.off_v;
      g_control_table.offset_u = s_foc.off_u;
      s_cal.sum_v = 0;
      s_cal.sum_u = 0;
      s_cal.cal_cnt = 0;
      s_cal.sweep_elec = 0;
      s_cal.sweep_elec_q8 = 0;
      s_cal.min_sin = 4095;
      s_cal.max_sin = 0;
      s_cal.min_cos = 4095;
      s_cal.max_cos = 0;
      s_cal.state = CAL_STATE_HALL_SWEEP;
    }
    bsp_pwm_set_compare(CAL_PHASE_DUTY, CAL_PHASE_DUTY, CAL_PHASE_DUTY);
#else
    if (s_cal.cal_cnt > CAL_SETTLE_TICKS) {
      s_cal.sum_v += s_foc.adc_v_12;
      s_cal.sum_u += s_foc.adc_u_12;
    }
    if (s_cal.cal_cnt >= CAL_TOTAL_TICKS) {
      s_foc.off_v_12 = (int16_t)((s_cal.sum_v + 512) >> 10);
      s_foc.off_u_12 = (int16_t)((s_cal.sum_u + 512) >> 10);
      s_foc.off_v = (int16_t)((s_foc.off_v_12 + 2) >> 2);
      s_foc.off_u = (int16_t)((s_foc.off_u_12 + 2) >> 2);
      s_foc.off_v_acc_q8 = (int32_t)s_foc.off_v_12 << 8;
      s_foc.off_u_acc_q8 = (int32_t)s_foc.off_u_12 << 8;
      g_control_table.offset_v = s_foc.off_v;
      g_control_table.offset_u = s_foc.off_u;
      s_cal.sum_v = 0;
      s_cal.sum_u = 0;
      s_cal.cal_cnt = 0;
      s_cal.state = CAL_STATE_ALIGN;
    }
    bsp_pwm_set_compare(CAL_PHASE_DUTY, CAL_PHASE_DUTY, CAL_PHASE_DUTY);
#endif
    break;

#if MOTOR_USE_HALL_MT9102
  case CAL_STATE_HALL_SWEEP:
    s_cal.cal_cnt++;
    if (s_cal.cal_cnt > CAL_SWEEP_PRELOCK_TICKS) {
      uint16_t ha = bsp_adc_read(BSP_ADC_HALL_SIN);
      uint16_t hb = bsp_adc_read(BSP_ADC_HALL_COS);
      if (ha < s_cal.min_sin)
        s_cal.min_sin = ha;
      if (ha > s_cal.max_sin)
        s_cal.max_sin = ha;
      if (hb < s_cal.min_cos)
        s_cal.min_cos = hb;
      if (hb > s_cal.max_cos)
        s_cal.max_cos = hb;
    }
    if (s_cal.cal_cnt >= CAL_SWEEP_TICKS) {
      uint16_t off_sin = (s_cal.min_sin + s_cal.max_sin) >> 1;
      uint16_t off_cos = (s_cal.min_cos + s_cal.max_cos) >> 1;
      uint16_t span_sin = s_cal.max_sin - s_cal.min_sin;
      uint16_t span_cos = s_cal.max_cos - s_cal.min_cos;
      if (span_sin > 2500 && span_cos > 2500 && off_sin > 1600 &&
          off_sin < 2400 && off_cos > 1600 && off_cos < 2400) {
        g_control_table.hall_offset_sin = off_sin;
        g_control_table.hall_offset_cos = off_cos;
        g_control_table.hall_gain_sin_num = span_cos;
        g_control_table.hall_gain_sin_den = span_sin;
        g_control_table.hall_calib_done = 1;
        bsp_mt9102_set_calibration(off_sin, off_cos, span_cos, span_sin);
      }
      s_cal.cal_cnt = 0;
      s_cal.align_ramp = (float)CAL_SWEEP_VOLT;
      s_cal.state = CAL_STATE_ALIGN;
    }
    foc_write_pwm_duty();
    break;
#endif

  default:
    foc_write_pwm_duty();
    break;
  }
}

/**
 * @func    foc_step
 * @brief   Execute one 20 kHz field-oriented control (FOC) iteration including Clarke/Park,
 *          outer-loop control (MIT Impedance or Position/Velocity), DQ current PI, and SVPWM.
 * @param   None
 * @retval  None
 */
static void foc_step(void) {
  int32_t tc, ts;
  int active;

  /* 1. Compute electrical angle and look up sin/cos */
  foc_update_elec_angle();
  tc = trig_cos(s_foc.elec);
  ts = trig_sin(s_foc.elec);

  /* 2. Sample phase currents and transform to synchronous d/q frame */
  foc_sample_phase_currents(tc, ts);

  /* 3. Update gate driver state, idle auto-zero offset, and mode transitions */
  active =
      (g_control_table.torque_enable == 1) && (g_control_table.calib_done == 1);
  foc_update_gate_and_idle_offset(active);

  /* 4. Run active control loop (Impedance / Position) or reset when disabled */
  if (!active) {
    foc_reset_inactive_state();
  } else if (g_control_table.control_mode == VEL_MODE_IMPEDANCE) {
    foc_ctrl_impedance_mode();
  } else {
    foc_ctrl_position_mode();
  }

  /* 5. Run Inverse Park + SVPWM (or calibration routine) and update TIM1 PWM */
  foc_apply_calib_and_pwm(tc, ts);
}

/* ------------------------------------------------------------------------------------------
 *  Public interface
 * ----------------------------------------------------------------------------------------
 */

/**
 * @func    motor_ctrl_fast_loop
 * @brief   High-frequency 20 kHz FOC control loop ISR entry point.
 * @param   raw_angle: Raw 14-bit mechanical rotor angle (0..16383)
 * @retval  None
 */
void motor_ctrl_fast_loop(int32_t raw_angle) {
  s_enc.raw_angle = raw_angle;
  angle_speed();
  foc_step();
}

/**
 * @func    motor_ctrl_load_calibration
 * @brief   Load current offsets, electrical angle offset, and default impedance gains from g_control_table.
 * @param   None
 * @retval  None
 */
void motor_ctrl_load_calibration(void) {
  s_foc.off_v = g_control_table.offset_v;
  s_foc.off_u = g_control_table.offset_u;
#if !MOTOR_USE_HALL_MT9102
  s_foc.off_v_12 = (int16_t)(s_foc.off_v << 2);
  s_foc.off_u_12 = (int16_t)(s_foc.off_u << 2);
  s_foc.off_v_acc_q8 = (int32_t)s_foc.off_v_12 << 8;
  s_foc.off_u_acc_q8 = (int32_t)s_foc.off_u_12 << 8;
#endif
  s_enc.angle_off = (int32_t)g_control_table.angle_offset;
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
  s_cmd.imp_kp = si_kp_to_rotor(g_control_table.kp);
  s_cmd.imp_kd = si_kd_to_rotor(g_control_table.kd);
  s_cmd.imp_tau_ff = si_tau_to_iq(g_control_table.tau_ff);
}

/**
 * @func    motor_ctrl_set_speed_cmd
 * @brief   Latch a new moving speed command from protocol context.
 * @param   speed: Moving speed command in RPM
 * @retval  None
 */
void motor_ctrl_set_speed_cmd(int16_t speed) { s_cmd.moving_speed = speed; }

/**
 * @func    motor_ctrl_set_goal_position
 * @brief   Latch a new goal position command from protocol context.
 * @param   position: Goal position in 14-bit output-shaft counts
 * @retval  None
 */
void motor_ctrl_set_goal_position(int16_t position) {
  s_cmd.goal_position = position;
}

/**
 * @func    motor_ctrl_set_kp
 * @brief   Convert and latch MIT impedance stiffness gain Kp from protocol context.
 * @param   kp: Output-shaft stiffness gain in SI register units (x0.001 N.m/rad)
 * @retval  None
 */
void motor_ctrl_set_kp(uint16_t kp) { s_cmd.imp_kp = si_kp_to_rotor(kp); }

/**
 * @func    motor_ctrl_set_kd
 * @brief   Convert and latch MIT impedance damping gain Kd from protocol context.
 * @param   kd: Output-shaft damping gain in SI register units (x0.001 N.m.s/rad)
 * @retval  None
 */
void motor_ctrl_set_kd(uint16_t kd) { s_cmd.imp_kd = si_kd_to_rotor(kd); }

/**
 * @func    motor_ctrl_set_tau_ff
 * @brief   Convert and latch MIT impedance feedforward torque from protocol context.
 * @param   tau_ff: Output-shaft feedforward torque in mN.m
 * @retval  None
 */
void motor_ctrl_set_tau_ff(int16_t tau_ff) {
  s_cmd.imp_tau_ff = si_tau_to_iq(tau_ff);
}

/**
 * @func    motor_ctrl_start_calibration
 * @brief   Start the non-blocking current-offset and rotor-alignment calibration state machine.
 * @param   None
 * @retval  true if calibration was started, false if already in progress
 */
bool motor_ctrl_start_calibration(void) {
  if (s_cal.state != CAL_STATE_IDLE) {
    return false;
  }
  g_control_table.calib_done = 0;
#if MOTOR_USE_HALL_MT9102
  g_control_table.torque_enable = 0;
  s_cal.cal_cnt = 0;
  s_cal.sum_t = 0;
  s_cal.sum_v = 0;
  s_cal.sum_u = 0;
  s_cal.align_cnt = 0;
  s_cal.align_ramp = 0.0f;
  s_cal.sweep_elec = 0;
  s_cal.sweep_elec_q8 = 0;
  s_cal.min_sin = 4095;
  s_cal.max_sin = 0;
  s_cal.min_cos = 4095;
  s_cal.max_cos = 0;
  s_cal.align_sum_raw = 0;
  s_cal.align_ref_raw = 0;
  s_cal.align_pos_plus = 0;
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
  s_cal.state = CAL_STATE_ZERO_TOTAL;
  return true;
}

/**
 * @func    motor_ctrl_get_position
 * @brief   Get the current output-shaft position clamped to [POSITION_MIN, POSITION_MAX].
 * @param   None
 * @retval  Current output-shaft position in 14-bit counts (16384 counts/rev)
 */
int16_t motor_ctrl_get_position(void) {
  int32_t out_pos = pos_rotor_to_out(s_enc.multiturn_14);
  if (out_pos > POSITION_MAX)
    return POSITION_MAX;
  if (out_pos < POSITION_MIN)
    return POSITION_MIN;
  return (int16_t)out_pos;
}

/**
 * @func    motor_ctrl_get_speed
 * @brief   Get the current estimated rotor speed.
 * @param   None
 * @retval  Estimated rotor speed in RPM
 */
int32_t motor_ctrl_get_speed(void) { return s_enc.speed; }

#if FEATURE_UART_STREAM_TELEMETRY
/**
 * @func    motor_ctrl_get_iu
 * @brief   Get the latest sampled phase U current for telemetry streaming.
 * @param   None
 * @retval  Phase U current in 10-bit ADC counts
 */
int16_t motor_ctrl_get_iu(void) { return s_foc.cur_iu; }

/**
 * @func    motor_ctrl_get_iv
 * @brief   Get the latest sampled phase V current for telemetry streaming.
 * @param   None
 * @retval  Phase V current in 10-bit ADC counts
 */
int16_t motor_ctrl_get_iv(void) { return s_foc.cur_iv; }

/**
 * @func    motor_ctrl_get_iw
 * @brief   Get the latest reconstructed phase W current for telemetry streaming.
 * @param   None
 * @retval  Phase W current in 10-bit ADC counts
 */
int16_t motor_ctrl_get_iw(void) { return s_foc.cur_iw; }

/**
 * @func    motor_ctrl_get_elec_angle
 * @brief   Get the current electrical angle for telemetry streaming.
 * @param   None
 * @retval  Electrical angle in 1/10000 rad (0..62831)
 */
int32_t motor_ctrl_get_elec_angle(void) { return s_foc.elec; }
#endif
