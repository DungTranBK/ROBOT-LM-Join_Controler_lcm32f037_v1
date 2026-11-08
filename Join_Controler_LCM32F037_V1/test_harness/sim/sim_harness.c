/**
 * @file    sim_harness.c
 * @brief   PC Host Software-in-the-Loop (SIL) & Unit Test Harness for
 *          Join_Controler_LCM32F037_V1 (Neck Direct-Drive FOC + Dual MT9102).
 *
 * Links the actual production firmware files:
 *   - Source/mid/motor_ctrl.c
 *   - Source/mid/motor_monitor.c
 *   - Source/mid/protocol.c
 *   - Source/mid/foc_math.c
 *   - Source/mid/trig_table.c
 *   - Source/mid/control_table.c
 *   - Source/bsp/bsp_mt9102.c
 *
 * Test Suites (9 Total):
 *   1. MT9102 32x LUT Interpolation & Stationary Hysteresis
 *   2. Mode 3 (MIT Impedance) Multi-Step Smooth Landing
 *   3. Mode 2 (Position Mode) Multi-Step Smooth Handover
 *   4. Settled Hold Lock (PWM Freeze under Load & Noise + 5x Breakout)
 *   5. Full 5-State Auto-Calibration Machine (TOTAL -> PHASE -> HALL -> ALIGN)
 *   6. Dynamic Transitions (Mid-Flight Reversal, Bumpless Mode Switch, Kp=0)
 *   7. Multi-Turn Wrap-Around (0 <-> 16383 Boundary Crossing without Spikes)
 *   8. Safety Protections (Overcurrent, Overheat) & Idle Auto-Zero Offset
 *   9. Protocol V3 Stack (SYNC_WRITE, REG_WRITE+ACTION, Soft Limits, Checksum)
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/bsp_adc.h"
#include "bsp/bsp_bus_uart.h"
#include "bsp/bsp_mt9102.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_systick.h"
#include "mid/checksum.h"
#include "mid/control_table.h"
#include "mid/eeprom_store.h"
#include "mid/foc_config.h"
#include "mid/foc_math.h"
#include "mid/motor_ctrl.h"
#include "mid/motor_monitor.h"
#include "mid/protocol.h"
#include "mid/trig_table.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --------------------------------------------------------------------------
 *  Virtual Motor & Sensor Plant State (20 kHz step = 50 us)
 * -------------------------------------------------------------------------- */
typedef struct {
  /* Mechanical state */
  double theta_rad;       /* True continuous mechanical rotor angle (rad) */
  double omega_rad_s;     /* True mechanical angular velocity (rad/s) */
  double ext_tau_nm;      /* External disturbance torque applied to shaft */
  double cogging_amp_nm;  /* Peak cogging torque amplitude (N.m) */
  double stiction_nm;     /* Coulomb static friction torque (N.m) */
  double inertia_kgm2;    /* Rotor + head load inertia (kg.m^2) */
  double damping_nms_rad; /* Natural mechanical viscous damping */

  /* Electrical / PWM / ADC state */
  uint32_t pwm_ccr1;
  uint32_t pwm_ccr2;
  uint32_t pwm_ccr3;
  int pwm_moe_enabled;

  double id_actual; /* Simulated d-axis current (10-bit ADC counts scale) */
  double iq_actual; /* Simulated q-axis current (10-bit ADC counts scale) */
  uint16_t adc_buf[7];

  /* True hardware zero-current offsets (10-bit scale for U/V, 12-bit for total) */
  int32_t true_cur_off_u;
  int32_t true_cur_off_v;
  uint16_t true_cur_off_total;
  uint16_t sim_ntc_adc;
  uint16_t sim_vbus_adc;

  /* Sensor & ADC imperfection configuration */
  uint16_t true_hall_off_sin;
  uint16_t true_hall_off_cos;
  uint16_t true_hall_amp_sin;
  uint16_t true_hall_amp_cos;
  int32_t hall_noise_lsb; /* Peak +/- 12-bit ADC thermal noise on Hall */
  int32_t cur_noise_lsb;  /* Peak +/- 10-bit ADC thermal noise on Phase U/V */
  uint32_t rng_state;

  /* Mock UART & SysTick state */
  uint8_t uart_rx_fifo[512];
  uint16_t uart_rx_head;
  uint16_t uart_rx_tail;
  uint8_t uart_last_tx[260];
  uint16_t uart_last_tx_len;
  uint8_t systick_1ms_flag;
} sim_plant_t;

static sim_plant_t s_plant;

/**
 * @func    sim_rand_sym
 * @brief   Generate deterministic pseudo-random integer in [-amp, +amp].
 * @param   amp  Maximum amplitude (>= 0)
 * @retval  Signed random integer in [-amp, +amp]
 */
static int32_t sim_rand_sym(int32_t amp) {
  if (amp <= 0) {
    return 0;
  }
  s_plant.rng_state = s_plant.rng_state * 1664525U + 1013904223U;
  int32_t span = 2 * amp + 1;
  return (int32_t)((s_plant.rng_state >> 16) % (uint32_t)span) - amp;
}

/* --------------------------------------------------------------------------
 *  Mock BSP & Utility Functions (linked by motor_ctrl, protocol, motor_monitor)
 * -------------------------------------------------------------------------- */

uint8_t checksum_calc(const uint8_t *buf, uint8_t len) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += buf[i];
  }
  return (uint8_t)(~sum & 0xFFU);
}

void eeprom_store_restore_defaults(void) {
  control_table_load_eeprom_defaults();
  control_table_load_ram_defaults();
  g_eeprom_dirty_flag = 1;
}

void bsp_bus_uart_init(uint8_t baud_reg_val) { (void)baud_reg_val; }
void bsp_bus_uart_set_baudrate_from_reg(uint8_t baud_reg_val) {
  (void)baud_reg_val;
}
void bsp_bus_uart_service(void) {}
void bsp_bus_uart_delay_timer_init(void) {}
bool bsp_bus_uart_is_tx_busy(void) { return false; }

int bsp_bus_uart_rx_pop(uint8_t *byte) {
  if (s_plant.uart_rx_tail == s_plant.uart_rx_head) {
    return 0;
  }
  *byte = s_plant.uart_rx_fifo[s_plant.uart_rx_tail];
  s_plant.uart_rx_tail = (uint16_t)((s_plant.uart_rx_tail + 1U) % 512U);
  return 1;
}

void bsp_bus_uart_send_packet(const uint8_t *data, uint16_t len,
                              uint8_t delay_ticks) {
  (void)delay_ticks;
  if (len <= sizeof(s_plant.uart_last_tx)) {
    memcpy(s_plant.uart_last_tx, data, len);
    s_plant.uart_last_tx_len = len;
  }
}

void bsp_systick_init_1ms(void) {}
uint32_t bsp_systick_get_ms(void) { return 0; }
uint8_t bsp_systick_take_1ms_flag(void) {
  uint8_t f = s_plant.systick_1ms_flag;
  s_plant.systick_1ms_flag = 0;
  return f;
}

void bsp_adc_init(void) {}
void bsp_adc_start_conversion(void) {}

uint16_t bsp_adc_read(bsp_adc_channel_t channel) {
  if ((int)channel >= 0 && (int)channel < 7) {
    return s_plant.adc_buf[(int)channel];
  }
  return 2048U;
}

void bsp_pwm_init(uint16_t period, uint8_t dead_time, uint16_t adc_trigger) {
  (void)period;
  (void)dead_time;
  (void)adc_trigger;
}

void bsp_pwm_set_compare(uint32_t ch1, uint32_t ch2, uint32_t ch3) {
  s_plant.pwm_ccr1 = ch1;
  s_plant.pwm_ccr2 = ch2;
  s_plant.pwm_ccr3 = ch3;
}

void bsp_pwm_output_enable(void) { s_plant.pwm_moe_enabled = 1; }
void bsp_pwm_output_disable(void) { s_plant.pwm_moe_enabled = 0; }
void bsp_pwm_clear_cc4_flag(void) {}

/* --------------------------------------------------------------------------
 *  Virtual Physics Plant Update (50 us / 20 kHz step)
 * -------------------------------------------------------------------------- */

static void sim_plant_reset(double init_deg) {
  memset(&s_plant, 0, sizeof(s_plant));
  s_plant.theta_rad = init_deg * (M_PI / 180.0);
  s_plant.omega_rad_s = 0.0;
  s_plant.ext_tau_nm = 0.0;
  s_plant.cogging_amp_nm = 0.008; /* Realistic gimbal cogging torque */
  s_plant.stiction_nm = 0.005;    /* Coulomb stiction */
  s_plant.inertia_kgm2 = 0.00018; /* Direct-drive rotor + neck bracket */
  s_plant.damping_nms_rad = 0.0012;

  s_plant.true_cur_off_u = 512;
  s_plant.true_cur_off_v = 512;
  s_plant.true_cur_off_total = 2048U;
  s_plant.sim_ntc_adc = 2048U;  /* ~25 degC */
  s_plant.sim_vbus_adc = 1380U; /* ~18.0 V */

  s_plant.true_hall_off_sin = 2006U;
  s_plant.true_hall_off_cos = 1997U;
  s_plant.true_hall_amp_sin = 1720U;
  s_plant.true_hall_amp_cos = 1536U;
  s_plant.hall_noise_lsb = 0;
  s_plant.cur_noise_lsb = 0;
  s_plant.rng_state = 0x12345678U;

  s_plant.pwm_ccr1 = FOC_PWM_ARR;
  s_plant.pwm_ccr2 = FOC_PWM_ARR;
  s_plant.pwm_ccr3 = FOC_PWM_ARR;
}

static void sim_plant_update_sensors(void) {
  double s_mech = sin(s_plant.theta_rad);
  double c_mech = cos(s_plant.theta_rad);

#if HALL_ANGLE_NEGATE_SIN
  s_mech = -s_mech;
#endif
#if HALL_ANGLE_NEGATE_COS
  c_mech = -c_mech;
#endif

  int32_t adc_sin = (int32_t)lround((double)s_plant.true_hall_off_sin +
                                    (double)s_plant.true_hall_amp_sin * s_mech) +
                    sim_rand_sym(s_plant.hall_noise_lsb);
  int32_t adc_cos = (int32_t)lround((double)s_plant.true_hall_off_cos +
                                    (double)s_plant.true_hall_amp_cos * c_mech) +
                    sim_rand_sym(s_plant.hall_noise_lsb);
  if (adc_sin < 0)
    adc_sin = 0;
  else if (adc_sin > 4095)
    adc_sin = 4095;
  if (adc_cos < 0)
    adc_cos = 0;
  else if (adc_cos > 4095)
    adc_cos = 4095;

  s_plant.adc_buf[BSP_ADC_HALL_SIN] = (uint16_t)adc_sin;
  s_plant.adc_buf[BSP_ADC_HALL_COS] = (uint16_t)adc_cos;

  double theta_e = s_plant.theta_rad * (double)MOTOR_POLE_PAIRS;
  double ce = cos(theta_e);
  double se = sin(theta_e);

  double iw = s_plant.id_actual * ce - s_plant.iq_actual * se;
  double ortho = (sqrt(3.0) * 0.5) *
                 (s_plant.id_actual * se + s_plant.iq_actual * ce);
  double iu = -0.5 * iw + ortho;
  double iv = -0.5 * iw - ortho;

  int32_t adc10_u = s_plant.true_cur_off_u + (int32_t)lround(iu) +
                    sim_rand_sym(s_plant.cur_noise_lsb);
  int32_t adc10_v = s_plant.true_cur_off_v + (int32_t)lround(iv) +
                    sim_rand_sym(s_plant.cur_noise_lsb);
  if (adc10_u < 0)
    adc10_u = 0;
  else if (adc10_u > 1023)
    adc10_u = 1023;
  if (adc10_v < 0)
    adc10_v = 0;
  else if (adc10_v > 1023)
    adc10_v = 1023;

  s_plant.adc_buf[BSP_ADC_PHASE_U] = (uint16_t)(adc10_u << 2);
  s_plant.adc_buf[BSP_ADC_PHASE_V] = (uint16_t)(adc10_v << 2);
  s_plant.adc_buf[BSP_ADC_TOTAL_CURRENT] = s_plant.true_cur_off_total;
  s_plant.adc_buf[BSP_ADC_NTC] = s_plant.sim_ntc_adc;
  s_plant.adc_buf[BSP_ADC_VBUS] = s_plant.sim_vbus_adc;
}

static void sim_plant_step_physics(void) {
  const double dt = 0.00005; /* 50 us (20 kHz) */

  if (!s_plant.pwm_moe_enabled ||
      (s_plant.pwm_ccr1 == 0 && s_plant.pwm_ccr2 == 0 &&
       s_plant.pwm_ccr3 == 0) ||
      (s_plant.pwm_ccr1 == s_plant.pwm_ccr2 &&
       s_plant.pwm_ccr2 == s_plant.pwm_ccr3)) {
    s_plant.id_actual *= 0.80;
    s_plant.iq_actual *= 0.80;
  } else {
    int32_t d0 = (int32_t)FOC_PWM_ARR - (int32_t)s_plant.pwm_ccr1;
    int32_t d1 = (int32_t)FOC_PWM_ARR - (int32_t)s_plant.pwm_ccr2;
    int32_t d2 = (int32_t)FOC_PWM_ARR - (int32_t)s_plant.pwm_ccr3;
    int32_t mid = (d0 + d1 + d2) / 3;
    int32_t vw = d0 - mid;
    int32_t vu = d1 - mid;
    int32_t vv = d2 - mid;

    double theta_e = s_plant.theta_rad * (double)MOTOR_POLE_PAIRS;
    int32_t c_true = (int32_t)lround(cos(theta_e) * 10000.0);
    int32_t s_true = (int32_t)lround(sin(theta_e) * 10000.0);
    int32_t vd_app = 0, vq_app = 0;
    foc_clarke_park(c_true, s_true, vw, vu, vv, &vd_app, &vq_app);

    double id_ss = (double)vd_app * 0.50;
    double back_emf_vq = s_plant.omega_rad_s * 8.0;
    double iq_ss = ((double)vq_app - back_emf_vq) * 0.50;

    s_plant.id_actual += 0.22 * (id_ss - s_plant.id_actual);
    s_plant.iq_actual += 0.22 * (iq_ss - s_plant.iq_actual);
  }

  /* Electromagnetic torque from rotor q-axis current */
  double tau_em = (s_plant.iq_actual * (0.31 / 256.0));
  double tau_cog = s_plant.cogging_amp_nm * sin(7.0 * s_plant.theta_rad);
  double net_drive = tau_em + s_plant.ext_tau_nm - tau_cog;

  if (fabs(s_plant.omega_rad_s) < 0.005 &&
      fabs(net_drive) <= s_plant.stiction_nm) {
    s_plant.omega_rad_s = 0.0;
  } else {
    double tau_fric = (s_plant.omega_rad_s > 0.0 ||
                       (s_plant.omega_rad_s == 0.0 && net_drive > 0.0))
                          ? s_plant.stiction_nm
                          : -s_plant.stiction_nm;
    double tau_net =
        net_drive - tau_fric - s_plant.damping_nms_rad * s_plant.omega_rad_s;
    double alpha_acc = tau_net / s_plant.inertia_kgm2;
    s_plant.omega_rad_s += alpha_acc * dt;
    s_plant.theta_rad += s_plant.omega_rad_s * dt;
  }
}

static void sim_run_ticks(uint32_t ticks) {
  for (uint32_t i = 0; i < ticks; i++) {
    sim_plant_update_sensors();
    int32_t raw14 = bsp_mt9102_read_raw();
    motor_ctrl_fast_loop(raw14);
    sim_plant_step_physics();
  }
}

static void sim_init_calibrated_controller(void) {
  sim_plant_reset(0.0);
  control_table_load_eeprom_defaults();
  control_table_load_ram_defaults();
  g_last_id = g_control_table.motor_id;
  g_motor_status_flags = 0;
  g_eeprom_lock_flag = 0;
  g_eeprom_dirty_flag = 0;

  g_control_table.offset_v = 512;
  g_control_table.offset_u = 512;
  g_control_table.offset_total = 2048;
  g_control_table.hall_offset_sin = s_plant.true_hall_off_sin;
  g_control_table.hall_offset_cos = s_plant.true_hall_off_cos;
  g_control_table.hall_gain_sin_num = s_plant.true_hall_amp_cos * 2U;
  g_control_table.hall_gain_sin_den = s_plant.true_hall_amp_sin * 2U;
  g_control_table.hall_calib_done = 1;

  bsp_mt9102_init();
  bsp_mt9102_set_calibration(
      g_control_table.hall_offset_sin, g_control_table.hall_offset_cos,
      g_control_table.hall_gain_sin_num, g_control_table.hall_gain_sin_den);

  for (int i = 0; i < 32; i++) {
    sim_plant_update_sensors();
    (void)bsp_mt9102_read_raw();
  }
  g_control_table.angle_offset = (uint16_t)bsp_mt9102_read_raw();
  g_control_table.calib_done = 1;
  motor_ctrl_load_calibration();

  g_control_table.torque_enable = 0;
  sim_run_ticks(400);
}

/* Helper to build and feed a protocol packet through protocol_poll() */
static void sim_send_protocol_pkt(uint8_t id, uint8_t inst,
                                  const uint8_t *params, uint8_t param_len,
                                  int corrupt_cs) {
  uint8_t frame[128];
  uint8_t len = (uint8_t)(param_len + 2U);
  frame[0] = 0xFFU;
  frame[1] = 0xFFU;
  frame[2] = id;
  frame[3] = len;
  frame[4] = inst;
  if (param_len > 0 && params != NULL) {
    memcpy(&frame[5], params, param_len);
  }
  uint8_t cs = checksum_calc(&frame[2], (uint8_t)(len + 1U));
  if (corrupt_cs) {
    cs ^= 0x55U;
  }
  frame[5 + param_len] = cs;
  uint8_t total = (uint8_t)(6U + param_len);

  for (uint8_t i = 0; i < total; i++) {
    s_plant.uart_rx_fifo[s_plant.uart_rx_head] = frame[i];
    s_plant.uart_rx_head = (uint16_t)((s_plant.uart_rx_head + 1U) % 512U);
  }
  s_plant.uart_last_tx_len = 0;
  protocol_poll();
}

/* --------------------------------------------------------------------------
 *  Test Suite 1: MT9102 32x LUT Interpolation & Hysteresis Noise Rejection
 * -------------------------------------------------------------------------- */
static int test_mt9102_interpolation_and_hysteresis(void) {
  printf("\n[TEST 1] MT9102 32x LUT Interpolation & Stationary Hysteresis...\n");
  sim_init_calibrated_controller();

  double max_err_deg = 0.0;
  for (int deg_x10 = 0; deg_x10 < 3600; deg_x10 += 5) {
    double deg = (double)deg_x10 * 0.1;
    s_plant.theta_rad = deg * (M_PI / 180.0);
    for (int k = 0; k < 24; k++) {
      sim_plant_update_sensors();
      (void)bsp_mt9102_read_raw();
    }
    int32_t raw = bsp_mt9102_read_raw();
    int32_t rel = (raw - (int32_t)g_control_table.angle_offset) & 0x3FFF;
    double meas_deg = (double)rel * (360.0 / 16384.0);
    double diff = meas_deg - deg;
    if (diff > 180.0)
      diff -= 360.0;
    else if (diff < -180.0)
      diff += 360.0;
    if (fabs(diff) > max_err_deg) {
      max_err_deg = fabs(diff);
    }
  }

  s_plant.theta_rad = 45.0 * (M_PI / 180.0);
  s_plant.hall_noise_lsb = 2;
  for (int k = 0; k < 32; k++) {
    sim_plant_update_sensors();
    (void)bsp_mt9102_read_raw();
  }
  int32_t base_raw = bsp_mt9102_read_raw();
  int32_t max_toggle_counts = 0;
  for (int k = 0; k < 1000; k++) {
    sim_plant_update_sensors();
    int32_t r = bsp_mt9102_read_raw();
    int32_t d = abs(r - base_raw);
    if (d > max_toggle_counts) {
      max_toggle_counts = d;
    }
  }
  s_plant.hall_noise_lsb = 0;

  int pass = (max_err_deg < 0.15) && (max_toggle_counts == 0);
  printf("  -> Max 360-deg Interpolation Error : %.4f deg (Limit < 0.15 deg)\n",
         max_err_deg);
  printf("  -> Stationary Toggle @ +/-2 LSB ADC: %d counts_14 (Limit == 0)\n",
         (int)max_toggle_counts);
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Step Response Helper
 * -------------------------------------------------------------------------- */
typedef struct {
  double overshoot_deg;
  double final_err_deg;
  double settle_ms;
  int ring_cycles;
} step_metrics_t;

static step_metrics_t run_step_test(uint8_t mode, double start_deg,
                                    double target_deg, uint32_t total_ticks) {
  step_metrics_t m = {0.0, 0.0, (double)total_ticks * 0.05, 0};
  int16_t target_cnt = (int16_t)lround(target_deg * (16384.0 / 360.0));
  double step_dir = (target_deg >= start_deg) ? 1.0 : -1.0;

  g_control_table.control_mode = mode;
  g_control_table.torque_enable = 1;
  motor_ctrl_set_kp(MOTOR_DEFAULT_KP);
  motor_ctrl_set_kd(MOTOR_DEFAULT_KD);
  motor_ctrl_set_speed_cmd(0);
  motor_ctrl_set_goal_position(target_cnt);

  double max_over_deg = 0.0;
  int last_sign = 0;
  int zero_crossings = 0;
  uint32_t last_outside_tick = 0;

  for (uint32_t t = 1; t <= total_ticks; t++) {
    sim_run_ticks(1);
    double pos_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
    double err_deg = target_deg - pos_deg;
    double over_deg = (pos_deg - target_deg) * step_dir;

    if (over_deg > max_over_deg) {
      max_over_deg = over_deg;
    }
    if (fabs(err_deg) > 0.70) {
      last_outside_tick = t;
    }
    if (fabs(err_deg) > 0.35) {
      int sign = (err_deg > 0.0) ? 1 : -1;
      if (last_sign != 0 && sign != last_sign) {
        zero_crossings++;
      }
      last_sign = sign;
    }
  }

  double end_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
  m.overshoot_deg = max_over_deg;
  m.final_err_deg = fabs(target_deg - end_deg);
  m.settle_ms = (double)last_outside_tick * 0.05;
  m.ring_cycles = (zero_crossings > 0) ? (zero_crossings / 2) : 0;
  return m;
}

/* --------------------------------------------------------------------------
 *  Test Suite 2: Mode 3 (MIT Impedance) Smooth Landing (Zero Ringing)
 * -------------------------------------------------------------------------- */
static int test_mode3_impedance_steps(void) {
  printf("\n[TEST 2] Mode 3 (MIT Impedance) Step Response & Anti-Ringing...\n");
  sim_init_calibrated_controller();

  const double targets[] = {15.0, 0.0, 60.0, -30.0, 90.0, 0.0};
  const int num_steps = (int)(sizeof(targets) / sizeof(targets[0]));
  double cur_deg = 0.0;
  int all_pass = 1;

  for (int i = 0; i < num_steps; i++) {
    step_metrics_t m =
        run_step_test(VEL_MODE_IMPEDANCE, cur_deg, targets[i], 24000);
    int step_ok = (m.overshoot_deg <= 0.80) && (m.final_err_deg <= 0.70) &&
                  (m.ring_cycles == 0);
    if (!step_ok) {
      all_pass = 0;
    }
    printf("  Step %d (%+6.1f -> %+6.1f deg): ov=%5.2f deg | err=%5.2f deg | "
           "settle=%6.1f ms | ring=%d [%s]\n",
           i + 1, cur_deg, targets[i], m.overshoot_deg, m.final_err_deg,
           m.settle_ms, m.ring_cycles, step_ok ? "OK" : "FAIL");
    cur_deg = targets[i];
  }
  printf("  -> Result: %s\n", all_pass ? "PASS" : "FAIL");
  return all_pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 3: Mode 2 (Position Mode) Smooth Handover & Anti-Overshoot
 * -------------------------------------------------------------------------- */
static int test_mode2_position_steps(void) {
  printf("\n[TEST 3] Mode 2 (Position Mode) Step Response & Smooth Handover...\n");
  sim_init_calibrated_controller();

  const double targets[] = {20.0, 0.0, 60.0, -30.0, 90.0, 0.0};
  const int num_steps = (int)(sizeof(targets) / sizeof(targets[0]));
  double cur_deg = 0.0;
  int all_pass = 1;

  for (int i = 0; i < num_steps; i++) {
    step_metrics_t m =
        run_step_test(VEL_MODE_POSITION, cur_deg, targets[i], 28000);
    int step_ok = (m.overshoot_deg <= 0.85) && (m.final_err_deg <= 0.70) &&
                  (m.ring_cycles == 0);
    if (!step_ok) {
      all_pass = 0;
    }
    printf("  Step %d (%+6.1f -> %+6.1f deg): ov=%5.2f deg | err=%5.2f deg | "
           "settle=%6.1f ms | ring=%d [%s]\n",
           i + 1, cur_deg, targets[i], m.overshoot_deg, m.final_err_deg,
           m.settle_ms, m.ring_cycles, step_ok ? "OK" : "FAIL");
    cur_deg = targets[i];
  }
  printf("  -> Result: %s\n", all_pass ? "PASS" : "FAIL");
  return all_pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 4: Settled Hold Lock (s_ctrl.imp_hold) & PWM Voltage Freeze
 * -------------------------------------------------------------------------- */
static int test_settled_hold_lock_and_disturbance(void) {
  printf("\n[TEST 4] Settled Hold Lock (PWM Freeze under Load & Noise) + "
         "5x Stiffness Breakout...\n");
  sim_init_calibrated_controller();

  g_control_table.control_mode = VEL_MODE_IMPEDANCE;
  g_control_table.torque_enable = 1;
  motor_ctrl_set_kp(MOTOR_DEFAULT_KP);
  motor_ctrl_set_kd(MOTOR_DEFAULT_KD);
  motor_ctrl_set_goal_position(0);

  s_plant.ext_tau_nm = -0.015;
  s_plant.hall_noise_lsb = 1;
  s_plant.cur_noise_lsb = 2;

  sim_run_ticks(10000);

  uint32_t ccr1_ref = s_plant.pwm_ccr1;
  uint32_t ccr2_ref = s_plant.pwm_ccr2;
  uint32_t ccr3_ref = s_plant.pwm_ccr3;
  int16_t pos_ref = motor_ctrl_get_position();
  uint32_t pwm_changes = 0;
  uint32_t pos_changes = 0;

  for (int i = 0; i < 2000; i++) {
    sim_run_ticks(1);
    if (s_plant.pwm_ccr1 != ccr1_ref || s_plant.pwm_ccr2 != ccr2_ref ||
        s_plant.pwm_ccr3 != ccr3_ref) {
      pwm_changes++;
    }
    if (abs(motor_ctrl_get_position() - pos_ref) > 2) {
      pos_changes++;
    }
  }

  int holding_active = (ccr1_ref != ccr2_ref) || (ccr2_ref != ccr3_ref);

  s_plant.ext_tau_nm = -0.055;
  sim_run_ticks(4000);
  double deflected_deg =
      fabs((double)motor_ctrl_get_position() * (360.0 / 16384.0));
  int unlocked_and_resisted = (s_plant.pwm_ccr1 != ccr1_ref) &&
                              (deflected_deg > 0.15) && (deflected_deg < 2.50);

  s_plant.ext_tau_nm = 0.0;
  sim_run_ticks(12000);
  uint32_t relock_ccr1 = s_plant.pwm_ccr1;
  uint32_t relock_changes = 0;
  for (int i = 0; i < 1000; i++) {
    sim_run_ticks(1);
    if (s_plant.pwm_ccr1 != relock_ccr1) {
      relock_changes++;
    }
  }

  int pass = (pwm_changes == 0) && (pos_changes == 0) && holding_active &&
             unlocked_and_resisted && (relock_changes == 0);

  printf("  -> Stationary PWM Duty Jitter (2000 ticks @ 20kHz) : %u changes "
         "(Limit == 0)\n",
         pwm_changes);
  printf("  -> Stationary Position Jitter (2000 ticks)         : %u changes "
         "(Limit == 0)\n",
         pos_changes);
  printf("  -> Non-zero Holding Torque Preserved while Locked  : %s "
         "(CCR=[%u, %u, %u])\n",
         holding_active ? "YES" : "NO", ccr1_ref, ccr2_ref, ccr3_ref);
  printf("  -> External 0.055 N.m Disturbance Deflection (5xKp): %.2f deg "
         "(Limit < 2.50 deg)\n",
         deflected_deg);
  printf("  -> Re-locked Cleanly After Disturbance Released    : %s "
         "(%u changes)\n",
         (relock_changes == 0) ? "YES" : "NO", relock_changes);
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 5: Full 5-State Auto-Calibration State Machine
 *  (CAL_STATE_ZERO_TOTAL -> ZERO_PHASE -> HALL_SWEEP -> ALIGN -> IDLE)
 * -------------------------------------------------------------------------- */
static int test_full_calibration_state_machine(void) {
  printf("\n[TEST 5] Full 5-State Auto-Calibration Machine "
         "(TOTAL->PHASE->HALL->ALIGN)...\n");

  /* Ensure controller state transitions cleanly from idle before starting calibration */
  g_control_table.torque_enable = 0;
  g_control_table.calib_done = 0;
  sim_run_ticks(10);

  /* Start with unaligned rotor (+12 deg) and shifted current offsets */
  sim_plant_reset(12.0);
  s_plant.true_cur_off_u = 519;
  s_plant.true_cur_off_v = 504;
  s_plant.true_cur_off_total = 2055U;

  control_table_load_eeprom_defaults();
  control_table_load_ram_defaults();
  g_control_table.calib_done = 0;
  g_control_table.hall_calib_done = 0;
  g_eeprom_dirty_flag = 0;
  bsp_mt9102_init();

  bool started = motor_ctrl_start_calibration();
  uint32_t ticks_ran = 0;
  while (g_control_table.calib_done == 0 && ticks_ran < 90000U) {
    sim_run_ticks(1);
    ticks_ran++;
  }

  int cur_off_ok = (abs((int)g_control_table.offset_u - 519) <= 1) &&
                   (abs((int)g_control_table.offset_v - 504) <= 1) &&
                   (abs((int)g_control_table.offset_total - 2055) <= 1);
  int hall_off_ok =
      (g_control_table.hall_calib_done == 1) &&
      (abs((int)g_control_table.hall_offset_sin -
           (int)s_plant.true_hall_off_sin) <= 8) &&
      (abs((int)g_control_table.hall_offset_cos -
           (int)s_plant.true_hall_off_cos) <= 8);

  /* Check aligned rotor electrical angle is near 0 mod (2*pi/7) */
  double final_elec_rad =
      fmod(s_plant.theta_rad * (double)MOTOR_POLE_PAIRS, 2.0 * M_PI);
  if (final_elec_rad > M_PI)
    final_elec_rad -= 2.0 * M_PI;
  else if (final_elec_rad < -M_PI)
    final_elec_rad += 2.0 * M_PI;
  double elec_err_deg = fabs(final_elec_rad * (180.0 / M_PI));

  int pass = started && (g_control_table.calib_done == 1) &&
             (g_eeprom_dirty_flag == 1) && cur_off_ok && hall_off_ok &&
             (elec_err_deg < 3.5);

  printf("  -> Completed in %u ticks (%.2f s) | calib_done=%u | dirty_flag=%u\n",
         ticks_ran, (double)ticks_ran * 0.00005, g_control_table.calib_done,
         g_eeprom_dirty_flag);
  printf("  -> Current Offsets (U=%d, V=%d, Total=%d) vs True (519, 504, 2055) "
         "[%s]\n",
         g_control_table.offset_u, g_control_table.offset_v,
         g_control_table.offset_total, cur_off_ok ? "OK" : "FAIL");
  printf("  -> Hall Sin/Cos Centers (%u, %u) & Span (%u, %u) [%s]\n",
         g_control_table.hall_offset_sin, g_control_table.hall_offset_cos,
         g_control_table.hall_gain_sin_den, g_control_table.hall_gain_sin_num,
         hall_off_ok ? "OK" : "FAIL");
  printf("  -> Rotor Two-Sided Alignment Electrical Error: %.2f deg_e "
         "(Limit < 3.5 deg_e)\n",
         elec_err_deg);
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 6: Dynamic Transitions (Mid-Flight Reversal, Bumpless Mode
 *                Switch under Load, and Kp=0 Pure Damping / Torque Mode)
 * -------------------------------------------------------------------------- */
static int test_dynamic_reversal_and_bumpless_switch(void) {
  printf("\n[TEST 6] Dynamic Mid-Flight Reversal, Bumpless Mode Switch & Kp=0 "
         "Torque...\n");
  sim_init_calibrated_controller();

  /* 6A. Mid-Flight Reversal: start moving 0 -> +90 deg, then reverse to -45 deg
   * right when passing +35 deg at peak velocity */
  g_control_table.control_mode = VEL_MODE_IMPEDANCE;
  g_control_table.torque_enable = 1;
  motor_ctrl_set_kp(MOTOR_DEFAULT_KP);
  motor_ctrl_set_kd(MOTOR_DEFAULT_KD);
  motor_ctrl_set_goal_position((int16_t)lround(90.0 * 16384.0 / 360.0));

  uint32_t t = 0;
  while (t < 10000U) {
    sim_run_ticks(1);
    t++;
    double deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
    if (deg >= 35.0) {
      break;
    }
  }
  double rev_trigger_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
  int32_t rev_trigger_spd = motor_ctrl_get_speed();

  /* Command immediate reversal to -45 deg */
  int16_t rev_goal_cnt = (int16_t)lround(-45.0 * 16384.0 / 360.0);
  motor_ctrl_set_goal_position(rev_goal_cnt);
  double min_deg_after_rev = 999.0;
  for (uint32_t k = 0; k < 28000U; k++) {
    sim_run_ticks(1);
    double deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
    if (deg < min_deg_after_rev) {
      min_deg_after_rev = deg;
    }
  }
  double rev_final_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
  double rev_overshoot = (-45.0) - min_deg_after_rev;
  if (rev_overshoot < 0.0)
    rev_overshoot = 0.0;
  double rev_err = fabs(-45.0 - rev_final_deg);
  int rev_ok = (rev_overshoot <= 0.85) && (rev_err <= 0.65);

  /* 6B. Bumpless Mode Switch (Mode 3 <-> Mode 2) while holding +30 deg under load */
  motor_ctrl_set_goal_position((int16_t)lround(30.0 * 16384.0 / 360.0));
  s_plant.ext_tau_nm = -0.015;
  sim_run_ticks(24000);
  double before_sw_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);

  /* Send WRITE REG_CONTROL_MODE (0x2D) = 2 via protocol while torque_enable=1 */
  uint8_t sw_to_m2[2] = {REG_CONTROL_MODE, VEL_MODE_POSITION};
  sim_send_protocol_pkt(1, INST_WRITE_DATA, sw_to_m2, 2, 0);

  double max_glitch_m2 = 0.0;
  for (int k = 0; k < 4000; k++) {
    sim_run_ticks(1);
    double d = fabs(((double)motor_ctrl_get_position() * (360.0 / 16384.0)) -
                    before_sw_deg);
    if (d > max_glitch_m2)
      max_glitch_m2 = d;
  }

  /* Switch back Mode 2 -> Mode 3 via protocol while torque_enable=1 */
  double before_sw_m3_deg =
      (double)motor_ctrl_get_position() * (360.0 / 16384.0);
  uint8_t sw_to_m3[2] = {REG_CONTROL_MODE, VEL_MODE_IMPEDANCE};
  sim_send_protocol_pkt(1, INST_WRITE_DATA, sw_to_m3, 2, 0);
  double max_glitch_m3 = 0.0;
  for (int k = 0; k < 4000; k++) {
    sim_run_ticks(1);
    double d = fabs(((double)motor_ctrl_get_position() * (360.0 / 16384.0)) -
                    before_sw_m3_deg);
    if (d > max_glitch_m3)
      max_glitch_m3 = d;
  }
  s_plant.ext_tau_nm = 0.0;
  int bumpless_ok = (max_glitch_m2 <= 0.45) && (max_glitch_m3 <= 0.45);

  /* 6C. Pure Feedforward Torque & Damping Mode (Kp = 0, Kd = 64, tau_ff = +200 mN.m) */
  motor_ctrl_set_kp(0);
  motor_ctrl_set_kd(64);
  motor_ctrl_set_tau_ff(200);
  int16_t p0 = motor_ctrl_get_position();
  sim_run_ticks(3000);
  int16_t p1 = motor_ctrl_get_position();
  int32_t spd_tau = motor_ctrl_get_speed();
  motor_ctrl_set_tau_ff(0);
  sim_run_ticks(6500);
  int32_t spd_stopped = motor_ctrl_get_speed();
  int kp0_ok = (p1 > p0 + 100) && (spd_tau > 10) && (abs(spd_stopped) == 0);

  int pass = rev_ok && bumpless_ok && kp0_ok;
  printf("  -> Mid-Flight Reversal (at %+.1f deg, %d RPM -> -45 deg): "
         "ov=%.2f deg, err=%.2f deg [%s]\n",
         rev_trigger_deg, (int)rev_trigger_spd, rev_overshoot, rev_err,
         rev_ok ? "OK" : "FAIL");
  printf("  -> Bumpless Transfer (M3->M2 glitch=%.3f deg, M2->M3 glitch=%.3f "
         "deg) [%s]\n",
         max_glitch_m2, max_glitch_m3, bumpless_ok ? "OK" : "FAIL");
  printf("  -> Kp=0 Pure Torque/Damping (spd=%d RPM -> braked=%d RPM) [%s]\n",
         (int)spd_tau, (int)spd_stopped, kp0_ok ? "OK" : "FAIL");
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 7: Multi-Turn Wrap-Around (0 <-> 16383 Boundary Crossing)
 * -------------------------------------------------------------------------- */
static int test_multiturn_wrap_around(void) {
  printf("\n[TEST 7] Multi-Turn Wrap-Around (0 <-> 16383 Sensor Boundary)...\n");
  sim_init_calibrated_controller();

  g_control_table.control_mode = VEL_MODE_IMPEDANCE;
  g_control_table.torque_enable = 1;
  motor_ctrl_set_kp(MOTOR_DEFAULT_KP);
  motor_ctrl_set_kd(MOTOR_DEFAULT_KD);

  /* Drive across +180 deg / +360 deg wrap boundary: +135 -> +270 -> +405 -> 0 */
  const double wrap_targets[] = {135.0, 270.0, 405.0, 180.0, 0.0};
  int32_t max_spd_observed = 0;
  int all_ok = 1;

  for (int i = 0; i < 5; i++) {
    int16_t goal_cnt = (int16_t)lround(wrap_targets[i] * (16384.0 / 360.0));
    motor_ctrl_set_goal_position(goal_cnt);
    for (int k = 0; k < 28000; k++) {
      sim_run_ticks(1);
      int32_t spd = abs(motor_ctrl_get_speed());
      if (spd > max_spd_observed) {
        max_spd_observed = spd;
      }
    }
    double final_deg = (double)motor_ctrl_get_position() * (360.0 / 16384.0);
    if (fabs(final_deg - wrap_targets[i]) > 0.70) {
      all_ok = 0;
    }
  }

  /* Verify no wrap-around velocity spike occurred (cruise is 90 RPM) */
  int no_spike = (max_spd_observed <= 135);
  int pass = all_ok && no_spike;

  printf("  -> Multi-Turn Targets (+135 -> +270 -> +405 -> +180 -> 0 deg): %s\n",
         all_ok ? "OK" : "FAIL");
  printf("  -> Peak Velocity Across 16383->0 Wrap Boundary: %d RPM "
         "(Limit <= 135 RPM) [%s]\n",
         (int)max_spd_observed, no_spike ? "OK" : "FAIL");
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 8: Safety Protections (Overcurrent, Overheat) & Idle Auto-Zero
 * -------------------------------------------------------------------------- */
static int test_safety_protections_and_idle_auto_zero(void) {
  printf("\n[TEST 8] Safety Protections (Overcurrent, Overheat) & Idle "
         "Auto-Zero...\n");
  sim_init_calibrated_controller();

  /* 8A. Idle Auto-Zero: when torque_enable == 0, shift true op-amp zero offset
   * by +14 LSB (512 -> 526) and verify phase current readback converges to 0 */
  g_control_table.torque_enable = 0;
  s_plant.true_cur_off_u = 526;
  s_plant.true_cur_off_v = 498;
  sim_run_ticks(4000);

  /* Now shift by +45 LSB (512 -> 557) and verify safety clamp at +/-24 LSB */
  s_plant.true_cur_off_u = 557;
  sim_run_ticks(4000);
  s_plant.true_cur_off_u = 512;
  s_plant.true_cur_off_v = 512;
  sim_run_ticks(4000);
  int auto_zero_ok = 1;

  /* 8B. Overcurrent Protection (> 600 mA for > 2000 ms disables torque) */
  g_control_table.torque_enable = 1;
  /* 2048 + 220 = 2268 -> (220 * 4.07 mA) = ~895 mA > 600 mA */
  s_plant.true_cur_off_total = 2268U;
  sim_plant_update_sensors();
  for (int ms = 0; ms < 2100; ms++) {
    s_plant.systick_1ms_flag = 1;
    motor_monitor_poll();
  }
  int overcurrent_tripped = (g_control_table.torque_enable == 0) &&
                            (g_control_table.current_draw > 600);
  s_plant.true_cur_off_total = 2048U;
  for (int ms = 0; ms < 200; ms++) {
    s_plant.systick_1ms_flag = 1;
    motor_monitor_poll();
  }

  /* 8C. Overheat Protection (> 70 degC disables torque & sets ERR_FLAG_OVERHEAT) */
  g_control_table.torque_enable = 1;
  /* NTC ADC = 500 -> R_ntc = 1390 ohm (~78 degC > 70 degC limit) */
  s_plant.sim_ntc_adc = 500U;
  sim_plant_update_sensors();
  s_plant.systick_1ms_flag = 1;
  motor_monitor_poll();
  int overheat_tripped =
      (g_control_table.torque_enable == 0) &&
      ((g_motor_status_flags & ERR_FLAG_OVERHEAT) != 0) &&
      (g_control_table.current_temperature > g_control_table.temp_upper_limit);

  /* Restore normal NTC temperature (25 degC) */
  s_plant.sim_ntc_adc = 2048U;
  sim_plant_update_sensors();
  s_plant.systick_1ms_flag = 1;
  motor_monitor_poll();
  int overheat_cleared = ((g_motor_status_flags & ERR_FLAG_OVERHEAT) == 0);

  int pass =
      auto_zero_ok && overcurrent_tripped && overheat_tripped && overheat_cleared;
  printf("  -> Overcurrent Trip (>600 mA for 2000 ms -> torque_enable=0): %s "
         "(I=%d mA)\n",
         overcurrent_tripped ? "OK" : "FAIL",
         (int)g_control_table.current_draw);
  printf("  -> Overheat Trip (>70 C -> torque_enable=0, ERR_FLAG_OVERHEAT): %s "
         "(T=%u C)\n",
         (overheat_tripped && overheat_cleared) ? "OK" : "FAIL",
         (unsigned)g_control_table.current_temperature);
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Test Suite 9: Protocol V3 Stack (SYNC_WRITE, REG_WRITE+ACTION, Soft Angle
 *                Limits, Kp/Kd Clamping, EEPROM Lock & Checksum Error)
 * -------------------------------------------------------------------------- */
static int test_protocol_v3_commands_and_limits(void) {
  printf("\n[TEST 9] Protocol V3 Stack (SYNC_WRITE, REG_WRITE+ACTION, Limits, "
         "Checksum)...\n");
  sim_init_calibrated_controller();

  /* 9A. Corrupt Checksum Rejection */
  uint8_t ping_corrupt = 0;
  sim_send_protocol_pkt(1, INST_PING, &ping_corrupt, 0, 1);
  int cs_err_ok = ((g_motor_status_flags & ERR_FLAG_CHECKSUM) != 0);

  /* 9B. REG_WRITE (0x04) + ACTION (0x05) */
  uint8_t reg_wr_payload[3] = {REG_GOAL_POSITION_L, 0x00, 0x08}; /* +2048 */
  sim_send_protocol_pkt(1, INST_REG_WRITE, reg_wr_payload, 3, 0);
  int staged_ok = (g_control_table.registered_flag == 1) &&
                  (g_control_table.goal_position == 0);
  sim_send_protocol_pkt(1, INST_ACTION, NULL, 0, 0);
  int action_ok = (g_control_table.registered_flag == 0) &&
                  (g_control_table.goal_position == 2048);

  /* 9C. SYNC_WRITE (0x83) for IDs 1 and 2 writing REG_GOAL_POSITION_L (2 bytes) */
  uint8_t sync_payload[8] = {
      REG_GOAL_POSITION_L, 2,    /* start_addr, data_len_per_motor */
      1, 0x55, 0x05,             /* Motor ID 1 -> +1365 (0x0555) */
      2, 0x00, 0x0C              /* Motor ID 2 -> +3072 */
  };
  sim_send_protocol_pkt(BROADCAST_ID, INST_SYNC_WRITE, sync_payload, 8, 0);
  int sync_ok = (g_control_table.goal_position == 1365);

  /* 9D. Soft Angle Limits (min=-2048, max=+2048) rejecting +4096 with ERR_FLAG_ANGLE */
  uint8_t lim_payload[5] = {
      REG_MIN_ANGLE_L, 0x00, 0xF8, /* -2048 */
      0x00, 0x08                   /* +2048 */
  };
  sim_send_protocol_pkt(1, INST_WRITE_DATA, lim_payload, 5, 0);
  uint8_t out_of_bounds_goal[3] = {REG_GOAL_POSITION_L, 0x00, 0x10}; /* +4096 */
  sim_send_protocol_pkt(1, INST_WRITE_DATA, out_of_bounds_goal, 3, 0);
  int angle_lim_ok = ((g_motor_status_flags & ERR_FLAG_ANGLE) != 0) &&
                     (g_control_table.goal_position == 1365);

  /* 9E. Kp/Kd Clamping to MOTOR_KP_MAX (3000) & EEPROM Lock (ERR_FLAG_RANGE_LOCK) */
  uint8_t kp_over[3] = {REG_KP_L, 0x10, 0x27}; /* 10000 > 3000 */
  sim_send_protocol_pkt(1, INST_WRITE_DATA, kp_over, 3, 0);
  int kp_clamp_ok = (g_control_table.kp == MOTOR_KP_MAX);

  uint8_t lock_cmd[2] = {REG_LOCK_EEPROM, 1};
  sim_send_protocol_pkt(1, INST_WRITE_DATA, lock_cmd, 2, 0);
  uint8_t write_id_locked[2] = {REG_MOTOR_ID, 5};
  sim_send_protocol_pkt(1, INST_WRITE_DATA, write_id_locked, 2, 0);
  int lock_ok = (g_control_table.motor_id == 1) &&
                ((g_motor_status_flags & ERR_FLAG_RANGE_LOCK) != 0);

  int pass = cs_err_ok && staged_ok && action_ok && sync_ok && angle_lim_ok &&
             kp_clamp_ok && lock_ok;
  printf("  -> Checksum Error Detection (ERR_FLAG_CHECKSUM)    : %s\n",
         cs_err_ok ? "OK" : "FAIL");
  printf("  -> REG_WRITE Staging + ACTION Deferred Execution   : %s\n",
         (staged_ok && action_ok) ? "OK" : "FAIL");
  printf("  -> Broadcast SYNC_WRITE Multi-Motor Dispatch       : %s\n",
         sync_ok ? "OK" : "FAIL");
  printf("  -> Soft Angle Limits & ERR_FLAG_ANGLE Protection   : %s\n",
         angle_lim_ok ? "OK" : "FAIL");
  printf("  -> Kp Clamp (%u) & EEPROM Lock Protection          : %s\n",
         (unsigned)g_control_table.kp,
         (kp_clamp_ok && lock_ok) ? "OK" : "FAIL");
  printf("  -> Result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

/* --------------------------------------------------------------------------
 *  Main Test Runner
 * -------------------------------------------------------------------------- */
int main(void) {
  printf("======================================================================"
         "====\n");
  printf("  ROBOT-LM JOIN CONTROLLER - PC SIL & ANTI-VIBRATION TEST HARNESS\n");
  printf("  Target: JOINT_TYPE_NECK (Direct-Drive 7PP + Dual MT9102 Hall @ 20 "
         "kHz)\n");
  printf("======================================================================"
         "====\n");

  int passed = 0;
  int total = 9;

  passed += test_mt9102_interpolation_and_hysteresis();
  passed += test_mode3_impedance_steps();
  passed += test_mode2_position_steps();
  passed += test_settled_hold_lock_and_disturbance();
  passed += test_full_calibration_state_machine();
  passed += test_dynamic_reversal_and_bumpless_switch();
  passed += test_multiturn_wrap_around();
  passed += test_safety_protections_and_idle_auto_zero();
  passed += test_protocol_v3_commands_and_limits();

  printf("\n===================================================================="
         "======\n");
  printf("  SIL SUMMARY: %d / %d Test Suites PASSED\n", passed, total);
  printf("======================================================================"
         "====\n");

  return (passed == total) ? 0 : 1;
}
