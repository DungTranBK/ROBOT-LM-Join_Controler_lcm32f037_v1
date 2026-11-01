/**
 * @file bsp_mt9102.c
 * @brief [BSP] Dual MT9102 linear Hall sensor -> rotor angle for FOC
 *
 * Hardware assumption (Neck joint board):
 *   Hall_A (sin channel) -> ADC channel BSP_ADC_HALL_SIN  (CH3, PA3)
 *   Hall_B (cos channel) -> ADC channel BSP_ADC_HALL_COS  (CH4, PA4)
 *   Both sensors powered from 3.3 V.
 */

#include "mid/foc_config.h"

#if MOTOR_USE_HALL_MT9102

#include "bsp/bsp_mt9102.h"
#include "bsp/bsp_adc.h"
#include "mid/trig_table.h"       /* TRIG_TWO_PI, TRIG_HALF_PI, trig_sin     */

extern const uint16_t g_sin_table[128];

#if DEBUG_HALL_ANGLE_LOG
#include "bsp/bsp_bus_uart.h"
#include "bsp/bsp_systick.h"
#include <stdio.h>                /* snprintf */
#endif

/* -------------------------------------------------------------------------
 *  Configuration constants
 * ---------------------------------------------------------------------- */

#define MT9102_OFFSET_SIN       1998
#define MT9102_OFFSET_COS       1986
#define MT9102_GAIN_SIN_NUM     3078
#define MT9102_GAIN_SIN_DEN     3449

typedef struct {
    int32_t off_sin;
    int32_t off_cos;
    int32_t gain_sin_num;
    int32_t gain_sin_den;
} mt9102_calib_t;

typedef struct {
    int32_t filt_sin;
    int32_t filt_cos;
    int32_t last_raw14;
    uint8_t init;
} mt9102_state_t;

typedef struct {
    volatile uint16_t snap_sin_adc;
    volatile uint16_t snap_cos_adc;
    volatile int32_t  snap_filt_sin;
    volatile int32_t  snap_filt_cos;
} mt9102_snapshot_t;

typedef struct {
    mt9102_calib_t    base;    /* Baseline parameters loaded from Flash */
    mt9102_calib_t    active;  /* Active runtime parameters */
    mt9102_state_t    state;   /* Filter and angle history */
    mt9102_snapshot_t snap;    /* ISR snapshot for debug/log */
} mt9102_dev_t;

static mt9102_dev_t s_mt9102 = {
    .base = {
        .off_sin      = MT9102_OFFSET_SIN,
        .off_cos      = MT9102_OFFSET_COS,
        .gain_sin_num = MT9102_GAIN_SIN_NUM,
        .gain_sin_den = MT9102_GAIN_SIN_DEN,
    },
    .active = {
        .off_sin      = MT9102_OFFSET_SIN,
        .off_cos      = MT9102_OFFSET_COS,
        .gain_sin_num = MT9102_GAIN_SIN_NUM,
        .gain_sin_den = MT9102_GAIN_SIN_DEN,
    },
    .state = {
        .filt_sin   = 0,
        .filt_cos   = TRIG_SCALE,
        .last_raw14 = 0,
        .init       = 0,
    },
    .snap = {0}
};

/* IIR low-pass filter state (Cartesian Q4 fixed-point).
 * HALL_IIR_SHIFT = 2 -> tau = 4 ticks (0.2 ms @ 20 kHz, fc ~ 800 Hz).
 * HALL_RAW14_HYST = 2 -> +/-2 counts_14 (+/-0.044 deg) stationary hysteresis.
 */
#define HALL_IIR_SHIFT         2
#define HALL_RAW14_HYST        2

#define HALL_ANGLE_NEGATE_SIN  0   /* 0 = match motor phase rotation direction */
#define HALL_ANGLE_NEGATE_COS  0   /* 0 = normal, 1 = mirror angle direction  */

/* -------------------------------------------------------------------------
 *  Interpolated quadrant-1 sine lookup (angle_q1 in 0..4096 -> 0..10000)
 *  Uses g_sin_table[128] with 5-bit (32x) linear interpolation between
 *  adjacent entries so raw14 has full 14-bit (0..16383) resolution instead
 *  of 32-count (0.70 deg) steps.
 * ---------------------------------------------------------------------- */
static inline int32_t hall_sin_q1(int32_t angle_q1)
{
    if (angle_q1 <= 0) {
        return 0;
    }
    if (angle_q1 >= 4096) {
        return TRIG_SCALE;
    }
    uint32_t idx  = (uint32_t)angle_q1 >> 5;   /* 0..127 */
    uint32_t frac = (uint32_t)angle_q1 & 31U;  /* 0..31  */
    int32_t s0 = (idx == 0U) ? 0 : (int32_t)g_sin_table[idx];
    int32_t s1 = (idx + 1U < 128U) ? (int32_t)g_sin_table[idx + 1U] : TRIG_SCALE;
    return s0 + (((s1 - s0) * (int32_t)frac) >> 5);
}

/* -------------------------------------------------------------------------
 *  atan2 directly returning 14-bit angle raw14 in 0..16383
 *  (no floating-point, no libm, no integer division in loop)
 * ---------------------------------------------------------------------- */
static int32_t hall_atan2_raw14(int32_t sin_n, int32_t cos_n)
{
    /* Handle degenerate cases */
    if (cos_n == 0) {
        return (sin_n >= 0) ? 4096 : 12288;
    }

    int32_t sin_abs = (sin_n < 0) ? -sin_n : sin_n;
    int32_t cos_abs = (cos_n < 0) ? -cos_n : cos_n;

    /* Binary search over [0, 4096] (12 iterations for exact 1-LSB resolution) */
    int32_t lo = 0;
    int32_t hi = 4096;
    int32_t mid;

    for (int i = 0; i < 12; i++) {
        mid = (lo + hi) >> 1;
        int32_t lhs = hall_sin_q1(mid);
        int32_t rhs = hall_sin_q1(4096 - mid);
        if ((lhs * cos_abs) < (rhs * sin_abs)) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    int32_t angle_q1 = lo; /* angle in [0, 4096] */

    /* Map to correct quadrant in 0..16383 */
    int32_t raw14;
    if (cos_n >= 0 && sin_n >= 0) {
        raw14 = angle_q1;                  /* Q1: 0 .. 4096      */
    } else if (cos_n < 0 && sin_n >= 0) {
        raw14 = 8192 - angle_q1;           /* Q2: 4096 .. 8192   */
    } else if (cos_n < 0 && sin_n < 0) {
        raw14 = 8192 + angle_q1;           /* Q3: 8192 .. 12288  */
    } else {
        raw14 = 16384 - angle_q1;          /* Q4: 12288 .. 16384 */
    }

    return raw14 & 0x3FFF;
}

/* -------------------------------------------------------------------------
 *  Public API
 * ---------------------------------------------------------------------- */

void bsp_mt9102_set_calibration(uint16_t off_sin, uint16_t off_cos, uint16_t gain_num, uint16_t gain_den)
{
    if (off_sin > 1000 && off_sin < 3000) {
        s_mt9102.base.off_sin   = (int32_t)off_sin;
        s_mt9102.active.off_sin = (int32_t)off_sin;
    }
    if (off_cos > 1000 && off_cos < 3000) {
        s_mt9102.base.off_cos   = (int32_t)off_cos;
        s_mt9102.active.off_cos = (int32_t)off_cos;
    }
    if (gain_num > 500 && gain_den > 500) {
        s_mt9102.base.gain_sin_num   = (int32_t)gain_num;
        s_mt9102.base.gain_sin_den   = (int32_t)gain_den;
        s_mt9102.active.gain_sin_num = (int32_t)gain_num;
        s_mt9102.active.gain_sin_den = (int32_t)gain_den;
    }
    s_mt9102.state.init = 0;
}

void bsp_mt9102_get_calibration(uint16_t *off_sin, uint16_t *off_cos, uint16_t *gain_num, uint16_t *gain_den)
{
    if (off_sin)  *off_sin  = (uint16_t)s_mt9102.active.off_sin;
    if (off_cos)  *off_cos  = (uint16_t)s_mt9102.active.off_cos;
    if (gain_num) *gain_num = (uint16_t)s_mt9102.active.gain_sin_num;
    if (gain_den) *gain_den = (uint16_t)s_mt9102.active.gain_sin_den;
}

void bsp_mt9102_recalibrate(void)
{
    s_mt9102.active = s_mt9102.base;

    /* Reset IIR filter */
    s_mt9102.state.filt_sin = 0;
    s_mt9102.state.filt_cos = TRIG_SCALE;
    s_mt9102.state.init     = 0;
}

void bsp_mt9102_init(void)
{
    bsp_mt9102_recalibrate();
}

int32_t bsp_mt9102_read_raw(void)
{
    /* ---- 1. Read raw ADC values, snapshot for debug log ---- */
    int32_t adc_a = (int32_t)bsp_adc_read(BSP_ADC_HALL_SIN);
    int32_t adc_b = (int32_t)bsp_adc_read(BSP_ADC_HALL_COS);
    s_mt9102.snap.snap_sin_adc = (uint16_t)adc_a;
    s_mt9102.snap.snap_cos_adc = (uint16_t)adc_b;

    /* ---- 2. Subtract DC offset ---- */
    int32_t sig_sin = adc_a - s_mt9102.active.off_sin;
    int32_t sig_cos = adc_b - s_mt9102.active.off_cos;

#if HALL_ANGLE_NEGATE_SIN
    sig_sin = -sig_sin;
#endif
#if HALL_ANGLE_NEGATE_COS
    sig_cos = -sig_cos;
#endif

    /* ---- 2b. Equalize Sin amplitude with active gain ratio ---- */
    sig_sin = (sig_sin * s_mt9102.active.gain_sin_num) / s_mt9102.active.gain_sin_den;

    /* ---- 3. Q4 fixed-point IIR low-pass filter in Cartesian space ---- */
    int32_t target_sin_q4 = sig_sin << 4;
    int32_t target_cos_q4 = sig_cos << 4;
    if (!s_mt9102.state.init) {
        s_mt9102.state.filt_sin = target_sin_q4;
        s_mt9102.state.filt_cos = target_cos_q4;
        s_mt9102.state.init     = 1;
    } else {
        s_mt9102.state.filt_sin += (target_sin_q4 - s_mt9102.state.filt_sin) >> HALL_IIR_SHIFT;
        s_mt9102.state.filt_cos += (target_cos_q4 - s_mt9102.state.filt_cos) >> HALL_IIR_SHIFT;
    }

    /* Cache filtered values for debug log */
    s_mt9102.snap.snap_filt_sin = s_mt9102.state.filt_sin >> 4;
    s_mt9102.snap.snap_filt_cos = s_mt9102.state.filt_cos >> 4;

    /* ---- 4. Interpolated atan2 on filtered values -> 14-bit raw14 (0..16383) ---- */
    int32_t raw14 = hall_atan2_raw14(s_mt9102.state.filt_sin, s_mt9102.state.filt_cos);

    /* Stationary hysteresis (+/-2 counts_14 = +/-0.044 deg) suppresses ADC LSB toggle at rest */
    int32_t diff = raw14 - s_mt9102.state.last_raw14;
    if (diff > 8192) {
        diff -= 16384;
    } else if (diff < -8192) {
        diff += 16384;
    }
    if (diff >= -HALL_RAW14_HYST && diff <= HALL_RAW14_HYST) {
        raw14 = s_mt9102.state.last_raw14;
    } else {
        s_mt9102.state.last_raw14 = raw14;
    }
    return raw14;
}

#if DEBUG_HALL_ANGLE_LOG

void bsp_mt9102_log_poll(void)
{
    static uint32_t s_last_ms = 0;

    uint32_t now = bsp_systick_get_ms();

    if ((uint32_t)(now - s_last_ms) < 250U) { return; }
    if (bsp_bus_uart_is_tx_busy())           { return; }
    s_last_ms = now;

    uint16_t adc_sin = s_mt9102.snap.snap_sin_adc;
    uint16_t adc_cos = s_mt9102.snap.snap_cos_adc;
    int32_t  raw14   = s_mt9102.state.last_raw14;

    int32_t elec_deg10 = (raw14 * 3600) / 16384;
    elec_deg10 = elec_deg10 % 3600;
    int32_t deg_int  = elec_deg10 / 10;
    int32_t deg_frac = elec_deg10 % 10;

    static char s_log_buf[64];
    int n = snprintf(s_log_buf, sizeof(s_log_buf),
                     "HALL A=%4u B=%4u raw14=%05d elec=%3d.%d deg\r\n",
                     (unsigned)adc_sin,
                     (unsigned)adc_cos,
                     (int)raw14,
                     (int)deg_int, (int)deg_frac);

    if (n > 0 && (unsigned)n < sizeof(s_log_buf)) {
        bsp_bus_uart_send_packet((const uint8_t *)s_log_buf, (uint16_t)n, 0);
    }
}

#endif /* DEBUG_HALL_ANGLE_LOG */

#endif /* MOTOR_USE_HALL_MT9102 */

