/**
 * @file motor_monitor.c
 * @brief [Middleware] 1 ms housekeeping (temperature, bus voltage, total
 * current, protections) Ported 1:1 from code_da_doc.bin 0x08004370
 * (motor_foc_periodic_1ms).
 */

#include "bsp/bsp_adc.h"
#include "bsp/bsp_systick.h"

#include "mid/control_table.h"
#include "mid/foc_config.h"
#include "mid/motor_ctrl.h"
#include "mid/motor_monitor.h"
#include "mid/ntc_table.h"

#if DEBUG_UART_MONITOR
#include "bsp/bsp_bus_uart.h"
#include <stdio.h> /* snprintf */
#endif

static int32_t s_cur_iir;       /* 0x200000AC */
static uint16_t s_cur_over_cnt; /* 0x200000A8 */

#if DEBUG_UART_MONITOR
static uint16_t s_dbg_ms_cnt = 0; /* counts 1 ms ticks, resets at 500 */
#endif

/* Bus voltage: ADC[4] (CH11) * scale - offset -> 0.1 V units */
#if MOTOR_USE_HALL_MT9102
#define VBUS_ADC_SCALE 0.134309995919466f
#else
#define VBUS_ADC_SCALE 0.10744799673557281f
#endif
#define VBUS_ADC_OFFSET 5U

/* NTC divider: R = x * R_PULLUP / (ADC_FULL - x), x = ADC[2] (CH2) */
#define NTC_R_PULLUP 10000.0f
#if MOTOR_USE_HALL_MT9102
#define NTC_ADC_FULL_SCALE 4095.0f
#else
#define NTC_ADC_FULL_SCALE 5119.0f
#endif
#define NTC_TEMP_OFFSET 40 /* table entry i = (i - 40) degC               */
#define NTC_TEMP_NOT_FOUND                                                     \
  (-100) /* sentinel while scanning the table           */
#define NTC_TEMP_MIN (-NTC_TEMP_OFFSET)
#define NTC_TEMP_MAX (NTC_TABLE_SIZE - 1 - NTC_TEMP_OFFSET)

/* Protection limits */
#define OVERHEAT_LIMIT_DEGC                                                    \
  DEFAULT_TEMP_UPPER_LIMIT /* 70 degC                        */
#if MOTOR_USE_HALL_MT9102
#define CURRENT_SCALE_MA 4.070003628730774f /* ADC counts -> mA (5.0V Vref) */
#define OVERCURRENT_LIMIT_MA 600            /* 1000 mA (1.0 A)                */
#else
#define CURRENT_SCALE_MA 3.256002902984619f /* ADC counts -> mA */
#define OVERCURRENT_LIMIT_MA 0x5DC          /* 1500 mA                        */
#endif
#define OVERCURRENT_TIME_MS 0x7D0 /* 2000 ms above the limit        */

void motor_monitor_poll(void) {
  if (bsp_systick_take_1ms_flag()) {
    uint16_t ntc_r;
    int16_t t, i;
    float f;
    int32_t cur;

    /* bus voltage: ADC[4] (CH11) * 0.107447 - 5  -> 0.1 V */
    g_control_table.current_voltage =
        (uint8_t)((uint32_t)((float)bsp_adc_read(BSP_ADC_VBUS) *
                             VBUS_ADC_SCALE) -
                  VBUS_ADC_OFFSET);

    /* NTC: R = x * 10000 / (5119 - x), x = ADC[2] (CH2) */
    f = (float)bsp_adc_read(BSP_ADC_NTC);
    ntc_r = (uint16_t)(uint32_t)((f * NTC_R_PULLUP) / (NTC_ADC_FULL_SCALE - f));

    t = NTC_TEMP_NOT_FOUND;
    for (i = 0; i < NTC_TABLE_SIZE - 1; i++) {
      if (g_ntc_table[i] >= ntc_r && g_ntc_table[i + 1] < ntc_r) {
        t = (int16_t)(i - NTC_TEMP_OFFSET);
      }
    }
    if (t == NTC_TEMP_NOT_FOUND) {
      if (g_ntc_table[0] < ntc_r) {
        t = NTC_TEMP_MIN;
      }
      if (g_ntc_table[NTC_TABLE_SIZE - 1] > ntc_r) {
        t = NTC_TEMP_MAX;
      }
    }
    if (t < 0) {
      t = 0;
    }
    g_control_table.current_temperature = (uint8_t)t;

    if ((uint8_t)t > g_control_table.temp_upper_limit) {
      g_control_table.torque_enable = 0;
      g_motor_status_flags |= ERR_FLAG_OVERHEAT;
    } else {
      g_motor_status_flags &= (uint8_t)~ERR_FLAG_OVERHEAT;
    }

    /* total current (mA), IIR with 1/32 */
    cur = (int32_t)((float)((int32_t)bsp_adc_read(BSP_ADC_TOTAL_CURRENT) -
                            g_control_table.offset_total) *
                    CURRENT_SCALE_MA);
    s_cur_iir = cur + (s_cur_iir - (s_cur_iir >> 5));
    g_control_table.current_draw = (int16_t)(s_cur_iir >> 5);

    if ((int16_t)(s_cur_iir >> 5) > OVERCURRENT_LIMIT_MA) {
      s_cur_over_cnt++;
      if (s_cur_over_cnt > OVERCURRENT_TIME_MS) {
        g_control_table.torque_enable = 0;
        s_cur_over_cnt = 0;
      }
    } else {
      s_cur_over_cnt = 0;
    }
  }

#if DEBUG_UART_MONITOR
  /* --- 500 ms debug line: fire once every 500 x 1 ms ticks --- */
  s_dbg_ms_cnt++;
  if (s_dbg_ms_cnt >= 500U) {
    s_dbg_ms_cnt = 0;
    if (!bsp_bus_uart_is_tx_busy()) {
      static char s_dbg_buf[64];
      int n = snprintf(s_dbg_buf, sizeof(s_dbg_buf),
                       "DBG pos=%6d spd=%5d cur=%5d V=%3u T=%2u fl=%02X\r\n",
                       (int)g_control_table.current_position,
                       (int)g_control_table.current_speed,
                       (int)g_control_table.current_draw,
                       (unsigned)g_control_table.current_voltage,
                       (unsigned)g_control_table.current_temperature,
                       (unsigned)g_motor_status_flags);
      if (n > 0 && (unsigned)n < sizeof(s_dbg_buf)) {
        bsp_bus_uart_send_packet((const uint8_t *)s_dbg_buf, (uint16_t)n, 0);
      }
    }
  }
#endif /* DEBUG_UART_MONITOR */

  g_control_table.current_position = motor_ctrl_get_position();
  g_control_table.current_speed = (int16_t)motor_ctrl_get_speed();
}
