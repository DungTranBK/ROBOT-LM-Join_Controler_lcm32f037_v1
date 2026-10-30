/**
 * @file main.c
 * @brief [Application layer] Main entry and cooperative main loop for the
 * LCM32F037 BLDC motor controller Reconstructed directly from code_da_doc.bin
 * 0x08003E68
 *
 *        Layering (a layer only calls downwards):
 *          app        main.c, app_isr.c              <- this file
 *          middleware motor_ctrl, motor_monitor, protocol, eeprom_store,
 * control_table bsp
 * bsp_clock/gpio/adc/opa/pwm/encoder/bus_uart/flash/nvic/systick/system system
 * startup + LM_SystemInit (core PLL), vendor SDK (SDK/)
 */

#include "bsp/bsp_adc.h"
#include "bsp/bsp_bus_uart.h"
#include "bsp/bsp_clock.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_mt6701.h"
#include "bsp/bsp_nvic.h"
#include "bsp/bsp_opa.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_system.h"
#include "bsp/bsp_systick.h"

#include "mid/control_table.h"
#include "mid/eeprom_store.h"
#include "mid/foc_config.h"
#include "mid/motor_ctrl.h"
#include "mid/motor_monitor.h"
#include "mid/protocol.h"

/**
 * @note  Interrupt handlers:
 *          SysTick_Handler                bsp_systick.c   (1 ms time base)
 *          DMAC_CH1_2_Handler, TIM2_Handler  bsp_bus_uart.c  (UART TX complete,
 * return delay) TIM1_CC_Handler                app_isr.c       (20 kHz FOC
 * loop) UART0 RX has no interrupt: it is filled by DMAC CH0 and polled in
 * protocol_poll() (matches code_da_doc.bin vector table).
 */

/* ---------------- Main Function ---------------- */

/**
 * @brief Main Entry Point
 *        Matching disasm 0x08003E68
 */
int main(void) {
  /* 1. Hardware & driver initialisation, same order as 0x08003E68 */
  bsp_clock_init();    /* 0x08004128 */
  eeprom_store_init(); /* 0x080041C8 : settings + calibration from flash */
  motor_ctrl_load_calibration(); /* 0x08004240..0x08004270 : table -> FOC
                                    offsets */
  bsp_gpio_init();               /* 0x08003D04 */
  bsp_adc_init();                /* 0x080038C0 */
  bsp_opa_init();                /* 0x08003FBC */
  bsp_pwm_init(FOC_PWM_ARR, FOC_PWM_DEAD_TIME,
               FOC_PWM_ADC_TRIG);  /* 0x080047BC */
  bsp_bus_uart_delay_timer_init(); /* 0x08004894 : TIM2 return-delay timer */
  bsp_mt6701_init();               /* 0x080046B8 */
  bsp_bus_uart_init(
      g_control_table.baud_rate); /* 0x080045FC / 0x080039F4 / 0x08004328 */

  /* 2. SysTick 1 ms: SystemCoreClock = 96 MHz, LOAD = 0x000176FF (disasm
   * 0x08003E94) */
  bsp_systick_init_1ms();

  /* 3. Interrupt priorities + enable (0x08003F10) */
  bsp_nvic_init();

#if FEATURE_UART_STREAM_TELEMETRY
  uint32_t s_last_stream_ms = 0;
#endif

  /* 4. Application Main Loop (disasm 0x08003EBA) */
  while (1) {
    /* Apply a pending baud-rate change once the reply has left the wire */
    bsp_bus_uart_service();

    /* Task A: Process incoming UART data from the DMA ring buffer */
    protocol_poll();

    /* Task B: 1ms periodic monitoring & safety task (disasm 0x08004370) */
    motor_monitor_poll();

    /* Task C: reboot request (instruction 0x64, disasm 0x08003ED6): save, wait
     * TX, reset */
    if (g_reboot_flag == REBOOT_FLAG_REQUEST) {
      eeprom_store_save();

      while (bsp_bus_uart_is_tx_busy())
        ;

      bsp_system_reset();
    }

    /* Task D: save settings when changed (disasm 0x08003EC8) */
    if (g_eeprom_dirty_flag == 1) {
      g_eeprom_dirty_flag = 0;
      eeprom_store_save();
    }

#if FEATURE_UART_STREAM_TELEMETRY
    /* Task E: Real-time telemetry streaming if enabled */
    if (protocol_get_stream_interval() > 0) {
      uint32_t now = bsp_systick_get_ms();
      if ((uint32_t)(now - s_last_stream_ms) >= (uint32_t)protocol_get_stream_interval()) {
        if (!bsp_bus_uart_is_tx_busy()) {
          s_last_stream_ms = now;
          protocol_send_stream_packet();
        }
      }
    }
#endif
  }
}
