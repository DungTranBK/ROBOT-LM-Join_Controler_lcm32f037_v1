/**
 * @file board_pins.h
 * @brief [BSP] EC4012 board pin map: port / pin number / alternate function per signal.
 *        Single place to change when the board is re-routed (see docs/peripheral_pinout.md).
 *
 *        PIN_<SIGNAL>_PORT : bsp_gpio_port_t
 *        PIN_<SIGNAL>_NUM  : pin number inside the port (0..15)
 *        PIN_<SIGNAL>_AF   : alternate function number written to AFRL/AFRH
 */

#ifndef __BOARD_PINS_H
#define __BOARD_PINS_H

#include "bsp/bsp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Bit mask of a pin inside its port. */
#define BOARD_PIN_MASK(sig)   (1UL << PIN_##sig##_NUM)

/* ---- TIM1 PWM, 3 half bridges (high side = CHx, low side = CHxN) ---- */
#define PIN_PWM_U_H_PORT      BSP_GPIO_PORT_B
#define PIN_PWM_U_H_NUM       4U
#define PIN_PWM_U_H_AF        2U      /* TIM1_CH1  */
#define PIN_PWM_U_L_PORT      BSP_GPIO_PORT_B
#define PIN_PWM_U_L_NUM       3U
#define PIN_PWM_U_L_AF        1U      /* TIM1_CH1N */
#define PIN_PWM_V_H_PORT      BSP_GPIO_PORT_B
#define PIN_PWM_V_H_NUM       6U
#define PIN_PWM_V_H_AF        4U      /* TIM1_CH2  */
#define PIN_PWM_V_L_PORT      BSP_GPIO_PORT_F
#define PIN_PWM_V_L_NUM       4U
#define PIN_PWM_V_L_AF        2U      /* TIM1_CH2N */
#define PIN_PWM_W_H_PORT      BSP_GPIO_PORT_B
#define PIN_PWM_W_H_NUM       7U
#define PIN_PWM_W_H_AF        5U      /* TIM1_CH3  */
#define PIN_PWM_W_L_PORT      BSP_GPIO_PORT_F
#define PIN_PWM_W_L_NUM       3U
#define PIN_PWM_W_L_AF        2U      /* TIM1_CH3N */

/* ---- SPI0: MT6701 angle sensor (HIP / KNEE) ---- */
#define PIN_ENC_NSS_PORT      BSP_GPIO_PORT_A
#define PIN_ENC_NSS_NUM       3U
#define PIN_ENC_NSS_AF        7U      /* SPI0_NSS  */
#define PIN_ENC_SCK_PORT      BSP_GPIO_PORT_A
#define PIN_ENC_SCK_NUM       4U
#define PIN_ENC_SCK_AF        0U      /* SPI0_SCK  */
#define PIN_ENC_MISO_PORT     BSP_GPIO_PORT_A
#define PIN_ENC_MISO_NUM      5U
#define PIN_ENC_MISO_AF       0U      /* SPI0_MISO */

/* ---- MT9102 dual linear Hall sensors (analog ADC inputs, NECK) ---- */
#define PIN_HALL_SIN_PORT     BSP_GPIO_PORT_A
#define PIN_HALL_SIN_NUM      3U
#define PIN_HALL_SIN_AF       1U      /* ADCIN[3]: Hall_A analog output (sin) */
#define PIN_HALL_COS_PORT     BSP_GPIO_PORT_A
#define PIN_HALL_COS_NUM      4U
#define PIN_HALL_COS_AF       2U      /* ADCIN[4]: Hall_B analog output (cos) */

/* ---- UART0 one-wire bus: AF7 = listen (RX), AF2 = transmit (TX) ---- */
#define PIN_BUS_PORT          BSP_GPIO_PORT_A
#define PIN_BUS_NUM           9U
#define PIN_BUS_AF_RX         7U
#define PIN_BUS_AF_TX         2U
#define PIN_BUS_AF            PIN_BUS_AF_RX   /* idle state */

/* ---- ADC inputs ---- */
#define PIN_VBUS_PORT         BSP_GPIO_PORT_A
#define PIN_VBUS_NUM          0U
#define PIN_VBUS_AF           1U      /* ADCIN[0]: bus voltage divider */
#define PIN_NTC_PORT          BSP_GPIO_PORT_A
#define PIN_NTC_NUM           1U
#define PIN_NTC_AF            1U      /* ADCIN[1]: NTC divider         */

/* ---- Op-amp inputs (shunt sensing), named like the datasheet ---- */
#define PIN_OP1P0_PORT        BSP_GPIO_PORT_A   /* OPA1 (+) / OPA_NIN reference */
#define PIN_OP1P0_NUM         6U
#define PIN_OP1P0_AF          2U
#define PIN_OP2N1_PORT        BSP_GPIO_PORT_A   /* OPA2 (-) */
#define PIN_OP2N1_NUM         7U
#define PIN_OP2N1_AF          1U
#define PIN_OP2P0_PORT        BSP_GPIO_PORT_B   /* OPA2 (+) */
#define PIN_OP2P0_NUM         0U
#define PIN_OP2P0_AF          2U
#define PIN_OP1N0_PORT        BSP_GPIO_PORT_B   /* OPA1 (-) */
#define PIN_OP1N0_NUM         1U
#define PIN_OP1N0_AF          1U
#define PIN_OP0N3_PORT        BSP_GPIO_PORT_F   /* OPA0 (-): total current shunt */
#define PIN_OP0N3_NUM         6U
#define PIN_OP0N3_AF          2U
#define PIN_OP0P3_PORT        BSP_GPIO_PORT_F   /* OPA0 (+): total current shunt */
#define PIN_OP0P3_NUM         7U
#define PIN_OP0P3_AF          2U

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_PINS_H */
