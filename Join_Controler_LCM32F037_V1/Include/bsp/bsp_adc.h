/**
 * @file bsp_adc.h
 * @brief [BSP] ADC1 (6 channels scanned, circular DMA into a RAM buffer)
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include "mid/foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if MOTOR_USE_HALL_MT9102
/* Index of a measurement inside the DMA result buffer (s_adc_buf[7], NECK):
 * Hardware scans backward (high channel to low channel):
 *   [0] CH12 -> OPA1 out: Phase V current
 *   [1] CH11 -> OPA0 out: Total bus current
 *   [2] CH4  -> PA4: Hall_B cos(theta)
 *   [3] CH3  -> PA3: Hall_A sin(theta)
 *   [4] CH2  -> OPA2 out: Phase U current
 *   [5] CH1  -> PA1: NTC temperature
 *   [6] CH0  -> PA0: Vbus bus voltage
 */
typedef enum {
    BSP_ADC_PHASE_V       = 0,   /* CH12 -> Phase V current (OPA1 out) */
    BSP_ADC_TOTAL_CURRENT = 1,   /* CH11 -> Total bus current (OPA0 out) */
    BSP_ADC_HALL_COS      = 2,   /* CH4  -> PA4 Hall Cos */
    BSP_ADC_HALL_SIN      = 3,   /* CH3  -> PA3 Hall Sin */
    BSP_ADC_PHASE_U       = 4,   /* CH2  -> Phase U current (OPA2 out) */
    BSP_ADC_NTC           = 5,   /* CH1  -> PA1 NTC */
    BSP_ADC_VBUS          = 6    /* CH0  -> PA0 Vbus voltage */
} bsp_adc_channel_t;
#else
/* Index of a measurement inside the DMA result buffer (original s_adc_buf[], HIP / KNEE) */
typedef enum {
    BSP_ADC_PHASE_U       = 0,   /* OPA2 -> shunt phase U            */
    BSP_ADC_PHASE_V       = 1,   /* OPA1 -> shunt phase V            */
    BSP_ADC_NTC           = 2,   /* PA1  -> NTC divider              */
    BSP_ADC_TOTAL_CURRENT = 3,   /* OPA0 -> total bus current shunt  */
    BSP_ADC_VBUS          = 4    /* PA0  -> bus voltage divider      */
} bsp_adc_channel_t;
#endif

void     bsp_adc_init(void);                       /* 0x080038C0 */
void     bsp_adc_start_conversion(void);           /* ADC_StartOfConversion(ADC1) */
uint16_t bsp_adc_read(bsp_adc_channel_t channel);  /* last raw 12-bit left-aligned sample */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
