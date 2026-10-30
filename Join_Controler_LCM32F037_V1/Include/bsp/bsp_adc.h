/**
 * @file bsp_adc.h
 * @brief [BSP] ADC1 (6 channels scanned, circular DMA into a RAM buffer)
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Index of a measurement inside the DMA result buffer (original s_adc_buf[]) */
typedef enum {
    BSP_ADC_PHASE_U       = 0,   /* OPA2 -> shunt phase U            */
    BSP_ADC_PHASE_V       = 1,   /* OPA1 -> shunt phase V            */
    BSP_ADC_NTC           = 2,   /* PA1  -> NTC divider              */
    BSP_ADC_TOTAL_CURRENT = 3,   /* OPA0 -> total bus current shunt  */
    BSP_ADC_VBUS          = 4    /* PA0  -> bus voltage divider      */
} bsp_adc_channel_t;

void     bsp_adc_init(void);                       /* 0x080038C0 */
void     bsp_adc_start_conversion(void);           /* ADC_StartOfConversion(ADC1) */
uint16_t bsp_adc_read(bsp_adc_channel_t channel);  /* last raw 12-bit left-aligned sample */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
