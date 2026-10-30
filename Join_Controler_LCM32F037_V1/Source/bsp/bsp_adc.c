/**
 * @file bsp_adc.c
 * @brief [BSP] ADC1 + DMA driver, ported 1:1 from code_da_doc.bin 0x080038C0
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_adc.h"
#include "lcm32f037_dma.h"

#include "bsp/bsp_adc.h"

static volatile uint16_t s_adc_buf[8] __attribute__((aligned(4)));  /* 0x20000404: CH0,1,2,10,11,12 */

/* 0x080038C0 : ADC (6 channels, scan, DMA circular -> s_adc_buf) */
void bsp_adc_init(void) {
    ADC_InitTypeDef adc;
    DMA_InitTypeDef dma;
    /* Original SDK build never writes ADC->TRIM for Vref 4 (golden TRIM = 0x0064008A);
     * the vendor copy loads it from foundry info 0x1FFFF5A8, so keep the reset value. */
    uint32_t adc_trim = ADC1->TRIM;

    ADC_VrefintSelect(ADC1, 4);

    adc.ADC_ClkMode              = (1UL << 27);
    adc.ADC_Resolution           = 0;
    adc.ADC_ContinuousConvMode   = DISABLE;
    adc.ADC_AutoWatiMode         = ENABLE;
    adc.ADC_DMA                  = ENABLE;
    adc.ADC_DiscMode             = DISABLE;
    adc.ADC_SamecMode            = ENABLE;
    adc.ADC_ExternalTrigConvEdge = 0;
    adc.ADC_ExternalTrigConv     = 0;
    adc.ADC_DataAlign            = 0;
    adc.ADC_DMATRIG_LEVEL        = 0;
    adc.ADC_ScanDirection        = 4;
    adc.ADC_Vref                 = 4;
    ADC_Init(ADC1, &adc);
    ADC1->TRIM = adc_trim;

    ADC_ChannelConfig(ADC1, 1UL << 2,  7, 1, 1);   /* CH2  */
    ADC_ChannelConfig(ADC1, 1UL << 1,  7, 1, 1);   /* CH1  */
    ADC_ChannelConfig(ADC1, 1UL << 0,  7, 1, 1);   /* CH0  */
    ADC_ChannelConfig(ADC1, 1UL << 12, 7, 1, 1);   /* CH12 */
    ADC_ChannelConfig(ADC1, 1UL << 11, 7, 1, 1);   /* CH11 */
    ADC_ChannelConfig(ADC1, 1UL << 10, 7, 1, 1);   /* CH10 */
    /* The original SDK build writes (AdjustTime << 4); this vendor copy hard-codes 7 << 4.
     * Golden dump: ADC_SMPR = 0x391 (convert 7, adjust 1, sample 1). */
    ADC1->SMPR = (ADC1->SMPR & ~0x70UL) | (1UL << 4);
    ADC_Cmd(ADC1, ENABLE);

    DMA_DeInit(DMA1_Channel2);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_adc_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = 6;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_M2M                = DMA_P2M_Enable;
    dma.DMA_MSIZE              = DMA_MSIZE_1;
    DMA_Init(DMA1_Channel2, &dma);
    DMA_RemapConfig(DMA1_Channel2, DMA_ReqNum_4, DMA_REQ_ADC);
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

void bsp_adc_start_conversion(void) {
    ADC_StartOfConversion(ADC1);
}

uint16_t bsp_adc_read(bsp_adc_channel_t channel) {
    return s_adc_buf[channel];
}
