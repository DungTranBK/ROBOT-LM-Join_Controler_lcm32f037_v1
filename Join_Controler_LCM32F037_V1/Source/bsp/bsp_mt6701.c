/**
 * @file bsp_mt6701.c
 * @brief [BSP] MT6701 angle sensor over SPI0, ported 1:1 from code_da_doc.bin
 *          0x080046B8 SPI init    0x08004158 sensor read
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_ssp.h"

#include "bsp/bsp_mt6701.h"

/* 0x080046B8 : SPI0 (16-bit, mode 3 like) for the MT6701 angle sensor */
void bsp_mt6701_init(void) {
    SPI_InitTypeDef spi;

    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = 0x000F;
    spi.SPI_CPOL              = 0;
    spi.SPI_CPHA              = 0x0080;
    spi.SPI_BaudRatePrescaler = 0x0300;
    spi.SPI_FirstBit          = 0;
    spi.SPI_SlaveTx           = 0;
    SPI_Init(SPI0, &spi);
    SPI_Cmd(SPI0, ENABLE);
}

int32_t bsp_mt6701_read_raw(void) {                                     /* 0x08004158 */
    SPI_SendData16(SPI0, 0xFFFF);
    while (SPI_GetFlagStatus(SPI0, SPI_FLAG_RNE) == RESET) { }
    return (int32_t)(SPI_ReceiveData16(SPI0) >> 2);
}
