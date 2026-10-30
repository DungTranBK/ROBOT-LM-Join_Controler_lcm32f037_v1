/**
 * @file bsp_mt6701.h
 * @brief [BSP] Absolute magnetic angle sensor (MT6701) read over SPI0
 */

#ifndef __BSP_MT6701_H
#define __BSP_MT6701_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    bsp_mt6701_init(void);         /* 0x080046B8 : SPI0 master, 16 bit */
int32_t bsp_mt6701_read_raw(void);     /* 0x08004158 : 14-bit angle, 0..16383 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MT6701_H */
