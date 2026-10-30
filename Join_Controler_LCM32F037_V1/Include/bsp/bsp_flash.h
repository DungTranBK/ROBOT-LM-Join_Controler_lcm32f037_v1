/**
 * @file bsp_flash.h
 * @brief [BSP] On-chip flash programming primitives (used by the EEPROM emulation service)
 */

#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_flash_unlock(void);
void bsp_flash_lock(void);
void bsp_flash_clear_status(void);                              /* FLASH_ClearFlag(0x34) */
void bsp_flash_erase_page(uint32_t page_addr);
bool bsp_flash_write_halfword(uint32_t addr, uint16_t data);    /* true = FLASH_COMPLETE */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FLASH_H */
