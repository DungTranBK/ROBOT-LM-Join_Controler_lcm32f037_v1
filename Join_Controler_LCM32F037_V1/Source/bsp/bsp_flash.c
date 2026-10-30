/**
 * @file bsp_flash.c
 * @brief [BSP] Thin wrapper around the vendor FLASH driver
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_flash.h"

#include "bsp/bsp_flash.h"

void bsp_flash_unlock(void) {
    FLASH_Unlock();
}

void bsp_flash_lock(void) {
    FLASH_Lock();
}

/* Status flag mask cleared before every program/erase (value taken from the original binary) */
#define BSP_FLASH_STATUS_FLAGS    0x34

void bsp_flash_clear_status(void) {
    FLASH_ClearFlag(BSP_FLASH_STATUS_FLAGS);
}

void bsp_flash_erase_page(uint32_t page_addr) {
    FLASH_ErasePage(page_addr);
}

bool bsp_flash_write_halfword(uint32_t addr, uint16_t data) {
    return FLASH_ProgramHalfWord(addr, data) == FLASH_COMPLETE;
}
