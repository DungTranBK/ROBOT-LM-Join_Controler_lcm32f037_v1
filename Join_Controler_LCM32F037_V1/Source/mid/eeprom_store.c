/**
 * @file eeprom_store.c
 * @brief [Middleware] Flash EEPROM emulation, ported 1:1 from code_da_doc.bin
 *          0x080041C8  load  (scan 2 pages x 4 blocks, newest valid block wins)
 *          0x080048F8  save  (round-robin, page erase when the page is full)
 *
 *        Flash access goes through the BSP layer (bsp_flash.h).
 */

#include <string.h>

#include "mid/foc_config.h"
#if MOTOR_USE_HALL_MT9102
#include "lcm32f037.h"
#endif
#include "bsp/bsp_flash.h"

#include "mid/eeprom_store.h"
#include "mid/control_table.h"
#include "mid/checksum.h"

static const uint32_t s_page_addr[EEPROM_PAGE_COUNT] = {            /* table @0x08004CB8 */
    EEPROM_FLASH_PAGE0_ADDR, EEPROM_FLASH_PAGE1_ADDR
};

static uint32_t s_counter  = 0;   /* 0x20000090: newest block sequence number    */
static uint16_t s_blk_off  = 0;   /* 0x2000008C: byte offset of newest block     */
static uint8_t  s_page_idx = 0;   /* 0x2000008A: page of newest block            */

/* 0x080048F8 */
void eeprom_store_save(void) {
    const uint8_t *tbl = (const uint8_t *)&g_control_table;
    uint8_t cs;
    uint8_t i;

    s_counter++;
    g_control_table.save_counter = s_counter;
    g_control_table.reboot_copy  = g_reboot_flag;
    g_control_table.block_marker = EEPROM_BLOCK_MARKER;

#if MOTOR_USE_HALL_MT9102
    __disable_irq();
#endif
    bsp_flash_unlock();

    for (;;) {
        bsp_flash_clear_status();

        s_blk_off += EEPROM_BLOCK_SIZE;
        if ((s_blk_off & EEPROM_PAGE_OFFSET_MASK) == 0) {
            s_blk_off  = 0;
            s_page_idx = (s_page_idx == 0) ? 1 : 0;
            bsp_flash_erase_page(s_page_addr[s_page_idx]);
        }

        for (i = 0; i < EEPROM_BLOCK_DATA_LEN; i += 2) {
            uint16_t hw = (uint16_t)(tbl[i] | (tbl[i + 1] << 8));
            if (!bsp_flash_write_halfword(s_page_addr[s_page_idx] + s_blk_off + i, hw)) {
                break;
            }
        }
        if (i < EEPROM_BLOCK_DATA_LEN) {
            continue;                       /* programming failed -> next block */
        }

        cs = checksum_calc((const uint8_t *)(s_page_addr[s_page_idx] + s_blk_off), EEPROM_BLOCK_DATA_LEN);
        g_control_table.block_checksum = cs;
        if (bsp_flash_write_halfword(s_page_addr[s_page_idx] + s_blk_off + EEPROM_BLOCK_CS_OFFSET,
                                     (uint16_t)((EEPROM_BLOCK_MARKER << 8) | cs))) {
            break;
        }
    }

    bsp_flash_lock();
#if MOTOR_USE_HALL_MT9102
    __enable_irq();
#endif
}

/* 0x080041C8 */
void eeprom_store_init(void) {
    uint8_t  p, b;
    uint8_t  found = 0;

    for (p = 0; p < EEPROM_PAGE_COUNT; p++) {
        for (b = 0; b < EEPROM_BLOCKS_PER_PAGE; b++) {
            const uint8_t *blk = (const uint8_t *)(s_page_addr[p] + (uint32_t)b * EEPROM_BLOCK_SIZE);

            if (blk[EEPROM_BLOCK_MARKER_OFFSET] != EEPROM_BLOCK_MARKER) continue;
            if (checksum_calc(blk, EEPROM_BLOCK_DATA_LEN) != blk[EEPROM_BLOCK_CS_OFFSET]) continue;
            {
                uint32_t counter = *(const uint32_t *)(blk + EEPROM_BLOCK_COUNTER_OFFSET);
                if ((int32_t)counter < (int32_t)s_counter) continue;
                s_counter  = counter;
                s_blk_off  = (uint16_t)(b * EEPROM_BLOCK_SIZE);
                s_page_idx = p;
                found      = 1;
            }
        }
    }

    if (found) {
        memcpy(&g_control_table, (const void *)(s_page_addr[s_page_idx] + s_blk_off), EEPROM_BLOCK_SIZE);
        if (g_control_table.calib_done == 0 &&
            (g_control_table.offset_v != 0 || g_control_table.offset_u != 0)) {
            g_control_table.calib_done = 1;
        }
    } else {
        control_table_load_eeprom_defaults();
        eeprom_store_save();
    }
    control_table_load_ram_defaults();

    g_last_id = g_control_table.motor_id;
    g_control_table.hw_tag = EEPROM_HW_TAG_VALUE;   /* original copies the byte at EEPROM_HW_TAG_SRC_ADDR */
}

/* 0x080042F4 : RESET instruction */
void eeprom_store_restore_defaults(void) {
    control_table_load_eeprom_defaults();
    control_table_load_ram_defaults();
    g_eeprom_dirty_flag = 1;
}
