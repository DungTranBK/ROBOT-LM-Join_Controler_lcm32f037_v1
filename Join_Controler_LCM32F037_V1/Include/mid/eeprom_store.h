/**
 * @file eeprom_store.h
 * @brief [Middleware] Flash EEPROM emulation (settings + calibration) in the last 1 KB of the 32 KB flash.
 *        Matching code_da_doc.bin: load 0x080041C8, defaults 0x08004740/0x08004788,
 *        save 0x080048F8.
 *
 *        Layout: 2 pages (0x08007C00, 0x08007E00) x 4 blocks of 128 bytes.
 *        Block = control table image, +0x78 sequence counter, +0x7E ~sum(0..0x7D), +0x7F = 0x4B.
 */

#ifndef __EEPROM_STORE_H
#define __EEPROM_STORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EEPROM_FLASH_PAGE0_ADDR       0x08007C00UL
#define EEPROM_FLASH_PAGE1_ADDR       0x08007E00UL
#define EEPROM_PAGE_COUNT             2
#define EEPROM_BLOCKS_PER_PAGE        4
#define EEPROM_PAGE_SIZE              0x200UL   /* = EEPROM_BLOCKS_PER_PAGE * EEPROM_BLOCK_SIZE */
#define EEPROM_PAGE_OFFSET_MASK       (EEPROM_PAGE_SIZE - 1)

/* Block layout (byte offsets inside one 128-byte block) */
#define EEPROM_BLOCK_COUNTER_OFFSET   0x78      /* uint32 sequence number        */
#define EEPROM_BLOCK_CS_OFFSET        0x7E      /* checksum byte                 */
#define EEPROM_BLOCK_MARKER_OFFSET    0x7F      /* marker byte (EEPROM_BLOCK_MARKER) */
#define EEPROM_BLOCK_DATA_LEN         EEPROM_BLOCK_CS_OFFSET /* bytes covered by checksum */

/* Hardware tag copied from the original firmware's flash */
#define EEPROM_HW_TAG_VALUE           0x43
#define EEPROM_HW_TAG_SRC_ADDR        0x08002000UL

/** Load the newest valid block into g_control_table (or write factory defaults). 0x080041C8 */
void eeprom_store_init(void);
/** Persist g_control_table into the next free block. 0x080048F8 */
void eeprom_store_save(void);
/** Factory defaults (RESET instruction) and schedule a save. 0x080042F4 */
void eeprom_store_restore_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_STORE_H */
