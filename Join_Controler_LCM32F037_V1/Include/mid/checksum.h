/**
 * @file checksum.h
 * @brief [Middleware] Inverted-sum checksum shared by the bus protocol and the EEPROM blocks
 */

#ifndef __CHECKSUM_H
#define __CHECKSUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @return ~(sum of the len bytes at buf), truncated to 8 bit */
uint8_t checksum_calc(const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CHECKSUM_H */
