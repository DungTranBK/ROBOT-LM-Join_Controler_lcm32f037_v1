/**
 * @file checksum.c
 * @brief [Middleware] Inverted-sum checksum (ex protocol_calc_checksum)
 */

#include "mid/checksum.h"

uint8_t checksum_calc(const uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}
