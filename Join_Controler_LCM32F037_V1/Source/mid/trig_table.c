/**
 * @file trig_table.c
 * @brief Trigonometry ported 1:1 from code_da_doc.bin
 *          0x08002444  sin(angle)   angle in 1/10000 rad, result * 10000
 *          0x08002434  cos(angle) = sin(15707 - angle)
 *        Table of 128 half-words at flash 0x4CC0, resolution 123 (1/10000 rad) per entry.
 */

#include "mid/trig_table.h"

/* Sine table (scaled by 10,000), original flash offset 0x4CC0 */
const uint16_t g_sin_table[128] = {
        1,   122,   245,   368,   490,   613,   735,   857,
      980,  1102,  1224,  1345,  1467,  1588,  1709,  1830,
     1950,  2071,  2191,  2310,  2429,  2548,  2667,  2785,
     2902,  3020,  3136,  3253,  3368,  3484,  3598,  3713,
     3826,  3939,  4052,  4164,  4275,  4386,  4496,  4605,
     4714,  4821,  4929,  5035,  5141,  5245,  5350,  5453,
     5555,  5657,  5758,  5858,  5957,  6055,  6152,  6248,
     6344,  6438,  6531,  6624,  6715,  6806,  6895,  6983,
     7071,  7157,  7242,  7326,  7409,  7491,  7572,  7651,
     7730,  7807,  7883,  7958,  8032,  8104,  8175,  8245,
     8314,  8382,  8448,  8513,  8577,  8639,  8700,  8760,
     8819,  8876,  8932,  8986,  9039,  9091,  9142,  9191,
     9238,  9285,  9329,  9373,  9415,  9456,  9495,  9533,
     9569,  9604,  9637,  9669,  9700,  9729,  9757,  9783,
     9807,  9831,  9852,  9873,  9891,  9909,  9924,  9939,
     9951,  9963,  9972,  9981,  9987,  9993,  9996,  9999,
};

/* x / 123 for 0 <= x < 16384 : (x * 17051) >> 21 is exact over that range (Cortex-M0 has no divider) */
static inline uint32_t div123(uint32_t x) {
    return (x * 17051UL) >> 21;
}

/* 0x08002444 */
int32_t trig_sin(int32_t angle) {
    while (angle < 0) {
        angle += TRIG_TWO_PI;
    }
    while (angle >= TRIG_TWO_PI) {
        angle -= TRIG_TWO_PI;
    }

    if (angle <= TRIG_HALF_PI) {
        return (int32_t)g_sin_table[div123((uint32_t)angle)];
    }
    if ((uint32_t)(angle - TRIG_HALF_PI) <= (uint32_t)(TRIG_HALF_PI + 1)) {
        return (int32_t)g_sin_table[div123((uint32_t)(TRIG_PI - angle))];
    }
    return -trig_sin(angle - TRIG_PI);
}

/* 0x08002434 */
int32_t trig_cos(int32_t angle) {
    return trig_sin(TRIG_HALF_PI - angle);
}
