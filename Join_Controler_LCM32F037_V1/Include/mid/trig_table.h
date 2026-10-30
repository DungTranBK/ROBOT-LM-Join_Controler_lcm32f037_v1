/**
 * @file trig_table.h
 * @brief Trigonometric Table (Sin/Cos) extracted directly from code_da_doc.bin (0x08004CC0)
 */

#ifndef __TRIG_TABLE_H
#define __TRIG_TABLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRIG_TWO_PI         62831   /* 2 * PI * 10000 (0x0000F56F) */
#define TRIG_PI             31415   /* PI * 10000     (0x00007AB7) */
#define TRIG_HALF_PI        15707   /* PI/2 * 10000   (0x00003D5B) */
#define TRIG_SCALE          10000   /* 1.0 = 10000    (0x00002710) */
#define TRIG_SQRT3_2        11547   /* (2 / sqrt(3)) * 10000 (0x00002D1B) */

extern const int16_t g_cos_table[128];

int32_t trig_cos(int32_t angle);
int32_t trig_sin(int32_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __TRIG_TABLE_H */
