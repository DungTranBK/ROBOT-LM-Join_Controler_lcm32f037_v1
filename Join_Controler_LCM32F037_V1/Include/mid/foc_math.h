/**
 * @file foc_math.h
 * @brief [Middleware] Hardware independent FOC transforms (fixed point, sin/cos scaled by 10000)
 */

#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Clarke + Park: phase currents -> (id, iq). c = cos, s = sin of the electrical angle (x10000). 0x08003B64 */
void foc_clarke_park(int32_t c, int32_t s, int32_t iw, int32_t iu, int32_t iv,
                     int32_t *id, int32_t *iq);

/** Inverse Park + min/max (SVPWM) centring, duties clamped to [FOC_PWM_DUTY_MIN, FOC_PWM_ARR].
 *  out[0..2] = duty of phases U, V, W. 0x08003C04 */
void foc_inv_park_svpwm(int32_t c, int32_t s, int32_t vd, int32_t vq, int32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MATH_H */
