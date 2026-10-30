/**
 * @file foc_math.c
 * @brief [Middleware] Clarke/Park and inverse Park + SVPWM, ported 1:1 from code_da_doc.bin
 *          0x08003B64 clarke_park()    0x08003C04 inv_park_svpwm()
 *          0x08003B24 imax()           0x08003B44 imin()
 *        No hardware access: pure math.
 */

#include "mid/foc_math.h"
#include "mid/foc_config.h"

static int32_t imax(int32_t a, int32_t b) { return (a > b) ? a : b; }   /* 0x08003B24 */
static int32_t imin(int32_t a, int32_t b) { return (a < b) ? a : b; }   /* 0x08003B44 */

/* 0x08003B64 */
void foc_clarke_park(int32_t c, int32_t s, int32_t iw, int32_t iu, int32_t iv,
                     int32_t *id, int32_t *iq) {

    int32_t t1   = (c * iw) / 10000;
    int32_t s_iu = s * iu;
    int32_t s_iv = s * iv;
    int32_t x    = t1 + (s_iu - s_iv) / 11547;
    int32_t c_iv = c * iv;
    int32_t c_iu = c * iu;
    int32_t t3   = (c_iv + c_iu) / 20000;

    *id = ((x - t3) * 2) / 3;

    int32_t y  = ((-s) * iw) / 10000 + (c_iu - c_iv) / 11547;
    int32_t q3 = (s_iv + s_iu) / 20000;

    *iq = ((y + q3) * 2) / 3;
}

/* 0x08003C04 : inverse Park + min/max (SVPWM) centring, duties clamped to [0x46, 0x4AF] */
void foc_inv_park_svpwm(int32_t c, int32_t s, int32_t vd, int32_t vq, int32_t *out) {

    int32_t a_num = c * vd - s * vq;
    int32_t alpha = a_num / 10000;
    int32_t b_num = s * vd + c * vq;
    int32_t b_div = b_num / 11547;
    int32_t a_div = a_num / 20000;

    int32_t vb = b_div - a_div;
    int32_t vc = (-b_num) / 11547 - a_div;

    int32_t vmin = imin(imin(alpha, vb), vc);
    int32_t vmax = imax(imax(alpha, vb), vc);
    int32_t mid  = (vmin + vmax) / 2;

    out[0] = imin(imax(alpha - mid + FOC_PWM_DUTY_MID, FOC_PWM_DUTY_MIN), FOC_PWM_ARR);
    out[1] = imin(imax(vb    - mid + FOC_PWM_DUTY_MID, FOC_PWM_DUTY_MIN), FOC_PWM_ARR);
    out[2] = imin(imax(vc    - mid + FOC_PWM_DUTY_MID, FOC_PWM_DUTY_MIN), FOC_PWM_ARR);
}
