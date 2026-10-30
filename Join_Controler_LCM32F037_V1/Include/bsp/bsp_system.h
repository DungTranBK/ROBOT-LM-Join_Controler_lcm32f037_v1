/**
 * @file bsp_system.h
 * @brief [BSP] Core-level helpers (software reset)
 */

#ifndef __BSP_SYSTEM_H
#define __BSP_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/** Mask interrupts and request a system reset (SCB->AIRCR SYSRESETREQ). Never returns. */
void bsp_system_reset(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYSTEM_H */
