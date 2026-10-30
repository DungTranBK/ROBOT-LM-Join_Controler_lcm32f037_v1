/**
 * @file bsp_system.c
 * @brief [BSP] Core-level helpers (software reset), matching disasm 0x08003ED6
 */

#include "system/lcm32_regs.h"
#include "bsp/bsp_system.h"

#define SCB_AIRCR_VECTKEY         0x05FA0000UL   /* write key, must accompany every AIRCR write */
#define SCB_AIRCR_SYSRESETREQ     0x00000004UL

void bsp_system_reset(void) {
    __disable_irq();
    SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    while (1);
}
