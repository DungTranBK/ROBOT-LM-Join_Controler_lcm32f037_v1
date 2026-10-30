/**
 * @file system_lcm32f037.c
 * @brief [System layer] Core clock bring-up for LCM32F037 (PLL 96 MHz)
 *        Ported 1:1 from code_da_doc.bin:
 *          SystemInit        0x080032C8 (+ RCC_DeInit 0x08002F14, PLL 0x08002678)
 *
 *        Peripheral clock gating and pin muxing (board_clock_init 0x08004128,
 *        board_gpio_init 0x08003D04) belong to the BSP layer: bsp_clock.c / bsp_gpio.c.
 *
 *        NOTE: SystemInit runs from Reset_Handler BEFORE __main, so it must not
 *        touch any RAM-initialised/zero-initialised global.
 */

#include "system/lcm32_regs.h"

#define CHIPCTRL_KEY_VALUE    0x87214365UL
#define SYSCTRL_KEY_VALUE     0x05FA659AUL

uint32_t SystemCoreClock = 96000000UL; /* 96 MHz (PLL, see SystemInit) */

/* ---------------- Keyed register access (IRQ masked during unlock+write) ---------------- */

static inline uint32_t irq_save(void) {
    uint32_t primask;
    __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r" (primask) : : "memory");
    return primask;
}

static inline void irq_restore(uint32_t primask) {
    __asm volatile ("msr primask, %0" : : "r" (primask) : "memory");
}

/* reg = (reg & ~clr) | set  after unlocking CHIPCTRL */
static void chipctrl_rmw(volatile uint32_t *reg, uint32_t clr, uint32_t set) {
    uint32_t pm = irq_save();
    CHIPCTRL->KEY = CHIPCTRL_KEY_VALUE;
    *reg = (*reg & ~clr) | set;
    irq_restore(pm);
}

static void chipctrl_wr(volatile uint32_t *reg, uint32_t val) {
    uint32_t pm = irq_save();
    CHIPCTRL->KEY = CHIPCTRL_KEY_VALUE;
    *reg = val;
    irq_restore(pm);
}

/* Exported: also used by the BSP layer for peripheral clock gating. */
void lm_sysctrl_rmw(volatile uint32_t *reg, uint32_t clr, uint32_t set) {
    uint32_t pm = irq_save();
    SYSCTRL->KEY = SYSCTRL_KEY_VALUE;
    *reg = (*reg & ~clr) | set;
    irq_restore(pm);
}

/* ---------------- SystemInit (0x080032C8) ---------------- */

void LM_SystemInit(void) {
    /* --- RCC_DeInit (0x08002F14) --- */
    chipctrl_wr(&CHIPCTRL->CLK_CFG, 0x00080000UL);
    chipctrl_wr(&CHIPCTRL->CTRL,    0x00000000UL);
    chipctrl_wr(&CHIPCTRL->PLL_CFG, 0x0000115EUL);
    chipctrl_wr(&CHIPCTRL->OSCH_CFG, 0x00000007UL);
    chipctrl_rmw(&CHIPCTRL->BDCR,  0, 0x80UL);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0, (1UL << 11));
    chipctrl_rmw(&CHIPCTRL->CFG28, (1UL << 11), 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, (3UL << 9), 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, (1UL << 16), 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0x01UL, 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0x40UL, 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0x30UL, 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0x0CUL, 0);
    chipctrl_rmw(&CHIPCTRL->CFG28, 0x02UL, 0);
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR0, 0, (1UL << 17));   /* SRAM0 */
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR0, 0, (1UL << 18));   /* SRAM1 */

    /* --- ANACTRL (0x40017C00) trim bits, clock gated on just for the access --- */
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR2, 0, 0x20UL);        /* ANACTRL clock ON */
    {
        volatile uint32_t *ana = (volatile uint32_t *)0x40017C00UL;
        ana[0] |= (5UL << 25);
        ana[1] |= (5UL << 25);
        ana[2] |= (5UL << 25);
        volatile uint32_t *ana_c0 = (volatile uint32_t *)0x40017CC0UL;
        ana_c0[0x38 / 4] = ana_c0[0x38 / 4];          /* write-back (clear sticky flags) */
    }
    lm_sysctrl_rmw(&SYSCTRL->ClkEnR2, 0x20UL, 0);        /* ANACTRL clock OFF */
    __asm volatile ("nop\n\tnop");

    /* --- HSI_SetSysClockTo96 (0x08002678) --- */
    chipctrl_rmw(&CHIPCTRL->LVR_CFG, 0x0CUL, 0x0CUL); /* PWR_LVRConfig(level 3) */
    PWR_CTRL_REG = 0x12UL;                            /* PWR_LVRCmd(ENABLE) */
    chipctrl_wr(&CHIPCTRL->CLK_CFG, 0x00080000UL);
    chipctrl_wr(&CHIPCTRL->PLL_CFG, 0x0000015EUL);    /* 96 MHz */
    chipctrl_rmw(&CHIPCTRL->CLK_CFG, 0, 0x00090000UL);
    while ((CHIPCTRL->STS & (1UL << 20)) == 0) { }    /* PLL lock */
    chipctrl_rmw(&CHIPCTRL->CLK_CFG, 0, 0x00290000UL);
    while ((CHIPCTRL->STS & (1UL << 22)) == 0) { }    /* SYSCLK switched to PLL */
}
