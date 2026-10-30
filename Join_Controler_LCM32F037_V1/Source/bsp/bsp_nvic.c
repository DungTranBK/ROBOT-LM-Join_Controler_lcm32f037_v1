/**
 * @file bsp_nvic.c
 * @brief [BSP] Interrupt priorities, ported 1:1 from code_da_doc.bin 0x08003F10
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_tim.h"      /* NVIC_Init / NVIC_InitTypeDef live in the vendor TIM header */

#include "bsp/bsp_nvic.h"

/* 0x08003F10 : interrupt priorities (SysTick 0, TIM1_CC 1, DMAC_CH1_2 2) */
void bsp_nvic_init(void) {
    NVIC_InitTypeDef n;

    SCB->SHP[1] &= 0x00FFFFFFUL;

    n.NVIC_IRQChannelCmd      = ENABLE;
    n.NVIC_IRQChannel         = 14;      /* TIM1 CC */
    n.NVIC_IRQChannelPriority = 1;
    NVIC_Init(&n);

    n.NVIC_IRQChannel         = 10;      /* DMAC CH1..2 : UART TX complete */
    n.NVIC_IRQChannelPriority = 2;
    NVIC_Init(&n);

    n.NVIC_IRQChannel         = 17;      /* TIM2 : return-delay timer, starts reply TX */
    n.NVIC_IRQChannelPriority = 2;
    NVIC_Init(&n);
}
