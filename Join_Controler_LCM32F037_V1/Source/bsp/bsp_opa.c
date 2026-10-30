/**
 * @file bsp_opa.c
 * @brief [BSP] Op-amp driver, ported 1:1 from code_da_doc.bin 0x08003FBC
 *        Uses the vendor SDK headers, so it must NOT include system/lcm32_regs.h.
 */

#include "lcm32f037.h"
#include "lcm32f037_opa.h"

#include "bsp/bsp_opa.h"

/* 0x08003FBC : 3 op-amps for the shunt resistors (+ vendor auto trim in OPA_Init) */
void bsp_opa_init(void) {
    OPA_InitTypeDef opa;

    OPA_NINSelect(OPA_NINselect_PA6);

    opa.OPA_Oppselect        = OPA0_Oppselect_PF7;
    opa.OPA_Opnselect        = OPA0_Opnselect_PF6;
    opa.OPA_Gain             = OPA_Gain_20;
    opa.OPA_BiasVoltage      = OPA_BiasVoltage_EN;
    opa.OPA_FeedbackRes      = OPA_FeedbackRes_EN;
    opa.OPA_Opoto_Gpio       = OPA_OPOEX_EN;
    opa.OPA_OppRES_short     = 0;
    opa.OPA_OpnRES_short     = 0;
    opa.OPA_BiasVoltageSource = 0;
    OPA_Init(OPA0, &opa);
    OPA_Cmd(OPA0, ENABLE);

    opa.OPA_Oppselect        = OPA1_Oppselect_PA6;
    opa.OPA_Opnselect        = OPA1_Opnselect_PB1;
    OPA_Init(OPA1, &opa);
    OPA_Cmd(OPA1, ENABLE);

    opa.OPA_Oppselect        = OPA2_Oppselect_PB0;
    opa.OPA_Opnselect        = OPA2_Opnselect_PA7;
    OPA_Init(OPA2, &opa);
    OPA_Cmd(OPA2, ENABLE);
}
