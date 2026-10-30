/**
 * @file bsp_opa.h
 * @brief [BSP] Three on-chip op-amps amplifying the shunt resistor voltages
 */

#ifndef __BSP_OPA_H
#define __BSP_OPA_H

#ifdef __cplusplus
extern "C" {
#endif

void bsp_opa_init(void);        /* 0x08003FBC : OPA0 total shunt, OPA1 phase V, OPA2 phase U */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OPA_H */
