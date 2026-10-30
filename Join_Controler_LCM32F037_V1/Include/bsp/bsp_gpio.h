/**
 * @file bsp_gpio.h
 * @brief [BSP] Pin multiplexing of the EC4012 motor board
 */

#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_GPIO_PORT_A = 0,
    BSP_GPIO_PORT_B = 1,
    BSP_GPIO_PORT_F = 2
} bsp_gpio_port_t;

void bsp_gpio_init(void);       /* all pin muxing (0x08003D04) */
void bsp_gpio_set_af(bsp_gpio_port_t port, uint32_t pin, uint32_t af);  /* 0x08002584 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
