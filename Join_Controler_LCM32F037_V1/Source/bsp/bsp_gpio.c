/**
 * @file bsp_gpio.c
 * @brief [BSP] GPIO driver + board pin map, ported 1:1 from code_da_doc.bin
 *          0x080024AC GPIO_Init, 0x08002584 GPIO_PinAFConfig, 0x08003D04 board_gpio_init
 */

#include "system/lcm32_regs.h"
#include "bsp/bsp_gpio.h"
#include "bsp/board_pins.h"

/* gpio_init_pins() parameters */
#define BSP_GPIO_MODE_AF      2U
#define BSP_GPIO_MODE_ANALOG  3U
#define BSP_GPIO_PUPD_NONE    0U
#define BSP_GPIO_PUPD_DOWN    2U

static GPIO_TypeDef * const s_gpio_port[] = { GPIOA, GPIOB, GPIOF };

/* GPIO_PinAFConfig: 4-bit alternate function per pin */
static void gpio_set_af(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af) {
    volatile uint32_t *afr = (pin < 8) ? &gpio->AFRL : &gpio->AFRH;
    uint32_t sh = (pin & 7U) * 4U;
    *afr = (*afr & ~(0xFUL << sh)) | ((af & 0xFUL) << sh);
}

void bsp_gpio_set_af(bsp_gpio_port_t port, uint32_t pin, uint32_t af) {
    gpio_set_af(s_gpio_port[port], pin, af);
}

/* GPIO_Init: mode 0=in 1=out 2=AF 3=analog; pupd 0=none 1=up 2=down 3=both;
 * otype 0=push-pull 1=open-drain */
static void gpio_init_pins(GPIO_TypeDef *gpio, uint32_t mask, uint32_t mode,
                           uint32_t speed, uint32_t otype, uint32_t pupd) {
    if (mode == 1U) {
        pupd = 0U;
    }
    for (uint32_t pin = 0; pin < 16U; pin++) {
        if ((mask & (1UL << pin)) == 0U) continue;
        gpio->MODER = (gpio->MODER & ~(3UL << (pin * 2U))) | (mode << (pin * 2U));

        gpio->SR_BSRR = speed ? (1UL << pin) : (1UL << (pin + 16U));

        switch (pupd) {
            case 1:  gpio->PU_BSRR = (1UL << pin); break;
            case 2:  gpio->PD_BSRR = (1UL << pin); break;
            case 3:  gpio->PU_BSRR = (1UL << pin); gpio->PD_BSRR = (1UL << pin); break;
            default: gpio->PU_BSRR = (1UL << (pin + 16U));
                     gpio->PD_BSRR = (1UL << (pin + 16U)); break;
        }

        if (otype == 0U) {
            gpio->OD_BSRR = (1UL << (pin + 16U));
            gpio->CS_BSRR = (1UL << (pin + 16U));
        } else if (otype == 1U) {
            gpio->OD_BSRR = (1UL << pin);
        } else {
            gpio->CS_BSRR = (1UL << pin);
        }
    }
}

/** Write the alternate function number of one board pin (see bsp/board_pins.h). */
#define BOARD_PIN_SET_AF(sig) \
    gpio_set_af(s_gpio_port[PIN_##sig##_PORT], PIN_##sig##_NUM, PIN_##sig##_AF)

/**
 * @brief Full pin mux of the motor board, same order/values as 0x08003D04.
 *        Pin assignment lives in bsp/board_pins.h.
 */
void bsp_gpio_init(void) {
    /* UART 1-wire bus */
    gpio_init_pins(s_gpio_port[PIN_BUS_PORT], BOARD_PIN_MASK(BUS),
                   BSP_GPIO_MODE_AF, 0, 0, BSP_GPIO_PUPD_NONE);
    BOARD_PIN_SET_AF(BUS);

    /* TIM1 PWM, port B pins: U/V/W high side + U low side */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_B],
                   BOARD_PIN_MASK(PWM_U_L) | BOARD_PIN_MASK(PWM_U_H) |
                   BOARD_PIN_MASK(PWM_V_H) | BOARD_PIN_MASK(PWM_W_H),
                   BSP_GPIO_MODE_AF, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(PWM_U_L);
    BOARD_PIN_SET_AF(PWM_U_H);
    BOARD_PIN_SET_AF(PWM_V_H);
    BOARD_PIN_SET_AF(PWM_W_H);

    /* TIM1 PWM, port F pins: V/W low side */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_F],
                   BOARD_PIN_MASK(PWM_W_L) | BOARD_PIN_MASK(PWM_V_L),
                   BSP_GPIO_MODE_AF, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(PWM_W_L);
    BOARD_PIN_SET_AF(PWM_V_L);

    /* SPI0: MT6701 angle sensor */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_A],
                   BOARD_PIN_MASK(ENC_NSS) | BOARD_PIN_MASK(ENC_SCK) | BOARD_PIN_MASK(ENC_MISO),
                   BSP_GPIO_MODE_AF, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(ENC_NSS);
    BOARD_PIN_SET_AF(ENC_SCK);
    BOARD_PIN_SET_AF(ENC_MISO);

    /* ADC inputs: Vbus, NTC */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_A],
                   BOARD_PIN_MASK(VBUS) | BOARD_PIN_MASK(NTC),
                   BSP_GPIO_MODE_ANALOG, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(VBUS);
    BOARD_PIN_SET_AF(NTC);

    /* Op-amp inputs, port A */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_A],
                   BOARD_PIN_MASK(OP1P0) | BOARD_PIN_MASK(OP2N1),
                   BSP_GPIO_MODE_ANALOG, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(OP1P0);
    BOARD_PIN_SET_AF(OP2N1);

    /* Op-amp inputs, port B */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_B],
                   BOARD_PIN_MASK(OP2P0) | BOARD_PIN_MASK(OP1N0),
                   BSP_GPIO_MODE_ANALOG, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(OP2P0);
    BOARD_PIN_SET_AF(OP1N0);

    /* Op-amp inputs, port F (total current shunt) */
    gpio_init_pins(s_gpio_port[BSP_GPIO_PORT_F],
                   BOARD_PIN_MASK(OP0N3) | BOARD_PIN_MASK(OP0P3),
                   BSP_GPIO_MODE_ANALOG, 0, 0, BSP_GPIO_PUPD_DOWN);
    BOARD_PIN_SET_AF(OP0N3);
    BOARD_PIN_SET_AF(OP0P3);
}
