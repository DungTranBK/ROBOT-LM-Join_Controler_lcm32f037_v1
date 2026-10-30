/**
 * @file lcm32_regs.h
 * @brief Hardware Register Definitions for LCM32F037 (LNCHIP / Linsynix)
 *        Derived directly from disassembled firmware code_da_doc.bin
 */

#ifndef __LCM32_REGS_H
#define __LCM32_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Memory Map ---------------- */
#define FLASH_BASE            (0x08000000UL)
#define SRAM_BASE             (0x20000000UL)
#define PERIPH_BASE           (0x40000000UL)

/* APB / AHB Peripheral Bases matching code_da_doc.bin & LCM32F037 datasheet */
#define TIM2_BASE             (0x40009000UL)
#define TIM1_BASE             (0x40010000UL)
#define UART0_BASE            (0x40014000UL)
#define UART1_BASE            (0x40015000UL)
#define OPA_CMP_BASE          (0x40017C00UL)
#define SPI1_BASE             (0x40018000UL)
#define ADC_BASE              (0x4001A000UL)
#define GPIOA_BASE            (0x48000000UL)
#define GPIOB_BASE            (0x48000200UL)
#define GPIOF_BASE            (0x48000A00UL)
#define DMAC_BASE             (0x48004000UL)

/* ---------------- Peripheral Structures ---------------- */

/* TIM1: Advanced Motor Control Timer (0x40010000) */
typedef struct {
    volatile uint32_t CR1;          /* 0x00 */
    volatile uint32_t CR2;          /* 0x04 */
    volatile uint32_t SMCR;         /* 0x08 */
    volatile uint32_t DIER;         /* 0x0C */
    volatile uint32_t SR;           /* 0x10 */
    volatile uint32_t EGR;          /* 0x14 */
    volatile uint32_t CCMR1;        /* 0x18 */
    volatile uint32_t CCMR2;        /* 0x1C */
    volatile uint32_t CCER;         /* 0x20 */
    volatile uint32_t CNT;          /* 0x24 */
    volatile uint32_t PSC;          /* 0x28 */
    volatile uint32_t ARR;          /* 0x2C: 1199 (0x04AF) for 40kHz PWM @ 96MHz */
    volatile uint32_t RCR;          /* 0x30 */
    volatile uint32_t CCR1;         /* 0x34: Phase U PWM Duty */
    volatile uint32_t CCR2;         /* 0x38: Phase V PWM Duty */
    volatile uint32_t CCR3;         /* 0x3C: Phase W PWM Duty */
    volatile uint32_t CCR4;         /* 0x40: ADC trigger compare */
    volatile uint32_t BDTR;         /* 0x44: Dead-time & Break control */
    volatile uint32_t DCR;          /* 0x48 */
    volatile uint32_t DMAR;         /* 0x4C */
} TIM1_TypeDef;

/* UART1 / UART0 (0x40014000) - ARM PrimeCell PL011 compatible */
typedef struct {
    volatile uint32_t DR;           /* 0x00: Data Register */
    volatile uint32_t RSR;          /* 0x04: Receive Status Register / Error Clear */
    uint32_t RESERVED0[4];          /* 0x08 - 0x14 */
    volatile uint32_t FR;           /* 0x18: Flag Register (BUSY=bit3, RXFE=bit4, TXFF=bit5) */
    uint32_t RESERVED1[2];          /* 0x1C - 0x20 */
    volatile uint32_t IBRD;         /* 0x24: Integer Baud Rate Register */
    volatile uint32_t FBRD;         /* 0x28: Fractional Baud Rate Register */
    volatile uint32_t LCR;          /* 0x2C: Line Control Register */
    volatile uint32_t CR;           /* 0x30: Control Register */
    volatile uint32_t IFLS;         /* 0x34: Interrupt FIFO Level Select */
    volatile uint32_t IMSC;         /* 0x38: Interrupt Mask Set/Clear */
    volatile uint32_t RIS;          /* 0x3C: Raw Interrupt Status */
    volatile uint32_t MIS;          /* 0x40: Masked Interrupt Status */
    volatile uint32_t ICR;          /* 0x44: Interrupt Clear Register */
    volatile uint32_t DMACR;        /* 0x48: DMA Control Register */
} UART_TypeDef;

/* SPI1 (0x40018000) */
typedef struct {
    volatile uint32_t CR1;          /* 0x00 */
    volatile uint32_t CR2;          /* 0x04 */
    volatile uint32_t SR;           /* 0x08: Status (RXNE=0x04, etc.) */
    volatile uint32_t DR;           /* 0x0C: Data 16-bit */
    volatile uint32_t CRCPR;        /* 0x10 */
    volatile uint32_t RXCRCR;       /* 0x14 */
    volatile uint32_t TXCRCR;       /* 0x18 */
} SPI_TypeDef;

/* ADC (0x4001A000) */
typedef struct {
    volatile uint32_t SR;           /* 0x00 */
    volatile uint32_t CR1;          /* 0x04 */
    volatile uint32_t CR2;          /* 0x08 */
    volatile uint32_t SMPR1;        /* 0x0C */
    volatile uint32_t SMPR2;        /* 0x10 */
    volatile uint32_t JOFR1;        /* 0x14 */
    volatile uint32_t JOFR2;        /* 0x18 */
    volatile uint32_t JOFR3;        /* 0x1C */
    volatile uint32_t JOFR4;        /* 0x20 */
    volatile uint32_t HTR;          /* 0x24 */
    volatile uint32_t LTR;          /* 0x28 */
    volatile uint32_t SQR1;         /* 0x2C */
    volatile uint32_t SQR2;         /* 0x30 */
    volatile uint32_t SQR3;         /* 0x34 */
    volatile uint32_t JSQR;         /* 0x38 */
    volatile uint32_t JDR1;         /* 0x3C */
    volatile uint32_t JDR2;         /* 0x40 */
    volatile uint32_t JDR3;         /* 0x44 */
    volatile uint32_t JDR4;         /* 0x48 */
    volatile uint32_t DR;           /* 0x4C */
} ADC_TypeDef;

/* OPA / CMP (0x40017C00) */
typedef struct {
    volatile uint32_t OPACR1;       /* 0x00: OPA0 control (Total bus current shunt - PF7/PF6) */
    volatile uint32_t OPACR2;       /* 0x04: OPA1 control (Phase V shunt - PA6/PB1) */
    volatile uint32_t OPACR3;       /* 0x08: OPA2 control (Phase U shunt - PB0/PA7) */
    volatile uint32_t OPATRIM;      /* 0x0C: 5-bit offset trim calibration */
} OPA_TypeDef;

/* DMAC channel (DesignWare AHB DMAC, 0x58 bytes per channel).
 * CH0 = 0x48004000, CH1 = 0x48004058 ... (verified: DMA_DeInit @0x08002094) */
typedef struct {
    volatile uint32_t SAR;          /* 0x00: Source address */
    uint32_t RESERVED0;             /* 0x04 */
    volatile uint32_t DAR;          /* 0x08: Destination address */
    uint32_t RESERVED1[3];          /* 0x0C - 0x14 */
    volatile uint32_t CTL_L;        /* 0x18: INT_EN[0] DST_TR_WIDTH[3:1] SRC_TR_WIDTH[6:4]
                                     *       DINC[8:7] SINC[10:9] DEST_MSIZE[13:11]
                                     *       SRC_MSIZE[16:14] TT_FC[22:20] */
    volatile uint32_t CTL_H;        /* 0x1C: BLOCK_TS[11:0] (read = items already transferred) */
    uint32_t RESERVED2[8];          /* 0x20 - 0x3C */
    volatile uint32_t CFG_L;        /* 0x40: CH_PRIOR[7:5] RELOAD_SRC[30] RELOAD_DST[31] */
    volatile uint32_t CFG_H;        /* 0x44: SRC_PER[10:7] DEST_PER[14:11] */
    uint32_t RESERVED3[4];          /* 0x48 - 0x54 */
} DMA_Channel_TypeDef;

/* DMAC common registers (offsets from DMAC_BASE) */
#define DMAC_REG(off)         (*(volatile uint32_t *)(DMAC_BASE + (off)))
#define DMAC_RAW_TFR          DMAC_REG(0x2C0)
#define DMAC_RAW_BLOCK        DMAC_REG(0x2C8)
#define DMAC_RAW_SRCTRAN      DMAC_REG(0x2D0)
#define DMAC_RAW_DSTTRAN      DMAC_REG(0x2D8)
#define DMAC_RAW_ERR          DMAC_REG(0x2E0)
#define DMAC_STATUS_TFR       DMAC_REG(0x2E8)
#define DMAC_STATUS_BLOCK     DMAC_REG(0x2F0)
#define DMAC_STATUS_SRCTRAN   DMAC_REG(0x2F8)
#define DMAC_STATUS_DSTTRAN   DMAC_REG(0x300)
#define DMAC_STATUS_ERR       DMAC_REG(0x308)
#define DMAC_MASK_BLOCK       DMAC_REG(0x318)
#define DMAC_CLEAR_TFR        DMAC_REG(0x338)
#define DMAC_CLEAR_BLOCK      DMAC_REG(0x340)
#define DMAC_CLEAR_SRCTRAN    DMAC_REG(0x348)
#define DMAC_CLEAR_DSTTRAN    DMAC_REG(0x350)
#define DMAC_CLEAR_ERR        DMAC_REG(0x358)
#define DMAC_CFG_REG          DMAC_REG(0x398)
#define DMAC_CH_EN_REG        DMAC_REG(0x3A0)

/* GPIO port (verified: GPIO_Init @0x080024AC, GPIO_PinAFConfig @0x08002584) */
typedef struct {
    volatile uint32_t MODER;        /* 0x00: 2 bits/pin (0=in, 1=out, 2=AF, 3=analog) */
    volatile uint32_t OD_BSRR;      /* 0x04: open-drain set(bit n) / reset(bit n+16) */
    volatile uint32_t SR_BSRR;      /* 0x08: slew rate set/reset */
    volatile uint32_t PU_BSRR;      /* 0x0C: pull-up set/reset */
    volatile uint32_t PD_BSRR;      /* 0x10: pull-down set/reset */
    uint32_t RESERVED0[4];          /* 0x14 - 0x20 */
    volatile uint32_t AFRL;         /* 0x24: AF for pins 0..7 (4 bits each) */
    volatile uint32_t AFRH;         /* 0x28: AF for pins 8..15 */
    uint32_t RESERVED1[3];          /* 0x2C - 0x34 */
    volatile uint32_t CS_BSRR;      /* 0x38: CMOS/Schmitt input select set/reset */
} GPIO_TypeDef;

/* SYSCTRL (0x48007000): peripheral clock gating, key register @0x7C */
typedef struct {
    volatile uint32_t ClkEnR0;      /* 0x00: AHB/APB clock enable */
    volatile uint32_t ClkEnR1;      /* 0x04: UART0_CLKSEL[13:12] (2 = RCH 16 MHz) */
    volatile uint32_t ClkEnR2;      /* 0x08: bit5 = ANACTRL */
    uint32_t RESERVED0[5];          /* 0x0C - 0x1C */
    volatile uint32_t EDU_CFG0;     /* 0x20: DMA request mapping, 4 bits per ReqNum */
    uint32_t RESERVED1[22];         /* 0x24 - 0x78 */
    volatile uint32_t KEY;          /* 0x7C: write 0x05FA659A to unlock next write */
} SYSCTRL_TypeDef;

/* CHIPCTRL (0x40017000): PLL / clock source, key register @0x20 */
typedef struct {
    volatile uint32_t CLK_CFG;      /* 0x00 */
    uint32_t RESERVED0;             /* 0x04 */
    volatile uint32_t CTRL;         /* 0x08 */
    volatile uint32_t PLL_CFG;      /* 0x0C */
    uint32_t RESERVED1;             /* 0x10 */
    volatile uint32_t OSCH_CFG;     /* 0x14 */
    volatile uint32_t STS;          /* 0x18: bit20 = PLL lock, bit22 = SYSCLK lock */
    uint32_t RESERVED2;             /* 0x1C */
    volatile uint32_t KEY;          /* 0x20: write 0x87214365 to unlock next write */
    volatile uint32_t BDCR;         /* 0x24 */
    volatile uint32_t CFG28;        /* 0x28 */
    uint32_t RESERVED3[4];          /* 0x2C - 0x38 */
    volatile uint32_t LVR_CFG;      /* 0x3C: LVR level bits [3:2] */
} CHIPCTRL_TypeDef;

#define CHIPCTRL_BASE         (0x40017000UL)
#define SYSCTRL_BASE          (0x48007000UL)
#define PWR_CTRL_REG          (*(volatile uint32_t *)0x4001D000UL)

/* Cortex-M0 System Control Space */
#define SCS_BASE              (0xE000E000UL)
#define SysTick_BASE          (SCS_BASE +  0x0010UL)
#define NVIC_BASE             (SCS_BASE +  0x0100UL)
#define SCB_BASE              (SCS_BASE +  0x0D00UL)

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_TypeDef;

typedef struct {
    volatile uint32_t ISER[1];
    uint32_t RESERVED0[31];
    volatile uint32_t ICER[1];
    uint32_t RSERVED1[31];
    volatile uint32_t ISPR[1];
    uint32_t RESERVED2[31];
    volatile uint32_t ICPR[1];
    uint32_t RESERVED3[31];
    uint32_t RESERVED4[64];
    volatile uint32_t IP[8];
} NVIC_TypeDef;

typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    uint32_t RESERVED0;
    volatile uint32_t AIRCR;        /* 0x0C: Reset register (0x05FA0004 for sys reset) */
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    uint32_t RESERVED1;
    volatile uint32_t SHP[2];
} SCB_TypeDef;

/* Peripheral Pointers */
#define TIM1                  ((TIM1_TypeDef *) TIM1_BASE)
#define UART0                 ((UART_TypeDef *) UART0_BASE)
#define UART1                 ((UART_TypeDef *) UART1_BASE)
#define SPI1                  ((SPI_TypeDef *) SPI1_BASE)
#define ADC1                  ((ADC_TypeDef *) ADC_BASE)
#define OPA                   ((OPA_TypeDef *) OPA_CMP_BASE)
#define DMA1_CH0              ((DMA_Channel_TypeDef *) (DMAC_BASE + 0x000))
#define DMA1_CH1              ((DMA_Channel_TypeDef *) (DMAC_BASE + 0x058))
#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOF                 ((GPIO_TypeDef *) GPIOF_BASE)
#define SYSCTRL               ((SYSCTRL_TypeDef *) SYSCTRL_BASE)
#define CHIPCTRL              ((CHIPCTRL_TypeDef *) CHIPCTRL_BASE)
#define SysTick               ((SysTick_TypeDef *) SysTick_BASE)
#define NVIC                  ((NVIC_TypeDef *) NVIC_BASE)
#define SCB                   ((SCB_TypeDef *) SCB_BASE)

/* System layer services (Source/system/system_lcm32f037.c).
 * Board / peripheral bring-up lives in the BSP layer (Source/bsp/). */
void LM_SystemInit(void);          /* Reset_Handler: RCC_DeInit + PLL 96 MHz (0x080032C8) */
void lm_sysctrl_rmw(volatile uint32_t *reg, uint32_t clr, uint32_t set); /* keyed SYSCTRL read-modify-write */

/* CMSIS Intrinsics */
#if defined(__CC_ARM)
  #define __disable_irq()     __disable_irq()
  #define __enable_irq()      __enable_irq()
#else
  #define __disable_irq()     __asm volatile ("cpsid i" : : : "memory")
  #define __enable_irq()      __asm volatile ("cpsie i" : : : "memory")
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LCM32_REGS_H */
