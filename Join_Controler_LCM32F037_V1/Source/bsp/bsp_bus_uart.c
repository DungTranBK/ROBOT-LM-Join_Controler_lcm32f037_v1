/**
 * @file bsp_bus_uart.c
 * @brief [BSP] Single-wire half-duplex UART0 driver (DMA based)
 *        Ported 1:1 from code_da_doc.bin:
 *          0x080045FC  UART0 FIFO + DMA setup (RX = DMAC CH0 circular, TX = DMAC CH1)
 *          0x080039F4  UART0 baud / frame / enable
 *          0x08004338  start TX  (PA9 -> AF2, load DMA CH1, enable)
 *          0x08001FB8  DMAC CH1..2 IRQ (TX done -> wait UART idle -> PA9 -> AF7)
 *          0x080044C4  RX head = DMAC CH0 CTL_H (items already received)
 *
 *        Hardware facts (all verified in the binary):
 *          - Bus pin   : PA9. AF2 = UART0_TX, AF7 = UART0_RX (idle / listening)
 *          - UART0 clk : SYSCLK 96 MHz (IBRD=6 @1 Mbaud) -> baud = 2,000,000 / (baud_reg + 1)
 *          - RX        : NO interrupt. DMA CH0 writes into a 255-byte circular
 *                        buffer, firmware polls CTL_H from the main loop.
 *          - TX        : DMA CH1, TC interrupt (IRQ10) releases the bus.
 *          - Return delay: TIM2 (0x40009000) one-pulse timer, 2 us/tick (PSC 191), IRQ17
 *                        starts the reply TX after ARR = return_delay ticks
 *                        (0x080048C8 init / 0x08003340 ISR / 0x080040E0 arm).
 */

#include "bsp/bsp_bus_uart.h"
#include "bsp/bsp_gpio.h"
#include "bsp/board_pins.h"
#include "system/lcm32_regs.h"

/* Golden dump: UART0 IBRD = 6, FBRD = 0 at 1 Mbaud, SYSCTRL ClkEnR1 = 0 -> 96 MHz / (16 * 6) */
#define UART0_CLOCK_HZ            96000000UL

#define DMA_CH_RX_NUM             0U   /* DMAC channel 0 (UART0_RX request) */
#define DMA_CH_TX_NUM             1U   /* DMAC channel 1 (UART0_TX request) */
#define DMA_REQ_UART0_TX          0UL
#define DMA_REQ_UART0_RX          1UL


#define BUS_BAUD_BASE_HZ          2000000UL  /* baud = BUS_BAUD_BASE_HZ / (baud_reg + 1) */
#define BUS_PKT_HEADER            0xFF
#define BUS_DMA_IRQ_NUM           10U        /* DMAC CH1..2 interrupt */

/* SYSCTRL / DMAC */
#define SYSCTRL_KEY_UNLOCK        0x05FA659AUL
#define DMAC_CFG_ENABLE           0x01UL
#define DMAC_CH_WE_MASK           0x101UL    /* channel enable / interrupt mask + write-enable bit */
#define DMA_CFG_H_DEFAULT         0x04UL     /* CFG_H reset value (FIFO mode)                      */
#define DMA_BLOCK_TS_MASK         0xFFFUL    /* BLOCK_TS field of CTL_H                            */

/* dma_init() parameters */
#define DMA_PERIPH_FIXED          3          /* SINC/DINC = no change                              */
#define DMA_MEM_INC               0
#define DMA_PRIO_TX               4
#define DMA_PRIO_RX               5
#define DMA_TT_FC_PERIPH_FLOW     2          /* TT_FC value used for both channels                 */

/* UART0 registers */
#define UART_LCR_8N1              0x60UL     /* 8 data bits, 1 stop, no parity */
#define UART_LCR_FIFO_EN          0x10UL
#define UART_IFLS_MASK            0x3FUL     /* RX/TX FIFO trigger levels      */
#define UART_DMACR_TXDMAE         0x02UL
#define UART_DMACR_RXDMAE         0x01UL
#define UART_CR_UARTEN            (1UL << 0)
#define UART_CR_TXE               (1UL << 8)
#define UART_CR_RXE               (1UL << 9)
#define UART_FR_BUSY              (1UL << 3)

/* TIM2 return-delay timer: 96 MHz / (191 + 1) = 2 us per tick */
#define BUS_DELAY_TIMER_PSC       191
#define BUS_TIM_CR1_OPM               0x08UL
#define BUS_TIM_SR_UIF                1UL
#define BUS_TIM_EGR_UG                1UL
#define BUS_TIM_DIER_UIE              1UL
#define BUS_TIM_CR1_CEN               1UL

static volatile uint8_t s_rx_buf[BSP_BUS_UART_RX_BUFFER_SIZE] = {0};   /* DMA CH0 destination (0x20000410) */
static volatile uint8_t s_tx_in_progress = 0;                          /* 0x20000079 */

static uint8_t  s_tx_buf[BSP_BUS_UART_TX_BUFFER_SIZE];        /* DMA CH1 source (0x2000060F) */
static uint16_t s_tx_len = 0;                                 /* 0x2000007F: DMA block size */
static uint16_t s_rx_tail = 0;
static volatile uint8_t s_pending_baud_valid = 0;
static volatile uint8_t s_pending_baud_reg = 0;

/* TIM2 (same register layout as TIM1) used as return-delay one-pulse timer */
#define TIM2_REGS                 ((volatile TIM1_TypeDef *) TIM2_BASE)

/* ---------------- Keyed SYSCTRL write (EDU_CFG0) ---------------- */

static void sysctrl_edu_cfg0_set(uint32_t clr, uint32_t set) {
    uint32_t pm;
    __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r" (pm) : : "memory");
    SYSCTRL->KEY = SYSCTRL_KEY_UNLOCK;
    SYSCTRL->EDU_CFG0 = (SYSCTRL->EDU_CFG0 & ~clr) | set;
    __asm volatile ("msr primask, %0" : : "r" (pm) : "memory");
}

/* ---------------- DMAC helpers (same semantics as vendor lcm32f037_dma.c) ---------------- */

static void reg_field(volatile uint32_t *reg, uint32_t shift, uint32_t mask, uint32_t val) {
    *reg = (*reg & ~(mask << shift)) | ((val & mask) << shift);
}

/* DMA_DeInit (0x08002094) */
static void dma_deinit(DMA_Channel_TypeDef *ch, uint32_t n) {
    uint32_t bit = 1UL << n;

    DMAC_CFG_REG |= DMAC_CFG_ENABLE;
    DMAC_CH_EN_REG = (DMAC_CH_EN_REG | (1UL << (n + 8U))) & ~bit;

    ch->SAR   = 0;
    ch->DAR   = 0;
    ch->CTL_L = 0;
    ch->CTL_H = 0;
    ch->CFG_L = 0;
    ch->CFG_H = DMA_CFG_H_DEFAULT;

    DMAC_RAW_TFR     &= ~bit;
    DMAC_RAW_BLOCK   &= ~bit;
    DMAC_RAW_SRCTRAN &= ~bit;
    DMAC_RAW_DSTTRAN &= ~bit;
    DMAC_RAW_ERR     &= ~bit;
    DMAC_STATUS_TFR     &= ~bit;
    DMAC_STATUS_BLOCK   &= ~bit;
    DMAC_STATUS_SRCTRAN &= ~bit;
    DMAC_STATUS_DSTTRAN &= ~bit;
    DMAC_STATUS_ERR     &= ~bit;
    DMAC_CLEAR_TFR     = bit;
    DMAC_CLEAR_BLOCK   = bit;
    DMAC_CLEAR_SRCTRAN = bit;
    DMAC_CLEAR_DSTTRAN = bit;
    DMAC_CLEAR_ERR     = bit;
}

/* DMA_ITConfig(ch, TC, ENABLE) (0x08002180): block-complete interrupt */
static void dma_it_enable(DMA_Channel_TypeDef *ch, uint32_t n) {
    ch->CTL_L |= 1UL;                          /* INT_EN */
    DMAC_MASK_BLOCK = (DMAC_CH_WE_MASK << n);    /* unmask block-complete of channel n */
}

/* DMA_Init (0x080021C0). periph_is_dst: 1 = memory -> peripheral */
static void dma_init(DMA_Channel_TypeDef *ch, uint32_t periph_addr, uint32_t mem_addr,
                     uint32_t periph_is_dst, uint32_t block_size,
                     uint32_t periph_inc, uint32_t mem_inc,
                     uint32_t circular, uint32_t priority, uint32_t tt_fc) {
    if (!periph_is_dst) {
        ch->SAR = periph_addr;
        ch->DAR = mem_addr;
        reg_field(&ch->CTL_L, 4, 7, 0);            /* SRC_TR_WIDTH = byte */
        reg_field(&ch->CTL_L, 1, 7, 0);            /* DST_TR_WIDTH = byte */
        reg_field(&ch->CTL_L, 9, 3, periph_inc);   /* SINC */
        reg_field(&ch->CTL_L, 7, 3, mem_inc);      /* DINC */
    } else {
        ch->DAR = periph_addr;
        ch->SAR = mem_addr;
        reg_field(&ch->CTL_L, 1, 7, 0);            /* DST_TR_WIDTH = byte */
        reg_field(&ch->CTL_L, 4, 7, 0);            /* SRC_TR_WIDTH = byte */
        reg_field(&ch->CTL_L, 9, 3, mem_inc);      /* SINC */
        reg_field(&ch->CTL_L, 7, 3, periph_inc);   /* DINC */
    }
    reg_field(&ch->CTL_H, 0, DMA_BLOCK_TS_MASK, block_size);   /* BLOCK_TS */
    reg_field(&ch->CFG_L, 30, 1, circular);        /* RELOAD_SRC */
    reg_field(&ch->CFG_L, 31, 1, circular);        /* RELOAD_DST */
    reg_field(&ch->CFG_L, 5, 7, priority);         /* CH_PRIOR */
    reg_field(&ch->CTL_L, 20, 7, tt_fc);           /* TT_FC */
    reg_field(&ch->CTL_L, 14, 7, 0);               /* SRC_MSIZE */
    reg_field(&ch->CTL_L, 11, 7, 0);               /* DEST_MSIZE */
}

/* DMA_RemapConfig (0x080022CC) */
static void dma_remap(DMA_Channel_TypeDef *ch, uint32_t req_num, uint32_t request) {
    reg_field(&ch->CFG_H, 11, 0xF, req_num);       /* DEST_PER */
    reg_field(&ch->CFG_H, 7,  0xF, req_num);       /* SRC_PER  */
    sysctrl_edu_cfg0_set(0xFUL << (req_num * 4U), request << (req_num * 4U));
}

/* DMA_Cmd(ENABLE) (0x08002028) */
static void dma_enable(uint32_t n) {
    DMAC_CH_EN_REG |= (DMAC_CH_WE_MASK << n);
}

/* ---------------- PA9 direction (0x08002584) ---------------- */

static void uart_pin_to_tx_mode(void) {
    bsp_gpio_set_af(PIN_BUS_PORT, PIN_BUS_NUM, PIN_BUS_AF_TX);
}

static void uart_pin_to_rx_mode(void) {
    bsp_gpio_set_af(PIN_BUS_PORT, PIN_BUS_NUM, PIN_BUS_AF_RX);
}

/* ---------------- UART0 config (0x080039F4 + UART_Init 0x080037FC) ---------------- */

static void uart_config(uint32_t baudrate) {
    uint32_t baud16, div_int, div_frac;

    UART0->CR = 0;                                      /* UART_Cmd(DISABLE) */

    /* UART_BaudRateConfig (0x08003718) with UART0 clock = SYSCLK 96 MHz */
    baud16   = baudrate * 16UL;
    div_int  = UART0_CLOCK_HZ / baud16;
    div_frac = UART0_CLOCK_HZ - baud16 * div_int;
    div_frac = (div_frac * 64UL + baud16 / 2UL) / baud16;
    UART0->IBRD = div_int;
    UART0->FBRD = div_frac;

    UART0->LCR |= UART_LCR_8N1;                           /* 8 data bits, 1 stop, no parity */
    UART0->CR  |= UART_CR_RXE | UART_CR_TXE;
    UART0->CR  |= UART_CR_UARTEN;                         /* UART_Cmd(ENABLE) */
}

/* baud = 2,000,000 / (reg + 1)  (0x080039FC..0x08003A10) */
static uint32_t baud_from_reg(uint8_t baud_reg_val) {
    return BUS_BAUD_BASE_HZ / ((uint32_t)baud_reg_val + 1UL);
}

/**
 * @brief Full UART0 + DMA bring-up (0x080045FC then 0x080039F4)
 */
void bsp_bus_uart_init(uint8_t baud_reg_val) {
    DMA_Channel_TypeDef *tx = DMA1_CH1;
    DMA_Channel_TypeDef *rx = DMA1_CH0;

    s_tx_buf[0] = BUS_PKT_HEADER;                       /* 0x08004328: packet header */
    s_tx_buf[1] = BUS_PKT_HEADER;
    s_rx_tail = 0;
    s_tx_in_progress = 0;

    /* --- FIFO + DMA request enables --- */
    UART0->LCR  |= UART_LCR_FIFO_EN;                    /* UART_FIFOEnable */
    UART0->IFLS  = (UART0->IFLS & ~UART_IFLS_MASK);     /* FIFO trigger levels = lowest */
    UART0->DMACR |= UART_DMACR_TXDMAE;
    UART0->DMACR |= UART_DMACR_RXDMAE;

    /* --- TX: DMAC CH1, memory -> UART0 DR, TC interrupt --- */
    dma_deinit(tx, DMA_CH_TX_NUM);
    dma_it_enable(tx, DMA_CH_TX_NUM);
    dma_init(tx, (uint32_t)&UART0->DR, (uint32_t)s_tx_buf,
             1 /*mem->periph*/, 0, DMA_PERIPH_FIXED, DMA_MEM_INC,
             0 /*normal*/, DMA_PRIO_TX, DMA_TT_FC_PERIPH_FLOW);
    dma_remap(tx, 1, DMA_REQ_UART0_TX);

    /* --- RX: DMAC CH0, UART0 DR -> 255-byte circular buffer --- */
    dma_deinit(rx, DMA_CH_RX_NUM);
    dma_init(rx, (uint32_t)&UART0->DR, (uint32_t)s_rx_buf,
             0 /*periph->mem*/, BSP_BUS_UART_RX_BUFFER_SIZE, DMA_PERIPH_FIXED, DMA_MEM_INC,
             1 /*circular*/, DMA_PRIO_RX, DMA_TT_FC_PERIPH_FLOW);
    dma_remap(rx, 0, DMA_REQ_UART0_RX);
    dma_enable(DMA_CH_RX_NUM);

    /* --- UART0 baud / frame / enable --- */
    uart_config(baud_from_reg(baud_reg_val));

    /* --- DMAC CH1..2 interrupt (IRQ10): TX complete releases the bus --- */
    NVIC->ICPR[0] = (1UL << BUS_DMA_IRQ_NUM);
    NVIC->ISER[0] = (1UL << BUS_DMA_IRQ_NUM);
}

/**
 * @brief Change baud rate from control-table register 0x04.
 *        Applied from bsp_bus_uart_service() once the pending status packet has
 *        been fully shifted out, otherwise the reply would be sent at the new
 *        baud rate and the host would see garbage.
 */
void bsp_bus_uart_set_baudrate_from_reg(uint8_t baud_reg_val) {
    s_pending_baud_reg = baud_reg_val;
    s_pending_baud_valid = 1;
}

void bsp_bus_uart_service(void) {
    if (s_pending_baud_valid && !s_tx_in_progress) {
        s_pending_baud_valid = 0;
        uart_config(baud_from_reg(s_pending_baud_reg));
    }
}

bool bsp_bus_uart_is_tx_busy(void) {
    return s_tx_in_progress != 0;
}

/* ---------------- RX (poll DMA position, 0x080044C4) ---------------- */

int bsp_bus_uart_rx_pop(uint8_t *byte) {
    uint16_t head = (uint16_t)(DMA1_CH0->CTL_H & DMA_BLOCK_TS_MASK);   /* items received so far */
    if (head >= BSP_BUS_UART_RX_BUFFER_SIZE) {
        head = (uint16_t)(head - BSP_BUS_UART_RX_BUFFER_SIZE);
    }
    if (s_rx_tail == head) {
        return 0;
    }
    *byte = s_rx_buf[s_rx_tail];
    if (++s_rx_tail >= BSP_BUS_UART_RX_BUFFER_SIZE) {
        s_rx_tail = 0;
    }
    return 1;
}

/* ---------------- TIM2 return-delay timer (0x08004894) ---------------- */

void bsp_bus_uart_delay_timer_init(void) {
    TIM2_REGS->PSC = BUS_DELAY_TIMER_PSC;  /* 96 MHz / 192 = 2 us per tick */
    TIM2_REGS->ARR = 0xFFFFFFFFUL;
    TIM2_REGS->EGR = BUS_TIM_EGR_UG;           /* load PSC/ARR */
    TIM2_REGS->SR &= ~BUS_TIM_SR_UIF;          /* TIM_ClearFlag(UPDATE) */
    TIM2_REGS->CR1 = (TIM2_REGS->CR1 & ~BUS_TIM_CR1_OPM) | BUS_TIM_CR1_OPM;   /* one-pulse mode */
    TIM2_REGS->DIER |= BUS_TIM_DIER_UIE;       /* update interrupt */
}

/* ---------------- TX (0x08004338) ---------------- */

/* 0x08004338: PA9 -> TX, DMA CH1 block size = length, SAR reload, enable */
static void uart_start_tx(void) {
    uart_pin_to_tx_mode();                                  /* PA9 -> AF2 (UART0_TX) */
    reg_field(&DMA1_CH1->CTL_H, 0, DMA_BLOCK_TS_MASK, s_tx_len);   /* block size = byte count */
    DMA1_CH1->SAR = (uint32_t)s_tx_buf;                     /* SAR increments, reload it */
    dma_enable(DMA_CH_TX_NUM);
}

/**
 * @brief Queue a reply. delay_ticks = return delay register (2 us units): 0 = send now,
 *        otherwise TIM2 is armed and IRQ17 starts the transmission (0x080040D6..0x08004106).
 */
void bsp_bus_uart_send_packet(const uint8_t *data, uint16_t len, uint8_t delay_ticks) {
    if (len == 0 || len > BSP_BUS_UART_TX_BUFFER_SIZE) return;

    /* Wait for a previous packet to leave the wire */
    while (s_tx_in_progress) { }

    for (uint16_t i = 0; i < len; i++) {
        s_tx_buf[i] = data[i];
    }
    s_tx_len = len;

    s_tx_in_progress = 1;

    if (delay_ticks != 0) {
        TIM2_REGS->ARR = delay_ticks;                       /* TIM_SetAutoreload */
        TIM2_REGS->CR1 |= BUS_TIM_CR1_CEN;                      /* TIM_Cmd(ENABLE), one pulse */
    } else {
        uart_start_tx();
    }
}

/* ---------------- IRQ17: TIM2 update (0x08003340) ---------------- */

void TIM2_Handler(void) {
    TIM2_REGS->SR &= ~BUS_TIM_SR_UIF;                              /* clear UIF */
    uart_start_tx();
}

/* ---------------- IRQ10: DMAC CH1..2, TX complete (0x08001FB8) ---------------- */

static void uart_tx_complete_irq(void) {
    if ((DMAC_STATUS_BLOCK & (1UL << DMA_CH_TX_NUM)) == 0) {   /* DMA_GetITStatus(TC1) */
        return;
    }
    DMAC_CLEAR_BLOCK = (1UL << DMA_CH_TX_NUM);                 /* DMA_ClearITPendingBit(TC1) */

    while (UART0->FR & UART_FR_BUSY) { }                       /* wait last bit shifted out (BUSY) */

    s_tx_in_progress = 0;                                      /* 0x20000079 */
    uart_pin_to_rx_mode();                                     /* release bus: PA9 -> AF7 (RX) */
}

void DMAC_CH1_2_Handler(void) {
    uart_tx_complete_irq();
}
