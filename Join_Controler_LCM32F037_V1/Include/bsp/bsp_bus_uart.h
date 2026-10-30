/**
 * @file bsp_bus_uart.h
 * @brief [BSP] Single-Wire Half-Duplex UART0 driver (PA9, DMA RX circular + DMA TX)
 *        Matching code_da_doc.bin 0x080045FC / 0x080039F4 / 0x08004338 / 0x08001FB8
 */

#ifndef __BSP_BUS_UART_H
#define __BSP_BUS_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BUS_UART_RX_BUFFER_SIZE   255   /* DMA CH0 circular buffer (BLOCK_TS = 0xFF) */
#define BSP_BUS_UART_TX_BUFFER_SIZE   256

void bsp_bus_uart_init(uint8_t baud_reg_val);
void bsp_bus_uart_set_baudrate_from_reg(uint8_t baud_reg_val);
void bsp_bus_uart_service(void);                              /* call from main loop */
int  bsp_bus_uart_rx_pop(uint8_t *byte);                      /* 1 = byte returned, 0 = empty */
void bsp_bus_uart_send_packet(const uint8_t *data, uint16_t len, uint8_t delay_ticks);
void bsp_bus_uart_delay_timer_init(void);                     /* TIM2 one-pulse return-delay timer, 0x08004894 */
bool bsp_bus_uart_is_tx_busy(void);                           /* reply still leaving the wire */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BUS_UART_H */
