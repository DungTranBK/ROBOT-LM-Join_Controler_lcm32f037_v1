/**
 * @file protocol.h
 * @brief [Middleware] Universal Bus Motor Protocol (V3): packet assembler,
 * parser and instruction dispatcher.
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#include "mid/foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PKT_HEADER_BYTE 0xFF
#define BROADCAST_ID 0xFE

/* Instruction Codes */
#define INST_PING 0x01
#define INST_READ_DATA 0x02
#define INST_WRITE_DATA 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_REBOOT_IAP                                                        \
  0x64 /* 100: Reboots to bootloader via NVIC_SystemReset */
#define INST_SYNC_WRITE 0x83

#if FEATURE_UART_STREAM_TELEMETRY
#define INST_STREAM_CURRENT                                                    \
  0x50 /* Real-time telemetry streaming: P1 = interval ms (0 = disable) */
#endif

/* Packet geometry */
#define PKT_BUFFER_SIZE 260 /* reg-write copy / status reply buffers */
#define PKT_MIN_FRAME_BYTES                                                    \
  4 /* FF FF ID LEN: LEN is known once received       */
#define PKT_FRAME_OVERHEAD                                                     \
  4 /* total frame = LEN + 4                          */
#define PKT_RX_ASSEMBLER_MARGIN                                                \
  8 /* slack above the bus RX ring size               */
#define PKT_STATUS_BASE_LAST                                                   \
  10 /* status reply without data: index of checksum (after 2-byte Pos +       \
        3-byte echo) */
#define PKT_STATUS_DATA_INDEX                                                  \
  10 /* first data byte of a READ reply                */
#define PKT_STATUS_LEN_OFFSET                                                  \
  3 /* reply LEN = last index - 3                     */

/* Request fields, relative to [ID, LEN, INST, P1, P2 ..] (header already
 * stripped) */
#define PKT_IDX_ID 0
#define PKT_IDX_LEN 1
#define PKT_IDX_INST 2
#define PKT_IDX_P1 3
#define PKT_IDX_P2 4
#define PKT_IDX_SYNC_FIRST_ID 5
#define PKT_IDX_SYNC_FIRST_DATA 6

/* Expected LEN field per instruction */
#define PKT_MIN_LEN_FIELD 2
#define PKT_MAX_LEN_FIELD 130
#define PKT_LEN_NO_PARAM 2 /* PING, ACTION, RESET, REBOOT */
#define PKT_LEN_READ 4
#define PKT_LEN_READ_DATA 4
#define PKT_LEN_WRITE_MIN 4 /* WRITE_DATA, REG_WRITE */
#define PKT_LEN_SYNC_WRITE_MIN 6
#define PKT_WRITE_LEN_OVERHEAD                                                 \
  3 /* data bytes = LEN - 3 (WRITE / ACTION)          */

/** Drain the bus receive buffer, assemble packets (FF FF ID LEN ...) and
 * dispatch them. Call from the main loop. */
void protocol_poll(void);

/** Dispatch one complete packet [0xFF, 0xFF, ID, LEN, INST, PARAMS.., CS]. */
void protocol_process_rx_packet(const uint8_t *rx_buf, uint8_t rx_len);

#if FEATURE_UART_STREAM_TELEMETRY
/** Set/Get telemetry stream interval in milliseconds (0 = off, 1 = 1ms/1kHz, 2
 * = 2ms/500Hz) */
void protocol_set_stream_interval(uint8_t interval_ms);
uint8_t protocol_get_stream_interval(void);

/** Send one 15-byte real-time telemetry packet: [0xAA, 0x55, ID, LEN=10, Iu(2),
 * Iw(2), Iv(2), Pos(2), Elec(2), CS] */
void protocol_send_stream_packet(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
