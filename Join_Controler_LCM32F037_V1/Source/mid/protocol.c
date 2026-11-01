/**
 * @file protocol.c
 * @brief [Middleware] Dynamixel-style v1.0 serial bus protocol (slave side)
 *
 *   0x08003EBA..0x08003FFE : RX state machine (Task A of main())
 *   0x08003A44             : instruction dispatcher
 *   0x08004070             : status-packet builder + TIM2 arming
 *   0x080049C8             : register write handler
 */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "bsp/bsp_bus_uart.h"

#include "mid/checksum.h"
#include "mid/control_table.h"
#include "mid/eeprom_store.h"
#include "mid/foc_config.h"
#include "mid/motor_ctrl.h"
#include "mid/protocol.h"

/* True only when both Low and High bytes of a 16-bit register are covered */
#define WROTE_REG16(s, e, reg_l, reg_h) ((s) <= (reg_l) && (e) >= (reg_h))

static uint8_t
    s_packet_buffer[BSP_BUS_UART_RX_BUFFER_SIZE + PKT_RX_ASSEMBLER_MARGIN];
static uint16_t s_packet_len = 0;

static uint8_t s_reg_pkt[PKT_BUFFER_SIZE];
static uint8_t s_tx_pkt[PKT_BUFFER_SIZE];
static uint8_t s_tx_last = 0;

#if FEATURE_UART_STREAM_TELEMETRY
static volatile uint8_t s_stream_interval_ms = 0;
#endif

static void protocol_status_reply(const uint8_t *rx) {
  uint8_t inst = rx[PKT_IDX_INST];
  uint8_t level = g_control_table.status_return_level;
  bool send = false;

  if (rx[PKT_IDX_ID] == g_last_id) {
    if (level == STATUS_RETURN_ALL || inst == INST_PING) {
      send = true;
    } else if (level == STATUS_RETURN_READ_ONLY && inst == INST_READ_DATA) {
      send = true;
    }
  }

  if (send) {
    int16_t cur_pos = motor_ctrl_get_position();
    g_control_table.current_position = cur_pos;

    if (inst == INST_READ_DATA && rx[PKT_IDX_P1] == 0x33U &&
        rx[PKT_IDX_P2] == 0x11U) {
      /* Calibration web tool compatibility frame (READ 0x33, len 0x11):
       * [FF FF ID 16 ERR 04 02 33 calib_done (8B) ang_off(2) off_v(2) off_u(2) off_t(2) CS] */
      const uint8_t *tbl = (const uint8_t *)&g_control_table;
      s_tx_last = 8U + 0x11U;
      s_tx_pkt[0] = PKT_HEADER_BYTE;
      s_tx_pkt[1] = PKT_HEADER_BYTE;
      s_tx_pkt[2] = g_last_id;
      s_tx_pkt[3] = (uint8_t)(s_tx_last - PKT_STATUS_LEN_OFFSET);
      s_tx_pkt[4] = g_motor_status_flags;
      s_tx_pkt[5] = rx[PKT_IDX_LEN];
      s_tx_pkt[6] = inst;
      s_tx_pkt[7] = rx[PKT_IDX_P1];
      s_tx_pkt[8] = g_control_table.calib_done;
      memset(&s_tx_pkt[9], 0, 8U);
      memcpy(&s_tx_pkt[17], &tbl[0x3CU], 8U);
      s_tx_pkt[s_tx_last] =
          checksum_calc(&s_tx_pkt[2], (uint8_t)(s_tx_last - 2U));

      bsp_bus_uart_send_packet(s_tx_pkt, (uint16_t)s_tx_last + 1U,
                               g_control_table.return_delay_time);
      g_last_id = g_control_table.motor_id;
      return;
    }

    s_tx_pkt[0] = PKT_HEADER_BYTE;
    s_tx_pkt[1] = PKT_HEADER_BYTE;
    s_tx_pkt[2] = g_last_id;
    s_tx_pkt[3] = (uint8_t)(s_tx_last - PKT_STATUS_LEN_OFFSET);
    s_tx_pkt[4] = g_motor_status_flags;
    s_tx_pkt[5] = (uint8_t)((uint16_t)cur_pos & 0xFFU);
    s_tx_pkt[6] = (uint8_t)((uint16_t)cur_pos >> 8);
    s_tx_pkt[7] = rx[PKT_IDX_LEN];
    s_tx_pkt[8] = inst;
    s_tx_pkt[9] = (inst == INST_WRITE_DATA || inst == INST_READ_DATA ||
                   inst == INST_REG_WRITE)
                      ? rx[PKT_IDX_P1]
                      : 0;
    s_tx_pkt[s_tx_last] = checksum_calc(&s_tx_pkt[2], (uint8_t)(s_tx_last - 2));

    bsp_bus_uart_send_packet(s_tx_pkt, (uint16_t)s_tx_last + 1U,
                             g_control_table.return_delay_time);
  }

  g_last_id = g_control_table.motor_id;
}

static void handle_write_registers(uint8_t start_addr, const uint8_t *data,
                                   uint8_t len) {
  uint8_t *table_bytes = (uint8_t *)(&g_control_table);
  if (len == 0) {
    return;
  }
  uint8_t end_addr = (uint8_t)(start_addr + len - 1U);
  uint16_t prev_kp = g_control_table.kp;
  uint16_t prev_kd = g_control_table.kd;
  int16_t prev_goal_pos = g_control_table.goal_position;

  /* Pass 1: Copy writable bytes into g_control_table and handle immediate
   * single-byte control registers */
  for (uint8_t i = 0; i < len; i++) {
    uint8_t addr = (uint8_t)(start_addr + i);

    /* Read-only: 0x00..0x02, 0x0A, 0x11, >= 0x32, and 0x24..0x2E (except 0x2D
     * control_mode) */
    if (addr == REG_RESERVED_0A || addr == REG_RESERVED_11 ||
        addr < CT_ADDR_WRITABLE_START || addr >= CT_ADDR_HIDDEN_START) {
      continue;
    }
    if (addr >= CT_ADDR_RO_RAM_START && addr <= CT_ADDR_RO_RAM_END &&
        addr != REG_CONTROL_MODE) {
      continue;
    }

    uint8_t prev_val = table_bytes[addr];
    table_bytes[addr] = data[i];

    if (addr < CT_ADDR_EEPROM_END) { /* EEPROM area */
#if MOTOR_HAS_GEARBOX || MOTOR_USE_HALL_MT9102
      /* Angle limits (0x06..0x09) are set by Master each startup
       * and act as volatile RAM registers (never saved to Flash, never blocked
       * by EEPROM lock). */
      if (addr >= REG_MIN_ANGLE_L && addr <= REG_MAX_ANGLE_H) {
        continue;
      }
#endif
      if (g_eeprom_lock_flag) {
        table_bytes[addr] = prev_val;
        g_motor_status_flags |= ERR_FLAG_RANGE_LOCK;
      } else if (data[i] != prev_val) {
        g_eeprom_dirty_flag = 1; /* trigger flash save only when changed */
        if (addr == REG_BAUD_RATE) {
          bsp_bus_uart_set_baudrate_from_reg(data[i]);
        }
      }
      continue;
    }

    switch (addr) {
    case REG_TORQUE_ENABLE: /* 0x18: clear stall error when engaged */
      if (g_control_table.torque_enable == 1) {
        g_motor_status_flags &= (uint8_t)~ERR_FLAG_STALL;
      }
      break;

    case REG_CALIBRATION_ENABLE: /* 0x19: start calibration state machine */
      /* Flash save (g_eeprom_dirty_flag = 1) is triggered automatically at the
       * end of CAL_STATE_ALIGN once all offsets are measured. */
      (void)motor_ctrl_start_calibration();
      break;

    case REG_CONTROL_MODE: /* 0x2D: explicit RAM mode switch (2=Position, 3=MIT)
                            */
      if (g_control_table.control_mode != VEL_MODE_POSITION &&
          g_control_table.control_mode != VEL_MODE_IMPEDANCE) {
        g_control_table.control_mode = prev_val;
        g_motor_status_flags |= ERR_FLAG_RANGE_LOCK;
      } else if (g_control_table.control_mode != prev_val) {
        /* Bumpless transfer on mode switch: lock current position & clear
         * speed/tau_ff */
        int16_t cur_pos = motor_ctrl_get_position();
        g_control_table.goal_position = cur_pos;
        g_control_table.moving_speed = 0;
        g_control_table.tau_ff = 0;
        prev_goal_pos = cur_pos;
        motor_ctrl_set_goal_position(cur_pos);
        motor_ctrl_set_speed_cmd(0);
        motor_ctrl_set_tau_ff(0);
        g_control_table.moving_flag = 0;
        g_motor_status_flags &= (uint8_t)~ERR_FLAG_ANGLE;
      }
      break;

    case REG_LOCK_EEPROM: /* 0x2F */
      if (g_control_table.lock_eeprom == 1) {
        g_eeprom_lock_flag = 1;
      }
      break;

    default:
      break;
    }
  }

  /* Pass 2: Coherent latching of any complete 16-bit register(s) written within
   * the unified Control Block (0x1A..0x23):
   *   - Supports full 10-byte write (0x1A..0x23) as well as partial writes:
   *     * 2 bytes: kp (0x1A..0x1B), kd (0x1C..0x1D), pos (0x1E..0x1F),
   *                spd (0x20..0x21), or tau_ff (0x22..0x23)
   *     * 4 bytes: kp+kd (0x1A..0x1D), pos+spd (0x1E..0x21), spd+tau (0x20..0x23)
   *     * 6 bytes: kp+kd+pos (0x1A..0x1F) or pos+spd+tau (0x1E..0x23)
   *     * 8 bytes: kp+kd+pos+spd (0x1A..0x21)
   */
  if (end_addr >= REG_KP_L && start_addr <= REG_TAU_FF_H) {
    bool wrote_kp = WROTE_REG16(start_addr, end_addr, REG_KP_L, REG_KP_H);
    bool wrote_kd = WROTE_REG16(start_addr, end_addr, REG_KD_L, REG_KD_H);
    bool wrote_pos = WROTE_REG16(start_addr, end_addr, REG_GOAL_POSITION_L,
                                 REG_GOAL_POSITION_H);
    bool wrote_spd = WROTE_REG16(start_addr, end_addr, REG_MOVING_SPEED_L,
                                 REG_MOVING_SPEED_H);
    bool wrote_tau =
        WROTE_REG16(start_addr, end_addr, REG_TAU_FF_L, REG_TAU_FF_H);

    if (wrote_kp) {
      if (g_control_table.kp > g_control_table.kp_max) {
#if MOTOR_USE_HALL_MT9102
        (void)prev_kp;
        g_control_table.kp = g_control_table.kp_max;
        motor_ctrl_set_kp(g_control_table.kp);
#else
        g_control_table.kp = prev_kp;
        g_motor_status_flags |= ERR_FLAG_RANGE_LOCK;
#endif
      } else {
        motor_ctrl_set_kp(g_control_table.kp);
      }
    }
    if (wrote_kd) {
      if (g_control_table.kd > g_control_table.kd_max) {
#if MOTOR_USE_HALL_MT9102
        (void)prev_kd;
        g_control_table.kd = g_control_table.kd_max;
        motor_ctrl_set_kd(g_control_table.kd);
#else
        g_control_table.kd = prev_kd;
        g_motor_status_flags |= ERR_FLAG_RANGE_LOCK;
#endif
      } else {
        motor_ctrl_set_kd(g_control_table.kd);
      }
    }
    if (wrote_spd) {
      motor_ctrl_set_speed_cmd(g_control_table.moving_speed);
    }
    if (wrote_tau) {
      motor_ctrl_set_tau_ff(g_control_table.tau_ff);
    }

    if (wrote_pos) {
      if (g_control_table.goal_position < g_control_table.min_angle_limit ||
          g_control_table.goal_position > g_control_table.max_angle_limit) {
        g_control_table.goal_position = prev_goal_pos;
        g_motor_status_flags |= ERR_FLAG_ANGLE;
      } else {
        g_motor_status_flags &= (uint8_t)~ERR_FLAG_ANGLE;
        motor_ctrl_set_goal_position(g_control_table.goal_position);
        g_control_table.torque_enable = 1;
        g_motor_status_flags &= (uint8_t)~ERR_FLAG_STALL;
      }
    } else if (wrote_spd || wrote_tau) {
      g_control_table.torque_enable = 1;
      g_motor_status_flags &= (uint8_t)~ERR_FLAG_STALL;
    }
  }
}

static void flag_parse_error(void) { g_motor_status_flags |= ERR_FLAG_PARSE; }

void protocol_process_rx_packet(const uint8_t *rx_buf, uint8_t rx_len) {
  (void)rx_len;
  const uint8_t *p = &rx_buf[2];

  if (p[PKT_IDX_ID] != g_last_id && p[PKT_IDX_ID] != BROADCAST_ID) {
    return;
  }

  uint8_t len = p[PKT_IDX_LEN];

  s_tx_last = PKT_STATUS_BASE_LAST;
  g_motor_status_flags &= ERR_FLAGS_KEEP_ON_NEW_PACKET;

  if (len < PKT_MIN_LEN_FIELD || len > PKT_MAX_LEN_FIELD) {
    g_motor_status_flags |= ERR_FLAG_RANGE_LOCK;
    protocol_status_reply(p);
    return;
  }

  if (checksum_calc(p, (uint8_t)(len + 1U)) != p[len + 1U]) {
    g_motor_status_flags |= ERR_FLAG_CHECKSUM;
    protocol_status_reply(p);
    return;
  }

  switch (p[PKT_IDX_INST]) {
  case INST_PING:
    if (len != PKT_LEN_NO_PARAM) {
      flag_parse_error();
    }
    break;

  case INST_READ_DATA:
    if (len != PKT_LEN_READ) {
      flag_parse_error();
    } else {
      uint8_t start_addr = p[PKT_IDX_P1];
      uint8_t read_len = p[PKT_IDX_P2];
      s_tx_last = (uint8_t)(read_len + PKT_STATUS_DATA_INDEX);
      for (uint16_t i = 0; i < read_len; i++) {
        uint16_t a = (uint16_t)start_addr + i;
        s_tx_pkt[PKT_STATUS_DATA_INDEX + i] =
            (a < EEPROM_BLOCK_SIZE) ? ((const uint8_t *)&g_control_table)[a]
                                    : 0;
      }
    }
    break;

  case INST_WRITE_DATA:
    if (len < PKT_LEN_WRITE_MIN) {
      flag_parse_error();
    } else {
      handle_write_registers(p[PKT_IDX_P1], &p[PKT_IDX_P2],
                             (uint8_t)(len - PKT_WRITE_LEN_OVERHEAD));
    }
    break;

  case INST_REG_WRITE:
    if (len < PKT_LEN_WRITE_MIN) {
      flag_parse_error();
    } else {
      memcpy(s_reg_pkt, p, (size_t)len + 2U);
      g_control_table.registered_flag = 1;
    }
    break;

  case INST_ACTION:
    if (len != PKT_LEN_NO_PARAM || g_control_table.registered_flag != 1) {
      flag_parse_error();
    } else {
      handle_write_registers(
          s_reg_pkt[PKT_IDX_P1], &s_reg_pkt[PKT_IDX_P2],
          (uint8_t)(s_reg_pkt[PKT_IDX_LEN] - PKT_WRITE_LEN_OVERHEAD));
      g_control_table.registered_flag = 0;
    }
    break;

  case INST_RESET:
    if (len != PKT_LEN_NO_PARAM) {
      flag_parse_error();
    } else {
      eeprom_store_restore_defaults();
      g_eeprom_dirty_flag = 1;
    }
    break;

  case INST_SYNC_WRITE:
    if (len < PKT_LEN_SYNC_WRITE_MIN) {
      flag_parse_error();
    } else {
      int32_t end =
          (int32_t)p[PKT_IDX_P1] - (int32_t)p[PKT_IDX_SYNC_FIRST_DATA] + 1;
      uint8_t idx = PKT_IDX_SYNC_FIRST_ID;
      uint8_t per = p[PKT_IDX_P2];
      while (end > (int32_t)idx) {
        if (g_control_table.motor_id == p[idx]) {
          handle_write_registers(p[PKT_IDX_P1], &p[idx + 1], per);
          break;
        }
        idx = (uint8_t)(idx + 1U + per);
      }
    }
    break;

  case INST_REBOOT_IAP:
    if (len != PKT_LEN_NO_PARAM) {
      flag_parse_error();
    } else {
      g_reboot_flag = REBOOT_FLAG_REQUEST;
    }
    break;

#if FEATURE_UART_STREAM_TELEMETRY
  case INST_STREAM_CURRENT:
    if (len < 3) {
      flag_parse_error();
    } else {
      s_stream_interval_ms = p[PKT_IDX_P1];
    }
    break;
#endif

  default:
    flag_parse_error();
    break;
  }

  protocol_status_reply(p);
}

void protocol_poll(void) {
  uint8_t b;

  while (bsp_bus_uart_rx_pop(&b)) {
    if (s_packet_len == 0) {
      if (b == PKT_HEADER_BYTE) {
        s_packet_buffer[s_packet_len++] = b;
      }
    } else if (s_packet_len == 1) {
      if (b == PKT_HEADER_BYTE) {
        s_packet_buffer[s_packet_len++] = b;
      } else {
        s_packet_len = 0;
      }
    } else if (s_packet_len == 2 && b == PKT_HEADER_BYTE) {
      /* a further 0xFF where the ID is expected is skipped */
    } else {
      s_packet_buffer[s_packet_len++] = b;
      if (s_packet_len >= PKT_MIN_FRAME_BYTES) {
        uint16_t expected_total_len =
            (uint16_t)s_packet_buffer[PKT_MIN_FRAME_BYTES - 1] +
            PKT_FRAME_OVERHEAD; /* Length + 4 */
        if (expected_total_len > BSP_BUS_UART_RX_BUFFER_SIZE) {
          s_packet_len = 0; /* corrupt length byte, resync */
        } else if (s_packet_len >= expected_total_len) {
          protocol_process_rx_packet(s_packet_buffer, (uint8_t)s_packet_len);
          s_packet_len = 0;
        }
      }
    }
  }
}

#if FEATURE_UART_STREAM_TELEMETRY
void protocol_set_stream_interval(uint8_t interval_ms) {
  s_stream_interval_ms = interval_ms;
}

uint8_t protocol_get_stream_interval(void) { return s_stream_interval_ms; }

void protocol_send_stream_packet(void) {
  static uint8_t s_stream_pkt[15];
  int16_t iu = motor_ctrl_get_iu();
  int16_t iw = motor_ctrl_get_iw();
  int16_t iv = motor_ctrl_get_iv();
  int16_t pos = g_control_table.current_position;
  uint16_t elec = (uint16_t)motor_ctrl_get_elec_angle();

  s_stream_pkt[0] = 0xAA;
  s_stream_pkt[1] = 0x55;
  s_stream_pkt[2] = g_control_table.motor_id;
  s_stream_pkt[3] = 10;
  s_stream_pkt[4] = (uint8_t)(iu & 0xFF);
  s_stream_pkt[5] = (uint8_t)((uint16_t)iu >> 8);
  s_stream_pkt[6] = (uint8_t)(iw & 0xFF);
  s_stream_pkt[7] = (uint8_t)((uint16_t)iw >> 8);
  s_stream_pkt[8] = (uint8_t)(iv & 0xFF);
  s_stream_pkt[9] = (uint8_t)((uint16_t)iv >> 8);
  s_stream_pkt[10] = (uint8_t)(pos & 0xFF);
  s_stream_pkt[11] = (uint8_t)((uint16_t)pos >> 8);
  s_stream_pkt[12] = (uint8_t)(elec & 0xFF);
  s_stream_pkt[13] = (uint8_t)(elec >> 8);
  s_stream_pkt[14] = checksum_calc(&s_stream_pkt[2], 12);

  bsp_bus_uart_send_packet(s_stream_pkt, 15, 0);
}
#endif
