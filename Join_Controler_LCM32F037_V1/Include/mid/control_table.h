/**
 * @file control_table.h
 * @brief [Middleware] Device Control Table (register map shared by protocol,
 * motor control and EEPROM store) + device status/state flags.
 */

#ifndef __CONTROL_TABLE_H
#define __CONTROL_TABLE_H

#include <stdint.h>
#include "mid/foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Status Error Flags (Byte Status in response packet) */
#define ERR_FLAG_NONE 0x00
#define ERR_FLAG_VOLTAGE (1 << 0)  /* 0x01: Voltage outside limits */
#define ERR_FLAG_ANGLE (1 << 1)    /* 0x02: Goal position exceeds bounds */
#define ERR_FLAG_OVERHEAT (1 << 2) /* 0x04: Temperature > temp_upper_limit */
#define ERR_FLAG_RANGE_LOCK                                                    \
  (1 << 3) /* 0x08: Parameter exceeds range / EEPROM locked */
#define ERR_FLAG_CHECKSUM (1 << 4) /* 0x10: Checksum validation failed */
#define ERR_FLAG_STALL (1 << 5)    /* 0x20: Overload / Stalled rotor */
#define ERR_FLAG_PARSE                                                         \
  (1 << 6) /* 0x40: Packet format / unknown instruction */

/* Flags that survive the start of a new packet (bits 3, 4, 6 are transient and
 * cleared) */
#define ERR_FLAGS_KEEP_ON_NEW_PACKET                                           \
  ((uint8_t)~(ERR_FLAG_RANGE_LOCK | ERR_FLAG_CHECKSUM | ERR_FLAG_PARSE))

/* Status Return Level (register 0x10) */
#define STATUS_RETURN_NEVER 0     /* PING only                       */
#define STATUS_RETURN_READ_ONLY 1 /* PING and READ                   */
#define STATUS_RETURN_ALL 2       /* every instruction               */

/* g_reboot_flag values */
#define REBOOT_FLAG_NONE 0xFF
#define REBOOT_FLAG_REQUEST 0x5A

/* Control Table Register Addresses (EEPROM Area: 0x00 ~ 0x17) */
#define REG_MODEL_NUM_L 0x00
#define REG_MODEL_NUM_H 0x01
#define REG_FW_VER 0x02
#define REG_MOTOR_ID 0x03
#define REG_BAUD_RATE 0x04
#define REG_RETURN_DELAY 0x05
#define REG_MIN_ANGLE_L 0x06
#define REG_MIN_ANGLE_H 0x07
#define REG_MAX_ANGLE_L 0x08
#define REG_MAX_ANGLE_H 0x09
#define REG_RESERVED_0A 0x0A
#define REG_TEMP_LIMIT 0x0B
#define REG_VOLT_LOW_LIMIT 0x0C
#define REG_VOLT_UPP_LIMIT 0x0D
#define REG_MAX_TORQUE_L 0x0E
#define REG_MAX_TORQUE_H 0x0F
#define REG_STATUS_RETURN_LEVEL 0x10
#define REG_RESERVED_11 0x11
#define REG_ALARM_TORQUE_DISABLE 0x12

/* Control Table Register Addresses (RAM Area: 0x18 ~ 0x31) */
#define REG_TORQUE_ENABLE 0x18
#define REG_CALIBRATION_ENABLE 0x19

/* Unified 10-byte Control Block (0x1A ~ 0x23):
 *   0x1A..0x1B : kp            (uint16_t, 1 LSB = 0.001 N.m/rad at output shaft, MIT mode)
 *   0x1C..0x1D : kd            (uint16_t, 1 LSB = 0.001 N.m.s/rad at output shaft, MIT mode)
 *   0x1E..0x1F : goal_position (int16_t,  signed multi-turn counts, Position & MIT mode)
 *   0x20..0x21 : moving_speed  (int16_t,  RPM, Velocity, Position limit & MIT mode)
 *   0x22..0x23 : tau_ff        (int16_t,  1 LSB = 0.001 N.m [mN.m] at output shaft, MIT feedforward torque)
 */
#define REG_KP_L 0x1A
#define REG_KP_H 0x1B
#define REG_KD_L 0x1C
#define REG_KD_H 0x1D
#define REG_GOAL_POSITION_L 0x1E
#define REG_GOAL_POSITION_H 0x1F
#define REG_MOVING_SPEED_L 0x20
#define REG_MOVING_SPEED_H 0x21
#define REG_TAU_FF_L 0x22
#define REG_TAU_FF_H 0x23

/* Read-only Feedback Registers (0x24 ~ 0x2C, 0x2E) & RAM Control Mode (0x2D) */
#define REG_CURRENT_POSITION_L 0x24
#define REG_CURRENT_POSITION_H 0x25
#define REG_CURRENT_SPEED_L 0x26
#define REG_CURRENT_SPEED_H 0x27
#define REG_CURRENT_DRAW_L 0x28
#define REG_CURRENT_DRAW_H 0x29
#define REG_CURRENT_VOLTAGE 0x2A
#define REG_CURRENT_TEMPERATURE 0x2B
#define REG_REGISTERED_FLAG 0x2C
#define REG_CONTROL_MODE 0x2D /* RW: 2=Position, 3=MIT (default 3) */
#define REG_MOVING_FLAG 0x2E
#define REG_LOCK_EEPROM 0x2F
#define REG_PUNCH_L 0x30
#define REG_PUNCH_H 0x31
#define REG_KP_MAX_L 0x32 /* R: Max allowable Kp [0.001 N.m/rad]   */
#define REG_KP_MAX_H 0x33
#define REG_KD_MAX_L 0x34 /* R: Max allowable Kd [0.001 N.m.s/rad] */
#define REG_KD_MAX_H 0x35

#define CONTROL_TABLE_SIZE 54

/* Control table areas (register addresses) */
#define CT_ADDR_WRITABLE_START REG_MOTOR_ID      /* 0x00..0x02 are read-only */
#define CT_ADDR_EEPROM_END REG_TORQUE_ENABLE     /* < 0x18 are persistent    */
#define CT_ADDR_RO_RAM_START REG_CURRENT_POSITION_L /* 0x24..0x2E (except 0x2D) */
#define CT_ADDR_RO_RAM_END REG_MOVING_FLAG
#define CT_ADDR_HIDDEN_START 0x32 /* hidden block: hw_tag, calib_done */

/* Position resolution: 16384 counts (14-bit) per output-shaft revolution (2*pi rad) */
#define POSITION_COUNTS_PER_REV 16384
#define POSITION_SINGLE_TURN_MAX (POSITION_COUNTS_PER_REV - 1)
#define POSITION_MIN (-32768)
#define POSITION_MAX 32767

/* Factory defaults, EEPROM area */
#define DEFAULT_MODEL_NUMBER 0x000C
#define DEFAULT_FIRMWARE_VERSION 0x1C
#define DEFAULT_MOTOR_ID 0x01
#define DEFAULT_BAUD_RATE 1
#define DEFAULT_RETURN_DELAY_TIME 0xFA
#define DEFAULT_MIN_ANGLE_LIMIT POSITION_MIN
#define DEFAULT_MAX_ANGLE_LIMIT POSITION_MAX
#define DEFAULT_TEMP_UPPER_LIMIT 0x46    /* 70 degC */
#define DEFAULT_VOLTAGE_LOWER_LIMIT 0x3C /* 6.0 V   */
#define DEFAULT_VOLTAGE_UPPER_LIMIT 0xA0 /* 16.0 V  */
#define DEFAULT_MAX_TORQUE 1023
#define DEFAULT_STATUS_RETURN_LEVEL STATUS_RETURN_ALL
#define DEFAULT_ALARM_TORQUE_DISABLE 0x24

/* Factory defaults, RAM area */
#define DEFAULT_CONTROL_MODE 3 /* 3 = MIT Impedance mode on startup */
#define DEFAULT_KP MOTOR_DEFAULT_KP /* Per-joint default Kp (see foc_config.h) */
#define DEFAULT_KD MOTOR_DEFAULT_KD /* Per-joint default Kd (see foc_config.h) */
#define DEFAULT_PUNCH 0x20

/* Complete Control Table Memory Structure (128-byte Flash/RAM image) */
#pragma pack(push, 1)
typedef struct {
  /* --- EEPROM Area (Address 0x00 ~ 0x17) --- */
  uint16_t model_number;         /* 0x00 ~ 0x01: Default 12 (0x000C) */
  uint8_t firmware_version;      /* 0x02: Default 28 (0x1C) */
  uint8_t motor_id;              /* 0x03: Default 1 (0x01) */
  uint8_t baud_rate;             /* 0x04: Default 1 (1Mbps) */
  uint8_t return_delay_time;     /* 0x05: Default 250 (500us) */
  int16_t min_angle_limit;       /* 0x06 ~ 0x07: Default -32768 */
  int16_t max_angle_limit;       /* 0x08 ~ 0x09: Default +32767 */
  uint8_t reserved_0a;           /* 0x0A: 0 */
  uint8_t temp_upper_limit;      /* 0x0B: Default 70 degC (0x46) */
  uint8_t voltage_lower_limit;   /* 0x0C: Default 60 (6.0V) */
  uint8_t voltage_upper_limit;   /* 0x0D: Default 160 (16.0V) */
  uint16_t max_torque;           /* 0x0E ~ 0x0F: Default 1023 (0x03FF) */
  uint8_t status_return_level;   /* 0x10: Default 2 */
  uint8_t reserved_11;           /* 0x11: Reserved (0) */
  uint8_t alarm_torque_disable;  /* 0x12: Default 0x24 */
  uint8_t reserved_eeprom[5];    /* 0x13 ~ 0x17 */

  /* --- RAM Area (Address 0x18 ~ 0x35) --- */
  uint8_t torque_enable;       /* 0x18: 0=free, 1=engage */
  uint8_t calibration_enable;  /* 0x19: 1=trigger cal */
  uint16_t kp;                 /* 0x1A ~ 0x1B: Stiffness gain [x1000 N.m/rad at output shaft] */
  uint16_t kd;                 /* 0x1C ~ 0x1D: Damping gain [x1000 N.m.s/rad at output shaft] */
  int16_t goal_position;       /* 0x1E ~ 0x1F: Target position (-32768..+32767) */
  int16_t moving_speed;        /* 0x20 ~ 0x21: Target speed / speed limit (RPM) */
  int16_t tau_ff;              /* 0x22 ~ 0x23: Feedforward torque [x1000 N.m (mN.m) at output shaft] */
  int16_t current_position;    /* 0x24 ~ 0x25: Measured output position */
  int16_t current_speed;       /* 0x26 ~ 0x27: Measured velocity */
  int16_t current_draw;        /* 0x28 ~ 0x29: Measured current in mA */
  uint8_t current_voltage;     /* 0x2A: Unit 0.1V */
  uint8_t current_temperature; /* 0x2B: Unit 1 degC */
  uint8_t registered_flag;     /* 0x2C: 1 when Reg Write queued */
  uint8_t control_mode;        /* 0x2D: Active mode (2=Pos, 3=MIT) */
  uint8_t moving_flag;         /* 0x2E: 1 if moving in Position mode */
  uint8_t lock_eeprom;         /* 0x2F: 1 to lock EEPROM */
  uint16_t punch;              /* 0x30 ~ 0x31: Kickstart current */
  uint16_t kp_max;             /* 0x32 ~ 0x33: Read-only max Kp [x1000 N.m/rad] */
  uint16_t kd_max;             /* 0x34 ~ 0x35: Read-only max Kd [x1000 N.m.s/rad] */

  /* --- Hidden part of the 128-byte EEPROM block (0x36 ~ 0x7F) --- */
  uint8_t hw_tag;                   /* 0x36: 0x43 */
  uint8_t calib_done;               /* 0x37: 1 = calibration valid */
  uint8_t reserved_38[4];           /* 0x38 ~ 0x3B */
  uint16_t angle_offset;            /* 0x3C ~ 0x3D: rotor zero (raw 14-bit) */
  int16_t offset_v;                 /* 0x3E ~ 0x3F: phase V ADC zero */
  int16_t offset_u;                 /* 0x40 ~ 0x41: phase U ADC zero */
  int16_t offset_total;             /* 0x42 ~ 0x43: total-current ADC zero */
#if MOTOR_USE_HALL_MT9102
  uint16_t hall_offset_sin;          /* 0x44 ~ 0x45: MT9102 Sin midpoint offset */
  uint16_t hall_offset_cos;          /* 0x46 ~ 0x47: MT9102 Cos midpoint offset */
  uint16_t hall_gain_sin_num;        /* 0x48 ~ 0x49: Gain ratio numerator */
  uint16_t hall_gain_sin_den;        /* 0x4A ~ 0x4B: Gain ratio denominator */
  uint8_t  hall_calib_done;          /* 0x4C: 1 if Hall calibration valid */
  uint8_t  reserved_4d[0x78 - 0x4D]; /* 0x4D ~ 0x77 */
#else
  uint8_t reserved_44[0x78 - 0x44]; /* 0x44 ~ 0x77 */
#endif
  uint32_t save_counter;            /* 0x78 ~ 0x7B: flash block sequence number */
  uint8_t reserved_7c;              /* 0x7C */
  uint8_t reboot_copy;              /* 0x7D */
  uint8_t block_checksum;           /* 0x7E: ~sum(bytes 0..0x7D) */
  uint8_t block_marker;             /* 0x7F: 0x4B = valid block */
} control_table_t;
#pragma pack(pop)

#define EEPROM_BLOCK_SIZE 128
#define EEPROM_BLOCK_MARKER 0x4B

extern control_table_t g_control_table;
extern volatile uint8_t g_motor_status_flags;
extern volatile uint8_t g_reboot_flag;
extern volatile uint8_t g_eeprom_dirty_flag;
extern volatile uint8_t g_eeprom_lock_flag;
extern volatile uint8_t g_last_id;

void control_table_load_eeprom_defaults(void);
void control_table_load_ram_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TABLE_H */
