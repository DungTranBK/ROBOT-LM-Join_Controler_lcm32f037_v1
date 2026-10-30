/**
 * @file control_table.c
 * @brief [Middleware] Control Table storage + factory defaults
 *          0x08004740  EEPROM-area defaults        0x08004788  RAM-area defaults
 */

#include "mid/control_table.h"
#include "mid/foc_config.h"

/* Control table (original @0x2000068E). Filled by eeprom_store_init() (0x080041C8). */
control_table_t g_control_table;

volatile uint8_t g_motor_status_flags = 0;  /* 0x2000006E */
volatile uint8_t g_reboot_flag = REBOOT_FLAG_NONE;  /* 0x20000064 (initial value 0xFF) */
volatile uint8_t g_eeprom_dirty_flag = 0;   /* 0x2000007A */
volatile uint8_t g_eeprom_lock_flag = 0;    /* 0x2000007C */
volatile uint8_t g_last_id = 0;             /* 0x2000007E */

/* 0x08004740 */
void control_table_load_eeprom_defaults(void) {
    control_table_t *t = &g_control_table;

    t->model_number          = DEFAULT_MODEL_NUMBER;
    t->firmware_version      = DEFAULT_FIRMWARE_VERSION;
    t->motor_id              = DEFAULT_MOTOR_ID;
    t->baud_rate             = DEFAULT_BAUD_RATE;
    t->return_delay_time     = DEFAULT_RETURN_DELAY_TIME;
    t->min_angle_limit       = DEFAULT_MIN_ANGLE_LIMIT;
    t->max_angle_limit       = DEFAULT_MAX_ANGLE_LIMIT;
    t->temp_upper_limit      = DEFAULT_TEMP_UPPER_LIMIT;
    t->voltage_lower_limit   = DEFAULT_VOLTAGE_LOWER_LIMIT;
    t->voltage_upper_limit   = DEFAULT_VOLTAGE_UPPER_LIMIT;
    t->max_torque            = DEFAULT_MAX_TORQUE;
    t->status_return_level   = DEFAULT_STATUS_RETURN_LEVEL;
    t->reserved_11           = 0;
    t->alarm_torque_disable  = DEFAULT_ALARM_TORQUE_DISABLE;
}

/* 0x08004788 : runs after every load (RAM area is never persistent) */
void control_table_load_ram_defaults(void) {
    control_table_t *t = &g_control_table;

#if MOTOR_HAS_GEARBOX
    /* Geared motor: MCU is blind to absolute output angle at power-on;
     * always start with full range in RAM until Master sets session limits. */
    t->min_angle_limit   = DEFAULT_MIN_ANGLE_LIMIT;
    t->max_angle_limit   = DEFAULT_MAX_ANGLE_LIMIT;
#endif
    t->torque_enable      = 0;
    t->calibration_enable = 0;
    t->kp                 = DEFAULT_KP;
    t->kd                 = DEFAULT_KD;
    t->goal_position      = t->current_position;
    t->moving_speed       = 0;
    t->tau_ff             = 0;
    t->registered_flag    = 0;
    t->control_mode       = DEFAULT_CONTROL_MODE;
    t->moving_flag        = 0;
    t->lock_eeprom        = 0;
    t->punch              = DEFAULT_PUNCH;
    t->kp_max             = MOTOR_KP_MAX;
    t->kd_max             = MOTOR_KD_MAX;
}
