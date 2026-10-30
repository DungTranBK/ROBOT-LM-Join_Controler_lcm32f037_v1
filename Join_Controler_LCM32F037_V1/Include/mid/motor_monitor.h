/**
 * @file motor_monitor.h
 * @brief [Middleware] 1 ms housekeeping: bus voltage, temperature, total current, protections,
 *        and publication of position/speed into the control table.
 *        Matching code_da_doc.bin 0x08004370.
 */

#ifndef __MOTOR_MONITOR_H
#define __MOTOR_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** Call from the main loop; the measurement part runs once per elapsed 1 ms tick. */
void motor_monitor_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_MONITOR_H */
