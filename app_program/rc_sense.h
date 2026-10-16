/*
 *  rc_sense.h  -  the shared Sense task (shared; integration owner Buddy 5)
 *
 *  Report §3, design decision "single Sense task": ONE periodic task
 *  polls every sensor that has to be polled, so there is one stack and
 *  one wake-up per period instead of one per sensor:
 *
 *    every RC_PERIOD_LINE_MS (5 ms)  drv_ir_sample_line()  S3, IR line pair
 *    every RC_PERIOD_IMU_MS  (10 ms) drv_imu_sample()      S4, IMU over I2C
 *
 *  Each driver publishes its own event (RC_EVT_LINE_SAMPLE,
 *  RC_EVT_IMU_SAMPLE); this task only decides WHEN to sample. It lives
 *  here, beside sub_nav, because it belongs to no single subsystem: S3
 *  and S4 own their drivers, nobody else calls them.
 *
 *  Sensors that are interrupt-driven are NOT polled here: the encoders
 *  (S2) and the ultrasonic echo (S5, paced by the Scan task).
 */
#ifndef RC_SENSE_H
#define RC_SENSE_H

#include "rc_types.h"

/* Create and start the Sense task at RC_PRI_SENSE. Call once from
 * usermain(), after every subsystemN_init() (the drivers it polls must
 * exist). Safe to call again: a second call does nothing and returns
 * RC_OK. Returns RC_ERR_HARDWARE if the kernel refuses the task. */
rc_result_t rc_sense_start(void);

#if APP_TEST
/* TEST BUILD ONLY: hold = true makes the Sense task skip its line and IMU
 * samples (it keeps waking on schedule, it just publishes nothing), so a
 * self-test can inject fake RC_EVT_LINE_SAMPLE / RC_EVT_IMU_SAMPLE events
 * without the real ones overwriting them. Interrupts are not affected:
 * the encoder, echo and barcode ISRs and their bottom halves keep running.
 * Always release it (false) at the end of the test. */
void rc_sense_test_hold(bool hold);
#endif

#endif /* RC_SENSE_H */
