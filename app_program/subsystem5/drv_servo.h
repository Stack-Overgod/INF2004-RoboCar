/*
 *  drv_servo.h  -  Buddy 5 hardware layer
 *
 *  Standard hobby servo on a Robo Pico servo port: 50 Hz frame, pulse
 *  width from about 500 us to 2400 us.
 *
 *  There is no position feedback on an RC servo, so "has it arrived" is
 *  a timer, not a measurement. drv_servo_settle_ms tells the caller how
 *  long to wait before trusting a range reading taken at the new angle.
 *  The caller waits by arming its own state machine, never by spinning.
 */
#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "rc_types.h"

/* The physical limits of the servo bracket's swing, in degrees. 0 is
 * fully one side, 180 fully the other, 90 is straight ahead (see
 * the Buddy 5 guide for how "straight ahead" is set mechanically).
 * Every angle passed into this driver gets clamped to this range. */
#define RC_SERVO_ANGLE_MIN  (0)
#define RC_SERVO_ANGLE_MAX  (180)

/* Sets up the PWM (Pulse Width Modulation — a way to encode a value, here
 * an angle, as how long a repeating electrical pulse stays "on") signal
 * on the servo pin and centers the sensor at 90 degrees (straight ahead).
 * Call once at boot. */
rc_result_t drv_servo_init(void);

/* Command an angle in degrees. Returns immediately (does NOT wait for the
 * servo arm to physically get there — see drv_servo_settle_ms below for
 * how the caller knows when it's safe to trust a reading). */
rc_result_t drv_servo_set_angle(int16_t deg);

/* Last commanded angle. NOTE: this is not a real sensor reading — cheap
 * hobby servos have no position feedback, so this is just "what we last
 * told it to do," which may not be exactly where it physically is. */
int16_t drv_servo_get_angle(void);

/* Milliseconds the servo needs to travel from `from` to `to` and stop
 * ringing (mechanically wobbling back and forth after a fast move, like
 * a diving board settling). Used by sub_scan to schedule the ping after
 * the move, since there's no feedback to say "arrived." */
uint32_t drv_servo_settle_ms(int16_t from, int16_t to);

/* Stop sending pulses. The servo goes limp and stops drawing current,
 * which matters on a shared battery when the motors are also loaded. */
rc_result_t drv_servo_release(void);

#endif /* DRV_SERVO_H */
