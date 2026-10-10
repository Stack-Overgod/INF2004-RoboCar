/*
 *  drv_motor.h  -  Buddy 2 hardware layer
 *
 *  Robo Pico drives each motor with two PWM pins rather than a PWM pin
 *  plus a direction bit. Forward is PWM on MxA with MxB held low, reverse
 *  swaps them, brake is both high, coast is both low.
 *
 *  This layer is pure output. It has no idea how fast the car is actually
 *  going. Closing that loop is sub_motion's job.
 *
 *  Beginner notes (see TEAM_GUIDE.md §1 for the bigger picture):
 *  - "PWM" = Pulse Width Modulation. The pin is flipped on/off thousands
 *    of times a second; the *fraction* of time it's on ("duty cycle")
 *    controls how much average power the motor sees. 100% duty = full
 *    power, 0% = off. There's no analogue voltage knob on this chip, so
 *    this on/off flicker trick is how "half speed" is created digitally.
 *  - "permille" = parts per thousand, this codebase's stand-in for a
 *    percentage with two extra digits of precision (avoids needing
 *    fractional numbers, which this chip can't do fast - see §2 in
 *    TEAM_GUIDE.md, "No floating point anywhere").
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "rc_types.h"

/* An "enum" (enumeration) is just a named list of integer constants -
 * here, the two ways a motor can be told to stop. Using the names
 * RC_MOTOR_COAST / RC_MOTOR_BRAKE instead of raw 0/1 makes call sites
 * self-explanatory. */
typedef enum {
    RC_MOTOR_COAST = 0,  /* power cut, wheel free-spins down on its own */
    /* both pins driven high, shorts the motor, stops fast */
    RC_MOTOR_BRAKE
} rc_motor_stop_t;

/* Call once at boot. Configures PWM on both motors' pin pairs and zeroes
 * them so the car doesn't move the instant power is applied. */
rc_result_t drv_motor_init(void);

/* Signed duty, -1000..+1000 permille (i.e. -100.0% .. +100.0%). Positive
 * is forward, negative is reverse. Returns immediately, no ramping -
 * commanding a big jump in duty snaps straight there; add ramping in
 * sub_motion if you want smoother acceleration. */
rc_result_t drv_motor_set(rc_side_t side, int16_t duty_permille);

/* Convenience: set both wheels' duty in a single call. Used constantly by
 * sub_motion since almost every move commands both wheels together. */
rc_result_t drv_motor_set_pair(int16_t left_permille, int16_t right_permille);

/* Stop both wheels, either coasting or braking - see rc_motor_stop_t above. */
rc_result_t drv_motor_stop(rc_motor_stop_t mode);

/* Last commanded value, for telemetry. (Wheel direction is measured by
 * the encoder's B channel now - see drv_encoder.h - so this is no longer
 * used to infer it.) */
int16_t drv_motor_get(rc_side_t side);

#endif /* DRV_MOTOR_H */
