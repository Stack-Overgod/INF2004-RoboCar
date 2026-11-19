/*
 *  drv_servo.c
 *
 *  Aims the ultrasonic sensor left/right with the scan servo on
 *  RC_PIN_SERVO_SCAN. See docs/buddy5-scan-avoidance/buddy5-scan-avoidance.md
 *  for how this fits into a scan.
 */
#include "../rc_prelude.h"
#include "drv_servo.h"
#include "../rc_config.h"
#include "../rc_pwm.h"

/* Standard hobby-servo PWM frame rate: one pulse every 20 ms. */
#define SERVO_HZ            (50UL)

/* The pulse width (how long the pin stays "on" within each 20 ms frame)
 * sent at each end of our limited swing: PULSE_MIN_US at
 * RC_SERVO_ANGLE_MIN (70 deg) and PULSE_MAX_US at RC_SERVO_ANGLE_MAX
 * (110 deg), centred on PULSE_CENTRE_US for 90 deg (straight ahead). A
 * hobby servo centres at about 1500 us and turns ~10.6 us per degree, so
 * the ±20 deg swing is ±211 us. Everything in between is linearly
 * interpolated to an angle (see drv_servo_set_angle below).
 *
 * Calibrate on the bench: if the sensor does not point straight ahead at
 * rest, move PULSE_CENTRE_US (~10.6 us per degree; the direction depends
 * on how the servo is mounted). If the servo hits its stop (buzzes)
 * before 70/110, make PULSE_HALF_US smaller. */
#define PULSE_CENTRE_US     (1500UL)
#define PULSE_HALF_US       (211UL)
#define PULSE_MIN_US        (PULSE_CENTRE_US - PULSE_HALF_US)
#define PULSE_MAX_US        (PULSE_CENTRE_US + PULSE_HALF_US)

/* Total degrees of travel the servo bracket covers (110 - 70 = 40). Used
 * as the divisor below to turn "degrees requested" into "fraction of the
 * pulse-width range." */
#define SWEEP_DEG           (RC_SERVO_ANGLE_MAX - RC_SERVO_ANGLE_MIN)

/* Settle time estimate for drv_servo_settle_ms. A typical 9 g servo is
 * around 0.12 s per 60 degrees unloaded.
 *
 * US_PER_DEG: microseconds of travel time per degree of commanded
 * movement (a rough figure for a generic 9 g servo; time the real servo
 * to refine it, since it sets how long every scan step waits).
 * SETTLE_FIXED_MS: milliseconds added on top of travel time for the arm
 * to stop ringing, even for a very short hop. */
#define US_PER_DEG          (2200UL)
#define SETTLE_FIXED_MS     (25UL)

/* Last commanded angle. A hobby servo has no position feedback, so this
 * is a command record, not a measurement. */
static int16_t last_angle = 90;

/* Sets up 50 Hz PWM on the servo's signal pin and centres the sensor at
 * 90 degrees (straight ahead). Call once at boot. */
rc_result_t drv_servo_init(void)
{
    rc_result_t res = rc_pwm_init_pin(RC_PIN_SERVO_SCAN, SERVO_HZ);

    if (res != RC_OK) {
        return res;
    }
    return drv_servo_set_angle(90);
}

/* Moves the servo to `deg` degrees by converting the angle into a pulse
 * width and handing that to the PWM driver. Returns immediately — it
 * does not wait for the arm to physically arrive (see
 * drv_servo_settle_ms for how a caller knows when it's safe to trust a
 * new reading). */
rc_result_t drv_servo_set_angle(int16_t deg)
{
    uint32_t pulse_us;

    /* clamp to the bracket's travel limits */
    if (deg < RC_SERVO_ANGLE_MIN) {
        deg = RC_SERVO_ANGLE_MIN;
    }
    if (deg > RC_SERVO_ANGLE_MAX) {
        deg = RC_SERVO_ANGLE_MAX;
    }

    /* Linear interpolation: RC_SERVO_ANGLE_MIN -> PULSE_MIN_US,
     * RC_SERVO_ANGLE_MAX -> PULSE_MAX_US, everything else proportional in
     * between. The angle is measured from RC_SERVO_ANGLE_MIN (not from 0),
     * since the swing does not start at 0 deg. Integer arithmetic only,
     * multiply before divide to keep precision. */
    pulse_us = PULSE_MIN_US
             + (((PULSE_MAX_US - PULSE_MIN_US)
                 * (uint32_t)(deg - RC_SERVO_ANGLE_MIN)) / SWEEP_DEG);

    last_angle = deg;

    return rc_pwm_set_pulse_us(RC_PIN_SERVO_SCAN, pulse_us);
}

/* Returns the angle last commanded via drv_servo_set_angle (not a
 * measurement). */
int16_t drv_servo_get_angle(void)
{
    return last_angle;
}

/* Estimates how many milliseconds to wait after commanding a move from
 * `from` degrees to `to` degrees before the arm has arrived and stopped
 * ringing, i.e. before a range reading at the new angle can be trusted
 * (sub_scan.c step_to()). */
uint32_t drv_servo_settle_ms(int16_t from, int16_t to)
{
    /* travel in degrees, in 32 bits so the int16_t difference cannot
     * overflow; only the magnitude matters */
    int32_t  travel = (int32_t)to - (int32_t)from;
    uint32_t mag;

    if (travel < 0) {
        travel = -travel;
    }
    /* travel time in whole milliseconds */
    mag = ((uint32_t)travel * US_PER_DEG) / 1000UL;

    return mag + SETTLE_FIXED_MS;
}
