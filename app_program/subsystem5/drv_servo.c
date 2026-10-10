/*
 *  drv_servo.c
 *
 *  Aims the ultrasonic sensor left/right, the same way you'd swivel a
 *  security camera to look around instead of only straight ahead. See
 *  docs/buddy5-scan-avoidance/buddy5-scan-avoidance.md for the security-camera
 * analogy and
 *  the full walkthrough of how this fits into a scan.
 */
#include "rc_prelude.h"
#include "drv_servo.h"
#include "rc_config.h"
#include "rc_pwm.h"

/* Standard hobby-servo PWM frame rate: the signal pin is pulsed once
 * every 1/50 second (20 ms). This is fixed by the servo's own design,
 * not something we get to tune. */
#define SERVO_HZ            (50UL)

/* The pulse width (how long the pin stays "on" within each 20 ms frame)
 * that a hobby servo reads as "go to one extreme" (500 us) and "go to
 * the other extreme" (2400 us). These numbers come from the servo's own
 * datasheet/convention, not from this codebase — most hobby servos use
 * something close to this range. Everything in between is linearly
 * interpolated to an angle (see drv_servo_set_angle below). */
#define PULSE_MIN_US        (500UL)
#define PULSE_MAX_US        (2400UL)

/* Total degrees of travel the servo bracket covers (180 - 0 = 180). Used
 * as the divisor below to turn "degrees requested" into "fraction of the
 * full pulse-width range." */
#define SWEEP_DEG           (RC_SERVO_ANGLE_MAX - RC_SERVO_ANGLE_MIN)

/* Typical 9 g servo is around 0.12 s per 60 degrees unloaded. Add a
 * fixed allowance for the mechanism to stop ringing. Measure yours and
 * shrink these, they dominate total scan time.
 *
 * US_PER_DEG: microseconds of travel time estimated per degree of
 * commanded movement (used, divided by 1000, as milliseconds-per-degree
 * in drv_servo_settle_ms below). This is a rough guess for a generic 9g
 * servo — the Buddy 5 guide, "How to get started" step 3 asks you to
 * time your actual servo and correct this constant, since it directly
 * controls how long every scan step waits before trusting a reading.
 * SETTLE_FIXED_MS: a flat number of milliseconds added on top of travel
 * time, to let the mechanical arm stop wobbling ("ringing," like a
 * diving board settling) after it stops moving, even for a very short
 * hop where travel time alone would be almost zero. */
#define US_PER_DEG          (2200UL)
#define SETTLE_FIXED_MS     (25UL)

/* Remembers the last angle we told the servo to go to. There is no
 * feedback sensor on a cheap hobby servo, so this is our only record of
 * "where the sensor is currently pointed" — it is a command history, not
 * a measurement. `static` here means this variable is private to this
 * .c file (see the note on `static` in sub_line.c for the general idea). */
static int16_t last_angle = 90;

/* Sets up 50 Hz PWM on the servo's signal pin and centers the sensor at
 * 90 degrees (straight ahead). Call once at boot, after the bracket has
 * been mechanically aligned per the Buddy 5 guide, "How to get started"
 * step 2. */
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

    /* Clamp to the bracket's physical travel limits so a caller can never
     * accidentally command an out-of-range angle that could jam the
     * mechanism or read garbage past the servo's real limits. */
    if (deg < RC_SERVO_ANGLE_MIN) {
        deg = RC_SERVO_ANGLE_MIN;
    }
    if (deg > RC_SERVO_ANGLE_MAX) {
        deg = RC_SERVO_ANGLE_MAX;
    }

    /* Linear interpolation: 0 deg -> PULSE_MIN_US, 180 deg -> PULSE_MAX_US,
     * everything else proportional in between. All integer arithmetic
     * (multiply before divide, to keep as much precision as possible
     * without ever using a fractional/float type — see TEAM_GUIDE.md §5,
     * \"No floating point anywhere\"). */
    pulse_us = PULSE_MIN_US
             + (((PULSE_MAX_US - PULSE_MIN_US) * (uint32_t)deg) / SWEEP_DEG);

    last_angle = deg;

    /* Actually program the PWM hardware to emit that pulse width every
     * 20 ms frame from now on. */
    return rc_pwm_set_pulse_us(RC_PIN_SERVO_SCAN, pulse_us);
}

/* Returns whatever angle was last commanded via drv_servo_set_angle —
 * again, not a real-time measurement, just "what we asked for last." */
int16_t drv_servo_get_angle(void)
{
    return last_angle;
}

/* Estimates how many milliseconds to wait after commanding a move from
 * `from` degrees to `to` degrees before the arm has both arrived and
 * stopped physically wobbling — i.e. before a range reading taken at the
 * new angle can be trusted. sub_scan.c calls this before every ping so
 * it waits exactly as long as needed and no longer (see step_to() in
 * sub_scan.c). */
uint32_t drv_servo_settle_ms(int16_t from, int16_t to)
{
    /* How far the servo has to travel, in degrees. Computed in a wider
     * signed 32-bit type so subtracting two int16_t values can't
     * misbehave near their limits, then made positive since we only care
     * about the distance travelled, not the direction. */
    int32_t  travel = (int32_t)to - (int32_t)from;
    uint32_t mag;

    if (travel < 0) {
        travel = -travel;
    }
    /* Travel time = degrees * (microseconds per degree), converted to
     * whole milliseconds by dividing by 1000. */
    mag = ((uint32_t)travel * US_PER_DEG) / 1000UL;

    /* Add the fixed "let it stop wobbling" allowance on top. */
    return mag + SETTLE_FIXED_MS;
}

/* Stops sending PWM pulses entirely, letting the servo motor go limp
 * (no holding torque) and stop drawing current. Useful because the servo
 * and the drive motors share one battery — a servo move during a sudden
 * motor current spike (e.g. starting from a stop) can brown out the
 * Pico's power supply. */
rc_result_t drv_servo_release(void)
{
    return rc_pwm_enable(RC_PIN_SERVO_SCAN, false);
}
