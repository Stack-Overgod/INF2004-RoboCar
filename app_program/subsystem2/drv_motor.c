/*
 *  drv_motor.c
 *
 *  Turns permille duty commands into actual PWM signals on the motor
 *  driver pins. This file does NOT know how fast the wheel is turning -
 *  it just obeys "spin this hard, this direction." See sub_motion.c for
 *  the code that decides *what* duty to send based on target speed.
 */
#include "rc_prelude.h"
#include "drv_motor.h"
#include "rc_config.h"
#include "rc_pwm.h"

/* PWM switching frequency in Hz. 20 kHz is above human hearing (so the
 * motor doesn't whine) and well inside what the Robo Pico output stage
 * handles. Drop to 10 kHz if your motors whine at low duty. */
#define MOTOR_PWM_HZ    (20000UL)

/* Largest legal duty value in "permille" (parts per thousand -> 1000 =
 * 100.0% power). Used to clamp any duty command that overshoots. */
#define DUTY_MAX        (1000)

/* One motor's wiring + last-known state.
 * pin_a / pin_b: the two PWM-capable GPIO pins that drive this motor
 *   (see drv_motor.h - forward drives pin_a, reverse drives pin_b).
 * last: the most recently commanded signed duty (-1000..1000), kept so
 *   drv_motor_get() can report it to telemetry.
 */
typedef struct {
    uint32_t pin_a;
    uint32_t pin_b;
    int16_t  last;
} motor_t;

/* Fixed array of the two motors, indexed by rc_side_t (RC_SIDE_LEFT=0,
 * RC_SIDE_RIGHT=1). "static" here means this array is private to this
 * file - no other .c file can see or touch it directly; they must go
 * through the functions below. Pin numbers come from rc_config.h, the
 * single place all GPIO numbers live in this project. */
static motor_t motors[2] = {
    { RC_PIN_MOTOR_L_A, RC_PIN_MOTOR_L_B, 0 },
    { RC_PIN_MOTOR_R_A, RC_PIN_MOTOR_R_B, 0 }
};

/* Set up PWM hardware on both motors' pin pairs and make sure both start
 * at zero duty (motors off) so the car doesn't lurch the moment power is
 * applied. Call once at boot. */
rc_result_t drv_motor_init(void)
{
    uint32_t    i;
    rc_result_t res;

    for (i = 0U; i < 2U; i++) {
        res = rc_pwm_init_pin(motors[i].pin_a, MOTOR_PWM_HZ);
        if (res != RC_OK) {
            return res;
        }
        res = rc_pwm_init_pin(motors[i].pin_b, MOTOR_PWM_HZ);
        if (res != RC_OK) {
            return res;
        }
        (void)rc_pwm_set_duty(motors[i].pin_a, 0U);
        (void)rc_pwm_set_duty(motors[i].pin_b, 0U);
        motors[i].last = 0;
    }
    return RC_OK;
}

/* Command one wheel's power. duty_permille ranges -1000..+1000
 * (i.e. -100.0%..+100.0%): positive drives forward, negative reverse,
 * zero stops (coasts) that wheel. This is the lowest-level "make the
 * wheel spin" call in the whole project - everything else (PID output in
 * sub_motion.c, line-follower steering) eventually funnels through here. */
rc_result_t drv_motor_set(rc_side_t side, int16_t duty_permille)
{
    motor_t *m;

    if (side > RC_SIDE_RIGHT) {
        return RC_ERR_PARAM;
    }
    /* Clamp: never let a caller accidentally command more than 100%
     * power in either direction. */
    if (duty_permille > DUTY_MAX) {
        duty_permille = DUTY_MAX;
    }
    if (duty_permille < -DUTY_MAX) {
        duty_permille = -DUTY_MAX;
    }

    m = &motors[side];
    m->last = duty_permille;

    /* Drop the opposite pin before raising the driving one, so the H
     * bridge (the chip inside the motor driver that lets current flow
     * either direction through the motor) is never told to do both at
     * once - that would be a short circuit / brake fight. */
    if (duty_permille >= 0) {
        (void)rc_pwm_set_duty(m->pin_b, 0U);
        (void)rc_pwm_set_duty(m->pin_a, (uint16_t)duty_permille);
    } else {
        (void)rc_pwm_set_duty(m->pin_a, 0U);
        (void)rc_pwm_set_duty(m->pin_b, (uint16_t)(-duty_permille));
    }
    return RC_OK;
}

/* Convenience wrapper: command both wheels in one call instead of two
 * separate drv_motor_set() calls. Used everywhere sub_motion.c needs to
 * update both wheels together (which is almost always). */
rc_result_t drv_motor_set_pair(int16_t left_permille, int16_t right_permille)
{
    (void)drv_motor_set(RC_SIDE_LEFT, left_permille);
    (void)drv_motor_set(RC_SIDE_RIGHT, right_permille);
    return RC_OK;
}

/* Stop both wheels. RC_MOTOR_COAST cuts power so the wheel free-spins to
 * a stop under its own friction; RC_MOTOR_BRAKE drives both pins high,
 * which shorts the motor windings together and stops it quickly (electrical
 * braking, not a physical brake pad). */
rc_result_t drv_motor_stop(rc_motor_stop_t mode)
{
    uint32_t i;
    /* Ternary operator: level = DUTY_MAX if braking, else 0. Reads as
     * "if mode == BRAKE, use max duty on both pins (short them); else use
     * zero (let them float)." */
    uint16_t level = (mode == RC_MOTOR_BRAKE) ? (uint16_t)DUTY_MAX : 0U;

    for (i = 0U; i < 2U; i++) {
        (void)rc_pwm_set_duty(motors[i].pin_a, level);
        (void)rc_pwm_set_duty(motors[i].pin_b, level);
        motors[i].last = 0;
    }
    return RC_OK;
}

/* Report the last duty value this wheel was commanded to. Doesn't measure
 * anything live - it's just "what did we last tell it to do." Telemetry
 * reports it; direction of travel is measured by the two-channel encoder
 * (drv_encoder_dir), not inferred from this. */
int16_t drv_motor_get(rc_side_t side)
{
    return (side > RC_SIDE_RIGHT) ? 0 : motors[side].last;
}
