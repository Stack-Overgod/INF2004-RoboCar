/*
 *  rc_pwm.h
 *
 *  Thin layer over the BSP PWM helpers.
 *
 *  The BSP gives pwm_set_pin / pwm_set_wrap / pwm_set_cc /
 *  pwm_set_enabled but no clock divider, and the reset divider of 1.0 at
 *  a 125 MHz system clock puts the lowest achievable frequency at about
 *  1.9 kHz. A 50 Hz servo is impossible without touching PWM_CHx_DIV, so
 *  this module adds that one register write and wraps the rest in a
 *  frequency-and-duty API.
 *
 *  Slice sharing matters: channels A and B of one slice share TOP and
 *  DIV, so they share a frequency. The pin map in rc_config.h is chosen
 *  so that pins which must share a frequency land on the same slice.
 *
 *  Slice = (gpio >> 1) & 7:
 *     GP8/GP9   slice 4   right motor (MOTOR 1), both channels, same frequency
 *     GP10/GP11 slice 5   left motor  (MOTOR 2), both channels, same frequency
 *     GP14/GP15 slice 7   scan servo on GP15 (servo port 4) at 50 Hz
 *
 *  Slices 4, 5 and 7 are therefore unavailable to StartPhysicalTimer,
 *  which on this port is built on the PWM block.
 */

#ifndef RC_PWM_H
#define RC_PWM_H

#include "rc_types.h"

/*
 *  Configure the slice that owns `pin` for `freq_hz`, route the pin to
 *  the PWM function, and start the slice with a 0 % duty.
 *
 *  Calling this twice for two pins on the same slice with different
 *  frequencies is a bug: the second call wins for both.
 */
rc_result_t rc_pwm_init_pin(uint32_t pin, uint32_t freq_hz);

/* Duty in parts per thousand, 0 .. 1000. */
rc_result_t rc_pwm_set_duty(uint32_t pin, uint16_t permille);

/* Pulse width in microseconds. Used for servos, where the datasheet
 * specifies microseconds rather than duty. */
rc_result_t rc_pwm_set_pulse_us(uint32_t pin, uint32_t pulse_us);

rc_result_t rc_pwm_enable(uint32_t pin, bool on);

#endif /* RC_PWM_H */
