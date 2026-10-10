/*
 *  rc_pwm.c
 */

#include "rc_prelude.h"
#include "rc_pwm.h"
#include "rc_config.h"

#define SYS_CLK_HZ          (125000000UL)
#define PWM_SLICE(pin)      (((pin) >> 1) & 0x07U)
#define PWM_CH_DIV_REG(s)   (PWM_BASE + PWM_CHx_DIV + ((s) * 0x14U))

#define PWM_DIV_INT_SHIFT   (4U)
#define WRAP_MAX            (65535UL)
#define PERMILLE_FULL       (1000UL)

/* Remember the wrap per slice so duty and pulse conversions do not have
 * to read the register back. */
static uint32_t slice_wrap[8];
static uint32_t slice_div[8];

rc_result_t rc_pwm_init_pin(uint32_t pin, uint32_t freq_hz)
{
    uint32_t slice;
    uint32_t div_int;
    uint32_t wrap;

    if ((pin >= 30U) || (freq_hz == 0U)) {
        return RC_ERR_PARAM;
    }

    slice = PWM_SLICE(pin);

    /* Pick the smallest integer divider that brings wrap under 16 bits,
     * which keeps the most duty resolution available. */
    div_int = 1U;
    while (((SYS_CLK_HZ / (div_int * freq_hz)) > WRAP_MAX)
           && (div_int < 255U)) {
        div_int++;
    }

    wrap = (SYS_CLK_HZ / (div_int * freq_hz));
    if (wrap == 0U) {
        return RC_ERR_PARAM;
    }
    wrap -= 1U;

    slice_wrap[slice] = wrap;
    slice_div[slice]  = div_int;

    /* Integer part sits in bits 11:4, fractional part in 3:0. We use an
     * integer divider only, so the fraction stays zero. */
    out_w(PWM_CH_DIV_REG(slice), div_int << PWM_DIV_INT_SHIFT);

    (void)pwm_set_pin(pin);
    (void)pwm_set_wrap(pin, wrap);
    (void)pwm_set_cc(pin, 0U);
    (void)pwm_set_enabled(pin, TRUE);

    return RC_OK;
}

rc_result_t rc_pwm_set_duty(uint32_t pin, uint16_t permille)
{
    uint32_t slice;
    uint32_t cc;

    if (pin >= 30U) {
        return RC_ERR_PARAM;
    }
    if (permille > (uint16_t)PERMILLE_FULL) {
        permille = (uint16_t)PERMILLE_FULL;
    }

    slice = PWM_SLICE(pin);
    if (slice_wrap[slice] == 0U) {
        return RC_ERR_STATE;
    }

    cc = ((slice_wrap[slice] + 1U) * (uint32_t)permille) / PERMILLE_FULL;

    return (pwm_set_cc(pin, cc) == E_OK) ? RC_OK : RC_ERR_PARAM;
}

rc_result_t rc_pwm_set_pulse_us(uint32_t pin, uint32_t pulse_us)
{
    uint32_t slice;
    uint32_t ticks_per_ms;
    uint32_t cc;

    if (pin >= 30U) {
        return RC_ERR_PARAM;
    }

    slice = PWM_SLICE(pin);
    if (slice_div[slice] == 0U) {
        return RC_ERR_STATE;
    }

    /* Counter ticks per millisecond after the divider, then scale the
     * pulse. At 125 MHz with the divider of 39 that a 50 Hz servo frame
     * needs, the counter runs at 3.205 MHz: 3205 ticks per ms. Working
     * per millisecond keeps that fraction (a per-microsecond figure would
     * truncate to 3 and make every pulse 6 % short, so "90 degrees"
     * would land nearer 80). pulse_us is at most a few thousand, so the
     * product stays well inside 32 bits. */
    ticks_per_ms = (SYS_CLK_HZ / slice_div[slice]) / 1000UL;
    cc = (pulse_us * ticks_per_ms) / 1000UL;

    if (cc > slice_wrap[slice]) {
        cc = slice_wrap[slice];
    }

    return (pwm_set_cc(pin, cc) == E_OK) ? RC_OK : RC_ERR_PARAM;
}

rc_result_t rc_pwm_enable(uint32_t pin, bool on)
{
    if (pin >= 30U) {
        return RC_ERR_PARAM;
    }
    return (pwm_set_enabled(pin, on ? TRUE : FALSE) == E_OK)
           ? RC_OK : RC_ERR_PARAM;
}
