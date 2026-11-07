/*
 *  terrain_ramp.c  -  Buddy 4
 *
 *  Ramp height = sum of (distance rolled x sin(slope)). The formula and
 *  why it does not drift are in terrain_ramp.h. Integer-only: the
 *  Cortex-M0+ has no FPU, and this runs at 100 Hz on the FAST lane.
 */

/* No kernel header in this file (it also builds on the laptop), so
 * rc_prelude.h is not needed for the size_t clash it guards against. */
#include <stddef.h>

#include "terrain_ramp.h"
#include "../rc_config.h"       /* RC_ENC_UM_PER_TICK, RC_PERIOD_IMU_MS */

/* Window length in ms and 1 g in mm/s^2 x 1000, for a_fwd in mg. */
#define WIN_MS          ((int64_t)RAMP_WIN * (int64_t)RC_PERIOD_IMU_MS)
#define G_MM_S2_X1000   (9806650LL)

static uint32_t isqrt32(uint32_t v)
{
    uint32_t r = 0U;
    uint32_t b = 1UL << 30;

    while (b > v) {
        b >>= 2;
    }
    while (b != 0U) {
        if (v >= (r + b)) {
            v -= r + b;
            r = (r >> 1) + b;
        } else {
            r >>= 1;
        }
        b >>= 2;
    }
    return r;
}

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Sample `back` steps before the newest (0 = newest). */
static uint32_t at(const ramp_t *r, uint32_t back)
{
    return (r->head + RAMP_RING - 1U - back) % RAMP_RING;
}

/* Odometry ticks (left + right) to micrometres of ground travel. */
static int32_t ticks2_to_um(int32_t t2)
{
    return (int32_t)(((int64_t)t2 * (int64_t)RC_ENC_UM_PER_TICK) / 2);
}

/* asin(s) in whole degrees, s in permille. asin(x) ~ x + x^3/6 is within
 * 0.3 deg up to 35 deg, steeper than anything the car can climb. */
static int32_t asin_deg(int32_t pm)
{
    int64_t x = pm;
    int64_t a = x + ((x * x * x) / 6000000LL);   /* still permille */

    return (int32_t)((a * 573LL + 5000LL) / 10000LL);
}

void ramp_reset(ramp_t *r)
{
    uint32_t i;

    for (i = 0U; i < RAMP_RING; i++) {
        r->odo2[i] = 0;
        r->ax[i] = 0;
        r->az[i] = 0;
    }
    r->n = 0U;
    r->head = 0U;
    r->odo2_now = 0;
    r->h_um = 0;
    r->zero16 = 0;
    r->on_slope = false;
    r->confirmed = false;
    r->start_ms = 0U;
    r->base_um = 0;
    r->peak_um = 0;
    r->len_um = 0;
    r->level_um = 0;
    r->max_pm = 0;
    r->last_pm = 0;
}

int32_t ramp_height_mm(const ramp_t *r)
{
    return r->h_um / 1000;
}

bool ramp_step(ramp_t *r, int32_t d_ticks2, int16_t ax, int16_t az,
               uint32_t now_ms, ramp_result_t *out)
{
    uint32_t i;
    int32_t  sum_ax = 0;
    int32_t  sum_az = 0;

    r->odo2_now += d_ticks2;
    r->odo2[r->head] = r->odo2_now;
    r->ax[r->head] = ax;
    r->az[r->head] = az;
    r->head = (r->head + 1U) % RAMP_RING;
    if (r->n < RAMP_RING) {
        r->n++;
        r->last_pm = 0;
        return false;
    }

    /* Forward acceleration from the encoders: distance in the newest
     * window minus the one before, over the window time squared.
     * um / ms^2 is exactly m/s^2, so mg = dd_um * 1000 / (T^2 * 9.80665). */
    int32_t d1_um = ticks2_to_um(r->odo2[at(r, 0U)] -
                                 r->odo2[at(r, RAMP_WIN)]);
    int32_t d0_um = ticks2_to_um(r->odo2[at(r, RAMP_WIN)] -
                                 r->odo2[at(r, 2U * RAMP_WIN)]);
    int32_t a_fwd_mg = (int32_t)(((int64_t)(d1_um - d0_um) * 1000000000LL)
                                 / (WIN_MS * WIN_MS * G_MM_S2_X1000));

    /* Tilt averaged over the same two windows, so both are centred on
     * the sample RAMP_WIN back. */
    for (i = 0U; i < (2U * RAMP_WIN); i++) {
        sum_ax += r->ax[at(r, i)];
        sum_az += r->az[at(r, i)];
    }
    int32_t gx = (sum_ax / (int32_t)(2U * RAMP_WIN)) - a_fwd_mg;
    int32_t gz = sum_az / (int32_t)(2U * RAMP_WIN);
    uint32_t mag = isqrt32((uint32_t)((gx * gx) + (gz * gz)));
    /* under ~0.2 g total the car is lifted or falling: no slope reading */
    int32_t s_pm = (mag < 200U) ? 0 : (int32_t)((gx * 1000) / (int32_t)mag);
    int32_t ds_um = ticks2_to_um(r->odo2[at(r, RAMP_WIN)] -
                                 r->odo2[at(r, RAMP_WIN + 1U)]);

    if (mag >= 200U) {
        s_pm -= r->zero16 / 16;
    }
    r->last_pm = s_pm;

    if (!r->on_slope) {
        if (ds_um == 0) {
            return false;       /* standing still: height holds */
        }
        if (iabs32(s_pm) < RAMP_ENTER_PERMILLE) {
            /* driving on the level: learn what level reads as. Slowly
             * and only close to the zero, so the foot of a ramp is not
             * learned as level. */
            if ((mag >= 200U) && (iabs32(s_pm) < RAMP_ZERO_BAND)) {
                r->zero16 += (s_pm * 16) / RAMP_ZERO_TAU;
            }
            return false;
        }
        r->on_slope = true;
        r->confirmed = false;
        r->start_ms = now_ms;
        r->base_um = r->h_um;
        r->peak_um = r->h_um;
        r->len_um = 0;
        r->level_um = 0;
        r->max_pm = 0;
    }

    if (!r->confirmed) {
        if (iabs32(s_pm) < RAMP_ENTER_PERMILLE) {
            /* a rock or a bump, not a slope: undo it */
            r->h_um = r->base_um;
            r->on_slope = false;
            return false;
        }
        if (r->len_um >= (RAMP_CONFIRM_MM * 1000)) {
            r->confirmed = true;
        }
    }

    /* the formula: dh = ds * sin(theta) */
    r->h_um += (int32_t)(((int64_t)ds_um * s_pm) / 1000);
    r->len_um += iabs32(ds_um);
    if (r->h_um > r->peak_um) {
        r->peak_um = r->h_um;
    }
    if (iabs32(s_pm) > r->max_pm) {
        r->max_pm = iabs32(s_pm);
    }
    if (iabs32(s_pm) < RAMP_ENTER_PERMILLE) {
        r->level_um += iabs32(ds_um);
    } else {
        r->level_um = 0;
    }

    if (r->level_um < (RAMP_LEVEL_MM * 1000)) {
        if ((now_ms - r->start_ms) > RAMP_MAX_MS) {
            /* far too long for a ramp on this track: not a real slope */
            r->h_um = r->base_um;
            r->on_slope = false;
        }
        return false;
    }

    /* level again for RAMP_LEVEL_MM: the slope is over */
    r->on_slope = false;
    int32_t rise_mm = (r->peak_um - r->base_um) / 1000;
    int32_t net_mm = (r->h_um - r->base_um) / 1000;

    if (iabs32(r->h_um) < (RAMP_REZERO_MM * 1000)) {
        r->h_um = 0;            /* back on the ground: drop the residue */
    }
    if ((rise_mm < RAMP_MIN_REPORT_MM) &&
        (iabs32(net_mm) < RAMP_MIN_REPORT_MM)) {
        return false;
    }
    if (out != NULL) {
        out->rise_mm = rise_mm;
        out->net_mm = net_mm;
        out->len_mm = (r->len_um - r->level_um) / 1000;
        out->max_deg = asin_deg(r->max_pm);
    }
    return true;
}
