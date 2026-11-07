/*
 *  sub_terrain.c
 *
 *  Buddy 4: Terrain and motion event classification.
 *  Uses the LSM303D accelerometer to detect speed humps, estimate peak
 *  height via chassis geometry cross-checked with wheel odometry, and flag
 *  impacts/shocks without relying on a non-existent gyroscope.
 */

#include "../rc_prelude.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "../rc_config.h"
#include "../rc_types.h"
#include "../rc_time.h"
#include "../rc_event.h"
#include "../rc_fault.h"
#include "drv_imu.h"
#include "../subsystem2/drv_encoder.h"
#include "sub_terrain.h"
#include "terrain_ramp.h"

/* ------------------------------------------------------------------ *
 *  Detection thresholds and timing windows
 * ------------------------------------------------------------------ */

#define PITCH_ENTER_DDEG        (30)     /* 3.0 deg nose-up to enter hump   */
#define PITCH_EXIT_DDEG         (20)     /* 2.0 deg deadband to exit hump   */
#define PITCH_SMOOTH_N          (16)     /* pitch average, ~160 ms          */
#define LEVEL_TAU               (256)    /* level re-learnt over ~2.6 s     */
#define HUMP_MIN_MS             (150U)   /* Discard noise spikes < 150 ms   */
#define HUMP_FLAT_ABORT_MS      (300U)   /* Level this long ends a climb    */
#define HUMP_MAX_MS             (10000U) /* Abort if climb takes > 10 s     */
#define MIN_HUMP_TRAVEL_MM      (30U)    /* Reject if car rolled < 30 mm    */
#define HUMP_MIN_REPORT_MM      (5U)     /* Lower humps are not reported    */

#define IMPACT_MG               (RC_IMPACT_MG)   /* FR13, rc_config.h */

/* 1: print the IMU readings to the console once a second (bench check
 * of the axes). Keep 0 in the repo. */
#ifndef S4_IMU_PRINT
#define S4_IMU_PRINT            (0)
#endif
#if S4_IMU_PRINT
#include <tm/tmonitor.h>
#endif

#define ACCEL_LINEAR_MG         (250)    /* Linear push threshold on X      */
#define TURN_SPEED_DIFF_MM_S    (150)    /* Wheel speed diff for turning    */

/* Hump state machine phases */
typedef enum {
    H_FLAT = 0,
    H_CLIMBING,
    H_DESCENDING
} hump_state_t;

/* ------------------------------------------------------------------ *
 *  Internal subsystem state
 * ------------------------------------------------------------------ */

static hump_state_t          hump_state;
static uint32_t              hump_start_ms;
static int16_t               hump_peak_pitch;
static int32_t               pitch_x16;     /* smooth_pitch() state, x16 */
static int32_t               pitch_lvl_x256; /* level_dev(): level pitch */
static int32_t               roll_lvl_x256;  /* level_dev(): level roll  */
static uint32_t              hump_flat_ms;  /* climbing, level since; 0 = no */
/* roll_step(): one-wheel hump, smoothed roll x16 and the episode */
static int32_t               roll_x16;
static bool                  roll_on;
static int32_t               roll_abs;      /* |roll| off level, ddeg */
static uint32_t              roll_start_ms;
static uint32_t              roll_flat_ms;
static int16_t               roll_peak;
static uint32_t              roll_dist0;
static uint32_t              dist_at_hump_start;
static uint16_t              max_peak_height_mm;
static rc_motion_class_t     current_motion_class;
static uint32_t              max_wcet_us;
static bool                  is_subscribed = false;

static sub_terrain_hump_cb_t hump_cb;
static void                 *hump_cb_ctx;

/* Ramp height (terrain_ramp.c). Only on_sample() touches `ramp`; the
 * height is copied out to one halfword for other tasks to read. */
static ramp_t                ramp;
static uint32_t              ramp_cnt[2];   /* encoder counts last sample */
static volatile int16_t      height_mm;

/* Helper: average distance both wheels have rolled since boot, mm.
 * Built from the raw edge counts, which only ever go up.
 * drv_encoder_distance_mm() is NOT used: sub_motion resets that baseline
 * at the start of every move, which would break a hump that spans one. */
static uint32_t get_distance_mm(void)
{
    uint64_t ticks = ((uint64_t)drv_encoder_count(RC_SIDE_LEFT)
                      + (uint64_t)drv_encoder_count(RC_SIDE_RIGHT)) / 2U;

    return (uint32_t)((ticks * (uint64_t)RC_ENC_UM_PER_TICK) / 1000U);
}

/* Signed encoder ticks since the last call, left + right. The counts
 * only go up, so the sign comes from each wheel's direction. */
static int32_t ramp_ticks2(void)
{
    int32_t  d2 = 0;
    uint32_t s;

    for (s = 0U; s < 2U; s++) {
        rc_side_t side = (s == 0U) ? RC_SIDE_LEFT : RC_SIDE_RIGHT;
        uint32_t  c = drv_encoder_count(side);
        int32_t   d = (int32_t)(c - ramp_cnt[s]);

        ramp_cnt[s] = c;
        d2 += (drv_encoder_dir(side) < 0) ? -d : d;
    }
    return d2;
}

/* One IMU sample into the ramp integrator; publishes RC_EVT_RAMP_END
 * when a slope has just finished. */
static void ramp_sample(int16_t ax, int16_t az, uint32_t now_ms)
{
    ramp_result_t res;
    int32_t       h;

    if (ramp_step(&ramp, ramp_ticks2(), ax, az, now_ms, &res)) {
        rc_event_t out_evt = {0};

        out_evt.id = RC_EVT_RAMP_END;
        out_evt.u.ramp.rise_mm = RC_SAT_U(res.rise_mm, RC_BITS_RAMP_RISE);
        out_evt.u.ramp.net_mm = RC_SAT_S(res.net_mm, RC_BITS_HEIGHT);
        out_evt.u.ramp.max_deg = RC_SAT_U(res.max_deg, RC_BITS_RAMP_DEG);
        out_evt.u.ramp.len_mm = RC_SAT_U(res.len_mm, RC_BITS_RAMP_LEN);
        (void)rc_event_publish(&out_evt);
    }

    h = ramp_height_mm(&ramp);
    height_mm = (int16_t)((h > 511) ? 511 : ((h < -512) ? -512 : h));
}

/* ------------------------------------------------------------------ *
 *  Mathematical Modeling: Small-Angle Geometry
 * ------------------------------------------------------------------ */

/*
 * Converts peak pitch in deci-degrees (0.1 deg) into vertical rise in mm
 * using the car's front-to-rear axle distance (RC_AXLE_DIST_MM, FR5).
 *
 * Assumed Hump Geometry (FR5):
 * Symmetric non-plateaued ramp profile where ramp length L_ramp ≈ L.
 * Formula: rise = L * sin(pitch)
 * Small-angle approximation: sin(theta) ≈ theta (in radians).
 * Conversion factor: 1 ddeg = pi / 1800 rad ≈ 1745 / 1,000,000 rad.
 */
static uint16_t pitch_to_height_mm(int16_t peak_ddeg)
{
    if (peak_ddeg <= 0) {
        return 0U;
    }

    /* Convert tenths of a degree to milliradians */
    int32_t peak_mrad = ((int32_t)peak_ddeg * 1745) / 1000;

    /* FR5: rise = L * sin(pitch), L = front-to-rear axle distance */
    int32_t rise_mm = ((int32_t)RC_AXLE_DIST_MM * peak_mrad) / 1000;
    if (rise_mm < 0) {
        rise_mm = 0;
    }

    return (uint16_t)rise_mm;
}

/* A smoothed tilt (x16 deci-degrees) less what level ground reads, in
 * deci-degrees. The IMU's zero is set at power-up (drv_imu_calibrate),
 * often while the car is being handled, so it can be a few degrees off;
 * this learns the level reading instead, over ~LEVEL_TAU samples, but
 * only while `learn` (not in a hump) and within PITCH_EXIT_DDEG of it,
 * so a hump or a slope is never learned as level. *lvl_x256 holds the
 * level in deci-degrees x256: the extra bits keep a 0.1 degree gap
 * moving it. */
static int32_t level_dev(int32_t tilt_x16, int32_t *lvl_x256, bool learn)
{
    int32_t dev = ((tilt_x16 * 16) - *lvl_x256) / 256;

    if (learn && (dev > -PITCH_EXIT_DDEG) && (dev < PITCH_EXIT_DDEG)) {
        *lvl_x256 += ((tilt_x16 * 16) - *lvl_x256) / LEVEL_TAU;
    }
    return dev;
}

/* Pitch for the hump state machine, deci-degrees, smoothed. One raw
 * 10 ms sample shakes by several degrees while the motors run, which on
 * its own is enough to cross PITCH_ENTER_DDEG and then -PITCH_EXIT_DDEG
 * within a few hundred ms. A real hump holds its pitch for seconds at line
 * speed (~30 mm/s). */
static int16_t smooth_pitch(int16_t raw_ddeg)
{
    int32_t dev;

    /* Exponential average, 1/PITCH_SMOOTH_N of the gap per sample
     * (time constant PITCH_SMOOTH_N x 10 ms); pitch_x16 keeps 4 extra
     * fraction bits so small steps are not lost to integer division. */
    pitch_x16 += (((int32_t)raw_ddeg * 16) - pitch_x16) / PITCH_SMOOTH_N;
    dev = level_dev(pitch_x16, &pitch_lvl_x256, hump_state == H_FLAT);
    return (int16_t)dev;
}

/* Report a finished hump: peak height (mm), how long it lasted and its
 * steepest tilt (pitch or roll, deci-degrees). */
static void hump_publish(uint16_t height_mm, uint32_t duration,
                         int16_t peak_ddeg)
{
    if (height_mm < HUMP_MIN_REPORT_MM) {
        return;     /* a jolt (the START jerk reads ~2 mm), not a hump */
    }
    if (height_mm > max_peak_height_mm) {
        max_peak_height_mm = height_mm;
    }

    /* Broadcast completion event */
    rc_event_t out_evt = {0};
    out_evt.id = RC_EVT_HUMP_END;
    /* 7 / 13 / 8 bits, clamped (not wrapped) */
    out_evt.u.hump.peak_mm =
        RC_SAT_U(height_mm, RC_BITS_HUMP_PEAK);
    out_evt.u.hump.duration_ms =
        RC_SAT_U(duration, RC_BITS_HUMP_MS);
    out_evt.u.hump.max_pitch_deg =
        RC_SAT_S(peak_ddeg / 10, RC_BITS_PITCH);
    (void)rc_event_publish(&out_evt);

    if (hump_cb != NULL) {
        hump_cb(height_mm, duration, hump_cb_ctx);
    }
}

/* A hump episode has ended (nose-up, then level, or nose-up, nose-down,
 * then level): report it unless it was too short, or the car was driving
 * but rolled less than MIN_HUMP_TRAVEL_MM (a jolt, not a hump).
 * need_travel: also drop it when the wheels did not turn at all (a car
 * tilted by hand and put back, which is not a crossing). */
static void hump_finish(uint32_t now_ms, bool need_travel)
{
    uint32_t duration = now_ms - hump_start_ms;
    uint32_t current_dist = get_distance_mm();
    uint32_t dist_travelled = 0U;

    if (current_dist >= dist_at_hump_start) {
        dist_travelled = current_dist - dist_at_hump_start;
    }

    if (duration >= HUMP_MIN_MS) {
        /*
         * ENCODER CROSS-CHECK:
         * Discard false positives if wheels didn't roll forward.
         * Stationary bench tests have 0 wheel speed and 0 travel.
         */
        bool encoders_active =
            (drv_encoder_speed_mm_s(RC_SIDE_LEFT) != 0) ||
            (drv_encoder_speed_mm_s(RC_SIDE_RIGHT) != 0) ||
            (dist_travelled > 0U);

        if ((!encoders_active && !need_travel) ||
            (dist_travelled >= MIN_HUMP_TRAVEL_MM)) {

            /* Primary estimate: Chassis Axle Distance Geometry */
            uint32_t h_geom = pitch_to_height_mm(hump_peak_pitch);
            uint16_t final_height = (uint16_t)h_geom;

            /* Secondary estimate: Distance Travelled * sin(pitch) */
            if (encoders_active) {
                int32_t peak_mrad =
                    ((int32_t)hump_peak_pitch * 1745) / 1000;
                uint32_t dist_ramp = dist_travelled / 2U;
                uint32_t h_odo =
                    (uint32_t)(((int32_t)dist_ramp * peak_mrad) /
                               1000);
                final_height = (uint16_t)((h_geom + h_odo) / 2U);
            }

            hump_publish(final_height, duration, hump_peak_pitch);
        }
    }
    hump_state = H_FLAT;
}

/* Hump under ONE wheel: the car rolls sideways instead of pitching
 * (the track's hump is met by the right wheel, so the car leans left;
 * the side does not matter here, |roll| is used). A rise h under one
 * wheel rolls the car by asin(h / RC_WHEEL_BASE_MM), ~5 deg for 10 mm,
 * so h = RC_WHEEL_BASE_MM x sin(peak roll). Same enter / level-for-
 * HUMP_FLAT_ABORT_MS / distance rules as the pitch state machine; the
 * wheels must have rolled MIN_HUMP_TRAVEL_MM (a car tilted by hand is
 * not a crossing). */
static void roll_step(int16_t ay, uint32_t now_ms)
{
    int32_t roll;

    roll_x16 += ((((int32_t)ay * 573) / 1000) * 16 - roll_x16) / PITCH_SMOOTH_N;
    roll = level_dev(roll_x16, &roll_lvl_x256, !roll_on);
    if (roll < 0) {
        roll = -roll;
    }
    roll_abs = roll;

    if (!roll_on) {
        if (roll >= PITCH_ENTER_DDEG) {
            roll_on = true;
            roll_start_ms = now_ms;
            roll_flat_ms = 0U;
            roll_peak = (int16_t)roll;
            roll_dist0 = get_distance_mm();
        }
        return;
    }

    if (roll > roll_peak) {
        roll_peak = (int16_t)roll;
    }
    if ((now_ms - roll_start_ms) > HUMP_MAX_MS) {
        roll_on = false;            /* far too long for a hump: a slope */
    } else if (roll >= PITCH_EXIT_DDEG) {
        roll_flat_ms = 0U;
    } else if (roll_flat_ms == 0U) {
        roll_flat_ms = now_ms | 1U;
    } else if ((now_ms - roll_flat_ms) > HUMP_FLAT_ABORT_MS) {
        uint32_t d = get_distance_mm();

        roll_on = false;
        if ((d >= roll_dist0) && ((d - roll_dist0) >= MIN_HUMP_TRAVEL_MM)) {
            int32_t mrad = ((int32_t)roll_peak * 1745) / 1000;
            int32_t h = ((int32_t)RC_WHEEL_BASE_MM * mrad) / 1000;

            hump_publish((uint16_t)h, now_ms - roll_start_ms, roll_peak);
        }
    } else {
        /* level, not yet for long enough */
    }
}

/* ------------------------------------------------------------------ *
 *  8-State Motion Classifier (FR14)
 * ------------------------------------------------------------------ */

static rc_motion_class_t classify(int16_t ax, int16_t ay)
{
    /* Priority 1: High-g Impact / Shock */
    /* FR13: max(|ax|, |ay|) >= RC_IMPACT_MG */
    if ((ax >= IMPACT_MG) || (ax <= -IMPACT_MG) ||
        (ay >= IMPACT_MG) || (ay <= -IMPACT_MG)) {
        return RC_MOTION_IMPACT;
    }

    /* Priority 2: Active Slope Traverse */
    if ((hump_state == H_CLIMBING) || roll_on) {
        return RC_MOTION_CLIMBING;      /* nose-up, or one wheel up */
    }
    if (hump_state == H_DESCENDING) {
        return RC_MOTION_DESCENDING;
    }

    /* Priority 3: Stationary Check via Buddy 2 Encoders */
    int32_t spd_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
    int32_t spd_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);

    if ((spd_l == 0) && (spd_r == 0)) {
        return RC_MOTION_STATIONARY;
    }

    /* Priority 4: Turning (Differential wheel speed analysis) */
    int32_t diff = spd_l - spd_r;
    if ((diff > TURN_SPEED_DIFF_MM_S) || (diff < -TURN_SPEED_DIFF_MM_S)) {
        return RC_MOTION_TURNING;
    }

    /* Priority 5: Forward / Backward Linear Acceleration */
    if (ax > ACCEL_LINEAR_MG) {
        return RC_MOTION_ACCELERATING;
    }
    if (ax < -ACCEL_LINEAR_MG) {
        return RC_MOTION_DECELERATING;
    }

    /* Priority 6: Steady-state Cruising */
    return RC_MOTION_CRUISING;
}

/* ------------------------------------------------------------------ *
 *  Event Handler: Runs on FAST lane every IMU sample (100 Hz)
 * ------------------------------------------------------------------ */

static void on_sample(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    if (evt == NULL) {
        return;
    }

    uint32_t t0_us  = (uint32_t)rc_time_us();
    int16_t  ax     = evt->u.imu.acc_x;
    int16_t  ay     = evt->u.imu.acc_y;
    /* Pitch in deci-degrees from acc_x, smoothed (smooth_pitch) */
    int16_t  pitch  = smooth_pitch((int16_t)(((int32_t)ax * 573) / 1000));
    uint32_t now_ms = rc_time_ms();

    /* --- Hump State Machine --- */
    switch (hump_state) {
    case H_FLAT:
        if (pitch >= PITCH_ENTER_DDEG) {
            hump_state = H_CLIMBING;
            hump_start_ms = now_ms;
            hump_peak_pitch = pitch;
            hump_flat_ms = 0U;

            /* Snapshot starting distance to validate real travel */
            dist_at_hump_start = get_distance_mm();

            rc_event_t out_evt = {0};
            out_evt.id = RC_EVT_HUMP_BEGIN;
            (void)rc_event_publish(&out_evt);
        }
        break;

    case H_CLIMBING:
        if (pitch > hump_peak_pitch) {
            hump_peak_pitch = pitch;
        }

        /* Nose dips down: car has crested the summit */
        if (pitch <= -PITCH_EXIT_DDEG) {
            hump_state = H_DESCENDING;
        } else if ((now_ms - hump_start_ms) > HUMP_MAX_MS) {
            /* Timed out: slope is too long to be a speed hump, reset */
            hump_state = H_FLAT;
        } else if (pitch >= PITCH_EXIT_DDEG) {
            hump_flat_ms = 0U;          /* still nose-up */
        } else if (hump_flat_ms == 0U) {
            hump_flat_ms = now_ms | 1U; /* level from now (never 0) */
        } else if ((now_ms - hump_flat_ms) > HUMP_FLAT_ABORT_MS) {
            /* Level for HUMP_FLAT_ABORT_MS without dipping nose-down. At
             * line speed this is the usual hump: the front caster rides
             * up and back down (or the car levels out on top) before
             * the drive wheels reach it. A forward surge on flat ground
             * (Risk G6) is dropped by hump_finish()'s distance check. */
            hump_finish(now_ms, true);
        } else {
            /* Level, not yet for long enough */
        }
        break;

    case H_DESCENDING:
        /* Level out: car returns within the exit band */
        if ((pitch >= -PITCH_EXIT_DDEG) && (pitch <= PITCH_EXIT_DDEG)) {
            hump_finish(now_ms, false);
        }
        break;

    default:
        hump_state = H_FLAT;
        break;
    }

    /* --- One-wheel hump: the car rolls (roll_step) --- */
    roll_step(ay, now_ms);

#if S4_IMU_PRINT
    {
        /* console only: the corrected readings once a second */
        static uint32_t n_print;

        if (++n_print >= 100U) {
            n_print = 0U;
            tm_printf((UB *)"[s4] ax %d ay %d az %d mg  pitch %d roll %d"
                      " ddeg\n", (INT)ax, (INT)ay, (INT)evt->u.imu.acc_z,
                      (INT)(pitch_x16 / 16), (INT)(roll_x16 / 16));
        }
    }
#endif

    /* --- Ramp height: h = sum(ds * sin(theta)), terrain_ramp.h --- */
    /* The ramp integrator gets acc_x less the learned level (mg = ddeg x
     * 1000 / 573). Leaning sideways means one wheel is on something, not
     * a ramp: the nose dips as it does on a slope, so while |roll| >=
     * PITCH_EXIT_DDEG it is fed level (its slope episode then lapses and
     * is undone). */
    {
        int16_t ramp_ax = ax;

        /* level ground reads 0 for the ramp too (level_dev) */
        ramp_ax -= (int16_t)(((pitch_lvl_x256 / 256) * 1000) / 573);
        if (roll_abs >= PITCH_EXIT_DDEG) {
            ramp_ax = 0;
        }
        ramp_sample(ramp_ax, evt->u.imu.acc_z, now_ms);
    }

    /* --- Motion Classification --- */
    rc_motion_class_t new_class = classify(ax, ay);

    if (new_class != current_motion_class) {
        current_motion_class = new_class;

        rc_event_t class_evt = {0};
        class_evt.id = RC_EVT_MOTION_CLASS;
        class_evt.u.motion_class.cls = new_class;
        (void)rc_event_publish(&class_evt);

        /* Publish dedicated impact alert for immediate safety stop */
        if (new_class == RC_MOTION_IMPACT) {
            rc_event_t impact_evt = {0};
            rc_fault_set(RC_FAULT_IMPACT);  /* cleared by START (sub_nav) */
            impact_evt.id = RC_EVT_IMPACT;
            (void)rc_event_publish(&impact_evt);
        }
    }

    uint32_t elapsed_us = (uint32_t)rc_time_us() - t0_us;
    if (elapsed_us > max_wcet_us) {
        max_wcet_us = elapsed_us;
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

rc_result_t sub_terrain_init(void)
{
    sub_terrain_reset();
    max_wcet_us = 0U;
    hump_cb = NULL;
    hump_cb_ctx = NULL;

    /* Subscribe once on FAST lane so vehicle controller can react promptly */
    /* rc_event_subscribe() returns a handle >= 0 on success (lane << 16 |
     * slot), not RC_OK, and a negative value on failure. */
    if (!is_subscribed) {
        if (rc_event_subscribe(RC_EVT_IMU_SAMPLE, RC_LANE_FAST,
                               on_sample, NULL) < 0) {
            return RC_ERR_NOSPACE;
        }
        is_subscribed = true;
    }

    return RC_OK;
}

void sub_terrain_reset(void)
{
    hump_state = H_FLAT;
    hump_start_ms = 0U;
    hump_peak_pitch = 0;
    pitch_x16 = 0;
    pitch_lvl_x256 = 0;
    roll_lvl_x256 = 0;
    hump_flat_ms = 0U;
    roll_x16 = 0;
    roll_on = false;
    roll_abs = 0;
    roll_flat_ms = 0U;
    roll_peak = 0;
    dist_at_hump_start = 0U;
    max_peak_height_mm = 0U;
    current_motion_class = RC_MOTION_STATIONARY;

    ramp_reset(&ramp);
    ramp_cnt[0] = drv_encoder_count(RC_SIDE_LEFT);
    ramp_cnt[1] = drv_encoder_count(RC_SIDE_RIGHT);
    height_mm = 0;
}

void sub_terrain_level_here(void)
{
    pitch_lvl_x256 = pitch_x16 * 16;
    roll_lvl_x256 = roll_x16 * 16;
    hump_state = H_FLAT;
    roll_on = false;
    ramp_reset(&ramp);
    ramp_cnt[0] = drv_encoder_count(RC_SIDE_LEFT);
    ramp_cnt[1] = drv_encoder_count(RC_SIDE_RIGHT);
    height_mm = 0;
}

rc_result_t sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx)
{
    hump_cb = cb;
    hump_cb_ctx = ctx;
    return RC_OK;
}

uint16_t sub_terrain_max_peak_mm(void)
{
    return max_peak_height_mm;
}

int16_t sub_terrain_height_mm(void)
{
    return height_mm;
}

rc_motion_class_t sub_terrain_motion_class(void)
{
    return current_motion_class;
}

uint32_t sub_terrain_wcet_us(void)
{
    return max_wcet_us;
}