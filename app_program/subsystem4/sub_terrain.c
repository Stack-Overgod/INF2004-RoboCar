/*
 *  sub_terrain.c
 *
 *  Buddy 4: Terrain and motion event classification.
 *  Uses the LSM303DLHC accelerometer to detect speed humps, estimate peak
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
#include "drv_imu.h"
#include "../subsystem2/drv_encoder.h"
#include "sub_terrain.h"

/* ------------------------------------------------------------------ *
 *  Detection thresholds and timing windows
 * ------------------------------------------------------------------ */

#define PITCH_ENTER_DDEG        (40)     /* 4.0 deg nose-up to enter hump   */
#define PITCH_EXIT_DDEG         (20)     /* 2.0 deg deadband to exit hump   */
#define HUMP_MIN_MS             (150U)   /* Discard noise spikes < 150 ms   */
#define HUMP_MAX_MS             (5000U)  /* Abort if climb takes > 5 s      */
#define MIN_HUMP_TRAVEL_MM      (30U)    /* Reject if car rolled < 30 mm    */

#define IMPACT_MG               (2200)   /* 2.2g threshold for collision    */
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
static uint32_t              dist_at_hump_start;
static uint16_t              max_peak_height_mm;
static rc_motion_class_t     current_motion_class;

static sub_terrain_hump_cb_t hump_cb;
static void                 *hump_cb_ctx;

/* Helper: Average forward distance from both wheel encoders */
static uint32_t get_distance_mm(void)
{
    uint32_t dist_l = drv_encoder_distance_mm(RC_SIDE_LEFT);
    uint32_t dist_r = drv_encoder_distance_mm(RC_SIDE_RIGHT);

    return (dist_l + dist_r) / 2U;
}

/* ------------------------------------------------------------------ *
 *  Mathematical Modeling: Small-Angle Geometry
 * ------------------------------------------------------------------ */

/*
 * Converts peak pitch in deci-degrees (0.1 deg) into vertical rise in mm
 * using the car's known physical wheelbase (RC_WHEEL_BASE_MM).
 *
 * Formula: rise = wheelbase * sin(pitch)
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

    /* Calculate rise: (wheelbase_mm * mrad) / 1000 */
    int32_t rise_mm = ((int32_t)RC_WHEEL_BASE_MM * peak_mrad) / 1000;
    if (rise_mm < 0) {
        rise_mm = 0;
    }

    return (uint16_t)rise_mm;
}

/* ------------------------------------------------------------------ *
 *  6-State Motion Classifier
 * ------------------------------------------------------------------ */

static rc_motion_class_t classify(int16_t ax, int16_t ay)
{
    /* Priority 1: High-g Impact / Shock */
    if ((ax > IMPACT_MG) || (ax < -IMPACT_MG) ||
        (ay > IMPACT_MG) || (ay < -IMPACT_MG)) {
        return RC_MOTION_IMPACT;
    }

    /* Priority 2: Active Slope Traverse */
    if (hump_state == H_CLIMBING) {
        return RC_MOTION_CLIMBING;
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

    int16_t  ax = evt->u.imu.acc_x;
    int16_t  ay = evt->u.imu.acc_y;
    int16_t  pitch = drv_imu_pitch_ddeg();
    uint32_t now_ms = rc_time_ms();

    /* --- Hump State Machine --- */
    switch (hump_state) {
    case H_FLAT:
        if (pitch >= PITCH_ENTER_DDEG) {
            hump_state = H_CLIMBING;
            hump_start_ms = now_ms;
            hump_peak_pitch = pitch;

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
        } else {
            /* Continue tracking climb */
        }
        break;

    case H_DESCENDING:
        /* Level out: car returns within the exit band */
        if ((pitch >= -PITCH_EXIT_DDEG) && (pitch <= PITCH_EXIT_DDEG)) {
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
                 * Also allows desk simulation if encoders are inactive (0).
                 */
                bool encoders_active = (current_dist > 0U);

                if (!encoders_active ||
                    (dist_travelled >= MIN_HUMP_TRAVEL_MM)) {

                    /* Primary estimate: Chassis Wheelbase Geometry */
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

                    if (final_height > max_peak_height_mm) {
                        max_peak_height_mm = final_height;
                    }

                    /* Broadcast completion event */
                    rc_event_t out_evt = {0};
                    out_evt.id = RC_EVT_HUMP_END;
                    out_evt.u.hump.peak_mm = final_height;
                    out_evt.u.hump.duration_ms = duration;
                    out_evt.u.hump.max_pitch_deg =
                        (int16_t)(hump_peak_pitch / 10);
                    (void)rc_event_publish(&out_evt);

                    if (hump_cb != NULL) {
                        hump_cb(final_height, duration, hump_cb_ctx);
                    }
                }
            }
            hump_state = H_FLAT;
        }
        break;

    default:
        hump_state = H_FLAT;
        break;
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
            impact_evt.id = RC_EVT_IMPACT;
            (void)rc_event_publish(&impact_evt);
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

rc_result_t sub_terrain_init(void)
{
    sub_terrain_reset();
    hump_cb = NULL;
    hump_cb_ctx = NULL;

    /* Subscribe on FAST lane so the vehicle controller can brake promptly */
    return rc_event_subscribe(RC_EVT_IMU_SAMPLE, RC_LANE_FAST,
                              on_sample, NULL);
}

void sub_terrain_reset(void)
{
    hump_state = H_FLAT;
    hump_start_ms = 0U;
    hump_peak_pitch = 0;
    dist_at_hump_start = 0U;
    max_peak_height_mm = 0U;
    current_motion_class = RC_MOTION_STATIONARY;
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

rc_motion_class_t sub_terrain_motion_class(void)
{
    return current_motion_class;
}