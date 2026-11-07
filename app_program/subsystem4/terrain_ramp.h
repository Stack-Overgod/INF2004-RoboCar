/*
 *  terrain_ramp.h  -  Buddy 4
 *
 *  Ramp (slope) height from the accelerometer plus wheel odometry.
 *
 *  Every IMU sample the car rolls a short distance ds along the ground.
 *  On a slope of angle theta that piece of ground rises ds * sin(theta),
 *  so the height climbed is the sum of those pieces:
 *
 *      h = sum( ds_k * sin(theta_k) )
 *
 *  sin(theta) comes straight from the gravity vector, with the car's own
 *  forward acceleration (from the encoders) taken out first so speeding
 *  up or braking does not look like a slope:
 *
 *      gx        = ax - a_fwd                       (milli-g)
 *      sin(theta) = gx / sqrt(gx^2 + az^2)
 *
 *  While the car drives on the level, the slope reading it still shows
 *  (IMU mounting, the chassis squatting under drive torque) is learned as
 *  the zero and taken off every reading.
 *
 *  This is one integration of a geometric quantity over distance, not a
 *  double integration of acceleration, so it does not drift with time
 *  (see the warning in drv_imu.h). Dividing by sqrt(gx^2 + az^2) instead
 *  of 1000 mg also cancels any scale error in the sensor.
 *
 *  Plain C, no kernel calls: the host test in test/ramp/ compiles this
 *  file on the laptop. sub_terrain.c feeds it one sample at a time.
 */
#ifndef TERRAIN_RAMP_H
#define TERRAIN_RAMP_H

#include <stdint.h>
#include <stdbool.h>

/* Window for the encoder speed/acceleration estimate, in samples
 * (10 samples = 100 ms at 100 Hz). The slope is read RAMP_WIN samples
 * late so the acceleration and the tilt line up in time. */
#define RAMP_WIN                (10U)
#define RAMP_RING               ((2U * RAMP_WIN) + 1U)

#define RAMP_ENTER_PERMILLE     (70)    /* sin(4.0 deg): steeper is a slope */
#define RAMP_CONFIRM_MM         (25)    /* a slope must last this far       */
#define RAMP_LEVEL_MM           (60)    /* level for this far = slope over  */
#define RAMP_MIN_REPORT_MM      (5)     /* smaller climbs are noise         */
#define RAMP_REZERO_MM          (10)    /* back this near 0 = on the ground */
#define RAMP_MAX_MS             (60000U)/* on a "slope" this long: drop it  */
#define RAMP_ZERO_BAND          (35)    /* sin(2.0 deg): learn the zero ... */
#define RAMP_ZERO_TAU           (256)   /* ... over ~2.6 s of level driving */

/* One finished slope episode (car went level again afterwards). */
typedef struct {
    int32_t  rise_mm;       /* highest point above where the slope began */
    int32_t  net_mm;        /* where it ended above where it began       */
    int32_t  len_mm;        /* ground distance rolled on the slope       */
    int32_t  max_deg;       /* steepest |slope| seen, whole degrees      */
} ramp_result_t;

typedef struct {
    /* last RAMP_RING samples: signed odometry (sum of both wheels, in
     * ticks) and the two accelerometer axes, mg */
    int32_t  odo2[RAMP_RING];
    int16_t  ax[RAMP_RING];
    int16_t  az[RAMP_RING];
    uint32_t n;             /* samples seen, saturates at RAMP_RING     */
    uint32_t head;          /* index the next sample goes into          */
    int32_t  odo2_now;

    /* height above the ground level at boot / reset, micrometres */
    int32_t  h_um;

    /* slope reading on level ground, permille x16 (learned) */
    int32_t  zero16;

    /* the current slope episode */
    bool     on_slope;
    bool     confirmed;     /* lasted RAMP_CONFIRM_MM                   */
    uint32_t start_ms;
    int32_t  base_um;
    int32_t  peak_um;
    int32_t  len_um;
    int32_t  level_um;      /* continuous level distance at its end     */
    int32_t  max_pm;        /* steepest |sin| so far, permille          */

    int32_t  last_pm;       /* latest sin(theta) less the zero, permille */
} ramp_t;

void ramp_reset(ramp_t *r);

/* Feed one IMU sample. d_ticks2 = signed encoder ticks since the last
 * sample, LEFT + RIGHT added (so twice the average); ax/az are the
 * bias-corrected accelerometer readings in mg (x forward, positive nose
 * up / speeding up; z up). Returns true and fills *out when a slope
 * episode just ended with a climb or drop worth reporting. */
bool ramp_step(ramp_t *r, int32_t d_ticks2, int16_t ax, int16_t az,
               uint32_t now_ms, ramp_result_t *out);

/* Current height above the starting ground level, mm. */
int32_t ramp_height_mm(const ramp_t *r);

#endif /* TERRAIN_RAMP_H */
