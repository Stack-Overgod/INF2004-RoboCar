/*
 *  drv_imu.h  -  Buddy 4 hardware layer
 *
 *  LSM303D over I2C (address 0x1D, WHO_AM_I 0x49).
 *
 *  Read this before writing any Buddy 4 code: THIS PART HAS NO
 *  GYROSCOPE. It is a 3-axis accelerometer and a 3-axis magnetometer on
 *  one die behind a single I2C address. That changes three
 *  of the tasks in the project brief:
 *
 *    - Tilt must come from the gravity vector in the accelerometer, not
 *      from integrating a rate.
 *    - Turn rate has no direct source. Use differential encoder counts
 *      from Buddy 2, which is more accurate anyway. The magnetometer
 *      will read heading but the motors sit centimetres away and will
 *      swamp it, so treat it as a slow correction at best.
 *    - Hump height cannot come from double-integrating acceleration; the
 *      drift over a two second climb is larger than the hump. Estimate
 *      peak pitch instead and convert with the wheelbase. See
 *      sub_terrain.c for that.
 *
 *  The driver itself just produces calibrated samples. Interpretation
 *  lives in sub_terrain.
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "../rc_types.h"

/* Sets up the I2C connection to the sensor board and switches on the
 * accelerometer (100 Hz) and magnetometer (continuous mode). Call this
 * once at boot, before anything else in this file. Returns an error
 * code (see rc_types.h) instead of crashing if the chip doesn't answer -
 * e.g. if the board isn't wired up yet. */
rc_result_t drv_imu_init(void);

/*
 *  Read one accelerometer sample. Blocks on the I2C driver's own wait,
 *  which is bounded by DEVCNF_I2C0_TMO, so it must be called from a task
 *  and never from an interrupt handler. The sense task owns this.
 *
 *  `x`, `y`, `z` are "pointers" - instead of returning one number, this
 *  function is handed the *addresses* of three variables the caller owns,
 *  and writes the results directly into them (via `*x = ...` etc). That's
 *  how a C function hands back more than one value at once. Units are
 *  milli-g (mg): thousandths of Earth's gravity, so 1000 = 1 g = the pull
 *  you feel standing still.
 */
rc_result_t drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z);

/* Same pattern as drv_imu_read_accel above, but for the magnetometer
 * (the compass sensor). Units are the chip's raw magnetic-field counts,
 * not physical units - we only ever compare them to each other, so the
 * raw scale is fine. */
rc_result_t drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z);

/*
 *  Sample both, apply the stored bias, publish RC_EVT_IMU_SAMPLE.
 *  Called from the sense task at RC_PERIOD_IMU_MS.
 */
void drv_imu_sample(void);

/*
 *  Zero the accelerometer with the car sitting level and still. Averages
 *  over `n` samples. Call once at start-up, before the run.
 *
 *  Why average many samples instead of reading once: every reading has a
 *  little bit of random electrical noise, and this board also has a
 *  small fixed manufacturing/mounting offset even when perfectly level.
 *  Averaging `n` samples cancels out the random noise and leaves just
 *  that fixed offset ("bias"), which we then subtract from every future
 *  reading so "level" reads as zero.
 */
rc_result_t drv_imu_calibrate(uint16_t n);

/* FR15: true once init + calibration succeeded and samples are flowing;
 * false means telemetry should report IMU_UNAVAILABLE. */
bool drv_imu_available(void);

#if APP_TEST
/* TEST BUILD ONLY: hold = true makes drv_imu_sample() publish nothing, so
 * a self-test can feed sub_terrain fake samples without real ones mixed
 * in. Always release it (false) at the end of the test. */
void drv_imu_test_hold(bool hold);
#endif

/* Pitch in tenths of a degree, from the gravity vector. Positive is nose
 * up. Only meaningful when the car is not accelerating hard.
 * ("Pitch" = how much the car is tilted nose-up/nose-down, like the front
 * of a see-saw. "Tenths of a degree" means the number 40 here means 4.0
 * degrees - this is a trick called fixed-point math: using a plain whole
 * number to represent a fractional value, because this chip has no
 * floating-point hardware to do "4.0" style math efficiently.) */
int16_t drv_imu_pitch_ddeg(void);

#endif /* DRV_IMU_H */
