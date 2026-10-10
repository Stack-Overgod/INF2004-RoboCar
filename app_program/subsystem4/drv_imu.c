/*
 *  drv_imu.c
 */
#include "../rc_prelude.h"
#include "../../device/include/dev_i2c.h"
#include "drv_imu.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* LSM303DLHC registers. The accelerometer and magnetometer are separate
 * dice with separate address spaces; do not mix them up.
 *
 * "Register" here just means a numbered memory slot inside the sensor
 * chip that you can write a setting into or read a measurement out of -
 * like a row in a tiny built-in spreadsheet. `#define` gives each slot
 * number a readable name so the code below never has bare hex constants
 * whose meaning you'd have to look up in a datasheet every time. */
/* accelerometer: turns it on, sets sample rate */
#define A_CTRL_REG1     (0x20U)
/* accelerometer: sets measurement range/resolution */
#define A_CTRL_REG4     (0x23U)
/* accelerometer: first of 6 output registers (X low byte) */
#define A_OUT_X_L       (0x28U)

#define M_CRA_REG       (0x00U)  /* magnetometer: sample-rate config register */
/* magnetometer: gain/sensitivity config register */
#define M_CRB_REG       (0x01U)
/* magnetometer: mode register (continuous vs sleep) */
#define M_MR_REG        (0x02U)
/* magnetometer: first of 6 output registers (X high byte) */
#define M_OUT_X_H       (0x03U)

/* Value written to A_CTRL_REG1 to configure the accelerometer:
 * 100 Hz sample rate, all three axes enabled, normal power mode. This is
 * a specific bit pattern defined by the LSM303DLHC datasheet, not an
 * arbitrary number - each bit in 0x57 turns on one of those features. */
#define A_ODR_100HZ     (0x57U)
/* Value written to A_CTRL_REG4: +/-2 g measurement range, high
 * resolution. At 2 g one LSB (least-significant bit, i.e. the smallest
 * change the sensor can report) is about 1 mg, which is the resolution
 * we want for hump detection - a wider range (e.g. +/-8g) would let the
 * car survive bigger shocks without clipping, but each step would then
 * represent more g's, so small tilts would be measured less precisely. */
#define A_FS_2G_HR      (0x08U)

/* Magnetometer sample rate: 75 Hz. */
#define M_ODR_75HZ      (0x18U)
/* Magnetometer gain setting: how many raw counts correspond to how much
 * magnetic field strength. */
#define M_GAIN_1_3      (0x20U)
/* Magnetometer mode: continuous-conversion (always taking new readings),
 * as opposed to single-shot or sleep. */
#define M_CONTINUOUS    (0x00U)

/* Auto-increment on a multi-byte read is the MSB of the sub-address on
 * the accelerometer die. The magnetometer auto-increments by default.
 *
 * In plain terms: normally, asking a register for data only gives you
 * that one register's byte, and you'd have to send six separate requests
 * to get X, Y, and Z (2 bytes each). "Auto-increment" mode says "after
 * you send me each byte, automatically move to the next register" so one
 * request can stream out all 6 bytes back-to-back. On this chip, you ask
 * for that behaviour by setting the top bit (0x80) of the register
 * address you send - i.e. OR'ing the starting register with this mask
 * (see A_OUT_X_L | AUTO_INC below). Forgetting this bit means every
 * multi-byte read would return garbage after the first two bytes. */
#define AUTO_INC        (0x80U)

/* unit 0 (GP4/GP5 after the HARDWARE.md patch) - the RTOS's name for the I2C
 * bus device */
#define I2C_DEVNAME     ((UB *)"iica")

/* `static` here means these variables are private to this file - no
 * other .c file can see or touch them directly, only through the
 * functions below. That's how this driver keeps its internal state
 * (calibration bias, last reading) hidden from the rest of the codebase. */
/* device handle for the opened I2C bus, or -1 if not open */
static ID      i2c_dd = -1;
/* calibration offset to subtract from raw accel X (mg) */
static int16_t bias_x;
/* calibration offset to subtract from raw accel Y (mg) */
static int16_t bias_y;
/* calibration offset to subtract from raw accel Z (mg), see note below */
static int16_t bias_z;
/* most recent bias-corrected accel X reading (mg), used by pitch calc */
static int16_t last_ax;
/* most recent bias-corrected accel Y reading (mg) */
static int16_t last_ay;
/* most recent bias-corrected accel Z reading (mg) */
static int16_t last_az;

/* ------------------------------------------------------------------ *
 *  I2C helpers
 *
 *  I2C (pronounced "eye-squared-C") is a simple two-wire protocol (one
 *  data wire, one clock wire) that lets several sensor chips share the
 *  same two Pico pins, each one answering only when its own address is
 *  sent first. `sadr` below is that chip address (accelerometer and
 *  magnetometer have different ones, even though they're on the same
 *  physical board). These two helpers are the only place in this file
 *  that actually talks to the RTOS's I2C device driver; everything else
 *  calls through them.
 * ------------------------------------------------------------------ */

/* Writes one byte `val` into register `reg` of the chip at address
 * `sadr`. Used for configuration (e.g. "turn on 100Hz sampling"), never
 * for reading sensor data. */
static ER reg_write(UW sadr, UB reg, UB val)
{
    T_I2C_EXEC exec;
    UB         snd[2];
    SZ         asz;

    snd[0] = reg;
    snd[1] = val;

    exec.sadr     = sadr;
    exec.snd_size = 2;
    exec.snd_data = snd;
    exec.rcv_size = 0;
    exec.rcv_data = NULL;

    return tk_swri_dev(i2c_dd, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
}

/* Reads `len` bytes in one go, starting at register `reg`, into buffer
 * `buf` (a pointer to caller-owned memory the results get written into -
 * the same "output via pointer" pattern used throughout this file).
 * "Burst" means one request that streams back several bytes instead of
 * one request per byte - see the AUTO_INC comment above for why that
 * matters here. */
static ER reg_read_burst(UW sadr, UB reg, UB *buf, INT len)
{
    T_I2C_EXEC exec;
    SZ         asz;

    exec.sadr     = sadr;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = len;
    exec.rcv_data = buf;

    return tk_swri_dev(i2c_dd, TDN_I2C_EXEC, &exec, sizeof(exec), &asz);
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

rc_result_t drv_imu_init(void)
{
    /* Open the shared I2C bus device by name. A non-positive handle
     * means the open failed (device missing/busy), so bail out with an
     * error rather than silently continuing to talk to nothing. */
    i2c_dd = tk_opn_dev(I2C_DEVNAME, TD_UPDATE);
    if (i2c_dd <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Turn the accelerometer on: 100Hz sampling, then +/-2g range. If
     * either write fails, the chip is probably not wired up correctly -
     * report it now instead of getting confusing all-zero readings
     * later. */
    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL_REG1, A_ODR_100HZ) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL_REG4, A_FS_2G_HR) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Magnetometer setup: rate, gain, then continuous mode. These are
     * `(void)`-cast to explicitly discard the return value - i.e. "yes,
     * I know this returns something, I'm deliberately not checking it"
     * (the magnetometer isn't required for the core hump/pitch feature,
     * so a failure here isn't treated as fatal the way accel setup is). */
    (void)reg_write(RC_I2C_ADDR_MAG, M_CRA_REG, M_ODR_75HZ);
    (void)reg_write(RC_I2C_ADDR_MAG, M_CRB_REG, M_GAIN_1_3);
    (void)reg_write(RC_I2C_ADDR_MAG, M_MR_REG,  M_CONTINUOUS);

    return RC_OK;
}

/* ------------------------------------------------------------------ *
 *  Reads
 * ------------------------------------------------------------------ */

rc_result_t drv_imu_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    /* raw bytes from the chip: [X low, X high, Y low, Y high, Z low, Z high] */
    UB b[6];

    /* Refuse to write through a NULL (invalid/empty) pointer - if the
     * caller passed no address to write into, we'd crash the moment we
     * tried `*x = ...`. Checking first turns a crash into a clean error
     * return. */
    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return RC_ERR_PARAM;
    }
    /* `A_OUT_X_L | AUTO_INC` uses bitwise OR (`|`) to combine the
     * register address with the auto-increment flag bit into one byte -
     * this is how you ask the chip "start streaming from A_OUT_X_L and
     * keep advancing register-by-register" in a single request, instead
     * of sending 6 separate one-byte read requests. */
    if (reg_read_burst(RC_I2C_ADDR_ACCEL, A_OUT_X_L | AUTO_INC, b, 6)
        != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Accelerometer is little endian and left justified: the 12 bit
     * result sits in the top bits, so shift down by 4.
     *
     * Step by step for X (Y and Z are identical):
     *   1. "Little endian" means the chip sends the LOW byte of the
     *      16-bit number first (b[0]) and the HIGH byte second (b[1]) -
     *      the opposite order you'd write the number on paper. So we
     *      shift b[1] left by 8 bits (`<< 8`, i.e. move it into the
     *      upper half of a 16-bit value) and combine it with b[0] using
     *      bitwise OR (`|`, which merges two sets of bits together) to
     *      rebuild the correct 16-bit value.
     *   2. "Left justified" means the sensor's actual 12-bit measurement
     *      was placed in the TOP 12 bits of that 16-bit value instead of
     *      the bottom 12 - so before we can use it as a normal number we
     *      shift everything right by 4 bits (`>> 4`) to slide it back
     *      down into place. Skip this shift and every reading would be
     *      16x too large.
     *   3. The final `(int16_t)` casts (converts) the result back to a
     *      signed 16-bit integer type, since tilt/acceleration can be
     *      negative (e.g. nose-down vs nose-up).
     */
    *x = (int16_t)(((int16_t)((uint16_t)b[1] << 8) | b[0]) >> 4);
    *y = (int16_t)(((int16_t)((uint16_t)b[3] << 8) | b[2]) >> 4);
    *z = (int16_t)(((int16_t)((uint16_t)b[5] << 8) | b[4]) >> 4);

    return RC_OK;
}

rc_result_t drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    /* raw bytes from the chip - note the different order, see below */
    UB b[6];

    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return RC_ERR_PARAM;
    }
    if (reg_read_burst(RC_I2C_ADDR_MAG, M_OUT_X_H, b, 6) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Magnetometer is BIG endian, and the axis order is X, Z, Y. Both of
     * those catch people out.
     *
     * "Big endian" is the opposite byte order from the accelerometer
     * above: this chip sends the HIGH byte first, so b[0] is shifted
     * left by 8 and b[1] is the low byte - no `>> 4` shift this time,
     * because (unlike the accelerometer) this chip's value already fills
     * the full 16 bits, it isn't left-justified.
     *
     * The axis order is also NOT X,Y,Z like you'd expect - the chip's
     * registers come out as X, then Z, then Y. Read them in the wrong
     * order and the car's "sideways" and "up/down" magnetic readings get
     * silently swapped, which would look like a bug in completely
     * unrelated code. */
    *x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    *z = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    *y = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

    return RC_OK;
}

/* Called on a timer (every RC_PERIOD_IMU_MS) from the sensing task. Reads
 * one fresh sample from both sensors, removes the calibration bias
 * measured at start-up, and broadcasts the result on the event bus for
 * sub_terrain.c (and anyone else) to react to. This is the only function
 * in this file that publishes an event - everything else just returns
 * numbers to its caller. */
void drv_imu_sample(void)
{
    rc_event_t evt;
    int16_t    ax = 0;
    int16_t    ay = 0;
    int16_t    az = 0;
    int16_t    mx = 0;
    int16_t    my = 0;
    int16_t    mz = 0;

    if (drv_imu_read_accel(&ax, &ay, &az) != RC_OK) {
        return;
    }
    /* `&mx` etc pass the ADDRESS of each local variable (the `&`
     * "address-of" operator) so drv_imu_read_mag can write its results
     * directly into them - same output-via-pointer pattern as above.
     * Magnetometer failure is ignored (`(void)`-discarded) rather than
     * aborting the whole sample, since accel/pitch is the feature that
     * matters most and shouldn't be held hostage by a flaky compass. */
    (void)drv_imu_read_mag(&mx, &my, &mz);

    ax -= bias_x;
    ay -= bias_y;
    /* Z keeps its 1 g offset on purpose: it is the reference the pitch
     * calculation needs. Only the level-sitting error is removed. */
    az -= bias_z;

    last_ax = ax;
    last_ay = ay;
    last_az = az;

    evt.id = RC_EVT_IMU_SAMPLE;
    evt.u.imu.acc_x = ax;
    evt.u.imu.acc_y = ay;
    evt.u.imu.acc_z = az;
    evt.u.imu.mag_x = mx;
    evt.u.imu.mag_y = my;
    evt.u.imu.mag_z = mz;

    (void)rc_event_publish(&evt);
}

/* See the header for why we average `n` samples instead of taking one:
 * it cancels random sensor noise and leaves just the fixed mounting/
 * manufacturing offset, which we store as "bias" and subtract from every
 * later reading. Must be called with the car sitting level and still. */
rc_result_t drv_imu_calibrate(uint16_t n)
{
    /* Sums use a wider 32-bit type (int32_t) than the individual
     * readings (int16_t) because adding up to `n` 16-bit values could
     * overflow a 16-bit total - the running sum needs more headroom than
     * any single sample. */
    int32_t sx = 0;
    int32_t sy = 0;
    int32_t sz = 0;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t i;
    uint16_t got = 0U;

    if (n == 0U) {
        return RC_ERR_PARAM;
    }

    for (i = 0U; i < n; i++) {
        if (drv_imu_read_accel(&x, &y, &z) == RC_OK) {
            sx += x;
            sy += y;
            sz += z;
            got++;
        }
        /* Yielding between samples, not spinning. This runs once at
         * start-up before the run begins, so the delay is free. */
        (void)tk_dly_tsk(5);
    }

    if (got == 0U) {
        return RC_ERR_HARDWARE;
    }

    bias_x = (int16_t)(sx / (int32_t)got);
    bias_y = (int16_t)(sy / (int32_t)got);

    /* Sitting level, Z reads 1 g. Store only the deviation from that so
     * the sign of pitch stays meaningful. */
    bias_z = (int16_t)((sz / (int32_t)got) - 1000);

    return RC_OK;
}

/* Turns the last accelerometer X reading into a tilt angle, without any
 * trigonometry functions and without a gyroscope. See sub_terrain.c and
 * the Buddy 4 guide for the full "why this works" explanation - short
 * version: gravity always pulls straight down, so when the car is level
 * all of that pull shows up on the Z axis; tilt the car nose-up and part
 * of that same pull rotates onto the X axis instead. More X reading
 * means more tilt. */
int16_t drv_imu_pitch_ddeg(void)
{
    /*
     *  Small-angle approximation: for pitch under about 25 degrees,
     *  asin(ax/g) is within a couple of percent of ax/g in radians, and
     *  a speed hump never exceeds that. Avoiding atan2 keeps this
     *  integer-only, which matters because the Cortex-M0+ has no FPU and
     *  a soft-float atan2 would cost thousands of cycles at 100 Hz.
     *
     *  ax is in mg, 1 g = 1000. radians = ax/1000.
     *  tenths of a degree = radians * 573.
     *
     *  Where does 573 come from? 1 radian = 180/pi degrees = 57.3
     *  degrees = 573 tenths-of-a-degree. So multiplying a radians value
     *  by 573 converts it straight to "tenths of a degree" in one step -
     *  it's just the degrees-per-radian conversion factor, scaled by 10
     *  so the result can stay a whole number (see drv_imu.h's note on
     *  fixed-point math). Division happens last so we keep as much
     *  precision as an int32_t allows before rounding down to int16_t.
     */
    return (int16_t)(((int32_t)last_ax * 573) / 1000);
}
