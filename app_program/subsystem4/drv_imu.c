/*
 *  drv_imu.c
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "../../device/include/dev_i2c.h"
#include "drv_imu.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_fault.h"

/* Sensor registers.
 *
 * "Register" here just means a numbered memory slot inside the sensor
 * chip that you can write a setting into or read a measurement out of -
 * like a row in a tiny built-in spreadsheet. `#define` gives each slot
 * number a readable name so the code below never has bare hex constants
 * whose meaning you'd have to look up in a datasheet every time. */
/* LSM303D register map. Unlike the older LSM303DLHC this is a SINGLE die
 * with ONE I2C address (0x1D with SA0 high) for both accelerometer and
 * magnetometer, a different register layout, and 16-bit right-justified
 * accelerometer output (not the DLHC's 12-bit left-justified). */
#define D_WHO_AM_I      (0x0FU)  /* identity register */
#define D_WHO_ID        (0x49U)  /* value WHO_AM_I returns on an LSM303D */

#define A_CTRL1         (0x20U)  /* accel: ODR, BDU, axis enables */
#define A_CTRL2         (0x21U)  /* accel: anti-alias BW, full scale */
#define A_OUT_X_L       (0x28U)  /* accel: first of 6 output regs (X low) */

#define M_CTRL5         (0x24U)  /* mag: resolution + ODR */
#define M_CTRL6         (0x25U)  /* mag: full scale */
#define M_CTRL7         (0x26U)  /* mag: conversion mode */
#define M_OUT_X_L       (0x08U)  /* mag: first of 6 output regs (X low) */

/* A_CTRL1 = 0x6F: AODR=0110 (100 Hz), BDU=1 (block data update, so a burst
 * read never mixes the old and new halves of a 16-bit sample), AZEN=AYEN=
 * AXEN=1 (all three axes enabled). */
#define A_CTRL1_VAL     (0x6FU)
/* A_CTRL2 = 0x00: AFS=000 (+/-2 g full scale). At +/-2 g the LSM303D
 * resolution is 0.061 mg/LSB - see drv_imu_read_accel for the scaling. */
#define A_CTRL2_VAL     (0x00U)

/* M_CTRL5 = 0x74: M_RES=11 (high resolution), M_ODR=101 (100 Hz). */
#define M_CTRL5_VAL     (0x74U)
/* M_CTRL6 = 0x20: MFS=01 (+/-4 gauss full scale). */
#define M_CTRL6_VAL     (0x20U)
/* M_CTRL7 = 0x00: MD=00 (continuous-conversion mode). */
#define M_CTRL7_VAL     (0x00U)

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

/* 1: print raw readings and CTRL1 / STATUS_A to the console once a
 * second (bench check). Keep 0 in the repo; sub_terrain.c has its own. */
#ifndef S4_IMU_PRINT
#define S4_IMU_PRINT    (0)
#endif

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

/* True once drv_imu_init() and drv_imu_calibrate() have both succeeded.
 * drv_imu_sample() does nothing until then, so the Sense thread (already
 * running when subsystem4_start() calibrates) never shares the I2C bus
 * with the calibration loop, and never polls a missing chip. */
static volatile bool imu_ready;

#if APP_TEST
/* subsystem4_test() feeds sub_terrain fake samples; real ones arriving in
 * between would break its hump sequence. See drv_imu_test_hold(). */
static volatile bool test_hold;

void drv_imu_test_hold(bool hold)
{
    test_hold = hold;
}
#endif

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
    UB snd[2];
    SZ asz;

    snd[0] = reg;
    snd[1] = val;

    /* A register write is a pure write with no read phase. The driver's
     * EXEC command cannot express that - it rejects rcv_size == 0 with
     * E_PAR, because EXEC is a combined write-then-read (repeated start).
     * So use the plain WRITE path instead: the device "start" argument is
     * the I2C slave address and the buffer carries [reg, val]. */
    return tk_swri_dev(i2c_dd, (W)sadr, snd, 2, &asz);
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

    /* Confirm the chip before configuring it. The LSM303D answers 0x49 on
     * WHO_AM_I. A failed read means nothing is ACKing on the bus; a
     * different value means the board is some other IMU and this register
     * map does not apply - either way, log it and bail. */
    {
        UB id = 0U;
        if (reg_read_burst(RC_I2C_ADDR_ACCEL, D_WHO_AM_I, &id, 1) != E_OK) {
            tm_printf((UB *)"[S4] IMU: no ACK at 0x%02x (WHO_AM_I read failed)\n",
                      (unsigned)RC_I2C_ADDR_ACCEL);
            return RC_ERR_HARDWARE;
        }
        if (id != D_WHO_ID) {
            tm_printf((UB *)"[S4] IMU: WHO_AM_I=0x%02x, expected 0x%02x "
                      "(LSM303D) - wrong chip?\n",
                      (unsigned)id, (unsigned)D_WHO_ID);
            return RC_ERR_HARDWARE;
        }
        tm_printf((UB *)"[S4] IMU: LSM303D detected (WHO_AM_I=0x49)\n");
    }

    /* Accelerometer: full scale first (CTRL2), then ODR + enables (CTRL1).
     * A failed write means the chip stopped responding - report it rather
     * than return confusing all-zero readings later. */
    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL2, A_CTRL2_VAL) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    if (reg_write(RC_I2C_ADDR_ACCEL, A_CTRL1, A_CTRL1_VAL) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* Magnetometer: resolution/ODR, full scale, then continuous mode. On
     * the LSM303D these share the accelerometer's address. (void)-cast
     * because the magnetometer is telemetry-only - a failure here must not
     * take down the hump/pitch feature, which needs only the accel. */
    (void)reg_write(RC_I2C_ADDR_MAG, M_CTRL5, M_CTRL5_VAL);
    (void)reg_write(RC_I2C_ADDR_MAG, M_CTRL6, M_CTRL6_VAL);
    (void)reg_write(RC_I2C_ADDR_MAG, M_CTRL7, M_CTRL7_VAL);

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

    /* LSM303D accelerometer output is 16-bit, little-endian (low byte
     * first) and RIGHT-justified - the whole 16-bit value is used, with
     * NO >>4 shift (the >>4 was for the DLHC's 12-bit left-justified
     * format). Rebuild each axis from its low+high byte, then scale to
     * milli-g: at +/-2 g full scale the sensitivity is 0.061 mg/LSB, so
     * mg = raw * 61 / 1000. The product is held in int32 because raw
     * (+/-32767) * 61 overflows 16 bits. Downstream (pitch, RC_IMPACT_MG)
     * expects mg with 1 g ~= 1000. */
    *x = (int16_t)(((int32_t)(int16_t)((uint16_t)b[1] << 8 | b[0]) * 61) / 1000);
    *y = (int16_t)(((int32_t)(int16_t)((uint16_t)b[3] << 8 | b[2]) * 61) / 1000);
    *z = (int16_t)(((int32_t)(int16_t)((uint16_t)b[5] << 8 | b[4]) * 61) / 1000);

    return RC_OK;
}

rc_result_t drv_imu_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    /* raw bytes from the chip - note the different order, see below */
    UB b[6];

    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return RC_ERR_PARAM;
    }
    if (reg_read_burst(RC_I2C_ADDR_MAG, M_OUT_X_L | AUTO_INC, b, 6) != E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* LSM303D magnetometer output is 16-bit, LITTLE-endian (low byte
     * first), in natural X, Y, Z order - different from the DLHC, which
     * was big-endian and X, Z, Y. Returned as raw counts: only telemetry
     * reads the magnetometer, so no unit scaling is applied. */
    *x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    *y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    *z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

    return RC_OK;
}

/* True while RC_FAULT_IMU is set. Only the Sense task touches it, so the
 * fault flag is only written when the state changes. */
static bool imu_fault_on;

/* Frozen-output check (drv_imu_sample): the last raw reading and how many
 * samples in a row have repeated it exactly. 50 = 0.5 s. */
#define IMU_STUCK_N     (50U)
static int16_t  prev_ax;
static int16_t  prev_ay;
static int16_t  prev_az;
static uint32_t same_n;

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

    if (!imu_ready) {
        /* not initialised / not calibrated / not fitted (FR15) */
        if (!imu_fault_on) {
            rc_fault_set(RC_FAULT_IMU);
            imu_fault_on = true;
        }
        return;
    }
#if APP_TEST
    if (test_hold) {
        return;     /* a self-test is feeding fake samples */
    }
#endif

    if (drv_imu_read_accel(&ax, &ay, &az) != RC_OK) {
        if (!imu_fault_on) {
            rc_fault_set(RC_FAULT_IMU);
            imu_fault_on = true;
        }
        return;
    }
    /* Frozen output: a converting accelerometer never returns the same
     * three readings IMU_STUCK_N samples running (its noise alone moves
     * them by a few mg), yet the chip can stop converting and keep
     * answering with its last sample. Flag it, restart the accelerometer
     * (power down, then CTRL2 / CTRL1 again) every IMU_STUCK_N samples
     * until the readings move, and publish nothing meanwhile. */
    if ((ax == prev_ax) && (ay == prev_ay) && (az == prev_az)) {
        if (++same_n >= IMU_STUCK_N) {
            same_n = 0U;
            if (!imu_fault_on) {
                tm_printf((UB *)"[S4] IMU: readings frozen, restarting"
                          " the accelerometer\n");
                rc_fault_set(RC_FAULT_IMU);
                imu_fault_on = true;
            }
            (void)reg_write(RC_I2C_ADDR_ACCEL, A_CTRL1, 0x00U);
            (void)reg_write(RC_I2C_ADDR_ACCEL, A_CTRL2, A_CTRL2_VAL);
            (void)reg_write(RC_I2C_ADDR_ACCEL, A_CTRL1, A_CTRL1_VAL);
        }
        if (imu_fault_on) {
            return;
        }
    } else {
        same_n = 0U;
    }
    prev_ax = ax;
    prev_ay = ay;
    prev_az = az;
    if (imu_fault_on) {
        rc_fault_clear(RC_FAULT_IMU);
        imu_fault_on = false;
    }
    /* `&mx` etc pass the ADDRESS of each local variable (the `&`
     * "address-of" operator) so drv_imu_read_mag can write its results
     * directly into them - same output-via-pointer pattern as above.
     * Magnetometer failure is ignored (`(void)`-discarded) rather than
     * aborting the whole sample, since accel/pitch is the feature that
     * matters most and shouldn't be held hostage by a flaky compass. */
    (void)drv_imu_read_mag(&mx, &my, &mz);

#if S4_IMU_PRINT
    {
        /* console only, once a second: raw (uncorrected) accel, CTRL1 as
         * the chip holds it now, and STATUS_A (0x27: low nibble = new
         * X/Y/Z data ready). A chip that stopped converting keeps its
         * last sample and shows no new data. */
        static uint32_t n_print;
        UB ctrl1 = 0U;
        UB status = 0U;
        ER e1;
        ER e2;

        if (++n_print >= 100U) {
            n_print = 0U;
            e1 = reg_read_burst(RC_I2C_ADDR_ACCEL, A_CTRL1, &ctrl1, 1);
            e2 = reg_read_burst(RC_I2C_ADDR_ACCEL, 0x27U, &status, 1);
            tm_printf((UB *)"[imu] raw ax %d ay %d az %d mg  ctrl1 0x%02x"
                      " (%d) status 0x%02x (%d)\n", (INT)ax, (INT)ay,
                      (INT)az, (INT)ctrl1, (INT)e1, (INT)status, (INT)e2);
        }
    }
#endif

    ax -= bias_x;
    ay -= bias_y;
    /* Z keeps its 1 g offset on purpose: it is the reference the pitch
     * calculation needs. Only the level-sitting error is removed. */
    az -= bias_z;

    last_ax = ax;
    last_ay = ay;
    last_az = az;

    evt.id = RC_EVT_IMU_SAMPLE;
    /* 12-bit milli-g (+/-2 g full scale), clamped. The magnetometer is
     * read above but not put on the bus: nothing subscribes to it. */
    evt.u.imu.acc_x = RC_SAT_S(ax, RC_BITS_ACC);
    evt.u.imu.acc_y = RC_SAT_S(ay, RC_BITS_ACC);
    evt.u.imu.acc_z = RC_SAT_S(az, RC_BITS_ACC);

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

    imu_ready = true;       /* drv_imu_sample() may run from now on */
    return RC_OK;
}

/* FR15: false means IMU_UNAVAILABLE - init or calibration failed, so no
 * IMU_SAMPLE events (and no hump / impact detection) this run. */
bool drv_imu_available(void)
{
    return imu_ready;
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
