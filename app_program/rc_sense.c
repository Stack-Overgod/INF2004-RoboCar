/*
 *  rc_sense.c  -  the shared Sense task. See rc_sense.h.
 *
 *  Moved out of subsystem3.c (Buddy 3's sense_thread) so that neither
 *  subsystem depends on the other: subsystem3.c no longer includes
 *  Buddy 4's drv_imu.h, and the IMU sampling no longer lives in Buddy 3's
 *  file. Line and IMU are both sampled every 10 ms (NFR1, revised from
 *  5 ms by the team to fit the 10 ms kernel tick).
 */
#include "rc_prelude.h"
#include "rc_config.h"
#include "rc_sense.h"
#include "rc_period.h"
#include "rc_time.h"
#include "subsystem3/drv_ir.h"
#include "subsystem4/drv_imu.h"

/* IMU samples once every this many line ticks: 10 ms / 10 ms = 1. */
#define SENSE_IMU_EVERY  ((uint32_t)(RC_PERIOD_IMU_MS / RC_PERIOD_LINE_MS))

#if (RC_PERIOD_IMU_MS % RC_PERIOD_LINE_MS) != 0
#error "RC_PERIOD_IMU_MS must be a whole multiple of RC_PERIOD_LINE_MS"
#endif

static ID sense_tskid;
static volatile uint32_t sense_overruns;   /* missed periods, for timing tests */

#if APP_TEST
/* A self-test is injecting fake line/IMU samples. See rc_sense_test_hold(). */
static volatile bool test_hold;
static volatile uint32_t sense_ticks;      /* loop passes, for S3-U05 */
static volatile uint32_t line_max_us;      /* longest drv_ir_sample_line() */
static volatile uint32_t imu_max_us;       /* longest drv_imu_sample() */

void rc_sense_test_hold(bool hold)
{
    test_hold = hold;
}

uint32_t rc_sense_ticks(void)
{
    return sense_ticks;
}

void rc_sense_max_us(uint32_t *line_us, uint32_t *imu_us)
{
    *line_us = line_max_us;
    *imu_us  = imu_max_us;
}

/* Run fn and keep the longest time it took in *max_us. */
static void timed(void (*fn)(void), volatile uint32_t *max_us)
{
    uint32_t t0 = rc_time_us();
    uint32_t dt;

    fn();
    dt = rc_time_us() - t0;
    if (dt > *max_us) {
        *max_us = dt;
    }
}
#define SENSE_LINE()  timed(drv_ir_sample_line, &line_max_us)
#define SENSE_IMU()   timed(drv_imu_sample, &imu_max_us)
#else
#define SENSE_LINE()  drv_ir_sample_line()
#define SENSE_IMU()   drv_imu_sample()
#endif

/*
 * Wakes every RC_PERIOD_LINE_MS. drv_ir_sample_line() publishes
 * RC_EVT_LINE_SAMPLE (sub_line on FAST, telemetry on SLOW). On every
 * SENSE_IMU_EVERY-th tick drv_imu_sample() publishes RC_EVT_IMU_SAMPLE
 * (sub_terrain); it returns at once while the IMU is not open, so a
 * missing IMU does not delay the line samples. tk_dly_tsk() can only fail
 * if the task is released early, which nothing does.
 */
static void sense_task(INT stacd, void *exinf)
{
    uint32_t tick = 0U;

    (void)stacd;
    (void)exinf;

    for (;;) {
        sense_overruns += rc_period_wait();
        tick++;
#if APP_TEST
        sense_ticks++;
        if (test_hold) {
            continue;       /* keep the cadence, publish nothing */
        }
#endif

        SENSE_LINE();

        if ((tick % SENSE_IMU_EVERY) == 0U) {
            SENSE_IMU();
        }
    }
}

rc_result_t rc_sense_start(void)
{
    T_CTSK ctsk = { 0 };

    if (sense_tskid > 0) {
        return RC_OK;                   /* already running */
    }

    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_SENSE;
    ctsk.stksz   = RC_STACK_SZ;         /* event-bus publish + SMP paths */
    ctsk.task    = sense_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    sense_tskid = tk_cre_tsk(&ctsk);
    if (sense_tskid < E_OK) {
        sense_tskid = 0;
        return RC_ERR_HARDWARE;
    }
    if (tk_sta_tsk(sense_tskid, 0) != E_OK) {
        (void)tk_del_tsk(sense_tskid);
        sense_tskid = 0;
        return RC_ERR_HARDWARE;
    }
    if (rc_period_start(sense_tskid, RC_PERIOD_LINE_MS) != RC_OK) {
        return RC_ERR_HARDWARE;
    }
    return RC_OK;
}

uint32_t rc_sense_overruns(void)
{
    return sense_overruns;
}
