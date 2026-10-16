/*
 *  rc_sense.c  -  the shared Sense task. See rc_sense.h.
 *
 *  Moved out of subsystem3.c (Buddy 3's sense_thread) so that neither
 *  subsystem depends on the other: subsystem3.c no longer includes
 *  Buddy 4's drv_imu.h, and the IMU sampling no longer lives in Buddy 3's
 *  file. The timing is unchanged: line every 5 ms, IMU every 2nd tick.
 */
#include "rc_prelude.h"
#include "rc_config.h"
#include "rc_sense.h"
#include "rc_period.h"
#include "subsystem3/drv_ir.h"
#include "subsystem4/drv_imu.h"

/* IMU samples once every this many line ticks: 10 ms / 5 ms = 2. */
#define SENSE_IMU_EVERY  ((uint32_t)(RC_PERIOD_IMU_MS / RC_PERIOD_LINE_MS))

#if (RC_PERIOD_IMU_MS % RC_PERIOD_LINE_MS) != 0
#error "RC_PERIOD_IMU_MS must be a whole multiple of RC_PERIOD_LINE_MS"
#endif

static ID sense_tskid;
static volatile uint32_t sense_overruns;   /* missed periods, for timing tests */

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
        drv_ir_sample_line();

        tick++;
        if ((tick % SENSE_IMU_EVERY) == 0U) {
            drv_imu_sample();
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
