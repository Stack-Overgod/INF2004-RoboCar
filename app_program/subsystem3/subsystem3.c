/*!
 * @file  subsystem3.c
 * @brief Subsystem 3 -- line following and barcode decoding.
 *
 * subsystem3_init()  sets up the IR driver, the line follower and the
 *                    barcode decoder: their event-bus subscriptions and the
 *                    barcode edge interrupt (left masked). No thread yet.
 * subsystem3_start() starts the sense thread, which samples the two line
 *                    sensors every RC_PERIOD_LINE_MS (5 ms, 200 Hz).
 *
 * Both need the shared layer (rc_defer, rc_event, rc_gpioirq) running
 * first. app_main.c starts it in "Phase 0", before any subsystem.
 *
 * Nothing here turns on steering or barcode reading. That is the job of
 * the mission logic (sub_nav, not yet ported) or of a test, through
 * sub_line_enable() and sub_barcode_arm(). See S3_FOLLOW_AT_START.
 *
 * NOTE: Kernel types and calls (ID, ER, INT, UB, T_CTSK, tk_*) keep their
 * kernel spelling; that is required interop, not a naming deviation.
 */

#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "subsystem3.h"
#include "../rc_config.h"
#include "drv_ir.h"
#include "sub_line.h"
#include "sub_barcode.h"

/*
 * Sense thread priority. RC_PRI_SENSE (7) is the framework's slot for the
 * sensor-sampling thread: above telemetry and the slow dispatcher, so line
 * samples stay on time. TEAM.md's template band for S3 is 16-17; record
 * this thread in TEAM.md section 4.
 */
#define S3_PRI_SENSE (RC_PRI_SENSE)

/*
 * 1: enable line following as soon as the sense thread starts.
 * 0: leave steering off until sub_line_enable(true) is called.
 * Keep 0 on the bench: once Buddy 2's motors are wired, 1 makes the car
 * drive away at power-up.
 */
#define S3_FOLLOW_AT_START (0)

/* Console messages. Kept as UB arrays because tm_printf() takes UB. */
static UB const msg_step_failed[] = "[S3] %s failed (%d)\n";
static UB const msg_not_started[] =
	"[S3] sense thread not started: subsystem3_init() failed\n";
static UB const name_drv_ir[]      = "drv_ir_init()";
static UB const name_sub_line[]    = "sub_line_init()";
static UB const name_sub_barcode[] = "sub_barcode_init()";
#if (S3_FOLLOW_AT_START)
static UB const name_line_enable[] = "sub_line_enable()";
#endif

/* Kernel handle of the sense thread; 0 until subsystem3_start() runs. */
static ID h_sense_thread = 0;

/* True once subsystem3_init() has set everything up. */
static bool b_init_ok = false;

static bool step_ok (rc_result_t result, UB const * p_name);
static void sense_thread (INT stacd, void * p_exinf);

/*!
 * @brief  Create subsystem 3's kernel objects. Does not start threads.
 *
 * Order matters: drv_ir first (it owns the pins and the barcode IRQ),
 * then the two consumers of its events.
 *
 * @return E_OK on success, E_SYS if any step failed.
 */
EXPORT ER subsystem3_init (void)
{
	ER result = E_SYS;

	/* Each step runs only if the one before it worked; step_ok() prints
	 * the first failure. */
	b_init_ok = step_ok(drv_ir_init(), name_drv_ir);
	if (b_init_ok)
	{
		b_init_ok = step_ok(sub_line_init(), name_sub_line);
	}
	if (b_init_ok)
	{
		b_init_ok = step_ok(sub_barcode_init(), name_sub_barcode);
	}
	if (b_init_ok)
	{
		result = E_OK;
	}

	return result;
}

/*!
 * @brief  Create and start subsystem 3's threads.
 *
 * The stack is RC_STACK_SZ, like the framework's own threads: the sample
 * path goes through the event bus, and SMP kernel paths are deep.
 *
 * @return E_OK on success, E_OBJ if subsystem3_init() failed, or the
 *         kernel error code from tk_cre_tsk() / tk_sta_tsk().
 */
EXPORT ER subsystem3_start (void)
{
	T_CTSK ctsk   = { 0 };
	ER     result = E_OBJ;

	if (!b_init_ok)
	{
		tm_printf(msg_not_started);
	}
	else
	{
		ctsk.exinf   = NULL;
		ctsk.tskatr  = TA_HLNG | TA_RNG3;
		ctsk.task    = sense_thread;
		ctsk.itskpri = S3_PRI_SENSE;
		ctsk.stksz   = RC_STACK_SZ;

		h_sense_thread = tk_cre_tsk(&ctsk);
		if (h_sense_thread < E_OK)
		{
			result         = h_sense_thread;
			h_sense_thread = 0;
		}
		else
		{
			result = tk_sta_tsk(h_sense_thread, 0);
		}
	}

#if (S3_FOLLOW_AT_START)
	if ((E_OK == result) && (!step_ok(sub_line_enable(true), name_line_enable)))
	{
		result = E_SYS;
	}
#endif

	return result;
}

/*!
 * @brief  Print a readable message if one set-up step failed.
 * @param  result  result of the step
 * @param  p_name  name of the step, for the message
 * @return true if the step succeeded.
 */
static bool step_ok (rc_result_t result, UB const * p_name)
{
	bool b_ok = (RC_OK == result);

	if (!b_ok)
	{
		tm_printf(msg_step_failed, p_name, result);
	}

	return b_ok;
}

/*!
 * @brief  Sense thread: samples the line sensors every RC_PERIOD_LINE_MS.
 *
 * drv_ir_sample_line() reads GP16/GP6 and publishes RC_EVT_LINE_SAMPLE,
 * which sub_line (fast lane) and Buddy 1's telemetry (slow lane) receive.
 * tk_dly_tsk() can only fail if the thread is released early, which
 * nothing does, so its result is not checked.
 *
 * @param  stacd    start code (unused)
 * @param  p_exinf  extended information (unused)
 */
static void sense_thread (INT stacd, void * p_exinf)
{
	/* NOTE: (void) marks parameters the kernel's task entry type needs
	 * but this thread does not use. */
	(void)stacd;
	(void)p_exinf;

	for (;;)
	{
		tk_dly_tsk(RC_PERIOD_LINE_MS);
		drv_ir_sample_line();
	}
}

/*** end of file ***/