/*!
 * @file  subsystem3.c
 * @brief Subsystem 3 -- line following and barcode decoding.
 *
 * subsystem3_init()  sets up the IR driver, the line follower and the
 *                    barcode decoder: their event-bus subscriptions and the
 *                    barcode edge interrupt (left masked). No thread yet.
 * subsystem3_start() nothing to start: the line pair is sampled every
 *                    RC_PERIOD_LINE_MS (5 ms, 200 Hz) by the shared Sense
 *                    task in ../rc_sense.c, which app_main.c starts after
 *                    every subsystem (report §3, single Sense task).
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
 * 1: enable line following as soon as subsystem3_start() runs.
 * 0: leave steering off until sub_line_enable(true) is called.
 * Keep 0 on the bench: once Buddy 2's motors are wired, 1 makes the car
 * drive away at power-up.
 */
#define S3_FOLLOW_AT_START (0)

/* Console messages. Kept as UB arrays because tm_printf() takes UB. */
static UB const msg_step_failed[] = "[S3] %s failed (%d)\n";
static UB const msg_not_started[] =
	"[S3] not started: subsystem3_init() failed\n";
static UB const name_drv_ir[]      = "drv_ir_init()";
static UB const name_sub_line[]    = "sub_line_init()";
static UB const name_sub_barcode[] = "sub_barcode_init()";
#if (S3_FOLLOW_AT_START)
static UB const name_line_enable[] = "sub_line_enable()";
#endif

/* True once subsystem3_init() has set everything up. */
static bool b_init_ok = false;

static bool step_ok (rc_result_t result, UB const * p_name);

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
 * @brief  Start subsystem 3. S3 has no thread of its own: the shared Sense
 *         task (../rc_sense.c) calls drv_ir_sample_line() every 5 ms.
 *
 * @return E_OK on success, E_OBJ if subsystem3_init() failed.
 */
EXPORT ER subsystem3_start (void)
{
	ER result = E_OBJ;

	if (!b_init_ok)
	{
		tm_printf(msg_not_started);
	}
	else
	{
		result = E_OK;
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

/*** end of file ***/