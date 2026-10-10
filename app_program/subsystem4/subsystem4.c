/*
 *	subsystem4.c
 *	Subsystem 4 -- Monitor: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_flg, tk_cre_mbf, tk_cre_alm
 */

#include "subsystem4.h"

EXPORT ID	mbf_summary;	/* IF-03 */
EXPORT ID	flg_events;	/* IF-03 */
EXPORT ID	alm_monitor;

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CALM	calm = {0};

	/* Event flag: a 32-bit pattern of bits.  Tasks wait for a bit (or a
	   combination of bits) to be set. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* allow several waiters */
	cflg.iflgptn = 0;			/* all bits clear to begin with */
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) {
		return flg_events;
	}

	/* Message buffer: COPIES variable-length data into a kernel ring
	   buffer.  Here it carries short text lines from subsystem 3. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * S4_SUMMARY_MAX;	/* total buffer size, bytes */
	cmbf.maxmsz = S4_SUMMARY_MAX;		/* largest single message */
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) {
		return mbf_summary;
	}

	/* Alarm handler: called ONCE, a given time after tk_sta_alm().
	   The monitor task re-arms it each time round its loop. */
	calm.almatr = TA_HLNG;
	calm.almhdr = (FP)monitor_alarm_handler;
	alm_monitor = tk_cre_alm(&calm);
	return (alm_monitor < E_OK) ? alm_monitor : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* No affinity: the scheduler may run it on either core */
	ID tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
	return (tid < E_OK) ? tid : E_OK;
}
