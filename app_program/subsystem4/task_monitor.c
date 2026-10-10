/*
 *	task_monitor.c
 *	Subsystem 4 -- the monitor task and its alarm handler.
 *
 *	APIs shown: tk_sta_alm, tk_stp_alm, alarm handler, tk_wai_flg,
 *	            tk_rcv_mbf, tk_loc_mtx/tk_unl_mtx, tk_ref_sem, tk_ref_mpf,
 *	            tk_ref_tsk, tk_sus_tsk, tk_rsm_tsk, tk_chg_pri,
 *	            tk_get_tid, tk_get_tim
 */

#include "subsystem4.h"
#include "../subsystem1/subsystem1.h"	/* IF-05: tid_led */
#include "../subsystem2/subsystem2.h"	/* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h"	/* IF-04: app_stats, mtx_stats */

EXPORT UW	s4_lines_received;

/*
 * Alarm handler: one-shot, interrupt context (same rules as a cyclic
 * handler: keep it short, never wait).
 */
EXPORT void monitor_alarm_handler(void *exinf)
{
	(void)exinf;
	tk_set_flg(flg_events, FLG_ALARM);
}

LOCAL const char *state_name(UINT tskstat)
{
	switch (tskstat) {
	case TTS_RUN:	return "RUN";
	case TTS_RDY:	return "READY";
	case TTS_WAI:	return "WAIT";
	case TTS_SUS:	return "SUSPEND";
	case TTS_WAS:	return "WAIT+SUSPEND";
	case TTS_DMT:	return "DORMANT";
	default:	return "?";
	}
}

EXPORT void monitor_task(INT stacd, void *exinf)
{
	UINT		flgptn;
	UB		text[S4_SUMMARY_MAX + 1];
	INT		len;
	APP_STATS	snap;
	T_RSEM		rsem;
	T_RMPF		rmpf;
	T_RTSK		rtsk;
	SYSTIM		now;
	UW		reports = 0;
	BOOL		led_paused = FALSE;
	ER		er;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"monitor: I am task %d\n", tk_get_tid());

	while (1) {
		/* 1. Arm the one-shot alarm, then wait for EITHER bit.
		      TWF_ORW = wake on any of the bits, TWF_CLR = clear them
		      on wake-up.  The extra timeout is just a safety net. */
		tk_sta_alm(alm_monitor, S4_ALARM_MS);
		er = tk_wai_flg(flg_events, FLG_ALARM | FLG_BATCH,
				TWF_ORW | TWF_CLR, &flgptn, S4_ALARM_MS * 2);
		tk_stp_alm(alm_monitor);	/* cancel it if a batch came first */

		if (er == E_TMOUT) {
			tm_printf((UB*)"monitor: nothing at all happened?\n");
			continue;
		}
		if (flgptn & FLG_ALARM) {
			tm_printf((UB*)"monitor: alarm -- no batch for %d ms\n",
				  S4_ALARM_MS);
		}

		/* 2. Drain the message buffer without waiting (TMO_POL).
		      tk_rcv_mbf returns the message length, or an error. */
		while ((len = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0) {
			text[len] = '\0';
			s4_lines_received++;
			tm_printf((UB*)"monitor: batch  %s\n", text);
		}

		/* 3. Copy the stats under subsystem 3's mutex, print outside
		      it: hold a mutex for as short a time as possible. */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		snap = app_stats;
		tk_unl_mtx(mtx_stats);

		tk_get_tim(&now);
		tk_ref_sem(sem_slots, &rsem);
		tk_ref_mpf(mpf_samples, &rmpf);
		tm_printf((UB*)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
			  "  free slots=%d  free blocks=%d\n",
			  now.lo, snap.received, snap.min, snap.max,
			  snap.received ? (INT)(snap.sum / (W)snap.received) : 0,
			  rsem.semcnt, (INT)rmpf.frbcnt);

		/* 4. Every few reports, pause or resume subsystem 1's LED task
		      (IF-05) and look at it with tk_ref_tsk. */
		if (++reports % S4_LED_TOGGLE_REPORTS == 0) {
			if (led_paused) {
				tk_rsm_tsk(tid_led);
				tk_chg_pri(tid_led, S1_PRI_LED);	/* back to normal */
			} else {
				tk_sus_tsk(tid_led);
				/* Changing the priority of a task: */
				tk_chg_pri(tid_led, S1_PRI_LED + 1);
			}
			led_paused = !led_paused;

			tk_ref_tsk(tid_led, &rtsk);
			tm_printf((UB*)"monitor: LED task %s, state=%s, pri=%d\n",
				  led_paused ? "paused" : "resumed",
				  state_name(rtsk.tskstat), rtsk.tskpri);
		}
	}
}
