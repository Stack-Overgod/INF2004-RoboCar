/*
 *  rc_period.h  -  exact periodic wake-ups on the 10 ms kernel tick.
 *
 *  Why not tk_dly_tsk(): the kernel adds one whole tick to every relative
 *  wait so it never returns early (kernel/tkernel/timer.c). With
 *  CNF_TIMER_PERIOD 10, tk_dly_tsk(10) in a loop runs every 20 ms and
 *  tk_dly_tsk(20) every 30 ms. A cyclic handler is scheduled on absolute
 *  times (previous + period), so its period is exact.
 *
 *  Usage: create and start the task, then call rc_period_start(). The
 *  task's loop waits with rc_period_wait(), which also reports how many
 *  periods it missed since the last call (an overrun counter for the
 *  timing tests).
 */
#ifndef RC_PERIOD_H
#define RC_PERIOD_H

#include "rc_prelude.h"
#include "rc_types.h"

/* Wake `tskid` every `period_ms`. period_ms must be a whole multiple of
 * RC_TICK_PERIOD_MS. Uses one cyclic handler (CNF_MAX_CYCID is 12). */
rc_result_t rc_period_start(ID tskid, uint32_t period_ms);

/* Call at the top of the task loop. Sleeps until the next period and
 * returns the number of extra wake-ups that piled up (0 = on time). */
static inline uint32_t rc_period_wait(void)
{
    INT missed;

    (void)tk_slp_tsk(TMO_FEVR);
    missed = tk_can_wup(TSK_SELF);
    return (missed > 0) ? (uint32_t)missed : 0U;
}

#endif /* RC_PERIOD_H */
