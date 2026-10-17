/*
 *  rc_period.c  -  see rc_period.h.
 */
#include "rc_prelude.h"
#include "rc_config.h"
#include "rc_period.h"

/* Cyclic handler: runs in timer-interrupt context, so it only wakes the
 * task. tk_wup_tsk() is legal from a handler. */
static void period_hdr(void *exinf)
{
    (void)tk_wup_tsk((ID)(intptr_t)exinf);
}

rc_result_t rc_period_start(ID tskid, uint32_t period_ms)
{
    T_CCYC ccyc = { 0 };
    ID     cycid;

    if ((tskid <= 0) || (period_ms == 0U)
        || ((period_ms % (uint32_t)RC_TICK_PERIOD_MS) != 0U)) {
        return RC_ERR_PARAM;
    }

    ccyc.exinf  = (void *)(intptr_t)tskid;
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)period_hdr;
    ccyc.cyctim = (RELTIM)period_ms;
    ccyc.cycphs = (RELTIM)period_ms;

    cycid = tk_cre_cyc(&ccyc);
    return (cycid > 0) ? RC_OK : RC_ERR_HARDWARE;
}
