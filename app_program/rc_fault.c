/*
 *  rc_fault.c  -  the shared fault bitmask. See rc_fault.h.
 */
#include "rc_prelude.h"
#include "rc_fault.h"

static ID fault_flgid;

rc_result_t rc_fault_init(void)
{
    T_CFLG cflg;

    if (fault_flgid > 0) {
        return RC_OK;
    }

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;

    fault_flgid = tk_cre_flg(&cflg);
    if (fault_flgid <= E_OK) {
        fault_flgid = 0;
        return RC_ERR_HARDWARE;
    }
    return RC_OK;
}

void rc_fault_set(uint32_t bits)
{
    if ((fault_flgid > 0) && (bits != 0U)) {
        (void)tk_set_flg(fault_flgid, (UINT)bits);
    }
}

/* tk_clr_flg ANDs the pattern in, so pass the complement. */
void rc_fault_clear(uint32_t bits)
{
    if ((fault_flgid > 0) && (bits != 0U)) {
        (void)tk_clr_flg(fault_flgid, (UINT)~bits);
    }
}

uint32_t rc_fault_get(void)
{
    T_RFLG rflg;

    if ((fault_flgid <= 0) || (tk_ref_flg(fault_flgid, &rflg) != E_OK)) {
        return 0U;
    }
    return (uint32_t)rflg.flgptn & RC_FAULT_ALL;
}

#if APP_TEST
/* All 32 flag bits, for rc_fault_test.c, which uses bits 8..10. */
uint32_t rc_fault_get_raw(void)
{
    T_RFLG rflg;

    if ((fault_flgid <= 0) || (tk_ref_flg(fault_flgid, &rflg) != E_OK)) {
        return 0U;
    }
    return (uint32_t)rflg.flgptn;
}
#endif
