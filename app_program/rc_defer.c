/*
 *  rc_defer.c
 */

#include "rc_prelude.h"
#include "rc_defer.h"
#include "rc_config.h"

#define DEFER_MAX       (16U)

typedef struct {
    rc_defer_drain_t drain;
    void            *ctx;
    bool             in_use;
} slot_t;

static slot_t   slots[DEFER_MAX];
static uint32_t n_slots;
static ID       defer_flgid;
static ID       defer_tskid;
static uint32_t drain_count;

static void defer_task(INT stacd, void *exinf)
{
    UINT     ptn;
    uint32_t i;
    ER       er;

    (void)stacd;
    (void)exinf;

    for (;;) {
        /* Wait for any registered bit. The timeout is a safety net: if a
         * flag were ever lost, pending work still drains within 50 ms
         * rather than stalling the whole car. */
        er = tk_wai_flg(defer_flgid, 0xFFFFU, TWF_ORW | TWF_CLR, &ptn, 50);

        if (er == E_TMOUT) {
            ptn = 0xFFFFU;      /* sweep everything once */
        } else if (er != E_OK) {
            continue;
        } else {
            drain_count++;
        }

        for (i = 0U; i < n_slots; i++) {
            if (slots[i].in_use && ((ptn & (1U << i)) != 0U)) {
                slots[i].drain(slots[i].ctx);
            }
        }
    }
}

rc_result_t rc_defer_init(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;
    uint32_t i;

    for (i = 0U; i < DEFER_MAX; i++) {
        slots[i].in_use = false;
        slots[i].drain  = NULL;
        slots[i].ctx    = NULL;
    }
    n_slots = 0U;

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;

    defer_flgid = tk_cre_flg(&cflg);
    if (defer_flgid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_DEFER;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = defer_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    defer_tskid = tk_cre_tsk(&ctsk);
    if (defer_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(defer_tskid, 0);

    return RC_OK;
}

int32_t rc_defer_register(rc_defer_drain_t drain, void *ctx)
{
    uint32_t i;

    if (drain == NULL) {
        return -1;
    }

    for (i = 0U; i < DEFER_MAX; i++) {
        if (!slots[i].in_use) {
            slots[i].drain  = drain;
            slots[i].ctx    = ctx;
            slots[i].in_use = true;
            if ((i + 1U) > n_slots) {
                n_slots = i + 1U;
            }
            return (int32_t)i;
        }
    }
    return -1;
}

void rc_defer_signal_i(int32_t handle)
{
    if ((handle >= 0) && (handle < (int32_t)DEFER_MAX)) {
        (void)tk_set_flg(defer_flgid, (UINT)(1U << (uint32_t)handle));
    }
}

uint32_t rc_defer_count(void)
{
    return drain_count;
}
