/*
 *  rc_wdog.c  -  see rc_wdog.h.
 *
 *  Registers from the RP2040 datasheet sections 4.7 (watchdog) and 2.13
 *  (power-on state machine).
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "rc_wdog.h"
#include "rc_config.h"
#include "rc_fault.h"

#define WDOG_BASE           (0x40058000UL)
#define WDOG_CTRL           (WDOG_BASE + 0x00UL)
#define WDOG_LOAD           (WDOG_BASE + 0x04UL)
#define WDOG_REASON         (WDOG_BASE + 0x08UL)
#define WDOG_SCRATCH0       (WDOG_BASE + 0x0CUL)
#define WDOG_CTRL_ENABLE    (1UL << 30)
#define WDOG_CTRL_PAUSE_DBG (7UL << 24)     /* pause while a debugger halts */
#define WDOG_REASON_TIMER   (1UL << 0)
#define WDOG_REASON_FORCE   (1UL << 1)

/* Which blocks a watchdog reset resets: all but the two oscillators. */
#define PSM_WDSEL           (0x40010008UL)
#define PSM_WDSEL_ALL_BUT_OSC (0x0001FFFCUL)

/* The counter counts down twice per 1 us tick (RP2040-E1), so it is
 * loaded with twice the time; 24 bits make 8.38 s the longest. */
#define WDOG_LOAD_VAL       ((uint32_t)RC_WDOG_MS * 1000UL * 2UL)
#if (RC_WDOG_MS * 2000UL) > 0xFFFFFFUL
#error "RC_WDOG_MS longer than the watchdog can count (8388 ms)"
#endif

/* SCRATCH0 survives a watchdog reset: the supervisor keeps the tasks that
 * missed their last check there, tagged so a power-on value is not read
 * as one. */
#define MISS_TAG            (0x57D00000UL)
#define MISS_TAG_MASK       (0xFFF00000UL)

/* One byte per task, so a check-in is a single store (no read-modify-
 * write that two cores could interleave). */
static volatile uint8_t alive[RC_WDOG_N];

static const char *const who_name[RC_WDOG_N] = { "Sense", "Motion", "FAST" };

void rc_wdog_checkin(rc_wdog_who_t who)
{
    if ((uint32_t)who < (uint32_t)RC_WDOG_N) {
        alive[who] = 1U;
    }
}

/* Cyclic handler (timer interrupt), every RC_WDOG_CHECK_MS: reload the
 * watchdog only if every task checked in since the last visit; otherwise
 * leave it to run down and note who is missing. */
static void wdog_supervisor(void *exinf)
{
    uint32_t missing = 0U;
    uint32_t i;

    (void)exinf;
    for (i = 0U; i < (uint32_t)RC_WDOG_N; i++) {
        if (alive[i] == 0U) {
            missing |= (1UL << i);
        }
        alive[i] = 0U;
    }
    if (missing == 0U) {
        out_w(WDOG_LOAD, WDOG_LOAD_VAL);
        out_w(WDOG_SCRATCH0, MISS_TAG);
    } else {
        out_w(WDOG_SCRATCH0, MISS_TAG | missing);
    }
}

/* The watchdog's REASON says TIMER for our own timeout and also for the
 * boot ROM's reboot after a UF2 copy, so the tag in SCRATCH0 tells them
 * apart: no tag = not armed by us (power-on, flash). Tag with tasks
 * missing = a task stopped checking in. Tag with none missing = the
 * supervisor itself stopped running (the core with the tick hung or
 * faulted). */
void rc_wdog_boot(void)
{
    uint32_t reason  = in_w(WDOG_REASON);
    uint32_t scratch = in_w(WDOG_SCRATCH0);
    uint32_t missing = scratch & ((1UL << RC_WDOG_N) - 1UL);
    bool     tagged  = ((scratch & MISS_TAG_MASK) == MISS_TAG);
    bool     was_reset;
    uint32_t i;

    was_reset = ((reason & WDOG_REASON_TIMER) != 0U) && tagged;
    if (!was_reset) {
        tm_printf((UB *)"[wdog] boot: %s\n",
                  ((reason & (WDOG_REASON_TIMER | WDOG_REASON_FORCE)) != 0U)
                      ? "reboot by boot ROM / flash" : "power-on / reset");
        out_w(WDOG_SCRATCH0, 0U);
        return;
    }
    tm_printf((UB *)"[wdog] boot: REBOOTED BY WATCHDOG, ");
    if (missing != 0U) {
        tm_printf((UB *)"stopped:");
        for (i = 0U; i < (uint32_t)RC_WDOG_N; i++) {
            if ((missing & (1UL << i)) != 0U) {
                tm_printf((UB *)" %s", who_name[i]);
            }
        }
        tm_printf((UB *)"\n");
    } else {
        tm_printf((UB *)"supervisor stopped (system hang)\n");
    }
    rc_fault_set(RC_FAULT_WDOG_RESET);      /* cleared by START (sub_nav) */
}

rc_result_t rc_wdog_start(void)
{
    T_CCYC   ccyc = { 0 };
    ID       cycid;
    uint32_t i;

    for (i = 0U; i < (uint32_t)RC_WDOG_N; i++) {
        alive[i] = 0U;
    }
    out_w(WDOG_SCRATCH0, MISS_TAG);
    out_w(PSM_WDSEL, PSM_WDSEL_ALL_BUT_OSC);
    out_w(WDOG_LOAD, WDOG_LOAD_VAL);
    out_w(WDOG_CTRL, WDOG_CTRL_ENABLE | WDOG_CTRL_PAUSE_DBG);

    ccyc.exinf  = NULL;
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)wdog_supervisor;
    ccyc.cyctim = (RELTIM)RC_WDOG_CHECK_MS;
    ccyc.cycphs = (RELTIM)RC_WDOG_CHECK_MS;
    cycid = tk_cre_cyc(&ccyc);
    if (cycid <= 0) {
        out_w(WDOG_CTRL, 0U);               /* no supervisor: no watchdog */
        return RC_ERR_HARDWARE;
    }
    tm_printf((UB *)"[wdog] on: %u ms, checked every %u ms\n",
              (unsigned)RC_WDOG_MS, (unsigned)RC_WDOG_CHECK_MS);
    return RC_OK;
}
