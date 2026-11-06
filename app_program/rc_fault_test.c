/*
 *  rc_fault_test.c  -  shared-layer tests CR1-T2 / CR1-T3 (TEST builds only)
 *
 *  CR1-T2  the fault bitmask loses no update when both cores and an
 *          interrupt-level handler change bits at the same time.
 *          Task A (core 0) and task B (core 1) each set and clear their
 *          own bit 10,000 times and check after every call that their bit
 *          reads back as written. A cyclic handler (task-independent, like
 *          an ISR) sets a third bit whenever it is clear, and the test
 *          task counts and clears it: every handler set must be seen once.
 *          A plain `mask |= bit` on the M0+ (no LDREX/STREX) would fail
 *          this; the kernel event flag must not. Test bits 8..10 are
 *          outside the telemetry byte, so the run is invisible on MQTT.
 *  CR1-T3  Sense task timing with the ADC barcode read added: no missed
 *          10 ms periods, and the longest ADC sample + threshold.
 *
 *  Called from usermain() after the subsystem tests.
 */
#include "rc_prelude.h"
#include "app.h"
#include "rc_fault.h"
#include "rc_sense.h"
#include "test_report.h"

#if APP_TEST

#define T2_LOOPS        (10000U)
#define T2_BIT_A        (1U << 8)
#define T2_BIT_B        (1U << 9)
#define T2_BIT_ISR      (1U << 10)
#define T2_PRI          (14)

static volatile uint32_t t2_bad[2];         /* read-backs that were wrong */
static volatile bool     t2_done[2];        /* one flag per task: no shared RMW */
static volatile uint32_t t2_isr_sets;
static volatile bool     t2_isr_on;

static void t2_worker(INT stacd, void *exinf)
{
    const uint32_t bit = (stacd == 0) ? T2_BIT_A : T2_BIT_B;
    uint32_t i;

    (void)exinf;
    for (i = 0U; i < T2_LOOPS; i++) {
        rc_fault_set(bit);
        if ((rc_fault_get_raw() & bit) == 0U) {
            t2_bad[stacd]++;
        }
        rc_fault_clear(bit);
        if ((rc_fault_get_raw() & bit) != 0U) {
            t2_bad[stacd]++;
        }
        if ((i % 100U) == 99U) {
            tk_dly_tsk(1);          /* let the handler and the other core in */
        }
    }
    t2_done[stacd] = true;
    tk_ext_tsk();                           /* dormant; the test deletes it */
}

static void t2_handler(void *exinf)
{
    (void)exinf;
    if (t2_isr_on && ((rc_fault_get_raw() & T2_BIT_ISR) == 0U)) {
        t2_isr_sets++;
        rc_fault_set(T2_BIT_ISR);
    }
}

static ID t2_start(INT which, UW prc)
{
    T_CTSK ctsk = { 0 };
    ID tid;

    ctsk.task    = (FP)t2_worker;
    ctsk.itskpri = T2_PRI;
    ctsk.stksz   = TASK_STACK_SIZE;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;
    ctsk.assprc  = prc;
#else
    (void)prc;
#endif
    tid = tk_cre_tsk(&ctsk);
    if ((tid > 0) && (tk_sta_tsk(tid, which) != E_OK)) {
        (void)tk_del_tsk(tid);
        tid = 0;
    }
    return tid;
}

static bool check_t2(void)
{
    T_CCYC ccyc = { 0 };
    ID cyc;
    ID ta;
    ID tb;
    uint32_t seen = 0U;
    uint32_t waited = 0U;
    bool ok;

    rc_fault_clear(T2_BIT_A | T2_BIT_B | T2_BIT_ISR);
    t2_bad[0] = 0U;
    t2_bad[1] = 0U;
    t2_done[0] = false;
    t2_done[1] = false;
    t2_isr_sets = 0U;

    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)t2_handler;
    ccyc.cyctim = 10;
    ccyc.cycphs = 10;
    cyc = tk_cre_cyc(&ccyc);
    t2_isr_on = true;

#if TK_SUPPORT_SMP
    ta = t2_start(0, TP_PRC1);
    tb = t2_start(1, TP_PRC2);
#else
    ta = t2_start(0, 0);
    tb = t2_start(1, 0);
#endif

    while (!(t2_done[0] && t2_done[1]) && (waited < 20000U)) {
        if ((rc_fault_get_raw() & T2_BIT_ISR) != 0U) {
            seen++;
            rc_fault_clear(T2_BIT_ISR);
        }
        tk_dly_tsk(1);
        waited += 10U;
    }
    t2_isr_on = false;
    tk_dly_tsk(30);
    if ((rc_fault_get_raw() & T2_BIT_ISR) != 0U) {
        seen++;
        rc_fault_clear(T2_BIT_ISR);
    }
    if (cyc > 0) {
        (void)tk_del_cyc(cyc);
    }
    if (ta > 0) {
        (void)tk_ter_tsk(ta);           /* fails harmlessly if it exited */
        (void)tk_del_tsk(ta);
    }
    if (tb > 0) {
        (void)tk_ter_tsk(tb);
        (void)tk_del_tsk(tb);
    }

    tm_printf((UB *)"[TEST]   T2: A %lu x2 calls, %lu bad; B %lu bad; handler"
              " set %lu, seen %lu; %lu ms\n", (unsigned long)T2_LOOPS,
              (unsigned long)t2_bad[0], (unsigned long)t2_bad[1],
              (unsigned long)t2_isr_sets, (unsigned long)seen,
              (unsigned long)waited);
    ok = (ta > 0) && (tb > 0) && (cyc > 0) && t2_done[0] && t2_done[1] &&
         (t2_bad[0] == 0U) && (t2_bad[1] == 0U) &&
         (t2_isr_sets > 0U) && (seen == t2_isr_sets) &&
         ((rc_fault_get_raw() & (T2_BIT_A | T2_BIT_B | T2_BIT_ISR)) == 0U);
    return ok;
}

/* 3 s of the Sense task as it runs (line + IMU + ADC barcode). */
static bool check_t3(void)
{
    uint32_t over0 = rc_sense_overruns();
    uint32_t line_us;
    uint32_t imu_us;
    uint32_t over;

    tk_dly_tsk(3000);
    over = rc_sense_overruns() - over0;
    rc_sense_max_us(&line_us, &imu_us);
    tm_printf((UB *)"[TEST]   T3: 3 s, %lu missed periods; longest line %lu us,"
              " IMU %lu us, ADC barcode %lu us\n", (unsigned long)over,
              (unsigned long)line_us, (unsigned long)imu_us,
              (unsigned long)rc_sense_bar_max_us());
    return (over == 0U) && (rc_sense_bar_max_us() < 1000U);
}

void rc_fault_test(void)
{
    TEST_INIT();

    TEST_CHECK("CR1-T2", check_t2(),
               "fault bitmask: 2 cores + handler, no lost update");
    TEST_CHECK("CR1-T3", check_t3(),
               "Sense: no missed 10 ms period, ADC read < 1 ms");
    TEST_SUMMARY("cr1");
}

#endif /* APP_TEST */
