/*
 *  rc_event.c
 *
 *  Ring buffer plus dispatcher task, one pair per lane.
 *
 *  Why a hand-rolled ring rather than a message buffer: tk_snd_mbf can
 *  wait, so it is not callable from an interrupt handler. The encoder and
 *  echo ISRs both need to publish. A short critical section around a
 *  power-of-two ring, followed by tk_set_flg (which is legal in an ISR),
 *  gives an ISR-safe publish that always returns in bounded time.
 */

#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "rc_event.h"
#include "rc_config.h"
#include "rc_time.h"

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

/* ------------------------------------------------------------------ *
 *  Critical section
 *
 *  Single core: masking local interrupts is enough. SMP: the ring can be
 *  touched from either core, so take the spinlock as well. Both forms
 *  preserve and restore the caller's interrupt state, so this is safe to
 *  use inside an ISR.
 * ------------------------------------------------------------------ */

#if TK_SUPPORT_SMP
static T_SPLOCK ring_lock;
/* NOTE: sts must be UINT, not uint32_t. ISpinLock takes UINT*, and on
 * arm-none-eabi uint32_t is 'long unsigned int' - same width, incompatible
 * pointer type. This only shows up in the SMP=1 build. */
#define RC_LOCK(sts)      ((void)ISpinLock(&ring_lock, &(sts)))
#define RC_UNLOCK(sts)    ((void)ISpinUnlock(&ring_lock, (sts)))
#else
#define RC_LOCK(sts)      DI(sts)
#define RC_UNLOCK(sts)    EI(sts)
#endif

#define RING_MASK         (RC_EVENT_RING_SZ - 1U)
#define FLG_WORK          (1U << 0)

/* Keep this many ring slots unavailable to high-rate "stream" events, so a
 * burst of samples can never crowd out a one-off control event
 * (MOTION_DONE, LINE_*, BARCODE_*, IMPACT, HUMP_*, COMMAND_RX, NAV_TIMEOUT,
 * ULTRA_RESULT). A dropped sample is replaced by its successor one period
 * later; a dropped one-off can wedge a state machine (Risk 1 / G10). */
#define RING_RESERVE      (16U)

/* ------------------------------------------------------------------ *
 *  Per-lane state
 * ------------------------------------------------------------------ */

typedef struct {
    rc_event_cb_t cb;
    void         *ctx;
    rc_evt_id_t   id;
    bool          in_use;
} subscription_t;

typedef struct {
    rc_event_t     ring[RC_EVENT_RING_SZ];
    volatile uint32_t head;         /* written by publishers  */
    volatile uint32_t tail;         /* written by dispatcher  */
    uint32_t       dropped;
    uint32_t       high_water;      /* max ring occupancy seen */
#if APP_TEST
    uint32_t       drop_by_id[RC_EVT_COUNT];
    uint32_t       max_wcet_us[RC_EVT_COUNT];  /* slowest single callback */
    uint32_t       max_lat_us[RC_EVT_COUNT];   /* worst publish->dispatch */
#endif
    ID             flgid;
    ID             tskid;
    subscription_t subs[RC_EVENT_MAX_SUBS];
} lane_t;

static lane_t lanes[RC_LANE_COUNT];

/*
 *  One bit per lane, per event id. Lets publish skip a lane that nobody
 *  is listening on, which keeps an unsubscribed high-rate event (encoder
 *  edges, for example) from filling the slow ring.
 */
static volatile uint8_t lane_mask[RC_EVT_COUNT];

/* ------------------------------------------------------------------ *
 *  Ring primitives. Caller holds the lock.
 * ------------------------------------------------------------------ */

/* High-rate events that are safe to drop under pressure: the next sample
 * supersedes the one lost. Everything else is a one-off and is protected
 * by RING_RESERVE. */
static bool is_stream_evt(rc_evt_id_t id)
{
    return (id == RC_EVT_LINE_SAMPLE) || (id == RC_EVT_IMU_SAMPLE)
        || (id == RC_EVT_ODOMETRY)    || (id == RC_EVT_ENCODER_EDGE)
        || (id == RC_EVT_MOTION_CLASS);
}

static bool ring_push(lane_t *ln, const rc_event_t *evt)
{
    uint32_t next = (ln->head + 1U) & RING_MASK;
    uint32_t used = (ln->head - ln->tail) & RING_MASK;

    if (next == ln->tail) {
        return false;               /* full, refuse rather than overwrite */
    }
    /* A stream event may not consume the reserved slots. */
    if (is_stream_evt(evt->id)
        && (used >= (RC_EVENT_RING_SZ - 1U - RING_RESERVE))) {
        return false;
    }
    if ((used + 1U) > ln->high_water) {
        ln->high_water = used + 1U;
    }
    ln->ring[ln->head] = *evt;
    ln->head = next;
    return true;
}

static bool ring_pop(lane_t *ln, rc_event_t *out)
{
    if (ln->tail == ln->head) {
        return false;
    }
    *out = ln->ring[ln->tail];
    ln->tail = (ln->tail + 1U) & RING_MASK;
    return true;
}

/* ------------------------------------------------------------------ *
 *  Publish
 * ------------------------------------------------------------------ */

static rc_result_t publish_common(rc_event_t *evt)
{
    UINT        sts;
    uint8_t     mask;
    rc_result_t res = RC_OK;
    uint32_t    i;

    if ((evt == NULL) || (evt->id >= RC_EVT_COUNT)) {
        return RC_ERR_PARAM;
    }

    evt->t_us = rc_time_us();
    mask = lane_mask[evt->id];

    if (mask == 0U) {
        return RC_OK;               /* nobody cares, not an error */
    }

    RC_LOCK(sts);
    for (i = 0U; i < (uint32_t)RC_LANE_COUNT; i++) {
        if ((mask & (1U << i)) != 0U) {
            if (!ring_push(&lanes[i], evt)) {
                lanes[i].dropped++;
#if APP_TEST
                lanes[i].drop_by_id[evt->id]++;
#endif
                res = RC_ERR_NOSPACE;
            }
        }
    }
    RC_UNLOCK(sts);

    /* Waking outside the lock keeps the critical section short. tk_set_flg
     * is one of the few calls legal from an interrupt handler. */
    for (i = 0U; i < (uint32_t)RC_LANE_COUNT; i++) {
        if ((mask & (1U << i)) != 0U) {
            (void)tk_set_flg(lanes[i].flgid, FLG_WORK);
        }
    }

    return res;
}

rc_result_t rc_event_publish(rc_event_t *evt)
{
    return publish_common(evt);
}

rc_result_t rc_event_publish_i(rc_event_t *evt)
{
    return publish_common(evt);
}

/* ------------------------------------------------------------------ *
 *  Subscribe
 * ------------------------------------------------------------------ */

int32_t rc_event_subscribe(rc_evt_id_t id,
                           rc_lane_t lane,
                           rc_event_cb_t cb,
                           void *ctx)
{
    lane_t  *ln;
    uint32_t i;
    UINT     sts;

    if ((id >= RC_EVT_COUNT) || (lane >= RC_LANE_COUNT) || (cb == NULL)) {
        return -1;
    }

    ln = &lanes[lane];

    for (i = 0U; i < RC_EVENT_MAX_SUBS; i++) {
        if (!ln->subs[i].in_use) {
            ln->subs[i].cb     = cb;
            ln->subs[i].ctx    = ctx;
            ln->subs[i].id     = id;
            ln->subs[i].in_use = true;

            RC_LOCK(sts);
            lane_mask[id] |= (uint8_t)(1U << (uint32_t)lane);
            RC_UNLOCK(sts);

            /* handle encodes both lane and slot */
            return (int32_t)(((uint32_t)lane << 16) | i);
        }
    }
    return -1;
}

rc_result_t rc_event_unsubscribe(int32_t handle)
{
    uint32_t lane;
    uint32_t slot;
    uint32_t i;
    UINT     sts;
    bool     still_used = false;
    lane_t  *ln;

    if (handle < 0) {
        return RC_ERR_PARAM;
    }

    lane = ((uint32_t)handle >> 16) & 0xFFFFU;
    slot = (uint32_t)handle & 0xFFFFU;

    if ((lane >= (uint32_t)RC_LANE_COUNT) || (slot >= RC_EVENT_MAX_SUBS)) {
        return RC_ERR_PARAM;
    }

    ln = &lanes[lane];
    if (!ln->subs[slot].in_use) {
        return RC_ERR_STATE;
    }

    {
        rc_evt_id_t id = ln->subs[slot].id;

        ln->subs[slot].in_use = false;
        ln->subs[slot].cb     = NULL;

        for (i = 0U; i < RC_EVENT_MAX_SUBS; i++) {
            if (ln->subs[i].in_use && (ln->subs[i].id == id)) {
                still_used = true;
                break;
            }
        }
        if (!still_used) {
            RC_LOCK(sts);
            lane_mask[id] &= (uint8_t)~(1U << lane);
            RC_UNLOCK(sts);
        }
    }
    return RC_OK;
}

uint32_t rc_event_dropped(rc_lane_t lane)
{
    if (lane >= RC_LANE_COUNT) {
        return 0U;
    }
    return lanes[lane].dropped;
}

uint32_t rc_event_high_water(rc_lane_t lane)
{
    if (lane >= RC_LANE_COUNT) {
        return 0U;
    }
    return lanes[lane].high_water;
}

#if APP_TEST
static const char *evt_name(rc_evt_id_t id)
{
    switch (id) {
    case RC_EVT_TICK:             return "TICK";
    case RC_EVT_ENCODER_EDGE:     return "ENCODER_EDGE";
    case RC_EVT_ODOMETRY:         return "ODOMETRY";
    case RC_EVT_MOTION_DONE:      return "MOTION_DONE";
    case RC_EVT_LINE_SAMPLE:      return "LINE_SAMPLE";
    case RC_EVT_LINE_LOST:        return "LINE_LOST";
    case RC_EVT_LINE_REACQUIRED:  return "LINE_REACQUIRED";
    case RC_EVT_BARCODE_EDGE:     return "BARCODE_EDGE";
    case RC_EVT_BARCODE_DECODED:  return "BARCODE_DECODED";
    case RC_EVT_IMU_SAMPLE:       return "IMU_SAMPLE";
    case RC_EVT_HUMP_BEGIN:       return "HUMP_BEGIN";
    case RC_EVT_HUMP_END:         return "HUMP_END";
    case RC_EVT_MOTION_CLASS:     return "MOTION_CLASS";
    case RC_EVT_IMPACT:           return "IMPACT";
    case RC_EVT_ULTRA_RESULT:     return "ULTRA_RESULT";
    case RC_EVT_SCAN_COMPLETE:    return "SCAN_COMPLETE";
    case RC_EVT_OBSTACLE_PROFILE: return "OBSTACLE_PROFILE";
    case RC_EVT_AVOIDANCE_PLAN:   return "AVOIDANCE_PLAN";
    case RC_EVT_COMMAND_RX:       return "COMMAND_RX";
    case RC_EVT_LINK_STATE:       return "LINK_STATE";
    case RC_EVT_NAV_TIMEOUT:      return "NAV_TIMEOUT";
    case RC_EVT_RECOVERY_FAILED:  return "RECOVERY_FAILED";
    case RC_EVT_BARCODE_SCAN:     return "BARCODE_SCAN";
    case RC_EVT_BARCODE_RAW:      return "BARCODE_RAW";
    case RC_EVT_SPIN_EDGES:       return "SPIN_EDGES";
    default:                      return "?";
    }
}

static const char *lane_name(uint32_t lane)
{
    return (lane == (uint32_t)RC_LANE_FAST) ? "FAST" : "SLOW";
}

void rc_event_probe_reset(void)
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < (uint32_t)RC_LANE_COUNT; i++) {
        lanes[i].dropped    = 0U;
        lanes[i].high_water = 0U;
        for (j = 0U; j < (uint32_t)RC_EVT_COUNT; j++) {
            lanes[i].drop_by_id[j]  = 0U;
            lanes[i].max_wcet_us[j] = 0U;
            lanes[i].max_lat_us[j]  = 0U;
        }
    }
}

void rc_event_report(void)
{
    uint32_t lane;
    uint32_t id;

    tm_printf((UB *)"[RISK1] --- event bus report ---\n");
    for (lane = 0U; lane < (uint32_t)RC_LANE_COUNT; lane++) {
        uint32_t wcet_sum = 0U;

        tm_printf((UB *)"[RISK1] lane %s: high_water=%u/%u dropped=%u\n",
                  lane_name(lane),
                  (unsigned)lanes[lane].high_water,
                  (unsigned)(RC_EVENT_RING_SZ - 1U),
                  (unsigned)lanes[lane].dropped);
        for (id = 1U; id < (uint32_t)RC_EVT_COUNT; id++) {
            uint32_t w = lanes[lane].max_wcet_us[id];
            uint32_t l = lanes[lane].max_lat_us[id];
            uint32_t d = lanes[lane].drop_by_id[id];

            if ((w == 0U) && (l == 0U) && (d == 0U)) {
                continue;
            }
            wcet_sum += w;
            tm_printf((UB *)"[RISK1]   %s wcet=%u us lat=%u us drop=%u\n",
                      evt_name((rc_evt_id_t)id),
                      (unsigned)w, (unsigned)l, (unsigned)d);
        }
        tm_printf((UB *)"[RISK1] lane %s sum(max wcet)=%u us (budget: 1 period)\n",
                  lane_name(lane), (unsigned)wcet_sum);
    }
}
#endif /* APP_TEST */

/* ------------------------------------------------------------------ *
 *  Dispatcher
 * ------------------------------------------------------------------ */

static void dispatch_lane(lane_t *ln)
{
    rc_event_t evt;
    UINT       sts;
    uint32_t   i;
    bool       got;

    for (;;) {
        RC_LOCK(sts);
        got = ring_pop(ln, &evt);
        RC_UNLOCK(sts);

        if (!got) {
            break;
        }

#if APP_TEST
        {
            uint32_t lat = rc_time_since(evt.t_us);
            if (lat > ln->max_lat_us[evt.id]) {
                ln->max_lat_us[evt.id] = lat;
            }
        }
#endif
        for (i = 0U; i < RC_EVENT_MAX_SUBS; i++) {
            if (ln->subs[i].in_use && (ln->subs[i].id == evt.id)) {
#if APP_TEST
                uint32_t t0 = rc_time_us();
#endif
                ln->subs[i].cb(&evt, ln->subs[i].ctx);
#if APP_TEST
                {
                    uint32_t dt = rc_time_since(t0);
                    if (dt > ln->max_wcet_us[evt.id]) {
                        ln->max_wcet_us[evt.id] = dt;
                    }
                }
#endif
            }
        }
    }
}

static void dispatcher_task(INT stacd, void *exinf)
{
    lane_t *ln = (lane_t *)exinf;
    UINT    ptn;

    (void)stacd;

    for (;;) {
        /* The timeout is a safety net, not a period. If a flag is ever
         * missed the ring still drains within 50 ms instead of stalling. */
        (void)tk_wai_flg(ln->flgid, FLG_WORK, TWF_ORW | TWF_CLR, &ptn, 50);
        dispatch_lane(ln);
    }
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

static rc_result_t event_init_once(void)
{
    static const INT lane_pri[RC_LANE_COUNT] = {
        RC_PRI_DISPATCH_FAST,
        RC_PRI_DISPATCH_SLOW
    };
    T_CFLG cflg;
    T_CTSK ctsk;
    uint32_t i;

    rc_time_init();

#if TK_SUPPORT_SMP
    (void)InitSpinLock(&ring_lock);
#endif

    for (i = 0U; i < (uint32_t)RC_LANE_COUNT; i++) {
        uint32_t j;
        lanes[i].head       = 0U;
        lanes[i].tail       = 0U;
        lanes[i].dropped    = 0U;
        lanes[i].high_water = 0U;
#if APP_TEST
        for (j = 0U; j < (uint32_t)RC_EVT_COUNT; j++) {
            lanes[i].drop_by_id[j]  = 0U;
            lanes[i].max_wcet_us[j] = 0U;
            lanes[i].max_lat_us[j]  = 0U;
        }
#endif
        for (j = 0U; j < RC_EVENT_MAX_SUBS; j++) {
            lanes[i].subs[j].in_use = false;
            lanes[i].subs[j].cb     = NULL;
            lanes[i].subs[j].ctx    = NULL;
        }
    }
    for (i = 0U; i < (uint32_t)RC_EVT_COUNT; i++) {
        lane_mask[i] = 0U;
    }

    for (i = 0U; i < (uint32_t)RC_LANE_COUNT; i++) {
        cflg.exinf   = NULL;
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0;

        lanes[i].flgid = tk_cre_flg(&cflg);
        if (lanes[i].flgid <= E_OK) {
            return RC_ERR_HARDWARE;
        }

        ctsk.itskpri = lane_pri[i];
        ctsk.stksz   = RC_STACK_SZ;
        ctsk.task    = dispatcher_task;
        ctsk.exinf   = &lanes[i];
        ctsk.tskatr  = TA_HLNG | TA_RNG3;

        lanes[i].tskid = tk_cre_tsk(&ctsk);
        if (lanes[i].tskid <= E_OK) {
            return RC_ERR_HARDWARE;
        }
        (void)tk_sta_tsk(lanes[i].tskid, 0);
    }

    return RC_OK;
}

/* Safe to call more than once. A second call used to wipe
 * every registration made so far (and, for rc_event, start a second set of
 * dispatcher tasks), which silently disconnected whoever had already
 * subscribed. subsystem5_test.c calls this again after start-up. Now only
 * the first successful call does the work. */
rc_result_t rc_event_init(void)
{
    static bool done;
    rc_result_t res;

    if (done) {
        return RC_OK;
    }
    res = event_init_once();
    if (res == RC_OK) {
        done = true;
    }
    return res;
}
