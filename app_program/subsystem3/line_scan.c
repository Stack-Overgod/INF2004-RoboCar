/*
 *  line_scan.c  -  Buddy 3, finding the line by spinning in place
 *
 *  See line_scan.h. One scan, step by step:
 *    PEEK    first line sample: build the sweep plan (the quick sweep
 *            toward a sensor already on black, then first_deg, then
 *            back_deg) and start the first sweep
 *    SWEEP   spin toward the sweep's end heading, recording each sensor's
 *            latest crossing (on, then off); stop once both sensors' last
 *            crossings pass ls_pair_ok(); at the end heading, the next
 *            sweep, or "not found" after the last one
 *    SETTLE  let the car stop (LS_SETTLE samples)
 *    ALIGN   turn to the middle of the four crossing headings
 *
 *  Headings come from the encoders, 0.1 degree units, + = left, 0 = where
 *  the scan started: a wheel travels LS_UM_PER_DEG per degree of spin in
 *  place, so the mean wheel travel gives the heading change.
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "line_scan.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_time.h"
#include "../subsystem2/sub_motion.h"
#include "../subsystem2/drv_encoder.h"

/* Each wheel's travel per degree of spin in place, um: (W/2) x pi/180. */
#define LS_UM_PER_DEG   ((uint32_t)RC_WHEEL_BASE_MM * 1000UL * 31416UL \
                         / (2UL * 180UL * 10000UL))

/* Both sensors' crossings must be the same line. Spinning in place the
 * sensors (~25-30 mm apart, ~80 mm ahead of the axle) are ~20 degrees
 * apart on their circle, so on one line the leading sensor (the left one
 * on a left spin) comes off it ~20 degrees before the trailing one. Foam
 * slip makes the encoder heading read up to ~1.35x the real turn, ~27
 * degrees. A pair finished further apart than this, or in the wrong
 * order, is two different lines and is not used. */
#define LS_PAIR_DEG     (40)
/* On the car both sensors come off one line within a few degrees of each
 * other, so encoder noise and slip can swap their order: the trailing
 * sensor may come off up to this much first and still count. */
#define LS_ORDER_SLACK_DEG (10)

/* Line samples (10 ms each) to let the car stop before the final turn. */
#define LS_SETTLE       (15U)

/* A final turn under 2 degrees is not made. */
#define LS_ALIGN_MIN    (20)        /* 0.1 deg */

/* Time limit: the sweeps' expected time plus LS_SPARE_MS (ramps,
 * settling), plus LS_ALIGN_MS for the final turn. A scan that has not
 * ended by then (stalled wheel, lost MOTION_DONE) reports "not found". */
#define LS_SPARE_MS     (2000U)
#define LS_ALIGN_MS     (5000U)

#define LS_PLAN_MAX     (3U)

typedef enum {
    LS_IDLE = 0, LS_PEEK, LS_SWEEP, LS_SETTLE_ST, LS_ALIGN
} ls_state_t;

static ls_state_t          ls_state;
static line_scan_cfg_t     cfg;
static line_scan_done_cb_t done_cb;
static void               *done_ctx;
/* the sweeps: the heading each one ends at, in order; each starts where
 * the last one ended */
static int32_t  plan[LS_PLAN_MAX];
static uint32_t plan_n;
static uint32_t sweep_i;
/* this sweep: direction (+1 left), start heading, end heading, encoder
 * counts at its start */
static int32_t  sw_dir;
static int32_t  sw_base;
static int32_t  sw_limit;
static uint32_t sw_l0;
static uint32_t sw_r0;
/* per sensor (0 left, 1 right): headings of its latest crossing (on, off),
 * progress (0 none, 1 on, 2 on and off), its last sample */
static int32_t  x_on[2];
static int32_t  x_off[2];
static uint8_t  x_seen[2];
static bool     x_prev[2];
static int32_t  target;
static uint32_t settle_n;
static uint32_t align_move;
static uint32_t start_us;
static uint32_t limit_us;

/* Heading now, 0.1 deg (+ = left). */
static int32_t heading(void)
{
    uint32_t dl = drv_encoder_count(RC_SIDE_LEFT) - sw_l0;
    uint32_t dr = drv_encoder_count(RC_SIDE_RIGHT) - sw_r0;
    uint32_t um = ((dl + dr) / 2U) * RC_ENC_UM_PER_TICK;

    return sw_base + (sw_dir * (int32_t)((um * 10U) / LS_UM_PER_DEG));
}

/* Start sweep `n` of the plan from heading `from`; false if there is none.
 * Each sensor's "previous" reading starts as white, so a sensor already on
 * black counts as going on at `from`: at a turnaround that keeps a
 * crossing the last sweep cut off at its end heading. */
static bool sweep_start(uint32_t n, int32_t from)
{
    if (n >= plan_n) {
        return false;
    }
    sweep_i   = n;
    sw_dir    = (plan[n] > from) ? 1 : -1;
    sw_base   = from;
    sw_limit  = plan[n];
    sw_l0     = drv_encoder_count(RC_SIDE_LEFT);
    sw_r0     = drv_encoder_count(RC_SIDE_RIGHT);
    x_seen[0] = 0U;
    x_seen[1] = 0U;
    x_prev[0] = false;
    x_prev[1] = false;
    /* sub_motion steer > 0 turns right; sw_dir > 0 is left */
    (void)sub_motion_drive_speed(0, (int16_t)(-sw_dir * cfg.speed_mm_s));
    return true;
}

/* True if both sensors' latest crossings are the same line: the sensor
 * that leads in this sweep's direction came off it first, and the other
 * within LS_PAIR_DEG after. */
static bool pair_ok(void)
{
    uint32_t lead  = (sw_dir > 0) ? 0U : 1U;
    int32_t  after = sw_dir * (x_off[1U - lead] - x_off[lead]);

    return (after >= -(LS_ORDER_SLACK_DEG * 10))
           && (after <= (LS_PAIR_DEG * 10));
}

/* End the scan: motors stopped, state idle, then the caller's callback
 * (which may start another scan). */
static void finish(bool found)
{
    line_scan_done_cb_t cb  = done_cb;
    void               *ctx = done_ctx;

    (void)sub_motion_stop(true);
    ls_state   = LS_IDLE;
    align_move = 0U;
    done_cb    = NULL;
    if (cb != NULL) {
        cb(found, ctx);
    }
}

/* Build the plan from the first sample and start the first sweep. */
static void plan_begin(const bool on[2])
{
    uint32_t i;
    uint32_t total = 0U;            /* degrees */
    int32_t  at    = 0;
    int32_t  leg;

    plan_n = 0U;
    if ((cfg.quick_deg > 0) && (on[0] != on[1])) {
        tm_printf((UB *)"[line] scan: %s sensor on the line, quick check"
                  " that way\n", on[0] ? "left" : "right");
        plan[plan_n] = (on[0] ? 1 : -1) * (int32_t)cfg.quick_deg * 10;
        plan_n++;
    }
    plan[plan_n] = (int32_t)cfg.first_dir * cfg.first_deg * 10;
    plan_n++;
    plan[plan_n] = -(int32_t)cfg.first_dir * cfg.back_deg * 10;
    plan_n++;

    for (i = 0U; i < plan_n; i++) {
        leg    = plan[i] - at;
        total += (uint32_t)((leg < 0) ? -leg : leg) / 10U;
        at     = plan[i];
    }
    /* um per degree / (mm per s) = ms per degree */
    limit_us = (((total * LS_UM_PER_DEG) / (uint32_t)cfg.speed_mm_s)
                + LS_SPARE_MS + LS_ALIGN_MS) * 1000U;
    start_us = rc_time_us();
    ls_state = LS_SWEEP;
    (void)sweep_start(0U, 0);
}

static void align_done(uint32_t move_id, bool completed,
                       uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((ls_state != LS_ALIGN) || (move_id != align_move)) {
        return;                     /* aborted, or an older scan's turn */
    }
    if (!completed) {
        tm_printf((UB *)"[line] scan: final turn aborted\n");
    }
    finish(completed);
}

/* One sweep sample: record crossings, check for a pair, or move on at the
 * sweep's end. */
static void sweep_sample(const bool on[2])
{
    int32_t  h        = heading();
    bool     finished = false;
    uint32_t i;

    for (i = 0U; i < 2U; i++) {
        if (on[i] && !x_prev[i]) {
            /* a new crossing replaces an earlier one: each sensor holds
             * its latest */
            x_on[i]   = h;
            x_seen[i] = 1U;
        } else if (!on[i] && x_prev[i] && (x_seen[i] == 1U)) {
            x_off[i]  = h;
            x_seen[i] = 2U;
            finished  = true;
        } else {
            /* no change */
        }
        x_prev[i] = on[i];
    }

    if (finished && (x_seen[0] == 2U) && (x_seen[1] == 2U)) {
        if (pair_ok()) {
            target = (x_on[0] + x_off[0] + x_on[1] + x_off[1]) / 4;
            tm_printf((UB *)"[line] scan: L %d..%d R %d..%d -> %d"
                      " (0.1 deg)\n", (INT)x_on[0], (INT)x_off[0],
                      (INT)x_on[1], (INT)x_off[1], (INT)target);
            (void)sub_motion_stop(true);
            ls_state = LS_SETTLE_ST;
            settle_n = LS_SETTLE;
            return;
        }
        tm_printf((UB *)"[line] scan: L off %d, R off %d are different"
                  " lines, still looking\n", (INT)x_off[0], (INT)x_off[1]);
    }
    if ((sw_dir * (h - sw_limit)) >= 0) {
        if (sweep_start(sweep_i + 1U, h)) {
            tm_printf((UB *)"[line] scan: next sweep to %d\n", (INT)sw_limit);
        } else {
            tm_printf((UB *)"[line] scan: no line found\n");
            finish(false);
        }
    }
}

/* After the settle: turn to the line's middle, or finish if already
 * there. */
static void align_begin(void)
{
    int32_t d = target - heading();     /* > 0: turn left */

    if ((d > -LS_ALIGN_MIN) && (d < LS_ALIGN_MIN)) {
        finish(true);
        return;
    }
    ls_state = LS_ALIGN;
    /* sub_motion: deg < 0 turns left; rounded to the degree. The scan
     * measured d in encoder degrees at the same spin, so the slip is
     * already in it. */
    align_move = sub_motion_turn_deg(
        (int16_t)(-((d + ((d > 0) ? 5 : -5)) / 10)), align_done, NULL);
    if (align_move == 0U) {
        tm_printf((UB *)"[line] scan: final turn refused\n");
        finish(false);
    }
}

static void on_sample(const rc_event_t *evt, void *ctx)
{
    bool on[2];

    (void)ctx;

    if (ls_state == LS_IDLE) {
        return;
    }
    if (rc_time_since(start_us) > limit_us) {
        tm_printf((UB *)"[line] scan: timed out\n");
        finish(false);
        return;
    }
    on[0] = evt->u.line.on_line_l;
    on[1] = evt->u.line.on_line_r;

    switch (ls_state) {
    case LS_PEEK:
        plan_begin(on);
        break;
    case LS_SWEEP:
        sweep_sample(on);
        break;
    case LS_SETTLE_ST:
        if (settle_n > 0U) {
            settle_n--;
        } else {
            align_begin();
        }
        break;
    case LS_ALIGN:
    case LS_IDLE:
    default:
        break;                      /* the final turn is running */
    }
}

/* ------------------------------------------------------------------ */

rc_result_t line_scan_init(void)
{
    ls_state = LS_IDLE;
    if (rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_FAST,
                           on_sample, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

rc_result_t line_scan_start(const line_scan_cfg_t *c,
                            line_scan_done_cb_t done, void *ctx)
{
    if ((c == NULL) || (c->speed_mm_s <= 0)) {
        return RC_ERR_PARAM;
    }
    if (ls_state != LS_IDLE) {
        return RC_ERR_STATE;
    }
    cfg        = *c;
    done_cb    = done;
    done_ctx   = ctx;
    align_move = 0U;
    /* the time limit starts with the plan; until then, one sample */
    start_us   = rc_time_us();
    limit_us   = LS_ALIGN_MS * 1000U;
    ls_state   = LS_PEEK;
    return RC_OK;
}

void line_scan_abort(void)
{
    if (ls_state == LS_IDLE) {
        return;
    }
    ls_state   = LS_IDLE;
    align_move = 0U;
    done_cb    = NULL;
    (void)sub_motion_stop(true);
}

bool line_scan_busy(void)
{
    return ls_state != LS_IDLE;
}

/*** end of file ***/
