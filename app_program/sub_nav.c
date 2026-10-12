/*
 *  sub_nav.c  -  mission state machine (shared; integration owner Buddy 5)
 *
 *  Report §6.5. All handlers run on the FAST dispatcher (rc_event), and
 *  sub_motion delivers its completion callbacks on that same dispatcher,
 *  so `state` is only written by one task. The exception is
 *  sub_nav_start(), called once from usermain() while the machine is
 *  IDLE/STOPPED, when no handler acts on anything.
 *
 *  Rule for IDLE: every event is ignored. Other buddies' self-tests
 *  publish fake IMPACT / COMMAND_RX events in a TEST=1 build; nav must not
 *  react to them before the run starts.
 *
 *  What talks to what (TEAM.md §5: IF-07..IF-09, IF-11, IF-14, IF-15):
 *    sub_nav -> sub_scan    set_watch() / start() / abort()      (Buddy 5)
 *    sub_nav -> sub_motion  turn_deg() / forward_mm() / drive() / stop()
 *                                                                (Buddy 2)
 *    sub_nav -> sub_line    enable() / begin_search()            (Buddy 3)
 *    sub_nav -> sub_barcode arm()                                (Buddy 3)
 *    events in: ULTRA_RESULT, AVOIDANCE_PLAN (S5), LINE_REACQUIRED (S3),
 *               IMPACT (S4), COMMAND_RX (S1)
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_nav.h"
#include "rc_config.h"
#include "rc_event.h"
#include "subsystem2/sub_motion.h"
#include "subsystem3/sub_line.h"
#include "subsystem3/sub_barcode.h"
#include "subsystem5/sub_scan.h"

/* FR6: a watch ping at or under this range starts avoidance. Placeholder
 * in the report ("Buddy 5 to confirm"). */
#define NAV_WATCH_TRIGGER_MM    (300U)

/* The forward watch always pings straight ahead (sub_scan.c, S_WATCH). */
#define NAV_WATCH_ANGLE_DEG     (90)

/* Bypass turns, spin in place. Open loop until Buddy 2's turn calibration
 * (TURN_SLIP_PERMILLE) is done. */
#define NAV_TURN_DEG            (90)

/* Duty while RECOVERING: sub_line does not steer while SEARCHING, so nav
 * keeps the car rolling forward until it crosses the line.
 * TODO Buddy 2/3: tune on the track. */
#define NAV_RECOVER_PERMILLE    (250)

/* ------------------------------------------------------------------ */

static volatile rc_nav_state_t state = RC_NAV_IDLE;

static bool         scan_pending;   /* we asked for the scan whose plan we await */
static rc_pl_plan_t plan;           /* copy of the plan being executed */
static uint32_t     bypass_step;    /* index into the bypass legs below */
static uint32_t     bypass_move;    /* id of the sub_motion move in flight */

static void enter(rc_nav_state_t next);

const char *sub_nav_state_name(rc_nav_state_t s)
{
    switch (s) {
    case RC_NAV_IDLE:            return "IDLE";
    case RC_NAV_FOLLOWING:       return "FOLLOWING";
    case RC_NAV_READING_BARCODE: return "READING_BARCODE";
    case RC_NAV_EXECUTING_CMD:   return "EXECUTING_CMD";
    case RC_NAV_AVOIDING:        return "AVOIDING";
    case RC_NAV_RECOVERING:      return "RECOVERING";
    case RC_NAV_STOPPED:         return "STOPPED";
    default:                     return "?";
    }
}

/* ------------------------------------------------------------------ *
 *  Obstacle bypass (FR8a/FR8b)
 *
 *  Report §6.5: "a three-leg bypass (turn, forward, turn back)". Using
 *  the plan's two distances from sub_scan's build_plan():
 *
 *      leg 1  step aside   turn out 90, forward lateral_mm
 *      leg 2  run past     turn back 90 (parallel), forward forward_mm
 *      leg 3  turn back    turn 90 toward the line, then RECOVERING
 *                          drives forward until sub_line finds it
 *
 *  Each move is queued, and its completion callback queues the next one.
 *  Nothing waits. A move that is refused (returns id 0) or ends with
 *  completed == false goes to STOPPED: the report treats any fault in a
 *  bypass leg as unrecoverable for Week 6.
 *
 *  TODO Buddy 5 + Buddy 2: the final 90-degree turn crosses the line at
 *  right angles. A shallower re-entry angle (e.g. 45) may be easier for
 *  sub_line to pick up; decide on the track.
 * ------------------------------------------------------------------ */

#define BYPASS_STEPS    (5U)

static void bypass_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx);

/* Turning toward the side the plan chose: deg < 0 is left (sub_motion.h). */
static int16_t out_deg(void)
{
    return (plan.action == RC_CMD_TURN_LEFT) ? (int16_t)(-NAV_TURN_DEG)
                                             : (int16_t)NAV_TURN_DEG;
}

static uint32_t bypass_issue(uint32_t step)
{
    switch (step) {
    case 0U: return sub_motion_turn_deg(out_deg(), bypass_done, NULL);
    case 1U: return sub_motion_forward_mm(plan.lateral_mm, bypass_done, NULL);
    case 2U: return sub_motion_turn_deg((int16_t)(-out_deg()), bypass_done, NULL);
    case 3U: return sub_motion_forward_mm(plan.forward_mm, bypass_done, NULL);
    case 4U: return sub_motion_turn_deg((int16_t)(-out_deg()), bypass_done, NULL);
    default: return 0U;
    }
}

static void bypass_next(void)
{
    bypass_move = bypass_issue(bypass_step);
    if (bypass_move == 0U) {
        tm_printf((UB *)"[nav] bypass step %d refused by sub_motion\n",
                  (INT)bypass_step);
        enter(RC_NAV_STOPPED);
    }
}

/* Runs on the FAST dispatcher - the same task that called bypass_next() -
 * so it cannot run before bypass_move has been stored. */
static void bypass_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_AVOIDING) || (move_id != bypass_move)) {
        return;     /* a stale move, e.g. one cut short by a STOP */
    }
    if (!completed) {
        tm_printf((UB *)"[nav] bypass step %d aborted\n", (INT)bypass_step);
        enter(RC_NAV_STOPPED);
        return;
    }

    bypass_step++;
    if (bypass_step >= BYPASS_STEPS) {
        enter(RC_NAV_RECOVERING);
    } else {
        bypass_next();
    }
}

/* ------------------------------------------------------------------ *
 *  State entry: what each state switches on and off
 * ------------------------------------------------------------------ */

static void enter(rc_nav_state_t next)
{
    rc_nav_state_t prev = state;

    if (next == prev) {
        return;
    }
    state = next;
    tm_printf((UB *)"[nav] %s -> %s\n",
              sub_nav_state_name(prev), sub_nav_state_name(next));

    switch (next) {
    case RC_NAV_FOLLOWING:
        /* sub_line steers through sub_motion_drive(); the barcode decoder
         * is armed for the whole of FOLLOWING (report §6.5); the watch
         * pings straight ahead every RC_PERIOD_SCAN_MS. */
        (void)sub_line_enable(true);
        (void)sub_barcode_arm(true);
        (void)sub_scan_set_watch(true, NAV_WATCH_TRIGGER_MM);
        break;

    case RC_NAV_AVOIDING:
        /* FR7a allows "slow to <= 150 mm/s or stop". Stop: the profile
         * is taken from a stationary car, and closed-loop speed needs
         * Buddy 2's encoder calibration (RC_ENC_TICKS_PER_REV is still a
         * placeholder). Line steering must be off first, or sub_line
         * would restart the motors on its next sample. */
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(false, 0U);
        (void)sub_motion_stop(true);
        break;

    case RC_NAV_RECOVERING:
        /* FR9: search for the line, resume on LINE_REACQUIRED.
         * TODO Buddy 5: the 3 s "not found -> STOPPED + recovery-failed
         * event" timeout. Needs a kernel alarm whose handler publishes an
         * event (handlers must not call sub_* directly), plus a new
         * event id in rc_types.h. */
        (void)sub_line_enable(true);
        (void)sub_line_begin_search();
        (void)sub_motion_drive(NAV_RECOVER_PERMILLE, 0);
        break;

    case RC_NAV_READING_BARCODE:
    case RC_NAV_EXECUTING_CMD:
        /* TODO (stub): see on_command_rx(). Nothing enters these yet. */
        break;

    case RC_NAV_STOPPED:
    case RC_NAV_IDLE:
    default:
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(false, 0U);
        if (sub_scan_busy()) {
            /* Only when a scan is running: sub_scan_abort() leaves
             * FLG_ABORT set, which would cut the NEXT scan short. */
            (void)sub_scan_abort();
        }
        scan_pending = false;
        (void)sub_motion_stop(true);
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Event handlers (all FAST lane)
 * ------------------------------------------------------------------ */

/* FR6: a forward-watch ping close enough to act on. */
static void on_ultra(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (state != RC_NAV_FOLLOWING) {
        return;     /* scan pings during AVOIDING land here too */
    }
    if (!evt->u.ultra.valid
        || (evt->u.ultra.angle_deg != NAV_WATCH_ANGLE_DEG)
        || (evt->u.ultra.range_mm > NAV_WATCH_TRIGGER_MM)) {
        return;
    }

    tm_printf((UB *)"[nav] obstacle at %d mm\n", (INT)evt->u.ultra.range_mm);
    enter(RC_NAV_AVOIDING);

    if (sub_scan_start() == RC_OK) {
        scan_pending = true;
    } else {
        tm_printf((UB *)"[nav] scan refused\n");
        enter(RC_NAV_STOPPED);
    }
}

/* FR8a: act on the plan from the scan we asked for. */
static void on_plan(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if ((state != RC_NAV_AVOIDING) || !scan_pending) {
        return;     /* e.g. a scan started by subsystem5_test() */
    }
    scan_pending = false;
    plan = evt->u.plan;

    switch (plan.action) {
    case RC_CMD_TURN_LEFT:
    case RC_CMD_TURN_RIGHT:
        tm_printf((UB *)"[nav] bypass %s, aside %d mm, past %d mm\n",
                  (plan.action == RC_CMD_TURN_LEFT) ? "left" : "right",
                  (INT)plan.lateral_mm, (INT)plan.forward_mm);
        bypass_step = 0U;
        bypass_next();
        break;

    case RC_CMD_STOP:
        tm_printf((UB *)"[nav] no safe gap\n");
        enter(RC_NAV_STOPPED);
        break;

    case RC_CMD_U_TURN:
        /* TODO (G5): reverse-and-reattempt is planned, not built.
         * sub_scan never produces this today. */
        enter(RC_NAV_STOPPED);
        break;

    case RC_CMD_GO_STRAIGHT:
    case RC_CMD_NONE:
    default:
        /* Nothing close after all: carry on. */
        enter(RC_NAV_FOLLOWING);
        break;
    }
}

/* FR9: sub_line found the track again after the bypass. */
static void on_line_reacquired(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if (state == RC_NAV_RECOVERING) {
        enter(RC_NAV_FOLLOWING);
    }
}

/* FR8b / FR13: an impact during the run aborts to STOPPED. */
static void on_impact(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        tm_printf((UB *)"[nav] impact\n");
        enter(RC_NAV_STOPPED);
    }
}

/* FR10b: STOP is honoured from any running state.
 * TODO (stub): the other commands (TURN_LEFT/RIGHT, GO_STRAIGHT, U_TURN)
 * go to EXECUTING_CMD as a queued sub_motion move, and MOTION_DONE
 * resumes FOLLOWING. The same code serves barcode commands
 * (RC_EVT_BARCODE_DECODED, READING_BARCODE). Build it once Buddy 2's
 * sub_motion and Buddy 3's sub_barcode are wired into their
 * subsystemN_init(). */
static void on_command_rx(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (state == RC_NAV_IDLE) {
        return;
    }
    if (evt->u.command.command == RC_CMD_STOP) {
        tm_printf((UB *)"[nav] STOP command\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    tm_printf((UB *)"[nav] command %d ignored (not built yet)\n",
              (INT)evt->u.command.command);
}

/* ------------------------------------------------------------------ */

rc_result_t sub_nav_init(void)
{
    if ((rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_FAST,
                            on_ultra, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_AVOIDANCE_PLAN, RC_LANE_FAST,
                               on_plan, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_LINE_REACQUIRED, RC_LANE_FAST,
                               on_line_reacquired, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_FAST,
                               on_impact, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_COMMAND_RX, RC_LANE_FAST,
                               on_command_rx, NULL) < 0)) {
        return RC_ERR_NOSPACE;
    }

    state = RC_NAV_IDLE;
    return RC_OK;
}

rc_result_t sub_nav_start(void)
{
    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        return RC_ERR_STATE;
    }

    /* TODO (report §6.5 IDLE): before this, usermain should calibrate the
     * IMU (Buddy 4) and set the base speed, 250 mm/s (Buddy 2). Both
     * belong in those buddies' subsystemN_init()/_start(). */
    scan_pending = false;
    enter(RC_NAV_FOLLOWING);
    return RC_OK;
}

rc_result_t sub_nav_stop(void)
{
    return sub_nav_inject_command(RC_CMD_STOP, 0);
}

rc_nav_state_t sub_nav_state(void)
{
    return state;
}

rc_result_t sub_nav_inject_command(rc_nav_cmd_t cmd, int32_t arg)
{
    rc_event_t evt = { 0 };

    evt.id                = RC_EVT_COMMAND_RX;
    evt.u.command.command = cmd;
    evt.u.command.arg     = arg;
    return rc_event_publish(&evt);
}
