/*
 *  sub_nav.c  -  mission state machine (shared; integration owner Buddy 5)
 *
 *  Report §6.5. All handlers run on the FAST dispatcher (rc_event), and
 *  sub_motion delivers its completion callbacks on that same dispatcher,
 *  so `state` is only written by one task. The exceptions are
 *  sub_nav_start() (and, in a TEST=1 build, sub_nav_test_force()), called
 *  only while the machine is IDLE/STOPPED, when no handler acts on
 *  anything.
 *
 *  Rule for IDLE: every event is ignored. Other buddies' self-tests
 *  publish fake IMPACT / COMMAND_RX events in a TEST=1 build; nav must not
 *  react to them before the run starts.
 *
 *  What talks to what (TEAM.md §5: IF-07..IF-09, IF-11, IF-14..IF-17):
 *    sub_nav -> sub_scan    set_watch() / start() / abort()      (Buddy 5)
 *    sub_nav -> sub_motion  turn_deg() / forward_mm() / drive() / stop()
 *                                                                (Buddy 2)
 *    sub_nav -> sub_line    enable() / begin_search()            (Buddy 3)
 *    sub_nav -> sub_barcode arm()                                (Buddy 3)
 *    events in:  ULTRA_RESULT, AVOIDANCE_PLAN (S5), LINE_REACQUIRED,
 *                BARCODE_DECODED (S3), IMPACT (S4), COMMAND_RX (S1),
 *                NAV_TIMEOUT (our own alarm handler)
 *    events out: COMMAND_RX (sub_nav_stop / inject), RECOVERY_FAILED (FR9)
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_nav.h"
#include "rc_config.h"
#include "rc_event.h"
#include "subsystem2/sub_motion.h"
#include "subsystem2/drv_encoder.h"
#include "subsystem3/sub_line.h"
#include "subsystem3/sub_barcode.h"
#include "subsystem5/sub_scan.h"

/* FR6: a watch ping at or under this range starts avoidance. 150 mm
 * after the 19 Oct track runs (300 stopped too early; the car rolled
 * ~60 mm at 250 mm/s). 2026-10-29, Buddy 5: obstacles are placed 10-20 cm
 * ahead and anything at 20 cm or closer is bypassed, so 200 mm. At the
 * ~30 mm/s line speed the car stops within ~5 mm of the trigger (one
 * RC_PERIOD_SCAN_MS ping period plus the brake). */
#define NAV_WATCH_TRIGGER_MM    (200U)
/* After the trigger the car creeps up to this range before the spin scan
 * measures the obstacle. 150 was tried (2026-10-29 19:03-19:07); Buddy 5
 * set it back to the trigger range, so the scan runs where the car
 * stopped (the creep only runs if the car stopped further out). */
#define NAV_MEASURE_MM          (NAV_WATCH_TRIGGER_MM)

/* The forward watch always pings straight ahead (sub_scan.c, S_WATCH). */
#define NAV_WATCH_ANGLE_DEG     (90)

/* Bypass and barcode turns, spin in place. Open loop until Buddy 2's
 * turn calibration (TURN_SLIP_PERMILLE) is done. */
#define NAV_TURN_DEG            (90)
/* +10: U-turns came up short on the track (2026-10-29, Buddy 5) */
#define NAV_UTURN_DEG           (190)
/* Barcode left turns: +5 degrees, they came up short on the track
 * (2026-10-29, Buddy 5). Right turns and bypass turns stay at 90. */
#define NAV_TURN_LEFT_DEG       (NAV_TURN_DEG + 5)
/* After a junction turn the branch line is just ahead: sweep only this
 * far each side to find it (2026-10-29, Buddy 5; a lost line still
 * sweeps sub_line's full SEARCH_SWEEP_DEG). */
#define NAV_JUNCTION_SWEEP_DEG  (35)

/* Obstacle bypass shape (see "Obstacle bypass" below), 2026-10-31
 * (Buddy 5): the car leaves the line at NAV_BYPASS_OUT_DEG, runs
 * parallel past the obstacle and comes back in at NAV_BYPASS_IN_DEG,
 * both 45 deg. The distances come from the measured obstacle (sub_scan
 * spin_plan: lateral_mm, forward_mm); nothing about its size is fixed
 * here.
 *
 * NAV_BYPASS_OUT_STRETCH_PERMILLE = 1000 / sin(NAV_BYPASS_OUT_DEG): how
 * much further leg 1 must drive at that angle to step lateral_mm
 * sideways; NAV_BYPASS_OUT_COS_PERMILLE = cos x 1000: how much of leg 1
 * is forward progress (leg 2 is forward_mm less that). Change all three
 * together:
 *     30 deg -> 2000 / 866    45 deg -> 1414 / 707    60 deg -> 1155 / 500
 * recover_timeout_ms() takes leg 3 to be as long as leg 1, true while
 * NAV_BYPASS_IN_DEG = NAV_BYPASS_OUT_DEG. */
#define NAV_BYPASS_OUT_DEG              (45)
#define NAV_BYPASS_OUT_STRETCH_PERMILLE (1414U)
#define NAV_BYPASS_OUT_COS_PERMILLE     (707U)
#define NAV_BYPASS_IN_DEG               (45)
/* Detour spin turns slip on the foam: at NAV_DETOUR_MM_S (turns run at
 * sub_motion's 60 mm/s floor) a turn the encoders count as 45 deg turned
 * only ~30 deg (2026-10-31 17:01-17:04, wheels 44-46 mm = 45 deg every
 * time), so every bypass turn is asked for 1.5x. sub_motion's
 * TURN_SLIP_PERMILLE was calibrated on the floor at cruise speed. */
#define NAV_DETOUR_TURN_PERMILLE        (1500)
/* Bypass turns to the left are scaled by this on top (the left motor
 * is ~15 % weaker than the right, which sub_motion's per-wheel
 * feedforward FF_GAIN_L_PERMILLE also allows for). History: 1056 (95 /
 * 90, barcode turns 2026-10-29); 1150 (2026-10-31 17:07, "left is
 * worse"); at 1150 the left turn out looked ~60 deg while the right
 * turn back looked 45 (17:14), so 1150 x 45 / 60 = 862. */
#define NAV_LEFT_TURN_PERMILLE          (862)
/* Extra sideways step beyond the plan's lateral_mm (which already holds
 * the obstacle edge + CAR_WIDTH_MM / 2, 20 mm a side of margin). */
#define NAV_BYPASS_EXTRA_ASIDE_MM       (10U)

/* Rejoining the line (2026-10-31, Buddy 5). The car drives in at
 * NAV_BYPASS_IN_DEG, line steering off, until either line sensor sees
 * the line; then NAV_LINE_SENSOR_AHEAD_MM further, which puts the axle
 * where the sensors crossed it; then LINE SCAN: it spins in place toward
 * the plan side, noting the heading at which each sensor goes on to and
 * off the line. The line runs midway between those four headings (the
 * pair then straddles it, which straddle mode reads as centred), so it
 * turns there and follows. The crossing angle is measured, not assumed:
 * on 2026-10-31 16:42 turn errors over the detour made it much shallower
 * than 45, and the old fixed "85 mm on, turn 45, sweep +-35" ended with
 * both sensors off the line. */
#define NAV_LINE_SENSOR_AHEAD_MM        (80U)   /* measured 2026-10-29 */
/* The first sweep goes NAV_LINESCAN_SPARE_DEG past the planned re-entry
 * angle; if it misses, the second sweep goes back the other way to the
 * same amount past the heading the scan began at. */
#define NAV_LINESCAN_SPARE_DEG          (45)
/* Scan spin wheel speed: ~61 deg/s, ~0.6 deg per 10 ms line sample (the
 * slowest spin that does not stall on the foam, sub_motion). */
#define NAV_LINESCAN_MM_S               (60)
/* Line samples (10 ms each) to let the car stop before the final turn. */
#define NAV_LINESCAN_SETTLE             (15U)
/* Each wheel's travel per degree of spin in place, um: (W/2) x pi/180 */
#define NAV_UM_PER_DEG  ((uint32_t)RC_WHEEL_BASE_MM * 1000UL * 31416UL \
                         / (2UL * 180UL * 10000UL))

/* FR3 "C - go straight": a short forward move carries the car across the
 * junction before line following takes over again. Placeholder - tune on
 * the track with Buddy 3. */
#define NAV_STRAIGHT_MM         (100U)
/* 1/1 samples in a row (RC_PERIOD_LINE_MS each) that make a junction for
 * a stored barcode command: 30 ms, ~1 mm at line speed, so one noisy
 * sample can't fire it (an 18 mm crossing line gives ~58 samples). */
#define NAV_JUNCTION_CONFIRM    (3U)
/* A junction is seen by the line sensors at the front, ahead of the drive
 * wheels, as they reach the near edge of the crossing line. Before a
 * junction turn the car drives this far forward so the wheels (the spin
 * centre) sit over the junction: 80 mm sensors-to-wheels (Buddy 5's
 * measurement, 2026-10-29) + 9 mm, half the 18 mm crossing line. */
#define NAV_JUNCTION_FWD_MM     (80U + 9U)

/* Duty while RECOVERING: sub_line does not steer while SEARCHING, so nav
 * keeps the car rolling forward until it crosses the line.
 * TODO Buddy 2/3: tune on the track. 188 = 250 less 25 % (2026-10-19). */
#define NAV_RECOVER_PERMILLE    (188)

/* State timeouts, ms (RELTIM).
 *   RECOVER  FR9: the drive back in gets NAV_RECOVER_SLACK_PCT of its
 *            planned length to reach the line (recover_timeout_ms), each
 *            rejoin move CMD, the line scan its two sweeps; then STOPPED
 *            + RECOVERY_FAILED.
 *   BARCODE  G10: start character seen but no data character -> back to
 *            FOLLOWING. At the ~30 mm/s line speed one character takes
 *            ~2.5 s to pass the sensor (2026-10-29 16:50: narrow ~156 ms,
 *            wide ~460 ms; a whole *X* ~7 s), so the 2 s placeholder
 *            expired before the letter and it was ignored. 6 s = 2.4x.
 *   CMD      Backstop for EXECUTING_CMD, report §8 risk "nav could miss a
 *            MOTION_DONE and stall in EXECUTING_CMD": a 180 degree spin
 *            takes about 2 s at cruise speed, so 5 s means it was lost. */
#define NAV_RECOVER_SLACK_PCT   (150U)
#define NAV_RECOVER_SPARE_MS    (2000U)  /* ramps and settling */
#define NAV_BARCODE_TIMEOUT_MS  (6000)
#define NAV_CMD_TIMEOUT_MS      (5000)

/* The obstacle detour (AVOIDING's turns and moves, and RECOVERING's
 * drive back to the line) runs at the line-following speed, not the
 * cruise speed (2026-10-29, Buddy 5): sub_line's closed-loop base, 175
 * permille x LINE_MM_S_AT_175 63 x LINE_SPEED_PCT 50 % = ~32 mm/s. The
 * cruise speed is put back when the detour ends. */
#define NAV_DETOUR_MM_S         (32U)

/* ------------------------------------------------------------------ */

static volatile rc_nav_state_t state = RC_NAV_IDLE;
/* cruise speed saved while the detour runs slow (0 = not saved) */
static uint16_t saved_cruise_mm_s;
/* RECOVERING, step by step (see "Rejoining the line"): SEEK drives in
 * until a sensor sees the line, ONTO carries the axle onto it, SCAN
 * spins to find both sensors' crossings, SETTLE lets the car stop,
 * ALIGN turns to the middle. */
typedef enum {
    RJ_SEEK, RJ_ONTO, RJ_SCAN, RJ_SETTLE, RJ_ALIGN
} rejoin_t;
static rejoin_t rejoin;
static uint32_t rejoin_move;
/* line scan: spin direction (+1 left), heading where this sweep began
 * and where it stops (0.1 deg, > 0 left, 0 = where the scan began),
 * encoder counts where it began, which sweep (0, 1) */
static int32_t  ls_dir;
static int32_t  ls_base;
static int32_t  ls_limit;
static uint32_t ls_l0;
static uint32_t ls_r0;
static uint32_t ls_sweep;
/* per sensor (0 left, 1 right): headings where it went on to / off the
 * line, progress (0 not yet, 1 on, 2 on and off), its last sample */
static int32_t  ls_on[2];
static int32_t  ls_off[2];
static uint8_t  ls_seen[2];
static bool     ls_prev[2];
/* the heading the line runs along, and the stop countdown */
static int32_t  ls_target;
static uint32_t ls_settle;
/* the creep up to NAV_MEASURE_MM before the spin scan */
static uint32_t approach_move;

static bool         scan_pending;   /* we asked for the scan whose plan we await */
static rc_pl_plan_t plan;           /* copy of the plan being executed */
static uint32_t     bypass_step;    /* index into the bypass legs below */
static uint32_t     bypass_move;    /* id of the sub_motion move in flight */
static uint32_t     cmd_move;       /* id of the EXECUTING_CMD move */
/* Barcode command stored until the next junction (2026-10-29, Buddy 5):
 * a barcode is read ahead of the junction it applies to. A junction is
 * NAV_JUNCTION_CONFIRM samples of 1/1 on the line pair (both black: the
 * pair straddles the line just outside its edges, so only a crossing line
 * darkens both). Cleared on START and in STOPPED / IDLE. */
static rc_nav_cmd_t pending_cmd = RC_CMD_NONE;
static uint32_t     junction_count; /* 1/1 samples in a row, pending set */
/* the EXECUTING_CMD move is a junction turn: find the line with
 * sub_line's sweep when it ends (2026-10-29, Buddy 5) */
static bool         cmd_at_junction;
/* the turn to make once the NAV_JUNCTION_FWD_MM approach move is done */
static rc_nav_cmd_t junction_turn;

/* State timer: one kernel alarm handler, re-armed on entry to each timed
 * state. The handler only publishes RC_EVT_NAV_TIMEOUT (handlers must not
 * call sub_*); on_timeout() does the work on the FAST dispatcher. Every
 * arm/cancel bumps tmo_gen, and the handler stamps the event with it, so
 * a timeout that was already queued when the state changed is ignored. */
static ID                tmo_almid;
static volatile uint32_t tmo_gen;

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
 *  State timer
 * ------------------------------------------------------------------ */

/* Alarm handler: interrupt context. Publish only. */
static void tmo_handler(void *exinf)
{
    rc_event_t e = { 0 };

    (void)exinf;
    e.id         = RC_EVT_NAV_TIMEOUT;
    e.u.tick.seq = tmo_gen;
    (void)rc_event_publish_i(&e);
}

static void tmo_cancel(void)
{
    tmo_gen++;
    if (tmo_almid > 0) {
        (void)tk_stp_alm(tmo_almid);
    }
}

static void tmo_arm(RELTIM ms)
{
    tmo_gen++;                      /* before starting: the handler reads it */
    if (tmo_almid > 0) {
        (void)tk_sta_alm(tmo_almid, ms);
    }
}

/* ------------------------------------------------------------------ *
 *  Obstacle bypass (FR8a/FR8b)
 *
 *  Report §6.5: "a three-leg bypass (turn, forward, turn back)". Using
 *  the plan's two distances from sub_scan's build_plan():
 *
 *      leg 1  step aside   turn out NAV_BYPASS_OUT_DEG, forward far
 *                          enough to end lateral_mm +
 *                          NAV_BYPASS_EXTRA_ASIDE_MM to the side
 *      leg 2  run past     turn back NAV_BYPASS_OUT_DEG (parallel to
 *                          the line), forward forward_mm
 *      leg 3  re-enter     turn NAV_BYPASS_IN_DEG toward the line, then
 *                          RECOVERING drives forward until a line
 *                          sensor finds it, and rejoins (line scan)
 *
 *  Seen from above, car driving up the page on the line (|), bypassing
 *  obstacle # on the left:
 *
 *                 |
 *                /|     leg 3: 45 deg back in, crosses the line
 *               / |            still heading forward
 *              |  |
 *              |  #     leg 2: parallel, past the obstacle
 *              |  |
 *               \ |     leg 1: 45 deg out
 *                \|
 *                 |     (car stopped here when the watch fired)
 *
 *  Each move is queued, and its completion callback queues the next one.
 *  Nothing waits. A move that is refused (returns id 0) or ends with
 *  completed == false goes to STOPPED: the report treats any fault in a
 *  bypass leg as unrecoverable for Week 6.
 * ------------------------------------------------------------------ */

#define BYPASS_STEPS    (5U)

static void bypass_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx);

/* `deg` turned toward the side the plan chose (deg < 0 is left,
 * sub_motion.h); negate it to turn back toward the line. */
static int16_t toward_plan_side(int16_t deg)
{
    return (plan.action == RC_CMD_TURN_LEFT) ? (int16_t)(-deg) : deg;
}

/* A bypass spin turn of `deg` real degrees: NAV_DETOUR_TURN_PERMILLE
 * for the slip, left turns also NAV_LEFT_TURN_PERMILLE. */
static uint32_t bypass_turn(int16_t deg, sub_motion_done_cb_t cb)
{
    int32_t d = ((int32_t)deg * NAV_DETOUR_TURN_PERMILLE) / 1000;

    if (d < 0) {
        d = (d * NAV_LEFT_TURN_PERMILLE) / 1000;
    }
    return sub_motion_turn_deg((int16_t)d, cb, NULL);
}

static uint32_t bypass_issue(uint32_t step)
{
    switch (step) {
    case 0U:
        return bypass_turn(toward_plan_side(NAV_BYPASS_OUT_DEG), bypass_done);
    case 1U:
        return sub_motion_forward_mm(
            (((uint32_t)plan.lateral_mm + NAV_BYPASS_EXTRA_ASIDE_MM)
             * NAV_BYPASS_OUT_STRETCH_PERMILLE) / 1000U,
            bypass_done, NULL);
    case 2U:
        return bypass_turn((int16_t)(-toward_plan_side(NAV_BYPASS_OUT_DEG)),
                           bypass_done);
    case 3U: {
        /* leg 1 already carried the car this far forward */
        uint32_t leg1 = (((uint32_t)plan.lateral_mm + NAV_BYPASS_EXTRA_ASIDE_MM)
                         * NAV_BYPASS_OUT_STRETCH_PERMILLE) / 1000U;
        uint32_t done = (leg1 * NAV_BYPASS_OUT_COS_PERMILLE) / 1000U;
        uint32_t leg2 = ((uint32_t)plan.forward_mm > done)
                        ? ((uint32_t)plan.forward_mm - done) : 1U;
        return sub_motion_forward_mm(leg2, bypass_done, NULL);
    }
    case 4U:
        return bypass_turn((int16_t)(-toward_plan_side(NAV_BYPASS_IN_DEG)),
                           bypass_done);
    default:
        return 0U;
    }
}

/* RECOVERING's limit for the drive back in: leg 3 is as long as leg 1
 * (in and out at the same angle over the same sideways step), so
 * NAV_RECOVER_SLACK_PCT of that at NAV_DETOUR_MM_S, plus ramps. */
static RELTIM recover_timeout_ms(void)
{
    uint32_t leg = (((uint32_t)plan.lateral_mm + NAV_BYPASS_EXTRA_ASIDE_MM)
                    * NAV_BYPASS_OUT_STRETCH_PERMILLE) / 1000U;

    return (RELTIM)(((((leg * NAV_RECOVER_SLACK_PCT) / 100U) * 1000U)
                     / NAV_DETOUR_MM_S) + NAV_RECOVER_SPARE_MS);
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
 *  Executing a navigation command (FR3, FR10b)
 *
 *  Report §6.5 EXECUTING_CMD: "the decoded symbol is mapped A->left,
 *  B->right, C->straight, D->U-turn and issued as a queued encoder move;
 *  MOTION_DONE(completed) resumes following, MOTION_DONE(aborted) stops."
 *  Barcode commands (sub_barcode maps the symbol) and remote commands
 *  (Buddy 1's COMMAND_RX) both come through here.
 * ------------------------------------------------------------------ */

static void cmd_done(uint32_t move_id, bool completed,
                     uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_EXECUTING_CMD) || (move_id != cmd_move)) {
        return;     /* stale, e.g. cut short by a STOP or the backstop */
    }
    if (completed) {
        enter(RC_NAV_FOLLOWING);
        if (cmd_at_junction) {
            (void)sub_line_find(NAV_JUNCTION_SWEEP_DEG);  /* the branch line */
        }
    } else {
        tm_printf((UB *)"[nav] command move aborted\n");
        enter(RC_NAV_STOPPED);
    }
}

static void execute(rc_nav_cmd_t cmd, const char *source)
{
    tm_printf((UB *)"[nav] %s command %d\n", source, (INT)cmd);
    cmd_at_junction = false;        /* on_line_sample sets it after this */
    enter(RC_NAV_EXECUTING_CMD);

    switch (cmd) {
    case RC_CMD_TURN_LEFT:
        cmd_move = sub_motion_turn_deg((int16_t)(-NAV_TURN_LEFT_DEG), cmd_done, NULL);
        break;
    case RC_CMD_TURN_RIGHT:
        cmd_move = sub_motion_turn_deg((int16_t)NAV_TURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_U_TURN:
        cmd_move = sub_motion_turn_deg((int16_t)NAV_UTURN_DEG, cmd_done, NULL);
        break;
    case RC_CMD_GO_STRAIGHT:
        cmd_move = sub_motion_forward_mm(NAV_STRAIGHT_MM, cmd_done, NULL);
        break;
    default:
        cmd_move = 0U;
        break;
    }

    if (cmd_move == 0U) {
        tm_printf((UB *)"[nav] command move refused by sub_motion\n");
        enter(RC_NAV_STOPPED);
    }
}

/* The junction approach move ended: make the stored turn, from over the
 * junction (see NAV_JUNCTION_FWD_MM). */
static void junction_fwd_done(uint32_t move_id, bool completed,
                              uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_EXECUTING_CMD) || (move_id != cmd_move)) {
        return;     /* stale, e.g. cut short by a STOP or the backstop */
    }
    if (!completed) {
        tm_printf((UB *)"[nav] junction approach aborted\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    tmo_arm(NAV_CMD_TIMEOUT_MS);    /* a fresh backstop for the turn */
    execute(junction_turn, "junction"); /* already EXECUTING_CMD */
    cmd_at_junction = (state == RC_NAV_EXECUTING_CMD);
}

/* Runs a stored barcode command at a junction. A turn first drives
 * NAV_JUNCTION_FWD_MM so it pivots over the junction; straight is a plain
 * NAV_STRAIGHT_MM move across it. Either way sub_line's sweep finds the
 * line when the move is done (cmd_done). */
static void execute_at_junction(rc_nav_cmd_t cmd)
{
    if (cmd == RC_CMD_GO_STRAIGHT) {
        execute(cmd, "junction");
        cmd_at_junction = (state == RC_NAV_EXECUTING_CMD);
        return;
    }
    tm_printf((UB *)"[nav] junction: %d mm forward, then command %d\n",
              (INT)NAV_JUNCTION_FWD_MM, (INT)cmd);
    cmd_at_junction = false;
    junction_turn   = cmd;
    enter(RC_NAV_EXECUTING_CMD);
    cmd_move = sub_motion_forward_mm(NAV_JUNCTION_FWD_MM, junction_fwd_done,
                                     NULL);
    if (cmd_move == 0U) {
        tm_printf((UB *)"[nav] command move refused by sub_motion\n");
        enter(RC_NAV_STOPPED);
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
    tmo_cancel();                   /* every timed state arms its own */
    tm_printf((UB *)"[nav] %s -> %s\n",
              sub_nav_state_name(prev), sub_nav_state_name(next));

    /* the detour runs at NAV_DETOUR_MM_S; anything else at the cruise
     * speed it had before */
    if ((next == RC_NAV_AVOIDING) || (next == RC_NAV_RECOVERING)) {
        if (saved_cruise_mm_s == 0U) {
            saved_cruise_mm_s = sub_motion_get_speed();
        }
        (void)sub_motion_set_speed(NAV_DETOUR_MM_S);
    } else if (saved_cruise_mm_s != 0U) {
        (void)sub_motion_set_speed(saved_cruise_mm_s);
        saved_cruise_mm_s = 0U;
    } else {
        /* not in a detour */
    }

    switch (next) {
    case RC_NAV_FOLLOWING:
        /* sub_line steers through sub_motion_drive(); the barcode decoder
         * is armed for the whole of FOLLOWING (report §6.5); the watch
         * pings straight ahead every RC_PERIOD_SCAN_MS. Coming back from
         * READING_BARCODE the decoder is still armed: re-arming would
         * empty its window in the middle of a character (it did at the
         * old 2 s timeout, and the letter was never decoded). */
        (void)sub_line_enable(true);
        if (prev != RC_NAV_READING_BARCODE) {
            (void)sub_barcode_arm(true);
        }
        (void)sub_scan_set_watch(true, NAV_WATCH_TRIGGER_MM);
        break;

    case RC_NAV_READING_BARCODE:
        /* Entered from FOLLOWING on a start character. Keep steering, the
         * watch and the decoder exactly as they are: re-arming the decoder
         * would throw away the widths it is collecting. No separate
         * reading speed: barcodes are read at the line-following speed. */
        tmo_arm(NAV_BARCODE_TIMEOUT_MS);
        break;

    case RC_NAV_EXECUTING_CMD:
        /* The turn is an encoder move; line steering must be off, or
         * sub_line would fight it once the move ends. The watch is off
         * too: a spin in place sweeps the sensor across the room. */
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(false, 0U);
        tmo_arm(NAV_CMD_TIMEOUT_MS);
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
        /* FR9: drive in until a line sensor sees the line, then rejoin
         * (rejoin_sample, "Rejoining the line"); give up after
         * recover_timeout_ms (on_timeout). sub_line stays off: nav steers
         * the whole rejoin and FOLLOWING turns it back on. */
        rejoin = RJ_SEEK;
        (void)sub_line_enable(false);
        (void)sub_motion_drive_speed((int16_t)NAV_DETOUR_MM_S, 0);
        tmo_arm(recover_timeout_ms());
        break;

    case RC_NAV_STOPPED:
    case RC_NAV_IDLE:
    default:
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        /* The watch keeps pinging straight ahead while the car stands
         * still, so the range can be checked against a ruler on the
         * dashboard (2026-10-29, Buddy 5). on_ultra ignores it: only
         * FOLLOWING / READING_BARCODE start avoidance. Not in a TEST
         * build: its pings would collide with the S5 tests' own. */
        (void)sub_scan_set_watch(!APP_TEST, NAV_WATCH_TRIGGER_MM);
        if (sub_scan_busy()) {
            /* Only when a scan is running: sub_scan_abort() leaves
             * FLG_ABORT set, which would cut the NEXT scan short. */
            (void)sub_scan_abort();
        }
        scan_pending = false;
        pending_cmd  = RC_CMD_NONE;     /* a restart begins clean */
        (void)sub_motion_stop(true);
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Event handlers (all FAST lane)
 * ------------------------------------------------------------------ */

/* FR6: a forward-watch ping close enough to act on. Also while reading a
 * barcode: the watch is still on then, and an obstacle comes first. */
/* The spin scan that measures the obstacle (FR8a); its plan arrives in
 * on_plan. */
static void start_measure(void)
{
    /* servo and chassis, stopped per ping (sub_scan "Step scan",
     * 2026-10-31); the continuous chassis spin is sub_scan_start_spin() */
    if (sub_scan_start_measure() == RC_OK) {
        scan_pending = true;
    } else {
        tm_printf((UB *)"[nav] scan refused\n");
        enter(RC_NAV_STOPPED);
    }
}

static void approach_done(uint32_t move_id, bool completed,
                          uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_AVOIDING) || (move_id != approach_move)) {
        return;
    }
    if (completed) {
        start_measure();
    } else {
        tm_printf((UB *)"[nav] approach aborted\n");
        enter(RC_NAV_STOPPED);
    }
}

static void on_ultra(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if ((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE)) {
        return;     /* scan pings during AVOIDING land here too */
    }
    if (!evt->u.ultra.valid
        || (evt->u.ultra.angle_deg != NAV_WATCH_ANGLE_DEG)
        || (evt->u.ultra.range_mm > NAV_WATCH_TRIGGER_MM)) {
        return;
    }

    tm_printf((UB *)"[nav] obstacle at %d mm\n", (INT)evt->u.ultra.range_mm);
    enter(RC_NAV_AVOIDING);

    if (evt->u.ultra.range_mm > (NAV_MEASURE_MM + 5U)) {
        /* creep up to NAV_MEASURE_MM, then measure (approach_done) */
        approach_move = sub_motion_forward_mm(
            (uint32_t)evt->u.ultra.range_mm - NAV_MEASURE_MM,
            approach_done, NULL);
        if (approach_move == 0U) {
            enter(RC_NAV_STOPPED);
        }
    } else {
        start_measure();
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

/* ------------------------------------------------------------------ *
 *  Rejoining the line (FR9), see NAV_LINE_SENSOR_AHEAD_MM
 * ------------------------------------------------------------------ */

static void rejoin_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx);

/* Car heading during the line scan, 0.1 deg (> 0 left), from the
 * encoders: the mean wheel travel since this sweep began over
 * NAV_UM_PER_DEG. */
static int32_t ls_heading(void)
{
    uint32_t dl = drv_encoder_count(RC_SIDE_LEFT) - ls_l0;
    uint32_t dr = drv_encoder_count(RC_SIDE_RIGHT) - ls_r0;
    uint32_t um = ((dl + dr) / 2U) * RC_ENC_UM_PER_TICK;

    return ls_base + (ls_dir * (int32_t)((um * 10U) / NAV_UM_PER_DEG));
}

/* One sweep: spin in direction `dir` (+1 left) from heading `from` until
 * heading `limit` (rejoin_sample watches it). */
static void ls_sweep_start(int32_t dir, int32_t from, int32_t limit)
{
    ls_base    = from;
    ls_dir     = dir;
    ls_limit   = limit;
    ls_l0      = drv_encoder_count(RC_SIDE_LEFT);
    ls_r0      = drv_encoder_count(RC_SIDE_RIGHT);
    ls_seen[0] = 0U;
    ls_seen[1] = 0U;
    ls_prev[0] = false;
    ls_prev[1] = false;
    /* sub_motion steer > 0 turns right; dir > 0 is left */
    (void)sub_motion_drive_speed(0, (int16_t)(-dir * NAV_LINESCAN_MM_S));
}

/* The line scan: first toward the plan side (the way that turns the car
 * back along the line), NAV_LINESCAN_SPARE_DEG past the planned
 * re-entry angle. The timeout covers both sweeps. */
static void ls_begin(void)
{
    int32_t  dir = (plan.action == RC_CMD_TURN_LEFT) ? 1 : -1;
    uint32_t sweep_deg = (2U * NAV_BYPASS_IN_DEG) + (3U * NAV_LINESCAN_SPARE_DEG);

    rejoin   = RJ_SCAN;
    ls_sweep = 0U;
    /* um per degree / (mm per s) = ms per degree */
    tmo_arm((RELTIM)(((sweep_deg * NAV_UM_PER_DEG) / NAV_LINESCAN_MM_S)
                     + NAV_RECOVER_SPARE_MS));
    ls_sweep_start(dir, 0, dir * (NAV_BYPASS_IN_DEG + NAV_LINESCAN_SPARE_DEG) * 10);
}

/* FR9 / FR12: give up, brake, and say so. */
static void rejoin_fail(const char *why)
{
    rc_event_t out = { 0 };

    tm_printf((UB *)"[nav] rejoin failed: %s\n", why);
    out.id = RC_EVT_RECOVERY_FAILED;
    (void)rc_event_publish(&out);
    enter(RC_NAV_STOPPED);
}

/* RECOVERING's line samples (from on_line_sample). */
static void rejoin_sample(const rc_pl_line_t *s)
{
    bool     on[2];
    int32_t  h, d;
    uint32_t i;

    on[0] = s->on_line_l;
    on[1] = s->on_line_r;

    switch (rejoin) {
    case RJ_SEEK:
        if (on[0] || on[1]) {
            tm_printf((UB *)"[nav] line found: driving onto it\n");
            rejoin      = RJ_ONTO;
            tmo_arm(NAV_CMD_TIMEOUT_MS);
            rejoin_move = sub_motion_forward_mm(NAV_LINE_SENSOR_AHEAD_MM,
                                                rejoin_done, NULL);
            if (rejoin_move == 0U) {
                enter(RC_NAV_STOPPED);
            }
        }
        break;

    case RJ_SCAN:
        h = ls_heading();
        for (i = 0U; i < 2U; i++) {
            if (on[i] && !ls_prev[i] && (ls_seen[i] == 0U)) {
                ls_on[i]   = h;
                ls_seen[i] = 1U;
            } else if (!on[i] && ls_prev[i] && (ls_seen[i] == 1U)) {
                ls_off[i]  = h;
                ls_seen[i] = 2U;
            } else {
                /* no change */
            }
            ls_prev[i] = on[i];
        }
        if ((ls_seen[0] == 2U) && (ls_seen[1] == 2U)) {
            /* both sensors have crossed it: the line runs midway */
            ls_target = (ls_on[0] + ls_off[0] + ls_on[1] + ls_off[1]) / 4;
            tm_printf((UB *)"[nav] line scan: L %d..%d R %d..%d -> %d"
                      " (0.1 deg)\n", (INT)ls_on[0], (INT)ls_off[0],
                      (INT)ls_on[1], (INT)ls_off[1], (INT)ls_target);
            (void)sub_motion_stop(true);
            rejoin    = RJ_SETTLE;
            ls_settle = NAV_LINESCAN_SETTLE;
        } else if ((ls_dir * (h - ls_limit)) >= 0) {
            if (ls_sweep == 0U) {
                tm_printf((UB *)"[nav] line scan: sweeping back\n");
                ls_sweep = 1U;
                ls_sweep_start(-ls_dir, h, -ls_dir * NAV_LINESCAN_SPARE_DEG * 10);
            } else {
                (void)sub_motion_stop(true);
                rejoin_fail("line scan found no line");
            }
        } else {
            /* still sweeping */
        }
        break;

    case RJ_SETTLE:
        if (ls_settle > 0U) {
            ls_settle--;
            break;
        }
        d      = ls_target - ls_heading();      /* > 0: turn left */
        rejoin = RJ_ALIGN;
        if ((d > -20) && (d < 20)) {
            tm_printf((UB *)"[nav] on the line\n");
            enter(RC_NAV_FOLLOWING);
        } else {
            tmo_arm(NAV_CMD_TIMEOUT_MS);
            /* sub_motion: deg < 0 turns left; rounded to the degree.
             * Not bypass_turn(): the scan measured d in encoder degrees
             * at the same spin speed, so the slip is already in it. */
            rejoin_move = sub_motion_turn_deg(
                (int16_t)(-((d + ((d > 0) ? 5 : -5)) / 10)), rejoin_done, NULL);
            if (rejoin_move == 0U) {
                enter(RC_NAV_STOPPED);
            }
        }
        break;

    case RJ_ONTO:
    case RJ_ALIGN:
    default:
        break;                          /* a move is running */
    }
}

static void rejoin_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_RECOVERING) || (move_id != rejoin_move)) {
        return;
    }
    if (!completed) {
        tm_printf((UB *)"[nav] rejoin move aborted\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    if (rejoin == RJ_ONTO) {
        ls_begin();
    } else if (rejoin == RJ_ALIGN) {
        tm_printf((UB *)"[nav] on the line\n");
        enter(RC_NAV_FOLLOWING);
    } else {
        /* not expecting a move */
    }
}

/* FR3 / report §6.5: '*' (start or stop character) opens READING_BARCODE;
 * a data character while reading is STORED (pending_cmd) and run at the
 * next junction (on_line_sample), not at once (2026-10-29, Buddy 5). A
 * data character seen in FOLLOWING without a start character is ignored,
 * as the report enters READING_BARCODE only on a start character. */
static void on_barcode(const rc_event_t *evt, void *ctx)
{
    const rc_pl_barcode_t *b = &evt->u.barcode;

    (void)ctx;

    if (state == RC_NAV_FOLLOWING) {
        if (b->symbol == '*') {
            enter(RC_NAV_READING_BARCODE);
        }
    } else if (state == RC_NAV_READING_BARCODE) {
        if (b->command != RC_CMD_NONE) {
            pending_cmd    = b->command;    /* a newer barcode replaces it */
            junction_count = 0U;
            tm_printf((UB *)"[nav] barcode '%c': command %d stored for the"
                      " next junction\n", (INT)b->symbol, (INT)b->command);
            enter(RC_NAV_FOLLOWING);
        }
        /* '*' again (the stop character) or an unmapped symbol: keep
         * reading until the decode timeout. */
    } else {
        /* not expecting a barcode */
    }
}

/* A stored barcode command runs at the next junction: NAV_JUNCTION_CONFIRM
 * samples of 1/1 (both black) on the line pair while following. Not while
 * a barcode is being read: its bars can darken both sensors too, and the
 * read stays open until 2 s after its last edge (sub_barcode
 * NOISE_WINDOW_US), which covers the closing '*'. sub_line drives
 * straight over the 1/1 for those few samples; execute() then turns line
 * steering off for the turn. */
static void on_line_sample(const rc_event_t *evt, void *ctx)
{
    const rc_pl_line_t *s = &evt->u.line;
    rc_nav_cmd_t        cmd;

    (void)ctx;

    if (state == RC_NAV_RECOVERING) {
        rejoin_sample(s);
        return;
    }
    if ((pending_cmd == RC_CMD_NONE)
        || ((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE))
        || !s->on_line_l || !s->on_line_r || sub_line_reading()) {
        junction_count = 0U;
        return;
    }
    junction_count++;
    if (junction_count >= NAV_JUNCTION_CONFIRM) {
        cmd            = pending_cmd;
        pending_cmd    = RC_CMD_NONE;
        junction_count = 0U;
        execute_at_junction(cmd);
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

/* FR10b: STOP is honoured from any running state. START (G) begins the
 * run from IDLE (the MQTT build boots into IDLE and waits for it, see
 * app_main.c) and restarts it from STOPPED: put the car on the line
 * first. Turn / straight / U-turn commands are taken only while following
 * the line (FOLLOWING or READING_BARCODE); in the middle of a bypass, a
 * recovery or another command they are ignored. */
static void on_command_rx(const rc_event_t *evt, void *ctx)
{
    rc_nav_cmd_t cmd = evt->u.command.command;

    (void)ctx;

    if (state == RC_NAV_IDLE) {
        if (cmd == RC_CMD_START) {
            tm_printf((UB *)"[nav] START command\n");
            (void)sub_nav_start();
        }
        return;     /* anything else: not running yet */
    }
    if (cmd == RC_CMD_STOP) {
        tm_printf((UB *)"[nav] STOP command\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    if (cmd == RC_CMD_START) {
        if (state == RC_NAV_STOPPED) {
            tm_printf((UB *)"[nav] START command\n");
            (void)sub_nav_start();
        } else {
            tm_printf((UB *)"[nav] START ignored in %s\n",
                      sub_nav_state_name(state));
        }
        return;
    }
    execute(cmd, "remote");
}

/* FR9 / FR12: sub_line gave up after a sustained line loss during normal
 * following (its own 3 s search timeout + brake, Buddy 3's sub_line) and
 * published RECOVERY_FAILED. Bring nav to STOPPED so the mission state
 * matches the stopped car, with line following, the barcode decoder and
 * the forward watch all turned off (enter(STOPPED) does that). We also
 * publish RECOVERY_FAILED ourselves from the bypass-recovery timeout
 * below; receiving our own event while already STOPPED is a harmless
 * no-op, which is why this guards on the current state. */
static void on_recovery_failed(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;

    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        tm_printf((UB *)"[nav] recovery failed -> STOPPED\n");
        enter(RC_NAV_STOPPED);
    }
}

/* Our own state timer fired (tmo_handler). */
static void on_timeout(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (evt->u.tick.seq != tmo_gen) {
        return;     /* the state changed after the alarm was queued */
    }

    switch (state) {
    case RC_NAV_RECOVERING:
        rejoin_fail((rejoin == RJ_SEEK) ? "line not reached"
                                        : "rejoin step timed out");
        break;

    case RC_NAV_READING_BARCODE:
        tm_printf((UB *)"[nav] barcode not decoded, carrying on\n");
        enter(RC_NAV_FOLLOWING);
        break;

    case RC_NAV_EXECUTING_CMD:
        tm_printf((UB *)"[nav] command move never finished\n");
        enter(RC_NAV_STOPPED);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */

rc_result_t sub_nav_init(void)
{
    T_CALM calm = { 0 };

    calm.exinf  = NULL;
    calm.almatr = TA_HLNG;
    calm.almhdr = (FP)tmo_handler;
    tmo_almid   = tk_cre_alm(&calm);
    if (tmo_almid <= E_OK) {
        tmo_almid = 0;
        return RC_ERR_HARDWARE;
    }

    if ((rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_FAST,
                            on_ultra, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_AVOIDANCE_PLAN, RC_LANE_FAST,
                               on_plan, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_FAST,
                               on_barcode, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_FAST,
                               on_impact, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_COMMAND_RX, RC_LANE_FAST,
                               on_command_rx, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_NAV_TIMEOUT, RC_LANE_FAST,
                               on_timeout, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_RECOVERY_FAILED, RC_LANE_FAST,
                               on_recovery_failed, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_FAST,
                               on_line_sample, NULL) < 0)) {
        return RC_ERR_NOSPACE;
    }

    state = RC_NAV_IDLE;
#if !APP_TEST
    /* IDLE at boot: watch pings for the dashboard, as in enter(IDLE) */
    (void)sub_scan_set_watch(true, NAV_WATCH_TRIGGER_MM);
#endif
    return RC_OK;
}

rc_result_t sub_nav_start(void)
{
    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        return RC_ERR_STATE;
    }

    /* Report §6.5 IDLE: the IMU is calibrated (Buddy 4) and the base
     * speed set, 250 mm/s (Buddy 2), before this - in those buddies'
     * subsystemN_init()/_start(), which usermain() has already run. */
    scan_pending   = false;
    pending_cmd    = RC_CMD_NONE;
    junction_count = 0U;
    enter(RC_NAV_FOLLOWING);
    (void)sub_line_find(0);         /* start of a run: full sweep */
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

#if APP_TEST
rc_result_t sub_nav_test_force(rc_nav_state_t s)
{
    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        return RC_ERR_STATE;
    }
    scan_pending = false;
    enter(s);
    return RC_OK;
}
#endif
