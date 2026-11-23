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
 *  Rule for IDLE: every event except a START command is ignored. Other
 *  subsystems' self-tests publish fake LIFTED / COMMAND_RX events in a
 *  TEST=1 build; nav must not react to them before the run starts.
 *
 *  What talks to what (TEAM.md §5: IF-07..IF-09, IF-11, IF-14..IF-17):
 *    sub_nav -> sub_scan    set_watch() / start_measure() / abort() / busy()
 *                                                                (Buddy 5)
 *    sub_nav -> sub_motion  turn_deg() / forward_mm() / drive_speed() /
 *                           stop() / get_speed() / set_speed()   (Buddy 2)
 *    sub_nav -> sub_line    enable() / reading()                 (Buddy 3)
 *    sub_nav -> line_scan   start() / abort(): START line-up, bypass
 *                           rejoin, junction branch line         (Buddy 3)
 *    sub_nav -> sub_barcode arm()                                (Buddy 3)
 *    sub_nav -> sub_terrain level_here() on START                (Buddy 4)
 *    events in:  ULTRA_RESULT, AVOIDANCE_PLAN (S5), LINE_SAMPLE,
 *                BARCODE_DECODED (S3), LIFTED (S4), COMMAND_RX (S1),
 *                RECOVERY_FAILED (S3 or our own), NAV_TIMEOUT (our own
 *                alarm handler)
 *    events out: COMMAND_RX (sub_nav_stop / inject), RECOVERY_FAILED (FR9)
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_nav.h"
#include "rc_config.h"
#include "rc_event.h"
#include "rc_fault.h"
#include "rc_time.h"
#include "subsystem2/sub_motion.h"
#include "subsystem2/drv_encoder.h"
#include "subsystem3/line_scan.h"
#include "subsystem3/sub_line.h"
#include "subsystem3/sub_barcode.h"
#include "subsystem4/sub_terrain.h"
#include "subsystem5/sub_scan.h"

/* FR6: a watch ping at or under this range starts avoidance. Obstacles
 * are placed 10-20 cm ahead and anything at 20 cm or closer is bypassed.
 * At the ~30 mm/s line speed the car stops within ~5 mm of the trigger
 * (one RC_PERIOD_SCAN_MS ping period plus the brake). */
#define NAV_WATCH_TRIGGER_MM    (200U)
/* After the trigger the car creeps up to this range before the scan
 * measures the obstacle. Equal to the trigger, so the scan runs where the
 * car stopped (the creep only runs if the car stopped further out). */
#define NAV_MEASURE_MM          (NAV_WATCH_TRIGGER_MM)

/* The forward watch always pings straight ahead (sub_scan.c, S_WATCH). */
#define NAV_WATCH_ANGLE_DEG     (90)

/* Bypass and barcode turns, spin in place, in degrees asked of
 * sub_motion (which applies its TURN_SLIP_PERMILLE). */
#define NAV_TURN_DEG            (90)
/* +10: a 180 degree request comes up short on the foam */
#define NAV_UTURN_DEG           (190)
/* Barcode left turns: +5 degrees, because they come up short on the
 * foam. Right turns and bypass turns stay at 90. */
#define NAV_TURN_LEFT_DEG       (NAV_TURN_DEG + 5)
/* After a junction turn the branch line is just ahead: sweep only this
 * far each side to find it (a lost line still sweeps sub_line's full
 * SEARCH_SWEEP_DEG). */
#define NAV_JUNCTION_SWEEP_DEG  (35)

/* Obstacle bypass shape (see "Obstacle bypass" below): the car leaves
 * the line at NAV_BYPASS_OUT_DEG, runs parallel past the obstacle and
 * comes back in at NAV_BYPASS_IN_DEG, both 45 deg. The distances come
 * from the measured obstacle (sub_scan spin_plan: lateral_mm,
 * forward_mm); nothing about its size is fixed here.
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
 * sub_motion's 60 mm/s floor) the car turns well short of what the
 * encoders count, so every bypass turn is asked for 1.35x. How much it
 * slips varies with the mat and the tyres. sub_motion's
 * TURN_SLIP_PERMILLE was calibrated on the floor at cruise speed. */
#define NAV_DETOUR_TURN_PERMILLE        (1350)
/* Bypass turns to the left are scaled by this on top of
 * NAV_DETOUR_TURN_PERMILLE, so that left and right detour turns reach the
 * same real angle on the foam: 958 makes a left turn 1.29x. An eyeball
 * estimate: re-check with tape marks if the detour drifts. */
#define NAV_LEFT_TURN_PERMILLE          (958)
/* Extra sideways step beyond the plan's lateral_mm (which already holds
 * the obstacle edge + CAR_WIDTH_MM / 2, 20 mm a side of margin). Sized so
 * that, for lateral_mm up to ~95 mm, the car still clears the obstacle
 * when the turn out comes up ~15 deg short on the foam (leg 1 x sin 30
 * still exceeds lateral_mm). */
#define NAV_BYPASS_EXTRA_ASIDE_MM       (40U)

/* Rejoining the line. The car drives in at NAV_BYPASS_IN_DEG, line
 * steering off, until either line sensor sees
 * the line; then NAV_LINE_SENSOR_AHEAD_MM further, which puts the axle
 * where the sensors crossed it; then the line scan (subsystem3/line_scan.c)
 * in place, toward the plan side first: it finds the line both sensors
 * cross and turns the car along it (the pair then straddles it, which
 * straddle mode reads as centred). The crossing angle is measured, not
 * assumed, because turn errors over the detour can make it much
 * shallower than 45 degrees. */
#define NAV_LINE_SENSOR_AHEAD_MM        (80U)   /* sensors ahead of the axle */

/* Every line scan nav asks for (line_scan_cfg_t: first direction, quick
 * check, first sweep, second sweep, wheel speed). Speeds: 60 mm/s
 * (~61 deg/s, ~0.6 deg per 10 ms sample) is the slowest spin that does not
 * stall on the foam; START runs at 30 for twice the samples per crossing.
 *   REJOIN    after a bypass: NAV_BYPASS_IN_DEG + 45 toward the plan side,
 *             then 45 past the start heading the other way (FR10).
 *   START     FR16: if exactly one sensor is on black, the line is right
 *             beside the car on that side: a NAV_START_QUICK_DEG sweep
 *             toward it carries the other sensor across the same line
 *             (~20 degrees between the sensors, ~13 for the line at their
 *             radius, plus foam slip). Then NAV_START_SCAN_DEG left and
 *             right: a line up to ~45 degrees off. Nothing: STOPPED.
 *   JUNCTION  FR6: after a junction turn the branch line is ahead:
 *             NAV_JUNCTION_SWEEP_DEG each side, the turn's direction
 *             first (an under-turn leaves the line that way). */
#define NAV_LINESCAN_SPARE_DEG          (45)
#define NAV_LINESCAN_MM_S               (60)
#define NAV_START_SCAN_MM_S             (30)
#define NAV_START_QUICK_DEG             (40)
#define NAV_START_SCAN_DEG              (50)

/* FR3 "C - go straight": a short forward move carries the car across the
 * junction before line following takes over again. Placeholder, to be
 * tuned on the track. */
#define NAV_STRAIGHT_MM         (100U)
/* 1/1 samples in a row (RC_PERIOD_LINE_MS each) that make a junction for
 * a stored barcode command: 30 ms, ~1 mm at line speed, so one noisy
 * sample can't fire it (an 18 mm crossing line gives ~58 samples). */
#define NAV_JUNCTION_CONFIRM    (3U)
/* A junction is seen by the line sensors at the front, ahead of the drive
 * wheels, as they reach the near edge of the crossing line. Before a
 * junction turn the car drives this far forward so the wheels (the spin
 * centre) sit over the junction: 80 mm sensors-to-wheels + 9 mm, half
 * the 18 mm crossing line. */
#define NAV_JUNCTION_FWD_MM     (80U + 9U)

/* State timeouts, ms (RELTIM).
 *   RECOVER  FR9: the drive back in gets NAV_RECOVER_SLACK_PCT of its
 *            planned length to reach the line (recover_timeout_ms), the
 *            move onto it CMD; then STOPPED + RECOVERY_FAILED. A line scan
 *            keeps its own time limit (line_scan.c).
 *   BARCODE  G10: start character seen but no data character -> back to
 *            FOLLOWING. At the ~30 mm/s line speed one character takes
 *            ~2.5 s to pass the sensor (narrow ~156 ms, wide ~460 ms; a
 *            whole *X* ~7 s), so 6 s = 2.4x one character.
 *   CMD      Backstop for EXECUTING_CMD, report §8 risk "nav could miss a
 *            MOTION_DONE and stall in EXECUTING_CMD": a 180 degree spin
 *            takes about 2 s at cruise speed, so 5 s means it was lost. */
#define NAV_RECOVER_SLACK_PCT   (150U)
#define NAV_RECOVER_SPARE_MS    (2000U)  /* ramps and settling */
#define NAV_BARCODE_TIMEOUT_MS  (6000)
#define NAV_CMD_TIMEOUT_MS      (5000)

/* The obstacle detour (AVOIDING's turns and moves, and RECOVERING's
 * drive back to the line) runs at the line-following speed, not the
 * cruise speed: sub_line's closed-loop base, 175 permille x
 * LINE_MM_S_AT_175 63 x LINE_SPEED_PCT 50 % = ~32 mm/s. The cruise speed
 * is put back when the detour ends. */
#define NAV_DETOUR_MM_S         (32U)

/* Barcode re-read (see "Barcode re-read"): a pass that was clearly a
 * barcode but gave no letter is read again backwards, then forwards: three
 * passes in all, then the car carries on. The car reverses from where the
 * read ended to NAV_REREAD_BACK_EXTRA_MM before where it began (the read
 * is noticed at its third edge, and the decoder needs white before the
 * first bar), at NAV_DETOUR_MM_S, the speed of the first pass. It reverses
 * blind (the ultrasonic looks forward) over ground it has just crossed;
 * a read longer than NAV_REREAD_MAX_MM is not reversed over. */
#define NAV_REREAD_BACK_EXTRA_MM    (60U)
#define NAV_REREAD_MAX_MM           (450U)
/* Backstop for the reverse: 450 mm at 32 mm/s is ~14 s. */
#define NAV_REREAD_TIMEOUT_MS       (20000)

/* ------------------------------------------------------------------ */

static volatile rc_nav_state_t state = RC_NAV_IDLE;
/* cruise speed saved while the detour runs slow (0 = not saved) */
static uint16_t saved_cruise_mm_s;
/* RECOVERING, step by step (see "Rejoining the line"): SEEK drives in
 * until a sensor sees the line, ONTO carries the axle onto it, SCAN is
 * the line scan (line_scan.c). */
typedef enum {
    RJ_SEEK, RJ_ONTO, RJ_SCAN
} rejoin_t;
static rejoin_t rejoin;
static uint32_t rejoin_move;
/* this RECOVERING is the START line-up (NAV_START_SCAN_DEG), not a
 * bypass rejoin; cleared on entry to any other state */
static bool     rejoin_start;
/* a line scan nav started is running (sub_line's own lost-line scan is
 * not nav's to stop) */
static bool     nav_scanning;
/* the creep up to NAV_MEASURE_MM before the step scan */
static uint32_t approach_move;

static bool         scan_pending;   /* we asked for the scan whose plan we await */
static rc_pl_plan_t plan;           /* copy of the plan being executed */
static uint32_t     bypass_step;    /* index into the bypass legs below */
static uint32_t     bypass_move;    /* id of the sub_motion move in flight */
static uint32_t     cmd_move;       /* id of the EXECUTING_CMD move */
/* Barcode command stored until the next junction: a barcode is read
 * ahead of the junction it applies to. A junction is
 * NAV_JUNCTION_CONFIRM samples of 1/1 on the line pair (both black: the
 * pair straddles the line just outside its edges, so only a crossing line
 * darkens both). Cleared on START and in STOPPED / IDLE. */
static rc_nav_cmd_t pending_cmd = RC_CMD_NONE;
static uint8_t      pending_sym = RC_SYM_NONE;  /* the letter it came from */
static uint32_t     junction_count; /* 1/1 samples in a row, pending set */
/* the EXECUTING_CMD move is a junction turn: scan for the branch line
 * when it ends (junction_scan_begin) */
static bool         cmd_at_junction;
static rc_nav_cmd_t cmd_kind;       /* the EXECUTING_CMD command */
/* Barcode re-read: NONE = first pass; BACK = reversing over the barcode
 * (state READING_BARCODE, line steering, watch, junctions and remote
 * turns off); FWD = the last pass, following forward again. Back to NONE
 * when the last pass's summary arrives or the car leaves FOLLOWING /
 * READING_BARCODE. */
typedef enum {
    RR_NONE, RR_BACK, RR_FWD
} reread_t;
static reread_t     reread;
static uint32_t     reread_move;    /* id of the reverse move */
static uint32_t     reread_fwd_us;  /* the last pass began (bus time) */
/* a letter was read in this re-read, and whether it maps to a command
 * (see on_barcode: a command letter replaces one without; otherwise the
 * first letter stays) */
static bool         reread_got;
static bool         reread_got_cmd;
static bool         read_was_on;    /* sub_line_reading() last sample */
static uint32_t     read_start_mm;  /* odometer where the read began */
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
static void ls_begin(void);

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

/* Stores (or, with RC_CMD_NONE / RC_SYM_NONE, clears) the barcode command
 * waiting for the next junction, and publishes RC_EVT_CMD_STORED when it
 * changes, so the dashboard can show it. */
static void pending_set(rc_nav_cmd_t cmd, uint8_t sym)
{
    rc_event_t out = { 0 };

    if ((cmd == pending_cmd) && (sym == pending_sym)) {
        return;
    }
    pending_cmd = cmd;
    pending_sym = sym;
    out.id = RC_EVT_CMD_STORED;
    out.u.barcode.sym     = sym;
    out.u.barcode.command = cmd;
    (void)rc_event_publish(&out);
}

/* ------------------------------------------------------------------ *
 *  Obstacle bypass (FR8a/FR8b)
 *
 *  Report §6.5: "a three-leg bypass (turn, forward, turn back)". Using
 *  the plan's two distances from sub_scan's spin_plan():
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
 *  bypass leg as unrecoverable.
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

/* Leg 1's length, mm: far enough at NAV_BYPASS_OUT_DEG to end
 * lateral_mm + NAV_BYPASS_EXTRA_ASIDE_MM to the side. */
static uint32_t bypass_leg1_mm(void)
{
    return (((uint32_t)plan.lateral_mm + NAV_BYPASS_EXTRA_ASIDE_MM)
            * NAV_BYPASS_OUT_STRETCH_PERMILLE) / 1000U;
}

static uint32_t bypass_issue(uint32_t step)
{
    switch (step) {
    case 0U:
        return bypass_turn(toward_plan_side(NAV_BYPASS_OUT_DEG), bypass_done);
    case 1U:
        return sub_motion_forward_mm(bypass_leg1_mm(), bypass_done, NULL);
    case 2U:
        return bypass_turn((int16_t)(-toward_plan_side(NAV_BYPASS_OUT_DEG)),
                           bypass_done);
    case 3U: {
        /* leg 1 already carried the car this far forward */
        uint32_t done = (bypass_leg1_mm() * NAV_BYPASS_OUT_COS_PERMILLE) / 1000U;
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
    uint32_t leg = bypass_leg1_mm();

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
 *  (COMMAND_RX) both come through here.
 * ------------------------------------------------------------------ */

/* The junction command's line scan ended (line_scan.c, FAST lane). Found:
 * FOLLOWING. Not found: FR9 / FR12 as for a failed rejoin, STOPPED with
 * RECOVERY_FAILED. */
static void junction_scan_done(bool found, void *ctx)
{
    rc_event_t out = { 0 };

    (void)ctx;

    nav_scanning = false;
    if (state != RC_NAV_EXECUTING_CMD) {
        return;
    }
    if (found) {
        enter(RC_NAV_FOLLOWING);
        return;
    }
    tm_printf((UB *)"[nav] junction: branch line not found\n");
    out.id = RC_EVT_RECOVERY_FAILED;
    (void)rc_event_publish(&out);
    enter(RC_NAV_STOPPED);
}

/* FR6: after a junction command, the line scan for the branch line,
 * NAV_JUNCTION_SWEEP_DEG each side, the turn's direction first (a left
 * turn left; right and the U-turn, a right spin, right; straight left).
 * Stays in EXECUTING_CMD until the scan ends; the scan keeps its own time
 * limit, so the state timer stops. */
static void junction_scan_begin(void)
{
    line_scan_cfg_t c;

    c.first_dir  = ((cmd_kind == RC_CMD_TURN_RIGHT)
                    || (cmd_kind == RC_CMD_U_TURN)) ? (int16_t)-1 : (int16_t)1;
    c.quick_deg  = 0;
    c.first_deg  = NAV_JUNCTION_SWEEP_DEG;
    c.back_deg   = NAV_JUNCTION_SWEEP_DEG;
    c.speed_mm_s = NAV_LINESCAN_MM_S;
    tmo_cancel();
    if (line_scan_start(&c, junction_scan_done, NULL) == RC_OK) {
        nav_scanning = true;
    } else {
        tm_printf((UB *)"[nav] junction: line scan refused\n");
        enter(RC_NAV_STOPPED);
    }
}

static void cmd_done(uint32_t move_id, bool completed,
                     uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_EXECUTING_CMD) || (move_id != cmd_move)) {
        return;     /* stale, e.g. cut short by a STOP or the backstop */
    }
    if (completed && cmd_at_junction) {
        junction_scan_begin();      /* the branch line */
    } else if (completed) {
        enter(RC_NAV_FOLLOWING);
    } else {
        tm_printf((UB *)"[nav] command move aborted\n");
        enter(RC_NAV_STOPPED);
    }
}

static void execute(rc_nav_cmd_t cmd, const char *source)
{
    tm_printf((UB *)"[nav] %s command %d\n", source, (INT)cmd);
    cmd_at_junction = false;        /* on_line_sample sets it after this */
    cmd_kind        = cmd;
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
 * NAV_STRAIGHT_MM move across it. Either way a line scan finds the line
 * when the move is done (cmd_done, junction_scan_begin). */
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
    if (nav_scanning) {
        /* a scan nav started belongs to the state being left */
        line_scan_abort();
        nav_scanning = false;
    }
    if (next != RC_NAV_RECOVERING) {
        rejoin_start = false;
    }
    if ((next != RC_NAV_FOLLOWING) && (next != RC_NAV_READING_BARCODE)) {
        reread = RR_NONE;           /* a re-read ends with the line */
    }
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
        /* sub_line steers through sub_motion; the barcode decoder is
         * armed for the whole of FOLLOWING (report §6.5); the watch pings
         * straight ahead. Coming back from READING_BARCODE the decoder is
         * still armed: re-arming would empty its window in the middle of
         * a character. */
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
         * is taken from a stationary car. Line steering must be off
         * first, or sub_line would restart the motors on its next
         * sample. */
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        (void)sub_scan_set_watch(false, 0U);
        (void)sub_motion_stop(true);
        break;

    case RC_NAV_RECOVERING:
        /* FR9: drive in until a line sensor sees the line, then rejoin
         * (rejoin_sample, "Rejoining the line"); give up after
         * recover_timeout_ms (on_timeout). sub_line stays off: nav steers
         * the whole rejoin and FOLLOWING turns it back on. At START it is
         * only the line scan, in place (NAV_START_SCAN_DEG). */
        (void)sub_line_enable(false);
        if (rejoin_start) {
            ls_begin();
        } else {
            rejoin = RJ_SEEK;
            (void)sub_motion_drive_speed((int16_t)NAV_DETOUR_MM_S, 0);
            tmo_arm(recover_timeout_ms());
        }
        break;

    case RC_NAV_STOPPED:
    case RC_NAV_IDLE:
    default:
        (void)sub_line_enable(false);
        (void)sub_barcode_arm(false);
        /* The watch keeps pinging straight ahead while the car stands
         * still, so the range can be checked against a ruler on the
         * dashboard. on_ultra ignores it: only
         * FOLLOWING / READING_BARCODE start avoidance. Not in a TEST
         * build: its pings would collide with the S5 tests' own. */
        (void)sub_scan_set_watch(!APP_TEST, NAV_WATCH_TRIGGER_MM);
        if (sub_scan_busy()) {
            /* Only when a scan is running; the next scan request
             * clears any abort left over. */
            (void)sub_scan_abort();
        }
        scan_pending = false;
        pending_set(RC_CMD_NONE, RC_SYM_NONE);  /* a restart begins clean */
        (void)sub_motion_stop(true);
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Event handlers (all FAST lane)
 * ------------------------------------------------------------------ */

/* The step scan that measures the obstacle (FR8a): servo and chassis,
 * stopped per ping (sub_scan "Step scan"); its plan arrives in on_plan. */
static void start_measure(void)
{
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

/* FR6: a forward-watch ping close enough to act on. Also while reading a
 * barcode: the watch is still on then, and an obstacle comes first. */
static void on_ultra(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE))
        || (reread == RR_BACK) || RC_STAND_SOAK) {
        return;     /* scan pings during AVOIDING land here too */
    }
    if (!RC_ULTRA_VALID(&evt->u.ultra)
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

/* FR9 / FR12: give up, brake, and say so. */
static void rejoin_fail(const char *why)
{
    rc_event_t out = { 0 };

    tm_printf((UB *)"[nav] rejoin failed: %s\n", why);
    out.id = RC_EVT_RECOVERY_FAILED;
    (void)rc_event_publish(&out);
    enter(RC_NAV_STOPPED);
}

/* The START line-up found no line (or never finished): STOPPED. A wider
 * search that stops on the first black either sensor touches could pick
 * another line, a junction branch or a mark on the mat; the operator is at
 * the car at START, so they put it on the line and send START again. */
static void start_scan_missed(void)
{
    tm_printf((UB *)"[nav] start line scan found no line within %d deg:"
              " place the car on the line and send START\n",
              (INT)NAV_START_SCAN_DEG);
    enter(RC_NAV_STOPPED);
}

/* RECOVERING's line scan ended (line_scan.c, FAST lane). */
static void rejoin_scan_done(bool found, void *ctx)
{
    (void)ctx;

    nav_scanning = false;
    if ((state != RC_NAV_RECOVERING) || (rejoin != RJ_SCAN)) {
        return;
    }
    if (found) {
        tm_printf((UB *)"[nav] on the line\n");
        enter(RC_NAV_FOLLOWING);
    } else if (rejoin_start) {
        start_scan_missed();
    } else {
        rejoin_fail("line scan found no line");
    }
}

/* RECOVERING's line scan: the START line-up, or after a bypass toward the
 * plan side first (see NAV_LINESCAN_SPARE_DEG). The scan keeps its own
 * time limit, so the state timer stops. */
static void ls_begin(void)
{
    line_scan_cfg_t c;

    if (rejoin_start) {
        c.first_dir  = 1;
        c.quick_deg  = NAV_START_QUICK_DEG;
        c.first_deg  = NAV_START_SCAN_DEG;
        c.back_deg   = NAV_START_SCAN_DEG;
        c.speed_mm_s = NAV_START_SCAN_MM_S;
    } else {
        c.first_dir  = (plan.action == RC_CMD_TURN_LEFT) ? 1 : -1;
        c.quick_deg  = 0;
        c.first_deg  = NAV_BYPASS_IN_DEG + NAV_LINESCAN_SPARE_DEG;
        c.back_deg   = NAV_LINESCAN_SPARE_DEG;
        c.speed_mm_s = NAV_LINESCAN_MM_S;
    }
    rejoin = RJ_SCAN;
    tmo_cancel();
    if (line_scan_start(&c, rejoin_scan_done, NULL) == RC_OK) {
        nav_scanning = true;
    } else {
        tm_printf((UB *)"[nav] line scan refused\n");
        enter(RC_NAV_STOPPED);
    }
}

/* RECOVERING's line samples (from on_line_sample): the drive in, until a
 * sensor sees the line. */
static void rejoin_sample(const rc_pl_line_t *s)
{
    if ((rejoin != RJ_SEEK) || (!s->on_line_l && !s->on_line_r)) {
        return;
    }
    tm_printf((UB *)"[nav] line found: driving onto it\n");
    rejoin      = RJ_ONTO;
    tmo_arm(NAV_CMD_TIMEOUT_MS);
    rejoin_move = sub_motion_forward_mm(NAV_LINE_SENSOR_AHEAD_MM,
                                        rejoin_done, NULL);
    if (rejoin_move == 0U) {
        enter(RC_NAV_STOPPED);
    }
}

/* The drive onto the line ended: scan for it. */
static void rejoin_done(uint32_t move_id, bool completed,
                        uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_RECOVERING) || (move_id != rejoin_move)
        || (rejoin != RJ_ONTO)) {
        return;
    }
    if (!completed) {
        tm_printf((UB *)"[nav] rejoin move aborted\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    ls_begin();
}

/* ------------------------------------------------------------------ *
 *  Barcode re-read
 *
 *  Pass 1 is the normal read while following. If its summary
 *  (RC_EVT_BARCODE_SCAN, sent when the read ends, 2 s after its last
 *  edge) shows a barcode but no letter (scan_needs_reread), pass 2
 *  reverses over it with the decoder armed (Code 39 reads either way);
 *  pass 3 follows the line forward over it again. A letter from any pass
 *  is stored as usual (on_barcode). After pass 3 the car carries on
 *  whatever it read. The reverse stays in READING_BARCODE, so the
 *  dashboard shows a read in progress.
 * ------------------------------------------------------------------ */

/* Odometer: mean of both wheels' distance since the last move began
 * (line following is not a move, so it runs on through a read). */
static uint32_t odo_mm(void)
{
    return (drv_encoder_distance_mm(RC_SIDE_LEFT)
            + drv_encoder_distance_mm(RC_SIDE_RIGHT)) / 2U;
}

/* Was this pass clearly over a barcode, yet gave no letter? Then it is
 * worth reading again. More than NAV_REREAD_MIN_EDGES edges is well over
 * one character (9 elements) and far more than a tile joint or speck; a
 * pass that decoded only '*' still counts as no letter. text holds the
 * first 3 symbols decoded. */
#define NAV_REREAD_MIN_EDGES    (15U)

static bool scan_needs_reread(const rc_pl_bar_scan_t *s)
{
    uint32_t i;

    if (s->edges <= NAV_REREAD_MIN_EDGES) {
        return false;
    }
    for (i = 0U; (i < sizeof(s->text)) && (s->text[i] != '\0'); i++) {
        if (s->text[i] != '*') {
            return false;       /* a letter was read */
        }
    }
    return true;
}

/* The reverse (pass 2) ended: follow forward over the barcode again
 * (pass 3). The decoder is re-armed so the reverse read is closed (and
 * its summary sent, stamped before reread_fwd_us) before pass 3 starts. */
static void reread_back_done(uint32_t move_id, bool completed,
                             uint32_t travelled_mm, void *ctx)
{
    (void)travelled_mm;
    (void)ctx;

    if ((state != RC_NAV_READING_BARCODE) || (reread != RR_BACK)
        || (move_id != reread_move)) {
        return;
    }
    if (!completed) {
        tm_printf((UB *)"[nav] barcode re-read: reverse aborted\n");
        enter(RC_NAV_STOPPED);
        return;
    }
    (void)sub_barcode_arm(false);
    (void)sub_barcode_arm(true);
    reread        = RR_FWD;
    reread_fwd_us = rc_time_us();
    tm_printf((UB *)"[nav] barcode re-read: pass 3, forward\n");
    enter(RC_NAV_FOLLOWING);
}

/* Pass 2: reverse to NAV_REREAD_BACK_EXTRA_MM before where the read
 * began. Not when the distance is unknown (a move reset the odometer) or
 * longer than NAV_REREAD_MAX_MM. */
static void reread_begin(void)
{
    uint32_t now = odo_mm();
    uint32_t back;

    if (now < read_start_mm) {
        return;
    }
    back = (now - read_start_mm) + NAV_REREAD_BACK_EXTRA_MM;
    if (back > NAV_REREAD_MAX_MM) {
        tm_printf((UB *)"[nav] barcode re-read: %d mm is too far to"
                  " reverse, carrying on\n", (INT)back);
        return;
    }
    tm_printf((UB *)"[nav] barcode not read: pass 2, reversing %d mm\n",
              (INT)back);
    enter(RC_NAV_READING_BARCODE);
    reread         = RR_BACK;
    reread_got     = false;
    reread_got_cmd = false;
    (void)sub_line_enable(false);
    (void)sub_scan_set_watch(false, 0U);
    if (saved_cruise_mm_s == 0U) {
        saved_cruise_mm_s = sub_motion_get_speed();
    }
    (void)sub_motion_set_speed(NAV_DETOUR_MM_S);
    tmo_arm(NAV_REREAD_TIMEOUT_MS);
    reread_move = sub_motion_backward_mm(back, reread_back_done, NULL);
    if (reread_move == 0U) {
        tm_printf((UB *)"[nav] barcode re-read: reverse refused\n");
        enter(RC_NAV_STOPPED);
    }
}

/* A read ended (sub_barcode, FAST lane): start a re-read, or end one. */
static void on_bar_scan(const rc_event_t *evt, void *ctx)
{
    const rc_pl_bar_scan_t *s = &evt->u.bar_scan;

    (void)ctx;

    if ((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE)) {
        return;
    }
    switch (reread) {
    case RR_NONE:
        if (scan_needs_reread(s)) {
            reread_begin();
        }
        break;

    case RR_FWD:
        if ((int32_t)(evt->t_us - reread_fwd_us) < 0) {
            break;          /* the reverse read, closed as pass 3 began */
        }
        tm_printf((UB *)"[nav] barcode re-read done (%s), carrying on\n",
                  scan_needs_reread(s) ? "no letter on pass 3" : "read");
        reread = RR_NONE;
        break;

    case RR_BACK:
    default:
        break;              /* the reverse is judged when it ends */
    }
}

/* FR3 / report §6.5: '*' (start or stop character) opens READING_BARCODE;
 * a data character while reading is STORED (pending_cmd) and run at the
 * next junction (on_line_sample), not at once. A data character seen in
 * FOLLOWING without a start character is ignored, as the report enters
 * READING_BARCODE only on a start character. */
static void on_barcode(const rc_event_t *evt, void *ctx)
{
    const rc_pl_barcode_t *b = &evt->u.barcode;

    (void)ctx;

    if ((reread != RR_NONE) && (b->sym != RC_SYM_STAR)) {
        /* In a re-read a letter with a command (whichever letters
         * sub_barcode maps to commands: b->command) replaces one without;
         * otherwise the first letter stays, so pass 3 (or a late letter
         * from the reverse read) cannot swap one command for another. */
        bool has_cmd = (b->command != RC_CMD_NONE);

        if (reread_got && (!has_cmd || reread_got_cmd)) {
            tm_printf((UB *)"[nav] barcode re-read: '%c' ignored, already"
                      " read\n", (INT)rc_sym_char(b->sym));
            if ((state == RC_NAV_READING_BARCODE) && (reread == RR_FWD)) {
                enter(RC_NAV_FOLLOWING);
            }
            return;
        }
        reread_got     = true;
        reread_got_cmd = has_cmd;
    }

    if (state == RC_NAV_FOLLOWING) {
        if (b->sym == RC_SYM_STAR) {
            enter(RC_NAV_READING_BARCODE);
        }
    } else if (state == RC_NAV_READING_BARCODE) {
        if (b->command != RC_CMD_NONE) {
            pending_set(b->command, (uint8_t)b->sym);   /* a newer one replaces it */
            junction_count = 0U;
            tm_printf((UB *)"[nav] barcode '%c': command %d stored for the"
                      " next junction\n", (INT)rc_sym_char(b->sym), (INT)b->command);
            if (reread != RR_BACK) {
                enter(RC_NAV_FOLLOWING);    /* reversing: the move ends it */
            }
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
    if (sub_line_reading() && !read_was_on) {
        read_start_mm = odo_mm();   /* where a re-read reverses to */
    }
    read_was_on = sub_line_reading();
    if ((pending_cmd == RC_CMD_NONE) || (reread == RR_BACK)
        || ((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE))
        || !s->on_line_l || !s->on_line_r || sub_line_reading()) {
        junction_count = 0U;
        return;
    }
    junction_count++;
    if (junction_count >= NAV_JUNCTION_CONFIRM) {
        cmd            = pending_cmd;
        pending_set(RC_CMD_NONE, RC_SYM_NONE);
        junction_count = 0U;
        execute_at_junction(cmd);
    }
}

/* The car was lifted or overturned (sub_terrain): stop the motors and end
 * the run in STOPPED. Lifting the car is how a run is ended by hand; START
 * begins a new one. */
static void on_lifted(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if ((state != RC_NAV_IDLE) && (state != RC_NAV_STOPPED)) {
        tm_printf((UB *)"[nav] lifted (%d)\n", (INT)evt->u.lifted.why);
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
    if (((state != RC_NAV_FOLLOWING) && (state != RC_NAV_READING_BARCODE))
        || (reread == RR_BACK)) {
        tm_printf((UB *)"[nav] remote command %d ignored in %s\n",
                  (INT)cmd, sub_nav_state_name(state));
        return;
    }
    execute(cmd, "remote");
}

/* FR9 / FR12: sub_line gave up after a sustained line loss during normal
 * following (its search sweep found nothing and it braked) and
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
        if (rejoin_start) {
            (void)sub_motion_stop(true);
            start_scan_missed();
        } else {
            rejoin_fail((rejoin == RJ_SEEK) ? "line not reached"
                                            : "rejoin step timed out");
        }
        break;

    case RC_NAV_READING_BARCODE:
        if (reread == RR_BACK) {
            tm_printf((UB *)"[nav] barcode re-read: reverse never"
                      " finished\n");
            enter(RC_NAV_STOPPED);
            break;
        }
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
        || (rc_event_subscribe(RC_EVT_BARCODE_SCAN, RC_LANE_FAST,
                               on_bar_scan, NULL) < 0)
        || (rc_event_subscribe(RC_EVT_LIFTED, RC_LANE_FAST,
                               on_lifted, NULL) < 0)
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

    /* Report §6.5 IDLE: the IMU is calibrated and the base speed set
     * before this, in subsystemN_init()/_start(), which usermain() has
     * already run. */
    scan_pending   = false;
    pending_set(RC_CMD_NONE, RC_SYM_NONE);
    junction_count = 0U;
    /* the car stands on the track: that is level for the IMU, and START
     * acknowledges an earlier lift or watchdog reset */
    sub_terrain_level_here();
    rc_fault_clear(RC_FAULT_LIFTED | RC_FAULT_WDOG_RESET);
    /* line up on the line first (NAV_START_SCAN_DEG), then FOLLOWING */
    if (RC_STAND_SOAK) {
        enter(RC_NAV_FOLLOWING);    /* stand soak: no line-up scan */
        return RC_OK;
    }
    rejoin_start   = true;
    enter(RC_NAV_RECOVERING);
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
