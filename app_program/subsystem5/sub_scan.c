/*
 *  sub_scan.c
 *
 *  The "brain" of Buddy 5's module: drives the servo+ultrasonic pair
 *  through a scan (like slowly panning a security camera left to right),
 *  collects the (angle, distance) readings, and turns them into a
 *  description of the nearest obstacle plus a decision on what the car
 *  should do about it. See docs/buddy5-scan-avoidance/buddy5-scan-avoidance.md
 * for the full
 *  walkthrough and the sonar/security-camera analogies.
 */
#include "rc_prelude.h"
#include "sub_scan.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "rc_config.h"
#include "rc_event.h"

/* Anything further than this is background, not an obstacle. Both
 * build_profile() (deciding what counts as a "hit") and build_plan()
 * (deciding whether to react at all) use this same threshold, so a wall
 * or object further than 400mm away is simply ignored as "open space,"
 * exactly like a car's parking sensor going silent once nothing is close
 * enough to matter. */
#define OBSTACLE_MM         (400U)

/* Minimum gap the car needs to fit through, plus margin. Measure your
 * car's widest point and add 40 mm. Used by build_plan() to decide
 * whether a side has enough room to swerve into, and by build_profile()
 * indirectly (clearance_left_mm / clearance_right_mm are compared
 * against it). Currently a placeholder value — see the Buddy 5 guide
 * "How to get started" step 4. */
#define CAR_WIDTH_MM        (150U)

/* The scan state machine's states — `typedef enum` defines a new type
 * that can only hold one of these named values (see the fuller
 * explanation of this pattern in drv_ultrasonic.c's ultra_state_t). This
 * state machine is what lets scan_task() do a multi-second scan (moving
 * the servo, waiting for it to settle, pinging, waiting for the echo,
 * repeat) without ever blocking the rest of the car — each state
 * represents "what we're currently waiting on." */
typedef enum {
    /* nothing happening; ready to start a new scan on request */
    S_IDLE = 0,
    /* slow continuous forward ping while line-following, watching for something
     * getting close */
    S_WATCH,
    S_COARSE_MOVE,    /* servo is travelling to the next coarse-sweep angle */
    /* servo has arrived; waiting for the ultrasonic ping's echo result at this
     * angle */
    S_COARSE_PING,
    /* servo is travelling to the next fine-sweep angle (the zoomed-in re-scan
     * around the closest coarse hit) */
    S_FINE_MOVE,
    /* servo has arrived; waiting for the fine-sweep ping's echo result */
    S_FINE_PING,
    /* unused marker state (scan_task returns to S_IDLE or S_WATCH directly
     * instead) */
    S_DONE
} scan_state_t;

/* One measurement: the servo angle it was taken at, the distance
 * measured (millimetres), and whether the reading is trustworthy (false
 * if the ultrasonic driver reported a timeout/out-of-range result — see
 * drv_ultrasonic.c). A "struct" bundles several related values into one
 * named unit, the same way a form bundles several fields onto one sheet
 * of paper instead of keeping them as separate loose notes. */
typedef struct {
    int16_t  angle;
    uint16_t range_mm;
    bool     valid;
} point_t;

/* All `static` (file-private, see sub_line.c for the fuller explanation)
 * globals holding the scan module's running state. */
/* current state-machine state, see scan_state_t above */
static scan_state_t state;
/* every (angle, range) reading collected so far this scan — an "array", i.e. a
 * fixed-size row of point_t slots indexed 0..RC_SCAN_MAX_POINTS-1 */
static point_t      points[RC_SCAN_MAX_POINTS];
/* how many of the slots in `points` are actually filled in right now */
static uint32_t     n_points;
/* angle the servo was most recently commanded to (used to tag the next
 * ultrasonic result) */
static int16_t      cur_angle;
/* start angle of the fine (zoomed-in) re-scan window, set by find_fine_window()
 * start angle of the fine (zoomed-in) re-scan window, set by
 * find_fine_window() */
static int16_t      fine_from;
/* end angle of the fine re-scan window */
static int16_t      fine_to;
/* distance threshold for the forward watch mode (currently stored but not yet
 * acted on here — sub_nav.c decides when to escalate to a full scan) */
static uint16_t     watch_trigger_mm = 300U;
/* whether continuous forward-watch mode is currently requested */
static bool         watch_on;
/* kernel task ID for the background scan_task */
static ID           scan_tskid;
/* kernel "event flag" object ID used to wake scan_task when something happens
 * (see FLG_* below) */
static ID           scan_flgid;
/* optional callback registered via sub_scan_on_complete() */
static sub_scan_done_cb_t user_cb;
/* free-form context pointer passed back to user_cb unchanged */
static void        *user_ctx;

/* Event flag bits. An "event flag" is a kernel object a task can sleep
 * on (tk_wai_flg) until one of a chosen set of bits gets set by someone
 * else (tk_set_flg) — the RTOS equivalent of a doorbell the task waits
 * for instead of constantly checking a mailbox. `1U << 0` etc. give each
 * meaning its own separate bit, so multiple things can be signalled at
 * once without colliding (see the bitwise-shift note in
 * drv_ultrasonic.c's alarm_arm for the same trick used on hardware
 * registers). */
/* an ultrasonic ping just produced a result */
#define FLG_RESULT      (1U << 0)
/* sub_scan_start() was called; begin a scan */
#define FLG_START       (1U << 1)
/* sub_scan_abort() was called; stop what's running */
#define FLG_ABORT       (1U << 2)

/* ------------------------------------------------------------------ *
 *  Profiling
 *
 *  Turns the collected (angle, range) points into the five numbers the
 *  brief asks for. All integer, all cheap.
 * ------------------------------------------------------------------ */

/* Walks every collected (angle, range) reading once and boils it down
 * into the five numbers the mission logic actually needs: how close is
 * the nearest thing, at what bearing, how wide does it look, and how
 * much open space is there on each side of straight-ahead. `p` is a
 * pointer to the caller's output struct — this function fills it in
 * rather than returning a value, since it needs to set five separate
 * fields at once (C functions can only return one value directly). */
static void build_profile(rc_pl_profile_t *p)
{
    uint32_t i;
    /* running "smallest distance seen so far"; 0xFFFF (65535) is a deliberately
     * huge starting value nothing real can beat */
    uint16_t closest = 0xFFFFU;
    /* angle of the closest hit so far; defaults to straight ahead */
    int16_t  closest_angle = 90;
    /* leftmost/first angle (in sweep order) that counted as "close" (-1 = none
     * yet) */
    int16_t  first_hit = -1;
    /* rightmost/last angle that counted as "close" */
    int16_t  last_hit = -1;
    /* count of bearings to the left of centre that were free of obstacles */
    uint32_t clear_l = 0U;
    /* count of bearings to the right of centre that were free of obstacles */
    uint32_t clear_r = 0U;

    p->n_points          = 0U;
    p->closest_angle_deg = 0;
    p->closest_mm        = 0U;
    p->width_mm          = 0U;
    p->clearance_left_mm = 0U;
    p->clearance_right_mm = 0U;
    p->n_points = (uint8_t)n_points;

    /* `for (i = 0; i < n_points; i++)` walks the array of collected
     * points from index 0 up to (but not including) n_points — the
     * standard C way to visit every filled-in element of an array one at
     * a time. */
    for (i = 0U; i < n_points; i++) {
        if (!points[i].valid) {
            /* skip readings the ultrasonic driver marked as failed/out-of-range
             * skip readings the ultrasonic driver marked as failed/out-of-
             * range */
            continue;
        }
        if (points[i].range_mm < closest) {
            closest       = points[i].range_mm;
            closest_angle = points[i].angle;
        }
        if (points[i].range_mm < OBSTACLE_MM) {
            /* This reading counts as "close" (an obstacle, not open
             * background). Track the first and last angle (in sweep
             * order) that qualified — together they mark the angular
             * span the obstacle occupies, used below for the width
             * estimate. */
            if (first_hit < 0) {
                first_hit = points[i].angle;
            }
            last_hit = points[i].angle;
        } else {
            /* Free bearings on each side of straight ahead become the
             * clearance figures. Counting bearings rather than measuring
             * a gap is crude but it is what a single ranging beam can
             * honestly support. */
            if (points[i].angle < 90) {
                clear_r++;
            } else if (points[i].angle > 90) {
                clear_l++;
            } else {
                /* dead ahead, neither side */
            }
        }
    }

    if (closest == 0xFFFFU) {
        /* nothing seen at all — still at the untouched 0xFFFF starting value,
         * so no reading ever beat it */
        return;
    }

    p->closest_mm        = closest;
    p->closest_angle_deg = closest_angle;

    /*
     *  Estimating obstacle width from the angular span at the measured
     *  distance — the trigonometry, spelled out:
     *
     *      width = 2 * range * tan(span/2)
     *
     *  Picture the sensor at the tip of a triangle, with the obstacle's
     *  left and right edges as the other two corners; `range` is roughly
     *  the distance to the middle of it, and `span` is the angle (in
     *  degrees) the obstacle sweeps through from the sensor's point of
     *  view (first_hit to last_hit). Basic right-triangle trig says
     *  half the width is range * tan(half the span) — this is the same
     *  formula an old-fashioned surveyor would use to estimate the width
     *  of something across a field just by pacing off one distance and
     *  reading two angles.
     *
     *  Why tan(x) is replaced with plain x here: for spans under about
     *  60 degrees, tan(x) (in radians) is close enough to x itself that
     *  the error is smaller than the HC-SR04's own 15 degree beam width
     *  (i.e. the sensor's own fuzziness already dwarfs this
     *  approximation's error). This "small-angle approximation" trades a
     *  tiny bit of accuracy for completely avoiding a call to a real
     *  tan() function — the RP2040 has no hardware floating-point unit
     *  (FPU), so a software tan() would be slow and would also need
     *  float/double math, which this whole codebase avoids (see
     *  TEAM_GUIDE.md §5). Everything below is done with only integer
     *  multiply/divide.
     *
     *  The arithmetic, step by step:
     *    span_ddeg: the angular span in *tenths of a degree* (so it's
     *      still a whole number even though the real difference may not
     *      be a multiple of 10) — last_hit minus first_hit, times 10.
     *    span_mrad: that span converted to *milliradians* (thousandths of
     *      a radian) — since 1 degree = pi/180 radians = 17.4533 tenths
     *      of a milliradian per tenth-of-a-degree, multiplying by 1745
     *      and dividing by 1000 does that conversion using only integers.
     *    width_mm: closest (the distance in mm) times span_mrad (the
     *      angle in "radians x 1000"), divided by 1000 to undo that same
     *      scaling — this is exactly `range * span_in_radians`, which by
     *      the small-angle approximation above stands in for
     *      `2 * range * tan(span/2)` (the factor of 2 and the /2 on the
     *      angle cancel out algebraically, which is why the code doesn't
     *      show them separately).
     */
    if ((first_hit >= 0) && (last_hit >= first_hit)) {
        int32_t span_ddeg = ((int32_t)last_hit - (int32_t)first_hit) * 10;
        int32_t span_mrad = (span_ddeg * 1745) / 1000;
        p->width_mm = (uint16_t)(((int32_t)closest * span_mrad) / 1000);
    }

    /* Clearance isn't measured as an actual gap width — it's a count of
     * how many sampled bearings on each side came back "clear" (further
     * than OBSTACLE_MM), multiplied by the angular spacing between
     * samples (RC_SCAN_COARSE_STEP degrees apart) to turn "N clear
     * bearings" into a rough linear distance. This is intentionally
     * crude (see the comment above the loop) — a single narrow ranging
     * beam sweeping in steps can only honestly report "this direction
     * was open," not a precise continuous gap width. */
    p->clearance_left_mm  = (uint16_t)(clear_l * RC_SCAN_COARSE_STEP);
    p->clearance_right_mm = (uint16_t)(clear_r * RC_SCAN_COARSE_STEP);
}

/* ------------------------------------------------------------------ *
 *  Avoidance planning
 *
 *  TODO Buddy 5: this picks a side and a step distance from the
 *  clearance counts. It is a starting point, not a finished planner.
 *  Two things to add once it drives:
 *
 *    1. Remember which side you went last time. On a narrow track,
 *       alternating sides on successive obstacles wastes distance.
 *    2. The brief lists "reverse and reattempt". That case belongs here,
 *       when both clearances are below CAR_WIDTH_MM.
 * ------------------------------------------------------------------ */

/* Turns the profile built above into an actual decision: go straight,
 * turn left/right by how much, or stop. `p` is read-only input (the
 * profile); `plan` is the output this function fills in — same "pointer
 * as an out-parameter" pattern as build_profile above. */
static void build_plan(const rc_pl_profile_t *p, rc_pl_plan_t *plan)
{
    plan->action     = RC_CMD_NONE;
    plan->lateral_mm = 0U;
    plan->forward_mm = 0U;

    if (p->closest_mm == 0U) {
        /* build_profile() left closest_mm at its initial 0 only when it
         * returned early having seen nothing at all — i.e. a totally
         * empty scan, not "something 0mm away." Treat that as clear. */
        plan->action = RC_CMD_GO_STRAIGHT;
        return;
    }

    if (p->closest_mm > OBSTACLE_MM) {
        /* Nearest thing found is still further than the "it's an
         * obstacle" threshold — treat it as background, not a reason to
         * react. */
        plan->action = RC_CMD_GO_STRAIGHT;
        return;
    }

    if ((p->clearance_left_mm < CAR_WIDTH_MM)
        && (p->clearance_right_mm < CAR_WIDTH_MM)) {
        /* Neither side has enough measured clearance to fit the car
         * through — nowhere safe to swerve, so stop rather than try to
         * squeeze past. See the TODO below about adding a "reverse and
         * reattempt" option here instead of just stopping. */
        plan->action = RC_CMD_STOP;
        return;
    }

    /* Pick whichever side reported more open space. `>=` (rather than
     * `>`) means a tie is broken toward turning left — an arbitrary but
     * consistent choice (see the TODO below about remembering the last
     * direction chosen instead). */
    if (p->clearance_left_mm >= p->clearance_right_mm) {
        plan->action = RC_CMD_TURN_LEFT;
    } else {
        plan->action = RC_CMD_TURN_RIGHT;
    }

    /* Step aside by half the car width plus half the obstacle width, and
     * run past it by its width plus a margin. In other words: swerve
     * just far enough sideways that the car's near edge clears the
     * obstacle's far edge, then drive forward far enough to be fully
     * past it before straightening out again. These are simple geometric
     * estimates, not a measured path — sub_motion.c (Buddy 2's module)
     * is what actually executes the resulting turn/distance commands. */
    plan->lateral_mm = (uint16_t)((CAR_WIDTH_MM / 2U) + (p->width_mm / 2U));
    plan->forward_mm = (uint16_t)(p->width_mm + 100U);
}

/* ------------------------------------------------------------------ *
 *  Result callback from the ultrasonic driver. INTERRUPT context, so it
 *  does the absolute minimum and hands off to the scan task.
 * ------------------------------------------------------------------ */

/* Registered with drv_ultrasonic_on_result() in sub_scan_init(). This
 * runs in INTERRUPT context (see drv_ultrasonic.h's note on
 * drv_ultra_cb_t), so it does the bare minimum: store the reading into
 * the next free slot of `points` (guarded by a bounds check so a scan
 * can never write past the end of the fixed-size array — a classic C
 * bug this codebase avoids by checking first), then wake scan_task by
 * setting FLG_RESULT. All the real work (turning points into a profile
 * and a plan) happens later in normal task context. */
static void on_range(uint16_t range_mm, bool valid, void *ctx)
{
    (void)ctx;

    if (n_points < RC_SCAN_MAX_POINTS) {
        points[n_points].angle    = cur_angle;
        points[n_points].range_mm = range_mm;
        points[n_points].valid    = valid;
        n_points++;
    }
    (void)tk_set_flg(scan_flgid, FLG_RESULT);
}

/* ------------------------------------------------------------------ *
 *  Scan task
 *
 *  Every wait in here is a kernel wait with a timeout, so the task is
 *  off the run queue for the whole servo travel and the whole 30 ms of
 *  ultrasonic flight time.
 * ------------------------------------------------------------------ */

/* Runs once a scan (coarse, and fine if applicable) has finished
 * collecting points. Builds the profile and plan, publishes both on the
 * event bus, calls any registered direct callback, and resets the state
 * machine to idle. This is the single place a completed scan gets
 * reported to the rest of the car. */
static void publish_and_finish(void)
{
    rc_pl_profile_t profile;
    rc_pl_plan_t    plan;
    rc_event_t      evt;

    build_profile(&profile);
    build_plan(&profile, &plan);

    /* Publish on the event bus (see TEAM_GUIDE.md §1.2) so any
     * subscriber finds out, without this module needing to know who's
     * listening. sub_nav.c (the shared mission state machine) subscribes
     * to RC_EVT_AVOIDANCE_PLAN to know when to leave RC_NAV_AVOIDING and
     * hand off to sub_motion.c's turn/drive commands; sub_telemetry.c
     * (Buddy 1) subscribes to RC_EVT_OBSTACLE_PROFILE to report it. */
    evt.id         = RC_EVT_OBSTACLE_PROFILE;
    evt.u.profile  = profile;
    (void)rc_event_publish(&evt);

    evt.id    = RC_EVT_AVOIDANCE_PLAN;
    evt.u.plan = plan;
    (void)rc_event_publish(&evt);

    /* Also call the optional direct callback registered via
     * sub_scan_on_complete(), if any. */
    if (user_cb != NULL) {
        user_cb(&profile, &plan, user_ctx);
    }

    state = S_IDLE;
}

/* Moves the servo to `angle`, waits (via a real kernel sleep, not a
 * busy-loop — see tk_dly_tsk below) for it to settle, then starts one
 * ultrasonic ping at that angle. Returns true if the ping was
 * successfully started. This is the one function both the coarse and
 * fine sweep loops call for each angle they visit — see run_coarse() and
 * run_fine() below. */
static bool step_to(int16_t angle)
{
    /* Ask the servo driver how long this particular move (from wherever
     * it currently is, to `angle`) will take to settle — see
     * drv_servo_settle_ms()'s own comments in drv_servo.c for the math. */
    uint32_t settle = drv_servo_settle_ms(drv_servo_get_angle(), angle);

    /* remember the angle so on_range() can tag the coming result with it */
    cur_angle = angle;
    (void)drv_servo_set_angle(angle);
    /* tk_dly_tsk() puts THIS task to sleep for `settle` milliseconds,
     * handing the CPU to other tasks (like the motion PID loop) in the
     * meantime — this is the "real kernel wait" the Buddy 5 guide
     * section and this file's own header comment refer to; it's the
     * opposite of a busy-wait loop that would hog the CPU doing nothing. */
    (void)tk_dly_tsk((RELTIM)settle);

    return (drv_ultrasonic_ping(angle) == RC_OK);
}

/* Blocks (sleeps) this task until either an ultrasonic result arrives
 * (FLG_RESULT) or a scan abort is requested (FLG_ABORT), whichever comes
 * first — or until the 100ms timeout expires. Returns true only if a
 * real result arrived without an abort. */
static bool wait_result(void)
{
    UINT ptn;
    ER   er;

    /* Timeout is the driver's own echo timeout plus slack. If the driver
     * is working this never expires; it is here so a wedged measurement
     * cannot stall the scan forever.
     * TWF_ORW = wait for ANY of the listed bits (OR), not all of them.
     * TWF_CLR = automatically clear the bit(s) once they wake this task,
     * so the next wait starts fresh instead of immediately re-triggering
     * on a stale flag. */
    er = tk_wai_flg(scan_flgid, FLG_RESULT | FLG_ABORT,
                    TWF_ORW | TWF_CLR, &ptn, 100);

    return (er == E_OK) && ((ptn & FLG_ABORT) == 0U);
}

/* The coarse sweep: steps the servo from RC_SCAN_COARSE_START to
 * RC_SCAN_COARSE_END in RC_SCAN_COARSE_STEP increments (constants
 * defined in rc_config.h), pinging and collecting a result at each stop.
 * This is the "look broadly across the whole arc" first pass — like
 * slowly panning a security camera all the way across a room once before
 * zooming in on anything interesting. Stops early (returns) if a
 * wait_result() call reports an abort or timeout. */
static void run_coarse(void)
{
    int16_t a;

    n_points = 0U;   /* start this scan's point collection fresh */

    for (a = RC_SCAN_COARSE_START; a <= RC_SCAN_COARSE_END;
         a = (int16_t)(a + RC_SCAN_COARSE_STEP)) {
        state = S_COARSE_MOVE;
        if (!step_to(a)) {
            /* ping failed to start (e.g. driver busy); skip this angle rather
             * than getting stuck */
            continue;
        }
        state = S_COARSE_PING;
        if (!wait_result()) {
            /* aborted or wedged — bail out of the whole coarse sweep */
            return;
        }
    }
}

/* Looks back over the coarse sweep's collected points to find the single
 * closest "obstacle" hit, then computes a narrow angular window
 * (fine_from..fine_to) centred on it for a more detailed re-scan. This
 * is the "zoom in on what you found" step — like a security camera
 * panning broadly, spotting movement, then swivelling back to get a
 * closer look at just that spot. Returns false if nothing close enough
 * to be worth a fine scan was found (in which case run_fine() is simply
 * skipped — see scan_task below). */
static bool find_fine_window(void)
{
    uint32_t i;
    uint16_t closest = 0xFFFFU;
    int16_t  best = -1;

    for (i = 0U; i < n_points; i++) {
        if (points[i].valid && (points[i].range_mm < closest)
            && (points[i].range_mm < OBSTACLE_MM)) {
            closest = points[i].range_mm;
            best    = points[i].angle;
        }
    }

    if (best < 0) {
        return false;
    }

    fine_from = (int16_t)(best - RC_SCAN_COARSE_STEP);
    fine_to   = (int16_t)(best + RC_SCAN_COARSE_STEP);

    /* Clamp the fine window to the servo's physical travel limits, the
     * same way drv_servo_set_angle() clamps a single angle — a window
     * centred near either end of the sweep must not ask for an angle the
     * hardware can't reach. */
    if (fine_from < RC_SERVO_ANGLE_MIN) {
        fine_from = RC_SERVO_ANGLE_MIN;
    }
    if (fine_to > RC_SERVO_ANGLE_MAX) {
        fine_to = RC_SERVO_ANGLE_MAX;
    }
    return true;
}

/* The fine sweep: re-scans only the narrow fine_from..fine_to window
 * (set by find_fine_window() above) at the tighter RC_SCAN_FINE_STEP
 * angular spacing, adding these extra, more closely-spaced points on top
 * of whatever run_coarse() already collected in `points`. This sharpens
 * the eventual width/clearance estimate around the one obstacle that
 * matters, without paying the cost of scanning the whole arc that
 * finely. */
static void run_fine(void)
{
    int16_t a;

    for (a = fine_from; a <= fine_to; a = (int16_t)(a + RC_SCAN_FINE_STEP)) {
        state = S_FINE_MOVE;
        if (!step_to(a)) {
            continue;
        }
        state = S_FINE_PING;
        if (!wait_result()) {
            return;
        }
    }
}

/* The background task that drives the whole scan state machine. Created
 * once in sub_scan_init() and runs forever (`for (;;)`, an infinite
 * loop — the standard C idiom for "a task that never exits, just cycles
 * doing its job"). Every wait inside this loop (tk_dly_tsk, tk_wai_flg)
 * is a genuine kernel sleep, so while this task is "busy" scanning, it
 * is actually off the CPU almost the entire time — the motion PID loop
 * and everything else keeps running at full rate underneath it. */
static void scan_task(INT stacd, void *exinf)
{
    UINT ptn;
    ER   er;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (state == S_WATCH) {
            /* Forward watch: one ping straight ahead per period, used
             * while just line-following so the car notices something
             * getting close without running a full scan constantly.
             * sub_nav.c subscribes to RC_EVT_ULTRA_RESULT directly (not
             * through this module) and decides when a watch reading is
             * close enough to escalate into a real sub_scan_start(). */
            cur_angle = 90;
            (void)drv_servo_set_angle(90);
            if (drv_ultrasonic_ping(90) == RC_OK) {
                (void)wait_result();
            }
            (void)tk_dly_tsk(RC_PERIOD_SCAN_MS);
            continue;
        }

        /* Idle: sleep until sub_scan_start() sets FLG_START (or the
         * 100ms timeout expires, in which case just loop around and
         * check state/FLG_START again — this timeout is what lets the
         * S_WATCH branch above get re-checked periodically too). */
        er = tk_wai_flg(scan_flgid, FLG_START, TWF_ORW | TWF_CLR, &ptn, 100);
        if (er != E_OK) {
            continue;
        }

        /* The actual scan: sweep broadly (coarse), then if something was
         * found, zoom in on it (fine), then report the result. */
        run_coarse();
        if (find_fine_window()) {
            run_fine();
        }
        publish_and_finish();

        /* If a continuous watch was requested while this scan was
         * running, drop back into S_WATCH instead of S_IDLE once done. */
        if (watch_on) {
            state = S_WATCH;
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Watch mode result hook: publishes nothing extra, sub_nav subscribes
 *  to RC_EVT_ULTRA_RESULT directly and decides when to escalate.
 * ------------------------------------------------------------------ */

/* Creates the kernel objects this module needs (an event flag and a
 * background task — see the T_CFLG/T_CTSK structs below, which are the
 * RTOS's way of describing "please create one of these for me") and
 * registers on_range() as the ultrasonic driver's result callback. Call
 * once at boot, after drv_servo_init() and drv_ultrasonic_init(). */
rc_result_t sub_scan_init(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;

    /* T_CFLG describes the event flag to create: TA_TFIFO (tasks waiting
     * on it are woken in first-in-first-out order) and TA_WMUL (more
     * than one task is allowed to wait on it at once) are attribute
     * flags for the kernel object. tk_cre_flg() creates it and returns
     * an ID (a small integer handle used in every later tk_*_flg call);
     * a return value <= E_OK signals failure. */
    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;               /* no bits raised at creation */
    scan_flgid   = tk_cre_flg(&cflg);
    if (scan_flgid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    (void)drv_ultrasonic_on_result(on_range, NULL);

    /* T_CTSK describes the background task to create: which function it
     * runs (scan_task), its priority, its stack size, and attribute
     * flags. tk_cre_tsk() creates it (still not running yet — tk_sta_tsk
     * below actually starts it). */
    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_SENSE;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = scan_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    scan_tskid = tk_cre_tsk(&ctsk);
    if (scan_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(scan_tskid, 0);   /* actually starts scan_task running */

    state = S_IDLE;
    return RC_OK;
}

/* Requests a full (coarse + fine) scan. Returns immediately —
 * RC_ERR_BUSY if a scan (or anything other than idle/watch) is already
 * running, otherwise signals scan_task via FLG_START and returns RC_OK
 * right away; the actual scan work happens on scan_task's own time, and
 * the result arrives later through the callback/events described in
 * publish_and_finish() above. See TEAM_GUIDE.md §1.3 for why this
 * "return immediately, callback later" shape is used everywhere in this
 * codebase. */
rc_result_t sub_scan_start(void)
{
    if ((state != S_IDLE) && (state != S_WATCH)) {
        return RC_ERR_BUSY;
    }
    state = S_IDLE;
    return (tk_set_flg(scan_flgid, FLG_START) == E_OK) ? RC_OK : RC_ERR_STATE;
}

/* Requests scan_task stop whatever it's doing (it checks FLG_ABORT
 * inside wait_result()) and immediately forces the visible state back to
 * idle so a caller checking sub_scan_busy() right after this returns
 * sees "not busy" without needing to wait for scan_task to actually
 * notice the flag. */
rc_result_t sub_scan_abort(void)
{
    (void)tk_set_flg(scan_flgid, FLG_ABORT);
    state = S_IDLE;
    return RC_OK;
}

/* True while an actual scan (coarse or fine sweep) is running — the
 * S_WATCH "idle but pinging occasionally" mode does not count as busy. */
bool sub_scan_busy(void)
{
    return (state != S_IDLE) && (state != S_WATCH);
}

/* Stores the pointer to the caller's "scan finished" callback (and its
 * context pointer), called later from publish_and_finish(). */
rc_result_t sub_scan_on_complete(sub_scan_done_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/* Turns the slow continuous forward-ping watch mode on or off. When
 * turning on while currently idle, switches scan_task straight into
 * S_WATCH; when turning off while currently watching, drops back to
 * S_IDLE. If a real scan is already running, this only records the
 * request (via watch_on) — scan_task itself decides to enter S_WATCH
 * once that scan finishes (see the bottom of scan_task above) rather
 * than interrupting it here. */
rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm)
{
    watch_on         = on;
    watch_trigger_mm = trigger_mm;

    if (on && (state == S_IDLE)) {
        state = S_WATCH;
    } else if (!on && (state == S_WATCH)) {
        state = S_IDLE;
    } else {
        /* leave a running scan alone */
    }
    return RC_OK;
}
