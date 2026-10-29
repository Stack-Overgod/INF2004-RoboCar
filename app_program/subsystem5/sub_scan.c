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
#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_scan.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../subsystem2/drv_encoder.h"
#include "../subsystem2/sub_motion.h"

/* Anything further than this is background, not an obstacle. Both
 * build_profile() (deciding what counts as a "hit") and build_plan()
 * (deciding whether to react at all) use this same threshold, so a wall
 * or object further than 400mm away is simply ignored as "open space,"
 * exactly like a car's parking sensor going silent once nothing is close
 * enough to matter. */
#define OBSTACLE_MM         (400U)

/* Minimum gap the car needs to fit through, plus margin. Measure your
 * car's widest point and add 40 mm. Used by build_plan() to size the
 * sideways step of the bypass. (It is no longer compared against the
 * clearance figures: the ±20 deg servo cannot see far enough sideways to
 * measure a whole car width.)
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
static uint16_t     watch_trigger_mm = 200U;   /* = NAV_WATCH_TRIGGER_MM */
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
 *  Spin scan (2026-10-29, Buddy 5)
 *
 *  The servo swings only +-20 deg and the HC-SR04 beam is ~+-15 deg
 *  wide, so a 10-20 cm obstacle at ~20 cm echoes across the whole servo
 *  sweep: its edges can't be seen and the old scan reported the sweep
 *  width (~14 cm at 20 cm) and "no safe gap". For the bypass, sub_nav
 *  asks for a spin scan instead: the servo stays straight ahead and the
 *  CAR spins in place, SPIN_EDGE_DEG left, then right, then back to
 *  straight, pinging every RC_PERIOD_SCAN_MS while it turns slowly. The
 *  heading comes from the encoders (as sub_line's search sweep).
 *
 *  Geometry, seen from above, x to the right, y ahead, heading phi
 *  measured from the line, > 0 = left. The sensor sits on the car's
 *  centreline SPIN_SENSOR_AHEAD_MM ahead of the wheel axle (the spin
 *  centre), so an echo of range r at heading phi lies (a + r) from the
 *  axle in direction phi. Readings within SPIN_DEPTH_BAND_MM of the
 *  face line (face / cos phi from the axle, see face_at) are the
 *  obstacle; the outermost obstacle
 *  echo on each side is the beam's edge passing the obstacle's corner,
 *  so the corner lies SPIN_BEAM_HALF_DDEG back toward the centre from
 *  that heading, at the measured range from the sensor:
 *
 *      corner x = -(a * sin(phi) + r * sin(phi -+ beta))
 *
 *  (- beta on the left side, + beta on the right). Length = right
 *  corner x - left corner x. beta is calibrated on an object of known
 *  length: +1 deg of beta makes a ~20 cm-away object read ~6 mm
 *  shorter.
 * ------------------------------------------------------------------ */
/* Ultrasonic face ahead of the wheel axle, measured 2026-10-29. */
#define SPIN_SENSOR_AHEAD_MM    (85)
/* Car heading swing each side of straight, degrees. Measured from
 * 15 cm (sub_nav approaches first), a 20 cm obstacle's corners are ~23
 * deg off the axle and the beam keeps seeing them ~12 deg further, plus
 * the SPIN_GAP_PINGS clear readings that close an edge: +-50 (was 40,
 * from 20 cm). */
#define SPIN_EDGE_DEG           (50)
/* Wheel speed while pinging: ~26 deg/s, ~1.5 deg between pings. */
#define SPIN_SCAN_MM_S          (25)
/* Wheel speed for the last leg back to straight (no pings). */
#define SPIN_RETURN_MM_S        (70)
/* Effective beam half-angle, 0.1 deg. Calibrated 2026-10-29 17:47 on a
 * 10.4 cm object ~19 cm ahead: at 15 deg (the HC-SR04's nominal) it read
 * 13.5 cm (edges 3.2 cm left / 10.3 cm right of the line); a model of a
 * 20 deg beam reproduces that within 1 mm and, read at 20, gives
 * 10.1 cm. TODO Buddy 5: confirm with a second object length. */
#define SPIN_BEAM_HALF_DDEG     (200)
/* Readings within this of straight ahead (0.1 deg) give the face
 * distance. */
#define SPIN_FACE_DDEG          (100)
/* An echo is the obstacle's FACE if it is no further from the axle than
 * a flat face would be at that heading, face / cos(heading), plus this;
 * further is its sides or background. History (2026-10-29): a fixed 80
 * mm band let echoes from the obstacle's sides (4-6 cm behind the face)
 * stretch both edges (10.4 cm read 15.5-17.3); a fixed 35 mm band cut
 * the face itself off at 25-30 deg from 15 cm, where face / cos is
 * already +23..34 mm (10.4 cm read 5.0-5.7). */
#define SPIN_DEPTH_BAND_MM      (25)
/* This many clear pings in a row (in one pass, walking out from the
 * centre) end the obstacle; a single clear ping inside it is a dropout. */
#define SPIN_GAP_PINGS          (2)
/* Longest obstacle that is bypassed: 20 cm + the 1 cm error + 1 cm.
 * Longer, or an edge not found within the scan: STOP (no safe gap). */
#define SPIN_MAX_BYPASS_MM      (220)
#define SPIN_MAX_POINTS         (128U)
/* The obstacle's depth (front to back) is taken as HALF its measured
 * length (2026-10-29, Buddy 5; was square), so its far side is
 * SPIN_SENSOR_AHEAD_MM + closest + length / 2 ahead of the axle. The bypass runs on until the car's
 * rear (SPIN_REAR_MM behind the axle) is SPIN_PAST_MARGIN_MM past that.
 * Axle to rear bumper measured 2026-10-29. */
#define SPIN_REAR_MM            (55)
#define SPIN_PAST_MARGIN_MM     (30)
/* Each wheel's travel per degree of spin in place, um: (W/2) * pi/180 */
#define SPIN_UM_PER_DEG         ((uint32_t)RC_WHEEL_BASE_MM * 1000UL * 31416UL \
                                 / (2UL * 180UL * 10000UL))

/* sin(0..90 deg) x 10000, 1 deg steps */
static const int16_t sin_tab[91] = {
    0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564, 1736, 1908, 2079,
    2250, 2419, 2588, 2756, 2924, 3090, 3256, 3420, 3584, 3746, 3907,
    4067, 4226, 4384, 4540, 4695, 4848, 5000, 5150, 5299, 5446, 5592,
    5736, 5878, 6018, 6157, 6293, 6428, 6561, 6691, 6820, 6947, 7071,
    7193, 7314, 7431, 7547, 7660, 7771, 7880, 7986, 8090, 8192, 8290,
    8387, 8480, 8572, 8660, 8746, 8829, 8910, 8988, 9063, 9135, 9205,
    9272, 9336, 9397, 9455, 9511, 9563, 9613, 9659, 9703, 9744, 9781,
    9816, 9848, 9877, 9903, 9925, 9945, 9962, 9976, 9986, 9994, 9998,
    10000
};

/* the spin scan's readings: car heading (0.1 deg, > 0 = left) and range */
static int16_t      spin_ddeg[SPIN_MAX_POINTS];
static uint16_t     spin_mm[SPIN_MAX_POINTS];
static bool         spin_ok[SPIN_MAX_POINTS];
/* which leg each reading came from: 0 out left, 1 left to right, 2 back */
static uint8_t      spin_legno[SPIN_MAX_POINTS];
static uint8_t      spin_cur_leg;
static uint32_t     n_spin;
/* sub_scan_start_spin() asked for one; the last scan was one */
static volatile bool spin_req;
static bool         spin_mode;
/* the latest ping result, whatever the scan is doing (on_range) */
static volatile uint16_t last_range_mm;
static volatile bool     last_valid;
/* the running leg: start heading, direction (+1 left), encoder counts */
static int32_t      spin_from;
static int32_t      spin_dir;
static uint32_t     spin_l0;
static uint32_t     spin_r0;
/* edges found by spin_profile(), for spin_plan(): corner x, mm */
static int32_t      spin_xl;
static int32_t      spin_xr;
static bool         spin_has_l;
static bool         spin_has_r;

static void spin_profile(rc_pl_profile_t *p);
/* the last spin scan's per-pass edges, for RC_EVT_SPIN_EDGES */
static rc_pl_spin_t spin_dbg;
/* sub_scan_start_narrow() asked for one; the last scan was one */
static volatile bool narrow_req;
static bool         narrow_mode;
static void narrow_profile(rc_pl_profile_t *p);
static void spin_plan(const rc_pl_profile_t *p, rc_pl_plan_t *plan);

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
       /* smallest angle that counted as "close" (-1 = none yet). Smallest /
     * largest rather than first / last in array order, because the fine
     * sweep's points are appended after the coarse ones, out of order */
    int16_t  first_hit = -1;
    /* largest angle that counted as "close" */
    int16_t  last_hit = -1;
    /* how far (degrees from straight ahead) the outermost obstacle-free
     * bearing on the left / right of centre is; 0 = none free */
    int32_t  clear_l_deg = 0;
    int32_t  clear_r_deg = 0;

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
             * background). Track the smallest and largest angle that
             * qualified — together they mark the angular span the
             * obstacle occupies, used below for the width estimate. */
            if ((first_hit < 0) || (points[i].angle < first_hit)) {
                first_hit = points[i].angle;
            }
            if (points[i].angle > last_hit) {
                last_hit = points[i].angle;
            }
        } else {
            /* Free bearings on each side of straight ahead become the
             * clearance figures: remember the outermost free bearing on
             * each side. Crude, but it is what a single ranging beam on
             * a ±20 deg servo can honestly support. */
            if (points[i].angle < 90) {
                if ((90 - (int32_t)points[i].angle) > clear_r_deg) {
                    clear_r_deg = 90 - (int32_t)points[i].angle;
                }
            } else if (points[i].angle > 90) {
                if (((int32_t)points[i].angle - 90) > clear_l_deg) {
                    clear_l_deg = (int32_t)points[i].angle - 90;
                }
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

    /* Clearance isn't measured as an actual gap width — it's how far
     * sideways, at the obstacle's distance, the outermost "clear" bearing
     * on each side reaches: range * angle (small-angle approximation and
     * the same degrees -> milliradians step as the width above). With
     * only ±20 deg of servo travel this is at most ~0.35 x closest_mm, so
     * it is a "this side is open" indicator, not proof that a whole car
     * width fits (see build_plan). */
    p->clearance_left_mm  =
        (uint16_t)(((int32_t)closest * ((clear_l_deg * 17453) / 1000)) / 1000);
    p->clearance_right_mm =
        (uint16_t)(((int32_t)closest * ((clear_r_deg * 17453) / 1000)) / 1000);

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

    if ((p->clearance_left_mm == 0U) && (p->clearance_right_mm == 0U)) {
        /* No clear bearing on either side: the obstacle fills the whole
         * ±20 deg the servo can see, so there is no side known to be
         * open — stop rather than guess. (The servo cannot see far enough
         * sideways to check a full CAR_WIDTH_MM gap, so this only asks
         * whether a side is open at all.) See the TODO above about adding
         * a "reverse and reattempt" option here instead of just
         * stopping. */
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

    last_range_mm = range_mm;
    last_valid    = valid;
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

    if (narrow_mode) {
        narrow_profile(&profile);
        spin_plan(&profile, &plan);
    } else if (spin_mode) {
        spin_profile(&profile);
        spin_plan(&profile, &plan);
    } else {
        build_profile(&profile);
        build_plan(&profile, &plan);
    }

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

    if (spin_mode || narrow_mode) {
        evt.id     = RC_EVT_SPIN_EDGES;     /* car/01/spin, diagnostics */
        evt.u.spin = spin_dbg;
        (void)rc_event_publish(&evt);
    }

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

/* ------------------------------------------------------------------ *
 *  Spin scan: geometry, legs, profile, plan
 * ------------------------------------------------------------------ */

/* sin(ddeg / 10 deg) x 10000, any sign, |ddeg| <= 900 */
static int32_t isin(int32_t ddeg)
{
    int32_t a = (ddeg < 0) ? -ddeg : ddeg;
    int32_t d, f, v;

    if (a > 900) {
        a = 900;
    }
    d = a / 10;
    f = a % 10;
    v = sin_tab[d];
    if (d < 90) {
        v += ((sin_tab[d + 1] - sin_tab[d]) * f) / 10;
    }
    return (ddeg < 0) ? -v : v;
}

/* Car heading now, 0.1 deg (> 0 = left), from the encoders: the mean
 * wheel travel since the leg began over SPIN_UM_PER_DEG. */
static int32_t spin_heading(void)
{
    uint32_t dl = drv_encoder_count(RC_SIDE_LEFT) - spin_l0;
    uint32_t dr = drv_encoder_count(RC_SIDE_RIGHT) - spin_r0;
    uint32_t um = ((dl + dr) / 2U) * RC_ENC_UM_PER_TICK;

    return spin_from + (spin_dir * (int32_t)((um * 10U) / SPIN_UM_PER_DEG));
}

/* An abort was requested (sub_scan_abort), checked without waiting long. */
static bool spin_aborted(void)
{
    UINT ptn;

    return tk_wai_flg(scan_flgid, FLG_ABORT, TWF_ORW | TWF_CLR, &ptn, 10)
           == E_OK;
}

/* One leg: spin in place from `from` to heading `to` (0.1 deg), pinging
 * straight ahead (servo) on the way if `ping`. Returns the heading
 * reached, or INT32_MIN when aborted or wedged (motors stopped). */
static int32_t spin_leg(int32_t from, int32_t to, bool ping)
{
    int32_t  speed = ping ? SPIN_SCAN_MM_S : SPIN_RETURN_MM_S;
    uint32_t i, limit = ping ? 200U : 600U;     /* ~18 s / ~6 s */
    int32_t  h, h0;

    spin_from = from;
    spin_dir  = (to > from) ? 1 : -1;
    spin_l0   = drv_encoder_count(RC_SIDE_LEFT);
    spin_r0   = drv_encoder_count(RC_SIDE_RIGHT);
    /* sub_motion steer > 0 turns right; spin_dir > 0 is left */
    (void)sub_motion_drive_speed(0, (int16_t)(-spin_dir * speed));

    for (i = 0U; i < limit; i++) {
        h = spin_heading();
        if ((spin_dir * (h - to)) >= 0) {
            return h;
        }
        if (!ping) {
            if (spin_aborted()) {
                break;
            }
            continue;
        }
        h0 = h;
        if (drv_ultrasonic_ping(90) == RC_OK) {
            if (!wait_result()) {
                break;                          /* aborted */
            }
            if (n_spin < SPIN_MAX_POINTS) {
                spin_ddeg[n_spin] = (int16_t)((h0 + spin_heading()) / 2);
                spin_mm[n_spin]   = last_range_mm;
                spin_ok[n_spin]   = last_valid;
                spin_legno[n_spin] = spin_cur_leg;
                n_spin++;
            }
        }
        (void)tk_dly_tsk(RC_PERIOD_SCAN_MS);    /* no ghost echoes */
    }
    (void)sub_motion_stop(true);
    return INT32_MIN;
}

/* The whole spin: left to +SPIN_EDGE_DEG, right to -SPIN_EDGE_DEG, back
 * to straight, pinging on all three legs, so each side is crossed twice
 * (once outward, once inward) and the two crossings can be averaged.
 * Leaves the motors stopped. */
static void run_spin(void)
{
    int32_t  h;
    uint32_t settle = drv_servo_settle_ms(drv_servo_get_angle(), 90);

    n_spin    = 0U;
    cur_angle = 90;
    (void)drv_servo_set_angle(90);
    (void)tk_dly_tsk((RELTIM)settle);
    state = S_COARSE_PING;

    spin_cur_leg = 0U;
    h = spin_leg(0, SPIN_EDGE_DEG * 10, true);
    if (h != INT32_MIN) {
        spin_cur_leg = 1U;
        h = spin_leg(h, -SPIN_EDGE_DEG * 10, true);
    }
    if (h != INT32_MIN) {
        spin_cur_leg = 2U;
        h = spin_leg(h, 0, true);
    }
    (void)sub_motion_stop(true);
}

/* Corner x (mm, > 0 = right of the line) from where the beam leaves the
 * obstacle on one side: heading ddeg, range r from the sensor. side +1 =
 * left, -1 = right; the corner is the beam half-angle back toward the
 * centre. */
static int32_t corner_x(int32_t ddeg, int32_t r, int32_t side)
{
    int32_t psi = ddeg - (side * SPIN_BEAM_HALF_DDEG);

    return -((SPIN_SENSOR_AHEAD_MM * isin(ddeg)) + (r * isin(psi))) / 10000;
}

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Range from the axle at which a flat face `face` mm straight ahead lies
 * along heading ddeg: face / cos(ddeg). */
static int32_t face_at(int32_t face, int32_t ddeg)
{
    int32_t c = isin(900 - iabs(ddeg));     /* cos x 10000 */

    return (c > 0) ? ((face * 10000) / c) : 0x7FFFFFFF;
}

/* One pass over one side: the readings of leg `leg` on side `side` (+1
 * left), walked from the centre outward. The beam left the obstacle
 * between the last obstacle reading and the first of SPIN_GAP_PINGS
 * clear readings in a row after it: the edge heading is their midpoint
 * (pings are ~2-4 deg apart, so taking either one alone is off by half
 * a step), the range the last obstacle reading's. Returns false if the
 * pass never left the obstacle. */
static bool pass_edge(uint8_t leg, int32_t side, int32_t face,
                      int32_t *ddeg_out, int32_t *r_out)
{
    static uint8_t idx[SPIN_MAX_POINTS];
    uint32_t n = 0U, i, j, run = 0U;
    int32_t  last = -1, first_clear = -1;

    for (i = 0U; i < n_spin; i++) {
        if ((spin_legno[i] == leg) && ((side * spin_ddeg[i]) > 0)) {
            idx[n] = (uint8_t)i;
            n++;
        }
    }
    /* by distance from the centre (inward passes arrive reversed) */
    for (i = 1U; i < n; i++) {
        uint8_t t = idx[i];

        j = i;
        while ((j > 0U) && (iabs(spin_ddeg[idx[j - 1U]]) > iabs(spin_ddeg[t]))) {
            idx[j] = idx[j - 1U];
            j--;
        }
        idx[j] = t;
    }
    for (i = 0U; i < n; i++) {
        uint32_t k = idx[i];
        bool obst = spin_ok[k]
                    && ((SPIN_SENSOR_AHEAD_MM + (int32_t)spin_mm[k])
                        <= (face_at(face, spin_ddeg[k]) + SPIN_DEPTH_BAND_MM));

        if (obst) {
            last = (int32_t)k;
            run  = 0U;
        } else {
            if (run == 0U) {
                first_clear = (int32_t)k;
            }
            run++;
            if ((run >= SPIN_GAP_PINGS) && (last >= 0)) {
                *ddeg_out = (spin_ddeg[last] + spin_ddeg[first_clear]) / 2;
                *r_out    = (int32_t)spin_mm[last];
                return true;
            }
        }
    }
    return false;
}

/* Profile from the spin readings. closest = the face distance from the
 * sensor, straight ahead; width = obstacle length between the corners;
 * clearance_left/right = how far each corner is from the line (mm, at
 * least 1), or 0 when that edge was not found within the scan. */
static void spin_profile(rc_pl_profile_t *p)
{
    uint32_t i;
    int32_t  face = 0x7FFFFFFF, face_ddeg = 0, side;

    p->n_points           = (uint8_t)((n_spin > 255U) ? 255U : n_spin);
    spin_dbg.face_mm  = 0U;
    spin_dbg.found    = 0U;
    spin_dbg.n_points = p->n_points;
    p->closest_angle_deg  = 90;
    p->closest_mm         = 0U;
    p->width_mm           = 0U;
    p->clearance_left_mm  = 0U;
    p->clearance_right_mm = 0U;
    spin_has_l = false;
    spin_has_r = false;

    for (i = 0U; i < n_spin; i++) {
        int32_t r = SPIN_SENSOR_AHEAD_MM + (int32_t)spin_mm[i];

        if (spin_ok[i] && (spin_ddeg[i] >= -SPIN_FACE_DDEG)
            && (spin_ddeg[i] <= SPIN_FACE_DDEG) && (r < face)) {
            face      = r;
            face_ddeg = spin_ddeg[i];
        }
    }
    if ((face == 0x7FFFFFFF)
        || ((face - SPIN_SENSOR_AHEAD_MM) > (int32_t)OBSTACLE_MM)) {
        return;                         /* nothing ahead: closest 0 */
    }
    p->closest_mm        = (uint16_t)(face - SPIN_SENSOR_AHEAD_MM);
    p->closest_angle_deg = (int16_t)(90 + (face_ddeg / 10));
    spin_dbg.face_mm     = (uint16_t)face;

    /* raw readings on the console, heading (0.1 deg, > 0 left):range mm
     * (x = no echo), 8 a line: the data to calibrate and debug from */
    tm_printf((UB *)"[spin] %d points, face %d mm from the axle\n",
              (INT)n_spin, (INT)face);
    for (i = 0U; i < n_spin; i++) {
        if (spin_ok[i]) {
            tm_printf((UB *)" %d:%d", (INT)spin_ddeg[i], (INT)spin_mm[i]);
        } else {
            tm_printf((UB *)" %d:x", (INT)spin_ddeg[i]);
        }
        if (((i % 8U) == 7U) || ((i + 1U) == n_spin)) {
            tm_printf((UB *)"\n");
        }
    }

    /* each side is crossed by two legs (left: 0 and 1, right: 1 and 2);
     * the edge is the mean of the passes that found it, which cancels the
     * lag that makes an outward pass see the edge late and an inward one
     * early */
    for (side = 1; side >= -1; side -= 2) {
        uint8_t leg0 = (side > 0) ? 0U : 1U;
        int32_t sum_h = 0, sum_r = 0, n = 0, h, r;
        uint8_t leg;

        for (leg = leg0; leg <= (uint8_t)(leg0 + 1U); leg++) {
            if (pass_edge(leg, side, face, &h, &r)) {
                /* slot: left legs 0,1 -> 0,1; right legs 1,2 -> 2,3 */
                uint32_t slot = (side > 0) ? leg : (uint32_t)(leg + 1U);

                spin_dbg.ddeg[slot]  = (int16_t)h;
                spin_dbg.r_2mm[slot] = (uint8_t)((r > 510) ? 255 : (r / 2));
                spin_dbg.found      |= (uint8_t)(1U << slot);
                tm_printf((UB *)"[spin] %s edge, leg %d: %d (0.1 deg) at %d mm\n",
                          (side > 0) ? "left" : "right", (INT)leg, (INT)h, (INT)r);
                sum_h += h;
                sum_r += r;
                n++;
            }
        }
        if (n > 0) {
            int32_t x = corner_x(sum_h / n, sum_r / n, side);

            if (side > 0) {
                spin_xl    = x;
                spin_has_l = true;
                p->clearance_left_mm  = (uint16_t)((x < -1) ? -x : 1);
            } else {
                spin_xr    = x;
                spin_has_r = true;
                p->clearance_right_mm = (uint16_t)((x > 1) ? x : 1);
            }
        }
    }
    if (spin_has_l && spin_has_r && (spin_xr > spin_xl)) {
        p->width_mm = (uint16_t)(spin_xr - spin_xl);
    }
}

/* Plan from the spin profile: go round on the side AWAY from the
 * obstacle (2026-10-29, Buddy 5): with both edges seen, its centre
 * (xl + xr) / 2 right of the line (or on it) -> left, left of it ->
 * right; with one edge unseen, the obstacle reaches past the scan on
 * that side -> the other side. Obstacles longer than SPIN_MAX_BYPASS_MM,
 * or no edge seen: stop. */
static void spin_plan(const rc_pl_profile_t *p, rc_pl_plan_t *plan)
{
    uint32_t len;
    bool     go_left;

    plan->action     = RC_CMD_NONE;
    plan->lateral_mm = 0U;
    plan->forward_mm = 0U;

    if ((p->closest_mm == 0U) || (p->closest_mm > OBSTACLE_MM)) {
        plan->action = RC_CMD_GO_STRAIGHT;
        return;
    }
    if (((p->clearance_left_mm == 0U) && (p->clearance_right_mm == 0U))
        || (p->width_mm > SPIN_MAX_BYPASS_MM)) {
        plan->action = RC_CMD_STOP;
        return;
    }
    if (spin_has_l && spin_has_r) {
        go_left = ((spin_xl + spin_xr) >= 0);
    } else {
        go_left = spin_has_l;
    }
    tm_printf((UB *)"[spin] edges L %d%s R %d%s mm, centre %d -> go %s\n",
              (INT)spin_xl, spin_has_l ? "" : "(none)",
              (INT)spin_xr, spin_has_r ? "" : "(none)",
              (INT)((spin_xl + spin_xr) / 2), go_left ? "left" : "right");
    if (go_left) {
        plan->action     = RC_CMD_TURN_LEFT;
        plan->lateral_mm = (uint16_t)(p->clearance_left_mm + (CAR_WIDTH_MM / 2U));
    } else {
        plan->action     = RC_CMD_TURN_RIGHT;
        plan->lateral_mm = (uint16_t)(p->clearance_right_mm + (CAR_WIDTH_MM / 2U));
    }
    /* depth = length / 2 (the longest allowed length when one edge was
     * not seen). forward_mm = how far ahead of the axle's position
     * now the axle must get before turning back in (sub_nav subtracts
     * what leg 1 already covers). */
    len = (p->width_mm != 0U) ? p->width_mm : SPIN_MAX_BYPASS_MM;
    plan->forward_mm = (uint16_t)(SPIN_SENSOR_AHEAD_MM + p->closest_mm
                                  + (len / 2U) + SPIN_REAR_MM
                                  + SPIN_PAST_MARGIN_MM);
}

/* ------------------------------------------------------------------ *
 *  Narrow servo scan (2026-10-29, Buddy 5): the chassis stays still and
 *  the servo sweeps NARROW_HALF_DEG either side of straight ahead. With
 *  the HC-SR04's wide beam a 10-20 cm obstacle at ~20 cm usually echoes
 *  across the whole sweep, so an edge is often not seen; a missing edge
 *  is then placed from NARROW_ASSUME_MM (the longest obstacle on the
 *  course) and the length is reported as unknown (0).
 * ------------------------------------------------------------------ */
#define NARROW_HALF_DEG         (15)
#define NARROW_STEP_DEG         (3)
/* the longest obstacle on the course: used for a side not seen */
#define NARROW_ASSUME_MM        (200)

/* Servo 90 - NARROW_HALF_DEG .. 90 + NARROW_HALF_DEG, one ping a step,
 * left to right as sub_scan's angles run (> 90 = left), then back to 90. */
static void run_narrow(void)
{
    int16_t a;

    n_points = 0U;
    for (a = (int16_t)(90 - NARROW_HALF_DEG); a <= (int16_t)(90 + NARROW_HALF_DEG);
         a = (int16_t)(a + NARROW_STEP_DEG)) {
        state = S_COARSE_MOVE;
        if (!step_to(a)) {
            continue;
        }
        state = S_COARSE_PING;
        if (!wait_result()) {
            break;                              /* aborted */
        }
    }
    cur_angle = 90;
    (void)drv_servo_set_angle(90);
}

/* One side of the narrow sweep, walked out from straight ahead: the edge
 * is the midpoint between the last face reading and the first of
 * SPIN_GAP_PINGS clear ones (as pass_edge, but servo angles and the
 * sensor as the pivot). Returns false if the beam never left the face. */
static bool narrow_edge(int32_t side, int32_t face, int32_t *ddeg_out,
                        int32_t *r_out)
{
    int32_t  b, last = -1, first_clear = -1;
    uint32_t i, run = 0U;

    for (b = 0; b <= NARROW_HALF_DEG; b++) {
        int32_t ang = 90 + (side * b);

        for (i = 0U; i < n_points; i++) {
            bool obst;

            if (points[i].angle != ang) {
                continue;
            }
            obst = points[i].valid
                   && ((int32_t)points[i].range_mm
                       <= (face_at(face, b * 10) + SPIN_DEPTH_BAND_MM));
            if (obst) {
                last = (int32_t)i;
                run  = 0U;
            } else {
                if (run == 0U) {
                    first_clear = (int32_t)i;
                }
                run++;
                if ((run >= SPIN_GAP_PINGS) && (last >= 0)) {
                    *ddeg_out = (((int32_t)points[last].angle - 90)
                                 + ((int32_t)points[first_clear].angle - 90)) * 5;
                    *r_out    = (int32_t)points[last].range_mm;
                    return true;
                }
            }
        }
    }
    return false;
}

/* Profile from the narrow sweep, in the same form as spin_profile (so
 * spin_plan and car/01/spin work unchanged): closest = face straight
 * ahead, width = length if both edges were seen (else 0), clearances =
 * each edge's distance from the line, a side not seen being placed
 * NARROW_ASSUME_MM from the other edge (or centred on the closest echo
 * when neither was seen). */
static void narrow_profile(rc_pl_profile_t *p)
{
    uint32_t i;
    int32_t  face = 0x7FFFFFFF, face_ang = 90, side, h, r, xc;
    int32_t  x[2] = { 0, 0 };
    bool     seen[2] = { false, false };

    p->n_points           = (uint8_t)n_points;
    p->closest_angle_deg  = 90;
    p->closest_mm         = 0U;
    p->width_mm           = 0U;
    p->clearance_left_mm  = 0U;
    p->clearance_right_mm = 0U;
    spin_dbg.face_mm  = 0U;
    spin_dbg.found    = 0U;
    spin_dbg.n_points = p->n_points;
    spin_has_l = false;
    spin_has_r = false;

    for (i = 0U; i < n_points; i++) {
        if (points[i].valid && ((int32_t)points[i].range_mm < face)
            && (iabs((int32_t)points[i].angle - 90) <= 5)) {
            face     = (int32_t)points[i].range_mm;
            face_ang = points[i].angle;
        }
    }
    if ((face == 0x7FFFFFFF) || (face > (int32_t)OBSTACLE_MM)) {
        return;
    }
    p->closest_mm        = (uint16_t)face;
    p->closest_angle_deg = (int16_t)face_ang;
    spin_dbg.face_mm     = (uint16_t)face;

    for (side = 1; side >= -1; side -= 2) {
        uint32_t k = (side > 0) ? 0U : 1U;

        if (narrow_edge(side, face, &h, &r)) {
            uint32_t slot = (side > 0) ? 0U : 2U;

            /* the servo turns the sensor about itself: no arm */
            x[k]    = -(r * isin(h - (side * SPIN_BEAM_HALF_DDEG))) / 10000;
            seen[k] = true;
            spin_dbg.ddeg[slot]  = (int16_t)h;
            spin_dbg.r_2mm[slot] = (uint8_t)((r > 510) ? 255 : (r / 2));
            spin_dbg.found      |= (uint8_t)(1U << slot);
            tm_printf((UB *)"[narrow] %s edge %d (0.1 deg) at %d mm, x %d\n",
                      (side > 0) ? "left" : "right", (INT)h, (INT)r,
                      (INT)x[k]);
        }
    }
    if (seen[0] && seen[1]) {
        if (x[1] > x[0]) {
            p->width_mm = (uint16_t)(x[1] - x[0]);
        }
    } else if (seen[0]) {
        x[1] = x[0] + NARROW_ASSUME_MM;
    } else if (seen[1]) {
        x[0] = x[1] - NARROW_ASSUME_MM;
    } else {
        xc   = -(face * isin(((int32_t)face_ang - 90) * 10)) / 10000;
        x[0] = xc - (NARROW_ASSUME_MM / 2);
        x[1] = xc + (NARROW_ASSUME_MM / 2);
    }
    spin_xl    = x[0];
    spin_xr    = x[1];
    spin_has_l = true;          /* placed, seen or not: spin_plan uses */
    spin_has_r = true;          /* the centre to pick the side         */
    p->clearance_left_mm  = (uint16_t)((x[0] < -1) ? -x[0] : 1);
    p->clearance_right_mm = (uint16_t)((x[1] > 1) ? x[1] : 1);
    tm_printf((UB *)"[narrow] face %d mm, edges L %d%s R %d%s, length %d\n",
              (INT)face, (INT)x[0], seen[0] ? "" : "(assumed)",
              (INT)x[1], seen[1] ? "" : "(assumed)", (INT)p->width_mm);
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
        spin_mode   = spin_req;
        narrow_mode = narrow_req;
        spin_req    = false;
        narrow_req  = false;
        if (narrow_mode) {
            run_narrow();
        } else if (spin_mode) {
            run_spin();
        } else {
            run_coarse();
            if (find_fine_window()) {
                run_fine();
            }
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
    state      = S_IDLE;
    spin_req   = false;
    narrow_req = false;
    return (tk_set_flg(scan_flgid, FLG_START) == E_OK) ? RC_OK : RC_ERR_STATE;
}

/* As sub_scan_start(), but the narrow servo sweep (+-NARROW_HALF_DEG,
 * chassis still): see "Narrow servo scan". */
rc_result_t sub_scan_start_narrow(void)
{
    if ((state != S_IDLE) && (state != S_WATCH)) {
        return RC_ERR_BUSY;
    }
    state      = S_IDLE;
    spin_req   = false;
    narrow_req = true;
    return (tk_set_flg(scan_flgid, FLG_START) == E_OK) ? RC_OK : RC_ERR_STATE;
}

/* As sub_scan_start(), but the spin scan (see "Spin scan" above): the
 * car spins in place to measure the obstacle's length. The motors must
 * be free (sub_nav AVOIDING); they are stopped when it ends. */
rc_result_t sub_scan_start_spin(void)
{
    if ((state != S_IDLE) && (state != S_WATCH)) {
        return RC_ERR_BUSY;
    }
    state    = S_IDLE;
    spin_req = true;
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
