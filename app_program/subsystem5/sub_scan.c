/*
 *  sub_scan.c
 *
 *  Scan task: drives the servo + ultrasonic pair (and, for the step scan,
 *  the chassis) through a scan, collects the (angle, distance) readings,
 *  and turns them into an obstacle profile plus an avoidance plan. Also
 *  runs the forward watch ping while line following. See
 *  docs/buddy5-scan-avoidance/buddy5-scan-avoidance.md for the walkthrough.
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

/* Anything further than this is background, not an obstacle. The
 * profiles (what counts as a "hit") and the plans (whether to react at
 * all) use this same threshold. */
#define OBSTACLE_MM         (400U)

/* Car width plus margin (widest point + 40 mm). Used by the plans to size
 * the sideways step of the bypass. It is not compared against the
 * clearance figures: the +-20 deg servo cannot see far enough sideways to
 * measure a whole car width. */
#define CAR_WIDTH_MM        (150U)

/* The scan state machine's states: what scan_task is currently doing. */
typedef enum {
    /* nothing happening; ready to start a new scan on request */
    S_IDLE = 0,
    /* forward watch: a ping straight ahead, RC_PERIOD_SCAN_MS apart */
    S_WATCH,
    /* step scan running (servo and chassis measure the obstacle) */
    S_STEP
} scan_state_t;

/* current state-machine state, see scan_state_t above */
static scan_state_t state;
/* whether continuous forward-watch mode is currently requested */
static bool         watch_on;
/* kernel task ID for the background scan_task */
static ID           scan_tskid;
/* kernel "event flag" object ID used to wake scan_task when something happens
 * (see FLG_* below) */
static ID           scan_flgid;
/* semaphore counting scan requests: sub_scan_start_measure() signals it
 * and scan_task takes it. A count cannot be lost to a flag clear. */
static ID           scan_semid;

/* scan_flgid bits: scan_task sleeps on them (tk_wai_flg) until one is
 * set (tk_set_flg). */
/* an ultrasonic ping just produced a result */
#define FLG_RESULT      (1U << 0)
/* sub_scan_abort() was called; stop what's running */
#define FLG_ABORT       (1U << 2)

/* ------------------------------------------------------------------ *
 *  Obstacle geometry (used by the step scan)
 *
 *  Seen from above, x to the right, y ahead, headings measured from the
 *  line, > 0 = left. The sensor sits on the car's centreline
 *  SPIN_SENSOR_AHEAD_MM ahead of the wheel axle (the spin centre). The
 *  chassis heading comes from the encoders (spin_heading).
 * ------------------------------------------------------------------ */
/* Ultrasonic face ahead of the wheel axle, mm. */
#define SPIN_SENSOR_AHEAD_MM    (85)
/* Effective beam half-angle, 0.1 deg: how far past the corner the beam
 * centre points when the face echo is lost. Fitted to step scans of a
 * 20.5 cm object ~22 cm ahead (9 deg reads it 19-24 cm); 8 deg leans
 * slightly long, so the bypass errs wide. Each degree less makes a
 * ~20 cm-away object read ~8 mm longer. */
#define SPIN_BEAM_HALF_DDEG     (80)
/* An echo is the obstacle's FACE if it is no further than the face's
 * plane along the beam (step_on_face) plus this; further is its sides or
 * background. A wide band lets echoes from the obstacle's sides (4-6 cm
 * behind the face) stretch both edges. */
#define SPIN_DEPTH_BAND_MM      (25)
/* The longest obstacle planned for, mm: the brief's 10-20 cm (square),
 * plus room for the 20.5 cm course obstacle and the scan's overestimate.
 * The step scan looks this far out from the line for an edge; a longer
 * measured length is planned as this; with one edge unseen the obstacle
 * is taken to be this long. */
#define OBSTACLE_MAX_MM         (250)
/* The obstacle is square: its depth (front to
 * back) is its measured length, so its far side is SPIN_SENSOR_AHEAD_MM
 * + closest + length ahead of the axle. The bypass runs on until the
 * car's rear (SPIN_REAR_MM, axle to rear bumper) is SPIN_PAST_MARGIN_MM
 * past that. */
#define SPIN_REAR_MM            (55)
#define SPIN_PAST_MARGIN_MM     (30)
/* How much of the encoders' spin the chassis really turns, permille:
 * the wheels slip on the foam. 45 deg detour turns lose more (sub_nav
 * NAV_DETOUR_TURN_PERMILLE). Calibrate on a known object: if it reads
 * long, lower this. */
#define SPIN_TURN_EFF_PERMILLE  (800)
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

/* the latest ping result, whatever the scan is doing (on_range) */
static volatile uint16_t last_range_mm;
static volatile bool     last_valid;
/* the running chassis turn (chassis_to): start heading, direction
 * (+1 left), encoder counts at the start */
static int32_t      spin_from;
static int32_t      spin_dir;
static uint32_t     spin_l0;
static uint32_t     spin_r0;
/* edges found by step_profile(), for spin_plan(): corner x, mm */
static int32_t      spin_xl;
static int32_t      spin_xr;
static bool         spin_has_l;
static bool         spin_has_r;

/* the last step scan's edges, for RC_EVT_SPIN_EDGES */
static rc_pl_spin_t spin_dbg;
static void step_profile(rc_pl_profile_t *p);
static void spin_plan(const rc_pl_profile_t *p, rc_pl_plan_t *plan);

/* ------------------------------------------------------------------ *
 *  Result callback from the ultrasonic driver
 * ------------------------------------------------------------------ */

/* Registered with drv_ultrasonic_on_result() in sub_scan_init(); called
 * from the defer task (see drv_ultra_cb_t). Keeps the latest reading and
 * wakes scan_task with FLG_RESULT. */
static void on_range(uint16_t range_mm, bool valid, void *ctx)
{
    (void)ctx;

    last_range_mm = range_mm;
    last_valid    = valid;
    (void)tk_set_flg(scan_flgid, FLG_RESULT);
}

/* ------------------------------------------------------------------ *
 *  Scan task
 *
 *  Every wait in here is a kernel wait with a timeout, so the task is
 *  off the run queue for the whole servo travel and the whole 30 ms of
 *  ultrasonic flight time.
 * ------------------------------------------------------------------ */

/* Runs once a step scan has finished. Builds the profile and plan,
 * publishes both and the edge diagnostics on the event bus, and resets
 * the state machine to idle. This is the single place a completed scan
 * gets reported to the rest of the car. */
static void publish_and_finish(void)
{
    rc_pl_profile_t profile;
    rc_pl_plan_t    plan;
    rc_event_t      evt;

    step_profile(&profile);
    spin_plan(&profile, &plan);

    /* sub_nav acts on RC_EVT_AVOIDANCE_PLAN in AVOIDING; sub_telemetry
     * reports RC_EVT_OBSTACLE_PROFILE. */
    evt.id         = RC_EVT_OBSTACLE_PROFILE;
    evt.u.profile  = profile;
    (void)rc_event_publish(&evt);

    evt.id    = RC_EVT_AVOIDANCE_PLAN;
    evt.u.plan = plan;
    (void)rc_event_publish(&evt);

    evt.id     = RC_EVT_SPIN_EDGES;     /* car/01/spin, diagnostics */
    evt.u.spin = spin_dbg;
    (void)rc_event_publish(&evt);

    state = S_IDLE;
}

/* Moves the servo to `angle`, sleeps (tk_dly_tsk) until it has settled,
 * then starts one ultrasonic ping at that angle. Returns true if the ping
 * was started. */
static bool step_to(int16_t angle)
{
    uint32_t settle = drv_servo_settle_ms(drv_servo_get_angle(), angle);

    (void)drv_servo_set_angle(angle);
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
     * TWF_BITCLR = clear only the listed bits once they wake this task,
     * so the next wait starts fresh; TWF_CLR would clear every bit. */
    er = tk_wai_flg(scan_flgid, FLG_RESULT | FLG_ABORT,
                    TWF_ORW | TWF_BITCLR, &ptn, 100);

    return (er == E_OK) && ((ptn & FLG_ABORT) == 0U);
}

/* ------------------------------------------------------------------ *
 *  Geometry helpers, chassis heading, plan
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
 * wheel travel since the turn began over SPIN_UM_PER_DEG, less the slip
 * (SPIN_TURN_EFF_PERMILLE). */
static int32_t spin_heading(void)
{
    uint32_t dl = drv_encoder_count(RC_SIDE_LEFT) - spin_l0;
    uint32_t dr = drv_encoder_count(RC_SIDE_RIGHT) - spin_r0;
    uint32_t um = ((dl + dr) / 2U) * RC_ENC_UM_PER_TICK;

    return spin_from + (spin_dir * (int32_t)(((um * 10U) / SPIN_UM_PER_DEG)
                                             * SPIN_TURN_EFF_PERMILLE / 1000U));
}

/* An abort was requested (sub_scan_abort), checked without waiting long. */
static bool spin_aborted(void)
{
    UINT ptn;

    return tk_wai_flg(scan_flgid, FLG_ABORT, TWF_ORW | TWF_BITCLR, &ptn, 10)
           == E_OK;
}

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Plan from the step profile: go round on the side AWAY from the
 * obstacle: with both edges seen, its centre
 * (xl + xr) / 2 right of the line (or on it) -> left, left of it ->
 * right; with one edge unseen, the obstacle reaches past the scan on
 * that side -> the other side. No edge seen on either side: stop. */
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
    if ((p->clearance_left_mm == 0U) && (p->clearance_right_mm == 0U)) {
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
        plan->lateral_mm = RC_SAT_U(p->clearance_left_mm + (CAR_WIDTH_MM / 2U), RC_BITS_MM);
    } else {
        plan->action     = RC_CMD_TURN_RIGHT;
        plan->lateral_mm = RC_SAT_U(p->clearance_right_mm + (CAR_WIDTH_MM / 2U), RC_BITS_MM);
    }
    /* square: depth = length, the measured one up to OBSTACLE_MAX_MM (the
     * longest allowed when one edge was not seen). forward_mm = how far
     * ahead of the axle's position now the axle must get before turning
     * back in (sub_nav subtracts what leg 1 already covers). */
    len = p->width_mm;
    if ((len == 0U) || (len > (uint32_t)OBSTACLE_MAX_MM)) {
        len = (uint32_t)OBSTACLE_MAX_MM;
    }
    plan->forward_mm = RC_SAT_U(SPIN_SENSOR_AHEAD_MM + p->closest_mm
                                + len + SPIN_REAR_MM
                                + SPIN_PAST_MARGIN_MM, RC_BITS_MM);
}

/* ------------------------------------------------------------------ *
 *  Step scan: the obstacle's length from the servo
 *  AND the chassis, stopped for every ping.
 *
 *  The servo swings only RC_SERVO_HALF_SWING (20 deg) each side and the
 *  beam is ~SPIN_BEAM_HALF_DDEG wide, so from the 20 cm trigger the beam
 *  never gets off a 10-20 cm face with the chassis still. For each side
 *  in turn:
 *
 *    1. servo fully to that side, chassis straight: ping;
 *    2. while that echoes off the face, turn the chassis
 *       STEP_CHASSIS_DDEG further that way, stop, ping (no
 *       turning-while-pinging lag);
 *    3. at the first heading where it does not, walk the servo back
 *       STEP_FINE_DEG at a time until the face echoes again: the edge
 *       is between the two, to within STEP_FINE_DEG / 2;
 *    4. chassis back to straight.
 *
 *  A side whose beam is still on the face once it points past where the
 *  corner of the longest allowed obstacle could be (OBSTACLE_MAX_MM out
 *  from the line) has no edge: the obstacle runs on past the scan. How
 *  far the chassis turns therefore follows from the obstacle, ~10-15 deg
 *  for a 10 cm one, not from a fixed swing.
 *
 *  Geometry (from the axle at the start, x > 0 right of the line, y
 *  forward, headings > 0 left): chassis heading phi puts the sensor at
 *  (-A sin phi, A cos phi) and the beam axis along psi = phi + servo.
 *  The beam's inner edge is on the corner at the edge heading, so the
 *  corner lies at range r along psi - side * beta:
 *
 *      x = -(A sin phi + r sin(psi - side * beta))
 * ------------------------------------------------------------------ */
/* Chassis step while the beam is still on the face, 0.1 deg. */
#define STEP_CHASSIS_DDEG       (50)
/* Servo step back across the edge, degrees: the edge to +-0.5 deg,
 * ~2 mm at 20 cm. */
#define STEP_FINE_DEG           (1)
/* Wheel speed for the chassis steps: spin turns stall on the foam below
 * ~60 mm/s (sub_motion speed_floor). */
#define STEP_TURN_MM_S          (60)
/* Settle after a chassis step before reading the heading and pinging. */
#define STEP_SETTLE_MS          (150U)
/* Pings per stop; the nearest valid one counts (a face seen at an angle
 * sometimes returns no echo). */
#define STEP_PINGS              (2U)

/* chassis heading now, 0.1 deg (> 0 left), from the encoders */
static int32_t  step_phi;
/* the face straight ahead, from the sensor, mm (0 = no echo) */
static int32_t  step_face;
/* stops pinged this scan */
static uint32_t step_stops;
/* each side's edge (0 left, 1 right): beam heading, range, the chassis
 * heading it was pinged at, whether it was found */
static int32_t  step_edge_psi[2];
static int32_t  step_edge_r[2];
static int32_t  step_edge_phi[2];
static bool     step_edge_ok[2];

/* atan(y / x) in 0.1 deg, x > 0, y >= 0: a binary search on the sine
 * table (no maths library, no FPU). */
static int32_t iatan_ddeg(int32_t y, int32_t x)
{
    int32_t lo = 0, hi = 900;

    while (lo < hi) {
        int32_t m = (lo + hi) / 2;

        if (((int64_t)isin(m) * x) < ((int64_t)y * isin(900 - m))) {
            lo = m + 1;
        } else {
            hi = m;
        }
    }
    return lo;
}

/* Turns the chassis in place to heading `to` (0.1 deg, > 0 left), stops
 * and settles. step_phi is then the heading the encoders say it reached
 * (an overshoot is measured, not corrected). False if aborted or the
 * wheels did not get there within ~2 s. */
static bool chassis_to(int32_t to)
{
    uint32_t i;
    bool     ok = false;

    if (to == step_phi) {
        return true;
    }
    spin_from = step_phi;
    spin_dir  = (to > step_phi) ? 1 : -1;
    spin_l0   = drv_encoder_count(RC_SIDE_LEFT);
    spin_r0   = drv_encoder_count(RC_SIDE_RIGHT);
    /* sub_motion steer > 0 turns right; spin_dir > 0 is left */
    (void)sub_motion_drive_speed(0, (int16_t)(-spin_dir * STEP_TURN_MM_S));
    for (i = 0U; i < 200U; i++) {               /* 10 ms each */
        if ((spin_dir * (spin_heading() - to)) >= 0) {
            ok = true;
            break;
        }
        if (spin_aborted()) {
            break;
        }
    }
    (void)sub_motion_stop(true);
    (void)tk_dly_tsk((RELTIM)STEP_SETTLE_MS);
    step_phi = spin_heading();
    return ok;
}

/* STEP_PINGS pings at servo angle `servo` (90 = ahead, > 90 left): the
 * nearest valid range, 0 if none echoed, -1 if aborted. */
static int32_t step_ping(int16_t servo)
{
    uint32_t k;
    int32_t  best = 0;

    for (k = 0U; k < STEP_PINGS; k++) {
        if (!step_to(servo)) {                  /* settles, then pings */
            continue;
        }
        if (!wait_result()) {
            return -1;
        }
        if (last_valid && ((best == 0) || ((int32_t)last_range_mm < best))) {
            best = (int32_t)last_range_mm;
        }
        (void)tk_dly_tsk(RC_PERIOD_SCAN_MS);    /* no ghost echoes */
    }
    step_stops++;
    return best;
}

/* True if range r, pinged from chassis heading phi along beam heading
 * psi, is the face and not something behind it: no further than the
 * face's plane along the beam's nearer edge, plus SPIN_DEPTH_BAND_MM. */
static bool step_on_face(int32_t phi, int32_t psi, int32_t r)
{
    int32_t ys  = (SPIN_SENSOR_AHEAD_MM * isin(900 - iabs(phi))) / 10000;
    int32_t off = iabs(psi) - SPIN_BEAM_HALF_DDEG;
    int32_t c;

    if (r <= 0) {
        return false;
    }
    if (off < 0) {
        off = 0;
    }
    c = isin(900 - off);
    return (c > 0)
           && (r <= (((((SPIN_SENSOR_AHEAD_MM + step_face) - ys) * 10000) / c)
                     + SPIN_DEPTH_BAND_MM));
}

/* True once the beam's inner edge (psi - side * beta) points past where
 * the corner of the longest allowed obstacle could be: OBSTACLE_MAX_MM
 * out from the line on that side, on the face's plane. */
static bool step_past_max(int32_t side, int32_t phi, int32_t psi)
{
    int32_t xs = -(SPIN_SENSOR_AHEAD_MM * isin(phi)) / 10000;
    int32_t ys = (SPIN_SENSOR_AHEAD_MM * isin(900 - iabs(phi))) / 10000;
    int32_t dx = OBSTACLE_MAX_MM + (side * xs);     /* sensor to corner, sideways */
    int32_t dy = (SPIN_SENSOR_AHEAD_MM + step_face) - ys;

    if ((dx <= 0) || (dy <= 0)) {
        return true;
    }
    return ((side * psi) - SPIN_BEAM_HALF_DDEG)
           > (iatan_ddeg(dx, dy) + STEP_CHASSIS_DDEG);
}

/* One side (+1 left, -1 right), see "Step scan". Leaves the chassis
 * where the edge was found. False if aborted. */
static bool step_side(int32_t side)
{
    uint32_t k = (side > 0) ? 0U : 1U;
    int16_t  servo_out = (int16_t)(90 + (side * RC_SERVO_HALF_SWING));
    /* the last reading on the face: straight ahead to begin with */
    int32_t  last_psi = 0, last_r = step_face, last_phi = 0;
    int32_t  r, psi, clear_psi, s, p;
    uint32_t guard;

    step_edge_ok[k] = false;
    for (guard = 0U; guard < 20U; guard++) {
        psi = step_phi + (side * RC_SERVO_HALF_SWING * 10);
        r   = step_ping(servo_out);
        if (r < 0) {
            return false;
        }
        if (step_on_face(step_phi, psi, r)) {
            last_psi = psi;
            last_r   = r;
            last_phi = step_phi;
            if (step_past_max(side, step_phi, psi)) {
                tm_printf((UB *)"[step] %s: still on it at %d (0.1 deg),"
                          " longer than allowed\n",
                          (side > 0) ? "left" : "right", (INT)psi);
                return true;
            }
            if (!chassis_to(step_phi + (side * STEP_CHASSIS_DDEG))) {
                return false;
            }
            continue;
        }
        /* off the face here: walk the servo back to where it echoes */
        clear_psi = psi;
        for (s = RC_SERVO_HALF_SWING - STEP_FINE_DEG; s >= -RC_SERVO_HALF_SWING;
             s -= STEP_FINE_DEG) {
            p = step_phi + (side * s * 10);
            if ((side * (p - last_psi)) <= 0) {
                break;                  /* back at the last face reading */
            }
            r = step_ping((int16_t)(90 + (side * s)));
            if (r < 0) {
                return false;
            }
            if (step_on_face(step_phi, p, r)) {
                last_psi = p;
                last_r   = r;
                last_phi = step_phi;
                break;
            }
            clear_psi = p;
        }
        step_edge_psi[k] = (last_psi + clear_psi) / 2;
        step_edge_r[k]   = last_r;
        step_edge_phi[k] = last_phi;
        step_edge_ok[k]  = true;
        return true;
    }
    return true;
}

/* The whole step scan: face, left side, back, right side, back. Leaves
 * the servo ahead and the motors stopped. */
static void run_step(void)
{
    int32_t side, r;

    step_stops      = 0U;
    step_phi        = 0;
    step_face       = 0;
    step_edge_ok[0] = false;
    step_edge_ok[1] = false;
    state           = S_STEP;

    r = step_ping(90);
    if ((r > 0) && (r <= (int32_t)OBSTACLE_MM)) {
        step_face = r;
        for (side = 1; side >= -1; side -= 2) {
            if (!step_side(side) || !chassis_to(0)) {
                break;
            }
        }
    }
    (void)drv_servo_set_angle(90);
    (void)sub_motion_stop(true);
}

/* Profile from the step scan, in the form spin_plan and car/01/spin
 * take: closest = the face, width = length between the corners when both
 * were found, clearance_left/right = each corner's distance from the line
 * (at least 1), 0 when that edge was not found. */
static void step_profile(rc_pl_profile_t *p)
{
    int32_t  side, x;
    uint32_t k, slot;

    p->n_points           = RC_SAT_U(step_stops, RC_BITS_NPOINTS);
    p->closest_angle_deg  = 90;
    p->closest_mm         = 0U;
    p->width_mm           = 0U;
    p->clearance_left_mm  = 0U;
    p->clearance_right_mm = 0U;
    spin_dbg.face_mm  = 0U;
    spin_dbg.found    = 0U;
    spin_dbg.n_points = (uint8_t)((step_stops > 255U) ? 255U : step_stops);
    spin_has_l = false;
    spin_has_r = false;

    if (step_face <= 0) {
        return;                         /* nothing ahead: closest 0 */
    }
    p->closest_mm    = RC_SAT_U(step_face, RC_BITS_MM);
    spin_dbg.face_mm = (uint16_t)step_face;

    for (side = 1; side >= -1; side -= 2) {
        k = (side > 0) ? 0U : 1U;
        if (!step_edge_ok[k]) {
            continue;
        }
        x = -((SPIN_SENSOR_AHEAD_MM * isin(step_edge_phi[k]))
              + (step_edge_r[k]
                 * isin(step_edge_psi[k] - (side * SPIN_BEAM_HALF_DDEG))))
            / 10000;
        slot = (side > 0) ? 0U : 2U;
        spin_dbg.ddeg[slot]  = (int16_t)step_edge_psi[k];
        spin_dbg.r_2mm[slot] = (uint8_t)((step_edge_r[k] > 510) ? 255
                                         : (step_edge_r[k] / 2));
        spin_dbg.found      |= (uint8_t)(1U << slot);
        tm_printf((UB *)"[step] %s edge: beam %d, chassis %d (0.1 deg),"
                  " %d mm -> corner x %d\n", (side > 0) ? "left" : "right",
                  (INT)step_edge_psi[k], (INT)step_edge_phi[k],
                  (INT)step_edge_r[k], (INT)x);
        if (side > 0) {
            spin_xl    = x;
            spin_has_l = true;
            p->clearance_left_mm  = RC_SAT_U((x < -1) ? -x : 1, RC_BITS_CLEAR);
        } else {
            spin_xr    = x;
            spin_has_r = true;
            p->clearance_right_mm = RC_SAT_U((x > 1) ? x : 1, RC_BITS_CLEAR);
        }
    }
    if (spin_has_l && spin_has_r && (spin_xr > spin_xl)) {
        p->width_mm = RC_SAT_U(spin_xr - spin_xl, RC_BITS_MM);
    }
    tm_printf((UB *)"[step] face %d mm, %d stops, length %d mm\n",
              (INT)step_face, (INT)step_stops, (INT)p->width_mm);
}

/* The scan task, created in sub_scan_init(). Every wait in it is a
 * kernel sleep, so it is off the CPU for nearly the whole scan. */
static void scan_task(INT stacd, void *exinf)
{
    ER er;

    (void)stacd;
    (void)exinf;

    for (;;) {
        if (state == S_WATCH) {
            /* Forward watch: one ping straight ahead per period.
             * sub_nav subscribes to RC_EVT_ULTRA_RESULT itself and decides
             * when a reading is close enough to start avoidance. */
            (void)drv_servo_set_angle(90);
            if (drv_ultrasonic_ping(90) == RC_OK) {
                (void)wait_result();
            }
            (void)tk_dly_tsk(RC_PERIOD_SCAN_MS);
            continue;
        }

        /* Idle: sleep until a request signals scan_semid; the 100 ms
         * timeout re-checks S_WATCH. */
        er = tk_wai_sem(scan_semid, 1, 100);
        if (er != E_OK) {
            continue;
        }
        (void)tk_clr_flg(scan_flgid, ~(UINT)FLG_RESULT);    /* no stale echo */

        /* Run the step scan, then report the result. */
        run_step();
        publish_and_finish();

        /* If a continuous watch was requested while this scan was
         * running, drop back into S_WATCH instead of S_IDLE once done. */
        if (watch_on) {
            state = S_WATCH;
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

/* Creates the event flag, the request semaphore and the scan task, and
 * registers on_range() as the ultrasonic driver's result callback. Call
 * once at boot, after drv_servo_init() and drv_ultrasonic_init(). */
rc_result_t sub_scan_init(void)
{
    T_CFLG cflg;
    T_CSEM csem;
    T_CTSK ctsk;

    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;               /* no bits raised at creation */
    scan_flgid   = tk_cre_flg(&cflg);
    if (scan_flgid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    /* At most one request waits: a second start before scan_task takes
     * the first is the same request. */
    csem.exinf   = NULL;
    csem.sematr  = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 0;
    csem.maxsem  = 1;
    scan_semid   = tk_cre_sem(&csem);
    if (scan_semid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    (void)drv_ultrasonic_on_result(on_range, NULL);

    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_SCAN;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = scan_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3 | RC_CTL_TSKATR;
    ctsk.assprc  = RC_CTL_ASSPRC;   /* same core as the motion mutex's users */

    scan_tskid = tk_cre_tsk(&ctsk);
    if (scan_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(scan_tskid, 0);

    state = S_IDLE;
    return RC_OK;
}

/* Clears an abort left over from an earlier scan, then counts the
 * request. One request already waiting (E_QOVR) is not an error. Called
 * from the same task as sub_scan_abort() (sub_nav), so the two cannot
 * interleave. */
static rc_result_t request_scan(void)
{
    ER er;

    (void)tk_clr_flg(scan_flgid, ~(UINT)FLG_ABORT);
    er = tk_sig_sem(scan_semid, 1);
    return ((er == E_OK) || (er == E_QOVR)) ? RC_OK : RC_ERR_STATE;
}

/* Requests a step scan (see "Step scan"): servo and chassis measure the
 * obstacle's length. Returns at once: RC_ERR_BUSY if a scan is already
 * running, else RC_OK; the result arrives later as events
 * (publish_and_finish). The motors must be free (sub_nav AVOIDING); they
 * are stopped when it ends. */
rc_result_t sub_scan_start_measure(void)
{
    if ((state != S_IDLE) && (state != S_WATCH)) {
        return RC_ERR_BUSY;
    }
    state = S_IDLE;
    return request_scan();
}

/* Requests scan_task stop whatever it's doing (it checks FLG_ABORT
 * inside wait_result()) and immediately forces the visible state back to
 * idle so a caller checking sub_scan_busy() right after this returns
 * sees "not busy" without needing to wait for scan_task to actually
 * notice the flag. */
rc_result_t sub_scan_abort(void)
{
    (void)tk_wai_sem(scan_semid, 1, TMO_POL);  /* drop a request not yet taken */
    (void)tk_set_flg(scan_flgid, FLG_ABORT);
    state = S_IDLE;
    return RC_OK;
}

/* True while a step scan is running; the S_WATCH ping does not count as
 * busy. */
bool sub_scan_busy(void)
{
    return (state != S_IDLE) && (state != S_WATCH);
}

/* Turns the forward watch on or off. From idle it switches straight to
 * S_WATCH; from S_WATCH back to S_IDLE. During a scan it only records the
 * request (watch_on), and scan_task enters S_WATCH when the scan ends.
 * `trigger_mm` is not used here: sub_nav compares the watch ranges with
 * its own trigger. */
rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm)
{
    (void)trigger_mm;
    watch_on = on;

    if (on && (state == S_IDLE)) {
        state = S_WATCH;
    } else if (!on && (state == S_WATCH)) {
        state = S_IDLE;
    } else {
        /* leave a running scan alone */
    }
    return RC_OK;
}
