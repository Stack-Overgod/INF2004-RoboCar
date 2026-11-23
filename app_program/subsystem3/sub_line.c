/*
 *  sub_line.c  -  line follower on RC_EVT_LINE_SAMPLE, steering sub_motion
 */
#include "../rc_prelude.h"
#include "sub_line.h"
#include "line_scan.h"
#include "../subsystem2/sub_motion.h"
#include "../subsystem2/drv_encoder.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* 1 = the line is NARROWER than the gap between the L and R sensors (the
 * 18 mm track line, with the IR pair just outside its edges): centred =
 * 0/0, drift = 1/0 or 0/1, a crossing line / junction = 1/1. 0/0 only
 * means "lost" once it has lasted LOST_THRESHOLD samples, and each
 * correction is gentler so the car doesn't swing across the line.
 * 0 = line wider than the gap: 1/1 is centred, 0/0 is soon lost. */
#ifndef RC_LINE_STRADDLE
#define RC_LINE_STRADDLE    (1)
#endif

/* 2 s of 0/0 before the line counts as lost and the search starts: 200
 * samples of RC_PERIOD_LINE_MS, ~60 mm at line speed. Longer leaves the
 * car too far past the end of the line before it searches. In edge mode
 * (RC_LINE_STRADDLE 0) the car arcs back toward the last-seen side until
 * then. */
#define LOST_THRESHOLD      (200U)
/* Gentle first correction: position (+-500) x STEER_KP / STEER_DIV =
 * +-30 permille. A longer one-sided reading keeps tightening the turn
 * (STEER_STEP). */
#define STEER_KP            (3)
#define STEER_DIV           (50)

/* Both sensors on the line for this long means a junction or a barcode
 * lead-in, not a centred car. */
#define JUNCTION_THRESHOLD  (6U)

/* One sensor alone must read black for this many samples in a row before
 * the car steers. A crossing line or barcode bar met at a slight angle
 * darkens one sensor one sample before the other; without this wait the
 * car jerks towards it. 1 sample = 10 ms (~3 mm at cruise). */
#define STEER_CONFIRM       (2U)

/* Drift corrections (straddle mode). While one sensor stays on the line
 * the steer grows by STEER_STEP per sample from the gentle value; on 0/0
 * it decays by STEER_DECAY per sample instead of snapping straight (the
 * sensor flickers 01/00/01 on the line edge). Capped at STEER_MAX; a
 * reading that stays one-sided for CURVE_AFTER becomes a curve (below). */
#define STEER_STEP          (10)    /* +100 per 100 ms one-sided */
#define STEER_DECAY         (15)    /* back to straight in ~0.2 s */
/* A curve handing over to the other sensor starts at STEER_HANDOVER. With
 * both at 100 the car swings across the line every 0.25 s. */
#define STEER_MAX           (60)
#define STEER_HANDOVER      (50)

/* Derivative (D) damping for the drift correction. The error is -1 / 0 /
 * +1 (line under the left sensor / between / right). Each change of error
 * adds STEER_KD * (change) to a d_term that decays by D_DECAY per sample
 * (~30 ms). When the line comes back to the middle the d_term briefly
 * steers the other way, so the car stops turning before it overshoots;
 * when the line arrives under a sensor it adds a quick extra push.
 * This stops the car zig-zagging on straights. Not used in arcs. */
#define STEER_KD            (40)
#define D_DECAY             (700)   /* permille kept per sample */

/* Open loop: every duty this module commands is scaled by this, permille
 * (750 = 25 % slower). Includes the arcs and the inner-wheel floor, so the
 * floor drops to ~113: if the inner wheel stalls in turns (dead band, see
 * TURN_INNER_MIN), raise TURN_INNER_MIN rather than this. */
#define SPEED_SCALE         (750)

/* Open loop (LINE_CLOSED_LOOP 0): a curve turn is a near spin on the
 * spot, forward speed TURN_SPEED permille of the straight speed (100 =
 * 10 %), wheels base +- TURN_SPIN in opposite directions. */
#define TURN_SPEED          (100)
#define TURN_SPIN           (235)
/* Open loop: the inner wheel never goes below this duty. Between about
 * -100 and +100 it is in the motor dead band on the foam, the wheel locks
 * and the whole car stalls. */
#define TURN_INNER_MIN      (150)

/* Curves. One side for CURVE_AFTER samples (150 ms) commits the car to an
 * arc of the track's curve radius toward that side. The arc is HELD
 * through 0/0, because 0/0 is ambiguous: the line may be between the
 * sensors, or the car under-turned and the line slipped past the inner
 * sensor, and letting the turn fade then drives the car off the mat. The
 * curve ends when the OTHER sensor sees
 * the line (turned enough or a little past - it then corrects back), or
 * after CURVE_HOLD samples of 0/0.
 * A drift recentres within ~150 ms; a 140 mm corner keeps the line on
 * one side longer. Shorter values make ordinary drifts on a straight
 * commit arcs, and the car zig-zags. */
#define CURVE_AFTER         (16U)
#define CURVE_HOLD          (30U)
/* A 0/0 shorter than this (100 ms) does not reset the CURVE_AFTER count:
 * in a curve the sensor flickers on the line edge, and resetting on every
 * flicker would stop the curve ever committing. */
#define CURVE_GAP           (10U)
/* Escape: a one-sided reading of ESCAPE_MIN..ESCAPE_MAX samples that ends
 * in 0/0 means the line slid OUT past that sensor (a curve): a drift
 * correction cannot bring the line back to the middle that fast. On a
 * 140 mm curve the line crosses the sensor in ~35 mm (~0.13 s), before
 * CURVE_AFTER. The car then commits the arc toward that side, held for
 * CURVE_HOLD. */
/* Scaled with CURVE_AFTER: the same distance along the line. */
#define ESCAPE_MIN          (6U)
#define ESCAPE_MAX          (14U)   /* must stay below CURVE_AFTER */
/* ...and only if the line had sat between the sensors for this many
 * samples before that reading: a real curve starts from a
 * settled straight. Without this, every swing of a weave looks like an
 * escape, commits a full arc and swings the car back across. */
#define ESCAPE_SETTLED      (15U)   /* 150 ms */
/* 1 = drive on wheel SPEED (sub_motion_drive_speed, its PID finds the
 * duty), 0 = open-loop duty (sub_motion_drive). At half speed the open-loop
 * duty (~70 / 61) is inside the ~+-100 dead band on the foam and the
 * wheels do not turn; on speed the PID holds ~30 mm/s. Every module value
 * below stays in permille and is converted once, in line_drive(), so all
 * corrections and turns shrink with the speed and keep their shape (same
 * steering per mm travelled). */
#define LINE_CLOSED_LOOP    (1)
/* Wheel speed for one permille of command: ~63 mm/s measured at 175
 * open-loop, times LINE_SPEED_PCT. */
#define LINE_MM_S_AT_175    (63)
#define LINE_SPEED_PCT      (50)

/* Left steering is scaled up by this (permille): the left motor runs
 * faster at low duty, so an equal left correction turned less than a
 * right one (same ratio as ARC_SPEED_CAL_L / ARC_SPEED_CAL_R). Open loop
 * only: on speed the PID already matches the wheels. */
#if LINE_CLOSED_LOOP
#define STEER_TRIM_L        (1000)
#else
#define STEER_TRIM_L        ((1000 * ARC_SPEED_CAL_L) / ARC_SPEED_CAL_R)
#endif
#define ARC_RADIUS_MM       (140)   /* the track's curve radius */
/* The arc the wheels are TOLD to drive in a curve (closed loop), tighter
 * than the track:
 *  - the sensors are ~80 mm ahead of the wheels, so for them to follow the
 *    140 mm curve the wheels must drive sqrt(140^2 - 80^2) ~ 115 mm;
 *  - the wheels drive wider than asked: sub_motion's PID never brakes and
 *    its feedforward runs the inner wheel faster than wanted.
 * 75 mm (wheel ratio 6.9, ~55 / 8 mm/s) follows the curve. Much tighter
 * stalls the inner wheel (~7 mm/s and below) and the car pivots on it. */
#define ARC_TURN_RADIUS_MM  (75)
/* Right curves only: the right (inner) wheel's speed x this, permille,
 * the left (outer) wheel unchanged. Right turns on the 140 mm curves need
 * the right motor ~5 % faster. */
#define ARC_RIGHT_INNER_PERMILLE (1050)
/* Open loop, per turn direction, permille: the motors differ at low duty
 * (at 150 inner, right turns ran 275 / 130 mm/s and left turns 275 / 160).
 * Only their ratio is used, by STEER_TRIM_L. */
#define ARC_SPEED_CAL_R     (1210)  /* right turn: left motor is outer */
#define ARC_SPEED_CAL_L     (1410)  /* left turn: right motor is outer */

/* Both sensors black: a crossing line (the middle of a figure of 8) or a
 * barcode bar clears in under ~100 ms and is crossed straight. A 1/1 that
 * lasts longer than CROSS_AFTER during a curve means the outer sensor ran
 * onto something dark and the inner one found the line: keep turning. */
#define CROSS_AFTER         (15U)

/* FR4 line search. Once LOST: a slight turn ON THE MOVE toward the side
 * the line was last seen on (last_position), the likely side, at line
 * speed with a SEARCH_NUDGE_STEER correction until the heading has
 * changed SEARCH_NUDGE_DEG (~0.9 s, ~28 mm); skipped if no side was ever
 * seen. A sensor seeing the line during it ends the search. Then the line
 * scan (line_scan.c) in place, up to SEARCH_SWEEP_DEG toward the
 * last-seen side first (left if none), then SEARCH_SWEEP_DEG the other
 * way, at SEARCH_SPIN_MM_S per wheel: the car turns onto the line both
 * sensors crossed. Not found: brake and report RECOVERY_FAILED. */
#define SEARCH_SWEEP_DEG    (135)
#define SEARCH_NUDGE_DEG    (10)    /* ~0.9 s */
#define SEARCH_NUDGE_STEER  (60)    /* permille, = STEER_MAX */
/* Sensor tip ~125 mm/s: an 18 mm line stays under it ~14 samples; 135
 * degrees in ~1.9 s. */
#define SEARCH_SPIN_MM_S    (70)
/* Each wheel's travel per degree of turn, um: (W/2) x pi/180 = 977. */
#define SEARCH_UM_PER_DEG   ((uint32_t)RC_WHEEL_BASE_MM * 1000UL * 31416UL \
                             / (2UL * 180UL * 10000UL))

/* `static` at file scope means "only visible inside this .c file" */
/* current line-follow mode, see sub_line.h enum */
static rc_line_state_t state = RC_LINE_TRACKING;
/* is steering actually allowed right now? (see sub_line_enable) */
static bool     enabled;
/* true once a LOST search timed out: no output until re-enabled */
static bool     gave_up;
/* forward speed while tracking, in permille (parts per 1000 of full
 * speed); line_drive() converts it */
static int16_t  base_permille = 175;
/* consecutive samples with neither sensor on the line */
static uint32_t lost_count;
/* consecutive samples with both sensors on the line (junction candidate) */
static uint32_t both_count;
/* consecutive samples with only one sensor on the line, and which side */
static uint32_t one_count;
static int16_t  one_position;
/* current drift steer, permille, > 0 right (see STEER_STEP) */
static int16_t  steer_level;
/* committed curve: +1 right, -1 left, 0 none (see CURVE_AFTER) */
static int16_t  curve_dir;
/* 0/0 samples since the curve last saw the line (see CURVE_HOLD) */
static uint32_t curve_gap;
/* the curve-side sensor has seen the line during this curve: a 0/0 then
 * means the line is back in the middle, so the spin stops at once. Only
 * an escape spin (line not seen yet) is held through 0/0; holding every
 * spin through 0/0 swings the line across to the other sensor. */
static bool     curve_seen;
/* most recent position reading, kept so we know which way to arc if the line vanishes */
static int16_t  last_position;
/* 0/0 samples before the current one-sided reading (see ESCAPE_SETTLED) */
static uint32_t pre_gap;
/* 0/0 samples in a row while really going straight: no curve held, no
 * correction fading. 0/0 inside a curve hold must not count as settled,
 * or the car alternates right and left arcs. */
static uint32_t settled;
/* derivative damping term and the previous error (see STEER_KD) */
static int16_t  d_term;
static int16_t  prev_err;
/* sub_barcode is reading a barcode: the line is not declared LOST (see
 * sub_line_set_reading) */
static bool     reading;
/* line search after LOST (see SEARCH_SWEEP_DEG): the nudge is running,
 * both wheels' encoder clicks when it started and its direction (+1
 * right, -1 left); our line scan is running */
static bool     nudge_on;
static uint32_t nudge_l0;
static uint32_t nudge_r0;
static int32_t  nudge_dir;
static bool     lost_scan;

/* Left/right motor match, permille of base added to the left wheel and
 * taken off the right: at equal duty the left wheel runs ~15 % slower and
 * the car twists left on straights. 70 = left/right duty 1.07 / 0.93; 0
 * once the left wheel is fixed mechanically. Open loop only: on speed the
 * PID matches the wheels, and a trim would curve the car. */
#if LINE_CLOSED_LOOP
#define STRAIGHT_TRIM       (0)
#else
#define STRAIGHT_TRIM       (70)
#endif

/* Every motor command from this module goes through here. `base` and
 * `steer` are permille; closed loop turns them into wheel speeds
 * (LINE_MM_S_AT_175, LINE_SPEED_PCT), open loop into duty (STRAIGHT_TRIM,
 * SPEED_SCALE). Barcodes are read at this same speed. */
static void line_drive(int32_t base, int32_t steer)
{
#if LINE_CLOSED_LOOP
    int32_t k_num = LINE_MM_S_AT_175 * LINE_SPEED_PCT;
    int32_t k_den = 175 * 100;

    (void)sub_motion_drive_speed((int16_t)((base * k_num) / k_den),
                                 (int16_t)((steer * k_num) / k_den));
#else
    steer += (base * STRAIGHT_TRIM) / 1000;
    (void)sub_motion_drive((int16_t)((base * SPEED_SCALE) / 1000),
                           (int16_t)((steer * SPEED_SCALE) / 1000));
#endif
}

/* Drive with the current steer_level, slowing as the turn tightens. */
static void drive_level(void)
{
    int32_t total = (int32_t)steer_level + d_term;
    int32_t mag   = (total < 0) ? (-total * STEER_TRIM_L) / 1000 : total;
    int32_t base  = (int32_t)base_permille - (mag / 2);

    /* Open loop: keep the inner wheel out of the dead band (this speeds
     * the car up in corrections). On speed the PID turns the inner wheel
     * at any target; worst case STEER_MAX + d_term ~ 100 leaves it at
     * ~25 permille, still forwards. */
    if (!LINE_CLOSED_LOOP && (mag != 0) && ((base - mag) < TURN_INNER_MIN)) {
        base = TURN_INNER_MIN + mag;    /* keep the inner wheel turning */
    }
    line_drive(base, (total < 0) ? -mag : mag);
}

#if LINE_CLOSED_LOOP
/* Drive an arc of radius ARC_TURN_RADIUS_MM toward `dir` (+1 right, -1
 * left) at the line speed. For wheel spacing W the wheels run at
 * (R +- W/2) / R of the centre speed; on speed the PID holds that ratio,
 * so no inner-wheel floor is needed. */
static void drive_arc(int16_t dir)
{
    int32_t base  = base_permille;
    int32_t steer = (base * ((int32_t)RC_WHEEL_BASE_MM / 2)) / ARC_TURN_RADIUS_MM;
    int32_t left, right;

    if (dir < 0) {
        line_drive(base, -steer);
    } else {
        /* left wheel base + steer, right wheel scaled by
         * ARC_RIGHT_INNER_PERMILLE (line_drive: left = base + steer,
         * right = base - steer) */
        left  = base + steer;
        right = ((base - steer) * ARC_RIGHT_INNER_PERMILLE) / 1000;
        line_drive((left + right) / 2, (left - right) / 2);
    }
}
#else
/* Near spin toward `dir` (+1 right, -1 left), see TURN_SPEED. */
static void drive_arc(int16_t dir)
{
    int32_t base  = ((int32_t)base_permille * TURN_SPEED) / 1000;
    int32_t steer = TURN_SPIN;

    line_drive(base, (dir < 0) ? -steer : steer);
}
#endif

/* One more sample with the line under one sensor (position < 0 = left):
 * start gentle on a new side, otherwise tighten by STEER_STEP. */
static void tighten_toward(int16_t position)
{
    int32_t mag    = (position < 0) ? -(int32_t)position : (int32_t)position;
    int32_t gentle = (mag * STEER_KP) / STEER_DIV;
    int32_t lvl    = steer_level;

    if ((lvl == 0) || ((lvl < 0) != (position < 0))) {
        lvl = (position < 0) ? -gentle : gentle;
    } else {
        lvl += (position < 0) ? -STEER_STEP : STEER_STEP;
    }
    if (lvl > STEER_MAX) {
        lvl = STEER_MAX;
    } else if (lvl < -STEER_MAX) {
        lvl = -STEER_MAX;
    }
    steer_level = (int16_t)lvl;
}

/* One more 0/0 sample: let the turn fade out. */
static void decay_level(void)
{
    if (steer_level > STEER_DECAY) {
        steer_level = (int16_t)(steer_level - STEER_DECAY);
    } else if (steer_level < -STEER_DECAY) {
        steer_level = (int16_t)(steer_level + STEER_DECAY);
    } else {
        steer_level = 0;
    }
}

/* Publish an event that carries no payload. */
static void publish_id(rc_evt_id_t id)
{
    rc_event_t out = { 0 };

    out.id = id;
    (void)rc_event_publish(&out);
}

/* Degrees the nudge has turned so far: on the move (both wheels forward)
 * the heading change is HALF the outer-minus-inner wheel travel over
 * SEARCH_UM_PER_DEG. */
static int32_t nudge_heading(void)
{
    uint32_t dl    = drv_encoder_count(RC_SIDE_LEFT) - nudge_l0;
    uint32_t dr    = drv_encoder_count(RC_SIDE_RIGHT) - nudge_r0;
    uint32_t outer = (nudge_dir > 0) ? dl : dr;
    uint32_t inner = (nudge_dir > 0) ? dr : dl;
    uint32_t half  = (outer > inner) ? ((outer - inner) / 2U) : 0U;

    return (int32_t)((half * RC_ENC_UM_PER_TICK) / SEARCH_UM_PER_DEG);
}

/* The line scan of the search ended (line_scan.c, FAST lane). Found: the
 * car faces along the line, tracking resumes on the next sample. Not
 * found: FR4, brake (the scan did) and report it; stay silent until
 * sub_line_enable(). */
static void lost_scan_done(bool found, void *ctx)
{
    (void)ctx;

    lost_scan = false;
    if (found) {
        lost_count  = 0U;
        curve_dir   = 0;
        steer_level = 0;
        d_term      = 0;
        state       = RC_LINE_TRACKING;
        publish_id(RC_EVT_LINE_REACQUIRED);
    } else {
        gave_up = true;
        publish_id(RC_EVT_RECOVERY_FAILED);
    }
}

/* Start the search's line scan: the last-seen side first. */
static void lost_scan_begin(void)
{
    line_scan_cfg_t c;

    c.first_dir  = (last_position > 0) ? (int16_t)-1 : (int16_t)1;
    c.quick_deg  = 0;
    c.first_deg  = SEARCH_SWEEP_DEG;
    c.back_deg   = SEARCH_SWEEP_DEG;
    c.speed_mm_s = SEARCH_SPIN_MM_S;
    nudge_on     = false;
    if (line_scan_start(&c, lost_scan_done, NULL) == RC_OK) {
        lost_scan = true;
    } else {
        gave_up = true;             /* another scan owns the car */
        (void)sub_motion_stop(true);
        publish_id(RC_EVT_RECOVERY_FAILED);
    }
}

/* Starts the search at LOST: the nudge toward the last-seen side, or
 * straight to the scan if no side was ever seen. */
static void search_begin(void)
{
    if (last_position != 0) {
        nudge_on  = true;
        nudge_dir = (last_position < 0) ? -1 : 1;
        nudge_l0  = drv_encoder_count(RC_SIDE_LEFT);
        nudge_r0  = drv_encoder_count(RC_SIDE_RIGHT);
    } else {
        lost_scan_begin();
    }
}

/* One 0/0 sample of the nudge: drive it, then hand over to the scan. */
static void nudge_step(void)
{
    if (nudge_heading() >= SEARCH_NUDGE_DEG) {
        lost_scan_begin();
        return;
    }
    line_drive(base_permille, nudge_dir * SEARCH_NUDGE_STEER);
}

/* A sensor sees the line again (1/0, 0/1 or 1/1). Report §6.5: "any
 * sensor seeing the line -> LINE_REACQUIRED" after a loss (LOST), before
 * the junction check, because re-entering the line at an angle often
 * reads 1/1. */
static void line_seen(void)
{
    lost_count = 0U;
    nudge_on   = false;         /* a search nudge ends here */

    if (state == RC_LINE_LOST) {
        state = RC_LINE_TRACKING;
        publish_id(RC_EVT_LINE_REACQUIRED);
    }
}

/* ------------------------------------------------------------------ *
 *  Sample callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

static void on_sample(const rc_event_t *evt, void *ctx)
{
    const rc_pl_line_t *s = &evt->u.line;
    const int16_t       pos = rc_line_position(s);

    (void)ctx;

    if (!enabled || gave_up || line_scan_busy()) {
        return;                     /* a line scan has the motors */
    }

    {
        /* Derivative term (see STEER_KD): error -1 / 0 / +1. */
        int16_t err = 0;

        if (s->on_line_l != s->on_line_r) {
            err = (int16_t)((pos < 0) ? -1 : 1);
        }
        d_term = (int16_t)(((int32_t)d_term * D_DECAY) / 1000
                           + (int32_t)STEER_KD * (err - prev_err));
        prev_err = err;
    }

    if (s->on_line_l && s->on_line_r) {
        /* Both sensors see black at once; we require
         * JUNCTION_THRESHOLD consecutive ones first (debouncing). */
        line_seen();
        one_count = 0U;
        settled   = 0U;
        both_count++;

#if !RC_LINE_STRADDLE
        /* Edge mode: 1/1 is the centred car, so a curve is over as soon
         * as both sensors are back on the line. */
        curve_dir   = 0;
        steer_level = 0;
#endif
        if (curve_dir != 0) {
            if (both_count > CROSS_AFTER) {
                /* Too long for a crossing line: the outer sensor is on
                 * something dark and the other one found the line. Curve
                 * over, steer toward the new sensor. */
                int16_t toward = (int16_t)((curve_dir > 0) ? -500 : 500);

                curve_dir   = 0;
                steer_level = 0;
                tighten_toward(toward);
                drive_level();
            } else {
                /* A crossing line inside a curve (figure of 8): cross it
                 * straight, stay committed to the curve. */
                line_drive(base_permille, 0);
            }
            return;
        }

        if (both_count >= JUNCTION_THRESHOLD) {
            state = RC_LINE_JUNCTION;
        }
        /* Drive straight through (a crossing, junction or bar). */
        line_drive(base_permille, 0);

    } else if (s->on_line_l || s->on_line_r) {
        /* Exactly one sensor sees black: normal tracking case. */
        if ((one_count == 0U) || (pos != one_position)) {
            pre_gap = settled;
        }
        settled = 0U;
        line_seen();
        both_count = 0U;
        state      = RC_LINE_TRACKING;     /* also leaves JUNCTION */
        last_position = pos;

        if (curve_dir != 0) {
            if ((pos < 0) == (curve_dir < 0)) {
                curve_gap  = 0U;            /* still on the curve side */
                curve_seen = true;
                drive_arc(curve_dir);
                return;
            }
            /* The other sensor found the line: turned enough (or a
             * little past). Curve over, correct back toward it. */
            curve_dir    = 0;
            steer_level  = (int16_t)((pos < 0) ? -STEER_HANDOVER
                                                        : STEER_HANDOVER);
            one_position = pos;
            one_count    = STEER_CONFIRM;
            drive_level();
            return;
        }

        if (pos != one_position) {
            one_position = pos;
            one_count    = 0U;
        }
        if (one_count < CURVE_AFTER) {
            one_count++;
        }
        if (one_count >= CURVE_AFTER) {
            curve_dir   = (int16_t)((pos < 0) ? -1 : 1);
            curve_seen  = true;
            curve_gap   = 0U;
            steer_level = 0;
            drive_arc(curve_dir);
        } else if (one_count >= STEER_CONFIRM) {
            tighten_toward(pos);
            drive_level();
        } else {
            /* Not confirmed yet: may be the first half of a 1/1. */
            line_drive(base_permille, 0);
        }

    } else {
        /* Neither sensor sees black.*/
        both_count = 0U;
        lost_count++;

        if ((reading || RC_STAND_SOAK) && (lost_count >= LOST_THRESHOLD)) {
            /* Reading a barcode: the white spaces between the bars are
             * not a lost line. Hold the count just short of LOST, so the
             * steering below carries on as usual; the line is declared
             * lost only if it is still missing once the read ends. */
            lost_count = LOST_THRESHOLD - 1U;
        }

        if (lost_count >= LOST_THRESHOLD) {
            /* Officially lost now: a slight turn on the move toward the
             * last-seen side, then the line scan in place (see
             * SEARCH_NUDGE_DEG, SEARCH_SWEEP_DEG). */
            if (state != RC_LINE_LOST) {
                state       = RC_LINE_LOST;
                curve_dir   = 0;
                steer_level = 0;
                d_term      = 0;
                publish_id(RC_EVT_LINE_LOST);
                search_begin();
            }
            if (nudge_on) {
                nudge_step();
            }
        } else {
#if RC_LINE_STRADDLE
            /* 0/0, not lost yet. In a curve: hold the arc (see
             * CURVE_HOLD). Otherwise the line is between the sensors:
             * let the drift correction fade out. */
            if ((curve_dir == 0) && (steer_level == 0)) {
                settled++;
            } else {
                settled = 0U;
            }
            if ((curve_dir == 0) && (lost_count == 1U)
                && (one_count >= ESCAPE_MIN) && (one_count <= ESCAPE_MAX)
                && (pre_gap >= ESCAPE_SETTLED)) {
                /* The line just slid out past the sensor (see ESCAPE_MIN). */
                curve_dir = (int16_t)((one_position < 0) ? -1 : 1);
                curve_gap = 0U;
                curve_seen = false;
                steer_level = 0;
            }
            if (curve_dir != 0) {
                curve_gap++;
                if (!curve_seen && (curve_gap <= CURVE_HOLD)) {
                    drive_arc(curve_dir);
                    return;
                }
                curve_dir   = 0;
                steer_level = 0;
            }
            decay_level();
            drive_level();
            if (lost_count > CURVE_GAP) {
                one_count = 0U;     /* a short flicker keeps the count */
            }
#else
            /* Edge mode: 0/0 = off the line, which slid out past the
             * sensor that saw it last (a curve, or a drift). Follow the
             * curve arc (drive_arc) back toward that side. While a barcode is
             * read its white spaces read 0/0 too: keep straight then. */
            if (reading) {
                line_drive(base_permille, 0);
            } else {
                drive_arc((last_position < 0) ? -1 : 1);
            }
#endif
        }
    }
}

/* ------------------------------------------------------------------ */

/* Call once at boot.*/
rc_result_t sub_line_init(void)
{
    if (rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_FAST,
                           on_sample, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    state   = RC_LINE_TRACKING;
    enabled = false;
    gave_up = false;
    return RC_OK;
}

/* Turns steering on/off. Also ends a search (its scan too) and clears a
 * failed one, so enabling again starts afresh. */
rc_result_t sub_line_enable(bool on)
{
    if (lost_scan) {
        line_scan_abort();
        lost_scan = false;
    }
    enabled     = on;
    gave_up     = false;
    nudge_on    = false;
    if (state == RC_LINE_LOST) {
        state = RC_LINE_TRACKING;   /* a new run starts a new search */
    }
    lost_count  = 0U;
    both_count  = 0U;
    one_count   = 0U;
    steer_level = 0;
    curve_dir   = 0;
    d_term      = 0;
    prev_err    = 0;
    return RC_OK;
}

/* sub_barcode calls this when a read starts (on) and ends (off). Only
 * the LOST declaration is held off; steering is unchanged. */
rc_result_t sub_line_set_reading(bool on)
{
    if (!on && reading && (lost_count >= (LOST_THRESHOLD - 1U))) {
        /* The read held the count just short of LOST; start the 0/0
         * count afresh, or the next 0/0 (a centred car) reads as lost
         * at once. */
        lost_count = 0U;
    }
    reading = on;
    return RC_OK;
}

bool sub_line_reading(void)
{
    return reading;
}

/*** end of file ***/
