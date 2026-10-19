/*
 *  sub_line.c
 */
#include "../rc_prelude.h"
#include "sub_line.h"
#include "../subsystem2/sub_motion.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* 1 = the line is NARROWER than the gap between the L and R sensors (the
 * track's 18 mm line, 2026-10-18: over it only one LED ever goes out). A
 * centred car then reads 0/0 with the line between the sensors, so 0/0
 * only means "lost" once it has lasted LOST_THRESHOLD samples, and each
 * correction is gentler so the car doesn't swing across the line.
 * 0 = line wider than the gap: 1/1 is centred, 0/0 is soon lost. */
#ifndef RC_LINE_STRADDLE
#define RC_LINE_STRADDLE    (1)
#endif

#if RC_LINE_STRADDLE
/* 5 s of 0/0 before we call the line lost (Buddy 5's track call,
 * 2026-10-19; was 2 s). 500 samples of RC_PERIOD_LINE_MS. */
#define LOST_THRESHOLD      (500U)
/* Gentle correction, +-30 permille: wheels 380 / 320 at base 350, about
 * 46 mm/s wheel difference = 0.4 rad/s, so ~2-5 degrees over the 0.1 to
 * 0.23 s a correction lasts (Buddy 5's on-track target: ~5 degrees).
 * +-125 and +-75 were too steep for small drifts (2026-10-19). A longer
 * one-sided reading keeps tightening the turn (STEER_STEP). */
#define STEER_KP            (3)
#define STEER_DIV           (50)
#else
/* How many consecutive all-dark samples before we call the line lost.*/
#define LOST_THRESHOLD      (20U)
#define STEER_KP            (2)
#define STEER_DIV           (2)
#endif

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
/* 60, and a curve handing over to the other sensor starts at
 * STEER_HANDOVER: at 100 / 100 the car swung across the line every 0.25 s
 * for 3 s (2026-10-19 19:41, 176/408 <-> 400/200). */
#define STEER_MAX           (60)
#define STEER_HANDOVER      (50)

/* Derivative (D) damping for the drift correction. The error is -1 / 0 /
 * +1 (line under the left sensor / between / right). Each change of error
 * adds STEER_KD * (change) to a d_term that decays by D_DECAY per sample
 * (~30 ms). When the line comes back to the middle the d_term briefly
 * steers the other way, so the car stops turning before it overshoots;
 * when the line arrives under a sensor it adds a quick extra push.
 * (2026-10-19: the car zig-zagged on the straights.) Not used in arcs. */
#define STEER_KD            (40)
#define D_DECAY             (700)   /* permille kept per sample */

/* Every duty this module commands is scaled by this, permille: 750 =
 * everything 25 % slower (Buddy 5's on-track call, 2026-10-19). Includes
 * the arcs and the inner-wheel floor, so the floor drops to ~113: if the
 * inner wheel now stalls in turns (dead band, see TURN_INNER_MIN), raise
 * TURN_INNER_MIN rather than this. */
#define SPEED_SCALE         (750)
/* Extra power for the curve arcs only (both wheels), permille, on top of
 * SPEED_SCALE: 1100 = +10 % (Buddy 5's track call, 2026-10-19; the arcs
 * at 75 % bogged down). Drift corrections are not affected. */
#define ARC_POWER           (1100)

/* Curve turn = near spin on the spot (Buddy 5's track call, 2026-10-19
 * 20:00): forward speed TURN_SPEED permille of the straight speed (100 =
 * 10 %), wheels base +- TURN_SPIN in opposite directions. Before the
 * SPEED_SCALE: 35 +- 235 -> 270 / -200, motors ~202 / -150, both out of
 * the dead band, ~130 deg/s. Replaces the R = 140 mm arc (the ARC_*
 * values above are no longer used by drive_arc). */
#define TURN_SPEED          (100)
#define TURN_SPIN           (235)
/* The inner wheel never goes below this duty: between about -100 and
 * +100 it is in the motor dead band on the foam, the wheel locks and the
 * whole car stalls (2026-10-19: 450 / 50, 350 / -150 and 500 / -100 all
 * stalled, 500 / -100 for 25 s). */
#define TURN_INNER_MIN      (150)

/* Curves. One side for CURVE_AFTER samples (150 ms) commits the car to an
 * arc of the track's curve radius toward that side. The arc is HELD
 * through 0/0, because 0/0 is ambiguous: the line may be between the
 * sensors, or the car under-turned and the line slipped past the inner
 * sensor (2026-10-19 video: the car let the turn fade at the bottom-left
 * corner and drove off the mat). The curve ends when the OTHER sensor sees
 * the line (turned enough or a little past - it then corrects back), or
 * after CURVE_HOLD samples of 0/0.
 * Arc wheels for radius R, wheel spacing W: outer / inner = (R + W/2) /
 * (R - W/2), inner at TURN_INNER_MIN. ARC_SPEED_CAL_L/R correct for the
 * open-loop duty-to-speed curve of each motor (measured below). */
/* 150 ms (flickers allowed, CURVE_GAP) / 0.4 s: at 100 ms / 0.6 s ordinary drifts on a straight
 * committed arcs and the car zig-zagged left/right every 0.25 s
 * (2026-10-19 19:05 run). A drift recentres within ~150 ms; a 140 mm
 * corner keeps the line on one side longer. */
#define CURVE_AFTER         (8U)    /* 80 ms: 150 ms derailed on the tight loop (2026-10-19 20:09) */
#define CURVE_HOLD          (30U)
/* A 0/0 shorter than this (100 ms) does not reset the CURVE_AFTER count:
 * in a curve the sensor flickers on the line edge (2026-10-19 19:26:
 * a left curve never committed because every flicker reset the count). */
#define CURVE_GAP           (10U)
/* Escape: a one-sided reading of ESCAPE_MIN..ESCAPE_MAX samples that ends
 * in 0/0 means the line slid OUT past that sensor (a curve): a drift
 * correction cannot bring the line back to the middle that fast. On a
 * 140 mm curve the line crosses the sensor in ~35 mm (~0.13 s), before
 * CURVE_AFTER. (2026-10-19 19:31: the left sensor saw the left curve for
 * about one telemetry sample, then 0/0, and the car drove off the mat.)
 * The car then commits the arc toward that side, held for CURVE_HOLD. */
#define ESCAPE_MIN          (3U)
#define ESCAPE_MAX          (7U)    /* must stay below CURVE_AFTER */
/* ...and only if the line had sat between the sensors for this many
 * samples (250 ms) before that reading: a real curve starts from a
 * settled straight. Without this, every swing of a weave looked like an
 * escape, committed a full arc and swung the car back across (2026-10-19
 * 19:48: arcs 316/112 <-> 112/368 every 0.25 s). */
#define ESCAPE_SETTLED      (15U)   /* 150 ms */
/* Left steering is scaled up by this (permille): the left motor runs
 * faster at low duty, so an equal left correction turned less than a
 * right one (same ratio as ARC_SPEED_CAL_L / ARC_SPEED_CAL_R). */
#define STEER_TRIM_L        ((1000 * ARC_SPEED_CAL_L) / ARC_SPEED_CAL_R)
#define ARC_RADIUS_MM       (140)   /* Buddy 5's track measurement */
/* Per direction, permille: the motors differ at low duty. 2026-10-19
 * 19:05-19:14 arcs at 150 inner: right turns ran 275 / 130 mm/s (ratio
 * 2.1), left turns 275 / 160 (ratio 1.8, R ~190 mm: left turns missed
 * the curves). Scaled so both reach the 2.33 ratio of R = 140 mm. */
#define ARC_SPEED_CAL_R     (1210)  /* right turn: left motor is outer */
#define ARC_SPEED_CAL_L     (1410)  /* left turn: right motor is outer */
#define ARC_OUTER(cal)      ((TURN_INNER_MIN * (ARC_RADIUS_MM \
                              + (int32_t)RC_WHEEL_BASE_MM / 2) * (cal)) \
                             / ((ARC_RADIUS_MM - (int32_t)RC_WHEEL_BASE_MM / 2) * 1000))

/* Both sensors black: a crossing line (the middle of a figure of 8) or a
 * barcode bar clears in under ~100 ms and is crossed straight. A 1/1 that
 * lasts longer than CROSS_AFTER during a curve means the outer sensor ran
 * onto something dark and the inner one found the line: keep turning. */
#define CROSS_AFTER         (15U)

/* FR9 / FR12: how long the follower searches after declaring the line
 * LOST before it gives up, brakes and reports a failed recovery. 3 s is
 * FR9's value (placeholder - Buddy 5 to confirm). Counted in line
 * samples, one every RC_PERIOD_LINE_MS. Does not apply in SEARCHING: there
 * sub_nav owns the timeout (RECOVERING, after a bypass). */
#define LOST_SEARCH_MS      (3000U)
#define LOST_GIVE_UP        (LOST_THRESHOLD + \
                             (LOST_SEARCH_MS / RC_PERIOD_LINE_MS))

/* `static` at file scope means "only visible inside this .c file" */
/* current line-follow mode, see sub_line.h enum */
static rc_line_state_t state = RC_LINE_TRACKING;
/* is steering actually allowed right now? (see sub_line_enable) */
static bool     enabled;
/* true once a LOST search timed out: no output until re-enabled */
static bool     gave_up;
/* forward speed while tracking, in permille (parts per 1000 of full speed).
 * 350 (2026-10-19, slower for the track runs). At 350 the car stalled on
 * the ridge between two foam tiles (2026-10-18: 285 -> 3 mm/s), so the
 * joint must be taped; 400 if it still stalls. */
static int16_t  base_permille = 350;
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
 * an escape spin (line not seen yet) is held through 0/0. (2026-10-19
 * 20:04: spins held through 0/0 swung the line across to the other
 * sensor, left/right spins alternated every 0.25 s.) */
static bool     curve_seen;
/* most recent position reading, kept so we know which way to arc if the line vanishes */
static int16_t  last_position;
/* 0/0 samples before the current one-sided reading (see ESCAPE_SETTLED) */
static uint32_t pre_gap;
/* 0/0 samples in a row while really going straight: no curve held, no
 * correction fading. 0/0 inside a curve hold must not count as settled
 * (2026-10-19 19:53: right arc -> left arc -> right arc). */
static uint32_t settled;
/* derivative damping term and the previous error (see STEER_KD) */
static int16_t  d_term;
static int16_t  prev_err;

/* Every motor command from this module goes through here (SPEED_SCALE). */
static void line_drive(int32_t base, int32_t steer)
{
    (void)sub_motion_drive((int16_t)((base * SPEED_SCALE) / 1000),
                           (int16_t)((steer * SPEED_SCALE) / 1000));
}

#if !RC_LINE_STRADDLE
/* Turns a raw sideways position error into a motor steering command.*/
static void steer_from_position(int16_t position)
{

    int16_t steer = (int16_t)((int32_t)position * STEER_KP / STEER_DIV);

    line_drive(base_permille, steer);
}
#endif

/* Drive with the current steer_level, slowing as the turn tightens. */
static void drive_level(void)
{
    int32_t total = (int32_t)steer_level + d_term;
    int32_t mag   = (total < 0) ? (-total * STEER_TRIM_L) / 1000 : total;
    int32_t base  = (int32_t)base_permille - (mag / 2);

    if ((mag != 0) && ((base - mag) < TURN_INNER_MIN)) {
        base = TURN_INNER_MIN + mag;    /* keep the inner wheel turning */
    }
    line_drive(base, (total < 0) ? -mag : mag);
}

/* Drive the ARC_RADIUS_MM arc toward `dir` (+1 right, -1 left). */
static void drive_arc(int16_t dir)
{
    int32_t base  = ((int32_t)base_permille * TURN_SPEED) / 1000;
    int32_t steer = TURN_SPIN;

    line_drive(base, (dir < 0) ? -steer : steer);
}

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

/* A sensor sees the line again (1/0, 0/1 or 1/1). Report §6.5: "any
 * sensor seeing the line -> LINE_REACQUIRED", after a real loss (LOST) as
 * well as after a deliberate search (SEARCHING), and before the junction
 * check, because re-entering the line at an angle often reads 1/1. */
static void line_seen(void)
{
    lost_count = 0U;

    if ((state == RC_LINE_LOST) || (state == RC_LINE_SEARCHING)) {
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

    (void)ctx;

    if (!enabled || gave_up) {
        return;
    }

    {
        /* Derivative term (see STEER_KD): error -1 / 0 / +1. */
        int16_t err = 0;

        if (s->on_line_l != s->on_line_r) {
            err = (int16_t)((s->position < 0) ? -1 : 1);
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

        if (curve_dir != 0) {
            if (both_count > CROSS_AFTER) {
                /* Too long for a crossing line: the outer sensor is on
                 * something dark and the other one found the line. Curve
                 * over, steer toward the new sensor (2026-10-19: the car
                 * drove straight past the line here). */
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
        if ((one_count == 0U) || (s->position != one_position)) {
            pre_gap = settled;
        }
        settled = 0U;
        line_seen();
        both_count = 0U;
        state      = RC_LINE_TRACKING;     /* also leaves JUNCTION */
        last_position = s->position;

        if (curve_dir != 0) {
            if ((s->position < 0) == (curve_dir < 0)) {
                curve_gap  = 0U;            /* still on the curve side */
                curve_seen = true;
                drive_arc(curve_dir);
                return;
            }
            /* The other sensor found the line: turned enough (or a
             * little past). Curve over, correct back toward it. */
            curve_dir    = 0;
            steer_level  = (int16_t)((s->position < 0) ? -STEER_HANDOVER
                                                        : STEER_HANDOVER);
            one_position = s->position;
            one_count    = STEER_CONFIRM;
            drive_level();
            return;
        }

        if (s->position != one_position) {
            one_position = s->position;
            one_count    = 0U;
        }
        if (one_count < CURVE_AFTER) {
            one_count++;
        }
        if (one_count >= CURVE_AFTER) {
            curve_dir   = (int16_t)((s->position < 0) ? -1 : 1);
            curve_seen  = true;
            curve_gap   = 0U;
            steer_level = 0;
            drive_arc(curve_dir);
        } else if (one_count >= STEER_CONFIRM) {
            tighten_toward(s->position);
            drive_level();
        } else {
            /* Not confirmed yet: may be the first half of a 1/1. */
            line_drive(base_permille, 0);
        }

    } else {
        /* Neither sensor sees black.*/
        both_count = 0U;
        lost_count++;

        if (state == RC_LINE_SEARCHING) {
            /* Deliberately off-line. Keep whatever sub_nav commanded,
             * do not steer and do not declare it lost; sub_nav times
             * this search out itself (FR9). */
            return;
        }

        if (lost_count >= LOST_GIVE_UP) {
            /* FR9 / FR12: searched for LOST_SEARCH_MS without finding
             * the line. Brake and report it; stay silent until
             * sub_line_enable() or sub_line_begin_search() is called. */
            gave_up = true;
            (void)sub_motion_stop(true);
            publish_id(RC_EVT_RECOVERY_FAILED);
        } else if (lost_count >= LOST_THRESHOLD) {
            /* Officially lost now. */
            if (state != RC_LINE_LOST) {
                state = RC_LINE_LOST;
                publish_id(RC_EVT_LINE_LOST);
            }
            /* Arc toward the side the line was last seen on. */
            line_drive(base_permille / 2,
                                   (last_position < 0) ? -400 : 400);
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
            /* Brief dropout just keep steering if the
             * last known position still holds */
            steer_from_position(last_position);
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

/* Turns steering on/off. Also clears a timed-out search, so enabling
 * again starts afresh. */
rc_result_t sub_line_enable(bool on)
{
    enabled     = on;
    gave_up     = false;
    lost_count  = 0U;
    both_count  = 0U;
    one_count   = 0U;
    steer_level = 0;
    curve_dir   = 0;
    d_term      = 0;
    prev_err    = 0;
    return RC_OK;
}

/* Left the line on purpose so it looks for the
 * line again instead of immediately declaring it lost. */
rc_result_t sub_line_begin_search(void)
{
    state       = RC_LINE_SEARCHING;
    gave_up     = false;
    lost_count  = 0U;
    steer_level = 0;
    curve_dir   = 0;
    d_term      = 0;
    prev_err    = 0;
    return RC_OK;
}

/* Reports the current line-follow mode (tracking / lost / searching /
 * junction) so other modules can react. */
rc_line_state_t sub_line_state(void)
{
    return state;
}

/* Sets the forward cruising speed used while tracking, in permille*/
rc_result_t sub_line_set_base(int16_t permille)
{
    base_permille = permille;
    return RC_OK;
}
/*** end of file ***/
