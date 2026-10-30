/*
 *  sub_line.c
 */
#include "../rc_prelude.h"
#include "sub_line.h"
#include "../subsystem2/sub_motion.h"
#include "../subsystem2/drv_encoder.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* 1 = the line is NARROWER than the gap between the L and R sensors (the
 * track's 18 mm line, 2026-10-18: over it only one LED ever goes out). A
 * centred car then reads 0/0 with the line between the sensors, so 0/0
 * only means "lost" once it has lasted LOST_THRESHOLD samples, and each
 * correction is gentler so the car doesn't swing across the line.
 * 0 = line wider than the gap: 1/1 is centred, 0/0 is soon lost.
 * 2026-10-29: still 1. Buddy 5 moved the IR pair in to JUST OUTSIDE the
 * line edges (minimal leeway, to stop the jitter): centred = 0/0, drift
 * = 1/0 or 0/1, a crossing line / junction = 1/1. (The 14:54 run read 1/1
 * on the straights with the pair set too close; mode 0 was tried for
 * that and is kept below.) */
#ifndef RC_LINE_STRADDLE
#define RC_LINE_STRADDLE    (1)
#endif

#if RC_LINE_STRADDLE
/* 2 s of 0/0 before we call the line lost and start the search (Buddy
 * 5's track call, 2026-10-29: 10 s, then 5 s, then 3 s, left the car too
 * far past the line end before searching). 200 samples of
 * RC_PERIOD_LINE_MS, ~60 mm at line speed. */
#define LOST_THRESHOLD      (200U)
/* Gentle correction, +-30 permille: wheels 380 / 320 at base 350, about
 * 46 mm/s wheel difference = 0.4 rad/s, so ~2-5 degrees over the 0.1 to
 * 0.23 s a correction lasts (Buddy 5's on-track target: ~5 degrees).
 * +-125 and +-75 were too steep for small drifts (2026-10-19). A longer
 * one-sided reading keeps tightening the turn (STEER_STEP). */
#define STEER_KP            (3)
#define STEER_DIV           (50)
#else
/* Samples of 0/0 (off the line) before we call the line lost: 2 s, Buddy
 * 5's call (2026-10-29; was 3 s). Until then the car arcs back toward the side the
 * line was last seen on (see on_sample), so this is the time it gets to
 * find the line again before LOST. */
#define LOST_THRESHOLD      (200U)
/* Same gentle first correction as straddle mode (+-30 permille, then
 * STEER_STEP): the tuned strength, now at the closed-loop speed. */
#define STEER_KP            (3)
#define STEER_DIV           (50)
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
/* 160 ms (2026-10-29: doubled from 80 ms with the speed halved, Buddy 5's
 * track call; 150 ms derailed on the tight loop at base 350 on 2026-10-19
 * 20:09, at twice today's speed). */
#define CURVE_AFTER         (16U)
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
/* Doubled with CURVE_AFTER (2026-10-29): the same distance takes twice
 * as long at half speed. */
#define ESCAPE_MIN          (6U)
#define ESCAPE_MAX          (14U)   /* must stay below CURVE_AFTER */
/* ...and only if the line had sat between the sensors for this many
 * samples (250 ms) before that reading: a real curve starts from a
 * settled straight. Without this, every swing of a weave looked like an
 * escape, committed a full arc and swung the car back across (2026-10-19
 * 19:48: arcs 316/112 <-> 112/368 every 0.25 s). */
#define ESCAPE_SETTLED      (15U)   /* 150 ms */
/* 1 = drive on wheel SPEED (sub_motion_drive_speed, its PID finds the
 * duty), 0 = open-loop duty (sub_motion_drive). 2026-10-29, Buddy 5's
 * track call: everything 50 % slower. Open-loop half power is ~70 / 61
 * duty, inside the ~+-100 dead band on the foam (the wheels would not
 * turn); on speed the PID holds ~30 mm/s (it held ~35 mm/s at ~110 / 89
 * duty on 2026-10-26). Every module value below stays in permille and is
 * converted once, in line_drive(), so all corrections and turns shrink
 * with the speed and keep their shape (same steering per mm travelled). */
#define LINE_CLOSED_LOOP    (1)
/* Wheel speed for one permille of command: ~63 mm/s was measured at 175
 * open-loop (2026-10-26), times LINE_SPEED_PCT. */
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
#define ARC_RADIUS_MM       (140)   /* Buddy 5's track measurement */
/* The arc the wheels are TOLD to drive in a curve (closed loop), tighter
 * than the track (2026-10-29, Buddy 5: "increase turn angle"; the 15:42
 * run held 0/1 for 8 s in a curve, then lost the line):
 *  - the sensors are ~80 mm ahead of the wheels, so for them to follow the
 *    140 mm curve the wheels must drive sqrt(140^2 - 80^2) ~ 115 mm;
 *  - asked for 140 mm the wheels drove ~165 mm (45 / 22 mm/s, ratio 2.0
 *    vs 2.33): sub_motion's PID never brakes and its feedforward puts the
 *    inner wheel at ~22 mm/s when ~19 is wanted.
 * 100 mm (ratio 3.5, ~48 / 14 mm/s) should land near 115 mm. Still ran
 * wide on the track (2026-10-29 15:5x): +20 % turn (curvature), 100 / 1.2
 * = 83 mm (ratio 5.1, ~52 / 10 mm/s target). Another +10 %: 83 / 1.1
 * = 75 mm (ratio 6.9, ~55 / 8 mm/s target). 71 mm (~7 mm/s inner) was
 * tried and stalled the inner wheel (0-4 mm/s, 17:03): the car pivoted on
 * it, so back to 75. */
#define ARC_TURN_RADIUS_MM  (75)
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

/* FR9 / FR12 line search (2026-10-29, Buddy 5; replaces a 3 s arc toward
 * the last side). Once LOST the car spins in place SEARCH_SWEEP_DEG left,
 * then back right to SEARCH_SWEEP_DEG right of where it started, at
 * SEARCH_SPIN_MM_S per wheel (closed loop). The first sample with either
 * sensor on the line ends the search: the spin stops, the car drives
 * forward toward the line and the normal correction takes over. Line not
 * found by the end of the right sweep: brake and report RECOVERY_FAILED.
 * The angle comes from the encoder clicks (both wheels, any direction):
 * a wheel travels (W/2) * pi / 180 = 977 um per degree of spin. Does not
 * apply in SEARCHING: there sub_nav owns the search (RECOVERING, after a
 * bypass). */
#define SEARCH_SWEEP_DEG    (135)   /* 2026-10-29: 90 -> 135 (Buddy 5) */
/* sub_line_find() passes its own limit: after a junction turn the branch
 * line is close, so sub_nav sweeps only 35 degrees (2026-10-29, Buddy 5). */
/* Before the sweep: a slight turn ON THE MOVE toward the side the line
 * was last seen on (last_position), the likely side, to check there first
 * (2026-10-29, Buddy 5): line speed with a SEARCH_NUDGE_STEER correction
 * until the heading has changed SEARCH_NUDGE_DEG (~0.9 s, ~28 mm).
 * Skipped if no side was ever seen. The sweep limits stay measured from
 * the heading at LOST. */
#define SEARCH_NUDGE_DEG    (10)    /* 2026-10-29: 20 -> 10 (~1.8 s -> ~0.9 s) */
#define SEARCH_NUDGE_STEER  (60)    /* permille, = STEER_MAX */
/* 2026-10-29: 40 -> 70 (Buddy 5: sweep faster). Sensor tip ~125 mm/s:
 * an 18 mm line stays under it ~14 samples; 135 degrees in ~1.9 s. */
#define SEARCH_SPIN_MM_S    (70)
#define SEARCH_UM_PER_DEG   ((uint32_t)RC_WHEEL_BASE_MM * 1000UL * 31416UL \
                             / (2UL * 180UL * 10000UL))

/* `static` at file scope means "only visible inside this .c file" */
/* current line-follow mode, see sub_line.h enum */
static rc_line_state_t state = RC_LINE_TRACKING;
/* is steering actually allowed right now? (see sub_line_enable) */
static bool     enabled;
/* true once a LOST search timed out: no output until re-enabled */
static bool     gave_up;
/* forward speed while tracking, in permille (parts per 1000 of full speed).
 *  At 350 the car stalled on
 * the ridge between two foam tiles, so the
 * joint must be taped; 400 if it still stalls. */
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
/* sub_barcode is reading a barcode: the line is not declared LOST (see
 * sub_line_set_reading) */
static bool     reading;
/* line search after LOST (see SEARCH_SWEEP_DEG): which leg is running,
 * both wheels' encoder clicks when it started, the leg's direction (+1
 * right, -1 left) and the heading it started from (degrees from the
 * heading at LOST, < 0 = left) */
typedef enum { SWEEP_NONE = 0, SWEEP_NUDGE, SWEEP_LEFT, SWEEP_RIGHT } sweep_t;
static sweep_t  sweep;
static uint32_t sweep_clicks_l0;
static uint32_t sweep_clicks_r0;
static int32_t  sweep_dir;
static int32_t  sweep_from_deg;
/* this search's sweep limit each side, degrees (SEARCH_SWEEP_DEG, or the
 * one sub_line_find() was given) */
static int32_t  sweep_limit = SEARCH_SWEEP_DEG;

/* Left/right motor match, permille of base added to the left wheel and
 * taken off the right. 2026-10-26: at 131/131 the left wheel ran ~15 %
 * slower (stand 83 vs 97 mm/s, mat 60 vs 75; encoders L/R 0.84), so the
 * car twisted left on straights. 70 -> left/right duty 1.07 / 0.93.
 * 0 once the left wheel is fixed mechanically. Open loop only: on speed
 * the PID matches the wheels, and a trim would curve the car. */
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
/* Drive the ARC_RADIUS_MM arc toward `dir` (+1 right, -1 left) at the
 * line speed (2026-10-29, Buddy 5: keep the 140 mm radius). For wheel
 * spacing W the wheels run at (R +- W/2) / R of the centre speed, 1.4 /
 * 0.6 for R = 140 mm, W = 112 mm; on speed the PID holds that ratio, so
 * no inner-wheel floor is needed (the open-loop near spin below was the
 * answer to the dead band). */
static void drive_arc(int16_t dir)
{
    int32_t base  = base_permille;
    int32_t steer = (base * ((int32_t)RC_WHEEL_BASE_MM / 2)) / ARC_TURN_RADIUS_MM;

    line_drive(base, (dir < 0) ? -steer : steer);
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

/* Heading now, degrees from the heading at LOST (< 0 = left). Clicks
 * count up whichever way a wheel turns, so the leg's direction gives the
 * sign. In place (LEFT / RIGHT legs) the heading change is the MEAN wheel
 * travel over SEARCH_UM_PER_DEG; on the move (NUDGE, both wheels forward)
 * it is HALF the outer-minus-inner travel over the same. */
static int32_t sweep_heading(void)
{
    uint32_t dl = drv_encoder_count(RC_SIDE_LEFT) - sweep_clicks_l0;
    uint32_t dr = drv_encoder_count(RC_SIDE_RIGHT) - sweep_clicks_r0;
    uint32_t half;

    if (sweep == SWEEP_NUDGE) {
        uint32_t outer = (sweep_dir > 0) ? dl : dr;
        uint32_t inner = (sweep_dir > 0) ? dr : dl;

        half = (outer > inner) ? ((outer - inner) / 2U) : 0U;
    } else {
        half = (dl + dr) / 2U;
    }
    return sweep_from_deg
           + (sweep_dir * (int32_t)((half * RC_ENC_UM_PER_TICK)
                                    / SEARCH_UM_PER_DEG));
}

/* Starts one leg of the line search from the current heading. */
static void sweep_begin(sweep_t leg, int32_t dir)
{
    int32_t from = (sweep == SWEEP_NONE) ? 0 : sweep_heading();

    sweep           = leg;
    sweep_dir       = dir;
    sweep_from_deg  = from;
    sweep_clicks_l0 = drv_encoder_count(RC_SIDE_LEFT);
    sweep_clicks_r0 = drv_encoder_count(RC_SIDE_RIGHT);
}

/* Starts the search at LOST: the nudge toward the last-seen side, or
 * straight to the left sweep if no side was ever seen. */
static void search_begin(void)
{
    sweep = SWEEP_NONE;
    if (last_position != 0) {
        sweep_begin(SWEEP_NUDGE, (last_position < 0) ? -1 : 1);
    } else {
        sweep_begin(SWEEP_LEFT, -1);
    }
}

/* One 0/0 sample while LOST: spin the current leg; nudge done -> sweep
 * left to -sweep_limit -> sweep right to +sweep_limit -> give up. */
static void search_step(void)
{
    int32_t h;

    if (sweep == SWEEP_NONE) {
        search_begin();             /* LOST carried over a re-enable */
    }
    h = sweep_heading();
    if ((sweep == SWEEP_NUDGE)
        && (((sweep_dir < 0) ? -h : h) >= SEARCH_NUDGE_DEG)) {
        sweep_begin(SWEEP_LEFT, -1);
        h = sweep_heading();
    }
    if ((sweep == SWEEP_LEFT) && (h <= -sweep_limit)) {
        sweep_begin(SWEEP_RIGHT, 1);
        h = sweep_heading();
    }
    if ((sweep == SWEEP_RIGHT) && (h >= sweep_limit)) {
        /* FR9 / FR12: not found either side. Brake and report it; stay
         * silent until sub_line_enable() or sub_line_begin_search(). */
        sweep   = SWEEP_NONE;
        gave_up = true;
        (void)sub_motion_stop(true);
        publish_id(RC_EVT_RECOVERY_FAILED);
        return;
    }
    if (sweep == SWEEP_NUDGE) {
        line_drive(base_permille, sweep_dir * SEARCH_NUDGE_STEER);
    } else {
        (void)sub_motion_drive_speed(0, (int16_t)(sweep_dir * SEARCH_SPIN_MM_S));
    }
}

/* A sensor sees the line again (1/0, 0/1 or 1/1). Report §6.5: "any
 * sensor seeing the line -> LINE_REACQUIRED", after a real loss (LOST) as
 * well as after a deliberate search (SEARCHING), and before the junction
 * check, because re-entering the line at an angle often reads 1/1. */
static void line_seen(void)
{
    lost_count = 0U;
    sweep      = SWEEP_NONE;    /* a search spin ends here */

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

        if (reading && (lost_count >= LOST_THRESHOLD)) {
            /* Reading a barcode: the white spaces between the bars are
             * not a lost line. Hold the count just short of LOST, so the
             * steering below carries on as usual; the line is declared
             * lost only if it is still missing once the read ends. */
            lost_count = LOST_THRESHOLD - 1U;
        }

        if (lost_count >= LOST_THRESHOLD) {
            /* Officially lost now: a slight turn on the move toward the
             * last-seen side, then sweep left, then right in place (see
             * SEARCH_NUDGE_DEG, SEARCH_SWEEP_DEG). */
            if (state != RC_LINE_LOST) {
                state       = RC_LINE_LOST;
                curve_dir   = 0;
                steer_level = 0;
                d_term      = 0;
                publish_id(RC_EVT_LINE_LOST);
                sweep_limit = SEARCH_SWEEP_DEG;
                search_begin();
            }
            search_step();
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
             * ARC_RADIUS_MM arc back toward that side. While a barcode is
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

/* Turns steering on/off. Also clears a timed-out search, so enabling
 * again starts afresh. */
rc_result_t sub_line_enable(bool on)
{
    enabled     = on;
    gave_up     = false;
    sweep       = SWEEP_NONE;
    sweep_limit = SEARCH_SWEEP_DEG;
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

/* sub_barcode calls this when a read starts (on) and ends (off). Only
 * the LOST declaration is held off; steering is unchanged. */
rc_result_t sub_line_set_reading(bool on)
{
    if (!on && reading && (lost_count >= (LOST_THRESHOLD - 1U))) {
        /* The read held the count just short of LOST; start the 0/0
         * count afresh, or the next 0/0 (a centred car) reads as lost
         * at once (2026-10-26 20:57/20:58: LOST then REACQUIRED). */
        lost_count = 0U;
    }
    reading = on;
    return RC_OK;
}

bool sub_line_reading(void)
{
    return reading;
}

/* sub_nav: find the line now, with the in-place sweep (left then right,
 * no nudge: there is no last-seen side yet). For the start of a run and
 * after a junction turn (2026-10-29, Buddy 5). The first sample with a
 * sensor on the line ends it, as after LOST; not found: RECOVERY_FAILED.
 * sweep_deg: how far each side (SEARCH_SWEEP_DEG for a full search).
 * Call after sub_line_enable(true). */
rc_result_t sub_line_find(int16_t sweep_deg)
{
    sweep_limit   = (sweep_deg > 0) ? sweep_deg : SEARCH_SWEEP_DEG;
    state         = RC_LINE_LOST;   /* no LINE_LOST event: not a loss */
    lost_count    = LOST_THRESHOLD;
    last_position = 0;
    curve_dir     = 0;
    steer_level   = 0;
    d_term        = 0;
    search_begin();
    return RC_OK;
}

/*** end of file ***/
