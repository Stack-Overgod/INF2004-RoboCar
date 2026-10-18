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
/* 1 s of 0/0 (about 28 cm at cruise) before we call the line lost. */
#define LOST_THRESHOLD      (200U)
/* +-125 permille: the inner wheel slows to 225, the outer speeds to 475.
 * +-250 turned the car ~30 degrees in one 0.26 s correction on the
 * track (2026-10-18), swinging the sensors right across the line. */
#define STEER_KP            (1)
#define STEER_DIV           (4)
#else
/* How many consecutive all-dark samples before we call the line lost.*/
#define LOST_THRESHOLD      (20U)
#define STEER_KP            (2)
#define STEER_DIV           (2)
#endif

/* Both sensors on the line for this long means a junction or a barcode
 * lead-in, not a centred car. */
#define JUNCTION_THRESHOLD  (6U)

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
 * 400, not 350: at 350 the car stalled on the ridge between two foam
 * track tiles (2026-10-18 run log: 285 -> 3 mm/s at constant duty). */
static int16_t  base_permille = 400;
/* consecutive samples with neither sensor on the line */
static uint32_t lost_count;
/* consecutive samples with both sensors on the line (junction candidate) */
static uint32_t both_count;
/* most recent position reading, kept so we know which way to arc if the line vanishes */
static int16_t  last_position;

/* Turns a raw sideways position error into a motor steering command.*/
static void steer_from_position(int16_t position)
{

    int16_t steer = (int16_t)((int32_t)position * STEER_KP / STEER_DIV);

    (void)sub_motion_drive(base_permille, steer);
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

    if (s->on_line_l && s->on_line_r) {
        /* Both sensors see black at once; we require
         * JUNCTION_THRESHOLD consecutive ones first (debouncing). */
        line_seen();
        both_count++;

        if (both_count >= JUNCTION_THRESHOLD) {
            state = RC_LINE_JUNCTION;
        }
        /* Drive straight through. */
        (void)sub_motion_drive(base_permille, 0);

    } else if (s->on_line_l || s->on_line_r) {
        /* Exactly one sensor sees black: normal tracking case. */
        line_seen();
        both_count = 0U;
        state      = RC_LINE_TRACKING;     /* also leaves JUNCTION */
        last_position = s->position;
        steer_from_position(s->position);

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
            (void)sub_motion_drive(base_permille / 2,
                                   (last_position < 0) ? -400 : 400);
        } else {
#if RC_LINE_STRADDLE
            /* 0/0, not lost yet: the line is between the sensors, so
             * the car is on target - drive straight. */
            (void)sub_motion_drive(base_permille, 0);
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
    enabled    = on;
    gave_up    = false;
    lost_count = 0U;
    both_count = 0U;
    return RC_OK;
}

/* Left the line on purpose so it looks for the
 * line again instead of immediately declaring it lost. */
rc_result_t sub_line_begin_search(void)
{
    state      = RC_LINE_SEARCHING;
    gave_up    = false;
    lost_count = 0U;
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
