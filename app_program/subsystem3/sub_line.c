/*
 *  sub_line.c
 */
#include "../rc_prelude.h"
#include "sub_line.h"
#include "../subsystem2/sub_motion.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* How many consecutive all-dark samples before we call the line lost.*/
#define LOST_THRESHOLD      (20U)

/* Both sensors on the line for this long means a junction or a barcode
 * lead-in, not a centred car. */
#define JUNCTION_THRESHOLD  (6U)

/* `static` at file scope means "only visible inside this .c file" */
/* current line-follow mode, see sub_line.h enum */
static rc_line_state_t state = RC_LINE_TRACKING;
/* is steering actually allowed right now? (see sub_line_enable) */
static bool     enabled;
/* forward speed while tracking, in permille (parts per 1000 of full speed) */
static int16_t  base_permille = 350;
/* consecutive samples with neither sensor on the line */
static uint32_t lost_count;
/* consecutive samples with both sensors on the line (junction candidate) */
static uint32_t both_count;
/* most recent position reading, kept so we know which way to arc if the line vanishes */
static int16_t  last_position;

#define STEER_KP        (2)

/* Turns a raw sideways position error into a motor steering command.*/
static void steer_from_position(int16_t position)
{

    int16_t steer = (int16_t)((int32_t)position * STEER_KP / 2);

    (void)sub_motion_drive(base_permille, steer);
}

/* ------------------------------------------------------------------ *
 *  Sample callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

static void on_sample(const rc_event_t *evt, void *ctx)
{
    const rc_pl_line_t *s = &evt->u.line;
    rc_event_t          out;

    (void)ctx;

    if (!enabled) {
        return;
    }

    if (s->on_line_l && s->on_line_r) {
        /* Both sensors see black at once; we require
         * JUNCTION_THRESHOLD consecutive ones first (debouncing). */
        both_count++;
        lost_count = 0U;

        if (both_count >= JUNCTION_THRESHOLD) {
            state = RC_LINE_JUNCTION;
        }
        /* Drive straight through. A junction is where the barcode
         * usually starts, so sub_barcode arms itself off this state. */
        (void)sub_motion_drive(base_permille, 0);

    } else if (s->on_line_l || s->on_line_r) {
        /* Exactly one sensor sees black: normal tracking case. */
        both_count = 0U;
        lost_count = 0U;

        if (state != RC_LINE_TRACKING) {
            if (state == RC_LINE_SEARCHING) {
                /* We were deliberately off-line and just found it again */
                out.id = RC_EVT_LINE_REACQUIRED;
                (void)rc_event_publish(&out);
            }
            state = RC_LINE_TRACKING;
        }
        last_position = s->position;
        steer_from_position(s->position);

    } else {
        /* Neither sensor sees black.*/
        both_count = 0U;
        lost_count++;

        if (state == RC_LINE_SEARCHING) {
            /* Deliberately off-line. Keep whatever sub_scan commanded,
             * do not steer and do not declare it lost. */
            return;
        }

        if (lost_count >= LOST_THRESHOLD) {
            /* Officially lost now. */
            if (state != RC_LINE_LOST) {
                state = RC_LINE_LOST;
                out.id = RC_EVT_LINE_LOST;
                (void)rc_event_publish(&out);
            }
            /* Arc toward the side the line was last seen on. */
            (void)sub_motion_drive(base_permille / 2,
                                   (last_position < 0) ? -400 : 400);
        } else {
            /* Brief dropout just keep steering if the
             * last known position still holds */
            steer_from_position(last_position);
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
    return RC_OK;
}

/* Turns steering on/off. */
rc_result_t sub_line_enable(bool on)
{
    enabled    = on;
    lost_count = 0U;
    both_count = 0U;
    return RC_OK;
}

/* Left the line on purpose so it looks for the
 * line again instead of immediately declaring it lost. */
rc_result_t sub_line_begin_search(void)
{
    state      = RC_LINE_SEARCHING;
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
