/*
 *  sub_scan.h  -  Buddy 5
 *
 *  The five stages from the brief, run as one non-blocking state
 *  machine: coarse scan, fine scan, profile, plan, recover.
 *
 *  The whole thing is driven by two events, the servo settle timeout and
 *  the ultrasonic result. Nothing polls and nothing waits. A full coarse
 *  scan at five angles takes roughly 5 x (servo travel + 30 ms of flight
 *  time), around 700 ms, during which the CPU is essentially idle and
 *  the motion PID keeps running at its normal rate.
 */
/* Include guard — see sub_telemetry.h for why this pattern is needed. */
#ifndef SUB_SCAN_H
#define SUB_SCAN_H

#include "rc_types.h"

/* Function pointer type (see sub_telemetry.h's sub_telemetry_cmd_cb_t
 * comment for the general idea) for "a function that gets called once a
 * scan finishes." `profile` describes what was found (closest obstacle,
 * its width, clearance on each side); `plan` is the decided response
 * (go straight / turn / stop, and by how much). Both are passed as
 * `const` pointers — the callback may read them but must not modify the
 * scan module's own copies. */
typedef void (*sub_scan_done_cb_t)(const rc_pl_profile_t *profile,
                                   const rc_pl_plan_t *plan,
                                   void *ctx);

/* Creates the background scan task and its event flag, and subscribes to
 * ultrasonic results. Call once at boot. */
rc_result_t sub_scan_init(void);

/*
 *  Start a coarse scan. Returns immediately. The result arrives through
 *  the callback and through RC_EVT_OBSTACLE_PROFILE and
 *  RC_EVT_AVOIDANCE_PLAN.
 */
rc_result_t sub_scan_start(void);

/* Cancels a scan currently in progress and returns the state machine to
 * idle. */
rc_result_t sub_scan_abort(void);
/* True while a coarse or fine scan is actively running (not just the
 * slow forward watch — see sub_scan_set_watch below). */
bool sub_scan_busy(void);

/* Registers a function to call once a scan completes, in addition to the
 * two events always published (RC_EVT_OBSTACLE_PROFILE and
 * RC_EVT_AVOIDANCE_PLAN, see sub_scan_start's comment). Pass NULL for
 * `cb` to clear it. */
rc_result_t sub_scan_on_complete(sub_scan_done_cb_t cb, void *ctx);

/*
 *  Continuous forward watch. Pings straight ahead at a slow rate while
 *  the car is line following, and publishes RC_EVT_ULTRA_RESULT. sub_nav
 *  watches those and calls sub_scan_start when something gets close.
 */
rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm);

#endif /* SUB_SCAN_H */
