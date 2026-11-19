/*
 *  sub_scan.h  -  obstacle scan and avoidance planning
 *
 *  A scan task runs the step scan (servo and chassis measure an
 *  obstacle), builds the obstacle profile and the avoidance plan, and
 *  runs the forward watch ping. Every wait is a kernel sleep (servo
 *  settle, chassis step, ultrasonic result), so the CPU is essentially
 *  idle during a scan.
 */
#ifndef SUB_SCAN_H
#define SUB_SCAN_H

#include "../rc_types.h"

/* Creates the scan task, its event flag and request semaphore, and
 * registers for ultrasonic results. Call once at boot. */
rc_result_t sub_scan_init(void);

/* Cancels a scan currently in progress and returns the state machine to
 * idle. */
rc_result_t sub_scan_abort(void);

/* Starts the step scan. Returns immediately (RC_ERR_BUSY if a scan is
 * running); the result arrives as RC_EVT_OBSTACLE_PROFILE and
 * RC_EVT_AVOIDANCE_PLAN. The servo swings fully out and the chassis
 * steps, stopped for each ping, until the beam leaves the
 * obstacle on each side; length from the two corners (0 if one was not
 * found). The profile's clearance_left/right_mm are then each edge's
 * distance from the line (0 = edge not found). Also publishes
 * RC_EVT_SPIN_EDGES. The motors must be free; they are stopped at the
 * end. */
rc_result_t sub_scan_start_measure(void);

/* True while a scan is running (not the forward watch, see
 * sub_scan_set_watch below). */
bool sub_scan_busy(void);

/*
 *  Continuous forward watch. Pings straight ahead, RC_PERIOD_SCAN_MS
 *  between pings, each result published as RC_EVT_ULTRA_RESULT.
 *  sub_nav watches those and starts avoidance when something gets close.
 *  `trigger_mm` is accepted but not used by this module.
 */
rc_result_t sub_scan_set_watch(bool on, uint16_t trigger_mm);

#endif /* SUB_SCAN_H */
