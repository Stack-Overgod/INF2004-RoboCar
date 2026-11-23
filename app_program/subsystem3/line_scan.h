/*
 *  line_scan.h  -  Buddy 3, finding the line by spinning in place
 *
 *  The one line search every caller uses: sub_nav at START (FR16), after
 *  an obstacle bypass (FR10) and after a junction turn (FR6), and sub_line
 *  when the line is lost (FR4). The car spins in place while both line
 *  sensors are watched; a line counts as found only when both sensors have
 *  crossed the SAME line (the leading sensor first, the other within
 *  LS_PAIR_DEG of heading after it), and the car then turns to the middle
 *  of the four crossing headings. The caller chooses the angles and the
 *  speed (line_scan_cfg_t) and what to do when nothing is found.
 *
 *  Runs on the FAST lane: its own RC_EVT_LINE_SAMPLE subscriber drives
 *  it, and the done callback is called from that lane (from the sample
 *  handler or from sub_motion's completion callback), like sub_motion's
 *  move callbacks. While a scan runs, sub_line does not steer.
 */
#ifndef LINE_SCAN_H
#define LINE_SCAN_H

#include "../rc_types.h"

/* Angles in degrees, + = left. */
typedef struct {
    int16_t first_dir;  /* +1 left / -1 right: direction of the first sweep */
    int16_t quick_deg;  /* > 0: if exactly one sensor is on black when the
                         * scan starts, first sweep this far toward it */
    int16_t first_deg;  /* first main sweep ends this far in first_dir */
    int16_t back_deg;   /* the second sweep ends this far on the other side
                         * of the heading the scan started at */
    int16_t speed_mm_s; /* wheel speed while spinning (closed loop) */
} line_scan_cfg_t;

/* found: true = the car now faces along the line (stopped);
 * false = no line within the sweeps, or the scan timed out (stopped). */
typedef void (*line_scan_done_cb_t)(bool found, void *ctx);

/* Subscribe to the line samples. Call once at boot. */
rc_result_t line_scan_init(void);

/* Start a scan from the current heading. The first line sample decides
 * the quick sweep. `done` is called exactly once, unless the scan is
 * aborted. RC_ERR_STATE if a scan is already running. */
rc_result_t line_scan_start(const line_scan_cfg_t *cfg,
                            line_scan_done_cb_t done, void *ctx);

/* Stop a running scan and the motors, without calling `done`. Nothing
 * happens if no scan is running. */
void line_scan_abort(void);

/* True while a scan runs (from line_scan_start until `done` or abort). */
bool line_scan_busy(void);

#endif /* LINE_SCAN_H */
/*** end of file ***/
