/*
 *  sub_line.h  -  Buddy 3, line following
 *
 *  Subscribes to RC_EVT_LINE_SAMPLE and turns it into steering commands
 *  for sub_motion. Owns the "line lost" and "line reacquired" decisions
 *  (RC_EVT_LINE_LOST, RC_EVT_LINE_REACQUIRED, RC_EVT_RECOVERY_FAILED); a
 *  lost line is searched for with line_scan.h.
 */
#ifndef SUB_LINE_H
#define SUB_LINE_H

#include "../rc_types.h"

typedef enum {
    /* normal case: at least one sensor sees the line, steering actively */
    RC_LINE_TRACKING = 0,
    /* neither sensor has seen the line for LOST_THRESHOLD samples in a row */
    RC_LINE_LOST,
    /* both sensors on black for JUNCTION_THRESHOLD samples: likely a barcode
     * lead-in */
    RC_LINE_JUNCTION
} rc_line_state_t;

/* Call once at boot, before enabling. */
rc_result_t sub_line_init(void);

/* Start or stop acting on samples. While disabled no sample is processed
 * and no steering is issued (sub_nav disables it while avoiding). */
rc_result_t sub_line_enable(bool on);

/* sub_barcode: a barcode read started (true) or ended (false). While
 * reading the line is never declared LOST; steering is unchanged. */
rc_result_t sub_line_set_reading(bool on);

/* true while sub_barcode is reading a barcode (see sub_line_set_reading):
 * its white spaces read 0/0, which is not a junction or a lost line. */
bool sub_line_reading(void);

#endif /* SUB_LINE_H */
/*** end of file ***/