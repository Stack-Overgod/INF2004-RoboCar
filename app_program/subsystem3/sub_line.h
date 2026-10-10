/*
 *  sub_line.h  -  Buddy 3, line following
 *
 *  Subscribes to RC_EVT_LINE_SAMPLE and turns it into steering commands
 *  for sub_motion. Owns the "line lost" and "line reacquired" decisions,
 *  which the obstacle avoidance path in sub_scan depends on.
 */
#ifndef SUB_LINE_H
#define SUB_LINE_H

#include "../rc_types.h"

typedef enum {
    /* normal case: at least one sensor sees the line, steering actively */
    RC_LINE_TRACKING = 0,
    /* neither sensor has seen the line for LOST_THRESHOLD samples in a row */
    RC_LINE_LOST,
    /* deliberately off-line, e.g. going round an obstacle */
    RC_LINE_SEARCHING,
    /* both sensors on black for JUNCTION_THRESHOLD samples: likely a barcode
     * lead-in */
    RC_LINE_JUNCTION
} rc_line_state_t;

/* Call once at boot, before enabling. */
rc_result_t sub_line_init(void);

/* Start or stop acting on samples. While disabled the subsystem still
 * tracks state but issues no steering, which is what sub_scan wants
 * while it is driving the bypass. */
rc_result_t sub_line_enable(bool on);

/* Tell the follower we are deliberately off the line and it should look
 * for it rather than report it lost. Used during obstacle bypass. */
rc_result_t sub_line_begin_search(void);

rc_line_state_t sub_line_state(void);

/* Base speed in permille used while tracking. */
rc_result_t sub_line_set_base(int16_t permille);

#endif /* SUB_LINE_H */
/*** end of file ***/