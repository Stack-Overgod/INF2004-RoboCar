/*
 *  sub_nav.h  -  mission state machine (shared; integration owner Buddy 5)
 *
 *  The one module that is nobody's subsystem and everybody's problem: it
 *  owns the top-level state of the run (report §6.5) and decides who is
 *  driving at any moment. It holds no hardware and runs no task of its
 *  own. Every transition happens inside an event handler on the FAST
 *  dispatcher, or inside a sub_motion completion callback, which runs on
 *  that same dispatcher - so the state is only ever changed by one task.
 *
 *  SKELETON. Implemented now: IDLE -> FOLLOWING, the obstacle path
 *  (watch ping -> AVOIDING -> scan -> plan -> bypass -> RECOVERING ->
 *  FOLLOWING), and STOPPED (remote STOP, IMPACT, aborted move, no safe
 *  route). Stubbed until the other buddies' code is wired:
 *  READING_BARCODE, EXECUTING_CMD and the FR9 recovery timeout. See the
 *  TODOs in sub_nav.c.
 */
#ifndef SUB_NAV_H
#define SUB_NAV_H

#include "rc_types.h"

typedef enum {
    RC_NAV_IDLE = 0,            /* start-up; every event is ignored      */
    RC_NAV_FOLLOWING,           /* line follow + forward watch           */
    RC_NAV_READING_BARCODE,     /* stub                                  */
    RC_NAV_EXECUTING_CMD,       /* stub                                  */
    RC_NAV_AVOIDING,            /* scanning, then driving the bypass     */
    RC_NAV_RECOVERING,          /* bypass done, searching for the line   */
    RC_NAV_STOPPED              /* motors braked; sub_nav_start() leaves */
} rc_nav_state_t;

/* Subscribe to the events below. Call once from usermain(), after every
 * subsystemN_init() (the handlers call sub_motion, sub_line, sub_barcode
 * and sub_scan). Leaves the state machine in IDLE. */
rc_result_t sub_nav_init(void);

/* Begin the run: IDLE or STOPPED -> FOLLOWING. Call from usermain() once
 * every subsystem has started (report §6.5, IDLE). Returns RC_ERR_STATE
 * from any other state. */
rc_result_t sub_nav_start(void);

/* Stop the run from any task. Goes through the event bus as a STOP
 * command, so the transition still runs on the FAST dispatcher. */
rc_result_t sub_nav_stop(void);

rc_nav_state_t sub_nav_state(void);
const char    *sub_nav_state_name(rc_nav_state_t s);

/* Inject a command as if it had arrived over Buddy 1's link: publishes
 * RC_EVT_COMMAND_RX. `arg` is reserved; pass 0. Safe from any task. */
rc_result_t sub_nav_inject_command(rc_nav_cmd_t cmd, int32_t arg);

#endif /* SUB_NAV_H */
