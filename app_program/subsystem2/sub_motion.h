/*
 *  sub_motion.h  -  Buddy 2
 *
 *  The motion API the rest of the team calls. The brief asks for
 *  moveForward(distance), moveBackward(distance), turnLeft(angle),
 *  turnRight(angle) and stop(); here they are sub_motion_forward_mm(),
 *  sub_motion_backward_mm(), sub_motion_turn_deg(-angle / +angle) and
 *  sub_motion_stop(). The obvious implementation of those blocks until
 *  the move finishes, and that cannot work here: the line follower needs
 *  to keep steering while the car drives, and the scanner needs to keep
 *  pinging. See TEAM_GUIDE.md §1.3 ("Non-blocking - the one rule that
 *  shapes everything") for why this pattern is used everywhere in this
 *  project, not just here.
 *
 *  So every command here is a request that returns immediately and
 *  reports completion through a callback (a "callback" is just a
 *  function pointer you hand over in advance, which the callee invokes
 *  later when the thing you asked for is actually done - like leaving
 *  your phone number so a restaurant can text you when your table is
 *  ready, instead of you standing at the counter waiting). One move is
 *  active at a time; a new command pre-empts the old one and the old
 *  one's callback fires with completed = false.
 *
 *      uint32_t id = sub_motion_forward_mm(300, on_done, NULL);
 *      // returns now, car is still moving
 *
 *  Underneath, a control task runs every RC_PERIOD_MOTION_MS. Each wheel
 *  has its own PID speed loop with feedforward; a move ramps up, cruises
 *  and slows down near its goal; and a straight-line correction keeps the
 *  two wheels' distances equal, so the car holds its heading. A move that
 *  cannot make progress - a blocked wheel, an unplugged encoder, or one
 *  that takes far too long - is stopped and reported with
 *  completed = false instead of driving blind.
 *
 *  "PID" = Proportional-Integral-Derivative control, the same idea as a
 *  car's cruise control - see sub_motion.c for the full explanation next
 *  to the actual math, and the Buddy 2 guide (docs/buddy2-motion/) for
 *  the plain-English overview.
 */
#ifndef SUB_MOTION_H
#define SUB_MOTION_H

#include "rc_types.h"

/* The shape of a "move finished" callback function. A "function pointer"
 * (the `(*sub_motion_done_cb_t)` syntax) is a variable that holds the
 * address of a function instead of holding data - it lets you pass
 * "a function to call later" around like any other value. Whoever calls
 * sub_motion_forward_mm() etc. supplies one of these; sub_motion.c
 * invokes it once the move ends.
 *   move_id      - which move this refers to (matches the id returned
 *                  when the move was started, useful if you started more
 *                  than one and want to know which finished).
 *   completed    - true if the move reached its goal normally, false if
 *                  it was cut short (stopped, pre-empted by a newer move
 *                  request, or aborted because a wheel stalled or the
 *                  move took far too long).
 *   travelled_mm - how far the car actually moved during this attempt;
 *                  for a turn, how far each wheel ran along its arc.
 *   ctx          - your own pointer, handed back unchanged; a way to pass
 *                  "which object/struct is this callback for" without
 *                  global variables.
 * Runs in the fast-lane dispatcher task (not inside an interrupt, and not
 * inside sub_motion), after RC_EVT_MOTION_DONE is published - so it may
 * start the next move straight away. Must not block - see TEAM_GUIDE.md
 * §1.2/§1.3. */
typedef void (*sub_motion_done_cb_t)(uint32_t move_id,
                                     bool completed,
                                     uint32_t travelled_mm,
                                     void *ctx);

/* Boot-time setup: PID gains, the control task, and the subscriber that
 * delivers callbacks. Call once at startup, after rc_event_init() and the
 * motor and encoder drivers, before any other sub_motion_* call. */
rc_result_t sub_motion_init(void);

/* Cruise speed for distance moves, mm/s. This is the "set speed" dial on
 * the cruise-control analogy - the PID holds the wheels at this speed
 * between the ramp up and the slow-down at the end of a move. Takes
 * effect at once, even on a move already running. */
rc_result_t sub_motion_set_speed(uint16_t mm_s);

/*
 *  Queued moves. Each returns a move id (used to match it up with the
 *  completion callback/event later), or 0 if the request was rejected.
 *  Passing cb = NULL is fine if you only want the RC_EVT_MOTION_DONE
 *  event (published on the event bus - see TEAM_GUIDE.md §1.2) instead
 *  of a direct callback.
 *
 *  sub_motion_turn_deg spins the car in place: deg < 0 turns left
 *  (anticlockwise), deg > 0 turns right.
 */
uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx);
uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb,
                                void *ctx);
uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx);

/*
 *  Continuous mode, for the line follower. Sets a base speed and a
 *  steering bias in permille (parts per thousand - this codebase's
 *  integer-only stand-in for a percentage, see TEAM_GUIDE.md §5); no
 *  distance target, no completion callback - it just keeps driving until
 *  told otherwise. This is the mode the car spends most of its run in.
 *  steer > 0 steers right (left wheel faster), steer < 0 steers left.
 *  Open loop: it sets motor duty directly and never reads the encoders,
 *  so it works before they are wired, but the speed sags with the
 *  battery. Returns RC_ERR_BUSY, changing nothing, while a queued move or
 *  turn is running - it never cuts one short.
 */
rc_result_t sub_motion_drive(int16_t base_permille, int16_t steer_permille);

/*
 *  The closed-loop version of sub_motion_drive(): base and steer are
 *  speeds in mm/s, and the PID holds the left wheel at base + steer and
 *  the right at base - steer, whatever the battery level. Needs wired,
 *  calibrated encoders (if a wheel reports no movement while pushed hard,
 *  the car stops). Same steer sign and RC_ERR_BUSY rule as above.
 */
rc_result_t sub_motion_drive_speed(int16_t base_mm_s, int16_t steer_mm_s);

/* Cancel whatever move is running and stop the motors. brake=true stops
 * fast (electrical braking); brake=false coasts to a stop. */
rc_result_t sub_motion_stop(bool brake);

/* True while a queued move (forward/backward/turn) is still running.
 * False during continuous drive mode or when idle. */
bool sub_motion_busy(void);

/* The speed the controller is asking one wheel for right now, mm/s,
 * negative when reversing; 0 when idle or in open-loop drive. For logs
 * such as the motion bench's step response - it changes nothing. */
int32_t sub_motion_target_mm_s(rc_side_t side);

#endif /* SUB_MOTION_H */
