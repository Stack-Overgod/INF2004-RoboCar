/*
 *  sub_motion.c
 *
 *  The "brain" of Buddy 2's module: decides what duty to send the motors
 *  every control cycle so the wheels track a target speed (PID control)
 *  or a target distance/turn, and runs the non-blocking move state
 *  machine described in sub_motion.h. See docs/buddy2-motion/buddy2-motion.md
 *  for the full plain-English walkthrough.
 *
 *  What happens every RC_PERIOD_MOTION_MS (20 ms), in motion_task():
 *
 *    encoders   -> measured speed of each wheel
 *    move goal  -> speed profile: ramp up, cruise, slow down near the end
 *               -> straight-line correction: trim the wheel that is ahead
 *               -> a target speed for each wheel
 *    each wheel -> feedforward + PID -> duty -> drv_motor
 *
 *  Sharing rules. The motion state below is used by motion_task AND by
 *  whichever task calls the public API (the event dispatchers, the bench).
 *  One kernel mutex guards all of it, so a request can never land half way
 *  through a control cycle. Completion callbacks are never called with the
 *  mutex held: finishing a move publishes RC_EVT_MOTION_DONE, and this
 *  module's own fast-lane subscriber calls the requester's callback from
 *  the dispatcher task (TEAM_GUIDE.md §1.3). That also means a callback
 *  that starts the next move (sub_nav's bypass legs do) never runs inside
 *  this module.
 */
#include "rc_prelude.h"
#include <tm/tmonitor.h>
#include "sub_motion.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_event.h"

/* ------------------------------------------------------------------ *
 *  Tuning constants
 *
 *  Each value is a starting point worked out from the hardware's rough
 *  specs, not a measurement. The motion bench (bench_motion() in
 *  app/app_bench.c, BUILD.md §5.2) prints what you need to replace them.
 * ------------------------------------------------------------------ */

/* Largest duty in "permille" (parts per thousand: 1000 = 100.0 %), the
 * same limit drv_motor.c clamps to. */
#define DUTY_MAX                (1000)

/* Fixed-point scale for the PID gains. The RP2040 has no FPU, so a gain
 * of 1.0 is stored as 256, 0.5 as 128, and so on. */
#define PID_SCALE               (256)

/* PID gains, stored x PID_SCALE. The speed error is in mm/s and the
 * output is duty in permille, so:
 *   PID_KP 256 (1.0) - 50 mm/s too slow adds 50 permille at once.
 *   PID_KI 512 (2.0) - still 50 mm/s too slow adds another 100 permille
 *                      for every second it stays that way.
 *   PID_KD 0         - start with PI. The measured speed jitters from
 *                      tick to tick and D amplifies that jitter; try a
 *                      small value (e.g. 8) while tuning.
 * Record the bench's step response for each setting you try: that log is
 * the PID tuning report. */
#define PID_KP                  (256)
#define PID_KI                  (512)
#define PID_KD                  (0)

/* Feedforward: a guess at the duty a wheel needs for a given speed, so
 * the PID only has to correct the difference instead of discovering the
 * whole duty from nothing.
 *   FF_START_PERMILLE - duty at which a wheel on the floor just starts
 *                       to turn (motor friction, the "deadband").
 *   FF_TOP_SPEED_MM_S - wheel speed at 100 % duty, car on the floor.
 * Both are guesses; measure them with the motion bench. */
#define FF_START_PERMILLE       (150)
#define FF_TOP_SPEED_MM_S       (400)

/* Speed profile for queued moves. A move starts at MIN_SPEED_MM_S, speeds
 * up by ACCEL_MM_S_PER_CYCLE every 20 ms (1 m/s^2, gentle enough not to
 * spin the wheels), cruises, and over the last DECEL_ZONE_MM slows back
 * down to MIN_SPEED_MM_S, so the brake at the goal stops the car close to
 * it instead of coasting past. */
#define MIN_SPEED_MM_S          (60)
#define ACCEL_MM_S_PER_CYCLE    (20)
#define DECEL_ZONE_MM           (60U)

/* Spin turns: wheel speed while turning, and the slow-down zone measured
 * along each wheel's arc (a 90 degree turn is only ~86 mm of arc). */
#define TURN_SPEED_MM_S         (150)
#define TURN_DECEL_ZONE_MM      (30U)

/* Wheels slip when spinning in place, so a turn usually falls short of
 * the geometry. Permille: 1000 = trust the geometry, 1100 = drive each
 * wheel 10 % further. Measure ~10 turns with a protractor, then set
 *     TURN_SLIP_PERMILLE = 1000 * angle asked / average angle measured. */
#define TURN_SLIP_PERMILLE      (1000U)

/* Straight-line correction. For every mm one wheel has travelled further
 * than the other, that wheel's target speed drops by SYNC_GAIN mm/s and
 * the other's rises by the same, capped at SYNC_MAX_MM_S and at half the
 * current speed. The difference between the two wheels' distances IS the
 * car's heading error, so this steers it back onto the straight line. */
#define SYNC_GAIN               (4)
#define SYNC_MAX_MM_S           (80)

/* Safety. A wheel pushed at STALL_DUTY_PERMILLE or more that reports no
 * movement for STALL_CYCLES (0.6 s) is stalled - blocked, or its encoder
 * is unplugged - so the car stops instead of driving blind at full power.
 * A move also stops if it takes more than twice its expected time plus
 * MOVE_TIMEOUT_EXTRA_MS. Either way its callback reports completed=false. */
#define STALL_DUTY_PERMILLE     (500)
#define STALL_CYCLES            (30U)
#define MOVE_TIMEOUT_EXTRA_MS   (2000U)

/* Longest any caller waits for the motion mutex. motion_task holds it for
 * a few tens of microseconds per cycle, so this is only a safety net - a
 * dispatcher callback must never block for long (rc_event.h). */
#define LOCK_TMO_MS             (20)

/* Move callbacks waiting to be delivered, and finished moves waiting to
 * be published. In practice at most two are ever in flight (the move that
 * finished and the one that pre-empted it); the rest is headroom. */
#define CB_SLOTS                (8U)
#define DONE_QUEUE              (8U)

/* ------------------------------------------------------------------ *
 *  Types
 * ------------------------------------------------------------------ */

/* What the car is doing right now. */
typedef enum {
    MODE_IDLE = 0,      /* motors off, nothing to do */
    MODE_DISTANCE,      /* straight line to a distance goal, fwd or back */
    MODE_TURN,          /* spin in place to an angle goal */
    MODE_CONTINUOUS,    /* open-loop duty + steer: sub_motion_drive() */
    MODE_SPEED          /* closed-loop speed + steer: drive_speed() */
} mode_t;

/* One wheel's speed controller.
 *   kp, ki, kd - gains, x PID_SCALE (see PID_KP above).
 *   i_term     - the integral's contribution so far, in permille
 *                x PID_SCALE. Storing the contribution (not the raw sum of
 *                errors) makes the anti-windup clamp a plain duty limit.
 *   prev_meas  - last cycle's measured speed, for the D term. D acts on
 *                the measurement, not the error, so a new target (e.g.
 *                the start of a move) causes no sudden "derivative kick".
 */
typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t i_term;
    int32_t prev_meas;
} pid_t;

/* Everything the control loop keeps about one wheel. */
typedef struct {
    pid_t    pid;
    int8_t   dir;          /* +1 forward, -1 reverse, this cycle */
    int32_t  target;       /* target speed this cycle, mm/s, >= 0 */
    int32_t  raw;          /* measured speed, mm/s, >= 0 */
    int32_t  meas;         /* raw after a light low-pass: the PID input */
    int32_t  duty;         /* duty sent this cycle, permille, >= 0 */
    uint32_t start_count;  /* encoder count when the current move began */
    uint32_t stall_cycles; /* cycles in a row pushing hard, no motion */
} wheel_t;

/* A callback waiting for its move to finish. id 0 = free slot. */
typedef struct {
    uint32_t             id;
    sub_motion_done_cb_t cb;
    void                *ctx;
} cb_slot_t;

/* A finished move waiting to be published as RC_EVT_MOTION_DONE. */
typedef struct {
    uint32_t id;
    bool     completed;
    uint32_t travelled_mm;
} done_t;

/* ------------------------------------------------------------------ *
 *  State. "static" keeps all of it private to this file. Everything
 *  except the two kernel ids is guarded by motion_mtxid.
 * ------------------------------------------------------------------ */

static ID               motion_mtxid;
static ID               motion_tskid;
/* volatile: sub_motion_busy() reads it without the mutex */
static volatile mode_t  mode;
static wheel_t          wheels[2];              /* indexed by rc_side_t */

/* cruise speed for distance moves, set by sub_motion_set_speed() */
static volatile uint16_t cruise_mm_s = 250U;

/* The queued move in progress (MODE_DISTANCE / MODE_TURN). */
static uint32_t goal_ticks;     /* average ticks per wheel to cover */
static uint32_t decel_ticks;    /* start slowing this many ticks out */
static int32_t  ramp_mm_s;      /* speed limit while ramping up */
static uint32_t move_cycles;    /* control cycles since the move began */
static uint32_t timeout_cycles; /* give up after this many */
/* +1 or -1. MODE_DISTANCE: +1 forward, -1 backward. MODE_TURN: +1 right
 * (clockwise seen from above), -1 left. */
static int8_t   move_sign = 1;
static uint32_t move_id_cur;    /* 0 = no queued move */
static uint32_t move_id_next = 1U;

/* The continuous modes. */
static int16_t  base_cmd;       /* MODE_CONTINUOUS, permille */
static int16_t  steer_cmd;
static int16_t  base_spd;       /* MODE_SPEED, mm/s */
static int16_t  steer_spd;

static cb_slot_t cb_slots[CB_SLOTS];
static done_t    done_q[DONE_QUEUE];
static uint32_t  done_n;

/* Set with the mutex held, printed by motion_task after releasing it -
 * printing is slow and must not happen inside the lock. */
static const char *fault_msg;

/* ------------------------------------------------------------------ *
 *  Small helpers
 * ------------------------------------------------------------------ */

static bool motion_lock(void)
{
    return tk_loc_mtx(motion_mtxid, LOCK_TMO_MS) == E_OK;
}

static void motion_unlock(void)
{
    (void)tk_unl_mtx(motion_mtxid);
}

/* True while a queued move (forward/backward/turn) is running. */
static bool queued_move_active(void)
{
    return (mode == MODE_DISTANCE) || (mode == MODE_TURN);
}

/* Ticks one wheel has turned since the current move began. Unsigned
 * subtraction is wrap safe. It counts every tick whichever way the wheel
 * turns, which is what a move needs: the motors are driven in the
 * direction the move asks for, so the count is how far the wheel went. */
static uint32_t move_ticks(rc_side_t side)
{
    return drv_encoder_count(side) - wheels[side].start_count;
}

/* Micrometres to encoder ticks, rounded. 64-bit because a long distance
 * in micrometres does not fit 32 bits; used once per move, so cheap. */
static uint32_t um_to_ticks(uint64_t um)
{
    uint64_t t = (um + (RC_ENC_UM_PER_TICK / 2U)) / RC_ENC_UM_PER_TICK;

    return (t > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)t;
}

static uint32_t ticks_to_mm(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * RC_ENC_UM_PER_TICK) / 1000U);
}

/* Keep a signed duty inside -DUTY_MAX..+DUTY_MAX. */
static int32_t clamp_duty(int32_t v)
{
    if (v > DUTY_MAX) {
        return DUTY_MAX;
    }
    if (v < -DUTY_MAX) {
        return -DUTY_MAX;
    }
    return v;
}

/* The speed a move cruises at. Never below MIN_SPEED_MM_S, so a move can
 * always finish (a cruise speed of 0 would never get there). */
static int32_t cruise_speed(mode_t m)
{
    int32_t v = (m == MODE_TURN) ? TURN_SPEED_MM_S : (int32_t)cruise_mm_s;

    return (v < MIN_SPEED_MM_S) ? MIN_SPEED_MM_S : v;
}

/* ------------------------------------------------------------------ *
 *  PID with feedforward
 * ------------------------------------------------------------------ */

static void pid_reset(pid_t *p, int32_t meas)
{
    p->i_term    = 0;
    p->prev_meas = meas;
}

/* The duty a wheel roughly needs to turn at v mm/s: a straight line from
 * FF_START_PERMILLE (just moving) to 100 % at FF_TOP_SPEED_MM_S. */
static int32_t feedforward(int32_t v)
{
    if (v <= 0) {
        return 0;
    }
    return FF_START_PERMILLE
           + ((v * (DUTY_MAX - FF_START_PERMILLE)) / FF_TOP_SPEED_MM_S);
}

/* One control step for one wheel. target and meas are speeds in mm/s,
 * both >= 0 (direction is applied later, when the duty is sent). Returns
 * the duty, 0..DUTY_MAX permille.
 *
 * Like a car's cruise control: feedforward is the throttle you would
 * guess for that speed; P reacts to how wrong the speed is right now; I
 * slowly adds whatever steady push the guess was missing (friction, a
 * weaker motor, a low battery); D reacts to how fast the speed is
 * changing, to damp overshoot. */
static int32_t pid_step(pid_t *p, int32_t target, int32_t meas)
{
    int32_t err;
    int32_t d;
    int32_t out;

    if (target <= 0) {
        pid_reset(p, meas);
        return 0;                   /* no speed wanted: let it coast */
    }

    err = target - meas;

    /* Rate of change of the measured speed in mm/s per second, negated
     * so a wheel speeding up quickly is held back. */
    d = -((meas - p->prev_meas) * 1000) / RC_PERIOD_MOTION_MS;
    p->prev_meas = meas;

    out = ((feedforward(target) * PID_SCALE) + (p->kp * err) + p->i_term
           + (p->kd * d)) / PID_SCALE;

    /* Anti-windup: only integrate while the output can still act on the
     * error. Pinned at full duty and still too slow (or at zero and still
     * too fast), more integral would only store up a jolt for later. The
     * clamp also caps the integral at one full duty either way. */
    if (!(((out >= DUTY_MAX) && (err > 0)) || ((out <= 0) && (err < 0)))) {
        p->i_term += (p->ki * err * RC_PERIOD_MOTION_MS) / 1000;
        if (p->i_term > (DUTY_MAX * PID_SCALE)) {
            p->i_term = DUTY_MAX * PID_SCALE;
        }
        if (p->i_term < -(DUTY_MAX * PID_SCALE)) {
            p->i_term = -(DUTY_MAX * PID_SCALE);
        }
    }

    /* Never command the opposite direction to slow down; ease off to 0
     * and let friction (or the brake at the end of a move) do it. */
    if (out > DUTY_MAX) {
        out = DUTY_MAX;
    }
    if (out < 0) {
        out = 0;
    }
    return out;
}

/* ------------------------------------------------------------------ *
 *  Wheels
 * ------------------------------------------------------------------ */

/* Take this cycle's measurement for one wheel. The PID sees a value
 * halfway between this reading and the previous filtered value (a simple
 * low-pass filter), which smooths tick-to-tick jitter for ~20 ms of lag.
 *
 * Only the size of the speed is used: direction comes from the motor
 * command. A loose B wire makes the encoder's direction wrong, and if the
 * PID believed that, it would see "going backwards" and drive the wheel
 * to full power. */
static void wheel_measure(wheel_t *w, int32_t speed_mm_s)
{
    w->raw  = (speed_mm_s < 0) ? -speed_mm_s : speed_mm_s;
    w->meas = (w->meas + w->raw) / 2;
}

/* Run one wheel's PID towards `target` mm/s in direction `dir`, and keep
 * the stall watch up to date. Nothing is sent to the motors yet. */
static void wheel_control(wheel_t *w, int8_t dir, int32_t target)
{
    if (dir != w->dir) {
        pid_reset(&w->pid, w->meas);    /* reversing: start the PID fresh */
        w->dir = dir;
    }
    w->target = (target > 0) ? target : 0;
    w->duty   = pid_step(&w->pid, w->target, w->meas);

    if ((w->duty >= STALL_DUTY_PERMILLE) && (w->raw == 0)) {
        w->stall_cycles++;
    } else {
        w->stall_cycles = 0U;
    }
}

/* Send both wheels' duty, with direction, to the motor driver. */
static void wheels_apply(void)
{
    (void)drv_motor_set_pair(
        (int16_t)(wheels[RC_SIDE_LEFT].dir * wheels[RC_SIDE_LEFT].duty),
        (int16_t)(wheels[RC_SIDE_RIGHT].dir * wheels[RC_SIDE_RIGHT].duty));
}

/* Forget targets and duties, e.g. after the motors were stopped. */
static void wheels_zero(void)
{
    uint32_t i;

    for (i = 0U; i < 2U; i++) {
        wheels[i].target       = 0;
        wheels[i].duty         = 0;
        wheels[i].stall_cycles = 0U;
        pid_reset(&wheels[i].pid, wheels[i].meas);
    }
}

/* NULL, or the message for whichever wheel has stalled. */
static const char *stall_check(void)
{
    if (wheels[RC_SIDE_LEFT].stall_cycles >= STALL_CYCLES) {
        return "left wheel not turning (blocked, or its encoder is not"
               " counting) - stopped";
    }
    if (wheels[RC_SIDE_RIGHT].stall_cycles >= STALL_CYCLES) {
        return "right wheel not turning (blocked, or its encoder is not"
               " counting) - stopped";
    }
    return NULL;
}

/* ------------------------------------------------------------------ *
 *  Completion
 * ------------------------------------------------------------------ */

/* Publish every finished move still waiting. If a ring is full the rest
 * stay queued and motion_task retries next cycle, so a completion is never
 * silently lost (a lost one would leave sub_nav waiting forever). */
static void publish_done(void)
{
    rc_event_t evt;
    uint32_t   i;

    while (done_n > 0U) {
        evt.id = RC_EVT_MOTION_DONE;
        evt.u.motion_done.move_id      = done_q[0].id;
        evt.u.motion_done.completed    = done_q[0].completed;
        evt.u.motion_done.travelled_mm = done_q[0].travelled_mm;
        if (rc_event_publish(&evt) != RC_OK) {
            break;
        }
        for (i = 1U; i < done_n; i++) {
            done_q[i - 1U] = done_q[i];
        }
        done_n--;
    }
}

/* End the queued move: brake, go idle, and report it. `completed` is true
 * when it reached its goal, false when it was stopped, pre-empted by a
 * newer move, or aborted by the stall/timeout guards. For a turn,
 * travelled_mm is the distance each wheel ran along its arc. */
static void finish_move(bool completed)
{
    uint32_t travelled = ticks_to_mm((move_ticks(RC_SIDE_LEFT)
                                      + move_ticks(RC_SIDE_RIGHT)) / 2U);

    (void)drv_motor_stop(RC_MOTOR_BRAKE);
    wheels_zero();
    mode = MODE_IDLE;

    if (done_n < DONE_QUEUE) {
        done_q[done_n].id           = move_id_cur;
        done_q[done_n].completed    = completed;
        done_q[done_n].travelled_mm = travelled;
        done_n++;
    }
    move_id_cur = 0U;
    publish_done();
}

/* Remember a callback until its move's RC_EVT_MOTION_DONE arrives. */
static void cb_register(uint32_t id, sub_motion_done_cb_t cb, void *ctx)
{
    uint32_t i;

    for (i = 0U; i < CB_SLOTS; i++) {
        if (cb_slots[i].id == 0U) {
            break;
        }
    }
    if (i == CB_SLOTS) {
        /* Cannot happen in normal use - a slot is freed within one
         * dispatcher pass. Reuse slot 0 rather than drop the newest. */
        i = 0U;
    }
    cb_slots[i].id  = id;
    cb_slots[i].cb  = cb;
    cb_slots[i].ctx = ctx;
}

/* Our own subscriber to RC_EVT_MOTION_DONE, on the fast lane: find the
 * callback for that move and call it - here, in the dispatcher task, with
 * the mutex released, so the callback may start the next move. */
static void on_motion_done(const rc_event_t *evt, void *ctx)
{
    sub_motion_done_cb_t cb     = NULL;
    void                *cb_ctx = NULL;
    uint32_t             id     = evt->u.motion_done.move_id;
    uint32_t             i;

    (void)ctx;

    if ((id == 0U) || !motion_lock()) {
        return;
    }
    for (i = 0U; i < CB_SLOTS; i++) {
        if (cb_slots[i].id == id) {
            cb     = cb_slots[i].cb;
            cb_ctx = cb_slots[i].ctx;
            cb_slots[i].id = 0U;
            break;
        }
    }
    motion_unlock();

    if (cb != NULL) {
        cb(id, evt->u.motion_done.completed,
           evt->u.motion_done.travelled_mm, cb_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Starting a move (called with the mutex held)
 * ------------------------------------------------------------------ */

/* Start a queued move of goal_um micrometres per wheel: straight
 * (MODE_DISTANCE) or along each wheel's arc (MODE_TURN). A move already
 * running is pre-empted first and reported with completed = false.
 * Returns the new move's id. */
static uint32_t start_move(mode_t m, uint64_t goal_um, int8_t sign,
                           sub_motion_done_cb_t cb, void *ctx)
{
    uint32_t id;
    uint32_t i;
    uint32_t zone_mm = (m == MODE_TURN) ? TURN_DECEL_ZONE_MM : DECEL_ZONE_MM;
    uint64_t timeout_ms;

    if (queued_move_active()) {
        finish_move(false);
    }

    id = move_id_next;
    move_id_next++;
    if (move_id_next == 0U) {
        move_id_next = 1U;          /* 0 means "no move" */
    }
    if (cb != NULL) {
        cb_register(id, cb, ctx);
    }

    /* drv_encoder_distance_mm() restarts from 0 for each move too, as
     * telemetry's odometry figures always have. */
    drv_encoder_reset();
    for (i = 0U; i < 2U; i++) {
        wheels[i].start_count  = drv_encoder_count((rc_side_t)i);
        wheels[i].stall_cycles = 0U;
        pid_reset(&wheels[i].pid, wheels[i].meas);
    }

    goal_ticks  = um_to_ticks(goal_um);
    decel_ticks = um_to_ticks((uint64_t)zone_mm * 1000U);
    ramp_mm_s   = MIN_SPEED_MM_S;
    move_cycles = 0U;

    /* Expected time in ms is micrometres / (mm/s). Allow twice that plus
     * a margin for the ramps before calling the move stuck. */
    timeout_ms = ((goal_um / (uint64_t)cruise_speed(m)) * 2U)
                 + MOVE_TIMEOUT_EXTRA_MS;
    timeout_ms /= RC_PERIOD_MOTION_MS;
    timeout_cycles = (timeout_ms > 0xFFFFFFFFULL) ? 0xFFFFFFFFU
                                                  : (uint32_t)timeout_ms;

    move_sign   = sign;
    move_id_cur = id;
    mode        = m;        /* motion_task takes over from here */
    return id;
}

/* Lock, start the move, unlock. Returns 0 if the request was rejected. */
static uint32_t queue_move(mode_t m, uint64_t goal_um, int8_t sign,
                           sub_motion_done_cb_t cb, void *ctx)
{
    uint32_t id;

    if (!motion_lock()) {
        return 0U;
    }
    id = start_move(m, goal_um, sign, cb, ctx);
    motion_unlock();
    return id;
}

/* ------------------------------------------------------------------ *
 *  One control cycle, per mode (called with the mutex held)
 * ------------------------------------------------------------------ */

/* The speed the move should be at, given how far it has got: ramp up
 * from MIN_SPEED_MM_S, cruise, then slow linearly back to MIN_SPEED_MM_S
 * across the last decel_ticks. Only called while progress < goal_ticks. */
static int32_t profile_speed(uint32_t progress)
{
    int32_t  cruise    = cruise_speed(mode);
    int32_t  v         = cruise;
    uint32_t remaining = goal_ticks - progress;
    int32_t  v_decel;

    if (ramp_mm_s < cruise) {
        ramp_mm_s += ACCEL_MM_S_PER_CYCLE;
    }
    if (v > ramp_mm_s) {
        v = ramp_mm_s;
    }

    /* (cruise - MIN) is at most ~65000 and remaining < decel_ticks here,
     * so the product stays well inside 32 bits. */
    if (remaining < decel_ticks) {
        v_decel = MIN_SPEED_MM_S
                  + (int32_t)(((uint32_t)(cruise - MIN_SPEED_MM_S)
                               * remaining) / decel_ticks);
        if (v > v_decel) {
            v = v_decel;
        }
    }
    return (v < MIN_SPEED_MM_S) ? MIN_SPEED_MM_S : v;
}

/* Straight-line correction in mm/s: positive when the left wheel is
 * ahead, i.e. how much to slow the left wheel and speed up the right. */
static int32_t sync_correction(uint32_t t_l, uint32_t t_r, int32_t v)
{
    int32_t diff = (int32_t)(t_l - t_r);    /* ticks, + = left ahead */
    int32_t cap  = (v / 2 < SYNC_MAX_MM_S) ? (v / 2) : SYNC_MAX_MM_S;
    int64_t corr;

    /* ticks x um-per-tick / 1000 = mm ahead; x SYNC_GAIN = mm/s. 64-bit
     * because a blocked wheel can fall far behind before the stall guard
     * stops the move, and the product would then overflow 32 bits. */
    corr = ((int64_t)diff * (int64_t)RC_ENC_UM_PER_TICK * SYNC_GAIN) / 1000;

    if (corr > cap) {
        corr = cap;
    }
    if (corr < -cap) {
        corr = -cap;
    }
    return (int32_t)corr;
}

/* MODE_DISTANCE and MODE_TURN. */
static void run_move(void)
{
    uint32_t    t_l      = move_ticks(RC_SIDE_LEFT);
    uint32_t    t_r      = move_ticks(RC_SIDE_RIGHT);
    uint32_t    progress = (t_l + t_r) / 2U;
    int8_t      dir_l    = move_sign;
    int8_t      dir_r    = move_sign;
    int32_t     v;
    int32_t     corr;
    const char *stall;

    if (progress >= goal_ticks) {
        finish_move(true);
        return;
    }

    move_cycles++;
    if (move_cycles > timeout_cycles) {
        fault_msg = "move took too long - stopped";
        finish_move(false);
        return;
    }

    /* A spin turn drives the wheels in opposite directions: turning
     * right (+1), the left wheel goes forward and the right one back. */
    if (mode == MODE_TURN) {
        dir_r = (int8_t)-move_sign;
    }

    v    = profile_speed(progress);
    corr = sync_correction(t_l, t_r, v);
    wheel_control(&wheels[RC_SIDE_LEFT],  dir_l, v - corr);
    wheel_control(&wheels[RC_SIDE_RIGHT], dir_r, v + corr);

    stall = stall_check();
    if (stall != NULL) {
        fault_msg = stall;
        finish_move(false);
        return;
    }
    wheels_apply();
}

/* MODE_SPEED: hold each wheel at base +/- steer mm/s with the PID. */
static void run_speed(void)
{
    int32_t     v_l = (int32_t)base_spd + (int32_t)steer_spd;
    int32_t     v_r = (int32_t)base_spd - (int32_t)steer_spd;
    const char *stall;

    wheel_control(&wheels[RC_SIDE_LEFT],  (v_l < 0) ? (int8_t)-1 : (int8_t)1,
                  (v_l < 0) ? -v_l : v_l);
    wheel_control(&wheels[RC_SIDE_RIGHT], (v_r < 0) ? (int8_t)-1 : (int8_t)1,
                  (v_r < 0) ? -v_r : v_r);

    stall = stall_check();
    if (stall != NULL) {
        fault_msg = stall;
        (void)drv_motor_stop(RC_MOTOR_BRAKE);
        wheels_zero();
        mode = MODE_IDLE;
        return;
    }
    wheels_apply();
}

/* MODE_CONTINUOUS: steering as an open-loop differential on top of the
 * base duty - left = base + steer, right = base - steer. Positive steer
 * (sub_line's "steer right") speeds up the left wheel and slows the
 * right, pivoting the car right. Nothing here reads the encoders, so the
 * line follower works even before they are wired or calibrated; the
 * price is that the speed sags with the battery. drive_speed() is the
 * closed-loop alternative. */
static void run_continuous(void)
{
    int32_t out_l = (int32_t)base_cmd + (int32_t)steer_cmd;
    int32_t out_r = (int32_t)base_cmd - (int32_t)steer_cmd;

    /* Clamp in 32 bits before narrowing: base + steer can exceed int16. */
    (void)drv_motor_set_pair((int16_t)clamp_duty(out_l),
                             (int16_t)clamp_duty(out_r));
}

/* ------------------------------------------------------------------ *
 *  Control task
 *
 *  Wakes every RC_PERIOD_MOTION_MS. tk_dly_tsk yields the CPU for the
 *  whole interval; it is not a spin. Everything between wakeups is
 *  bounded arithmetic, done with the mutex held.
 * ------------------------------------------------------------------ */

static void motion_task(INT stacd, void *exinf)
{
    rc_event_t  evt;
    int32_t     sp_l;
    int32_t     sp_r;
    const char *msg;

    (void)stacd;
    (void)exinf;

    for (;;) {
        tk_dly_tsk(RC_PERIOD_MOTION_MS);

        /* Odometry for telemetry and Buddy 4's terrain code, every cycle
         * whatever the mode. Signed speeds (the encoder's own direction),
         * distances since the last move began. */
        sp_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
        sp_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);
        evt.id = RC_EVT_ODOMETRY;
        evt.u.odometry.speed_l_mm_s = sp_l;
        evt.u.odometry.speed_r_mm_s = sp_r;
        evt.u.odometry.dist_l_mm    = drv_encoder_distance_mm(RC_SIDE_LEFT);
        evt.u.odometry.dist_r_mm    = drv_encoder_distance_mm(RC_SIDE_RIGHT);
        (void)rc_event_publish(&evt);

        if (!motion_lock()) {
            continue;
        }

        wheel_measure(&wheels[RC_SIDE_LEFT],  sp_l);
        wheel_measure(&wheels[RC_SIDE_RIGHT], sp_r);

        switch (mode) {
        case MODE_DISTANCE:
        case MODE_TURN:
            run_move();
            break;
        case MODE_SPEED:
            run_speed();
            break;
        case MODE_CONTINUOUS:
            run_continuous();
            break;
        case MODE_IDLE:
        default:
            break;          /* motors already stopped */
        }

        publish_done();     /* retry anything a full ring refused */
        msg       = fault_msg;
        fault_msg = NULL;
        motion_unlock();

        if (msg != NULL) {
            tm_printf((UB *)"[motion] %s\n", msg);
        }
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

/* Boot-time setup: PID gains, the mutex, our RC_EVT_MOTION_DONE
 * subscriber, and the control task. Call once, after rc_event_init() and
 * the motor/encoder drivers, before any other sub_motion_* call. */
rc_result_t sub_motion_init(void)
{
    T_CTSK   ctsk;
    T_CMTX   cmtx;
    uint32_t i;

    for (i = 0U; i < 2U; i++) {
        wheels[i].pid.kp = PID_KP;
        wheels[i].pid.ki = PID_KI;
        wheels[i].pid.kd = PID_KD;
        wheels[i].dir    = 1;
        wheels[i].raw    = 0;
        wheels[i].meas   = 0;
    }
    wheels_zero();
    mode = MODE_IDLE;

    /* TA_INHERIT: while a higher-priority task (a dispatcher) waits for
     * the mutex, motion_task runs at that priority, so the wait stays as
     * short as motion_task's own critical section. ceilpri is only used
     * by TA_CEILING mutexes. */
    cmtx.exinf   = NULL;
    cmtx.mtxatr  = TA_INHERIT;
    cmtx.ceilpri = 1;
    motion_mtxid = tk_cre_mtx(&cmtx);
    if (motion_mtxid <= E_OK) {
        return RC_ERR_HARDWARE;
    }

    if (rc_event_subscribe(RC_EVT_MOTION_DONE, RC_LANE_FAST,
                           on_motion_done, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }

    /* The control task: priority RC_PRI_MOTION (lower number = higher
     * priority), RC_STACK_SZ of stack, written in C (TA_HLNG), running in
     * the least-privileged ring (TA_RNG3). */
    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_MOTION;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = motion_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    motion_tskid = tk_cre_tsk(&ctsk);
    if (motion_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    (void)tk_sta_tsk(motion_tskid, 0);

    return RC_OK;
}

/* Cruise speed for distance moves, mm/s. Takes effect at once, even on a
 * move already running. Values below MIN_SPEED_MM_S cruise at that. */
rc_result_t sub_motion_set_speed(uint16_t mm_s)
{
    cruise_mm_s = mm_s;
    return RC_OK;
}

/* moveForward(distance). Non-blocking: returns the move id at once while
 * motion_task drives; the callback fires when the move ends. */
uint32_t sub_motion_forward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    return queue_move(MODE_DISTANCE, (uint64_t)mm * 1000U, 1, cb, ctx);
}

/* moveBackward(distance): the same move with both wheels reversed. */
uint32_t sub_motion_backward_mm(uint32_t mm, sub_motion_done_cb_t cb, void *ctx)
{
    return queue_move(MODE_DISTANCE, (uint64_t)mm * 1000U, -1, cb, ctx);
}

/* turnLeft(angle) / turnRight(angle), spinning in place: deg < 0 turns
 * left (anticlockwise), deg > 0 turns right - the convention sub_nav
 * uses. Each wheel runs along a circle whose diameter is the wheel base,
 * so for `deg` degrees it covers
 *     RC_WHEEL_BASE_MM x pi x deg / 360   mm,
 * computed in micrometres as base x 31416 x deg / 3600 (31416 = pi x
 * 10^4), then stretched by TURN_SLIP_PERMILLE for wheel slip. */
uint32_t sub_motion_turn_deg(int16_t deg, sub_motion_done_cb_t cb, void *ctx)
{
    uint32_t mag  = (deg < 0) ? (uint32_t)(-(int32_t)deg) : (uint32_t)deg;
    int8_t   sign = (deg < 0) ? (int8_t)-1 : (int8_t)1;
    uint64_t arc_um;

    arc_um = ((uint64_t)RC_WHEEL_BASE_MM * 31416U * mag) / 3600U;
    arc_um = (arc_um * TURN_SLIP_PERMILLE) / 1000U;

    return queue_move(MODE_TURN, arc_um, sign, cb, ctx);
}

/* Continuous open-loop drive, for the line follower. base and steer are
 * duty in permille; see run_continuous(). Refused (RC_ERR_BUSY) while a
 * queued move or turn is running: switching mode would drop that move
 * without its callback, leaving whoever queued it (sub_nav) waiting. */
rc_result_t sub_motion_drive(int16_t base_permille, int16_t steer_permille)
{
    rc_result_t res = RC_OK;

    if (!motion_lock()) {
        return RC_ERR_BUSY;
    }
    if (queued_move_active()) {
        res = RC_ERR_BUSY;
    } else {
        base_cmd  = base_permille;
        steer_cmd = steer_permille;
        mode      = MODE_CONTINUOUS;
    }
    motion_unlock();
    return res;
}

/* Continuous closed-loop drive: the left wheel held at base + steer mm/s
 * and the right at base - steer, by the PID - the same speed whatever the
 * battery level. Needs working, calibrated encoders. Same RC_ERR_BUSY rule
 * as sub_motion_drive(). */
rc_result_t sub_motion_drive_speed(int16_t base_mm_s, int16_t steer_mm_s)
{
    rc_result_t res = RC_OK;
    uint32_t    i;

    if (!motion_lock()) {
        return RC_ERR_BUSY;
    }
    if (queued_move_active()) {
        res = RC_ERR_BUSY;
    } else {
        if (mode != MODE_SPEED) {
            for (i = 0U; i < 2U; i++) {
                wheels[i].stall_cycles = 0U;
                pid_reset(&wheels[i].pid, wheels[i].meas);
            }
        }
        base_spd  = base_mm_s;
        steer_spd = steer_mm_s;
        mode      = MODE_SPEED;
    }
    motion_unlock();
    return res;
}

/* stop(). Cancels whatever is running (a queued move reports
 * completed = false) and stops the motors: brake = true stops fast,
 * false coasts. The motors are stopped even if the mutex is unavailable -
 * stopping must never be refused. */
rc_result_t sub_motion_stop(bool brake)
{
    rc_motor_stop_t how = brake ? RC_MOTOR_BRAKE : RC_MOTOR_COAST;

    if (!motion_lock()) {
        (void)drv_motor_stop(how);
        return RC_ERR_BUSY;
    }
    if (queued_move_active()) {
        finish_move(false);
    }
    mode = MODE_IDLE;
    wheels_zero();
    (void)drv_motor_stop(how);
    motion_unlock();
    return RC_OK;
}

/* True while a queued move (forward/backward/turn) is running. False in
 * the continuous modes and when idle, which have no "finished" point. */
bool sub_motion_busy(void)
{
    return queued_move_active();
}

/* The speed the controller is currently asking one wheel for, mm/s,
 * signed by direction. 0 when idle or in open-loop drive. For logging
 * (the bench's step response) - it does not change anything. */
int32_t sub_motion_target_mm_s(rc_side_t side)
{
    mode_t m = mode;

    if ((side > RC_SIDE_RIGHT) || (m == MODE_IDLE) || (m == MODE_CONTINUOUS)) {
        return 0;
    }
    return (int32_t)wheels[side].dir * wheels[side].target;
}
