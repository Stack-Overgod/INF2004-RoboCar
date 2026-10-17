/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (Buddy 2: motion control).
 *	Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01 ...
 *
 *	BEFORE POWER-UP: put the car on a box with BOTH WHEELS OFF THE
 *	GROUND.  From S2-U06 on, these tests spin the wheels.  Set
 *	S2_TEST_MOTORS to 0 below to run only the tests that keep the motors
 *	off (S2-U01..U05).
 *
 *	The expected values use the DEFAULT, not-yet-measured settings in
 *	rc_config.h (RC_ENC_TICKS_PER_REV, RC_WHEEL_DIAM_MM, RC_WHEEL_BASE_MM)
 *	and sub_motion.c (TURN_SLIP_PERMILLE).  Every line to change after
 *	measuring has a "#TODO" comment directly above it.
 */

#include "../rc_prelude.h"
#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

#include "../rc_config.h"
#include "../rc_event.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "sub_motion.h"

/* 1 = also run the tests that spin the wheels (car on a box!).
 * 0 = only the tests that keep the motors off. */
#ifndef S2_TEST_MOTORS
#define S2_TEST_MOTORS		1
#endif

/* 0 = the accuracy checks (S2-U14, S2-U18) only print what they measured.
 * With the placeholder RC_ENC_TICKS_PER_REV (20) the code thinks one tick
 * is about 10 mm, but the real encoder gives hundreds of ticks per wheel
 * turn, so speed and distance are badly off and those checks would fail
 * for a reason that is not a bug. */
#ifndef S2_CALIBRATED
/* #TODO: set to 1 once RC_ENC_TICKS_PER_REV, RC_WHEEL_DIAM_MM,
 *        RC_WHEEL_BASE_MM and TURN_SLIP_PERMILLE have been measured. */
#define S2_CALIBRATED		0
#endif

/* ------------------------------------------------------------------ *
 *  Expected values.  Work each one out BY HAND from the settings, so a
 *  mistake in the code's own maths shows up as a FAIL.
 * ------------------------------------------------------------------ */

/* S2-U04: micrometres per encoder tick,
 *   31416 x RC_WHEEL_DIAM_MM / (10 x RC_ENC_TICKS_PER_REV).
 * Default: 31416 x 64 / (10 x 20) = 10053. */
/* #TODO: recalculate with your measured values,
 *        e.g. 31416 x 64 / (10 x 374) = 537. */
#define S2_EXP_UM_PER_TICK	(10053UL)

/* S2-U09: the wheels spin at S2_SPIN_PERMILLE for S2_SPIN_MS, and each
 * must give at least S2_MIN_SPIN_TICKS ticks.  20 passes any working
 * encoder; the test prints the real counts. */
#define S2_SPIN_PERMILLE	(400)
#define S2_SPIN_MS		(1000)
/* #TODO: set to about half of the tick counts S2-U09 prints. */
#define S2_MIN_SPIN_TICKS	(20U)

/* S2-U11: longest extra wait, after s2_stop_settle, for both wheels to
 * read speed 0.  A wheel still turning after this is a real fault. */
#define S2_STOP_TMO_MS		(1500)

/* S2-U13..U15: straight moves of S2_MOVE_MM.  The distance the car
 * reports must be within S2_DIST_TOL_PCT of the goal (NFR12: 5%). */
#define S2_MOVE_MM		(300U)
/* #TODO: confirm once NFR12 is frozen from your 1 m tape-measure runs. */
#define S2_DIST_TOL_PCT		(5U)

/* S2-U16..U18: 90 degree spin turns.  Each wheel runs an arc of
 *   RC_WHEEL_BASE_MM x 3.1416 x 90 / 360 x TURN_SLIP_PERMILLE / 1000.
 * Default: 110 x 3.1416 x 90 / 360 x 1000 / 1000 = 86 mm. */
/* #TODO: recalculate with your measured RC_WHEEL_BASE_MM and your
 *        calibrated TURN_SLIP_PERMILLE. */
#define S2_EXP_ARC_90_MM	(86U)
/* Allowed error on that arc: 5 mm is about 5 degrees (FR3) on 86 mm. */
/* #TODO: recalculate as S2_EXP_ARC_90_MM x 5 / 90 once FR3 is frozen. */
#define S2_ARC_TOL_MM		(5U)

/* Longest wait for a move to report back.  sub_motion gives up by itself
 * after twice the expected time plus 2 s; this adds margin on top. */
#define S2_MOVE_TMO_MS		(8000)

/* S2-U05: ODOMETRY is published every RC_PERIOD_MOTION_MS (NFR2: 20 ms),
 * so a 1000 ms window should catch 50, allowing +-5. */
#define S2_ODO_WINDOW_MS	(1000)
#define S2_ODO_MIN		(45U)
#define S2_ODO_MAX		(55U)

/* A wheel number that does not exist. */
#define S2_BAD_SIDE		((rc_side_t)2)

static volatile uint32_t	odo_count;

/* Count RC_EVT_ODOMETRY events for S2-U05. */
static void s2_on_odometry(const rc_event_t *evt, void *ctx)
{
	(void)evt;
	(void)ctx;
	odo_count++;
}

#if S2_TEST_MOTORS

/* What a move's completion callback reported.  s2_on_done() writes it in
 * the FAST dispatcher task and the test reads it, so it is volatile. */
typedef struct {
	volatile bool		b_done;
	volatile bool		b_completed;
	volatile uint32_t	move_id;
	volatile uint32_t	travelled_mm;
} s2_move_t;

static s2_move_t		move_a;
static s2_move_t		move_b;

/* sub_motion_done_cb_t: record the result in the s2_move_t given as ctx. */
static void s2_on_done(uint32_t move_id, bool completed,
		       uint32_t travelled_mm, void *ctx)
{
	s2_move_t *m = (s2_move_t *)ctx;

	m->move_id      = move_id;
	m->b_completed  = completed;
	m->travelled_mm = travelled_mm;
	m->b_done       = true;		/* last: the test polls this one */
}

static void s2_clear(s2_move_t *m)
{
	m->b_done       = false;
	m->b_completed  = false;
	m->move_id      = 0U;
	m->travelled_mm = 0U;
}

/* Sleep in 10 ms steps until the move reports back, or ms runs out. */
static bool s2_wait_done(const s2_move_t *m, INT ms)
{
	INT waited = 0;

	while (!m->b_done && (waited < ms)) {
		(void)tk_dly_tsk(10);
		waited += 10;
	}
	return m->b_done;
}

/* Poll until the controller has set a target for this wheel (it does so
 * on its next 20 ms cycle), or ms runs out.  Returns that target. */
static int32_t s2_wait_target(rc_side_t side, INT ms)
{
	INT     waited = 0;
	int32_t target = sub_motion_target_mm_s(side);

	while ((target == 0) && (waited < ms)) {
		(void)tk_dly_tsk(2);
		waited += 2;
		target = sub_motion_target_mm_s(side);
	}
	return target;
}

#if S2_CALIBRATED
/* True if value is within tol of expected. */
static bool s2_near(uint32_t value, uint32_t expected, uint32_t tol)
{
	return ((value + tol) >= expected) && (value <= (expected + tol));
}
#endif /* S2_CALIBRATED */

/* Poll in 10 ms steps until both wheels read speed 0, or ms runs out.
 * A free wheel on a box can coast or rock for a while after the brake,
 * so a fixed wait is not proof that it has stopped.  *waited gets the
 * time it took. */
static bool s2_wait_stopped(INT ms, INT *waited)
{
	*waited = 0;
	while (((drv_encoder_speed_mm_s(RC_SIDE_LEFT) != 0)
		|| (drv_encoder_speed_mm_s(RC_SIDE_RIGHT) != 0))
	       && (*waited < ms)) {
		(void)tk_dly_tsk(10);
		*waited += 10;
	}
	return (drv_encoder_speed_mm_s(RC_SIDE_LEFT) == 0)
	       && (drv_encoder_speed_mm_s(RC_SIDE_RIGHT) == 0);
}

/* Stop everything and let the wheels spin down before the next test. */
static void s2_stop_settle(void)
{
	(void)sub_motion_stop(true);
	(void)tk_dly_tsk(300);
}

#endif /* S2_TEST_MOTORS */

EXPORT void subsystem2_test(void)
{
	TEST_INIT();
	int32_t		h_odo;
	uint32_t	odo;

	/* ============ Motors stay off (safe on the bench) ============ */

	/* S2-U01: after boot, motion control is idle. */
	TEST_CHECK("S2-U01",
		   !sub_motion_busy()
		   && (sub_motion_target_mm_s(RC_SIDE_LEFT) == 0)
		   && (sub_motion_target_mm_s(RC_SIDE_RIGHT) == 0),
		   "motion is idle after boot (not busy, both targets 0)");

	/* S2-U02: the motor driver rejects a wheel that does not exist. */
	TEST_CHECK("S2-U02",
		   (drv_motor_set(S2_BAD_SIDE, 100) == RC_ERR_PARAM)
		   && (drv_motor_get(S2_BAD_SIDE) == 0),
		   "motor driver rejects an invalid wheel");

	/* S2-U03: every reader returns 0 for a wheel that does not exist,
	   instead of reading past the end of an array. */
	TEST_CHECK("S2-U03",
		   (drv_encoder_count(S2_BAD_SIDE) == 0U)
		   && (drv_encoder_period_us(S2_BAD_SIDE) == 0U)
		   && (drv_encoder_speed_mm_s(S2_BAD_SIDE) == 0)
		   && (drv_encoder_dir(S2_BAD_SIDE) == 0)
		   && (drv_encoder_distance_mm(S2_BAD_SIDE) == 0U)
		   && (sub_motion_target_mm_s(S2_BAD_SIDE) == 0),
		   "encoder and motion readers return 0 for an invalid wheel");

	/* S2-U04: the distance per tick in rc_config.h matches the value
	   worked out by hand. */
	tm_printf((UB*)"[TEST] S2-U04 info  RC_ENC_UM_PER_TICK = %u um\n",
		  (UINT)RC_ENC_UM_PER_TICK);
	TEST_CHECK("S2-U04",
		   RC_ENC_UM_PER_TICK == S2_EXP_UM_PER_TICK,
		   "um per encoder tick matches the hand calculation");

	/* S2-U05: ODOMETRY is published every 20 ms (NFR2). */
	odo_count = 0U;
	h_odo = rc_event_subscribe(RC_EVT_ODOMETRY, RC_LANE_SLOW,
				   s2_on_odometry, NULL);
	(void)tk_dly_tsk(S2_ODO_WINDOW_MS);
	if (h_odo >= 0) {
		(void)rc_event_unsubscribe(h_odo);
	}
	odo = odo_count;
	tm_printf((UB*)"[TEST] S2-U05 info  %d ODOMETRY events in %d ms\n",
		  (INT)odo, S2_ODO_WINDOW_MS);
	TEST_CHECK("S2-U05",
		   (h_odo >= 0) && (odo >= S2_ODO_MIN) && (odo <= S2_ODO_MAX),
		   "ODOMETRY published at 50 Hz (NFR2)");

#if S2_TEST_MOTORS
	{
		/* ======== Wheels turn from here on: car on a box! ======== */
		int16_t		duty_a;
		int16_t		duty_b;
		int16_t		duty_c;
		int16_t		duty_d;
		uint32_t	start_l;
		uint32_t	start_r;
		uint32_t	ticks_l;
		uint32_t	ticks_r;
		int32_t		spd_l;
		int32_t		spd_r;
		int32_t		tgt_l;
		int32_t		tgt_r;
		uint32_t	id;
		uint32_t	arc_right;
		uint32_t	arc_left;
		bool		b_busy;
		bool		b_stopped;
		INT		waited;
		rc_result_t	r1;
		rc_result_t	r2;

		/* S2-U06: drv_motor_set clamps to +-1000 permille and
		   remembers what it was told.  Stopped again at once. */
		(void)drv_motor_set(RC_SIDE_LEFT, 1500);
		duty_a = drv_motor_get(RC_SIDE_LEFT);
		(void)drv_motor_set(RC_SIDE_LEFT, -1500);
		duty_b = drv_motor_get(RC_SIDE_LEFT);
		(void)drv_motor_stop(RC_MOTOR_COAST);
		TEST_CHECK("S2-U06",
			   (duty_a == 1000) && (duty_b == -1000),
			   "drv_motor_set clamps to +-1000 permille");

		/* S2-U07: drv_motor_stop leaves both wheels at 0. */
		TEST_CHECK("S2-U07",
			   (drv_motor_get(RC_SIDE_LEFT) == 0)
			   && (drv_motor_get(RC_SIDE_RIGHT) == 0),
			   "drv_motor_stop sets both wheels to 0");

		/* S2-U08: sub_motion_drive applies left = base + steer and
		   right = base - steer AT ONCE (no 20 ms wait), clamped. */
		(void)sub_motion_drive(200, 50);
		duty_a = drv_motor_get(RC_SIDE_LEFT);
		duty_b = drv_motor_get(RC_SIDE_RIGHT);
		(void)sub_motion_drive(900, 300);
		duty_c = drv_motor_get(RC_SIDE_LEFT);
		duty_d = drv_motor_get(RC_SIDE_RIGHT);
		s2_stop_settle();
		TEST_CHECK("S2-U08",
			   (duty_a == 250) && (duty_b == 150)
			   && (duty_c == 1000) && (duty_d == 600),
			   "drive applies base+-steer at once, clamped");

		/* S2-U09 / S2-U10: spin both wheels forward for a while.
		   Both encoders must count, and both must read a positive
		   speed (channel B wired the right way round, A2). */
		start_l = drv_encoder_count(RC_SIDE_LEFT);
		start_r = drv_encoder_count(RC_SIDE_RIGHT);
		(void)sub_motion_drive(S2_SPIN_PERMILLE, 0);
		(void)tk_dly_tsk(S2_SPIN_MS);
		spd_l   = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
		spd_r   = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);
		ticks_l = drv_encoder_count(RC_SIDE_LEFT) - start_l;
		ticks_r = drv_encoder_count(RC_SIDE_RIGHT) - start_r;
		s2_stop_settle();
		tm_printf((UB*)"[TEST] S2-U09 info  ticks in %d ms: L %d, R %d;"
			  "  speed L %d, R %d mm/s\n", S2_SPIN_MS,
			  (INT)ticks_l, (INT)ticks_r, (INT)spd_l, (INT)spd_r);
		TEST_CHECK("S2-U09",
			   (ticks_l >= S2_MIN_SPIN_TICKS)
			   && (ticks_r >= S2_MIN_SPIN_TICKS),
			   "both encoders count while the wheels turn");
		TEST_CHECK("S2-U10",
			   (spd_l > 0) && (spd_r > 0),
			   "both wheels read a positive speed going forward");

		/* S2-U11: once the wheels have stopped, speed reads 0 (no
		   tick for longer than the 100 ms stall timeout).  The
		   wheels are given up to S2_STOP_TMO_MS after
		   s2_stop_settle to come to rest. */
		spd_l = drv_encoder_speed_mm_s(RC_SIDE_LEFT);
		spd_r = drv_encoder_speed_mm_s(RC_SIDE_RIGHT);
		b_stopped = s2_wait_stopped(S2_STOP_TMO_MS, &waited);
		tm_printf((UB*)"[TEST] S2-U11 info  after settle: speed L %d, "
			  "R %d mm/s; read 0 after %d more ms; now L %d, R %d\n",
			  (INT)spd_l, (INT)spd_r, waited,
			  (INT)drv_encoder_speed_mm_s(RC_SIDE_LEFT),
			  (INT)drv_encoder_speed_mm_s(RC_SIDE_RIGHT));
		TEST_CHECK("S2-U11", b_stopped,
			   "speed reads 0 once the wheels have stopped");

		/* S2-U12: drv_encoder_reset zeroes both distances. */
		drv_encoder_reset();
		TEST_CHECK("S2-U12",
			   (drv_encoder_distance_mm(RC_SIDE_LEFT) == 0U)
			   && (drv_encoder_distance_mm(RC_SIDE_RIGHT) == 0U),
			   "drv_encoder_reset zeroes both distances");

		/* S2-U13 / S2-U14: forward move.  It is accepted, busy while
		   it runs, and reports completed under its own id. */
		s2_clear(&move_a);
		id     = sub_motion_forward_mm(S2_MOVE_MM, s2_on_done, &move_a);
		b_busy = sub_motion_busy();
		(void)s2_wait_done(&move_a, S2_MOVE_TMO_MS);
		tm_printf((UB*)"[TEST] S2-U13 info  forward %d mm: reported "
			  "%d mm\n", (INT)S2_MOVE_MM, (INT)move_a.travelled_mm);
		TEST_CHECK("S2-U13",
			   (id != 0U) && b_busy && move_a.b_done
			   && move_a.b_completed && (move_a.move_id == id)
			   && !sub_motion_busy(),
			   "forward move runs and reports completed");
#if S2_CALIBRATED
		TEST_CHECK("S2-U14",
			   s2_near(move_a.travelled_mm, S2_MOVE_MM,
				   (S2_MOVE_MM * S2_DIST_TOL_PCT) / 100U),
			   "forward distance within tolerance (NFR12)");
#else
		tm_printf((UB*)"[TEST] S2-U14 SKIP  needs S2_CALIBRATED 1\n");
#endif
		s2_stop_settle();

		/* S2-U15: backward move drives both wheels in reverse. */
		s2_clear(&move_a);
		id    = sub_motion_backward_mm(S2_MOVE_MM, s2_on_done, &move_a);
		tgt_l = s2_wait_target(RC_SIDE_LEFT, 100);
		tgt_r = sub_motion_target_mm_s(RC_SIDE_RIGHT);
		(void)s2_wait_done(&move_a, S2_MOVE_TMO_MS);
		TEST_CHECK("S2-U15",
			   (id != 0U) && (tgt_l < 0) && (tgt_r < 0)
			   && move_a.b_done && move_a.b_completed,
			   "backward move reverses both wheels and completes");
		s2_stop_settle();

		/* S2-U16: turn right 90: left wheel forward, right back. */
		s2_clear(&move_a);
		id    = sub_motion_turn_deg(90, s2_on_done, &move_a);
		tgt_l = s2_wait_target(RC_SIDE_LEFT, 100);
		tgt_r = sub_motion_target_mm_s(RC_SIDE_RIGHT);
		(void)s2_wait_done(&move_a, S2_MOVE_TMO_MS);
		arc_right = move_a.travelled_mm;
		TEST_CHECK("S2-U16",
			   (id != 0U) && (tgt_l > 0) && (tgt_r < 0)
			   && move_a.b_done && move_a.b_completed,
			   "turn right 90: left fwd, right back, completes");
		s2_stop_settle();

		/* S2-U17: turn left 90: left wheel back, right forward. */
		s2_clear(&move_a);
		id    = sub_motion_turn_deg(-90, s2_on_done, &move_a);
		tgt_l = s2_wait_target(RC_SIDE_LEFT, 100);
		tgt_r = sub_motion_target_mm_s(RC_SIDE_RIGHT);
		(void)s2_wait_done(&move_a, S2_MOVE_TMO_MS);
		arc_left = move_a.travelled_mm;
		TEST_CHECK("S2-U17",
			   (id != 0U) && (tgt_l < 0) && (tgt_r > 0)
			   && move_a.b_done && move_a.b_completed,
			   "turn left 90: left back, right fwd, completes");
		s2_stop_settle();

		/* S2-U18: each wheel ran the expected arc for 90 degrees. */
		tm_printf((UB*)"[TEST] S2-U18 info  arc per wheel: right turn "
			  "%d mm, left turn %d mm (expected %d)\n",
			  (INT)arc_right, (INT)arc_left, (INT)S2_EXP_ARC_90_MM);
#if S2_CALIBRATED
		TEST_CHECK("S2-U18",
			   s2_near(arc_right, S2_EXP_ARC_90_MM, S2_ARC_TOL_MM)
			   && s2_near(arc_left, S2_EXP_ARC_90_MM,
				      S2_ARC_TOL_MM),
			   "90 degree turn arc within tolerance (FR3)");
#else
		tm_printf((UB*)"[TEST] S2-U18 SKIP  needs S2_CALIBRATED 1\n");
#endif

		/* S2-U19: a new move pre-empts the running one, which then
		   reports "not completed"; the new one completes. */
		s2_clear(&move_a);
		s2_clear(&move_b);
		(void)sub_motion_forward_mm(1000U, s2_on_done, &move_a);
		id = sub_motion_forward_mm(S2_MOVE_MM, s2_on_done, &move_b);
		(void)s2_wait_done(&move_b, S2_MOVE_TMO_MS);
		TEST_CHECK("S2-U19",
			   move_a.b_done && !move_a.b_completed
			   && move_b.b_done && move_b.b_completed
			   && (move_b.move_id == id),
			   "new move pre-empts old; old reports not done");
		s2_stop_settle();

		/* S2-U20: while a queued move runs, the continuous drive
		   modes are refused instead of cutting it short. */
		s2_clear(&move_a);
		(void)sub_motion_forward_mm(1000U, s2_on_done, &move_a);
		r1 = sub_motion_drive(300, 0);
		r2 = sub_motion_drive_speed(150, 0);
		TEST_CHECK("S2-U20",
			   (r1 == RC_ERR_BUSY) && (r2 == RC_ERR_BUSY),
			   "drive / drive_speed refused during a queued move");

		/* S2-U21: sub_motion_stop cancels that move, which reports
		   "not completed", and leaves motion idle. */
		r1 = sub_motion_stop(true);
		(void)s2_wait_done(&move_a, 1000);
		TEST_CHECK("S2-U21",
			   (r1 == RC_OK) && move_a.b_done && !move_a.b_completed
			   && !sub_motion_busy(),
			   "sub_motion_stop cancels the move (not completed)");
		s2_stop_settle();

		/* S2-U22: drive_speed asks for left = base + steer and
		   right = base - steer, in mm/s. */
		r1    = sub_motion_drive_speed(150, 30);
		tgt_l = s2_wait_target(RC_SIDE_LEFT, 100);
		tgt_r = sub_motion_target_mm_s(RC_SIDE_RIGHT);
		s2_stop_settle();
		TEST_CHECK("S2-U22",
			   (r1 == RC_OK) && (tgt_l == 180) && (tgt_r == 120),
			   "drive_speed targets base+-steer mm/s");

		/* Leave the car stopped for the next subsystem's tests. */
		(void)sub_motion_stop(true);
	}
#endif /* S2_TEST_MOTORS */

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
