/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5 (scan and avoidance).
 *	Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01 ...
 *
 *	Works on a bare Pico W:
 *	  servo   signal GP15, + to VBUS (pin 40), - to GND
 *	  HC-SR04 optional: TRIG GP2, ECHO GP3 through the 1k/2k divider
 *	With no sensor every ping simply times out, which still exercises the
 *	whole trigger -> timer alarm -> timeout -> deferred publish chain.
 *
 *	Watch the servo while S5-U09 runs: it should stop at 70, 80, 90, 100 
 * / and 110 degrees (plus a 5-degree fine sweep if something is closer 
 * / than the obstacle threshold). The servo only swings ±20 degrees.
 */

#include "../rc_prelude.h"	/* must be first: size_t clash, see the file */
#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

#include "../rc_defer.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_gpioirq.h"
#include "../rc_sense.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "sub_scan.h"
#include "../sub_nav.h"

/*
 * 0 now that the shared core and subsystem 5's drivers are brought up by
 * the system itself: app_main.c runs rc_defer/rc_event/rc_gpioirq in
 * Phase 0, and subsystem5_init() runs drv_servo_init / drv_ultrasonic_init
 * / sub_scan_init in Phase 1.  Re-initialising them here would create
 * duplicate kernel objects and double-register the ECHO IRQ, so the init
 * checks (S5-U01/U02/U05/U08) are dropped when this is 0; usermain()'s
 * CHECK() reports any init failure instead.
 *
 * Set back to 1 ONLY to run this test on a bare template app_main.c that
 * does not call subsystem5_init() -- then the test brings everything up
 * itself.  On this repo's app_main.c (which does call it), keep it 0.
 */
#ifndef S5_TEST_STANDALONE
#define S5_TEST_STANDALONE	0
#endif

/*
 * Set to 1 for a live bench after the tests finish: the servo sweeps
 * 70 -> 110 -> 70 degrees in 10-degree steps, pausing at each stop, and
 * prints that angle's range, forever.  Use it to SEE the servo turn and
 * to check distances with a hand or a box.  Put it back to 0 before
 * committing, so other buddies' TEST=1 runs stay quiet.
 */
#ifndef S5_LIVE_BENCH
#define S5_LIVE_BENCH	0
#endif

/* Stops the compiler moving memory accesses across this point, so a
 * result copied in a callback is complete before its "done" flag is
 * seen, and is re-read after it. */
#define S5_BARRIER()	__asm volatile ("" ::: "memory")

/* After rc_sense_test_hold(true): lets line samples already published
 * reach sub_line on both dispatchers before the test forces a state. */
#define S5_HOLD_SETTLE_MS	(100)

/* Written by the callbacks (dispatcher / scan task), read by the test. */
static volatile uint32_t ultra_count;
static volatile int16_t  ultra_angle;
static volatile uint16_t ultra_range;
static volatile bool     ultra_valid;

static volatile bool     scan_done;
static rc_pl_profile_t   scan_profile;
static rc_pl_plan_t      scan_plan;

static void on_ultra(const rc_event_t *evt, void *ctx)
{
	(void)ctx;
	ultra_angle = evt->u.ultra.angle_deg;
	ultra_range = evt->u.ultra.range_mm;
	ultra_valid = evt->u.ultra.valid;
	S5_BARRIER();
	ultra_count++;
}

static void on_scan(const rc_pl_profile_t *p, const rc_pl_plan_t *plan,
		    void *ctx)
{
	(void)ctx;
	scan_profile = *p;
	scan_plan    = *plan;
	S5_BARRIER();
	scan_done = true;
}

/* Sleep in 10 ms steps until ultra_count passes `from`, or `ms` runs out. */
static bool wait_ultra(uint32_t from, INT ms)
{
	while ((ultra_count == from) && (ms > 0)) {
		(void)tk_dly_tsk(10);
		ms -= 10;
	}
	S5_BARRIER();
	return ultra_count != from;
}

static bool wait_scan(INT ms)
{
	while (!scan_done && (ms > 0)) {
		(void)tk_dly_tsk(10);
		ms -= 10;
	}
	S5_BARRIER();
	return scan_done;
}

/* Sleep in 10 ms steps until sub_nav's state is (is == true) or is no
 * longer (is == false) `s`, or `ms` runs out. */
static bool wait_nav(rc_nav_state_t s, bool is, INT ms)
{
	while (((sub_nav_state() == s) != is) && (ms > 0)) {
		(void)tk_dly_tsk(10);
		ms -= 10;
	}
	return (sub_nav_state() == s) == is;
}

/* FR9: counts sub_nav's RC_EVT_RECOVERY_FAILED (S5-N11, S5-N12). */
static volatile uint32_t recov_failed;

static void on_recov_failed(const rc_event_t *evt, void *ctx)
{
	(void)evt;
	(void)ctx;
	recov_failed++;
}

/* Publish what Buddy 3's sub_barcode would on a decode. */
static void fake_barcode(char symbol, rc_nav_cmd_t cmd)
{
	rc_event_t e = { 0 };

	e.id                 = RC_EVT_BARCODE_DECODED;
	e.u.barcode.symbol   = symbol;
	e.u.barcode.command  = cmd;
	e.u.barcode.reversed = false;
	(void)rc_event_publish(&e);
}

/* Publish the line samples of a junction: both sensors black (1/1) for
 * longer than sub_nav's NAV_JUNCTION_CONFIRM (3 samples). */
static void fake_junction(void)
{
	rc_event_t e = { 0 };
	int        i;

	e.id                = RC_EVT_LINE_SAMPLE;
	e.u.line.on_line_l  = true;
	e.u.line.on_line_r  = true;
	e.u.line.position   = 0;
	for (i = 0; i < 5; i++) {
		(void)rc_event_publish(&e);
	}
}

/* True if nav is busy with the command (EXECUTING_CMD) or has already
 * stopped because sub_motion refused or aborted the move. Either means
 * the command was taken, with or without Buddy 2's motion wired. */
static bool nav_took_command(void)
{
	rc_nav_state_t s = sub_nav_state();

	return (s == RC_NAV_EXECUTING_CMD) || (s == RC_NAV_STOPPED);
}

/* Back to STOPPED between nav tests. */
static void nav_reset(void)
{
	(void)sub_nav_stop();
	(void)wait_nav(RC_NAV_STOPPED, true, 300);
}

static const char *plan_name(rc_nav_cmd_t c)
{
	switch (c) {
	case RC_CMD_TURN_LEFT:   return "TURN_LEFT";
	case RC_CMD_TURN_RIGHT:  return "TURN_RIGHT";
	case RC_CMD_GO_STRAIGHT: return "GO_STRAIGHT";
	case RC_CMD_U_TURN:      return "U_TURN";
	case RC_CMD_STOP:        return "STOP";
	default:                 return "NONE";
	}
}

#if S5_LIVE_BENCH
/* Live bench task (low priority, its own task so the test run can end
 * normally).  One stop every ~0.7 s so the movement is easy to see. */
static void s5_live_task(INT stacd, void *exinf)
{
	static const int16_t stops[] = { 70, 80, 90, 100, 110, 100, 90, 80 };
	uint32_t i = 0U;
	uint32_t before;
	int16_t  from;
	int16_t  to;

	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"[S5] live bench: servo sweeps 70..110, one range per stop\n");
	for (;;) {
		from = drv_servo_get_angle();
		to   = stops[i];
		i    = (i + 1U) % (sizeof(stops) / sizeof(stops[0]));

		(void)drv_servo_set_angle(to);
		(void)tk_dly_tsk((RELTIM)drv_servo_settle_ms(from, to) + 500);

		before = ultra_count;
		if ((drv_ultrasonic_ping(to) == RC_OK) && wait_ultra(before, 100)) {
			if (ultra_valid) {
				tm_printf((UB*)"[S5] %3d deg: %4d mm\n",
					  (INT)to, (INT)ultra_range);
			} else {
				tm_printf((UB*)"[S5] %3d deg: no echo\n", (INT)to);
			}
		} else {
			tm_printf((UB*)"[S5] %3d deg: ping failed\n", (INT)to);
		}
	}
}
#endif /* S5_LIVE_BENCH */

EXPORT void subsystem5_test(void)
{
	rc_result_t r1;
	rc_result_t r2;
	bool        busy;
	bool        got;
	uint32_t    before;
	int16_t     a_hi;
	int16_t     a_lo;

	TEST_INIT();

#if S5_TEST_STANDALONE
	/* S5-U01: the shared core our drivers sit on starts cleanly.
	 * Same order as the old app_main.c. */
	TEST_CHECK("S5-U01",
		   (rc_defer_init() == RC_OK)
		   && (rc_event_init() == RC_OK)
		   && (rc_gpioirq_init() == RC_OK),
		   "shared core starts (defer list, event bus, GPIO IRQ)");

	/* S5-U02: servo PWM comes up and centres. */
	TEST_CHECK("S5-U02",
		   (drv_servo_init() == RC_OK) && (drv_servo_get_angle() == 90),
		   "servo PWM starts on GP15 and centres at 90 deg");
#endif

	/* S5-U12: the signal really exists in hardware, not just in our
	 * variables: the PWM block is out of reset, GP15 is routed to PWM,
	 * and the GP15 slice counter is moving. */
	{
		const UW slice = (RC_PIN_SERVO_SCAN >> 1) & 0x07U;
		const UW ctr   = PWM_BASE + PWM_CHx_CTR + (slice * 0x14U);
		UW c1;
		UW c2;

		c1 = in_w(ctr);
		(void)tk_dly_tsk(2);
		c2 = in_w(ctr);
		TEST_CHECK("S5-U12",
			   ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) != 0U)
			   && ((in_w(GPIO_CTRL(RC_PIN_SERVO_SCAN)) & 0x1FU)
			       == GPIO_CTRL_FUNCSEL_PWM)
			   && (c1 != c2),
			   "GP15 is outputting PWM (block live, pin routed, counter running)");
	}

	/* S5-U03: out-of-range angles are clamped, never passed through. */
	(void)drv_servo_set_angle(200);
	a_hi = drv_servo_get_angle();
	(void)drv_servo_set_angle(-30);
	a_lo = drv_servo_get_angle();
	(void)drv_servo_set_angle(90);
	TEST_CHECK("S5-U03",
		   (a_hi == RC_SERVO_ANGLE_MAX) && (a_lo == RC_SERVO_ANGLE_MIN)
		   && (drv_servo_get_angle() == 90),
		   "servo clamps commands to 70..110 deg (±20 deg swing)");

	/* S5-U04: the settle estimate behaves sensibly.  Checks properties,
	 * not exact numbers, so it keeps passing after US_PER_DEG is tuned. */
	TEST_CHECK("S5-U04",
		   (drv_servo_settle_ms(90, 90) > 0U)
		   && (drv_servo_settle_ms(70, 110) == drv_servo_settle_ms(110, 70))
		   && (drv_servo_settle_ms(70, 110) > drv_servo_settle_ms(80, 90)),
		   "settle time is positive, symmetric and grows with travel");

#if S5_TEST_STANDALONE
	/* S5-U05: ultrasonic claims its pins, IRQ and both timer alarms. */
	TEST_CHECK("S5-U05", drv_ultrasonic_init() == RC_OK,
		   "ultrasonic driver claims TRIG/ECHO, IRQ and timer alarms");
#endif

	/* S5-U06 / U07: one ping, end to end.  Every ping must end in a
	 * result event (a range, or "invalid" after the 30 ms timeout). */
	(void)rc_event_subscribe(RC_EVT_ULTRA_RESULT, RC_LANE_SLOW,
				 on_ultra, NULL);
	(void)tk_dly_tsk(500);			/* let the servo reach 90 */

	before = ultra_count;
	r1   = drv_ultrasonic_ping(90);
	busy = drv_ultrasonic_busy();
	r2   = drv_ultrasonic_ping(90);		/* must be refused */
	got  = wait_ultra(before, 100);

	TEST_CHECK("S5-U06",
		   (r1 == RC_OK) && got && (ultra_angle == 90),
		   "a ping ends in a result event within 100 ms, tagged 90 deg");
	TEST_CHECK("S5-U07",
		   busy && (r2 == RC_ERR_BUSY),
		   "a second ping while one is in flight is refused");

	if (got && ultra_valid) {
		tm_printf((UB*)"[INFO] S5 range straight ahead: %d mm\n",
			  (INT)ultra_range);
	} else {
		tm_printf((UB*)"[INFO] S5 straight ahead: no echo"
			  " (no sensor wired, or nothing within range)\n");
	}

#if S5_TEST_STANDALONE
	/* S5-U08: scan task and its event flag are created. */
	TEST_CHECK("S5-U08", sub_scan_init() == RC_OK,
		   "scan task starts");
#endif

	/* S5-U09: a whole coarse (+ fine, if something is close) scan runs
	 * and reports.  Coarse is 70..110 deg in 10-deg steps = 5 bearings;
	 * every bearing is recorded, even when it timed out. */
	scan_done = false;
	(void)sub_scan_on_complete(on_scan, NULL);
	r1  = sub_scan_start();
	got = wait_scan(5000);

	TEST_CHECK("S5-U09",
		   (r1 == RC_OK) && got && (scan_profile.n_points >= 5U),
		   "full scan finishes within 5 s with all 5 coarse bearings");

	if (got) {
		tm_printf((UB*)"[INFO] S5 profile: %d points, closest %d mm"
			  " at %d deg, width %d mm, clear L=%d R=%d\n",
			  (INT)scan_profile.n_points,
			  (INT)scan_profile.closest_mm,
			  (INT)scan_profile.closest_angle_deg,
			  (INT)scan_profile.width_mm,
			  (INT)scan_profile.clearance_left_mm,
			  (INT)scan_profile.clearance_right_mm);
		tm_printf((UB*)"[INFO] S5 plan: %s lateral %d mm forward %d mm\n",
			  plan_name(scan_plan.action),
			  (INT)scan_plan.lateral_mm, (INT)scan_plan.forward_mm);
	}

	(void)sub_scan_on_complete(NULL, NULL);
	(void)drv_servo_set_angle(90);

	/* ---- S5-N01..N06: sub_nav skeleton, obstacle path -------------
	 * usermain() ran sub_nav_init() but, in a TEST=1 build, NOT
	 * sub_nav_start(), so nav is still IDLE here.  Works on a bare board:
	 * with Buddy 2's sub_motion not wired yet, a bypass is refused and
	 * nav goes to STOPPED, which is a valid end of the obstacle path. */

	/* S5-N01: nav ignores events before the run starts (S1's tests
	 * published fake IMPACT and STOP events earlier in this run). */
	TEST_CHECK("S5-N01", sub_nav_state() == RC_NAV_IDLE,
		   "sub_nav is IDLE before sub_nav_start()");

	/* S5-N02: start turns on FOLLOWING (line, barcode, watch). */
	TEST_CHECK("S5-N02",
		   (sub_nav_start() == RC_OK)
		   && (sub_nav_state() == RC_NAV_FOLLOWING),
		   "sub_nav_start(): IDLE -> FOLLOWING");

	/* S5-N03: a close watch reading starts avoidance.  Faked, so it
	 * passes with or without a sensor. */
	{
		rc_event_t e = { 0 };

		e.id              = RC_EVT_ULTRA_RESULT;
		e.u.ultra.angle_deg = 90;
		e.u.ultra.range_mm  = 200U;
		e.u.ultra.valid     = true;
		(void)rc_event_publish(&e);
	}
	TEST_CHECK("S5-N03", wait_nav(RC_NAV_AVOIDING, true, 200),
		   "200 mm ahead while FOLLOWING -> AVOIDING within 200 ms");

	/* S5-N04: the scan finishes and its plan is acted on: FOLLOWING
	 * (nothing close), STOPPED (no gap, or bypass refused) or
	 * RECOVERING (bypass driven, once sub_motion is wired). */
	got = wait_nav(RC_NAV_AVOIDING, false, 20000);	/* step scan ~6-10 s */
	TEST_CHECK("S5-N04", got,
		   "scan plan handled: nav leaves AVOIDING within 20 s");
	tm_printf((UB*)"[INFO] S5 nav after plan: %s\n",
		  sub_nav_state_name(sub_nav_state()));

	/* S5-N05: STOP from a running state (FR10b path, via the bus). */
	(void)sub_nav_stop();
	(void)wait_nav(RC_NAV_STOPPED, true, 200);
	r1 = sub_nav_start();				/* back to FOLLOWING */
	r2 = sub_nav_stop();
	TEST_CHECK("S5-N05",
		   (r1 == RC_OK) && (r2 == RC_OK)
		   && wait_nav(RC_NAV_STOPPED, true, 200) && !sub_scan_busy(),
		   "sub_nav_stop(): FOLLOWING -> STOPPED, no scan left running");

	/* S5-N06: once STOPPED, a close reading is ignored. */
	{
		rc_event_t e = { 0 };

		e.id              = RC_EVT_ULTRA_RESULT;
		e.u.ultra.angle_deg = 90;
		e.u.ultra.range_mm  = 200U;
		e.u.ultra.valid     = true;
		(void)rc_event_publish(&e);
	}
	(void)tk_dly_tsk(200);
	TEST_CHECK("S5-N06", sub_nav_state() == RC_NAV_STOPPED,
		   "a close reading while STOPPED does not restart avoidance");

	/* ---- S5-N08..N13: barcode, remote commands, state timeouts -----
	 * All events are faked, so these pass on a bare board.  ONCE BUDDY
	 * 2'S MOTION IS WIRED THE WHEELS TURN during N09, N11 and N13: run
	 * the test image with the car on a stand. */
	(void)rc_event_subscribe(RC_EVT_RECOVERY_FAILED, RC_LANE_FAST,
				 on_recov_failed, NULL);

	/* S5-N08: a Code 39 start character while FOLLOWING opens
	 * READING_BARCODE (report §6.5). */
	r1 = sub_nav_start();
	fake_barcode('*', RC_CMD_NONE);
	TEST_CHECK("S5-N08",
		   (r1 == RC_OK) && wait_nav(RC_NAV_READING_BARCODE, true, 200),
		   "start character while FOLLOWING -> READING_BARCODE");

	/* S5-N09: the data character 'A' is stored, not run (2026-10-29):
	 * nav goes back to FOLLOWING, and at the next junction (line pair
	 * 1/1) runs the left turn (FR3): EXECUTING_CMD, or STOPPED if
	 * sub_motion refuses the move.  Real line samples are held, so what
	 * is under the sensors can't make the junction early or late. */
	rc_sense_test_hold(true);
	(void)tk_dly_tsk(S5_HOLD_SETTLE_MS);
	fake_barcode('A', RC_CMD_TURN_LEFT);
	got = wait_nav(RC_NAV_FOLLOWING, true, 200);
	tm_printf((UB*)"[INFO] S5 nav after barcode A: %s (stored)\n",
		  sub_nav_state_name(sub_nav_state()));
	fake_junction();
	got = got && wait_nav(RC_NAV_FOLLOWING, false, 200) && nav_took_command();
	rc_sense_test_hold(false);
	TEST_CHECK("S5-N09", got,
		   "barcode 'A' stored, then junction (1/1) -> EXECUTING_CMD (or STOPPED if refused)");
	tm_printf((UB*)"[INFO] S5 nav after junction: %s\n",
		  sub_nav_state_name(sub_nav_state()));
	nav_reset();

	/* S5-N10: a start character with no data character times out back
	 * to FOLLOWING (G10 decode timeout, 2 s). */
	r1 = sub_nav_start();
	fake_barcode('*', RC_CMD_NONE);
	got = wait_nav(RC_NAV_READING_BARCODE, true, 200);
	TEST_CHECK("S5-N10",
		   (r1 == RC_OK) && got
		   && wait_nav(RC_NAV_FOLLOWING, true, 2600),
		   "READING_BARCODE with no data -> FOLLOWING within 2.6 s");
	nav_reset();

	/* S5-N11 / S5-N12 drive the line state by hand, so the real line
	 * samples are held: otherwise whatever is under the sensors (a stand
	 * reads "11", on the line) makes sub_line report the line found and
	 * cancels RECOVERING before the 3 s are up.  Released after N12. */
	rc_sense_test_hold(true);
	(void)tk_dly_tsk(S5_HOLD_SETTLE_MS);

	/* S5-N11: FR9 - RECOVERING with no line found gives up after 3 s:
	 * STOPPED plus one RC_EVT_RECOVERY_FAILED. */
	before = recov_failed;
	r1 = sub_nav_test_force(RC_NAV_RECOVERING);
	got = (r1 == RC_OK) && wait_nav(RC_NAV_STOPPED, true, 3500);
	S5_BARRIER();
	TEST_CHECK("S5-N11", got && (recov_failed == before + 1U),
		   "RECOVERING, line not found -> STOPPED + RECOVERY_FAILED in 3 s");
	/* No nav_reset() here: N11 already ends STOPPED, and the STOP that
	 * sub_nav_stop() queues would land after N12 forces RECOVERING. */

	/* S5-N12: finding the line cancels the FR9 timer: FOLLOWING, and
	 * still FOLLOWING (no RECOVERY_FAILED) after the 3 s have passed. */
	before = recov_failed;
	r1 = sub_nav_test_force(RC_NAV_RECOVERING);
	{
		rc_event_t e = { 0 };

		e.id = RC_EVT_LINE_REACQUIRED;
		(void)rc_event_publish(&e);
	}
	got = (r1 == RC_OK) && wait_nav(RC_NAV_FOLLOWING, true, 200);
	(void)tk_dly_tsk(3500);
	S5_BARRIER();
	TEST_CHECK("S5-N12",
		   got && (sub_nav_state() == RC_NAV_FOLLOWING)
		   && (recov_failed == before),
		   "LINE_REACQUIRED in RECOVERING -> FOLLOWING, timer cancelled");
	nav_reset();
	rc_sense_test_hold(false);	/* real line samples flow again */

	/* S5-N13: a remote turn is executed while following (FR10b), and
	 * ignored once STOPPED: only STOP is honoured from every state. */
	r1 = sub_nav_start();
	(void)sub_nav_inject_command(RC_CMD_TURN_RIGHT, 0);
	got = (r1 == RC_OK) && wait_nav(RC_NAV_FOLLOWING, false, 200)
	      && nav_took_command();
	nav_reset();
	(void)sub_nav_inject_command(RC_CMD_TURN_LEFT, 0);
	(void)tk_dly_tsk(200);
	TEST_CHECK("S5-N13", got && (sub_nav_state() == RC_NAV_STOPPED),
		   "remote turn runs from FOLLOWING, is ignored when STOPPED");

	(void)drv_servo_set_angle(90);

	TEST_SUMMARY("subsystem5");

#if S5_LIVE_BENCH
	(void)app_start_task((FP)s5_live_task, S5_PRI_MAIN, 0, "s5_live");
#endif
}

#endif /* APP_TEST */
