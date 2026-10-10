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
 *	Watch the servo while S5-U09 runs: it should stop at 30, 60, 90, 120
 *	and 150 degrees (plus a 6-degree fine sweep if something is closer
 *	than the obstacle threshold).
 */

#include "../rc_prelude.h"	/* must be first: size_t clash, see the file */
#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

#include "../rc_defer.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_gpioirq.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "sub_scan.h"

/*
 * 1 while app_main.c is still the template's: nothing else brings up the
 * shared core (defer list, event bus, GPIO IRQ) or subsystem 5's drivers,
 * so this test does it.  Set to 0 once Step S's app_main.c is merged:
 * that file initialises all of these itself, and a second init would
 * create duplicate kernel objects.
 */
#ifndef S5_TEST_STANDALONE
#define S5_TEST_STANDALONE	1
#endif

/*
 * Set to 1 for a live bench after the tests finish: the servo sweeps
 * 30 -> 150 -> 30 degrees in 30-degree steps, pausing at each stop, and
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
	static const int16_t stops[] = { 30, 60, 90, 120, 150, 120, 90, 60 };
	uint32_t i = 0U;
	uint32_t before;
	int16_t  from;
	int16_t  to;

	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"[S5] live bench: servo sweeps 30..150, one range per stop\n");
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
		   "servo clamps commands to 0..180 deg");

	/* S5-U04: the settle estimate behaves sensibly.  Checks properties,
	 * not exact numbers, so it keeps passing after US_PER_DEG is tuned. */
	TEST_CHECK("S5-U04",
		   (drv_servo_settle_ms(90, 90) > 0U)
		   && (drv_servo_settle_ms(30, 150) == drv_servo_settle_ms(150, 30))
		   && (drv_servo_settle_ms(30, 150) > drv_servo_settle_ms(60, 90)),
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
	 * and reports.  Coarse is 30..150 deg in 30-deg steps = 5 bearings;
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

	TEST_SUMMARY("subsystem5");

#if S5_LIVE_BENCH
	(void)app_start_task((FP)s5_live_task, S5_PRI_MAIN, 0, "s5_live");
#endif
}

#endif /* APP_TEST */
