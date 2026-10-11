/*
 *	subsystem5.c
 *	Subsystem 5 -- Ultrasonic scanning and obstacle avoidance.
 *	Student ID: 2500347.
 *
 *	The template's two entry points. usermain() runs every _init() first,
 *	then every _start(), so all kernel objects exist before any task runs
 *	(app.h).  app_main.c brings up the shared core (rc_defer, rc_event,
 *	rc_gpioirq) in Phase 0, BEFORE this _init() runs, which the drivers
 *	below rely on (drv_ultrasonic uses rc_gpioirq + rc_defer; sub_scan
 *	subscribes to the event bus).
 *
 *	NOTE: the S5 modules return rc_result_t (RC_OK == 0); the template
 *	wants ER.  RC_OK becomes E_OK, anything else E_SYS so usermain()'s
 *	CHECK() flags it (same convention as subsystem1.c).
 *
 *	Init order is fixed by the drivers: servo and ultrasonic hardware
 *	must be up before sub_scan_init(), which registers the ultrasonic
 *	result callback and creates the Scan task (see sub_scan.c).
 *
 *	What this subsystem does NOT start here: the forward watch and full
 *	coarse/fine scans.  Per report §6.5 those are driven by sub_nav
 *	(sub_nav_set/sub_scan_set_watch while FOLLOWING, sub_scan_start() on a
 *	close reading).  sub_nav is the Step-S integration piece and does not
 *	exist on this branch yet, so S5 comes up idle: the Scan task is
 *	created and parked, drivers are ready, and nothing pings until sub_nav
 *	(or the TEST=1 self-test) asks.  This also keeps a TEST=1 build clean
 *	-- no background pings competing with subsystem5_test()'s own pings.
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "sub_scan.h"

EXPORT ER subsystem5_init(void)
{
	/* Servo PWM first (centres at 90 deg), then the ultrasonic driver
	 * (claims TRIG/ECHO, the ECHO IRQ and two TIMER alarms), then the
	 * scan module (registers the ultrasonic callback and creates the
	 * Scan task).  sub_scan_init() REQUIRES the two drivers above it. */
	if (drv_servo_init() != RC_OK) {
		return E_SYS;
	}
	if (drv_ultrasonic_init() != RC_OK) {
		return E_SYS;
	}
	if (sub_scan_init() != RC_OK) {
		return E_SYS;
	}
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	/* Nothing to start here.  sub_scan_init() already created and
	 * started the Scan task (it parks in S_IDLE waiting on its event
	 * flag).  The forward watch and avoidance scans are turned on by
	 * sub_nav during FOLLOWING (report §6.5); until sub_nav exists,
	 * leaving S5 idle is the faithful boot-time behaviour. */
	return E_OK;
}
