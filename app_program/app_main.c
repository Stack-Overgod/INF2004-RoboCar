/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in phases:
 *	  0. the shared layer (rc_defer, rc_event, rc_gpioirq, rc_fault)
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	then starts the Sense task and the watchdog, and puts the initial
 *	task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The subsystems, one per folder
 *	------------------------------
 *	  subsystem1/  WiFi/MQTT telemetry and remote commands
 *	  subsystem2/  motors, wheel encoders, wheel-speed PID, moves/turns
 *	  subsystem3/  IR line following and Code 39 barcode
 *	  subsystem4/  IMU: hump/terrain, lift/overturn detection
 *	  subsystem5/  ultrasonic + servo scan, obstacle avoidance
 *	  sub_nav.c    the mission state machine (shared)
 *
 *	They talk through the event bus (rc_event.h): a subsystem publishes
 *	an event and whoever needs it subscribes.  The interfaces are listed
 *	in TEAM.md section 5.
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#include "rc_defer.h"		/* shared layer: interrupt bottom halves */
#include "rc_event.h"		/* shared layer: the event bus */
#include "rc_gpioirq.h"		/* shared layer: one GPIO interrupt, many pins */
#include "rc_fault.h"		/* shared layer: fault bitmask (event flag) */
#include "sub_nav.h"		/* shared layer: mission state machine (report §6.5) */
#include "rc_sense.h"		/* shared layer: the one Sense task (report §3) */
#include "rc_wdog.h"		/* shared layer: watchdog + reset reason */
#include "rc_config.h"		/* shared layer: RC_RISK1_SOAK_SECS, pins */
#include "subsystem2/sub_motion.h"	/* TEST summary: motion cycle time */

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

/*
 * The USB console enables the USB controller interrupt (USBCTRL_IRQ, 5)
 * without setting a priority, so it stays at the NVIC reset level 0, above
 * PendSV. Its handler wakes the USB service task, so it is moved to the
 * PendSV level (3), where it cannot preempt a task switch. The NVIC is per
 * core and the console takes this interrupt on core 0, so the write runs in
 * a short task pinned to core 0.
 */
#define INTNO_USBCTRL		(5U)
#define INTPRI_PENDSV_LEVEL	(3U)

LOCAL void usb_irq_prio_task(INT stacd, void *exinf)
{
	UW	reg = NVIC_IPR((INTNO_USBCTRL / 4U) * 4U);
	UINT	shift = (INTNO_USBCTRL % 4U) * 8U;
	UINT	sts;

	(void)stacd;
	(void)exinf;
	DI(sts);
	out_w(reg, (in_w(reg) & ~(0xFFUL << shift))
		   | ((UW)(INTPRI_PENDSV_LEVEL << 6) << shift));
	EI(sts);
	tm_printf((UB*)"[irq] USB interrupt priority %u\n",
		  (UINT)((in_w(reg) >> (shift + 6U)) & 3U));
	tk_ext_tsk();
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 0: the shared layer every subsystem builds on.  The event bus
	   must be running before any subsystemN_init() subscribes to it, and
	   rc_defer before any driver registers a bottom half.  Each of these
	   is safe to call again later (the second call does nothing). */
	if (rc_defer_init() != RC_OK) {
		tm_printf((UB*)"ERROR rc_defer_init\n");
	}
	if (rc_event_init() != RC_OK) {
		tm_printf((UB*)"ERROR rc_event_init\n");
	}
	if (rc_gpioirq_init() != RC_OK) {
		tm_printf((UB*)"ERROR rc_gpioirq_init\n");
	}
	if (rc_fault_init() != RC_OK) {		/* before any driver sets a fault */
		tm_printf((UB*)"ERROR rc_fault_init\n");
	}
	rc_wdog_boot();				/* report a watchdog reboot */
#if TK_SUPPORT_SMP
	(void)app_start_task((FP)usb_irq_prio_task, 1, TP_PRC1, "usb_irq_prio");
#else
	(void)app_start_task((FP)usb_irq_prio_task, 1, 0, "usb_irq_prio");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");

	/* Phase 1b: the mission state machine subscribes to everyone's
	   events.  After every subsystemN_init(), because its handlers call
	   sub_motion, sub_line, sub_barcode and sub_scan. */
	if (sub_nav_init() != RC_OK) {
		tm_printf((UB*)"ERROR sub_nav_init\n");
	}

	/* Phase 2: every subsystem starts its tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	/* Phase 2b: the shared Sense task polls the IR line pair (S3, 10 ms)
	   and the IMU (S4, 10 ms); the barcode ADC streams by FIFO interrupt
	   (drv_ir.c, RC_BAR_ADC_FIFO).  After every subsystemN_start(), so S4's
	   boot calibration has finished before the first IMU sample. */
	if (rc_sense_start() != RC_OK) {
		tm_printf((UB*)"ERROR rc_sense_start\n");
	}

	/* Phase 2c: every task that checks in is running: arm the watchdog */
	if (rc_wdog_start() != RC_OK) {
		tm_printf((UB*)"ERROR rc_wdog_start\n");
	}

	tm_printf((UB*)"all subsystems started\n\n");

#if !APP_TEST
	/* Phase 3: begin the run (sub_nav_start, report §6.5).  Not in a
	   TEST=1 build: the forward watch would ping in the background and
	   collide with the subsystem tests' own pings.  subsystem5_test()
	   starts and stops nav itself (S5-N02..N05). */
#if TM_MQTT
	/* MQTT build: stay in IDLE, motors off, until START (G) arrives on
	   the cmd topic (sub_nav on_command_rx), so the car never drives off
	   on its own after a boot or flash. */
	tm_printf((UB*)"start: IDLE, waiting for START (G) over MQTT\n");
#else
	/* No MQTT: nothing can send START, so start the run now. START goes
	   through the event bus like a dashboard START, so sub_nav (and the
	   sub_terrain re-level it does) runs on the FAST lane that owns it.
	   Not after a watchdog reset (rc_wdog_boot set WDOG_RESET): the car
	   restarted because something hung, possibly in the middle of a
	   manoeuvre and off the line, so it stays in IDLE until a person
	   resets it. */
	if ((rc_fault_get() & RC_FAULT_WDOG_RESET) != 0U) {
		tm_printf((UB*)"start: watchdog reset, staying in IDLE\n");
	} else if (sub_nav_inject_command(RC_CMD_START, 0) != RC_OK) {
		tm_printf((UB*)"ERROR START\n");
	}
#endif
#endif

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	rc_fault_test();		/* fault bitmask, Sense timing */

#if RC_RISK1_SOAK_SECS > 0
	/* Risk 1 soak: run the control path under worst-case load and print
	   the event-bus report every 2 s.  CAR ON A STAND - the wheels move.
	   Apply load by hand: spin both wheels at full speed, swipe a barcode
	   under the sensor every few seconds, and (NET=lwip MQTT=1 build) flood
	   the cmd topic.  Set the duration in rc_config.h RC_RISK1_SOAK_SECS. */
	{
		INT s;
		tm_printf((UB*)"[RISK1] soak start (%d s); car MUST be on a stand\n",
			  (int)RC_RISK1_SOAK_SECS);
		rc_event_probe_reset();
		rc_gpioirq_probe_reset();
		if (sub_nav_inject_command(RC_CMD_START, 0) != RC_OK) {
			tm_printf((UB*)"[RISK1] WARN: START failed\n");
		}
		for (s = 0; s < (INT)RC_RISK1_SOAK_SECS; s++) {
			(void)tk_dly_tsk(1000);
			if ((s & 1) != 0) {
				rc_event_report();
				tm_printf((UB*)"[RISK1] gpio_irq max=%u us over %u irqs\n",
					  (unsigned)rc_gpioirq_max_us(),
					  (unsigned)rc_gpioirq_irq_count());
			}
		}
		tm_printf((UB*)"[RISK1] soak end\n");
	}
#endif

	/* Final event-bus / IRQ snapshot from the test run (and soak, if any). */
	rc_event_report();
	tm_printf((UB*)"[RISK1] gpio_irq max=%u us over %u irqs\n",
		  (unsigned)rc_gpioirq_max_us(),
		  (unsigned)rc_gpioirq_irq_count());

	/* NFR5 split by source, and the task run times for the response-time
	 * table (rc_config.h task priorities). */
	{
		uint32_t line_us = 0U;
		uint32_t imu_us = 0U;

		rc_sense_max_us(&line_us, &imu_us);
		tm_printf((UB*)"[RISK1] isr cb max: enc L %u, enc R %u, echo %u us\n",
			  (unsigned)rc_gpioirq_pin_max_us(RC_PIN_ENC_L_A),
			  (unsigned)rc_gpioirq_pin_max_us(RC_PIN_ENC_R_A),
			  (unsigned)rc_gpioirq_pin_max_us(RC_PIN_ULTRA_ECHO));
		tm_printf((UB*)"[RISK1] task max: sense line %u + imu %u + bar %u us,"
			  " motion %u us\n",
			  (unsigned)line_us, (unsigned)imu_us,
			  (unsigned)rc_sense_bar_max_us(),
			  (unsigned)sub_motion_wcet_us());
		tm_printf((UB*)"[RISK1] missed periods: sense %u, motion %u\n",
			  (unsigned)rc_sense_overruns(),
			  (unsigned)sub_motion_overruns());
	}

	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
