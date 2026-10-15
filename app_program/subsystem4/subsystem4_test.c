/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01 ...
 */

#include "../rc_prelude.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_time.h"
#include "subsystem4.h"
#include "drv_imu.h"
#include "sub_terrain.h"
#include "../test_report.h"

#if APP_TEST

/* How long to let the FAST dispatcher handle a fake sample. The tests run
 * in usermain() at priority 1; the dispatcher (priority 5) only gets the
 * CPU while this task sleeps, so every check must come after a delay. */
#define S4_SETTLE_MS	(20)

static volatile uint16_t cb_last_peak_mm = 0U;
static volatile uint32_t cb_call_count   = 0U;

static void test_hump_cb(uint16_t peak_mm, uint32_t duration_ms, void *ctx)
{
	(void)duration_ms;
	(void)ctx;
	cb_last_peak_mm = peak_mm;
	cb_call_count++;
}

/* Publish what drv_imu_sample() would, then wait until it is handled. */
static void publish_imu(int16_t ax, int16_t ay, int16_t az)
{
	rc_event_t evt = {0};

	evt.id = RC_EVT_IMU_SAMPLE;
	evt.u.imu.acc_x = ax;
	evt.u.imu.acc_y = ay;
	evt.u.imu.acc_z = az;
	(void)rc_event_publish(&evt);
	(void)tk_dly_tsk(S4_SETTLE_MS);
}

EXPORT void subsystem4_test(void)
{
	int16_t dummy_x = 0, dummy_y = 0, dummy_z = 0;

	TEST_INIT();

	tm_printf((UB*)"[INFO] S4 IMU %s\n",
		  drv_imu_available() ? "available" : "UNAVAILABLE (FR15)");

	/* S4-U01: Null pointer guard checks on IMU reads */
	TEST_CHECK("S4-U01",
		drv_imu_read_accel(NULL, &dummy_y, &dummy_z) == RC_ERR_PARAM &&
		drv_imu_read_accel(&dummy_x, NULL, &dummy_z) == RC_ERR_PARAM &&
		drv_imu_read_mag(&dummy_x, &dummy_y, NULL) == RC_ERR_PARAM,
		"drv_imu null pointer parameter rejection");

	/* S4-U02: Zero sample calibration guard */
	TEST_CHECK("S4-U02",
		drv_imu_calibrate(0U) == RC_ERR_PARAM,
		"drv_imu_calibrate rejects 0 sample count");

	/* S4-U03: Terrain subsystem initialization and reset state.  A second
	 * sub_terrain_init() (usermain already ran it) must succeed without
	 * subscribing again. */
	TEST_CHECK("S4-U03",
		sub_terrain_init() == RC_OK &&
		sub_terrain_motion_class() == RC_MOTION_STATIONARY &&
		sub_terrain_max_peak_mm() == 0U,
		"sub_terrain_init resets state to stationary and 0mm peak");

	/* S4-U04: Impact detection on high-G spike (>= RC_IMPACT_MG).
	 * From here to U07 only fake samples reach sub_terrain. */
	drv_imu_test_hold(true);
	(void)tk_dly_tsk(S4_SETTLE_MS);	/* let a real sample in flight drain */
	sub_terrain_reset();
	publish_imu(1900, 0, 1000);
	TEST_CHECK("S4-U04",
		sub_terrain_motion_class() == RC_MOTION_IMPACT,
		"high acc_x spike triggers RC_MOTION_IMPACT");

	/* S4-U05: Hump climbing state detection (pitch >= 4.0 deg -> ax >= 70 mg) */
	publish_imu(0, 0, 1000);	/* back below the impact threshold */
	sub_terrain_reset();
	(void)sub_terrain_on_hump(test_hump_cb, NULL);
	cb_last_peak_mm = 0U;
	cb_call_count   = 0U;

	publish_imu(150, 0, 1000); /* ~8.5 deg nose-up */
	TEST_CHECK("S4-U05",
		sub_terrain_motion_class() == RC_MOTION_CLIMBING,
		"nose-up pitch enters H_CLIMBING state");

	/* S4-U06: Hump descending state transition */
	(void)tk_dly_tsk(160);      /* exceed HUMP_MIN_MS (150ms) */
	publish_imu(-80, 0, 1000);  /* dip below -2.0 deg to enter H_DESCENDING */
	TEST_CHECK("S4-U06",
		sub_terrain_motion_class() == RC_MOTION_DESCENDING,
		"nose-down pitch transitions to H_DESCENDING");

	/* S4-U07: Hump completion, peak height calculation, and callback */
	publish_imu(0, 0, 1000);    /* level out within [-2.0, +2.0] deg */
	TEST_CHECK("S4-U07",
		sub_terrain_max_peak_mm() > 0U &&
		cb_call_count == 1U &&
		cb_last_peak_mm == sub_terrain_max_peak_mm() &&
		sub_terrain_motion_class() == RC_MOTION_STATIONARY,
		"leveling out records non-zero hump peak_mm and fires callback");

	(void)sub_terrain_on_hump(NULL, NULL);
	sub_terrain_reset();
	drv_imu_test_hold(false);	/* real samples flow again */

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
