/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU & Terrain Classification (Buddy 4)
 *
 *	subsystem4_init():  terrain state machine and its IMU_SAMPLE
 *	                    subscription (FAST lane).
 *	subsystem4_start(): IMU over I2C, then the boot calibration (report
 *	                    §6.5 IDLE: the car must sit level and still).
 *	                    Sampling itself is done by the shared Sense thread
 *	                    (subsystem3.c), which calls drv_imu_sample() every
 *	                    RC_PERIOD_IMU_MS once drv_imu_available() is true.
 *
 *	An IMU failure is NOT fatal (FR15): the car runs without hump and
 *	impact detection, and drv_imu_available() reports IMU_UNAVAILABLE.
 */

#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "../rc_config.h"
#include "subsystem4.h"
#include "drv_imu.h"
#include "sub_terrain.h"

EXPORT ER subsystem4_init(void)
{
	if (sub_terrain_init() != RC_OK) {
		tm_printf((UB*)"[S4] sub_terrain_init() failed\n");
		return E_SYS;
	}
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	if (drv_imu_init() != RC_OK) {
		tm_printf((UB*)"[S4] IMU not found: IMU_UNAVAILABLE (FR15)\n");
		return E_OK;
	}
	if (drv_imu_calibrate(RC_IMU_CAL_SAMPLES) != RC_OK) {
		tm_printf((UB*)"[S4] IMU calibration failed: IMU_UNAVAILABLE (FR15)\n");
	}
	return E_OK;
}
