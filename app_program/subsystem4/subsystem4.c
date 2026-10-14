/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU & Terrain Classification (Buddy 4)
 *
 *	subsystem4_init():  initialize terrain state machine and event subscription.
 *	subsystem4_start(): initialize IMU hardware over I2C.
 */

#include "../rc_prelude.h"
#include "subsystem4.h"
#include "drv_imu.h"
#include "sub_terrain.h"

EXPORT ER subsystem4_init(void)
{
	if (sub_terrain_init() != RC_OK) {
		return E_SYS;
	}
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* Non-fatal if sensor board is not wired up during bench testing */
	(void)drv_imu_init();
	return E_OK;
}