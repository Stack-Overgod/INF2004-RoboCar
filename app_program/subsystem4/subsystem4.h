/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU & Terrain Classification (Buddy 4)
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "drv_imu.h"
#include "sub_terrain.h"

#define S4_PRI_MAIN	18		/* priority band 18-19 (TEAM.md) */

IMPORT ER subsystem4_init(void);
IMPORT ER subsystem4_start(void);

#endif /* SUBSYSTEM4_H */