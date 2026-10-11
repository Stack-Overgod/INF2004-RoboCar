/*
 *	subsystem5.h
 *	Subsystem 5 -- Ultrasonic scanning and obstacle avoidance.
 *	Student ID: 2500347.
 *
 *	Public interface of subsystem 5.  Other subsystems include it as
 *	"../subsystem5/subsystem5.h".  Put here ONLY what other subsystems
 *	may use; everything else stays local in the .c files.
 *
 *	Today nothing crosses the boundary through this header: S5 publishes
 *	its results as events (RC_EVT_ULTRA_RESULT, RC_EVT_OBSTACLE_PROFILE,
 *	RC_EVT_AVOIDANCE_PLAN) and exposes its API through sub_scan.h, which
 *	sub_nav includes directly.  So this header only pulls in app.h and
 *	defines the background priority used by the TEST=1 live-bench task.
 *	The real Scan task runs at RC_PRI_SENSE (7); see rc_config.h, the
 *	single source of truth for every priority (report §3, TABLE 8/9).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../rc_config.h"

/* Low background priority for the optional TEST=1 live-bench sweep only.
 * The production Scan task does NOT use this -- it is created inside
 * sub_scan_init() at RC_PRI_SENSE. */
#define S5_PRI_MAIN	RC_PRI_TELEMETRY	/* 10 = lowest app priority */

#endif /* SUBSYSTEM5_H */
