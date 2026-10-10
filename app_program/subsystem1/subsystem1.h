/*
 *	subsystem1.h
 *	Subsystem 1 -- LED heartbeat.  Student ID: <student ID>.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_LED	12		/* priority band 12-13 (TEAM.md) */
#define S1_LED_PIN	19		/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250		/* LED toggles every tick */

/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID	tid_led;

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
