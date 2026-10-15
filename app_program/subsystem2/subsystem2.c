/*
 *  subsystem2.c
 *  Subsystem 2 -- motion control (Buddy 2): motors, encoders, sub_motion.
 *
 *  subsystem2_init():  motor PWM (stopped), encoder interrupts and their
 *                      bottom half, then sub_motion: PID gains, mutex,
 *                      MOTION_DONE subscriber and the motion task
 *                      (RC_PRI_MOTION, every RC_PERIOD_MOTION_MS).
 *  subsystem2_start(): sets the cruise speed (report §6.5 IDLE, 250 mm/s).
 *                      The motion task is already running; it idles until
 *                      someone asks for a move.
 *
 *  Needs the shared layer (rc_defer, rc_event, rc_gpioirq) first;
 *  app_main.c starts it in Phase 0.  Nothing here makes the car move:
 *  sub_nav (FOLLOWING) and sub_line decide that.
 */

#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "sub_motion.h"

/* Report §6.5 IDLE: base speed for distance moves and turns, mm/s. */
#define S2_CRUISE_MM_S  (250U)

/* Print the first failing step, return true if r is RC_OK. */
static bool s2_ok(rc_result_t r, const char *what)
{
  if (r != RC_OK) {
    tm_printf((UB*)"[S2] %s failed (%d)\n", what, (INT)r);
    return false;
  }
  return true;
}

static bool s2_init_ok;

EXPORT ER subsystem2_init(void)
{
  /* Order matters: sub_motion drives the motors and reads the
   * encoders, so both drivers come first (sub_motion.h). */
  s2_init_ok = s2_ok(drv_motor_init(),   "drv_motor_init()")
      && s2_ok(drv_encoder_init(), "drv_encoder_init()")
      && s2_ok(sub_motion_init(),  "sub_motion_init()");

  return s2_init_ok ? E_OK : E_SYS;
}

EXPORT ER subsystem2_start(void)
{
  if (!s2_init_ok) {
    return E_OBJ;
  }
  return s2_ok(sub_motion_set_speed(S2_CRUISE_MM_S),
         "sub_motion_set_speed()") ? E_OK : E_SYS;
}