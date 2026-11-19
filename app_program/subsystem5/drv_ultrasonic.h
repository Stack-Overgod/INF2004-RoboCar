/*
 *  drv_ultrasonic.h  -  HC-SR04 ultrasonic driver
 *
 *  HC-SR04, fully interrupt driven. No part of a measurement spins.
 *
 *  The classic Arduino driver does this:
 *
 *      digitalWrite(TRIG, HIGH); delayMicroseconds(10);
 *      digitalWrite(TRIG, LOW);  pulseIn(ECHO, HIGH);
 *
 *  Both of those are busy-waits, and pulseIn can hold the CPU for 30 ms
 *  when nothing echoes back. On a car that is running a PID loop at 50 Hz
 *  that is a missed control update and a swerve. Here:
 *
 *    ping()        raises TRIG, arms TIMER alarm 1 for +12 us, returns
 *    alarm 1 ISR   drops TRIG, unmasks the ECHO edge, arms alarm 2
 *                  as the timeout, returns
 *    ECHO rise ISR stamps the start, returns
 *    ECHO fall ISR records the width, signals the defer task, returns
 *    alarm 2 ISR   records an invalid result if no echo came back
 *
 *  The defer task (rc_defer) then converts the width to millimetres and
 *  publishes RC_EVT_ULTRA_RESULT. CPU time per measurement is a few
 *  microseconds spread over the interrupts; the flight time costs nothing.
 *
 *  The RP2040 TIMER alarms are unused by this mtk3 port (the kernel tick
 *  is SysTick and the physical timer is built on PWM), so alarms 1 and 2
 *  are ours. Alarms 0 and 3 are left free.
 */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "../rc_types.h"

/*
 *  Result callback type: (distance in mm, reading valid, context pointer).
 *  Called from the defer task (task context, RC_PRI_DEFER) once per
 *  completed measurement, right after RC_EVT_ULTRA_RESULT is published.
 *  Keep it short: it runs ahead of most other tasks.
 */
typedef void (*drv_ultra_cb_t)(uint16_t range_mm, bool valid, void *ctx);

/* Sets up TRIG as an output pin, registers (but leaves masked) the ECHO
 * pin interrupt, and claims TIMER alarms 1 and 2. Call once at boot. */
rc_result_t drv_ultrasonic_init(void);

/* Optional direct hook, in addition to the event published on every
 * completed measurement. `ctx` is handed back unchanged to `cb`. Pass
 * NULL for `cb` to clear it. */
rc_result_t drv_ultrasonic_on_result(drv_ultra_cb_t cb, void *ctx);

/*
 *  Start one measurement. Returns RC_ERR_BUSY if one is already in
 *  flight. Never blocks. The result arrives via the callback and via
 *  RC_EVT_ULTRA_RESULT, tagged with the servo angle at the time of the
 *  ping so a scan can pair range to bearing without extra bookkeeping.
 */
rc_result_t drv_ultrasonic_ping(int16_t tag_angle_deg);

/* True while a measurement (trigger pulse sent, or waiting on the echo)
 * is currently in progress. */
bool drv_ultrasonic_busy(void);

#endif /* DRV_ULTRASONIC_H */
