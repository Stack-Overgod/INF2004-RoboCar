/*
 *  drv_ultrasonic.h  -  Buddy 5 hardware layer
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
 *    ECHO fall ISR computes the width, publishes the result, returns
 *    alarm 2 ISR   publishes an invalid result if no echo came back
 *
 *  Total CPU time per measurement is a few microseconds spread over five
 *  interrupts. The 30 ms of flight time costs nothing.
 *
 *  The RP2040 TIMER alarms are unused by this mtk3 port (the kernel tick
 *  is SysTick and the physical timer is built on PWM), so alarms 1 and 2
 *  are ours. Alarms 0 and 3 are left free.
 */
/* Include guard — see sub_telemetry.h for the full explanation of why
 * this #ifndef/#define/#endif wrapper exists (prevents "redefined"
 * compiler errors if this header is #included more than once). */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "rc_types.h"

/*
 *  This is a *function pointer type* (see sub_telemetry.h for the fuller
 *  explanation of the syntax): any variable of type drv_ultra_cb_t can be
 *  pointed at a function that takes (a distance in mm, whether the
 *  reading is trustworthy, a free-form context pointer) and returns
 *  nothing. It's how the driver "calls you back" once a measurement
 *  finishes, instead of you having to sit and wait for it.
 *
 *  Result callback. Runs in INTERRUPT context (an ISR — a tiny function
 *  the chip jumps to automatically the instant a hardware event fires,
 *  pausing whatever else was running), so keep it to a store and a
 *  publish. If you want task context instead (i.e. ordinary code, safe
 *  to do more work in), subscribe to RC_EVT_ULTRA_RESULT and let the
 *  dispatcher hand it to you there instead.
 */
typedef void (*drv_ultra_cb_t)(uint16_t range_mm, bool valid, void *ctx);

/* Sets up TRIG as an output pin, registers (but leaves masked/disabled)
 * the ECHO pin interrupt, and claims two RP2040 hardware TIMER alarms
 * (see the big comment above for what those are used for). Call once at
 * boot. */
rc_result_t drv_ultrasonic_init(void);

/* Optional direct hook, in addition to the event published on every
 * completed measurement. `cb` is a pointer to your callback function (see
 * drv_ultra_cb_t above); `ctx` is any pointer you want handed back to you
 * unchanged when `cb` is called — a way to pass "your own data" through
 * without global variables. Pass NULL for `cb` to clear it. */
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
