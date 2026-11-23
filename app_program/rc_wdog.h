/*
 *  rc_wdog.h  -  hardware watchdog with a liveness check
 *
 *  The RP2040 watchdog reboots the chip unless it is reloaded in time
 *  (RC_WDOG_MS). It is reloaded only while the control path is alive:
 *  the Sense task, the Motion task and the FAST event dispatcher each
 *  check in (rc_wdog_checkin) every time round their loop, and a cyclic
 *  handler reloads the watchdog every RC_WDOG_CHECK_MS only if all three
 *  have checked in since its last visit. A task that stops (stuck in a
 *  driver, a deadlock, a crashed core) therefore reboots the car.
 *
 *  After a watchdog reboot rc_wdog_boot() reports it on the console and
 *  sets RC_FAULT_WDOG_RESET (the dashboard shows it until START), naming
 *  the tasks that had stopped checking in. The MQTT build then waits in
 *  IDLE for START, so a reboot never sets the car moving by itself.
 *
 *  The watchdog block's tick generator (WATCHDOG_TICK) also drives the
 *  1 us TIMER (rc_time.c); nothing here changes it.
 */
#ifndef RC_WDOG_H
#define RC_WDOG_H

#include "rc_types.h"

typedef enum {
    RC_WDOG_SENSE = 0,
    RC_WDOG_MOTION,
    RC_WDOG_FAST,
    RC_WDOG_N
} rc_wdog_who_t;

/* Read why the chip last reset and report a watchdog reboot (console,
 * RC_FAULT_WDOG_RESET). Call once at boot, after rc_fault_init(). */
void rc_wdog_boot(void);

/* Start the watchdog and its supervisor. Call once, after every task that
 * checks in is running (end of start-up, before the run begins). */
rc_result_t rc_wdog_start(void);

/* The calling task is alive. Cheap (one byte store); any context. */
void rc_wdog_checkin(rc_wdog_who_t who);

/* RC_WDOG_TRACE: the FAST dispatcher is about to call `cb` for event
 * `evt_id` / has returned from it. Two register writes each. */
void rc_wdog_trace_enter(uint32_t evt_id, const void *cb);
void rc_wdog_trace_leave(void);

#endif /* RC_WDOG_H */
