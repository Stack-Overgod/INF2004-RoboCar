/*
 *  drv_ultrasonic.c
 *
 *  HC-SR04 ultrasonic sensor: send a short pulse, time how long the echo
 *  takes to come back, and convert that time to a distance. See the
 *  comment in drv_ultrasonic.h and docs/buddy5-scan-avoidance/ for the
 *  5-step interrupt relay this file implements instead of a busy-wait.
 */
#include "../rc_prelude.h"
#include "drv_ultrasonic.h"
#include "../rc_config.h"
#include "../rc_gpioirq.h"
#include "../rc_event.h"
#include "../rc_fault.h"
#include "../rc_defer.h"

/* RP2040 TIMER alarm interrupts are IRQ 0..3. A TIMER alarm raises its
 * interrupt when the free-running microsecond counter reaches a target,
 * so no code waits in a loop. Alarms 1 and 2 are used here. */
#define INTNO_TIMER_1       (1U)
#define INTNO_TIMER_2       (2U)
/* interrupt priority level given to both timer alarms: the same level as
 * PendSV, because the handlers call kernel services and must not preempt a
 * task switch */
#define INTPRI_TIMER        (3)

/* Alarm slot (0-3) for each use: alarm 1 times the TRIG pulse; alarm 2 is
 * the timeout in case no echo arrives. */
#define ALARM_TRIG          (1U)    /* fires when the trigger pulse ends */
#define ALARM_TMO           (2U)    /* fires if no echo comes back       */

/* ALARMn register address: offset 0x10 plus 4 bytes per slot. */
#define TIMER_ALARM_REG(n)  (TIMER_BASE + 0x10U + ((n) * 4U))

/*
 *  Speed of sound 343 m/s at 20 C, out and back, so
 *      mm = us * 343 / 2000
 *  `us` is the ECHO pulse width in microseconds. The sound travels to the
 *  obstacle and back, hence /2000 rather than /1000. Integer only; the
 *  truncation error is well under a millimetre, far below the module's
 *  own 3 mm spec.
 */
#define US_TO_MM(us)        (((us) * 343UL) / 2000UL)

/* The ping/echo state machine's states. */
typedef enum {
    ST_IDLE = 0,      /* no measurement running; ready for a new ping() */
    ST_TRIG,          /* TRIG pin is held high (the trigger pulse) */
    ST_WAIT_RISE,     /* TRIG dropped; waiting for ECHO to go high */
    ST_WAIT_FALL      /* ECHO is high; waiting for it to drop */
} ultra_state_t;

/* Every variable shared between an ISR and task code is `volatile`, so
 * the compiler re-reads it each time instead of caching it. */
static volatile ultra_state_t state = ST_IDLE;
/* microsecond timestamp when ECHO last went high, set in echo_isr */
static volatile uint32_t      t_rise;
/* servo angle this ping was taken at, so the result can be matched back to a
 * bearing */
static volatile int16_t       tag_angle;
/* RC_FAULT_NO_ECHO: set after ULTRA_MISS_N pings in a row that the sensor
 * never answered (ECHO never rose), cleared by the next answer. A ping it
 * answered with nothing in range (ECHO rose but stayed high past the
 * timeout) is a good reading of "clear ahead", not a fault. Three misses
 * (~200 ms of watch pings) ride out a single lost ping. Defer task only. */
#define ULTRA_MISS_N        (3U)
static bool                   echo_ok = true;
static uint32_t               miss_n;
/* optional callback registered via drv_ultrasonic_on_result() */
static drv_ultra_cb_t         user_cb;
/* the free-form context pointer handed back unchanged to user_cb */
static void                  *user_ctx;
/* rc_defer handle for ultra_drain (the bottom half) */
static int32_t                defer_h = -1;

/* Set by an ISR, consumed by the bottom half (ultra_drain), which does
 * the conversion and publishing in task context so the ISR stays short. */
/* how long ECHO stayed high, in microseconds — the raw measurement */
static volatile uint32_t      result_width_us;
/* true if a real echo was measured; false if we timed out instead */
static volatile bool          result_valid;
/* true if ECHO rose at all (the sensor answered), even if it then timed
 * out; false if it never rose */
static volatile bool          result_answered;
/* true once finish_i() has something for ultra_drain() to process */
static volatile bool          result_ready;

/* ------------------------------------------------------------------ *
 *  TIMER alarm helpers
 * ------------------------------------------------------------------ */

/* Arms hardware alarm slot `n` to fire `delay_us` microseconds from now.
 * The TIMER interrupt registers hold one bit per alarm (1U << n). */
static void alarm_arm(uint32_t n, uint32_t delay_us)
{
    UINT     sts;
    uint32_t target;

    out_w(TIMER_INTR, (1U << n));           /* clear any stale flag */
    clr_w(TIMER_INTF, (1U << n));           /* and any forced one */
    /* enable (unmask) this alarm's interrupt */
    set_w(TIMER_INTE, (1U << n));
    /* Writing the alarm register arms it: the timer counts microseconds
     * in TIMER_TIMERAWL, so "current time + delay" is the target.
     *
     * The alarm fires only when the counter EQUALS the target; a target
     * already passed would wait for the 32-bit wrap (71.6 min). So the
     * read and the write happen with interrupts off (nothing can delay
     * this core between them), and if the target has passed anyway the
     * alarm is disarmed and its interrupt forced (TIMER_INTF) instead.
     * The signed difference keeps that test right across the wrap. */
    DI(sts);
    target = in_w(TIMER_TIMERAWL) + delay_us;
    out_w(TIMER_ALARM_REG(n), target);
    if (((int32_t)(in_w(TIMER_TIMERAWL) - target) >= 0)
        && ((in_w(TIMER_INTR) & (1U << n)) == 0U)) {
        out_w(TIMER_ARMED, (1U << n));      /* write 1 to disarm */
        set_w(TIMER_INTF, (1U << n));       /* fire it now */
    }
    EI(sts);
}

/* Disarms hardware alarm slot `n` before it fires (e.g. the timeout once
 * the echo has been measured). */
static void alarm_cancel(uint32_t n)
{
    /* disable (mask) this alarm's interrupt */
    clr_w(TIMER_INTE, (1U << n));
    out_w(TIMER_ARMED, (1U << n));          /* write 1 to disarm */
    /* clear any pending flag so it can't fire late */
    out_w(TIMER_INTR, (1U << n));
    clr_w(TIMER_INTF, (1U << n));
}

/* ------------------------------------------------------------------ *
 *  Completion, shared by the success and timeout paths
 * ------------------------------------------------------------------ */

/*
 *  ISR side. Records the outcome and asks for a bottom half. The
 *  division that turns microseconds into millimetres, the event publish
 *  and the user callback all happen in ultra_drain, not here.
 *
 *  The two register operations that CANNOT be deferred stay: masking the
 *  echo pin and disarming the timeout. Leaving the echo unmasked for the
 *  length of a task hop would let a ringing 5 V divider generate spurious
 *  edges that the state machine would then have to filter.
 */
static void finish_i(uint32_t width_us, bool valid, bool answered)
{
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, false);
    alarm_cancel(ALARM_TMO);

    result_width_us = width_us;
    result_valid    = valid;
    result_answered = answered;
    result_ready    = true;
    state           = ST_IDLE;

    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Task context.
 * ------------------------------------------------------------------ */

/* The bottom half, run by the defer task after finish_i() signalled it:
 * converts the width to millimetres, updates the NO_ECHO fault, publishes
 * RC_EVT_ULTRA_RESULT and calls the optional callback. */
static void ultra_drain(void *ctx)
{
    rc_event_t evt;
    uint32_t   mm = 0U;
    uint32_t   width;
    bool       valid;
    bool       answered;

    (void)ctx;

    if (!result_ready) {
        return;
    }
    result_ready = false;

    width    = result_width_us;
    valid    = result_valid;
    answered = result_answered;

    if (valid) {
        mm = US_TO_MM(width);
        if ((mm > RC_ULTRA_MAX_MM) || (mm < RC_ULTRA_MIN_MM)) {
            valid = false;              /* out of the module's range */
        }
    }

    /* 12-bit range, 0 = no echo (no separate valid flag); 8-bit servo
     * angle the ping was tagged with. The NO_ECHO fault follows the
     * latest reading. */
    evt.id = RC_EVT_ULTRA_RESULT;
    evt.u.ultra.angle_deg = RC_SAT_U((tag_angle < 0) ? 0 : tag_angle,
                                     RC_BITS_ANGLE);
    evt.u.ultra.range_mm  = valid ? RC_SAT_U(mm, RC_BITS_MM) : 0U;
    if (valid && (evt.u.ultra.range_mm == 0U)) {
        evt.u.ultra.range_mm = 1U;      /* keep 0 meaning "no echo" */
    }
    /* before the publish: sub_nav reads the fault when the event lands */
    if (answered) {
        miss_n = 0U;
        if (!echo_ok) {
            rc_fault_clear(RC_FAULT_NO_ECHO);
            echo_ok = true;
        }
    } else if (miss_n < ULTRA_MISS_N) {
        miss_n++;
        if ((miss_n >= ULTRA_MISS_N) && echo_ok) {
            rc_fault_set(RC_FAULT_NO_ECHO);
            echo_ok = false;
        }
    } else {
        /* still not answering; the fault is already up */
    }
    (void)rc_event_publish(&evt);

    /* optional direct callback, in addition to the event */
    if (user_cb != NULL) {
        user_cb((uint16_t)mm, valid, user_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Interrupt handlers
 * ------------------------------------------------------------------ */

/*
 *  ===== STEP 2 of the 5-step ping/echo relay =====
 *  TIMER alarm 1 (ALARM_TRIG) fires RC_ULTRA_TRIG_US after ping() armed
 *  it in step 1. Drops TRIG (the HC-SR04 then emits its 40 kHz burst),
 *  unmasks the ECHO edge interrupt (steps 3/4) and arms the timeout
 *  alarm (step 5) in case no echo comes back.
 */
static void trig_done_handler(UINT intno)
{
    (void)intno;
    /* acknowledge this alarm's interrupt flag (hardware or forced) */
    out_w(TIMER_INTR, (1U << ALARM_TRIG));
    clr_w(TIMER_INTF, (1U << ALARM_TRIG));
    /* mask it again — one-shot, not recurring */
    clr_w(TIMER_INTE, (1U << ALARM_TRIG));
    if (state != ST_TRIG) {
        EndOfInt(INTNO_TIMER_1);            /* a late duplicate: no ping */
        return;
    }

    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 0U);   /* end the trigger pulse */

    state = ST_WAIT_RISE;
    /* start listening for the echo */
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, true);
    /* step 5's safety-net timer */
    alarm_arm(ALARM_TMO, RC_ULTRA_TIMEOUT_US);

    /* tell the interrupt controller this ISR is done */
    EndOfInt(INTNO_TIMER_1);
}

/*
 *  ===== STEP 5 of the 5-step ping/echo relay (only if the echo is lost) =====
 *  TIMER alarm 2 (ALARM_TMO) fires RC_ULTRA_TIMEOUT_US after step 2 armed
 *  it: ECHO never rose, or never fell, in time (obstacle too far, too
 *  soft or angled away). Reports an invalid reading so the scan does not
 *  wait forever. The `state != ST_IDLE` check covers the race where the
 *  echo completed (step 4) just before the timeout fired.
 */
static void timeout_handler(UINT intno)
{
    (void)intno;
    out_w(TIMER_INTR, (1U << ALARM_TMO));
    clr_w(TIMER_INTF, (1U << ALARM_TMO));

    if (state != ST_IDLE) {
        /* 0 width, not valid — see finish_i/ultra_drain. Still waiting
         * for the rise = the sensor never answered; waiting for the fall
         * = it answered but nothing reflected within range. */
        finish_i(0U, false, (state == ST_WAIT_FALL));
    }
    EndOfInt(INTNO_TIMER_2);
}

/*
 *  ===== STEPS 3 and 4 of the 5-step ping/echo relay =====
 *  ECHO edge callback (RC_EDGE_BOTH). The HC-SR04 holds ECHO high from
 *  the burst until the echo returns, so the pulse width is the flight
 *  time.
 *
 *  Step 3 (rising edge in ST_WAIT_RISE): stamp t_rise, go to ST_WAIT_FALL.
 *  Step 4 (falling edge in ST_WAIT_FALL): `t_us - t_rise` is the width in
 *  microseconds; finish_i() hands it to the bottom half for conversion.
 */
static void echo_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    (void)pin;
    (void)ctx;

    if ((state == ST_WAIT_RISE) && level) {
        t_rise = t_us;
        state  = ST_WAIT_FALL;
    } else if ((state == ST_WAIT_FALL) && !level) {
        finish_i(t_us - t_rise, true, true);
    } else {
        /* Spurious edge (ringing on the 5 V divider), ignore. */
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

/* Sets up the 5-step relay: TRIG as an output, the ECHO edge interrupt
 * (registered but masked), the two TIMER alarm handlers and the bottom
 * half (ultra_drain). Call once at boot. */
rc_result_t drv_ultrasonic_init(void)
{
    T_DINT      dint;
    rc_result_t res;

    defer_h = rc_defer_register(ultra_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    (void)gpio_set_pin(RC_PIN_ULTRA_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 0U);

    /* Echo is registered now but left masked. It is only unmasked for
     * the window between the trigger ending and the result arriving, so
     * a floating or noisy echo line cannot generate interrupts while the
     * car is doing something else. */
    res = rc_gpioirq_attach(RC_PIN_ULTRA_ECHO, RC_EDGE_BOTH, -1,
                            echo_isr, NULL);
    if (res != RC_OK) {
        return res;
    }
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, false);

    dint.intatr = TA_HLNG;
    dint.inthdr = trig_done_handler;
    if (tk_def_int(INTNO_TIMER_1, &dint) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    ClearInt(INTNO_TIMER_1);
    EnableInt(INTNO_TIMER_1, INTPRI_TIMER);

    dint.inthdr = timeout_handler;
    if (tk_def_int(INTNO_TIMER_2, &dint) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    ClearInt(INTNO_TIMER_2);
    EnableInt(INTNO_TIMER_2, INTPRI_TIMER);

    state = ST_IDLE;
    return RC_OK;
}

/* Stores the callback and its context for ultra_drain() to call. */
rc_result_t drv_ultrasonic_on_result(drv_ultra_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/*
 *  ===== STEP 1 of the 5-step ping/echo relay =====
 *  Task context (e.g. sub_scan.c's step_to()). Refuses with RC_ERR_BUSY
 *  if a measurement is running; otherwise records the tag angle, raises
 *  TRIG and arms alarm 1 for RC_ULTRA_TRIG_US (step 2). Never blocks.
 */
rc_result_t drv_ultrasonic_ping(int16_t tag_angle_deg)
{
    uint32_t sts;

    /* Check-then-set of `state` with interrupts off so an ISR cannot
     * interleave; EI(sts) restores the previous interrupt state. */
    DI(sts);
    if (state != ST_IDLE) {
        EI(sts);
        return RC_ERR_BUSY;
    }
    state     = ST_TRIG;
    tag_angle = tag_angle_deg;
    EI(sts);

    /* start the trigger pulse */
    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 1U);
    alarm_arm(ALARM_TRIG, RC_ULTRA_TRIG_US);     /* schedule step 2 to end it */

    return RC_OK;
}

/* True while any step of the 5-step relay is in progress. */
bool drv_ultrasonic_busy(void)
{
    return (state != ST_IDLE);
}
