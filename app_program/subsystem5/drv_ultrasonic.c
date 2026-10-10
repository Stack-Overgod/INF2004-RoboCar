/*
 *  drv_ultrasonic.c
 *
 *  The "shout and listen" driver, HC-SR04 ultrasonic sensor. Like a bat's
 *  echolocation or a submarine's sonar: send out a short pulse, measure
 *  how long the echo takes to bounce back, and that time tells you the
 *  distance (sound travels at a known, roughly constant speed through
 *  air). See the big comment in drv_ultrasonic.h and the Buddy 5 guide
 *  (docs/buddy5-scan-avoidance/) for the full 5-step interrupt relay this file
 *  implements instead of a simple busy-wait.
 */
#include "rc_prelude.h"
#include "drv_ultrasonic.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_defer.h"
#include "rc_time.h"

/* RP2040 TIMER alarm interrupts are IRQ 0..3.
 * A "TIMER alarm" is a piece of hardware built into the chip: you tell it
 * "interrupt me in X microseconds" and then forget about it — no code has
 * to sit in a loop counting down. When the time is up, the chip
 * automatically jumps to the ISR (interrupt handler) registered for that
 * alarm number. We use two of these alarms (1 and 2) instead of software
 * delay loops, which is what keeps this driver from ever blocking the
 * rest of the car's code. */
#define INTNO_TIMER_1       (1U)
#define INTNO_TIMER_2       (2U)
/* interrupt priority level given to both timer alarms */
#define INTPRI_TIMER        (2)

/* Which of the RP2040's hardware alarm "slots" (0-3) each timer use is
 * assigned to. Alarm 1 times the 12us TRIG pulse; alarm 2 is the
 * "give up and report failure" timeout in case no echo ever arrives. */
#define ALARM_TRIG          (1U)    /* fires when the trigger pulse ends */
#define ALARM_TMO           (2U)    /* fires if no echo comes back       */

/* Computes the hardware register address for a given alarm slot number,
 * by adding a fixed offset (0x10) plus 4 bytes per slot to the timer
 * peripheral's base address. This is how low-level register-based
 * hardware access works: every controllable thing on the chip is just a
 * numbered memory address you read/write. */
#define TIMER_ALARM_REG(n)  (TIMER_BASE + 0x10U + ((n) * 4U))

/*
 *  Speed of sound 343 m/s at 20 C, out and back, so
 *      mm = us * 343 / 2000
 *  Integer only. The error from truncation is well under a millimetre,
 *  far below the module's own 3 mm spec.
 *
 *  This is written as a macro (a text-substitution "function" expanded
 *  by the preprocessor before compilation, not a real function call) so
 *  it's cheap to use inline and works on whatever integer type is passed
 *  in. `us` here is the ECHO pulse width in microseconds; "out and back"
 *  means the sound travels to the obstacle AND back, so the raw
 *  distance*2 has to be halved again — that's where the extra factor of
 *  2 in "/ 2000" (instead of "/ 1000") comes from, on top of the usual
 *  m/s-to-mm/us unit conversion.
 */
#define US_TO_MM(us)        (((us) * 343UL) / 2000UL)

/* The ping/echo state machine's states. `typedef enum { ... } name;`
 * defines a new named type that can only ever hold one of these listed
 * values — the compiler will warn about anything else. This is how C
 * expresses "one of a small fixed set of modes" cleanly, and is exactly
 * the same idea as the scan_state_t state machine in sub_scan.c or
 * ultra_state_t in this file's own history below. */
typedef enum {
    ST_IDLE = 0,      /* no measurement running; ready for a new ping() */
    ST_TRIG,          /* TRIG pin is currently held high (the 12us pulse) */
    /* TRIG has been dropped; waiting for ECHO to go high (sound sent, waiting
     * for it to start bouncing back) */
    ST_WAIT_RISE,
    /* ECHO is high; waiting for it to drop (echo received, now measuring how
     * long it stayed high) */
    ST_WAIT_FALL
} ultra_state_t;

/* `volatile` tells the compiler "this variable can change at any time,
 * from code the compiler can't see coming (an interrupt) — never cache
 * its value in a register or optimize away a re-read." Every variable
 * touched by both an ISR and normal task code in this file is marked
 * volatile for that reason; without it, the compiler could wrongly
 * assume a value never changes between two reads and produce buggy
 * optimized code. */
static volatile ultra_state_t state = ST_IDLE;
/* microsecond timestamp when ECHO last went high, set in echo_isr */
static volatile uint32_t      t_rise;
/* servo angle this ping was taken at, so the result can be matched back to a
 * bearing */
static volatile int16_t       tag_angle;
/* optional direct callback registered via drv_ultrasonic_on_result() */
static drv_ultra_cb_t         user_cb;
/* the free-form context pointer handed back unchanged to user_cb */
static void                  *user_ctx;
/* handle for the registered "bottom half" deferred-work function (see
 * rc_defer_register below) */
static int32_t                defer_h = -1;

/* Set by an ISR, consumed by the bottom half (see finish_i / ultra_drain
 * below for what "bottom half" means here — the short version: the ISR
 * can't safely do division or publish events, so it just drops these
 * three values and asks a normal task to pick them up). */
/* how long ECHO stayed high, in microseconds — the raw measurement */
static volatile uint32_t      result_width_us;
/* true if a real echo was measured; false if we timed out instead */
static volatile bool          result_valid;
/* true once finish_i() has something for ultra_drain() to process */
static volatile bool          result_ready;

/* ------------------------------------------------------------------ *
 *  TIMER alarm helpers
 * ------------------------------------------------------------------ */

/* Arms hardware alarm slot `n` to fire `delay_us` microseconds from now.
 * `1U << n` is a "bitwise shift" — it produces a number with a single 1
 * bit in position `n` (e.g. n=1 gives binary ...00010). Hardware
 * registers like these are commonly organised as one bit per feature, so
 * shifting a 1 into the right position and OR-ing/AND-ing it in is the
 * standard way to touch just one alarm slot's bit without disturbing the
 * others sharing the same register. */
static void alarm_arm(uint32_t n, uint32_t delay_us)
{
    out_w(TIMER_INTR, (1U << n));           /* clear any stale flag */
    /* enable (unmask) this alarm's interrupt */
    set_w(TIMER_INTE, (1U << n));
    /* Writing the alarm register is what arms it: the hardware timer
     * counts up in TIMER_TIMERAWL, so "current time + delay" is the raw
     * tick count at which the alarm should fire. Reading the current
     * time here and adding to it is the whole "schedule an interrupt in
     * the future" mechanism — no software loop involved. */
    out_w(TIMER_ALARM_REG(n), in_w(TIMER_TIMERAWL) + delay_us);
}

/* Disarms hardware alarm slot `n` before it fires (e.g. because the
 * measurement already finished by another path, so the timeout alarm is
 * no longer needed). */
static void alarm_cancel(uint32_t n)
{
    /* disable (mask) this alarm's interrupt */
    clr_w(TIMER_INTE, (1U << n));
    out_w(TIMER_ARMED, (1U << n));          /* write 1 to disarm */
    /* clear any pending flag so it can't fire late */
    out_w(TIMER_INTR, (1U << n));
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
static void finish_i(uint32_t width_us, bool valid)
{
    (void)rc_gpioirq_enable(RC_PIN_ULTRA_ECHO, false);
    alarm_cancel(ALARM_TMO);

    result_width_us = width_us;
    result_valid    = valid;
    result_ready    = true;
    state           = ST_IDLE;

    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Task context.
 * ------------------------------------------------------------------ */

/* The "bottom half" — runs later, in normal task context (not an
 * interrupt), where it's safe to do the division in US_TO_MM, build an
 * event struct, and call rc_event_publish/user_cb. It was queued by
 * rc_defer_signal_i() inside finish_i() above; the scheduler runs it the
 * next time the deferred-work task gets a turn, which for a car running
 * at these loop rates is a matter of microseconds, not a noticeable
 * delay. */
static void ultra_drain(void *ctx)
{
    rc_event_t evt;
    uint32_t   mm = 0U;
    uint32_t   width;
    bool       valid;

    (void)ctx;

    if (!result_ready) {
        return;
    }
    result_ready = false;

    width = result_width_us;
    valid = result_valid;

    if (valid) {
        mm = US_TO_MM(width);
        if ((mm > RC_ULTRA_MAX_MM) || (mm < RC_ULTRA_MIN_MM)) {
            valid = false;              /* out of the module's range */
        }
    }

    /* Publish on the shared event bus (see TEAM_GUIDE.md §1.2) so any
     * subscriber — sub_scan.c's on_range(), and anything else that cares
     * — finds out a measurement completed, tagged with the angle it was
     * taken at. */
    evt.id = RC_EVT_ULTRA_RESULT;
    evt.u.ultra.angle_deg = tag_angle;
    evt.u.ultra.range_mm  = (uint16_t)mm;
    evt.u.ultra.valid     = valid;
    (void)rc_event_publish(&evt);

    /* Also call the optional direct callback, if one was registered via
     * drv_ultrasonic_on_result() — a second, more immediate way to learn
     * the result without going through the event bus. */
    if (user_cb != NULL) {
        user_cb((uint16_t)mm, valid, user_ctx);
    }
}

/* ------------------------------------------------------------------ *
 *  Interrupt handlers
 * ------------------------------------------------------------------ */

/*
 *  ===== STEP 2 of the 5-step ping/echo relay =====
 *  Hardware event that triggers this: TIMER alarm 1 (ALARM_TRIG) fires,
 *  exactly RC_ULTRA_TRIG_US (12us) after ping() armed it in step 1. This
 *  is the chip's own clock jumping straight to this function — nothing
 *  in software was sitting and waiting for it.
 *
 *  What it does: the 12us "shout" pulse on TRIG has now run its full
 *  length, so drop TRIG back to low (the HC-SR04 datasheet says a pulse
 *  this short is enough to make it emit a burst of 40kHz sound). Then
 *  un-mask (enable) the ECHO pin's interrupt — from this instant on, any
 *  voltage change on ECHO will itself trigger an interrupt (steps 3/4
 *  below). Finally arm the timeout alarm (ALARM_TMO/step 5) as a safety
 *  net, in case the echo never comes back at all (nothing in front of
 *  the sensor, or a wiring fault).
 */
static void trig_done_handler(UINT intno)
{
    (void)intno;
    /* acknowledge this alarm's interrupt flag */
    out_w(TIMER_INTR, (1U << ALARM_TRIG));
    /* mask it again — one-shot, not recurring */
    clr_w(TIMER_INTE, (1U << ALARM_TRIG));

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
 *  ===== STEP 5 of the 5-step ping/echo relay (only runs if the echo is
 *  lost) =====
 *  Hardware event that triggers this: TIMER alarm 2 (ALARM_TMO) fires,
 *  RC_ULTRA_TIMEOUT_US after step 2 armed it — meaning ECHO never rose,
 *  or rose but never fell, within the allowed window.
 *
 *  What it does: gives up and reports an invalid (failed) reading rather
 *  than leaving the scan waiting forever for a result that may never
 *  arrive (an obstacle can be too far away, too soft to bounce sound, or
 *  angled away from the sensor — the HC-SR04 simply won't always see an
 *  echo). The `state != ST_IDLE` check guards against a rare race where
 *  the echo already completed normally (see step 4) a moment before this
 *  timeout fired — in that case there is nothing left to time out.
 */
static void timeout_handler(UINT intno)
{
    (void)intno;
    out_w(TIMER_INTR, (1U << ALARM_TMO));

    if (state != ST_IDLE) {
        /* 0 width, not valid — see finish_i/ultra_drain */
        finish_i(0U, false);
    }
    EndOfInt(INTNO_TIMER_2);
}

/*
 *  ===== STEPS 3 and 4 of the 5-step ping/echo relay =====
 *  Hardware event that triggers this: the ECHO pin's voltage changes
 *  (either edge — rising or falling), which is what RC_EDGE_BOTH in
 *  drv_ultrasonic_init() registered this ISR for. The HC-SR04 raises
 *  ECHO the instant it starts listening and drops it the instant it
 *  hears the sound come back, so the pin itself IS the stopwatch — we
 *  just need to time-stamp the two edges.
 *
 *  Step 3 (rising edge, level == true, while state == ST_WAIT_RISE):
 *  the echo pulse has begun. Record the current timestamp in t_rise and
 *  move to ST_WAIT_FALL — now waiting for it to end.
 *
 *  Step 4 (falling edge, level == false, while state == ST_WAIT_FALL):
 *  the echo pulse has ended. `t_us - t_rise` is exactly how long ECHO
 *  stayed high, in microseconds — that duration is hand-carried into
 *  finish_i(), which stashes it for the bottom half (ultra_drain) to
 *  convert into millimetres via US_TO_MM. The actual division happens
 *  outside the ISR — see the comment on finish_i for why.
 */
static void echo_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    (void)pin;
    (void)ctx;

    if ((state == ST_WAIT_RISE) && level) {
        t_rise = t_us;
        state  = ST_WAIT_FALL;
    } else if ((state == ST_WAIT_FALL) && !level) {
        finish_i(t_us - t_rise, true);
    } else {
        /* Spurious edge, ignore. Happens on a noisy 5 V divider —
         * electrical ringing on the wire can look like extra edges to
         * the interrupt hardware even though nothing real changed. */
    }
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

/* Sets up the hardware for the whole 5-step relay: TRIG as an output pin,
 * the ECHO pin's interrupt (registered now but left masked/disabled —
 * see the comment in drv_ultrasonic.h), and the two TIMER alarm
 * interrupt handlers (trig_done_handler for alarm 1, timeout_handler for
 * alarm 2). Also registers the "bottom half" deferred-work function
 * (ultra_drain) that does the actual math and event publishing outside
 * of interrupt context. Call once at boot. */
rc_result_t drv_ultrasonic_init(void)
{
    T_DINT      dint;
    rc_result_t res;

    /* rc_defer_register hands back a small integer "handle" identifying
     * this deferred-work slot; rc_defer_signal_i() (called from finish_i,
     * inside an ISR) uses that handle to say "run ultra_drain soon,"
     * without the ISR itself calling ultra_drain directly. */
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

/* Stores the pointer to the caller's callback function (and its context
 * pointer) so ultra_drain() can call it later when a result is ready.
 * See the drv_ultra_cb_t typedef in the header for the callback's shape. */
rc_result_t drv_ultrasonic_on_result(drv_ultra_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/*
 *  ===== STEP 1 of the 5-step ping/echo relay =====
 *  Called by ordinary code (e.g. sub_scan.c's step_to()), not by an
 *  interrupt. Kicks off one measurement and returns immediately.
 *
 *  What it does: if a measurement is already running, refuses with
 *  RC_ERR_BUSY. Otherwise records which angle this ping is tagged with
 *  (so the eventual result can be matched to a bearing), raises TRIG
 *  high, and arms TIMER alarm 1 to fire again in RC_ULTRA_TRIG_US (12us)
 *  — which is step 2, trig_done_handler, picking up from here.
 */
rc_result_t drv_ultrasonic_ping(int16_t tag_angle_deg)
{
    uint32_t sts;

    /* DI()/EI() = Disable Interrupts / Enable Interrupts. This briefly
     * stops ALL interrupts (a "critical section") for the few
     * instructions that check-then-set `state`, so an ISR can't sneak in
     * between the check and the set and see/leave the state
     * inconsistent. `sts` saves the previous interrupt-enable status so
     * EI() can restore it exactly rather than assuming interrupts were
     * on before. Kept as short as absolutely possible since nothing else
     * can run while interrupts are off. */
    DI(sts);
    if (state != ST_IDLE) {
        EI(sts);
        return RC_ERR_BUSY;
    }
    state     = ST_TRIG;
    tag_angle = tag_angle_deg;
    EI(sts);

    /* start the 12us "shout" pulse */
    (void)gpio_set_val(RC_PIN_ULTRA_TRIG, 1U);
    alarm_arm(ALARM_TRIG, RC_ULTRA_TRIG_US);     /* schedule step 2 to end it */

    return RC_OK;
}

/* True while any step of the 5-step relay is in progress (state is
 * anything other than idle). */
bool drv_ultrasonic_busy(void)
{
    return (state != ST_IDLE);
}
