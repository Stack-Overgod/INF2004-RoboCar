/*
 *  drv_encoder.c
 *
 *  Reads the two wheel encoders and turns raw "click" edges into speed
 *  (mm/s) and distance (mm). Each encoder has two channels: A raises the
 *  interrupt and is counted; B is sampled at that instant to tell which
 *  way the wheel is turning. See drv_encoder.h for the odometry
 *  explanation.
 */
#include "rc_prelude.h"
#include "drv_encoder.h"
#include "rc_config.h"
#include "rc_gpioirq.h"
#include "rc_event.h"
#include "rc_defer.h"
#include "rc_time.h"

/* If no new edge arrives within this many microseconds, treat the wheel
 * as stopped rather than as "very slow". The encoder counts the motor
 * shaft before the gearbox, so a moving wheel gives hundreds of ticks per
 * turn: 100 ms without one means the wheel is creeping at a few mm/s at
 * most, which the PID can safely treat as stopped. */
#define STALL_TIMEOUT_US    (100000UL)

/* Minimum microseconds between two edges for them to count as separate
 * ticks ("debouncing"). The Hall-effect encoders on these motors switch
 * cleanly through the pin's Schmitt trigger, so this only rejects short
 * electrical glitches (motor PWM noise picked up by the encoder wires).
 * It must stay well below the real gap between ticks at top speed: at
 * 1000 ticks per wheel turn and 200 wheel RPM that gap is 300 us. The old
 * 500 us, chosen for a 20-slot optical disc, would throw away real ticks
 * at speed. */
#define DEBOUNCE_US         (50UL)

/* Per-wheel encoder state.
 * "volatile" tells the compiler "this value can change at any moment
 * from outside normal program flow (here, from an interrupt), so never
 * cache it in a register - always re-read it from memory." It's required
 * on every field the ISR writes and the rest of the code reads.
 *
 *   count       - total edges (clicks) seen since boot, only ever grows.
 *   last_us     - timestamp (microseconds) of the most recent accepted edge.
 *   period_us   - microseconds between the two most recent accepted edges;
 *                 this is what speed is calculated from.
 *   base_count  - a snapshot of count taken at the last drv_encoder_reset(),
 *                 so distance-since-reset = count - base_count.
 *   published   - the count value already turned into an RC_EVT_ENCODER_EDGE
 *                 event, so the bottom half doesn't republish unchanged data.
 *   dir         - +1 forward / -1 reverse, read from channel B on the most
 *                 recent channel-A edge.
 *   pin_a       - channel A GPIO: the one that raises the interrupt.
 *   pin_b       - channel B GPIO: sampled inside the ISR for direction.
 */
typedef struct {
    volatile uint32_t count;
    volatile uint32_t last_us;
    volatile uint32_t period_us;
    volatile uint32_t base_count;   /* snapshot at reset */
    volatile uint32_t published;    /* count already turned into events */
    volatile int8_t   dir;
    uint32_t          pin_a;
    uint32_t          pin_b;
} enc_t;

/* Handle for the "deferred" (bottom-half) worker registered with
 * rc_defer_register() below. An int32_t is used, not a pointer, because
 * that's the handle type the deferred-work system hands back - think of
 * it as a ticket number identifying "the encoder_drain job." */
static int32_t defer_h = -1;

/* The two encoders, indexed by rc_side_t (LEFT=0, RIGHT=1). Pin numbers
 * come from rc_config.h. */
static enc_t encs[2] = {
    { 0U, 0U, 0U, 0U, 0U, 1, RC_PIN_ENC_L_A, RC_PIN_ENC_L_B },
    { 0U, 0U, 0U, 0U, 0U, 1, RC_PIN_ENC_R_A, RC_PIN_ENC_R_B }
};

/* ------------------------------------------------------------------ *
 *  ISR (Interrupt Service Routine) - the function the processor jumps
 *  to immediately when an encoder pin changes state, pausing whatever
 *  else it was doing. Runs on processor 1 only, per the port's
 *  one-owner-core rule for shared peripheral interrupts.
 *
 *  Everything here is O(1) (fixed, tiny amount of work regardless of
 *  how busy the system is): a subtraction, two stores, one ring push.
 *  This is deliberate - see the comment inside for why.
 * ------------------------------------------------------------------ */

/* Parameters: pin/level identify which pin and whether it went high or
 * low (unused here - only rising edges are attached, see drv_encoder_init
 * below); t_us is the timestamp of the edge in microseconds; ctx is a
 * (void *) - a generic untyped pointer - that was registered alongside
 * this handler and is cast back to the enc_t this edge belongs to. Using
 * a void* + cast is the C way of giving one shared handler function
 * "per-instance data" without needing separate functions per encoder. */
static void encoder_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    enc_t   *e = (enc_t *)ctx;
    uint32_t delta;

    (void)pin;
    (void)level;

    /*
     *  Everything this handler does: one subtraction, one compare, one
     *  GPIO register read, four stores, one tk_set_flg (inside
     *  rc_defer_signal_i). No event is built here and nothing is
     *  published. The bottom half does that.
     *
     *  This matters because the two encoders, the echo pin and the barcode
     *  pin all share IO_IRQ_BANK0 (one shared interrupt line for a whole
     *  bank of GPIO pins). Time spent here is time a barcode edge
     *  timestamp is being delayed, and bar width is the data - see
     *  TEAM_GUIDE.md §1.2's "golden rule" for interrupt code.
     */
    delta = t_us - e->last_us;          /* wrap safe: unsigned subtraction
                                          * still gives the right answer
                                          * even if t_us's counter rolled
                                          * over past its max value */
    if (delta < DEBOUNCE_US) {
        return;                          /* electrical glitch, ignore */
    }

    /* Quadrature direction: the two channels are 90 degrees out of phase,
     * so at the instant A rises, B is still low in one direction and
     * already high in the other. Which is "forward" depends on how the
     * encoder is mounted - if a wheel reads backwards, swap its A and B
     * wires. One register read, allowed inside an ISR. */
    e->dir       = (gpio_get_val(e->pin_b) != 0U) ? (int8_t)-1 : (int8_t)1;
    e->last_us   = t_us;
    e->period_us = delta;
    e->count++;

    /* Hand off to the bottom half instead of doing real work here - see
     * the block comment above encoder_drain() below. The "_i" suffix is
     * this RTOS's convention for "safe to call from inside an interrupt." */
    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half ("deferred" work). Runs in normal task context (not
 *  inside an interrupt), so it's safe to do slightly more work here,
 *  including publishing onto the event bus.
 *
 *  Edges are coalesced: if several arrived before the drain ran, one
 *  event carries the latest count rather than replaying each edge. The
 *  consumers of RC_EVT_ENCODER_EDGE care about the running count and the
 *  latest period, not about receiving exactly one event per slot, and
 *  coalescing keeps a fast wheel from flooding the bus.
 * ------------------------------------------------------------------ */

static void encoder_drain(void *ctx)
{
    uint32_t   i;
    uint32_t   count;
    uint32_t   period;
    uint32_t   sts;
    rc_event_t evt;

    (void)ctx;

    for (i = 0U; i < 2U; i++) {
        /* DI()/EI() = "disable interrupts" / "enable interrupts" (sts
         * saves the previous interrupt state so EI restores it exactly).
         * This is a "critical section": without it, an ISR could update
         * count/period_us halfway through this read and hand back a
         * torn, inconsistent pair of values. */
        DI(sts);
        count  = encs[i].count;
        period = encs[i].period_us;
        EI(sts);

        if (count == encs[i].published) {
            continue;   /* nothing new since the last drain, skip */
        }
        encs[i].published = count;

        evt.id = RC_EVT_ENCODER_EDGE;
        evt.u.encoder.side     = (i == 0U) ? RC_SIDE_LEFT : RC_SIDE_RIGHT;
        evt.u.encoder.count    = count;
        evt.u.encoder.delta_us = period;
        (void)rc_event_publish(&evt);
    }
}

/* ------------------------------------------------------------------ */

/* Boot-time setup: registers the bottom-half worker and attaches the
 * rising-edge interrupt handler to each encoder pin. Must run before any
 * other drv_encoder_* call. */
rc_result_t drv_encoder_init(void)
{
    rc_result_t res;
    uint32_t    i;

    defer_h = rc_defer_register(encoder_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    for (i = 0U; i < 2U; i++) {
        encs[i].count      = 0U;
        encs[i].last_us    = rc_time_us();
        encs[i].period_us  = 0U;
        encs[i].base_count = 0U;
        encs[i].published  = 0U;
        encs[i].dir        = 1;

        /* Channel B is a plain input with a pull-up and Schmitt trigger,
         * same pad setup rc_gpioirq_attach gives channel A. No interrupt
         * on B: the ISR reads its level when A fires. */
        (void)gpio_set_pin(encs[i].pin_b, GPIO_MODE_IN);
        out_w(GPIO(encs[i].pin_b), GPIO_IE | GPIO_SHEMITT | GPIO_PUE);

        /* Rising edges of A only. Counting both edges would double the
         * resolution but the mark/space of the disc is not symmetric, so
         * the widths would alternate and ruin the period estimate. */
        res = rc_gpioirq_attach(encs[i].pin_a, RC_EDGE_RISE, 1,
                                encoder_isr, &encs[i]);
        if (res != RC_OK) {
            return res;
        }
    }
    return RC_OK;
}

/* Direction of the most recent edge: +1 forward, -1 reverse. */
int8_t drv_encoder_dir(rc_side_t side)
{
    return (side > RC_SIDE_RIGHT) ? (int8_t)0 : encs[side].dir;
}

/* Raw click count since boot for one wheel - never resets. */
uint32_t drv_encoder_count(rc_side_t side)
{
    return (side > RC_SIDE_RIGHT) ? 0U : encs[side].count;
}

/* Microseconds between the two most recent edges on one wheel - or the
 * time since the last edge, if that is already longer (the wheel is
 * slowing down). 0 if no edge has been seen yet, or the wheel has gone
 * quiet for longer than STALL_TIMEOUT_US. */
uint32_t drv_encoder_period_us(rc_side_t side)
{
    uint32_t period;
    uint32_t last;
    uint32_t since;
    uint32_t sts;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }

    /* Read both under one critical section: an edge landing between the
     * two reads would pair a fresh timestamp with a stale period. */
    DI(sts);
    period = encs[side].period_us;
    last   = encs[side].last_us;
    EI(sts);

    since = rc_time_us() - last;        /* wrap safe, as in the ISR */
    if ((period == 0U) || (since > STALL_TIMEOUT_US)) {
        return 0U;                      /* no tick yet, or stopped */
    }

    /* If more time has passed since the last tick than the last gap
     * between ticks, the wheel is slower than that gap says - it is at
     * most this fast. Using the longer time makes the speed fall smoothly
     * while a wheel stops, instead of freezing at the last reading until
     * the stall timeout and misleading the PID. */
    if (since > period) {
        period = since;
    }
    return period;
}

/* Converts "microseconds between clicks" into "millimetres per second."
 * This is the number sub_motion.c's PID loop compares against the target
 * speed every control cycle - it's the "actual speed" half of the
 * cruise-control analogy in the Buddy 2 guide. */
int32_t drv_encoder_speed_mm_s(rc_side_t side)
{
    uint32_t period = drv_encoder_period_us(side);
    int32_t  speed;

    if (period == 0U) {
        return 0;
    }

    /* RC_ENC_UM_PER_TICK (micrometres travelled per click, from
     * rc_config.h) divided by the microsecond period between clicks gives
     * micrometres-per-microsecond, i.e. metres/second, scaled by 1e6/1e6.
     * Multiplying the numerator by 1000 first shifts that result directly
     * into millimetres/second while staying in whole-number (integer)
     * math the whole way - there's no FPU on this chip, so no division
     * can be left as a fraction; it must land on a usable integer unit
     * immediately. */
    speed = (int32_t)((RC_ENC_UM_PER_TICK * 1000UL) / period);

    /* Direction comes from channel B, sampled in the ISR on the last edge,
     * so a wheel that is still coasting backwards after a reversal reads
     * negative even though the motor has already been told to go forward. */
    if (encs[side].dir < 0) {
        speed = -speed;
    }
    return speed;
}

/* Distance travelled (in mm) since the last drv_encoder_reset() call -
 * the actual odometry number sub_motion.c watches to know when a
 * "drive N mm" move has reached its goal. */
uint32_t drv_encoder_distance_mm(rc_side_t side)
{
    uint32_t ticks;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }
    ticks = encs[side].count - encs[side].base_count;

    return (uint32_t)((ticks * RC_ENC_UM_PER_TICK) / 1000UL);
}

/* Zero the "distance since reset" baseline (not the lifetime click
 * count) by snapshotting the current count into base_count. Called at
 * the start of every new move in sub_motion.c so each move's distance is
 * measured from its own starting line. */
void drv_encoder_reset(void)
{
    uint32_t sts;

    DI(sts);
    encs[RC_SIDE_LEFT].base_count  = encs[RC_SIDE_LEFT].count;
    encs[RC_SIDE_RIGHT].base_count = encs[RC_SIDE_RIGHT].count;
    EI(sts);
}
