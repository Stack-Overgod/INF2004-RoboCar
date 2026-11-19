/*
 *  drv_encoder.c  -  Buddy 2 hardware layer
 *
 *  Reads the two wheel encoders and turns raw "click" edges into speed
 *  (mm/s) and distance (mm). Each encoder has two channels: A raises the
 *  interrupt and is counted; B is sampled at that instant to tell which
 *  way the wheel is turning. See drv_encoder.h for the odometry
 *  explanation.
 */
#include "../rc_prelude.h"
#include "drv_encoder.h"
#include "../rc_config.h"
#include "../rc_gpioirq.h"
#include "../rc_time.h"

/*
 *  Critical section for the count / period / last-edge fields shared with
 *  encoder_isr(). Single core: masking local interrupts is enough. SMP:
 *  the ISR runs on core 0 while a reader may run on core 1, where masking
 *  interrupts excludes nothing, so both sides also take a spinlock.
 *  sts must be UINT (ISpinLock takes UINT *).
 */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK enc_lock;
#define ENC_LOCK(sts)       ((void)ISpinLock(&enc_lock, &(sts)))
#define ENC_UNLOCK(sts)     ((void)ISpinUnlock(&enc_lock, (sts)))
#else
#define ENC_LOCK(sts)       DI(sts)
#define ENC_UNLOCK(sts)     EI(sts)
#endif

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
 * RC_ENC_TICKS_PER_REV (640) per wheel turn and 200 wheel RPM that gap is
 * ~470 us. */
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
    volatile int8_t   dir;
    uint32_t          pin_a;
    uint32_t          pin_b;
} enc_t;

/* The two encoders, indexed by rc_side_t (LEFT=0, RIGHT=1). Pin numbers
 * come from rc_config.h. */
static enc_t encs[2] = {
    { 0U, 0U, 0U, 0U, 1, RC_PIN_ENC_L_A, RC_PIN_ENC_L_B },
    { 0U, 0U, 0U, 0U, 1, RC_PIN_ENC_R_A, RC_PIN_ENC_R_B }
};

/* ------------------------------------------------------------------ *
 *  ISR (Interrupt Service Routine) - the function the processor jumps
 *  to immediately when an encoder pin changes state, pausing whatever
 *  else it was doing. Runs on processor 1 (core 0) only, per the port's
 *  one-owner-core rule for shared peripheral interrupts.
 *
 *  Everything here is O(1) (fixed, tiny amount of work regardless of
 *  how busy the system is). This is deliberate - see the comment inside
 *  for why.
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
#if TK_SUPPORT_SMP
    UINT     sts;
#endif

    (void)pin;
    (void)level;

    /*
     *  Everything this handler does: one subtraction, one compare, one
     *  GPIO register read and four stores. It makes no kernel call:
     *  sub_motion reads the counts every control period instead.
     *
     *  This matters because the two encoders and the ultrasonic echo pin
     *  share IO_IRQ_BANK0 (one shared interrupt line for a whole bank of
     *  GPIO pins). Time spent here delays the other pins' edge
     *  timestamps, and for the echo the pulse width is the data.
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
#if TK_SUPPORT_SMP
    ENC_LOCK(sts);
#endif
    e->dir       = (gpio_get_val(e->pin_b) != 0U) ? (int8_t)-1 : (int8_t)1;
    e->last_us   = t_us;
    e->period_us = delta;
    e->count++;
#if TK_SUPPORT_SMP
    ENC_UNLOCK(sts);
#endif
}

/* ------------------------------------------------------------------ */

/* Boot-time setup: attaches the rising-edge interrupt handler to each
 * encoder pin. Must run before any other drv_encoder_* call. */
rc_result_t drv_encoder_init(void)
{
    rc_result_t res;
    uint32_t    i;

    for (i = 0U; i < 2U; i++) {
        encs[i].count      = 0U;
        encs[i].last_us    = rc_time_us();
        encs[i].period_us  = 0U;
        encs[i].base_count = 0U;
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
    UINT     sts;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }

    /* Read both under one critical section: an edge landing between the
     * two reads would pair a fresh timestamp with a stale period. */
    ENC_LOCK(sts);
    period = encs[side].period_us;
    last   = encs[side].last_us;
    ENC_UNLOCK(sts);

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
 * cruise-control analogy in docs/buddy2-motion. */
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
    UINT     sts;

    if (side > RC_SIDE_RIGHT) {
        return 0U;
    }
    /* Both under the lock: drv_encoder_reset() may move base_count from
     * the other core between the two reads. */
    ENC_LOCK(sts);
    ticks = encs[side].count - encs[side].base_count;
    ENC_UNLOCK(sts);

    return (uint32_t)((ticks * RC_ENC_UM_PER_TICK) / 1000UL);
}

/* Zero the "distance since reset" baseline (not the lifetime click
 * count) by snapshotting the current count into base_count. Called at
 * the start of every new move in sub_motion.c so each move's distance is
 * measured from its own starting line. */
void drv_encoder_reset(void)
{
    UINT sts;

    ENC_LOCK(sts);
    encs[RC_SIDE_LEFT].base_count  = encs[RC_SIDE_LEFT].count;
    encs[RC_SIDE_RIGHT].base_count = encs[RC_SIDE_RIGHT].count;
    ENC_UNLOCK(sts);
}
