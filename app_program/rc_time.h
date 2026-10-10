/*
 *  rc_time.h
 *
 *  Microsecond timebase, taken straight from the RP2040 TIMER block.
 *
 *  The kernel tick is 1 ms, which is far too coarse for an HC-SR04 echo
 *  (one tick is 170 mm of range) or for barcode bar widths. TIMERAWL is a
 *  free-running 1 MHz counter that needs no interrupt and no kernel call,
 *  so it is safe to read from an ISR.
 *
 *  The counter is 32 bit and wraps every ~71.6 minutes. Unsigned
 *  subtraction handles the wrap correctly, which is why rc_time_since()
 *  exists rather than callers writing `now - then` by hand.
 */

#ifndef RC_TIME_H
#define RC_TIME_H

#include <stdint.h>

/* Latch the TIMER block out of reset. Called by rc_event_init. */
void rc_time_init(void);

/* Free-running microseconds. ISR safe. */
uint32_t rc_time_us(void);

/* Microseconds elapsed since `then`, wrap safe. */
static inline uint32_t rc_time_since(uint32_t then)
{
    return rc_time_us() - then;
}

/* Milliseconds since boot, derived from the same counter. */
static inline uint32_t rc_time_ms(void)
{
    return rc_time_us() / 1000U;
}

#endif /* RC_TIME_H */
