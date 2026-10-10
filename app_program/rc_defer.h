/*
 *  rc_defer.h
 *
 *  Bottom halves. The mechanism that keeps every ISR in this tree down to
 *  stores and register writes.
 *
 *  The rule this tree follows
 *  --------------------------
 *  An interrupt handler may do ONLY:
 *
 *    - read the microsecond counter
 *    - a fixed number of register reads and writes (ack, mask, arm)
 *    - stores into its own driver's state, no loops over data
 *    - one subtraction, if it needs an elapsed time
 *    - rc_defer_signal_i(), which is a single tk_set_flg
 *
 *  It may NOT publish events, run a decoder, do division, or call a
 *  user-supplied callback. All of that happens in the bottom-half task.
 *
 *  Why this matters here specifically: the two encoder pins, the echo pin
 *  and the barcode pin all share one vector, IO_IRQ_BANK0. Time spent in
 *  any one handler is time the other three are blocked. A barcode edge
 *  arriving while the encoder handler is publishing an event is a barcode
 *  edge whose timestamp is now wrong, and bar width IS the data.
 *
 *  The bottom-half task runs above both event dispatchers, so a drain
 *  normally happens within microseconds of the ISR that requested it.
 */

#ifndef RC_DEFER_H
#define RC_DEFER_H

#include "rc_types.h"

/* Runs in the bottom-half TASK. May publish events and call user
 * callbacks. Must still return promptly: it is above the dispatchers. */
typedef void (*rc_defer_drain_t)(void *ctx);

rc_result_t rc_defer_init(void);

/* Returns a handle, or negative on failure. Up to 16 drains. */
int32_t rc_defer_register(rc_defer_drain_t drain, void *ctx);

/* ISR safe. One tk_set_flg, nothing else. */
void rc_defer_signal_i(int32_t handle);

/* Number of times a drain was requested. Useful for spotting a driver
 * that is signalling far more often than expected. */
uint32_t rc_defer_count(void);

#endif /* RC_DEFER_H */
