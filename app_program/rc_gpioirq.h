/*
 *  rc_gpioirq.h
 *
 *  Per-pin edge callbacks on top of the single IO_IRQ_BANK0 vector.
 *
 *  The RP2040 gives all 30 GPIOs one interrupt line. Three drivers need
 *  edges (two encoders, the ultrasonic echo, the barcode sensor), so
 *  something has to demultiplex. This module owns the vector and hands
 *  each pin to its registered handler.
 *
 *  Handler contract
 *  ----------------
 *   - Runs in INTERRUPT context, not task context.
 *   - Must be short. Timestamp, update a counter, publish an event,
 *     return. No printing, no waiting, no tk_ calls other than the ones
 *     documented as interrupt safe (tk_set_flg, tk_sig_sem, tk_isig_tim).
 *   - `level` is the pin state read at entry, which is a hint rather than
 *     a guarantee on a fast edge.
 */

#ifndef RC_GPIOIRQ_H
#define RC_GPIOIRQ_H

#include "rc_types.h"

typedef enum {
    RC_EDGE_FALL = 0x1,
    RC_EDGE_RISE = 0x2,
    RC_EDGE_BOTH = 0x3
} rc_edge_t;

typedef void (*rc_gpio_isr_t)(uint32_t pin, bool level, uint32_t t_us,
                              void *ctx);

/*
 *  Claim IO_IRQ_BANK0 and clear the table. Call once, before any driver
 *  that registers a pin.
 *
 *  Note for SMP builds: the vector is enabled in core 0's NVIC only, per
 *  the port's rule that a shared peripheral IRQ has exactly one owner
 *  core. Handlers therefore always run on processor 1.
 */
rc_result_t rc_gpioirq_init(void);

/*
 *  Route edges on `pin` to `cb`. Configures the pin as an input with the
 *  requested pull, enables the chosen edges, and starts delivery.
 *  pull: -1 pull-down, 0 none, +1 pull-up.
 */
rc_result_t rc_gpioirq_attach(uint32_t pin,
                              rc_edge_t edges,
                              int8_t pull,
                              rc_gpio_isr_t cb,
                              void *ctx);

rc_result_t rc_gpioirq_detach(uint32_t pin);

/* Mask or unmask one pin without losing its registration. Used by the
 * ultrasonic driver, which only wants echo edges while a ping is live. */
rc_result_t rc_gpioirq_enable(uint32_t pin, bool on);

#endif /* RC_GPIOIRQ_H */
