/*
 *  drv_ir.h  -  Buddy 3 hardware layer
 *
 *  Three reflective IR sensors. Two watch the line, one reads barcodes.
 */
#ifndef DRV_IR_H
#define DRV_IR_H

#include "../rc_types.h"

/* Names the three physical IR sensors so calls like drv_ir_on_line(ch)
 * can say which one they mean instead of passing a bare, easy-to-mix-up
 * number. */
typedef enum {
    RC_IR_LINE_L = 0,   /* left line sensor */
    RC_IR_LINE_R,       /* right line sensor */
    RC_IR_BARCODE       /* barcode sensor (digital DOUT only) */
} rc_ir_ch_t;

/* Barcode edge callback. INTERRUPT context. `width_us` is how long the
 *  level that just ended had been held, which is the bar or space width. */
typedef void (*drv_ir_edge_cb_t)(bool new_level, uint32_t width_us, void *ctx);

rc_result_t drv_ir_init(void);

/* Digital state. True means "over the line", after polarity correction. */
bool drv_ir_on_line(rc_ir_ch_t ch);

/* Poll both line sensors and publish RC_EVT_LINE_SAMPLE. Called from the
 * sense task at RC_PERIOD_LINE_MS. */
void drv_ir_sample_line(void);

rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx);

/* Barcode edges are only interesting while the car expects a barcode.
 * Masking when idle keeps a shiny patch of track from flooding the bus. */
rc_result_t drv_ir_barcode_enable(bool on);

/* Edges dropped because the decoder could not keep up. Should stay 0. */
uint32_t drv_ir_barcode_overruns(void);

#endif /* DRV_IR_H */
/*** end of file ***/