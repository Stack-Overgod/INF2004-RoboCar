/*
 *  drv_ir.c
 */
#include "../rc_prelude.h"
#include "drv_ir.h"
#include "../rc_config.h"
#include "../rc_gpioirq.h"
#include "../rc_event.h"
#include "../rc_defer.h"
#include "../rc_time.h"

/*
 *  Sensor polarity.*/
#define IR_ACTIVE_HIGH      (1)

#define ADC_DEVNAME         ((UB *)"adca")

/* ADC device descriptor once opened; -1/invalid until drv_ir_init() succeeds */
static ID       adc_dd = -1;
/* optional extra callback for raw barcode edges, set via
 * drv_ir_on_barcode_edge() */
static drv_ir_edge_cb_t bar_cb;
/* context pointer passed back to bar_cb untouched */
static void            *bar_ctx;
/* timestamp (microseconds) of the previous barcode edge, to compute the next
 * width */
static volatile uint32_t bar_last_us;
/* Record Edge boolean */
static volatile bool     bar_enabled;
/* handle for the "bottom half" deferred-work registration, see
 * rc_defer_register() */
static int32_t           defer_h = -1;

#define BAR_RING_SZ     (32U)
#define BAR_RING_MASK   (BAR_RING_SZ - 1U)

typedef struct {
    /* how long (microseconds) the level held before this edge */
    uint32_t width_us;
    /* the new level after this edge: true = just went high, false = just went
     * low */
    bool     level;
} bar_edge_rec_t;

/* the ring buffer's backing storage */
static bar_edge_rec_t    bar_ring[BAR_RING_SZ];
static volatile uint32_t bar_head;   /* next slot the ISR will write to */
/* next slot the bottom half will read from */
static volatile uint32_t bar_tail;
/* count of edges dropped because the ring filled up before being drained */
static volatile uint32_t bar_overrun;

/* ------------------------------------------------------------------ *
 *  Barcode edge ISR
 * ------------------------------------------------------------------ */
static void barcode_isr(uint32_t pin, bool level, uint32_t t_us, void *ctx)
{
    uint32_t width;
    uint32_t next;

    /*  the (void) casts tell the compiler that's intentional so it doesn't warn. */
    (void)pin;
    (void)ctx;

    if (!bar_enabled) {
        return;
    }

    /* bar_width is the data for Code 39, so the timestamp this handler
     *  was given must not be delayed by work that could happen later.*/
    width       = t_us - bar_last_us;
    bar_last_us = t_us;

    /* Compute where the next write would land without committing.
     * If land on `bar_tail`  ring is full — we must drop this edge.*/
    next = (bar_head + 1U) & BAR_RING_MASK;
    if (next == bar_tail) {
        bar_overrun++;      /* decoder is not keeping up, count it */
        return;
    }

    bar_ring[bar_head].width_us = width;
    bar_ring[bar_head].level    = level;
    bar_head = next;

    /* Hand off to the barcode_drain function, which runs
     * later in normal task context rather than inside this interrupt.
     * A normal kernel call here could corrupt scheduler state. */
    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Bottom half. Publishes one event per buffered edge, in order.
 * ------------------------------------------------------------------ */

/* Safer counterpart to barcode_isr() * above. Runs in normal task context */
static void barcode_drain(void *ctx)
{
    rc_event_t     evt;
    bar_edge_rec_t rec;

    (void)ctx;

    while (bar_tail != bar_head) {
        rec      = bar_ring[bar_tail];
        bar_tail = (bar_tail + 1U) & BAR_RING_MASK;

        evt.id = RC_EVT_BARCODE_EDGE;
        evt.u.bar_edge.level_high = rec.level;
        evt.u.bar_edge.width_us   = rec.width_us;
        (void)rc_event_publish(&evt);

        if (bar_cb != NULL) {
            bar_cb(rec.level, rec.width_us, bar_ctx);
        }
    }
}

uint32_t drv_ir_barcode_overruns(void)
{
    return bar_overrun;
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

/* Sets up the two line-sensor GPIO pins, attaches
 * barcode ISR, and opens ADC device*/
rc_result_t drv_ir_init(void)
{
    rc_result_t res;

    defer_h = rc_defer_register(barcode_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    /* Line sensors: plain inputs, polled. The comparator output is a
     * push-pull drive on both modules, so no pull is needed. */
    (void)gpio_set_pin(RC_PIN_IR_LINE_L, GPIO_MODE_IN);
    (void)gpio_set_pin(RC_PIN_IR_LINE_R, GPIO_MODE_IN);

    /* Barcode digital pin, interrupt driven, masked until armed. */
    res = rc_gpioirq_attach(RC_PIN_IR_BARCODE_D, RC_EDGE_BOTH, 0,
                            barcode_isr, NULL);
    if (res != RC_OK) {
        return res;
    }
    (void)rc_gpioirq_enable(RC_PIN_IR_BARCODE_D, false);
    bar_enabled = false;

    /* Barcode analogue pin, for threshold calibration. Opening the ADC
     * is allowed to fail: the digital path still works without it. */
    adc_dd = tk_opn_dev(ADC_DEVNAME, TD_READ);

    return RC_OK;
}

/* ------------------------------------------------------------------ *
 *  Reading
 * ------------------------------------------------------------------ */

/* Reads one sensor's digital state and returns whether it currently sees
 * the black line, correcting for board polarity (see IR_ACTIVE_HIGH). */
bool drv_ir_on_line(rc_ir_ch_t ch)
{
    uint32_t pin;
    uint32_t val;

    switch (ch) {
    case RC_IR_LINE_L:
        pin = RC_PIN_IR_LINE_L;
        break;
    case RC_IR_LINE_R:
        pin = RC_PIN_IR_LINE_R;
        break;
    case RC_IR_BARCODE:
        pin = RC_PIN_IR_BARCODE_D;
        break;
    default:
        return false;
    }

    val = gpio_get_val(pin);

    /* decides which branch to actually compile based on the
     * IR_ACTIVE_HIGH macro's value*/
#if IR_ACTIVE_HIGH
    return (val != 0U);
#else
    return (val == 0U);
#endif
}

/* Reads the raw ADC brightness value (0..4095) on the barcode sensor's
 * analogue pin.*/
uint16_t drv_ir_read_raw(rc_ir_ch_t ch)
{
    UW  buf = 0U;
    SZ  asz = 0;
    ER  er;

    if ((ch != RC_IR_BARCODE) || (adc_dd <= E_OK)) {
        return 0U;
    }

    /* The mtk3 ADC driver takes the channel number as the start
     * position of the read. */
    er = tk_srea_dev(adc_dd, (W)RC_ADC_CH_IR_BARCODE, &buf, 1, &asz);
    if (er != E_OK) {
        return 0U;
    }
    return (uint16_t)(buf & 0x0FFFU);
}

/* Polls both line sensors, turns the two on/off readings into a coarse sideways
 * position number, and publishes it as RC_EVT_LINE_SAMPLE */
void drv_ir_sample_line(void)
{
    rc_event_t evt;
    bool       l = drv_ir_on_line(RC_IR_LINE_L);
    bool       r = drv_ir_on_line(RC_IR_LINE_R);
    int16_t    pos;

    if (l && r) {
        pos = 0;
    } else if (l) {
        pos = -500;
    } else if (r) {
        pos = 500;
    } else {
        pos = 0;                        /* sub_line handles the lost case */
    }

    evt.id = RC_EVT_LINE_SAMPLE;
    evt.u.line.on_line_l  = l;
    evt.u.line.on_line_r  = r;
    evt.u.line.position   = pos;
    evt.u.line.barcode_raw = drv_ir_read_raw(RC_IR_BARCODE);

    (void)rc_event_publish(&evt);
}

/* Registers an extra callback invoked from INTERRUPT context (see
 * barcode_drain()) for every raw barcode edge */
rc_result_t drv_ir_on_barcode_edge(drv_ir_edge_cb_t cb, void *ctx)
{
    bar_cb  = cb;
    bar_ctx = ctx;
    return RC_OK;
}

/* Arms or disarms the barcode interrupt. */
rc_result_t drv_ir_barcode_enable(bool on)
{
    if (on) {
        bar_last_us = rc_time_us();
        bar_head    = 0U;
        bar_tail    = 0U;
    }
    bar_enabled = on;
    return rc_gpioirq_enable(RC_PIN_IR_BARCODE_D, on);
}
/*** end of file ***/