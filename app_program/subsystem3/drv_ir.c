/*
 *  drv_ir.c
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "drv_ir.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_fault.h"
#include "../rc_defer.h"
#include "../rc_time.h"

/*
 *  Sensor polarity.*/
#define IR_ACTIVE_HIGH      (1)

/* Barcode sensor, analogue: the AO pin (GP26) is read through the
 * template ADC device every Sense tick, so no trim pot needs setting. DO
 * (GP27) is not used: no IRQ, no pin setup. Level changes come out as
 * RC_EVT_BARCODE_EDGE for sub_barcode.
 *
 * Bars and spaces are found from the shape of the signal, not from one
 * fixed threshold: the sensor spot is about as wide as a narrow element,
 * so a 3 mm space between two bars never reads back down to paper white.
 * - A bar starts once the reading rises RC_BAR_ADC_PROM above the lowest
 *   point of the space before it; a space starts once it falls PROM below
 *   the bar's peak.
 * - A bar only counts if its peak is RC_BAR_ADC_MIN_BAR above the white
 *   level (running minimum), so mat texture is not read as bars.
 * - Each edge is timed where the reading crosses halfway between the
 *   peak and the valley either side of it, from a short history of
 *   readings. An edge is therefore sent one element late, which only
 *   delays the decode; the widths are unchanged. */
#define ADC_DEVNAME         ((UB *)"adca")

/* ADC descriptor, <= 0 if the open failed (barcode then stays silent) */
static ID                adc_dd = -1;
/* Recent readings (black high) and their times, for timing the edges.
 * 64 samples = 640 ms: one wide bar plus its slopes down to ~12 mm/s. */
#define BAR_HIST        (64U)   /* power of 2 */
#define BAR_HIST_MASK   (BAR_HIST - 1U)
/* A bar's falling edge is normally sent when the next bar starts; after
 * this many samples of space with no deeper valley it is sent anyway, so
 * the last bar of a code is not held until the next one. */
#define BAR_QUIET_N     (30U)

/* Sense task only */
static uint16_t          hist_v[BAR_HIST];
static uint32_t          hist_t[BAR_HIST];
static uint32_t          hist_n;        /* sample number of the next reading */
/* white level: running minimum, creeping up 1 count per sample */
static uint16_t          adc_white = 0xFFFFU;
static bool              adc_in_bar;    /* current element is a bar */
static uint16_t          ext_v = 0xFFFFU;   /* extreme of the current element */
static uint32_t          ext_n;
static uint16_t          valley_v;      /* lowest point before the current bar */
static uint32_t          valley_n;
static uint16_t          peak_v;        /* peak of the last accepted bar */
static uint32_t          peak_n;
static bool              fall_due;      /* that bar's falling edge is unsent */
static volatile uint16_t adc_now;       /* latest reading, for diagnostics */
static volatile uint16_t adc_peak;      /* last accepted bar peak, diagnostics */

/* optional extra callback for raw barcode edges, set via
 * drv_ir_on_barcode_edge() */
static drv_ir_edge_cb_t bar_cb;
/* context pointer passed back to bar_cb untouched */
static void            *bar_ctx;
/* Record Edge boolean */
static volatile bool     bar_enabled;
/* handle for the "bottom half" deferred-work registration, see rc_defer_register() */
static int32_t           defer_h = -1;

/* Edges are debounced on the LEVEL before the ring: a new level counts
 * once it has lasted BAR_GLITCH_US; if it goes back sooner, the previous
 * change was a spike and is undone. The PROM step already stops
 * chatter, so this only rejects a level that lasted a single 10 ms sample
 * (a speck of dirt): 1.5 samples. Keep ~3x below the narrow element time
 * (3 mm at ~31 mm/s = ~95 ms). */
#define BAR_GLITCH_US   ((RC_PERIOD_LINE_MS * 1000U * 3U) / 2U)

/* Debounce state (Sense task only). An element is held back (pend_*) until the
 * next real transition, because a spike can still undo the transition
 * that ended it; the last element of a barcode goes out with the next
 * edge. */
static bool              acc_valid;     /* acc_* hold a real transition */
static bool              acc_level;     /* pin level since acc_t        */
static uint32_t          acc_t;         /* time of the last transition  */
static uint32_t          acc_prev_t;    /* time of the one before it    */
static uint32_t          pend_w;        /* element ending at acc_t      */
static bool              pend_level;    /* level after acc_t            */
static bool              pend_valid;

#define BAR_RING_SZ     (64U)   /* power of 2 */
#define BAR_RING_MASK   (BAR_RING_SZ - 1U)

typedef struct {
    /* how long (microseconds) the level held before this edge */
    uint32_t width_us;
    /* the new level after this edge: true = just went high, false = just went low */
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
 *  Barcode level change, called by drv_ir_sample_barcode() in the Sense
 *  task. level true = black (bar).
 * ------------------------------------------------------------------ */
static void bar_level_in(bool level, uint32_t t_us)
{
    uint32_t width;
    uint32_t next;

    if (!bar_enabled) {
        return;
    }

    if (!acc_valid) {
        /* first edge after enabling: only a reference point */
        acc_level  = level;
        acc_t      = t_us;
        acc_prev_t = t_us;
        acc_valid  = true;
        pend_valid = false;
        return;
    }
    if (level == acc_level) {
        return;                     /* bounced back to the same level */
    }
    if ((t_us - acc_t) < BAR_GLITCH_US) {
        /* Changed again too soon: the last transition was a spike. The
         * element before it carries on. */
        acc_level  = level;
        acc_t      = acc_prev_t;
        pend_valid = false;
        return;
    }

    /* A real transition: queue the held element, hold the one that just
     * ended. */
    {
        bool have = pend_valid;

        width = pend_w;
        {
            bool l = pend_level;

            pend_w     = t_us - acc_t;
            pend_level = level;
            pend_valid = true;
            level      = l;
        }
        acc_prev_t = acc_t;
        acc_t      = t_us;
        acc_level  = pend_level;
        if (!have) {
            return;
        }
    }

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

    /* Hand off to barcode_drain(), on the defer task, so the Sense task
     * never waits on the event bus. */
    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Barcode ADC sampling (Sense task, every RC_PERIOD_LINE_MS)
 * ------------------------------------------------------------------ */

/* Reads GP26 once (0..4095), 0xFFFF if the ADC is not open or failed. */
static uint16_t adc_read(void)
{
    UW buf = 0U;
    SZ asz = 0;

    if (adc_dd <= E_OK) {
        return 0xFFFFU;
    }
    /* The mtk3 ADC driver takes the channel number as the start
     * position of the read. */
    if (tk_srea_dev(adc_dd, (W)RC_ADC_CH_IR_BARCODE, &buf, 1, &asz) != E_OK) {
        return 0xFFFFU;
    }
    return (uint16_t)(buf & 0x0FFFU);
}

/* Time at which the readings cross `level` on the way from sample a to
 * sample b, searched back from b and interpolated between two samples.
 * Falls back to the oldest reading still in the history. */
static uint32_t bar_cross_t(uint32_t a, uint32_t b, uint32_t level)
{
    uint32_t oldest = (hist_n > BAR_HIST) ? (hist_n - BAR_HIST) : 0U;
    uint32_t k;

    if (a < oldest) {
        a = oldest;
    }
    if (b < a) {
        return hist_t[a & BAR_HIST_MASK];
    }
    for (k = b; k > a; k--) {
        int32_t v1 = (int32_t)hist_v[k & BAR_HIST_MASK];
        int32_t v0 = (int32_t)hist_v[(k - 1U) & BAR_HIST_MASK];
        int32_t l  = (int32_t)level;

        if (((v0 < l) && (v1 >= l)) || ((v0 > l) && (v1 <= l))) {
            uint32_t t0 = hist_t[(k - 1U) & BAR_HIST_MASK];
            uint32_t t1 = hist_t[k & BAR_HIST_MASK];

            return t0 + (uint32_t)(((int32_t)(t1 - t0) * (l - v0)) / (v1 - v0));
        }
    }
    return hist_t[a & BAR_HIST_MASK];
}

void drv_ir_sample_barcode(void)
{
    uint32_t t = rc_time_us();
    uint16_t v = adc_read();
    uint32_t n;

    if (v == 0xFFFFU) {
        return;
    }
    adc_now = v;
    if (!RC_BAR_ADC_BLACK_HIGH) {
        v = (uint16_t)(0x0FFFU - v);
    }

    n = hist_n;
    hist_v[n & BAR_HIST_MASK] = v;
    hist_t[n & BAR_HIST_MASK] = t;
    hist_n++;

    if (v < adc_white) {
        adc_white = v;
    } else if (adc_white < 0x0FFFU) {
        adc_white++;
    }

    if (!adc_in_bar) {
        if (v < ext_v) {
            ext_v = v;
            ext_n = n;
        }
        /* the last bar ended halfway down to this space's valley */
        if (fall_due && (((uint32_t)v >= (uint32_t)ext_v + RC_BAR_ADC_PROM) ||
                         ((n - ext_n) >= BAR_QUIET_N))) {
            bar_level_in(false, bar_cross_t(peak_n, ext_n,
                                            ((uint32_t)peak_v + ext_v) / 2U));
            fall_due = false;
        }
        if (((uint32_t)v >= (uint32_t)ext_v + RC_BAR_ADC_PROM) &&
            ((uint32_t)v >= (uint32_t)adc_white + RC_BAR_ADC_PROM)) {
            valley_v   = ext_v;
            valley_n   = ext_n;
            adc_in_bar = true;
            ext_v      = v;
            ext_n      = n;
        }
        return;
    }

    if (v > ext_v) {
        ext_v = v;
        ext_n = n;
    }
    if ((uint32_t)v + RC_BAR_ADC_PROM > (uint32_t)ext_v) {
        return;                         /* still on the bar */
    }
    adc_in_bar = false;
    if ((uint32_t)ext_v < (uint32_t)adc_white + RC_BAR_ADC_MIN_BAR) {
        /* too faint for a bar: the space before it carries on */
        ext_v = (v < valley_v) ? v : valley_v;
        ext_n = (v < valley_v) ? n : valley_n;
        return;
    }
    /* the bar began halfway up from the valley before it */
    bar_level_in(true, bar_cross_t(valley_n, ext_n,
                                   ((uint32_t)valley_v + ext_v) / 2U));
    peak_v   = ext_v;
    peak_n   = ext_n;
    adc_peak = ext_v;
    fall_due = true;
    ext_v    = v;
    ext_n    = n;
}

void drv_ir_barcode_adc(uint16_t *raw, uint16_t *lo, uint16_t *hi)
{
    if (raw != NULL) {
        *raw = adc_now;
    }
    if (lo != NULL) {
        *lo = adc_white;
    }
    if (hi != NULL) {
        *hi = adc_peak;
    }
}

/* ------------------------------------------------------------------ *
 *  Publishes one event per buffered edge, in order.
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
        evt.u.bar_edge.level_high = rec.level ? 1U : 0U;
        evt.u.bar_edge.width_us   = RC_SAT_U(rec.width_us, RC_BITS_WIDTH_US);
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

/* Sets up the two line-sensor GPIO pins and opens the ADC for the barcode
 * sensor's AO (GP26). The barcode DO (GP27) is left alone: not configured,
 * no interrupt. */
rc_result_t drv_ir_init(void)
{
    defer_h = rc_defer_register(barcode_drain, NULL);
    if (defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    /* Line sensors: plain inputs, polled. The comparator output is a
     * push-pull drive on both modules, so no pull is needed. */
    (void)gpio_set_pin(RC_PIN_IR_LINE_L, GPIO_MODE_IN);
    (void)gpio_set_pin(RC_PIN_IR_LINE_R, GPIO_MODE_IN);

    /* Barcode AO. A failed open only loses barcodes, not line following,
     * so say so and carry on. */
    bar_enabled = false;
    adc_dd = tk_opn_dev(ADC_DEVNAME, TD_READ);
    if (adc_dd <= E_OK) {
        tm_printf((UB *)"drv_ir: ADC open failed (%d), no barcodes\n",
                  (int)adc_dd);
        rc_fault_set(RC_FAULT_BARCODE_ADC);
    }

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
        return adc_in_bar;              /* peak / valley detector, see above */
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

/* Polls both line sensors and publishes them as RC_EVT_LINE_SAMPLE: two
 * bits, the whole sample. The coarse position (-500 / 0 / +500) is
 * rc_line_position() on the receiving side. */
void drv_ir_sample_line(void)
{
    rc_event_t evt;

    evt.id = RC_EVT_LINE_SAMPLE;
    evt.u.line.on_line_l  = drv_ir_on_line(RC_IR_LINE_L) ? 1U : 0U;
    evt.u.line.on_line_r  = drv_ir_on_line(RC_IR_LINE_R) ? 1U : 0U;

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

/* Arms or disarms barcode edge reporting. The ADC keeps sampling either
 * way, so the low / high extremes are already learnt when a read starts. */
rc_result_t drv_ir_barcode_enable(bool on)
{
    if (on) {
        bar_enabled = false;
        bar_head    = 0U;
        bar_tail    = 0U;
        acc_valid   = false;
        pend_valid  = false;
    }
    bar_enabled = on;
    return (adc_dd > E_OK) ? RC_OK : RC_ERR_HARDWARE;
}
/*** end of file ***/
