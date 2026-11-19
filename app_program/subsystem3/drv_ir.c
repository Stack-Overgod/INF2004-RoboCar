/*
 *  drv_ir.c  -  IR line sensor pair (GPIO) and analogue barcode sensor (ADC)
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>
#include "drv_ir.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_fault.h"
#include "../rc_defer.h"
#include "../rc_time.h"

/* Line sensor polarity: 1 = the DO pin reads high over the black line. */
#define IR_ACTIVE_HIGH      (1)

/* Barcode sensor, analogue: the AO pin (GP26) goes to ADC channel 0, so
 * no trim pot needs setting. DO (GP27) is not used: no IRQ, no pin setup.
 * Level changes come out as RC_EVT_BARCODE_EDGE for sub_barcode.
 *
 * Sampling (rc_config.h RC_BAR_ADC_FIFO):
 * - FIFO: the ADC converts continuously at RC_BAR_ADC_HZ x
 *   RC_BAR_ADC_OVERSAMPLE. Its FIFO interrupt (every BAR_FIFO_THRESH
 *   readings) averages each RC_BAR_ADC_OVERSAMPLE readings into one value
 *   (the anti-alias filter, rc_config.h), puts it in bar_smp[] and
 *   signals the defer task, where bar_samples_drain() runs the edge
 *   detector. A sustained stream, so no task ever waits for a conversion
 *   and the rate is set by the ADC clock, not the tick.
 * - Device: one reading per Sense tick through the "adca" driver.
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

/* Compiler barrier for the single-producer / single-consumer rings below:
 * the slot data is written before the head index moves, and read after
 * the head index is seen. The M0+ executes in order, so stopping the
 * compiler from reordering the accesses is enough. */
#define RING_BARRIER()      __asm__ volatile ("" ::: "memory")

/* Barcode samples per second */
#if RC_BAR_ADC_FIFO
#define BAR_HZ          (RC_BAR_ADC_HZ)
#else
#define BAR_HZ          (1000U / RC_PERIOD_LINE_MS)
#endif

#if !RC_BAR_ADC_FIFO
/* ADC descriptor, <= 0 if the open failed (barcode then stays silent) */
static ID                adc_dd = -1;
#endif
/* the ADC runs (either mode); false: no barcodes, RC_FAULT_BARCODE_ADC */
static bool              adc_ok;
/* Recent readings (black high) and their times, for timing the edges.
 * At least ~1 s: a bar's falling edge is timed only once the space after
 * it has shown its lowest point, up to BAR_QUIET_N later, and the bar's
 * fall must still be in the history then. */
#if RC_BAR_ADC_FIFO
#define BAR_HIST        (1024U) /* power of 2 */
#else
#define BAR_HIST        (128U)  /* power of 2 */
#endif
#define BAR_HIST_MASK   (BAR_HIST - 1U)
/* A bar's falling edge is normally sent when the next bar starts; after
 * this many samples (300 ms) of space with no deeper valley it is sent
 * anyway, so the last bar of a code is not held until the next one. */
#define BAR_QUIET_N     ((300U * BAR_HZ) / 1000U)
/* The white level creeps up 100 counts a second: one count every this
 * many samples. */
#define BAR_CREEP_EVERY (((BAR_HZ / 100U) > 0U) ? (BAR_HZ / 100U) : 1U)

/* Edge detector state, one task only (defer task in FIFO mode, Sense
 * otherwise) */
static uint16_t          hist_v[BAR_HIST];
static uint32_t          hist_t[BAR_HIST];
static uint32_t          hist_n;        /* sample number of the next reading */
/* white level: running minimum, creeping up (BAR_CREEP_EVERY) */
static uint16_t          adc_white = 0xFFFFU;
static uint32_t          creep_n;
static bool              adc_in_bar;    /* current element is a bar */
static uint16_t          ext_v = 0xFFFFU;   /* extreme of the current element */
static uint32_t          ext_n;
static uint16_t          valley_v;      /* lowest point before the current bar */
static uint32_t          valley_n;
static uint16_t          peak_v;        /* peak of the last accepted bar */
static uint32_t          peak_n;
static bool              fall_due;      /* that bar's falling edge is unsent */

/* optional extra callback for raw barcode edges, set via
 * drv_ir_on_barcode_edge() */
static drv_ir_edge_cb_t bar_cb;
/* context pointer passed back to bar_cb untouched */
static void            *bar_ctx;
/* edges are queued only while armed (drv_ir_barcode_enable) */
static volatile bool     bar_enabled;
/* handle for the "bottom half" deferred-work registration, see rc_defer_register() */
static int32_t           defer_h = -1;

/* Edges are debounced on the LEVEL before the ring: a new level counts
 * once it has lasted BAR_GLITCH_US; if it goes back sooner, the previous
 * change was a spike and is undone. The PROM step already stops
 * chatter, so this only rejects a level shorter than 15 ms (a speck of
 * dirt). Keep ~3x below the narrow element time (3 mm at ~31 mm/s =
 * ~95 ms). */
#define BAR_GLITCH_US   (15000U)

/* Debounce state (the producer task only: the defer task in FIFO mode,
 * Sense otherwise). An element is held back (pend_*) until the
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
static volatile uint32_t bar_head;   /* next slot bar_level_in() writes */
/* next slot the bottom half will read from */
static volatile uint32_t bar_tail;
/* count of edges dropped because the ring filled up before being drained */
static volatile uint32_t bar_overrun;

/* Re-arming resets the ring without touching another task's index: the
 * enable call sets bar_reset_req; the producer clears its debounce state
 * and marks where the ring stood (bar_skip_to, then bar_skip_req); the
 * consumer moves its tail there before publishing anything. */
static volatile bool     bar_reset_req;
static volatile bool     bar_skip_req;
static volatile uint32_t bar_skip_to;

/* ------------------------------------------------------------------ *
 *  Barcode level change at t_us, called by bar_sample() on the producer
 *  task. level true = black (bar).
 * ------------------------------------------------------------------ */
static void bar_level_in(bool level, uint32_t t_us)
{
    uint32_t width;
    uint32_t next;
    bool     have;
    bool     out_level;

    if (bar_reset_req) {
        acc_valid     = false;
        pend_valid    = false;
        bar_skip_to   = bar_head;
        bar_skip_req  = true;
        bar_reset_req = false;
    }
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
    have       = pend_valid;
    width      = pend_w;
    out_level  = pend_level;
    pend_w     = t_us - acc_t;
    pend_level = level;
    pend_valid = true;
    acc_prev_t = acc_t;
    acc_t      = t_us;
    acc_level  = level;
    if (!have) {
        return;
    }

    /* Ring full (the next slot is the reader's): drop this edge. */
    next = (bar_head + 1U) & BAR_RING_MASK;
    if (next == bar_tail) {
        bar_overrun++;      /* decoder is not keeping up, count it */
        return;
    }

    bar_ring[bar_head].width_us = width;
    bar_ring[bar_head].level    = out_level;
    RING_BARRIER();
    bar_head = next;

    /* Hand off to barcode_drain(), on the defer task, so the producer
     * never waits on the event bus. */
    rc_defer_signal_i(defer_h);
}

/* ------------------------------------------------------------------ *
 *  Barcode ADC sampling (see the top of the file for the two modes)
 * ------------------------------------------------------------------ */

#if !RC_BAR_ADC_FIFO
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
#endif

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

/* One reading v (0..4095) taken at t_us through the edge detector: the
 * peak / valley rules at the top of the file. Called for every sample, in
 * time order, from one task only (Sense or defer, by mode). */
static void bar_sample(uint16_t v, uint32_t t)
{
    uint32_t n;

    if (!RC_BAR_ADC_BLACK_HIGH) {
        v = (uint16_t)(0x0FFFU - v);
    }

    n = hist_n;
    hist_v[n & BAR_HIST_MASK] = v;
    hist_t[n & BAR_HIST_MASK] = t;
    hist_n++;

    if (v < adc_white) {
        adc_white = v;
        creep_n   = 0U;
    } else if ((adc_white < 0x0FFFU) && (++creep_n >= BAR_CREEP_EVERY)) {
        adc_white++;
        creep_n = 0U;
    } else {
        /* creeping, not yet a whole count */
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
        if (v < valley_v) {
            ext_v = v;
            ext_n = n;
        } else {
            ext_v = valley_v;
            ext_n = valley_n;
        }
        return;
    }
    /* the bar began halfway up from the valley before it */
    bar_level_in(true, bar_cross_t(valley_n, ext_n,
                                   ((uint32_t)valley_v + ext_v) / 2U));
    peak_v   = ext_v;
    peak_n   = ext_n;
    fall_due = true;
    ext_v    = v;
    ext_n    = n;
}

#if RC_BAR_ADC_FIFO
/* RP2040 ADC registers (datasheet 4.9.6). The kernel's ADC driver powers
 * the ADC and makes GP26 analogue at boot; from then on this file owns it. */
#define ADC_REG(off)        (0x4004C000UL + (off))
#define ADC_CS              ADC_REG(0x00U)
#define ADC_FCS             ADC_REG(0x08U)
#define ADC_FIFO            ADC_REG(0x0CU)
#define ADC_DIV             ADC_REG(0x10U)
#define ADC_INTE            ADC_REG(0x18U)
#define ADC_CS_EN           (1UL << 0)
#define ADC_CS_START_MANY   (1UL << 3)
#define ADC_CS_READY        (1UL << 8)
#define ADC_CS_AINSEL_POS   (12U)
#define ADC_FCS_EN          (1UL << 0)
#define ADC_FCS_UNDER       (1UL << 10)
#define ADC_FCS_OVER        (1UL << 11)
#define ADC_FCS_LEVEL_POS   (16U)
#define ADC_FCS_THRESH_POS  (24U)
#define ADC_CLK_HZ          (48000000UL)    /* clk_adc, from the USB PLL */
#define INTNO_ADC_FIFO      (22U)           /* ADC_IRQ_FIFO */
/* same level as PendSV: the handler calls kernel services, so it must not
 * preempt a task switch */
#define INTPRI_ADC_FIFO     (3)

/* The FIFO (8 deep) interrupts once this many readings are waiting: half
 * full, so the handler has 4 readings' time (500 us at 8 kHz) before one
 * would be dropped. */
#define BAR_FIFO_THRESH     (4U)
/* ADC conversion rate */
#define BAR_ADC_RATE_HZ     (BAR_HZ * RC_BAR_ADC_OVERSAMPLE)
#define ADC_FCS_CFG         (ADC_FCS_EN | (BAR_FIFO_THRESH << ADC_FCS_THRESH_POS))
#define ADC_CS_CFG          (ADC_CS_EN | (RC_ADC_CH_IR_BARCODE << ADC_CS_AINSEL_POS))

/* Averaged values from the interrupt to the defer task: 256 ms at 1 kHz. */
#define BAR_SMP_SZ          (256U)  /* power of 2 */
#define BAR_SMP_MASK        (BAR_SMP_SZ - 1U)
/* Time between readings; the readings themselves are the clock. */
#define BAR_PERIOD_US       (1000000UL / BAR_HZ)

static uint16_t          bar_smp[BAR_SMP_SZ];
static volatile uint32_t smp_head;      /* next slot the ISR writes */
static volatile uint32_t smp_tail;      /* next slot the defer task reads */
static volatile uint32_t smp_overrun;   /* values lost (ring or FIFO full) */
static uint32_t          os_sum;        /* ISR: readings summed so far */
static uint32_t          os_n;          /* ISR: how many */
static uint32_t          smp_count;     /* readings processed (defer task) */
static int32_t           smp_defer_h = -1;

/* ADC FIFO interrupt: add the waiting readings to the running sum; every
 * RC_BAR_ADC_OVERSAMPLE readings, put their average into bar_smp[] and
 * wake the defer task. A couple of microseconds; the detector itself runs
 * in bar_samples_drain(). */
static void adc_fifo_isr(UINT intno)
{
    uint32_t fcs   = in_w(ADC_FCS);
    uint32_t n     = (fcs >> ADC_FCS_LEVEL_POS) & 0xFU;
    bool     added = false;

    (void)intno;
    if ((fcs & ADC_FCS_OVER) != 0U) {
        out_w(ADC_FCS, ADC_FCS_CFG | ADC_FCS_OVER);     /* write 1 clears */
        smp_overrun++;
    }
    while (n > 0U) {
        uint32_t next;

        n--;
        os_sum += in_w(ADC_FIFO) & 0x0FFFU;
        if (++os_n < RC_BAR_ADC_OVERSAMPLE) {
            continue;
        }
        next = (smp_head + 1U) & BAR_SMP_MASK;
        if (next == smp_tail) {
            smp_overrun++;              /* defer task behind: drop it */
        } else {
            bar_smp[smp_head] = (uint16_t)(os_sum / RC_BAR_ADC_OVERSAMPLE);
            RING_BARRIER();
            smp_head = next;
            added    = true;
        }
        os_sum = 0U;
        os_n   = 0U;
    }
    if (added) {
        rc_defer_signal_i(smp_defer_h);     /* only when there is work */
    }
    EndOfInt(INTNO_ADC_FIFO);
}

/* Defer task: every waiting reading through the edge detector, stamped
 * from its position in the stream (BAR_PERIOD_US apart). */
static void bar_samples_drain(void *ctx)
{
    (void)ctx;

    while (smp_tail != smp_head) {
        uint16_t v;

        RING_BARRIER();
        v = bar_smp[smp_tail];

        smp_tail = (smp_tail + 1U) & BAR_SMP_MASK;
        bar_sample(v, smp_count * BAR_PERIOD_US);
        smp_count++;
    }
}

/* Starts the ADC converting channel RC_ADC_CH_IR_BARCODE at
 * BAR_ADC_RATE_HZ into its FIFO, with the FIFO interrupt on. */
static rc_result_t adc_fifo_start(void)
{
    T_DINT   dint;
    uint32_t spin;

    smp_defer_h = rc_defer_register(bar_samples_drain, NULL);
    if (smp_defer_h < 0) {
        return RC_ERR_NOSPACE;
    }

    DisableInt(INTNO_ADC_FIFO);
    out_w(ADC_INTE, 0U);
    out_w(ADC_CS, ADC_CS_CFG);
    for (spin = 0U; (in_w(ADC_CS) & ADC_CS_READY) == 0U; spin++) {
        if (spin > 100000U) {
            return RC_ERR_HARDWARE;     /* never ready: ADC not powered */
        }
    }
    /* one conversion every 1 + INT cycles of clk_adc */
    out_w(ADC_DIV, ((ADC_CLK_HZ / BAR_ADC_RATE_HZ) - 1UL) << 8);
    out_w(ADC_FCS, ADC_FCS_CFG | ADC_FCS_OVER | ADC_FCS_UNDER);
    while (((in_w(ADC_FCS) >> ADC_FCS_LEVEL_POS) & 0xFU) != 0U) {
        (void)in_w(ADC_FIFO);           /* empty it */
    }
    os_sum = 0U;
    os_n   = 0U;

    dint.intatr = TA_HLNG;
    dint.inthdr = adc_fifo_isr;
    if (tk_def_int(INTNO_ADC_FIFO, &dint) != E_OK) {
        return RC_ERR_HARDWARE;
    }
    ClearInt(INTNO_ADC_FIFO);
    out_w(ADC_INTE, 1U);                /* FIFO level >= THRESH */
    EnableInt(INTNO_ADC_FIFO, INTPRI_ADC_FIFO);
    out_w(ADC_CS, ADC_CS_CFG | ADC_CS_START_MANY);
    return RC_OK;
}

/* Nothing to do per Sense tick: the readings arrive by FIFO interrupt. */
void drv_ir_sample_barcode(void)
{
}
#else
void drv_ir_sample_barcode(void)
{
    uint32_t t = rc_time_us();
    uint16_t v = adc_read();

    if (v == 0xFFFFU) {
        return;
    }
    bar_sample(v, t);
}
#endif

/* ------------------------------------------------------------------ *
 *  Defer task: skips edges from before a re-arm, then publishes one
 *  RC_EVT_BARCODE_EDGE per buffered edge, in order.
 * ------------------------------------------------------------------ */
static void barcode_drain(void *ctx)
{
    rc_event_t     evt;
    bar_edge_rec_t rec;

    (void)ctx;

    if (bar_skip_req) {
        bar_tail     = bar_skip_to;     /* edges from before the re-arm */
        bar_skip_req = false;
    }
    while (bar_tail != bar_head) {
        RING_BARRIER();
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
#if RC_BAR_ADC_FIFO
    return bar_overrun + smp_overrun;
#else
    return bar_overrun;
#endif
}

/* ------------------------------------------------------------------ *
 *  Init
 * ------------------------------------------------------------------ */

/* Sets up the two line-sensor GPIO pins and starts the ADC for the barcode
 * sensor's AO (GP26): the FIFO stream, or the "adca" driver. The barcode
 * DO (GP27) is left alone: not configured, no interrupt. */
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

    /* Barcode AO. A failed start only loses barcodes, not line following,
     * so say so and carry on. */
    bar_enabled = false;
#if RC_BAR_ADC_FIFO
    if (adc_fifo_start() == RC_OK) {
        adc_ok = true;
    } else {
        tm_printf((UB *)"drv_ir: ADC FIFO start failed, no barcodes\n");
        rc_fault_set(RC_FAULT_BARCODE_ADC);
    }
#else
    adc_dd = tk_opn_dev(ADC_DEVNAME, TD_READ);
    if (adc_dd > E_OK) {
        adc_ok = true;
    } else {
        tm_printf((UB *)"drv_ir: ADC open failed (%d), no barcodes\n",
                  (int)adc_dd);
        rc_fault_set(RC_FAULT_BARCODE_ADC);
    }
#endif

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

/* Registers an extra callback, called on the defer task (barcode_drain())
 * for every raw barcode edge. NULL removes it. */
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
        bar_reset_req = true;           /* done by the ring's own tasks */
    }
    bar_enabled = on;
    return adc_ok ? RC_OK : RC_ERR_HARDWARE;
}
/*** end of file ***/
