/*
 *  sub_barcode.c
 */
#include "../rc_prelude.h"
#include "sub_barcode.h"
#include "drv_ir.h"
#include "sub_line.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* How many bar/space widths make up one character
 * Used as the size of the sliding "pattern" we try to classify. */
#define ELEMENTS        (9U)
/* How many widths we keep buffered at once.*/
#define WINDOW          (10U)

/* Flicker on the bar edges (DOUT chatter, under 10 ms) is merged back
 * into its element by the ISR (drv_ir.c BAR_GLITCH_US), so every width
 * that reaches on_edge() is a real bar or space.
 *
 * Noise filter: fewer than NOISE_EDGES edges within NOISE_WINDOW_US of
 * each other are a speck, not a barcode: nothing is reported and sub_line
 * is not told a read is on. A gap longer than NOISE_WINDOW_US ends the
 * attempt, so bars and spaces up to 2 s long (slow passes) stay in one
 * attempt. Log widths are 10 us uint16 and stop at 655 ms (to10us). */
#define NOISE_EDGES     (3U)
#define NOISE_WINDOW_US (2000000UL)

/* Why classify() rejected a window */
typedef enum {
    CLS_OK = 0,
    CLS_NO_RATIO,       /* wide and narrow too alike: not a character */
    CLS_SPACE_FIRST     /* window starts on a space: never a character */
} cls_result_t;

/* barcode table entry */
typedef struct {
    char     symbol;
    /* bit 0 holds the LAST. 1 = wide, 0 = narrow*/
    uint16_t pattern;   /* bit 8 is the first element, 1 means wide */
} code39_t;

/*
 *  Standard Code 39 patterns: the start/stop '*' and all 26 letters
 *  (2026-10-29: the demo-day barcodes can be any letter; symbol_to_cmd()
 *  picks the four that are commands).
 */
static const code39_t table[] = {
    { '*', 0x094U },    /* bars 00110, spaces 1000 -> 0 1001 0100 */
    { 'A', 0x109U },    /* bars 10001, spaces 0010 -> 1 0000 1001 */
    { 'B', 0x049U },    /* bars 01001, spaces 0010 -> 0 0100 1001 */
    { 'C', 0x148U },    /* bars 11000, spaces 0010 -> 1 0100 1000 */
    { 'D', 0x019U },    /* bars 00101, spaces 0010 -> 0 0001 1001 */
    { 'E', 0x118U },    /* bars 10100, spaces 0010 -> 1 0001 1000 */
    { 'F', 0x058U },    /* bars 01100, spaces 0010 -> 0 0101 1000 */
    { 'G', 0x00DU },    /* bars 00011, spaces 0010 -> 0 0000 1101 */
    { 'H', 0x10CU },    /* bars 10010, spaces 0010 -> 1 0000 1100 */
    { 'I', 0x04CU },    /* bars 01010, spaces 0010 -> 0 0100 1100 */
    { 'J', 0x01CU },    /* bars 00110, spaces 0010 -> 0 0001 1100 */
    { 'K', 0x103U },    /* bars 10001, spaces 0001 -> 1 0000 0011 */
    { 'L', 0x043U },    /* bars 01001, spaces 0001 -> 0 0100 0011 */
    { 'M', 0x142U },    /* bars 11000, spaces 0001 -> 1 0100 0010 */
    { 'N', 0x013U },    /* bars 00101, spaces 0001 -> 0 0001 0011 */
    { 'O', 0x112U },    /* bars 10100, spaces 0001 -> 1 0001 0010 */
    { 'P', 0x052U },    /* bars 01100, spaces 0001 -> 0 0101 0010 */
    { 'Q', 0x007U },    /* bars 00011, spaces 0001 -> 0 0000 0111 */
    { 'R', 0x106U },    /* bars 10010, spaces 0001 -> 1 0000 0110 */
    { 'S', 0x046U },    /* bars 01010, spaces 0001 -> 0 0100 0110 */
    { 'T', 0x016U },    /* bars 00110, spaces 0001 -> 0 0001 0110 */
    { 'U', 0x181U },    /* bars 10001, spaces 1000 -> 1 1000 0001 */
    { 'V', 0x0C1U },    /* bars 01001, spaces 1000 -> 0 1100 0001 */
    { 'W', 0x1C0U },    /* bars 11000, spaces 1000 -> 1 1100 0000 */
    { 'X', 0x091U },    /* bars 00101, spaces 1000 -> 0 1001 0001 */
    { 'Y', 0x190U },    /* bars 10100, spaces 1000 -> 1 1001 0000 */
    { 'Z', 0x0D0U }     /* bars 01100, spaces 1000 -> 0 1101 0000 */
};

/* '*' read backwards: the same pattern as 'P' read forwards */
#define STAR_REVERSED   (0x052U)

/* Number of entries in `table`, computed at compile time */
#define TABLE_N     (sizeof(table) / sizeof(table[0]))

/* sliding buffer of the most recent bar/space widths, in microseconds */
static uint32_t widths[WINDOW];
/* widths[i] was a bar (black), not a space */
static bool     is_bar[WINDOW];
/* how many entries in `widths` are currently valid */
static uint32_t n_widths;
/* decoding enabled? (see sub_barcode_arm) */
static bool     armed;
/* most recently decoded character, for telemetry/logging */
static char     last_symbol;
/* a "function pointer" — the caller-supplied function to invoke on a successful decode */
static sub_barcode_cb_t user_cb;
/* opaque context pointer passed back to user_cb untouched, so the caller can
 * tell which car/instance called it */
static void    *user_ctx;

/* the scan attempt being collected (see NOISE_WINDOW_US) */
static bool             scan_open;
static rc_pl_bar_scan_t scan;
/* bus timestamp of the attempt's latest edge */
static uint32_t         scan_last_us;
/* edges in the attempt, unsaturated, for NOISE_EDGES */
static uint32_t         scan_edges;
/* reading direction of the current code, fixed by its first '*' (see
 * lookup): 0 = not known yet, 1 = forwards, -1 = backwards. Cleared by the
 * closing '*', so the next code may be read either way round. */
static int8_t           scan_dir;
/* a letter has been decoded since the last '*' */
static bool             frame_letter;

/* every element of the attempt, 10 us units, for car/01/bcbits */
static uint16_t         raw_w[RC_BAR_RAW_MAX];
static uint32_t         raw_n;
static uint32_t         raw_more;
static bool             raw_first_bar;

/* ------------------------------------------------------------------ *
 *  Map a decoded character to a navigation command.
 * ------------------------------------------------------------------ */

static rc_nav_cmd_t symbol_to_cmd(char c)
{
    switch (c) {
    case 'A':
        return RC_CMD_TURN_LEFT;
    case 'B':
        return RC_CMD_TURN_RIGHT;
    case 'C':
        return RC_CMD_GO_STRAIGHT;
    case 'D':
        return RC_CMD_U_TURN;
    default:            /* the other 22 letters and '*' decode but do nothing */
        return RC_CMD_NONE;
    }
}

/* ------------------------------------------------------------------ *
 *  Try to decode the oldest nine elements in the window.
 * ------------------------------------------------------------------ */

/* Wide if at least WIDE_RATIO / 10 times the widest narrow one of the same
 * kind (bar or space). Code 39 is printed 3:1. */
#define WIDE_RATIO      (15U)

/* Index (from `first`, step 2) of the widest element not in `skip` (a bit
 * mask of indexes). */
static uint32_t widest(uint32_t first, uint32_t skip)
{
    uint32_t best = first;
    uint32_t i;

    while ((skip & (1U << best)) != 0U) {
        best += 2U;
    }
    for (i = first; i < ELEMENTS; i += 2U) {
        if (((skip & (1U << i)) == 0U) && (widths[i] > widths[best])) {
            best = i;
        }
    }
    return best;
}

/* A Code 39 character is 5 bars (2 wide) and 4 spaces (1 wide), starting
 * and ending on a bar. Bars and spaces are ranked SEPARATELY: the sensor
 * spot makes black read wider and white narrower (2026-10-26 21:55: one
 * midpoint for all 9 turned a long narrow bar wide in 2 of 3 characters).
 * The 2 widest bars and the widest space are wide, if they stand out by
 * WIDE_RATIO from the rest of their kind. */
static cls_result_t classify(uint16_t *pattern_out)
{
    uint32_t b1, b2, b3, s1, s2;
    uint32_t wide;
    uint16_t pattern = 0U;
    uint32_t i;

    if (!is_bar[0]) {
        return CLS_SPACE_FIRST;
    }
    b1 = widest(0U, 0U);
    b2 = widest(0U, (1U << b1));
    b3 = widest(0U, (1U << b1) | (1U << b2));   /* widest narrow bar */
    s1 = widest(1U, 0U);
    s2 = widest(1U, (1U << s1));                /* widest narrow space */

    if (((widths[b2] * 10U) < (widths[b3] * WIDE_RATIO))
        || ((widths[s1] * 10U) < (widths[s2] * WIDE_RATIO))) {
        return CLS_NO_RATIO;
    }

    wide = (1U << b1) | (1U << b2) | (1U << s1);
    for (i = 0U; i < ELEMENTS; i++) {
        pattern <<= 1;
        if ((wide & (1U << i)) != 0U) {
            pattern |= 1U;          /* bit 8 is the first element */
        }
    }
    *pattern_out = pattern;
    return CLS_OK;
}

/* Searches `table` for exactly `pattern`. */
static bool find(uint16_t pattern, char *out)
{
    uint32_t i;

    for (i = 0U; i < TABLE_N; i++) {
        if (table[i].pattern == pattern) {
            *out = table[i].symbol;
            return true;
        }
    }
    return false;
}

/* Decodes `pattern` read forwards or backwards. With all 26 letters in the
 * table, 6 letter pairs are each other's mirror image (K/U, N/Y, O/X, Q/W,
 * R/V, T/Z) and '*' backwards is 'P', so the direction must be known. A
 * barcode starts with '*': the attempt's first '*', either way round,
 * fixes the direction for the rest of the attempt. Before that, forwards
 * is tried first, as before. */
static bool lookup(uint16_t pattern, char *out, bool *reversed)
{
    uint16_t rev = 0U;
    uint32_t b;

    for (b = 0U; b < ELEMENTS; b++) {
        rev <<= 1;
        if ((pattern & (1U << b)) != 0U) {
            rev |= 1U;
        }
    }

    if (scan_dir == 0) {
        /* no '*' yet: a 'P' pattern is far more likely a backwards '*' */
        if ((pattern == STAR_REVERSED) || (rev == STAR_REVERSED)) {
            scan_dir  = (pattern == STAR_REVERSED) ? (int8_t)-1 : (int8_t)1;
            *out      = '*';
            *reversed = (scan_dir < 0);
            return true;
        }
    }
    if ((scan_dir >= 0) && find(pattern, out)) {
        *reversed = false;
        return true;
    }
    if ((scan_dir <= 0) && find(rev, out)) {
        *reversed = true;
        return true;
    }
    return false;
}

/* Drops the oldest width and shifts everything else down by one slot. */
static void shift_window(void)
{
    uint32_t i;

    for (i = 1U; i < n_widths; i++) {
        widths[i - 1U] = widths[i];
        is_bar[i - 1U] = is_bar[i];
    }
    n_widths--;
}

/* ------------------------------------------------------------------ *
 *  Scan attempt log. Same FAST dispatcher as on_edge(), so no locking.
 * ------------------------------------------------------------------ */

/* Width in the 10 us units of the log fields, stopping at 655 ms. */
static uint16_t to10us(uint32_t w)
{
    return (w >= 655350UL) ? 0xFFFFU : (uint16_t)(w / 10U);
}

/* Adds one to a log counter, stopping at 255 instead of wrapping. */
static uint8_t sat_inc(uint8_t v)
{
    return (v < 0xFFU) ? (uint8_t)(v + 1U) : v;
}

/* Records one element of the attempt; `bar` = it was black. */
static void raw_add(uint32_t w, bool bar)
{
    if (raw_n == 0U) {
        raw_first_bar = bar;
    }
    if (raw_n < RC_BAR_RAW_MAX) {
        raw_w[raw_n] = to10us(w);
        raw_n++;
    } else {
        raw_more++;
    }
}

/* Publishes the attempt's elements as wide (1) / narrow (0) bits. Each
 * element is compared with the midpoint of the widest and narrowest of
 * the 9 around it, like classify() does for its window; where those 9
 * have no 2:1 contrast it counts as narrow. */
static void raw_publish(void)
{
    rc_event_t out = { 0 };
    uint32_t   nar_sum = 0U, nar_n = 0U, wid_sum = 0U, wid_n = 0U;
    uint32_t   i, j, lo, hi, mn, mx;

    out.id = RC_EVT_BARCODE_RAW;
    out.u.bar_raw.n         = (uint8_t)raw_n;
    out.u.bar_raw.first_bar = raw_first_bar ? 1U : 0U;
    out.u.bar_raw.more      = (raw_more > 0xFFU) ? 0xFFU : (uint8_t)raw_more;

    for (i = 0U; i < raw_n; i++) {
        lo = (i >= 4U) ? (i - 4U) : 0U;
        hi = ((i + 4U) < raw_n) ? (i + 4U) : (raw_n - 1U);
        mn = 0xFFFFU;
        mx = 0U;
        for (j = lo; j <= hi; j++) {
            if (raw_w[j] < mn) {
                mn = raw_w[j];
            }
            if (raw_w[j] > mx) {
                mx = raw_w[j];
            }
        }
        if ((mx >= (mn * 2U)) && (raw_w[i] > ((mn + mx) / 2U))) {
            out.u.bar_raw.bits[i / 8U] |= (uint8_t)(1U << (i % 8U));
            wid_sum += raw_w[i];
            wid_n++;
        } else {
            nar_sum += raw_w[i];
            nar_n++;
        }
    }
    out.u.bar_raw.nar_10us = (uint16_t)((nar_n != 0U) ? (nar_sum / nar_n) : 0U);
    out.u.bar_raw.wid_10us = (uint16_t)((wid_n != 0U) ? (wid_sum / wid_n) : 0U);
    (void)rc_event_publish(&out);
}


/* Publishes the open attempt (if it is big enough) and closes it. */
static void scan_finish(void)
{
    rc_event_t out;

    if (scan_open && (scan_edges >= NOISE_EDGES)) {
        raw_publish();
        out.id         = RC_EVT_BARCODE_SCAN;
        out.u.bar_scan = scan;
        (void)rc_event_publish(&out);
    }
    scan_open = false;
    (void)sub_line_set_reading(false);
}

/* Counts one edge at bus time t_us, opening a new attempt if none is. */
static void scan_edge(uint32_t t_us)
{
    if (!scan_open) {
        scan.text[0]      = '\0';
        scan.edges        = 0U;
        scan.ok           = 0U;
        scan.f_ratio      = 0U;
        scan.f_wide       = 0U;
        scan.f_nomatch    = 0U;
        scan.f_glitch     = 0U;
        scan.last_nomatch = 0U;
        scan.min_10us     = 0xFFFFU;
        scan.max_10us     = 0U;
        scan_edges        = 0U;
        scan_dir          = 0;
        frame_letter      = false;
        raw_n             = 0U;
        raw_more          = 0U;
        scan_open         = true;
    }
    scan.edges   = sat_inc(scan.edges);
    scan_edges++;
    scan_last_us = t_us;
    if (scan_edges == NOISE_EDGES) {
        (void)sub_line_set_reading(true);   /* a barcode, not a speck */
    }
}

/* Widest / narrowest element of the attempt, in 10 us units. */
static void scan_width(uint32_t w)
{
    uint16_t w10 = to10us(w);

    if (w10 < scan.min_10us) {
        scan.min_10us = w10;
    }
    if (w10 > scan.max_10us) {
        scan.max_10us = w10;
    }
}

/* Appends a decoded symbol to the attempt's text (first 3 kept). */
static void scan_symbol(char symbol)
{
    uint32_t len = (uint32_t)scan.ok;

    if (len < (sizeof(scan.text) - 1U)) {
        scan.text[len]      = symbol;
        scan.text[len + 1U] = '\0';
    }
    scan.ok = sat_inc(scan.ok);
}

/* Line samples arrive every RC_PERIOD_LINE_MS whatever the barcode is
 * doing, so they serve as the clock that ends a quiet attempt. Signed
 * difference: a sample stamped just before the latest edge (two
 * publishers racing) must read as "not quiet", not wrap to huge. */
static void on_line_tick(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (scan_open
        && ((int32_t)(evt->t_us - scan_last_us) > (int32_t)NOISE_WINDOW_US)) {
        scan_finish();
    }
}

/* ------------------------------------------------------------------ *
 *  Edge callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

static void on_edge(const rc_event_t *evt, void *ctx)
{
    uint32_t     w = evt->u.bar_edge.width_us;
    uint16_t     pattern;
    char         symbol;
    bool         reversed;
    cls_result_t cls;
    rc_event_t   out;

    (void)ctx;

    if (!armed) {
        return;
    }

    if (w > NOISE_WINDOW_US) {
        /* a quiet gap: the previous attempt is over, this edge starts
         * the next one */
        scan_finish();
        scan_edge(evt->t_us);
        n_widths = 0U;
        return;
    }
    scan_edge(evt->t_us);
    scan_width(w);

    /* the element that just ended is the level before this edge; the
     * sensor reads high over black (drv_ir IR_ACTIVE_HIGH) */
    raw_add(w, !evt->u.bar_edge.level_high);

    /* Append the new width to the buffer, making room by sliding out the
     * oldest one first if we're already full. */
    if (n_widths >= WINDOW) {
        shift_window();
    }
    widths[n_widths] = w;
    is_bar[n_widths] = !evt->u.bar_edge.level_high;
    n_widths++;

    /* Not enough elements buffered yet to attempt a decode — wait for
     * more edges to arrive. */
    if (n_widths < ELEMENTS) {
        return;
    }

    cls = classify(&pattern);
    if (cls != CLS_OK) {
        if (cls == CLS_NO_RATIO) {
            scan.f_ratio = sat_inc(scan.f_ratio);
        }
        shift_window();             /* slide by one and try again */
        return;
    }

    if (!lookup(pattern, &symbol, &reversed)) {
        scan.f_nomatch    = sat_inc(scan.f_nomatch);
        scan.last_nomatch = pattern;
        shift_window();
        return;
    }

    /* Successful decode: reset the window */
    n_widths    = 0U;
    last_symbol = symbol;
    scan_symbol(symbol);
    if (symbol != '*') {
        frame_letter = true;
    } else if (frame_letter) {
        scan_dir     = 0;           /* closing '*': this code is done */
        frame_letter = false;
    } else {
        /* opening '*': lookup() has set scan_dir */
    }

    out.id = RC_EVT_BARCODE_DECODED;
    out.u.barcode.symbol   = symbol;
    out.u.barcode.command  = symbol_to_cmd(symbol);
    out.u.barcode.reversed = reversed;
    (void)rc_event_publish(&out);

    /* user_cb is a function pointer , might be NULL if nobody registered a
     *  callback via sub_barcode_on_decode(), check before calling through it */
    if (user_cb != NULL) {
        user_cb(symbol, symbol_to_cmd(symbol), user_ctx);
    }
}

/* ------------------------------------------------------------------ */

/* Call once at boot. Clears the width buffer, starts disarmed and subscribes 
* on_edge() to fire for every RC_EVT_BARCODE_EDGE event published by drv_ir.c's barcode ISR. */
rc_result_t sub_barcode_init(void)
{
    {
        uint32_t i;
        for (i = 0U; i < WINDOW; i++) {
            widths[i] = 0U;
        }
    }
    n_widths    = 0U;
    armed       = false;
    /* '\0' is the "nul" character: C's usual way to say "no value here" */
    last_symbol = '\0';
    scan_open   = false;

    if ((rc_event_subscribe(RC_EVT_BARCODE_EDGE, RC_LANE_FAST,
                            on_edge, NULL) < 0)
            || (rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_FAST,
                               on_line_tick, NULL) < 0)) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

/* Turns decoding on/off. Disarming also ends (and reports) any attempt in
 * progress, e.g. a barcode cut short by an obstacle stop. */
rc_result_t sub_barcode_arm(bool on)
{
    if (!on) {
        scan_finish();
    }
    armed    = on;
    n_widths = 0U;
    return drv_ir_barcode_enable(on);
}

/* Registers a function to call every time a character is successfully decoded. */
rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/* Returns the most recently decoded character, mainly for telemetry
 * to report on what the car has read so far. */
char sub_barcode_last(void)
{
    return last_symbol;
}
/*** end of file ***/
