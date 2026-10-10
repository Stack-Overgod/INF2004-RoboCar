/*
 *  sub_barcode.c
 */
#include "../rc_prelude.h"
#include "sub_barcode.h"
#include "drv_ir.h"
#include "../rc_config.h"
#include "../rc_event.h"

/* How many bar/space widths make up one character (see the block comment
 * above). Used as the size of the sliding "pattern" we try to classify. */
#define ELEMENTS        (9U)
/* How many widths we keep buffered at once.*/
#define WINDOW          (10U)

/* Reject anything implausible */
#define ELEM_MIN_US     (800UL)
#define ELEM_MAX_US     (200000UL)

/* barcode table entry */
typedef struct {
    char     symbol;
    /* bit 0 holds the LAST. 1 = wide, 0 = narrow*/
    uint16_t pattern;   /* bit 8 is the first element, 1 means wide */
} code39_t;

/*
 *  Standard Code 39 patterns,
 */
static const code39_t table[] = {
    { '*', 0x094U },    /* bars 00110, spaces 1000 -> 0 1001 0100 */
    { 'A', 0x109U },    /* bars 10001, spaces 0010 -> 1 0000 1001 */
    { 'B', 0x049U },    /* bars 01001, spaces 0010 -> 0 0100 1001 */
    { 'C', 0x148U },    /* bars 11000, spaces 0010 -> 1 0100 1000 */
    { 'D', 0x019U },    /* bars 00101, spaces 0010 -> 0 0001 1001 */
    { 'Z', 0x0D0U }     /* bars 01100, spaces 1000 -> 0 1101 0000 */
};

/* Number of entries in `table`, computed at compile time */
#define TABLE_N     (sizeof(table) / sizeof(table[0]))

/* sliding buffer of the most recent bar/space widths, in microseconds */
static uint32_t widths[WINDOW];
/* how many entries in `widths` are currently valid */
static uint32_t n_widths;
/* decoding enabled? (see sub_barcode_arm) */
static bool     armed;
/* most recently decoded character, for telemetry/logging */
static char     last_symbol;
/* a "function pointer" — the caller-supplied function to invoke on a successful
 * decode */
static sub_barcode_cb_t user_cb;
/* opaque context pointer passed back to user_cb untouched, so the caller can
 * tell which car/instance called it */
static void    *user_ctx;

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
    default:
        return RC_CMD_NONE;
    }
}

/* ------------------------------------------------------------------ *
 *  Try to decode the oldest nine elements in the window.
 * ------------------------------------------------------------------ */

static bool classify(uint16_t *pattern_out)
{
    /* start at the largest possible value so the first real width always beats
     * it */
    uint32_t min_w = 0xFFFFFFFFUL;
    /* start at zero so the first real width always beats it */
    uint32_t max_w = 0U;
    uint32_t mid;
    uint16_t pattern = 0U;
    uint32_t wide_count = 0U;
    uint32_t i;

    for (i = 0U; i < ELEMENTS; i++) {
        if (widths[i] < min_w) {
            min_w = widths[i];
        }
        if (widths[i] > max_w) {
            max_w = widths[i];
        }
    }

    /* A valid Code 39 character has a wide:narrow ratio of at least 2.
     * If the spread is smaller than that we are not looking at one. */
    if (max_w < (min_w * 2U)) {
        return false;
    }

    mid = (min_w + max_w) / 2U;

    for (i = 0U; i < ELEMENTS; i++) {
        pattern <<= 1;
        if (widths[i] > mid) {
            /* sets just the lowest bit to 1 */
            pattern |= 1U;
            wide_count++;
        }
    }

    /* Exactly three of the nine elements are wide in every Code 39
     * character. */
    if (wide_count != 3U) {
        return false;
    }

    *pattern_out = pattern;
    return true;
}

/* Searches `table` for a matching pattern. */
static bool lookup(uint16_t pattern, char *out, bool *reversed)
{
    uint32_t i;
    uint16_t rev = 0U;
    uint32_t b;

    for (i = 0U; i < TABLE_N; i++) {
        if (table[i].pattern == pattern) {
            *out      = table[i].symbol;
            *reversed = false;
            return true;
        }
    }

    for (b = 0U; b < ELEMENTS; b++) {
        rev <<= 1;
        if ((pattern & (1U << b)) != 0U) {
            rev |= 1U;
        }
    }

    for (i = 0U; i < TABLE_N; i++) {
        if (table[i].pattern == rev) {
            *out      = table[i].symbol;
            *reversed = true;
            return true;
        }
    }
    return false;
}

/* Drops the oldest width and shifts everything else down by one slot. */
static void shift_window(void)
{
    uint32_t i;

    for (i = 1U; i < n_widths; i++) {
        widths[i - 1U] = widths[i];
    }
    n_widths--;
}

/* ------------------------------------------------------------------ *
 *  Edge callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

static void on_edge(const rc_event_t *evt, void *ctx)
{
    uint32_t   w = evt->u.bar_edge.width_us;
    uint16_t   pattern;
    char       symbol;
    bool       reversed;
    rc_event_t out;

    (void)ctx;

    if (!armed) {
        return;
    }

    if ((w < ELEM_MIN_US) || (w > ELEM_MAX_US)) {
        n_widths = 0U;              /* implausible, restart the window */
        return;
    }

    /* Append the new width to the buffer, making room by sliding out the
     * oldest one first if we're already full. */
    if (n_widths >= WINDOW) {
        shift_window();
    }
    widths[n_widths] = w;
    n_widths++;

    /* Not enough elements buffered yet to attempt a decode — wait for
     * more edges to arrive. */
    if (n_widths < ELEMENTS) {
        return;
    }

    if (!classify(&pattern)) {
        shift_window();             /* slide by one and try again */
        return;
    }

    if (!lookup(pattern, &symbol, &reversed)) {
        shift_window();
        return;
    }

    /* Successful decode: reset the window */
    n_widths    = 0U;
    last_symbol = symbol;

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

    if (rc_event_subscribe(RC_EVT_BARCODE_EDGE, RC_LANE_FAST,
                           on_edge, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

/* Turns decoding on/off*/
rc_result_t sub_barcode_arm(bool on)
{
    armed    = on;
    n_widths = 0U;
    return drv_ir_barcode_enable(on);
}

/* Registers a function to call every time a character is successfully
 * decoded. */
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