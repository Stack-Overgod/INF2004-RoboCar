/*
 *  bar_frame.c  -  Buddy 3, see bar_frame.h
 *
 *  One read laid out in time, forwards ('*' X '*', 29 elements):
 *
 *      s .. s+8    start '*'   bars 0 0 1 1 0 (1 = wide)
 *      s+9         gap         narrow space
 *      s+10..s+18  letter      decoded with the decoder's own rule
 *      s+19        gap         narrow space
 *      s+20..s+28  stop '*'    bars 0 0 1 1 0, may be cut short
 *
 *  Read backwards every character is mirrored, so both '*'s show bars
 *  0 1 1 0 0 in time order and the letter decodes from its mirrored
 *  pattern. Only the bars of the '*'s are checked: the misreads seen on
 *  the track are spaces read too wide (the sensor spot makes white read
 *  narrower or wider with speed and height), while the bars read right.
 *  Wide/narrow for the '*' bars and the gaps is judged against the
 *  letter's own widths, which were read at the same speed moments apart.
 */
#include <stddef.h>
#include "bar_frame.h"

#define ELEMENTS        (9U)
/* The decoder's rule (sub_barcode.c classify): wide = at least 1.5x the
 * widest narrow one of the same kind. */
#define WIDE_RATIO      (15U)
/* Bars of '*' in time order, first bar = bit 4: forwards 00110,
 * backwards 01100. */
#define STAR_BARS_FWD   (0x06U)
#define STAR_BARS_REV   (0x0CU)
/* The stop '*' may be cut short; at least this many of its bars. */
#define STOP_MIN_BARS   (3U)
/* 1 = the gaps between characters must read narrow too. */
#ifndef BAR_FRAME_GAPS
#define BAR_FRAME_GAPS  (1)
#endif

/* Index (from `first`, step 2, within e[0..8]) of the widest element not
 * in `skip`. */
static uint32_t widest(const uint16_t *e, uint32_t first, uint32_t skip)
{
    uint32_t best = first;
    uint32_t i;

    while ((skip & (1U << best)) != 0U) {
        best += 2U;
    }
    for (i = first; i < ELEMENTS; i += 2U) {
        if (((skip & (1U << i)) == 0U) && (e[i] > e[best])) {
            best = i;
        }
    }
    return best;
}

/* The decoder's classification of 9 elements starting on a bar: pattern
 * (bit 8 = e[0]) and the bar / space thresholds halfway between the widest
 * narrow and the narrowest wide one. False if wide and narrow are too
 * alike. */
static bool classify(const uint16_t *e, uint16_t *pattern,
                     uint32_t *bar_thr, uint32_t *space_thr)
{
    uint32_t b1 = widest(e, 0U, 0U);
    uint32_t b2 = widest(e, 0U, (1U << b1));
    uint32_t b3 = widest(e, 0U, (1U << b1) | (1U << b2));
    uint32_t s1 = widest(e, 1U, 0U);
    uint32_t s2 = widest(e, 1U, (1U << s1));
    uint32_t wide;
    uint32_t i;
    uint16_t p = 0U;

    if ((((uint32_t)e[b2] * 10U) < ((uint32_t)e[b3] * WIDE_RATIO))
        || (((uint32_t)e[s1] * 10U) < ((uint32_t)e[s2] * WIDE_RATIO))) {
        return false;
    }
    wide = (1U << b1) | (1U << b2) | (1U << s1);
    for (i = 0U; i < ELEMENTS; i++) {
        p = (uint16_t)(p << 1);
        if ((wide & (1U << i)) != 0U) {
            p |= 1U;
        }
    }
    *pattern   = p;
    *bar_thr   = ((uint32_t)e[b2] + e[b3]) / 2U;
    *space_thr = ((uint32_t)e[s1] + e[s2]) / 2U;
    return true;
}

/* The 9-bit pattern read the other way round. */
static uint16_t mirror(uint16_t p)
{
    uint16_t r = 0U;
    uint32_t b;

    for (b = 0U; b < ELEMENTS; b++) {
        r = (uint16_t)(r << 1);
        if ((p & (1U << b)) != 0U) {
            r |= 1U;
        }
    }
    return r;
}

/* Do the bars from w[from] (every other element, up to 5, within n) match
 * `bars` (bit 4 = first)? Fewer than `min_bars` present: no. */
static bool bars_match(const uint16_t *w, uint32_t n, uint32_t from,
                       uint32_t thr, uint32_t bars, uint32_t min_bars)
{
    uint32_t k;
    uint32_t got = 0U;

    for (k = 0U; k < 5U; k++) {
        uint32_t i = from + (2U * k);
        bool     wide;

        if (i >= n) {
            break;
        }
        wide = (w[i] > thr);
        if (wide != ((bars & (1U << (4U - k))) != 0U)) {
            return false;
        }
        got++;
    }
    return got >= min_bars;
}

bool bar_frame_guess(const uint16_t *w, uint32_t n, bool first_bar,
                     bar_find_fn find, char *letter, bool *reversed,
                     uint32_t *at)
{
    uint32_t s;
    uint32_t found = 0U;
    char     c_found = 0;
    bool     rev_found = false;
    uint32_t at_found = 0U;

    if ((w == NULL) || (find == NULL) || (n < 25U)) {
        return false;           /* start, gap, letter, gap, 3 stop bars */
    }
    for (s = first_bar ? 0U : 1U; (s + 24U) < n; s += 2U) {
        const uint16_t *let = &w[s + 10U];
        uint16_t        p;
        uint32_t        bar_thr;
        uint32_t        space_thr;
        uint32_t        d;

        if (!classify(let, &p, &bar_thr, &space_thr)) {
            continue;
        }
        if (BAR_FRAME_GAPS
            && ((w[s + 9U] > space_thr) || (w[s + 19U] > space_thr))) {
            continue;           /* a gap between characters is narrow */
        }
        for (d = 0U; d < 2U; d++) {
            bool     rev  = (d != 0U);
            uint32_t bars = rev ? STAR_BARS_REV : STAR_BARS_FWD;
            char     c;

            if (!find(rev ? mirror(p) : p, &c) || (c == '*')) {
                continue;
            }
            if (!bars_match(w, n, s, bar_thr, bars, 5U)
                || !bars_match(w, n, s + 20U, bar_thr, bars, STOP_MIN_BARS)) {
                continue;
            }
            found++;
            c_found   = c;
            rev_found = rev;
            at_found  = s + 10U;
        }
    }
    if (found != 1U) {
        return false;           /* none, or ambiguous */
    }
    *letter   = c_found;
    *reversed = rev_found;
    *at       = at_found;
    return true;
}

/*** end of file ***/
