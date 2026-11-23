/*
 *  bar_frame.h  -  Buddy 3, recovering a letter whose '*' start/stop
 *                  characters were read badly
 *
 *  Every barcode on the track is '*', one letter, '*'. When the sensor
 *  reads a '*' with a wrong space width the decoder never sees the start
 *  character and, by FR5, drops the letter. bar_frame_guess() looks at a
 *  whole finished read and finds a letter that decodes cleanly AND has the
 *  bars of a '*' on BOTH sides, read the same way round: then the '*'s
 *  were there and only their spaces misread. Pure computation (no kernel
 *  calls), so it is tested on the PC (test/barframe/).
 */
#ifndef BAR_FRAME_H
#define BAR_FRAME_H

#include <stdint.h>
#include <stdbool.h>

/* Looks up a 9-element pattern (bit 8 = first element, 1 = wide) in the
 * Code 39 table: true and the symbol if found. */
typedef bool (*bar_find_fn)(uint16_t pattern, char *out);

/* w[0..n-1]: every element of one read, in time order (any unit, the same
 * for all); first_bar: w[0] is a bar. True if exactly one letter (not
 * '*') is framed by '*' bars on both sides as described above; then
 * *letter, *reversed (read backwards) and *at (index of its first
 * element). */
bool bar_frame_guess(const uint16_t *w, uint32_t n, bool first_bar,
                     bar_find_fn find, char *letter, bool *reversed,
                     uint32_t *at);

#endif /* BAR_FRAME_H */
/*** end of file ***/
