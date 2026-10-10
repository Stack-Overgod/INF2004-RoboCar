/*
 *  rc_fmt.h
 *
 *  A small bounded formatter, used instead of newlib's snprintf.
 *
 *  Two reasons, both of which matter to the resource efficiency part of the
 *  assessment:
 *
 *    1. newlib's vfprintf machinery is roughly 10-20 kB of flash once the
 *       linker pulls it in, on a part with 2 MB of external flash but only
 *       264 kB of RAM and a 125 MHz M0+. Telemetry formatting does not need
 *       it.
 *    2. Some newlib configurations reach for malloc inside printf. Calling
 *       malloc from a periodic task on an RTOS is avoidable, so avoid it.
 *
 *  Supported conversions, which is everything the telemetry layer uses:
 *
 *      %d  %i   int          %u   unsigned int
 *      %ld %li  long         %lu  unsigned long
 *      %c       char         %s   const char *
 *      %%       literal percent
 *
 *  No floats (there is no FPU), no width or precision, no padding. Output is
 *  always NUL terminated as long as size >= 1. Returns the number of
 *  characters written, excluding the terminator.
 */

#ifndef RC_FMT_H
#define RC_FMT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

int32_t rc_snprintf(char *buf, uint32_t size, const char *fmt, ...);
int32_t rc_vsnprintf(char *buf, uint32_t size, const char *fmt, va_list ap);

uint32_t rc_strlen(const char *s);

#endif /* RC_FMT_H */
