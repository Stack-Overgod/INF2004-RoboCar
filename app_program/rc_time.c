/*
 *  rc_time.c
 */

#include "rc_prelude.h"
#include "rc_time.h"

/* TIMER_TIMERAWL, the raw low word, is defined in the port's sysdef.h.
 * Reading the RAW register avoids the latching behaviour of TIMELR, which
 * requires reading the high word first and is not ISR friendly. */

void rc_time_init(void)
{
    /* The port already brings the TIMER block out of reset during
     * hardware init, so there is nothing to do here. The function exists
     * so callers have one obvious place to add a check if that changes. */
}

uint32_t rc_time_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}
