/*
 *  rc_time.c
 */

#include "rc_prelude.h"
#include "rc_time.h"

/* TIMER_TIMERAWL, the raw low word, is defined in the port's sysdef.h.
 * Reading the RAW register avoids the latching behaviour of TIMELR, which
 * requires reading the high word first and is not ISR friendly. */

/* The TIMER block doesn't count clock cycles; it counts "ticks" from the
 * watchdog's tick generator, and each tick is CYCLES cycles of clk_ref.
 * clk_ref is the 12 MHz crystal here, so CYCLES must be 12 for 1 tick = 1 us.
 *
 * WARNING: after a cold power-on CYCLES is left at 0, which the hardware
 * treats as 512, so the timer runs 512/12 = 42.7x slow. The boot ROM sets it
 * to 12 when it reboots after a UF2 flash, which is why the slow timer only
 * shows up after unplugging and replugging (or powering on from battery). */
#define WATCHDOG_TICK           (0x4005802CUL)  /* WATCHDOG_BASE + 0x2C */
#define WATCHDOG_TICK_ENABLE    (1UL << 9)
#define CLK_REF_MHZ             (12UL)          /* the XOSC crystal */

void rc_time_init(void)
{
    out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | CLK_REF_MHZ);
}

uint32_t rc_time_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}
