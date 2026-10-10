/*
 *  drv_encoder.h  -  Buddy 2 hardware layer
 *
 *  Two-channel (A/B, "quadrature") Hall-effect encoders, one built into
 *  the back of each gear motor. They read the motor shaft before the
 *  gearbox, so one wheel turn is hundreds of ticks (RC_ENC_TICKS_PER_REV
 *  in rc_config.h). The two channels are the same signal offset by a
 *  quarter of a cycle, so at the instant A rises, B is low when the wheel
 *  turns one way and high when it turns the other. Channel A raises the
 *  edge interrupt and is counted; channel B is read inside that interrupt
 *  to get direction. So direction IS measured from the wheel itself -
 *  including during the moment it is still coasting after a reversal -
 *  not guessed from the last motor command.
 *
 *  Which sense of B is "forward" depends on how the encoder is mounted.
 *  If a wheel reads negative when the car is driven forward, swap that
 *  encoder's A and B wires rather than adding a sign in software.
 *
 *  Timing comes from the free-running microsecond counter inside the
 *  edge ISR (Interrupt Service Routine - code the chip jumps to the
 *  instant the sensor pin changes state, see TEAM_GUIDE.md §1.2), not
 *  from the kernel tick, so speed is usable down to very low RPM without
 *  a long averaging window.
 *
 *  "Odometry" (used throughout this project) just means: turning a count
 *  of wheel clicks into a distance and a speed - the same idea as
 *  counting clicks on a bicycle's spoke card.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "rc_types.h"

/* Call once at boot. Attaches the interrupt handlers to both encoder
 * pins and registers the deferred ("bottom half") worker that turns raw
 * edges into published events - see drv_encoder.c for why the real work
 * can't happen inside the interrupt itself. */
rc_result_t drv_encoder_init(void);

/* Total edges (wheel-clicks) seen since boot, monotonic (only ever goes
 * up, never resets on its own). */
uint32_t drv_encoder_count(rc_side_t side);

/* Microseconds between the two most recent edges, or the time since the
 * last edge if that is longer (so a slowing wheel's speed falls instead
 * of freezing). Returns 0 before the first edge and when the wheel has
 * been still longer than the stall timeout (so a stopped wheel reads as
 * "0", not as "infinitely slow"). */
uint32_t drv_encoder_period_us(rc_side_t side);

/* Signed speed in mm/s, derived from the period and the direction read
 * from channel B. Integer maths throughout, there is no FPU
 * (floating-point hardware unit) on this chip, so no fractional numbers
 * are used anywhere in this path. */
int32_t drv_encoder_speed_mm_s(rc_side_t side);

/* Direction of the most recent edge on one wheel: +1 forward, -1
 * reverse, 0 for an invalid side. Sampled from channel B in the ISR. */
int8_t drv_encoder_dir(rc_side_t side);

/* Distance in mm since the last drv_encoder_reset() call - this is the
 * actual odometry distance count used by sub_motion.c to know when a
 * "drive forward 300mm" move is done. */
uint32_t drv_encoder_distance_mm(rc_side_t side);

/* Zero the distance baseline (not the raw click count) so a new move can
 * measure "distance travelled during just this move" starting from zero. */
void drv_encoder_reset(void);

#endif /* DRV_ENCODER_H */
