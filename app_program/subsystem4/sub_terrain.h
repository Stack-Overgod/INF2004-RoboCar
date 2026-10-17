/*
 *  sub_terrain.h  -  Buddy 4
 *
 *  Hump detection, peak measurement, motion classification and impact
 *  detection, all from the LSM303D accelerometer.
 *
 *  Read the warning at the top of drv_imu.h first. There is no
 *  gyroscope, which rules out the textbook approach to most of this.
 */
#ifndef SUB_TERRAIN_H
#define SUB_TERRAIN_H

#include "../rc_types.h"

/* This defines the *shape* of a "callback" function - a function you
 * write elsewhere that this module will call automatically once a hump
 * finishes. `sub_terrain_hump_cb_t` is a "function pointer type": any
 * function matching this signature (taking a peak height, a duration,
 * and a free-form context pointer, returning nothing) can be registered
 * with sub_terrain_on_hump() below and will get invoked for you. This is
 * how C does "notify me when X happens" without an event bus message. */
typedef void (*sub_terrain_hump_cb_t)(uint16_t peak_mm,
                                      uint32_t duration_ms,
                                      void *ctx);

/* Call once at boot. Subscribes this module to IMU sample events so it
 * starts watching for humps and tracking motion class automatically. */
rc_result_t sub_terrain_init(void);

/* Optional: register your own function to be called the instant a hump
 * finishes (in addition to the RC_EVT_HUMP_END event this module always
 * publishes). `ctx` is any pointer you want handed back to you unchanged
 * - useful for passing "which object called this" without global
 * variables. */
rc_result_t sub_terrain_on_hump(sub_terrain_hump_cb_t cb, void *ctx);

/* Highest hump peak seen this run, in mm. This is the number the brief
 * asks to be reported. */
uint16_t sub_terrain_max_peak_mm(void);

/* What the car is doing right now, e.g. cruising / turning / climbing a
 * hump / impact - see rc_motion_class_t in rc_types.h for the full list. */
rc_motion_class_t sub_terrain_motion_class(void);

/* Worst-case execution time (WCET) observed for on_sample() on the FAST
 * event dispatcher lane, in microseconds (must fit well within 5000 us). */
uint32_t sub_terrain_wcet_us(void);

/* Clears hump-tracking and motion-class state back to defaults, e.g. at
 * the start of a fresh run. */
void sub_terrain_reset(void);

#endif /* SUB_TERRAIN_H */