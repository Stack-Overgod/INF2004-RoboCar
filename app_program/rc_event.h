/*
 *  rc_event.h
 *
 *  The callback spine of the robot.
 *
 *  Producers (ISRs, drivers, subsystems) publish an rc_event_t. Consumers
 *  register a callback against an event id. Nothing blocks: publishing is
 *  a bounded copy into a lock-free-ish ring plus a tk_set_flg, and that is
 *  legal from interrupt context.
 *
 *  Two lanes exist so that a slow telemetry consumer can never delay the
 *  control loop:
 *
 *    RC_LANE_FAST   high priority dispatcher, control path only
 *    RC_LANE_SLOW   low priority dispatcher, telemetry, logging, planning
 *
 *  Callback contract
 *  -----------------
 *   - Callbacks run in the dispatcher TASK, never in the ISR, so they may
 *     call kernel API.
 *   - A callback must not block. No TMO_FEVR, no long waits. If it needs
 *     to wait, it should hand the work to its own task.
 *   - A callback must not call rc_event_subscribe or rc_event_unsubscribe
 *     for the lane it is running on.
 *   - Publishing a new event from inside a callback is allowed.
 */

#ifndef RC_EVENT_H
#define RC_EVENT_H

#include "rc_types.h"

typedef enum {
    RC_LANE_FAST = 0,
    RC_LANE_SLOW,
    RC_LANE_COUNT
} rc_lane_t;

/*
 *  Subscriber callback.
 *  evt points at storage owned by the dispatcher and is only valid for
 *  the duration of the call. Copy anything you want to keep.
 */
typedef void (*rc_event_cb_t)(const rc_event_t *evt, void *ctx);

/*
 *  Build the kernel objects and start both dispatcher tasks.
 *  Call once from usermain before any other rc_* init.
 */
rc_result_t rc_event_init(void);

/*
 *  Register a callback. Returns a handle >= 0, or a negative value on
 *  failure. Safe to call from task context during start-up.
 */
int32_t rc_event_subscribe(rc_evt_id_t id,
                           rc_lane_t lane,
                           rc_event_cb_t cb,
                           void *ctx);

rc_result_t rc_event_unsubscribe(int32_t handle);

/*
 *  Publish from task context. Fills evt->t_us for you.
 *  Never blocks. Returns RC_ERR_NOSPACE if a lane ring is full, which is
 *  a real condition worth counting rather than hiding.
 */
rc_result_t rc_event_publish(rc_event_t *evt);

/*
 *  Publish from interrupt context. Same behaviour, but uses the
 *  interrupt-safe lock and must not be called with the kernel lock held.
 */
rc_result_t rc_event_publish_i(rc_event_t *evt);

/*
 *  Number of events dropped because a ring was full, per lane.
 *  Non-zero means a consumer is too slow or a ring is too small.
 */
uint32_t rc_event_dropped(rc_lane_t lane);

#endif /* RC_EVENT_H */
