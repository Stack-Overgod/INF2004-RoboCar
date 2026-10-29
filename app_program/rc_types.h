/*
 *  rc_types.h
 *
 *  Types shared across every subsystem. Kept free of kernel headers so a
 *  decoder or planner can be unit-tested on a host build.
 */

#ifndef RC_TYPES_H
#define RC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ *
 *  Result codes. Distinct from the kernel's ER so a subsystem can
 *  report a domain failure without borrowing a kernel errno.
 * ------------------------------------------------------------------ */

typedef enum {
    RC_OK = 0,
    RC_ERR_PARAM,
    RC_ERR_STATE,
    RC_ERR_BUSY,
    RC_ERR_TIMEOUT,
    RC_ERR_NOSPACE,
    RC_ERR_HARDWARE
} rc_result_t;

/* ------------------------------------------------------------------ *
 *  Navigation vocabulary
 * ------------------------------------------------------------------ */

typedef enum {
    RC_CMD_NONE = 0,
    RC_CMD_TURN_LEFT,
    RC_CMD_TURN_RIGHT,
    RC_CMD_GO_STRAIGHT,
    RC_CMD_U_TURN,
    RC_CMD_STOP,
    RC_CMD_START    /* remote only: restart the run from STOPPED */
} rc_nav_cmd_t;

typedef enum {
    RC_MOTION_STATIONARY = 0,
    RC_MOTION_ACCELERATING,
    RC_MOTION_CRUISING,
    RC_MOTION_DECELERATING,
    RC_MOTION_TURNING,
    RC_MOTION_CLIMBING,
    RC_MOTION_DESCENDING,
    RC_MOTION_IMPACT
} rc_motion_class_t;

typedef enum {
    RC_SIDE_LEFT = 0,
    RC_SIDE_RIGHT
} rc_side_t;

/* ------------------------------------------------------------------ *
 *  Event identifiers
 *
 *  One per thing that can happen. Subsystems never call each other
 *  directly; they publish one of these and whoever cares subscribes.
 * ------------------------------------------------------------------ */

typedef enum {
    RC_EVT_NONE = 0,

    /* core */
    RC_EVT_TICK,                /* periodic, carries a sequence number   */

    /* buddy 2, motion */
    RC_EVT_ENCODER_EDGE,        /* published from ISR context            */
    RC_EVT_ODOMETRY,            /* speed and distance, RC_PERIOD_MOTION  */
    RC_EVT_MOTION_DONE,         /* a queued move finished or aborted     */

    /* buddy 3, line and barcode */
    RC_EVT_LINE_SAMPLE,         /* raw sensor state, every LINE period   */
    RC_EVT_LINE_LOST,
    RC_EVT_LINE_REACQUIRED,
    RC_EVT_BARCODE_EDGE,        /* published from ISR context            */
    RC_EVT_BARCODE_DECODED,

    /* buddy 4, IMU and terrain */
    RC_EVT_IMU_SAMPLE,
    RC_EVT_HUMP_BEGIN,
    RC_EVT_HUMP_END,            /* carries the measured peak             */
    RC_EVT_MOTION_CLASS,
    RC_EVT_IMPACT,

    /* buddy 5, scanning and avoidance */
    RC_EVT_ULTRA_RESULT,        /* one range measurement completed       */
    RC_EVT_SCAN_COMPLETE,
    RC_EVT_OBSTACLE_PROFILE,
    RC_EVT_AVOIDANCE_PLAN,

    /* buddy 1, comms */
    RC_EVT_COMMAND_RX,          /* a command arrived over the link       */
    RC_EVT_LINK_STATE,

    /* buddy 5, navigation (sub_nav). Appended so no existing id moves. */
    RC_EVT_NAV_TIMEOUT,         /* sub_nav's own state timer; tick.seq   */
    RC_EVT_RECOVERY_FAILED,     /* FR9: line not found after a bypass    */
    /* buddy 3, barcode.*/
    RC_EVT_BARCODE_SCAN,        /* summary of one pass over a barcode,   */
    RC_EVT_BARCODE_RAW,         /* the same pass as wide/narrow bits     */
    /* buddy 5, spin scan diagnostics. Appended so no existing id moves. */
    RC_EVT_SPIN_EDGES,          /* each pass's edge, after a spin scan   */


    RC_EVT_COUNT                /* keep last */
} rc_evt_id_t;

/* ------------------------------------------------------------------ *
 *  Event payloads
 *
 *  Every payload is small and copied by value into the ring, so a
 *  publisher never has to keep a buffer alive for a subscriber.
 * ------------------------------------------------------------------ */

typedef struct {
    uint32_t seq;
} rc_pl_tick_t;

typedef struct {
    rc_side_t side;
    uint32_t  count;            /* running edge count for that wheel     */
    uint32_t  delta_us;         /* gap since the previous edge           */
} rc_pl_encoder_t;

typedef struct {
    int32_t  speed_l_mm_s;
    int32_t  speed_r_mm_s;
    uint32_t dist_l_mm;
    uint32_t dist_r_mm;
} rc_pl_odometry_t;

typedef struct {
    uint32_t    move_id;
    bool        completed;      /* false means aborted                   */
    uint32_t    travelled_mm;
} rc_pl_motion_done_t;

typedef struct {
    bool     on_line_l;
    bool     on_line_r;
    int16_t  position;          /* -1000 far left .. +1000 far right     */
} rc_pl_line_t;

typedef struct {
    bool     level_high;        /* level after the edge                  */
    uint32_t width_us;          /* how long the previous level lasted    */
} rc_pl_bar_edge_t;

typedef struct {
    char         symbol;        /* decoded Code 39 character             */
    rc_nav_cmd_t command;       /* mapped navigation command             */
    bool         reversed;      /* decoded while scanning back to front  */
} rc_pl_barcode_t;

/* One scan attempt: every edge from the first after a quiet spell until
 * the sensor goes quiet again. Counters stop at 255. A failed window is
 * normal between characters of a good read; a pass with ok == 0 is the
 * one to look at. 16 bytes, so rc_event_t does not grow. */
typedef struct {
    char     text[4];           /* first 3 symbols decoded, NUL-ended    */
    uint8_t  edges;             /* bar/space edges seen                  */
    uint8_t  ok;                /* characters decoded                    */
    uint8_t  f_ratio;           /* window not wide:narrow >= 2           */
    uint8_t  f_wide;            /* window without exactly 3 wide         */
    uint8_t  f_nomatch;         /* 3 wide, but not in the Code 39 table  */
    uint8_t  f_glitch;          /* edge under ELEM_MIN_US, window reset  */
    uint16_t last_nomatch;      /* pattern of the last f_nomatch, 9 bits */
    uint16_t min_10us;          /* narrowest element, 10 us units        */
    uint16_t max_10us;          /* widest element, 10 us units           */
} rc_pl_bar_scan_t;


/* One scan attempt as the sensor saw it: every bar and space in order,
 * 1 = wide, 0 = narrow, each judged against its 9 neighbours (as the
 * decoder does), so a slow-down part way through does not turn every
 * later element wide. Edges under ELEM_MIN_US are left out. 16 bytes. */
#define RC_BAR_RAW_MAX  (64U)
typedef struct {
    uint8_t  n;                 /* elements recorded, <= RC_BAR_RAW_MAX  */
    uint8_t  first_bar;         /* 1: element 0 is a bar (black)         */
    uint8_t  bits[8];           /* element i wide: bits[i/8] bit (i%8)   */
    uint16_t nar_10us;          /* average narrow element, 10 us units   */
    uint16_t wid_10us;          /* average wide element, 10 us units     */
    uint8_t  more;              /* elements past RC_BAR_RAW_MAX, dropped */
    uint8_t  pad;
} rc_pl_bar_raw_t;

/* A spin scan's edges, one per pass (sub_scan): [0] left on the way out,
 * [1] left on the way back across, [2] right on the way out, [3] right on
 * the way back to straight. Heading in 0.1 deg (> 0 left) where the beam
 * left the obstacle, range in 2 mm units (0..510 mm). 16 bytes. */
typedef struct {
    uint16_t face_mm;           /* face distance from the axle           */
    int16_t  ddeg[4];
    uint8_t  r_2mm[4];
    uint8_t  found;             /* bit i: pass i found its edge          */
    uint8_t  n_points;          /* readings taken                        */
} rc_pl_spin_t;

typedef struct {
    int16_t acc_x;              /* raw, in milli-g                       */
    int16_t acc_y;
    int16_t acc_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} rc_pl_imu_t;

typedef struct {
    uint16_t peak_mm;           /* estimated peak height of the hump     */
    uint32_t duration_ms;
    int16_t  max_pitch_deg;
} rc_pl_hump_t;

typedef struct {
    rc_motion_class_t cls;
} rc_pl_motion_class_t;

typedef struct {
    int16_t  angle_deg;
    uint16_t range_mm;
    bool     valid;             /* false on echo timeout                 */
} rc_pl_ultra_t;

typedef struct {
    uint8_t  n_points;
    int16_t  closest_angle_deg;
    uint16_t closest_mm;
    uint16_t width_mm;
    uint16_t clearance_left_mm;
    uint16_t clearance_right_mm;
} rc_pl_profile_t;

typedef struct {
    rc_nav_cmd_t action;
    uint16_t     lateral_mm;    /* how far to step aside                 */
    uint16_t     forward_mm;    /* how far ahead of the axle the car must
                                 * get before turning back in (sub_nav
                                 * bypass; spin scan: past a square
                                 * obstacle's far side)                  */
} rc_pl_plan_t;

typedef struct {
    rc_nav_cmd_t command;
    int32_t      arg;
} rc_pl_command_t;

typedef struct {
    bool connected;
    bool broker_up;
} rc_pl_link_t;

/* ------------------------------------------------------------------ *
 *  The event itself
 * ------------------------------------------------------------------ */

typedef struct {
    rc_evt_id_t id;
    uint32_t    t_us;           /* microsecond stamp, filled by publish  */
    union {
        rc_pl_tick_t         tick;
        rc_pl_encoder_t      encoder;
        rc_pl_odometry_t     odometry;
        rc_pl_motion_done_t  motion_done;
        rc_pl_line_t         line;
        rc_pl_bar_edge_t     bar_edge;
        rc_pl_barcode_t      barcode;
        rc_pl_bar_scan_t     bar_scan;
        rc_pl_bar_raw_t      bar_raw;
        rc_pl_spin_t         spin;
        rc_pl_imu_t          imu;
        rc_pl_hump_t         hump;
        rc_pl_motion_class_t motion_class;
        rc_pl_ultra_t        ultra;
        rc_pl_profile_t      profile;
        rc_pl_plan_t         plan;
        rc_pl_command_t      command;
        rc_pl_link_t         link;
    } u;
} rc_event_t;

#endif /* RC_TYPES_H */
