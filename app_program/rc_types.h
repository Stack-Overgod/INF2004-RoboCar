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
    /* buddy 4, ramp height. Appended so no existing id moves. */
    RC_EVT_RAMP_END,            /* one slope climbed/descended, height   */


    RC_EVT_COUNT                /* keep last */
} rc_evt_id_t;

/* ------------------------------------------------------------------ *
 *  Minimum-bit fields
 *
 *  Every payload field below holds only the bits its valid range needs,
 *  as a C bit-field. The ranges and widths are the data representation
 *  table in docs/cr1/. Rules that keep the packing safe:
 *    - a value is clamped into its field (RC_SAT_U / RC_SAT_S) before it
 *      is stored, so an out-of-range reading saturates, never wraps;
 *    - payloads are only ever written whole, by one publisher, into a
 *      private rc_event_t, and copied by value through the event ring
 *      (rc_event.c locks the ring), so no two tasks or cores ever do a
 *      read-modify-write on the same packed word;
 *    - live shared flags (faults) are a kernel event flag, not a packed
 *      variable: see rc_fault.h.
 *  Bit-field layout is compiler-defined, so nothing here is ever sent as
 *  raw bytes: the MQTT frames (subsystem1/tm_frame.c) pack field by field.
 * ------------------------------------------------------------------ */

#define RC_BITS_MM          (12U)   /* 0..4095 mm: the ultrasonic tops out at 4000 */
#define RC_BITS_SPEED       (11U)   /* -1024..+1023 mm/s; car tops out ~530 */
#define RC_BITS_DIST        (16U)   /* 0..65535 mm odometer, wraps (deltas only) */
#define RC_BITS_ANGLE       (8U)    /* servo angle 0..180 deg */
#define RC_BITS_CMD         (3U)    /* rc_nav_cmd_t, 7 values */
#define RC_BITS_CLS         (3U)    /* rc_motion_class_t, 8 values */
#define RC_BITS_SYM         (5U)    /* barcode symbol code, 28 values */
#define RC_BITS_ACC         (12U)   /* +/-2047 mg; LSM303D at +/-2 g */
#define RC_BITS_HUMP_PEAK   (7U)    /* 0..127 mm; track humps are ~20-60 mm */
#define RC_BITS_HUMP_MS     (13U)   /* 0..8191 ms; HUMP_MAX_MS is 5000 */
#define RC_BITS_PITCH       (8U)    /* -128..+127 deg */
#define RC_BITS_HEIGHT      (10U)   /* -512..+511 mm, live height (ramp) */
#define RC_BITS_RAMP_RISE   (9U)    /* 0..511 mm climbed on one slope */
#define RC_BITS_RAMP_DEG    (6U)    /* 0..63 deg, steepest slope */
#define RC_BITS_RAMP_LEN    (12U)   /* 0..4095 mm rolled on one slope */
#define RC_BITS_MOVE_ID     (15U)   /* sub_motion ids 1..32767, then wrap */
#define RC_BITS_MOVE_MM     (16U)   /* 0..65535 mm per move */
#define RC_BITS_WIDTH_US    (31U)   /* barcode element, up to ~35 min */
#define RC_BITS_NPOINTS     (6U)    /* scan readings, saturates at 63 */
#define RC_BITS_CLEAR       (10U)   /* 0..1023 mm side clearance (more = clear) */
#define RC_BITS_ENC_DT      (17U)   /* 0..131071 us between encoder edges */

#define RC_FIELD_MAX(bits)  ((uint32_t)((1UL << (bits)) - 1UL))

/* Clamp into an unsigned / signed field of `bits` bits. The unsigned
 * clamp takes a 64-bit value so that a negative input clamps to 0 and any
 * uint32_t input keeps its value. */
static inline uint32_t rc_sat_u(int64_t v, uint32_t bits)
{
    int64_t m = (int64_t)RC_FIELD_MAX(bits);

    return (uint32_t)((v > m) ? m : ((v < 0) ? 0 : v));
}

static inline int32_t rc_sat_s(int32_t v, uint32_t bits)
{
    int32_t hi = (int32_t)RC_FIELD_MAX(bits - 1U);
    int32_t lo = -hi - 1;

    return (v > hi) ? hi : ((v < lo) ? lo : v);
}

#define RC_SAT_U(v, bits)   rc_sat_u((int64_t)(v), (bits))
#define RC_SAT_S(v, bits)   rc_sat_s((int32_t)(v), (bits))

/* Barcode symbol code (5 bits): 0 none, 1 the start/stop '*', 2..27 the
 * letters 'A'..'Z' (sub_barcode.c decodes all of them). */
#define RC_SYM_NONE         (0U)
#define RC_SYM_STAR         (1U)

static inline uint8_t rc_sym_code(char c)
{
    if (c == '*') {
        return (uint8_t)RC_SYM_STAR;
    }
    if ((c >= 'A') && (c <= 'Z')) {
        return (uint8_t)(2 + (c - 'A'));
    }
    return (uint8_t)RC_SYM_NONE;
}

static inline char rc_sym_char(uint32_t code)
{
    if (code == RC_SYM_STAR) {
        return '*';
    }
    if ((code >= 2U) && (code <= 27U)) {
        return (char)('A' + (int)(code - 2U));
    }
    return '\0';
}

/* ------------------------------------------------------------------ *
 *  Event payloads
 *
 *  Every payload is small and copied by value into the ring, so a
 *  publisher never has to keep a buffer alive for a subscriber.
 * ------------------------------------------------------------------ */

typedef struct {
    uint32_t seq;               /* full word: sub_nav's timer generation  */
} rc_pl_tick_t;

/* Encoder counts are full 32-bit words: one writer (the encoder ISR) per
 * word, so they are read without a lock. */
typedef struct {
    uint32_t  count;            /* running edge count for that wheel     */
    uint32_t  delta_us:17;      /* gap since the previous edge, clamped
                                 * (drv_encoder's stall timeout is 100 ms) */
    uint32_t  side:1;           /* rc_side_t                             */
    uint32_t  :14;
} rc_pl_encoder_t;

/* Distances are a 16-bit odometer that wraps every 65.5 m:
 * consumers use the difference between two samples (uint16_t subtract). */
typedef struct {
    int32_t  speed_l_mm_s:11;
    int32_t  speed_r_mm_s:11;
    uint32_t :10;
    uint16_t dist_l_mm;
    uint16_t dist_r_mm;
} rc_pl_odometry_t;

typedef struct {
    uint32_t move_id:15;        /* RC_BITS_MOVE_ID                       */
    uint32_t completed:1;       /* 0 means aborted                       */
    uint32_t travelled_mm:16;   /* RC_BITS_MOVE_MM                       */
} rc_pl_motion_done_t;

/* The two sensors are the whole sample; the coarse position (-500 / 0 /
 * +500) is derived from them by rc_line_position(). */
typedef struct {
    uint8_t  on_line_l:1;
    uint8_t  on_line_r:1;
} rc_pl_line_t;

static inline int16_t rc_line_position(const rc_pl_line_t *s)
{
    if (s->on_line_l && !s->on_line_r) {
        return -500;
    }
    if (s->on_line_r && !s->on_line_l) {
        return 500;
    }
    return 0;
}

typedef struct {
    uint32_t width_us:31;       /* how long the previous level lasted    */
    uint32_t level_high:1;      /* level after the edge (1 = black)      */
} rc_pl_bar_edge_t;

typedef struct {
    uint8_t  sym:5;             /* RC_SYM_* code; rc_sym_char() for text */
    uint8_t  command:3;         /* rc_nav_cmd_t                          */
    uint8_t  reversed:1;        /* decoded while scanning back to front  */
    uint8_t  :7;
} rc_pl_barcode_t;

/* One scan attempt: every edge from the first after a quiet spell until
 * the sensor goes quiet again. Counters stop at 255. A failed window is
 * normal between characters of a good read; a pass with ok == 0 is the
 * one to look at. Diagnostics, already byte-packed (8-bit counters that
 * saturate, 16-bit 10 us widths); 16 bytes. */
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

/* Accelerometer only: nothing subscribes to the magnetometer (drv_imu.c
 * reads it for its own checks). */
typedef struct {
    int32_t  acc_x:12;          /* milli-g, RC_BITS_ACC                  */
    int32_t  acc_y:12;
    int32_t  :8;
    int32_t  acc_z:12;
    int32_t  :20;
} rc_pl_imu_t;

typedef struct {
    uint32_t peak_mm:7;         /* estimated peak height, clamped 127    */
    uint32_t duration_ms:13;    /* HUMP_MAX_MS 5000 fits                 */
    int32_t  max_pitch_deg:8;
    uint32_t :4;
} rc_pl_hump_t;

/* RC_EVT_RAMP_END (terrain_ramp.c): one slope episode, ended when the car
 * was level again for RAMP_LEVEL_MM. Heights are relative to where the
 * slope began, so "up a ramp" is rise = net > 0 and "down it again" is
 * rise 0, net < 0; a hump is rise > 0, net ~ 0. 8 B. */
typedef struct {
    uint32_t rise_mm:9;         /* highest point above the start, 511    */
    int32_t  net_mm:10;         /* end height above the start, +-511     */
    uint32_t max_deg:6;         /* steepest |slope|, whole degrees       */
    uint32_t :7;
    uint32_t len_mm:12;         /* ground distance on the slope, 4095    */
    uint32_t :20;
} rc_pl_ramp_t;

typedef struct {
    uint8_t  cls:3;             /* rc_motion_class_t                     */
} rc_pl_motion_class_t;

/* range_mm 0 means no echo (timeout or out of range), so there is no
 * separate valid flag: use RC_ULTRA_VALID(). */
typedef struct {
    uint32_t range_mm:12;
    uint32_t angle_deg:8;       /* servo angle the ping was tagged with  */
    uint32_t :12;
} rc_pl_ultra_t;

#define RC_ULTRA_VALID(u)   ((u)->range_mm != 0U)

typedef struct {
    uint32_t n_points:6;        /* readings, saturates at 63             */
    uint32_t closest_angle_deg:8;
    uint32_t closest_mm:12;
    uint32_t :6;
    uint32_t width_mm:12;
    uint32_t clearance_left_mm:10;  /* 0..1023: beyond is "clear" anyway */
    uint32_t clearance_right_mm:10;
} rc_pl_profile_t;

typedef struct {
    uint32_t action:3;          /* rc_nav_cmd_t                          */
    uint32_t lateral_mm:12;     /* how far to step aside                 */
    uint32_t forward_mm:12;     /* how far ahead of the axle the car must
                                 * get before turning back in (sub_nav
                                 * bypass; spin scan: past a square
                                 * obstacle's far side)                  */
    uint32_t :5;
} rc_pl_plan_t;

/* arg is always 0 (reserved for a move length). */
typedef struct {
    uint32_t command:3;         /* rc_nav_cmd_t                          */
    int32_t  arg:29;
} rc_pl_command_t;

typedef struct {
    uint8_t  connected:1;
    uint8_t  broker_up:1;
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
        rc_pl_ramp_t         ramp;
        rc_pl_motion_class_t motion_class;
        rc_pl_ultra_t        ultra;
        rc_pl_profile_t      profile;
        rc_pl_plan_t         plan;
        rc_pl_command_t      command;
        rc_pl_link_t         link;
    } u;
} rc_event_t;

#endif /* RC_TYPES_H */
