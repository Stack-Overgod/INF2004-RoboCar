/*
 * sub_telemetry.c  --  Buddy 1 / subsystem 1
 *
 * Builds the car's telemetry messages and ships them through whatever sink
 * is installed (console by default, MQTT in a WiFi build), and passes
 * driving commands from the dashboard on to the vehicle controller.
 *
 * How this file links to the other buddies:
 *   - everything we report comes off the event bus. subs[] near the bottom
 *     is the full list of events we listen to, one row each, grouped by
 *     the buddy who publishes them. Nobody calls into this file to report
 *     something; they publish and we pick it up;
 *   - the exceptions are single-value read-backs with no event behind
 *     them: Buddy 2's drv_motor_get() (motor duty) and drv_encoder_count()
 *     (encoder clicks), sub_nav_state(), rc_fault_get() (the fault
 *     bitmask) and Buddy 4's sub_terrain_height_mm();
 *   - what we hand back goes out on the bus too: RC_EVT_COMMAND_RX for each
 *     command from the dashboard, RC_EVT_LINK_STATE whenever the network
 *     link comes up or goes down;
 *   - init() only subscribes and makes objects; start_task() starts the
 *     task (priority 12). All callbacks are on the slow lane, so nothing
 *     here can hold up the control path.
 * subsystem1.h has the same contract as a table; docs/buddy1-telemetry/
 * walks the other buddies through it.
 *
 * Every message is a compact binary frame (tm_frame.h: 2-byte header,
 * then each field in only the bits its range needs). On the laptop,
 * scripts/tm_frames.py turns frames back into JSON for the broker log and
 * the dashboard. Commands in are one ASCII byte.
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>

#include "sub_telemetry.h"
#include "subsystem1.h"             /* S1_PRI_TELEMETRY, app_start_task() */
#include "../rc_fmt.h"              /* rc_snprintf(), rc_strlen() */
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_time.h"
#include "../rc_fault.h"            /* rc_fault_get(), WIFI_LOSS bit */
#include "../sub_nav.h"             /* sub_nav_state(), read only */
#include "tm_frame.h"               /* tm_pack(), tm_to_json() */
#include "../subsystem2/drv_motor.h"    /* drv_motor_get(), Buddy 2 */
#include "../subsystem2/drv_encoder.h"  /* drv_encoder_count(), Buddy 2 */
#include "../subsystem4/sub_terrain.h"  /* sub_terrain_height_mm(), Buddy 4 */

/* Topic buffer holds "car/01/<leaf>". A frame is at most TM_FRAME_MAX
 * (32) bytes, far inside the kernel's message limit (S1_MSG_MAX, 240)
 * with the topic and its '\0'. */
#define TOPIC_MAX       (48)

/* One prefix for every topic (rc_config.h), so the publish topics and the
 * command topic can't drift apart. */
#define TOPIC_BASE      RC_TOPIC_BASE

/* Status (the heartbeat) goes out every 8th state tick: 8 x 250 ms = 2 s,
 * the period FR11a asks for. */
#define HEARTBEAT_TICKS (8U)

/* Every 4th state message (1 Hz) is the long form with the two 32-bit
 * encoder counts (27 B instead of 19 B); the dashboard keeps the last
 * counts in between. */
#define ENC_EVERY_TICKS (4U)

/* A buddy counts as "live" in the heartbeat if we've heard its stream this
 * recently. Those streams run at 17-200 Hz, so a whole second of silence
 * means it has stopped, not that it's between samples. */
#define LIVE_WINDOW_MS  (1000U)

/* The servo angle that points straight ahead, the middle of the servo's
 * RC_SERVO_ANGLE_MIN..RC_SERVO_ANGLE_MAX swing. Only readings this close to it count
 * as "the range ahead" in the state message; the rest of a sweep is
 * reported as a whole on car/01/obstacle. */
#define AHEAD_DEG       (90)
#define AHEAD_TOL_DEG   (5)

/* The transport we're currently sending through. */
static const sub_telemetry_sink_t *sink;

static uint8_t  state_seq;      /* header seq of state frames (task only) */
static uint8_t  evt_seq;        /* header seq of every other frame (locked) */
static uint32_t tx_count;       /* messages sent OK (in the heartbeat) */
static uint32_t tx_fail;        /* messages that failed (in the heartbeat) */
static uint32_t rx_cmds;        /* command characters received */
static uint32_t retries;        /* times the link was found down and retried */

static sub_telemetry_cmd_cb_t cmd_cb;
static void *cmd_ctx;

static ID telem_tid;            /* the telemetry task, for the self-tests */

/* Latest readings, cached by the event callbacks and read by the task.
 * Each is written by one callback and read by one task, and each is a
 * single byte or halfword (one store / one load on the M0+, never torn),
 * so no lock is needed; volatile stops the compiler caching a stale value.
 * Each is the smallest C type that holds its field (the frame itself then
 * packs them to the exact bit width). The two line bits live in ONE
 * byte written by ONE callback (on_line), so there is no read-modify-write
 * between writers. */
static volatile int16_t  st_speed_l;        /* mm/s, 11-bit range */
static volatile int16_t  st_speed_r;
static volatile uint16_t st_dist_mm;        /* 16-bit odometer, wraps */
static volatile uint8_t  st_line;           /* bit 0 left, bit 1 right */
static volatile uint8_t  st_motion_cls;     /* latest RC_EVT_MOTION_CLASS */
static volatile uint8_t  st_peak_mm;        /* tallest hump so far, 7-bit */
static volatile uint16_t st_range_mm;       /* straight ahead, 0 = no echo */
static volatile uint8_t  st_last_bc;        /* RC_SYM_* code, 0 = none yet */

/* The motion class last sent on car/01/motion (-1 = nothing yet). Only
 * the telemetry task touches it. */
static int16_t sent_cls = -1;

/* When each buddy's stream was last heard, in ms since boot, 0 = never.
 * Nothing can arrive at 0 ms (the kernel is still booting then), so 0 is
 * free to mean "never". */
typedef enum
{
    LIVE_B2_ODOMETRY = 0,       /* RC_EVT_ODOMETRY, 50 Hz */
    LIVE_B3_LINE,               /* RC_EVT_LINE_SAMPLE, 200 Hz */
    LIVE_B4_IMU,                /* RC_EVT_IMU_SAMPLE, 100 Hz */
    LIVE_B5_ULTRA,              /* RC_EVT_ULTRA_RESULT, ~17 Hz while watching */
    LIVE_COUNT
} live_src_t;

static volatile uint32_t live_ms[LIVE_COUNT];

/* The network link, as last put on the bus. link_sent is false until the
 * first announcement has gone out. */
static rc_pl_link_t link_now;
static bool         link_sent;

#if APP_TEST
static volatile bool link_fake_on;
static rc_pl_link_t  link_fake;
#endif

/* --------------------------------------------------------------------
 * Counter lock (dual-core)
 *
 * Some counters are bumped from two tasks -- the telemetry task and the
 * slow event dispatcher (via on_notable) -- so a bare count++ can drop a
 * count when both cores hit it at once. Same lock pattern as rc_event.c:
 * a spinlock on SMP, interrupt masking on single core. NOT a mutex: the
 * dispatcher is never allowed to wait.
 * -------------------------------------------------------------------- */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK count_lock;
/* NOTE: ISpinLock wants a UINT*, so sts must be UINT, not uint32_t. */
#define COUNT_LOCK(sts)      ((void)ISpinLock(&count_lock, &(sts)))
#define COUNT_UNLOCK(sts)    ((void)ISpinUnlock(&count_lock, (sts)))
#else
#define COUNT_LOCK(sts)      DI(sts)
#define COUNT_UNLOCK(sts)    EI(sts)
#endif

static void count_inc(uint32_t *ctr)
{
    UINT sts;

    COUNT_LOCK(sts);
    (*ctr)++;
    COUNT_UNLOCK(sts);
}

/* Next event-frame seq. Event frames come from the dispatcher and the
 * telemetry task both, so take it under the same lock. */
static uint8_t next_evt_seq(void)
{
    UINT sts;
    uint8_t s;

    COUNT_LOCK(sts);
    s = evt_seq++;
    COUNT_UNLOCK(sts);
    return s;
}

/* --------------------------------------------------------------------
 * Console sink -- always there, no radio needed.
 * -------------------------------------------------------------------- */

static rc_result_t console_open(void)
{
    return RC_OK;
}

/* The payload is a binary frame now, so show it as the JSON it stands
 * for, with its size on the wire. */
static rc_result_t console_publish(const char *topic, const char *payload,
                                   uint16_t len)
{
    char json[256];

    if (!tm_to_json((const uint8_t *)payload, len, json, sizeof(json)))
    {
        (void)rc_snprintf(json, sizeof(json), "(bad frame)");
    }
    tm_printf((UB *)"[telem] %s %uB %s\n", topic, (unsigned int)len, json);
    return RC_OK;
}

static rc_result_t console_close(void)
{
    return RC_OK;
}

static bool console_is_up(void)
{
    return true;    /* no cable to unplug */
}

static const sub_telemetry_sink_t console_sink =
{
    "console", console_open, console_publish, console_close, console_is_up
};

const sub_telemetry_sink_t *sub_telemetry_console_sink(void)
{
    return &console_sink;
}

/* --------------------------------------------------------------------
 * Framing (binary, tm_frame.c) and the one send() everything funnels
 * through.
 * -------------------------------------------------------------------- */

/* Pack vals[] as a frame of `type`, build the full topic, give up if the
 * transport is down, send, and keep the tallies. One place for the
 * counting so callers don't repeat it. */
static rc_result_t send(tm_type_t type, uint8_t sq, const int32_t *vals)
{
    char topic[TOPIC_MAX];
    uint8_t frame[TM_FRAME_MAX];
    uint16_t len;
    rc_result_t res;

    if ((sink == NULL) || !sink->is_up())
    {
        count_inc(&tx_fail);
        return RC_ERR_STATE;
    }

    len = tm_pack(type, sq, vals, frame, (uint16_t)sizeof(frame));
    if (len == 0U)
    {
        count_inc(&tx_fail);
        return RC_ERR_PARAM;
    }
    (void)rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE,
                      tm_leaf(type));

    res = sink->publish(topic, (const char *)frame, len);
    if (res == RC_OK)
    {
        count_inc(&tx_count);
    }
    else
    {
        count_inc(&tx_fail);
    }
    return res;
}

/* An event frame: next event seq. */
static rc_result_t send_evt(tm_type_t type, const int32_t *vals)
{
    return send(type, next_evt_seq(), vals);
}

/* The per-tick status frame: speed, distance, motor duty, line sensors,
 * motion class, tallest hump, range ahead, last barcode, nav state, the
 * fault bitmask and the height above the start level (ramps) -- the
 * telemetry FR10a asks for -- and once a second the encoder counts too. Every field is a cached value updated by the
 * event callbacks below, except encoder counts and motor duty, which
 * Buddy 2 reports live through drv_encoder_count() and drv_motor_get(),
 * the nav state (sub_nav_state(), one byte), the faults (one kernel
 * read of the event flag) and Buddy 4's live height (one halfword).
 * Order = f_state[] in tm_frame.c. */
static void publish_state(bool with_enc)
{
    int32_t v[16];

    v[0]  = (int32_t)rc_time_ms();
    v[1]  = st_speed_l;
    v[2]  = st_speed_r;
    v[3]  = (int32_t)st_dist_mm;
    v[4]  = drv_motor_get(RC_SIDE_LEFT);
    v[5]  = drv_motor_get(RC_SIDE_RIGHT);
    v[6]  = (int32_t)st_line;
    v[7]  = (int32_t)st_motion_cls;
    v[8]  = (int32_t)st_peak_mm;
    v[9]  = (int32_t)st_range_mm;
    v[10] = (int32_t)st_last_bc;
    v[11] = (int32_t)sub_nav_state();
    v[12] = (int32_t)rc_fault_get();
    v[13] = (int32_t)sub_terrain_height_mm();
    v[14] = (int32_t)drv_encoder_count(RC_SIDE_LEFT);
    v[15] = (int32_t)drv_encoder_count(RC_SIDE_RIGHT);

    (void)send(with_enc ? TM_T_STATE_ENC : TM_T_STATE, state_seq, v);
    state_seq++;
}

/* Bit (src) set if we've heard from this buddy in the last second. */
static int32_t live_bit(live_src_t src, uint32_t now)
{
    uint32_t t = live_ms[src];

    return ((t != 0U) && ((now - t) <= LIVE_WINDOW_MS))
           ? (int32_t)(1U << (uint32_t)src) : 0;
}

/* The slower "is the link healthy" frame (FR11a): uptime, send tallies,
 * event-bus drops, active sink, the WiFi and broker state, commands
 * received, link retries, and which buddies' streams are arriving.
 * Counters saturate at their field width (tm_pack). A non-zero drop
 * count is a real warning. "live" is four bits in buddy order -- bit 0
 * Buddy 2 odometry, 1 Buddy 3 line, 2 Buddy 4 IMU, 3 Buddy 5 ultrasonic --
 * shown as "1011" (everyone but Buddy 3). A 0 for Buddy 4 is how an IMU
 * that never came up shows (FR15), as well as RC_FAULT_IMU. */
static void publish_heartbeat(void)
{
    int32_t v[11];
    uint32_t now = rc_time_ms();

    v[0]  = (int32_t)now;
    v[1]  = (int32_t)tx_count;
    v[2]  = (int32_t)tx_fail;
    v[3]  = (int32_t)rc_event_dropped(RC_LANE_FAST);
    v[4]  = (int32_t)rc_event_dropped(RC_LANE_SLOW);
    v[5]  = (sink == &console_sink) ? 0 :
            (((sink != NULL) && (sink->name[0] == 'm')) ? 1 : 2);
    v[6]  = link_now.connected ? 1 : 0;
    v[7]  = link_now.broker_up ? 1 : 0;
    v[8]  = (int32_t)rx_cmds;
    v[9]  = (int32_t)retries;
    v[10] = live_bit(LIVE_B2_ODOMETRY, now) | live_bit(LIVE_B3_LINE, now) |
            live_bit(LIVE_B4_IMU, now) | live_bit(LIVE_B5_ULTRA, now);

    (void)send(TM_T_STATUS, state_seq, v);
}

/* --------------------------------------------------------------------
 * Event callbacks. All on the slow lane, so telemetry never jumps ahead
 * of the control path. They run on the dispatcher task, so they stay
 * short: cache and return, or send one message and return.
 * -------------------------------------------------------------------- */

static void mark_live(live_src_t src)
{
    uint32_t now = rc_time_ms();

    live_ms[src] = (now != 0U) ? now : 1U;
}

/* Buddy 2: speed and distance, every 20 ms. */
static void on_odometry(const rc_event_t *evt, void *ctx)
{
    uint16_t l = evt->u.odometry.dist_l_mm;
    uint16_t r = evt->u.odometry.dist_r_mm;

    (void)ctx;
    mark_live(LIVE_B2_ODOMETRY);
    st_speed_l = (int16_t)evt->u.odometry.speed_l_mm_s;
    st_speed_r = (int16_t)evt->u.odometry.speed_r_mm_s;

    /* Average of the two wheels' 16-bit odometers. Go half the signed
     * gap from the left one, so it still averages right when one of them
     * has just wrapped past 65535 and the other hasn't. */
    st_dist_mm = (uint16_t)(l + (int16_t)(uint16_t)(r - l) / 2);
}

/* Buddy 3: the two line sensors, every 10 ms. */
static void on_line(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    mark_live(LIVE_B3_LINE);
    st_line = (uint8_t)(evt->u.line.on_line_l | (evt->u.line.on_line_r << 1));
}

/* Buddy 4: we only use the IMU stream to know it's alive (see "live" in
 * the heartbeat). Buddy 4 turns it into everything else. */
static void on_imu(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;
    mark_live(LIVE_B4_IMU);
}

/* Buddy 4: the motion class, sent only when it changes. Cache it for the
 * per-tick line; the task streams the change on car/01/motion (see
 * stream_motion_class, which explains why it isn't sent from here). */
static void on_motion_class(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_motion_cls = (uint8_t)evt->u.motion_class.cls;
}

/* Buddy 5: one ultrasonic reading. Keep it only if the servo was pointing
 * straight ahead; an echo timeout reads as 0 ("nothing in range"). */
static void on_ultra(const rc_event_t *evt, void *ctx)
{
    int32_t off = (int32_t)evt->u.ultra.angle_deg - AHEAD_DEG;

    (void)ctx;
    mark_live(LIVE_B5_ULTRA);
    if ((off >= -AHEAD_TOL_DEG) && (off <= AHEAD_TOL_DEG))
    {
        st_range_mm = (uint16_t)evt->u.ultra.range_mm;
    }
}

/* Everything that gets its own message the moment it happens: barcode scan attempt,
 * hump, impact, obstacle, avoidance plan, lost/found, move done.
 * Two of them also feed the per-tick line, so cache those on the way. */
static void on_notable(const rc_event_t *evt, void *ctx)
{
    (void)ctx;

    if (evt->id == RC_EVT_BARCODE_DECODED)
    {
        st_last_bc = (uint8_t)evt->u.barcode.sym;
    }
    else if (evt->id == RC_EVT_HUMP_END)
    {
        if (evt->u.hump.peak_mm > st_peak_mm)
        {
            st_peak_mm = (uint8_t)evt->u.hump.peak_mm;
        }
    }
    else
    {
        /* nothing to cache for the rest */
    }

    (void)sub_telemetry_publish_event(evt);
}

/* Turn one "noteworthy" event into its own frame on its own topic. Each
 * case fills v[] in the order of that type's table in tm_frame.c. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    int32_t  v[TM_FIELDS_MAX];
    uint32_t i;

    if (evt == NULL)
    {
        return RC_ERR_PARAM;
    }

    switch (evt->id)
    {
        case RC_EVT_BARCODE_DECODED:
            v[0] = (int32_t)evt->u.barcode.sym;
            v[1] = (int32_t)evt->u.barcode.command;
            v[2] = (int32_t)evt->u.barcode.reversed;
            return send_evt(TM_T_BARCODE, v);

        case RC_EVT_BARCODE_SCAN:
        {
            /* text: up to 3 symbols as 5-bit codes; the last unmatched
             * pattern only when there was one (else 0) */
            const rc_pl_bar_scan_t *s = &evt->u.bar_scan;

            v[0] = 0;
            for (i = 0U; (i < 3U) && (s->text[i] != '\0'); i++)
            {
                v[0] |= (int32_t)((uint32_t)rc_sym_code(s->text[i])
                                  << (5U * i));
            }
            v[1] = s->ok;
            v[2] = s->edges;
            v[3] = s->f_ratio;
            v[4] = s->f_wide;
            v[5] = s->f_nomatch;
            v[6] = s->f_glitch;
            v[7] = (s->f_nomatch != 0U) ? (int32_t)s->last_nomatch : 0;
            v[8] = (s->min_10us == 0xFFFFU) ? 0 : (int32_t)s->min_10us;
            v[9] = s->max_10us;
            return send_evt(TM_T_BCSCAN, v);
        }

        case RC_EVT_SPIN_EDGES:
        {
            /* each pass: heading 0.1 deg (> 0 left) and range in 2 mm
             * units; `found` says which passes are real */
            const rc_pl_spin_t *sp = &evt->u.spin;

            v[0] = sp->face_mm;
            v[1] = sp->n_points;
            v[2] = sp->found;
            for (i = 0U; i < 4U; i++)
            {
                v[3U + (2U * i)] = sp->ddeg[i];
                v[4U + (2U * i)] = sp->r_2mm[i];
            }
            return send_evt(TM_T_SPIN, v);
        }

        case RC_EVT_BARCODE_RAW:
        {
            /* the 64 wide/narrow bits as 8 bytes; the laptop prints them
             * as "010010100 0 011010000 ..." */
            const rc_pl_bar_raw_t *r = &evt->u.bar_raw;

            v[0] = r->n;
            v[1] = r->first_bar;
            for (i = 0U; i < 8U; i++)
            {
                v[2U + i] = r->bits[i];
            }
            v[10] = r->nar_10us;
            v[11] = r->wid_10us;
            v[12] = r->more;
            return send_evt(TM_T_BCBITS, v);
        }

        case RC_EVT_HUMP_END:
            v[0] = (int32_t)evt->u.hump.peak_mm;
            v[1] = (int32_t)evt->u.hump.duration_ms;
            v[2] = (int32_t)evt->u.hump.max_pitch_deg;
            return send_evt(TM_T_HUMP, v);

        case RC_EVT_RAMP_END:
            v[0] = (int32_t)evt->u.ramp.rise_mm;
            v[1] = (int32_t)evt->u.ramp.net_mm;
            v[2] = (int32_t)evt->u.ramp.max_deg;
            v[3] = (int32_t)evt->u.ramp.len_mm;
            return send_evt(TM_T_RAMP, v);

        case RC_EVT_OBSTACLE_PROFILE:
            v[0] = (int32_t)evt->u.profile.n_points;
            v[1] = (int32_t)evt->u.profile.closest_mm;
            v[2] = (int32_t)evt->u.profile.closest_angle_deg;
            v[3] = (int32_t)evt->u.profile.width_mm;
            v[4] = (int32_t)evt->u.profile.clearance_left_mm;
            v[5] = (int32_t)evt->u.profile.clearance_right_mm;
            return send_evt(TM_T_OBSTACLE, v);

        case RC_EVT_AVOIDANCE_PLAN:
            v[0] = (int32_t)evt->u.plan.action;
            v[1] = (int32_t)evt->u.plan.lateral_mm;
            v[2] = (int32_t)evt->u.plan.forward_mm;
            return send_evt(TM_T_AVOID, v);

        case RC_EVT_IMPACT:
            v[0] = 1;
            return send_evt(TM_T_IMPACT, v);

        case RC_EVT_MOTION_CLASS:
            v[0] = (int32_t)evt->u.motion_class.cls;
            return send_evt(TM_T_MOTION, v);

        case RC_EVT_LINE_LOST:
            v[0] = 1;
            return send_evt(TM_T_LINE, v);

        case RC_EVT_LINE_REACQUIRED:
            v[0] = 0;
            return send_evt(TM_T_LINE, v);

        case RC_EVT_MOTION_DONE:
            v[0] = (int32_t)evt->u.motion_done.move_id;
            v[1] = (int32_t)evt->u.motion_done.completed;
            v[2] = (int32_t)evt->u.motion_done.travelled_mm;
            return send_evt(TM_T_MOVE, v);

        default:
            return RC_ERR_PARAM;
    }
}

/* --------------------------------------------------------------------
 * The link (design section 6.5)
 * -------------------------------------------------------------------- */

static void link_read(rc_pl_link_t *out)
{
    bool connected = false;
    bool broker_up = false;

#if APP_TEST
    if (link_fake_on)
    {
        *out = link_fake;
        return;
    }
#endif
    /* via locals: the payload fields are 1-bit, no address to take */
    sub_telemetry_mqtt_link(&connected, &broker_up);
    out->connected = connected ? 1U : 0U;
    out->broker_up = broker_up ? 1U : 0U;
}

/* Once a tick: if WiFi or the broker came up or went down since the last
 * look, say so on the bus with RC_EVT_LINK_STATE. The first tick always
 * announces, so anyone who subscribed in their _init() learns the starting
 * state. If the bus is full we keep the old state and try next tick. */
static void track_link(void)
{
    rc_pl_link_t now;
    rc_event_t evt;

    link_read(&now);
    if (link_sent && (now.connected == link_now.connected) &&
        (now.broker_up == link_now.broker_up))
    {
        return;
    }

    evt.id = RC_EVT_LINK_STATE;
    evt.u.link = now;
    if (rc_event_publish(&evt) == RC_OK)
    {
        link_now = now;
        link_sent = true;
    }

    /* Fault bit: WiFi or the broker down. Only the console sink has no
     * link to lose. */
    if ((sink == &console_sink) || (now.connected && now.broker_up))
    {
        rc_fault_clear(RC_FAULT_WIFI_LOSS);
    }
    else
    {
        rc_fault_set(RC_FAULT_WIFI_LOSS);
    }
}

void sub_telemetry_get_link(rc_pl_link_t *out)
{
    if (out != NULL)
    {
        *out = link_now;
    }
}

#if APP_TEST
void sub_telemetry_test_fake_link(bool on, bool connected, bool broker_up)
{
    link_fake.connected = connected ? 1U : 0U;
    link_fake.broker_up = broker_up ? 1U : 0U;
    link_fake_on = on;
}
#endif

/* --------------------------------------------------------------------
 * The task.
 * -------------------------------------------------------------------- */

/* Checked once per tick before publishing. The console sink is always up,
 * so this returns true straight away on a plain build. For a network sink
 * it's the reconnect: on a drop, close, wait a growing back-off, try once
 * to reopen. The wait is a kernel sleep, never a busy loop, so a dead link
 * only ever slows this lowest-priority task. */
static bool telemetry_link_ok(void)
{
    static uint32_t backoff = RC_NET_BACKOFF_MIN_MS;

    if ((sink != NULL) && sink->is_up())
    {
        backoff = RC_NET_BACKOFF_MIN_MS;
        return true;
    }

    if (sink == NULL)
    {
        return false;
    }

    count_inc(&retries);
    (void)sink->close();
    tk_dly_tsk((INT)backoff);
    (void)sink->open();

    backoff <<= 1;
    if (backoff > RC_NET_BACKOFF_MAX_MS)
    {
        backoff = RC_NET_BACKOFF_MAX_MS;
    }

    return sink->is_up();
}

/* FR14: every change of motion class gets its own message. Buddy 4 only
 * publishes on a change, but a class that flickers could still change
 * dozens of times a second, and the kernel sends at most 20 MQTT messages
 * a second -- the queue would fill and state messages would start
 * failing. So the callback only caches the class and this sends it, at
 * most once a tick. A class that comes and goes inside one tick isn't
 * streamed; IMPACT, the one that matters, has its own topic anyway. */
static void stream_motion_class(void)
{
    rc_event_t evt;
    int16_t cls = (int16_t)st_motion_cls;

    if (cls == sent_cls)
    {
        return;
    }

    evt.id = RC_EVT_MOTION_CLASS;
    evt.u.motion_class.cls = (uint8_t)cls;
    if (sub_telemetry_publish_event(&evt) == RC_OK)
    {
        sent_cls = cls;
    }
}

static void telemetry_task(INT stacd, void *exinf)
{
    uint32_t ticks = 0U;

    (void)stacd;
    (void)exinf;

    if (sink != NULL)
    {
        (void)sink->open();
    }

    for (;;)
    {
        /* Sleep one period. In an MQTT build this also takes any command
         * the moment the kernel hands it over, so a remote STOP isn't left
         * waiting for the end of the tick. */
        sub_telemetry_mqtt_wait(RC_PERIOD_TELEM_MS);

        track_link();

        /* Skip this tick if the link is down -- no point hammering a dead
         * sink; send() would just count another failure. */
        if (!telemetry_link_ok())
        {
            continue;
        }

        publish_state((ticks % ENC_EVERY_TICKS) == 0U);
        stream_motion_class();

        ticks++;
        if ((ticks % HEARTBEAT_TICKS) == 0U)
        {
            publish_heartbeat();
        }
    }
}

/* --------------------------------------------------------------------
 * Start-up (two passes: init then start).
 * -------------------------------------------------------------------- */

/* Everything Buddy 1 listens to, grouped by who publishes it. This table
 * IS the link to the other buddies: if your event is in here, Buddy 1
 * reports it; you don't call anything. All on the slow lane. */
typedef struct
{
    rc_evt_id_t   id;
    rc_event_cb_t cb;
} telem_sub_t;

static const telem_sub_t subs[] =
{
    /* Buddy 2, motion */
    { RC_EVT_ODOMETRY,          on_odometry     },  /* state: spd, dist   */
    { RC_EVT_MOTION_DONE,       on_notable      },  /* car/01/move        */

    /* Buddy 3, line and barcode */
    { RC_EVT_LINE_SAMPLE,       on_line         },  /* state: line        */
    { RC_EVT_LINE_LOST,         on_notable      },  /* car/01/line        */
    { RC_EVT_LINE_REACQUIRED,   on_notable      },  /* car/01/line        */
    { RC_EVT_BARCODE_DECODED,   on_notable      },  /* car/01/barcode, bc */
    { RC_EVT_BARCODE_SCAN,      on_notable      },  /* car/01/bcscan      */
    { RC_EVT_BARCODE_RAW,       on_notable      },  /* car/01/bcbits      */

    /* Buddy 4, IMU and terrain */
    { RC_EVT_IMU_SAMPLE,        on_imu          },  /* status: live       */
    { RC_EVT_MOTION_CLASS,      on_motion_class },  /* cls, car/01/motion */
    { RC_EVT_HUMP_END,          on_notable      },  /* car/01/hump, peak  */
    { RC_EVT_RAMP_END,          on_notable      },  /* car/01/ramp        */
    { RC_EVT_IMPACT,            on_notable      },  /* car/01/impact      */

    /* Buddy 5, scanning and avoidance */
    { RC_EVT_ULTRA_RESULT,      on_ultra        },  /* state: rng         */
    { RC_EVT_OBSTACLE_PROFILE,  on_notable      },  /* car/01/obstacle    */
    { RC_EVT_AVOIDANCE_PLAN,    on_notable      },  /* car/01/avoid       */
    { RC_EVT_SPIN_EDGES,        on_notable      }   /* car/01/spin        */
};

/* Pass one: subscribe to everything in subs[] and, in a WiFi build, make
 * the message buffers and switch the sink to MQTT. No task yet. A failed
 * subscription is reported (subsystem1_init turns it into an error that
 * usermain prints) but the rest still go ahead. */
rc_result_t sub_telemetry_init(void)
{
    rc_result_t res = RC_OK;
    uint32_t i;

    sink = &console_sink;

    for (i = 0U; i < (sizeof(subs) / sizeof(subs[0])); i++)
    {
        if (rc_event_subscribe(subs[i].id, RC_LANE_SLOW, subs[i].cb, NULL)
            < 0)
        {
            tm_printf((UB *)"telemetry: can't subscribe to event %d\n",
                      (int)subs[i].id);
            res = RC_ERR_NOSPACE;
        }
    }

#if TM_MQTT
    if (sub_telemetry_mqtt_init() == RC_OK)
    {
        sink = sub_telemetry_mqtt_sink();
    }
    else
    {
        tm_printf((UB *)"telemetry: no MQTT buffers, using the console\n");
    }
#endif

    return res;
}

/* Pass two: start the telemetry task in subsystem 1's priority band
 * (TEAM.md section 4). It's the slow lane, so it never holds up control. */
rc_result_t sub_telemetry_start_task(void)
{
    telem_tid = app_start_task(telemetry_task, S1_PRI_TELEMETRY, 0,
                               "telemetry");

    return (telem_tid < E_OK) ? RC_ERR_HARDWARE : RC_OK;
}

/* The telemetry task's ID (0 before start_task). Lets subsystem1_test.c
 * check its priority and that it sleeps between ticks. */
int32_t sub_telemetry_task_id(void)
{
    return (int32_t)telem_tid;
}

/* Swap the transport. Closes the old one, points at the new (or console on
 * NULL), opens it. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *new_sink)
{
    if (sink != NULL)
    {
        (void)sink->close();
    }
    sink = (new_sink != NULL) ? new_sink : &console_sink;

    return sink->open();
}

/* --------------------------------------------------------------------
 * Commands.
 * -------------------------------------------------------------------- */

rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx)
{
    cmd_cb = cb;
    cmd_ctx = ctx;
    return RC_OK;
}

/* (L/R/S/U/X) and the barcode letters (A/B/C/D), either case; P also means
 * stop and G (go) restarts a stopped run. Anything else is RC_CMD_NONE and
 * gets ignored. Lives here rather
 * than in the MQTT file so it can be tested without a WiFi build. */
rc_nav_cmd_t sub_telemetry_parse_cmd(char c)
{
    switch (c)
    {
        case 'L': case 'l': case 'A': case 'a': return RC_CMD_TURN_LEFT;
        case 'R': case 'r': case 'B': case 'b': return RC_CMD_TURN_RIGHT;
        case 'S': case 's': case 'C': case 'c': return RC_CMD_GO_STRAIGHT;
        case 'U': case 'u': case 'D': case 'd': return RC_CMD_U_TURN;
        case 'X': case 'x': case 'P': case 'p': return RC_CMD_STOP;
        case 'G': case 'g':                     return RC_CMD_START;
        default:                                return RC_CMD_NONE;
    }
}

/* A character from car/01/cmd. Pass a real command on first, so a STOP
 * isn't held up by the ack, then tell the dashboard what we made of it on
 * car/01/ack: the byte received and the command (0 if it was ignored).
 * The laptop shows a quote, a backslash or a control byte as '?'. */
rc_nav_cmd_t sub_telemetry_receive_command(char c)
{
    int32_t v[2];
    rc_nav_cmd_t cmd = sub_telemetry_parse_cmd(c);

    count_inc(&rx_cmds);
    if (cmd != RC_CMD_NONE)
    {
        (void)sub_telemetry_deliver_command(cmd, 0);
    }

    v[0] = (int32_t)(uint8_t)c;
    v[1] = (int32_t)cmd;
    (void)send_evt(TM_T_ACK, v);

    return cmd;
}

/* Called once a command has been decoded. It always goes on the bus as
 * RC_EVT_COMMAND_RX, which the vehicle controller subscribes to on the
 * fast lane; the callback, if someone registered one, gets it as well.
 * Either way we don't touch navigation's state from here. */
rc_result_t sub_telemetry_deliver_command(rc_nav_cmd_t cmd, int32_t arg)
{
    rc_event_t evt;
    rc_result_t res;

    evt.id = RC_EVT_COMMAND_RX;
    evt.u.command.command = (uint32_t)cmd;
    evt.u.command.arg = RC_SAT_S(arg, 29U);
    res = rc_event_publish(&evt);

    if (cmd_cb != NULL)
    {
        cmd_cb(cmd, arg, cmd_ctx);
    }
    return res;
}
