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
 *   - the two exceptions are Buddy 2's read-backs drv_motor_get() (motor
 *     duty) and drv_encoder_count() (encoder clicks). There's no event for
 *     either, and both just read a value Buddy 2 already keeps;
 *   - what we hand back goes out on the bus too: RC_EVT_COMMAND_RX for each
 *     command from the dashboard, RC_EVT_LINK_STATE whenever the network
 *     link comes up or goes down;
 *   - init() only subscribes and makes objects; start_task() starts the
 *     task (priority 12). All callbacks are on the slow lane, so nothing
 *     here can hold up the control path.
 * subsystem1.h has the same contract as a table; docs/buddy1-telemetry/
 * walks the other buddies through it.
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>

#include "sub_telemetry.h"
#include "subsystem1.h"             /* S1_PRI_TELEMETRY, app_start_task() */
#include "../rc_fmt.h"              /* rc_snprintf(), rc_strlen() */
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_time.h"
#include "../subsystem2/drv_motor.h"    /* drv_motor_get(), Buddy 2 */
#include "../subsystem2/drv_encoder.h"  /* drv_encoder_count(), Buddy 2 */

/* Topic buffer holds "car/01/<leaf>"; payload buffer holds one JSON body.
 * Bump PAYLOAD_MAX first if a message ever looks cut off, but it can't go
 * past 226: the kernel's message limit (S1_MSG_MAX, 240) has to hold the
 * topic "car/01/state" and its '\0' as well. */
#define TOPIC_MAX       (48)
#define PAYLOAD_MAX     (224)   /* worst-case state is 207; see S1-I10 */
#define ACK_MAX         (32)

/* One prefix for every topic (rc_config.h), so the publish topics and the
 * command topic can't drift apart. */
#define TOPIC_BASE      RC_TOPIC_BASE

/* Status (the heartbeat) goes out every 8th state tick: 8 x 250 ms = 2 s,
 * the period FR11a asks for. */
#define HEARTBEAT_TICKS (8U)

/* A buddy counts as "live" in the heartbeat if we've heard its stream this
 * recently. Those streams run at 17-200 Hz, so a whole second of silence
 * means it has stopped, not that it's between samples. */
#define LIVE_WINDOW_MS  (1000U)

/* The servo angle that points straight ahead. Buddy 5's sweep runs 30 to
 * 150 degrees with 90 in the middle. Only readings this close to it count
 * as "the range ahead" in the state message; the rest of a sweep is
 * reported as a whole on car/01/obstacle. */
#define AHEAD_DEG       (90)
#define AHEAD_TOL_DEG   (5)

/* The transport we're currently sending through. */
static const sub_telemetry_sink_t *sink;

static uint32_t seq;            /* stamped into each state message */
static uint32_t tx_count;       /* messages sent OK (in the heartbeat) */
static uint32_t tx_fail;        /* messages that failed (in the heartbeat) */
static uint32_t rx_cmds;        /* command characters received */
static uint32_t retries;        /* times the link was found down and retried */

static sub_telemetry_cmd_cb_t cmd_cb;
static void *cmd_ctx;

static ID telem_tid;            /* the telemetry task, for the self-tests */

/* Latest readings, cached by the event callbacks and read by the task.
 * Each is written by one callback and read by one task, so a word-sized
 * copy is safe without a lock; volatile stops the compiler caching a
 * stale value across the callback that updates it. */
static volatile int32_t  st_speed_l;
static volatile int32_t  st_speed_r;
static volatile uint32_t st_dist_mm;
static volatile bool     st_line_l;
static volatile bool     st_line_r;
static volatile int32_t  st_motion_cls;    /* latest RC_EVT_MOTION_CLASS */
static volatile uint32_t st_peak_mm;        /* tallest hump seen so far */
static volatile uint32_t st_range_mm;       /* straight ahead, 0 = no echo */
static volatile char     st_last_bc;        /* most recent barcode symbol */

/* The motion class last sent on car/01/motion (-1 = nothing yet). Only
 * the telemetry task touches it. */
static int32_t sent_cls = -1;

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

/* --------------------------------------------------------------------
 * Console sink -- always there, no radio needed.
 * -------------------------------------------------------------------- */

static rc_result_t console_open(void)
{
    return RC_OK;
}

static rc_result_t console_publish(const char *topic, const char *payload,
                                   uint16_t len)
{
    (void)len;      /* tm_printf finds the end of the string itself */
    tm_printf((UB *)"[telem] %s %s\n", topic, payload);
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
 * Framing (JSON) and the one send() everything funnels through.
 * -------------------------------------------------------------------- */

/* Build the full topic, give up if the transport is down, send, and keep
 * the tallies. One place for the counting so callers don't repeat it. */
static rc_result_t send(const char *leaf, const char *payload)
{
    char topic[TOPIC_MAX];
    rc_result_t res;

    if ((sink == NULL) || !sink->is_up())
    {
        count_inc(&tx_fail);
        return RC_ERR_STATE;
    }

    (void)rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf);

    res = sink->publish(topic, payload, (uint16_t)rc_strlen(payload));
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

/* The per-tick status line: speed, distance, encoder counts, motor duty,
 * line sensors, motion class, tallest hump, range ahead, last barcode --
 * the telemetry FR10a asks for. Every field is a cached value updated by
 * the event callbacks below, except encoder counts and motor duty, which
 * Buddy 2 reports live through drv_encoder_count() and drv_motor_get(). */
static void publish_state(void)
{
    char buf[PAYLOAD_MAX];

    /* NOTE: with every number at its longest this is 207 characters, so
     * PAYLOAD_MAX is 224. subsystem1_test.c S1-I10 checks it on the car,
     * so add a field there too if you add one here. */
    (void)rc_snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"t\":%lu,\"spd_l\":%ld,\"spd_r\":%ld,"
        "\"dist\":%lu,\"enc_l\":%lu,\"enc_r\":%lu,"
        "\"m_l\":%d,\"m_r\":%d,\"line\":\"%c%c\","
        "\"cls\":%d,\"peak\":%u,\"rng\":%u,\"bc\":\"%c\"}",
        (unsigned long)seq,
        (unsigned long)rc_time_ms(),
        (long)st_speed_l,
        (long)st_speed_r,
        (unsigned long)st_dist_mm,
        (unsigned long)drv_encoder_count(RC_SIDE_LEFT),
        (unsigned long)drv_encoder_count(RC_SIDE_RIGHT),
        (int)drv_motor_get(RC_SIDE_LEFT),
        (int)drv_motor_get(RC_SIDE_RIGHT),
        st_line_l ? '1' : '0',
        st_line_r ? '1' : '0',
        (int)st_motion_cls,
        (unsigned int)st_peak_mm,
        (unsigned int)st_range_mm,
        (st_last_bc != '\0') ? st_last_bc : '-');

    seq++;
    (void)send("state", buf);
}

/* '1' if we've heard from this buddy in the last second, else '0'. */
static char live_flag(live_src_t src, uint32_t now)
{
    uint32_t t = live_ms[src];

    return ((t != 0U) && ((now - t) <= LIVE_WINDOW_MS)) ? '1' : '0';
}

/* The slower "is the link healthy" line (FR11a): uptime, send tallies,
 * event-bus drops, active sink, the WiFi and broker state, commands
 * received, link retries, and which buddies' streams are arriving. A
 * non-zero drop count is a real warning. "live" is four flags in buddy
 * order -- Buddy 2 odometry, 3 line, 4 IMU, 5 ultrasonic -- so "1011"
 * means everyone but Buddy 3 is talking. A '0' for Buddy 4 is how an IMU
 * that never came up shows (FR15), rather than a silent zero. */
static void publish_heartbeat(void)
{
    char buf[PAYLOAD_MAX];
    uint32_t now = rc_time_ms();

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"up\":%lu,\"tx\":%lu,\"fail\":%lu,"
        "\"drop_fast\":%lu,\"drop_slow\":%lu,\"sink\":\"%s\","
        "\"wifi\":%d,\"mqtt\":%d,\"cmds\":%lu,\"retries\":%lu,"
        "\"live\":\"%c%c%c%c\"}",
        (unsigned long)now,
        (unsigned long)tx_count,
        (unsigned long)tx_fail,
        (unsigned long)rc_event_dropped(RC_LANE_FAST),
        (unsigned long)rc_event_dropped(RC_LANE_SLOW),
        (sink != NULL) ? sink->name : "none",
        link_now.connected ? 1 : 0,
        link_now.broker_up ? 1 : 0,
        (unsigned long)rx_cmds,
        (unsigned long)retries,
        live_flag(LIVE_B2_ODOMETRY, now),
        live_flag(LIVE_B3_LINE, now),
        live_flag(LIVE_B4_IMU, now),
        live_flag(LIVE_B5_ULTRA, now));

    (void)send("status", buf);
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
    uint32_t l = evt->u.odometry.dist_l_mm;
    uint32_t r = evt->u.odometry.dist_r_mm;

    (void)ctx;
    mark_live(LIVE_B2_ODOMETRY);
    st_speed_l = evt->u.odometry.speed_l_mm_s;
    st_speed_r = evt->u.odometry.speed_r_mm_s;

    /* Average of the two wheels. Halve each one first: (l + r) / 2 would
     * overflow 32 bits once the sum passes ~4,295 km. */
    st_dist_mm = (l / 2U) + (r / 2U) + (l & r & 1U);
}

/* Buddy 3: the two line sensors, every 10 ms. */
static void on_line(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    mark_live(LIVE_B3_LINE);
    st_line_l = evt->u.line.on_line_l;
    st_line_r = evt->u.line.on_line_r;
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
    st_motion_cls = (int32_t)evt->u.motion_class.cls;
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
        st_range_mm = evt->u.ultra.valid ? evt->u.ultra.range_mm : 0U;
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
        st_last_bc = evt->u.barcode.symbol;
    }
    else if (evt->id == RC_EVT_HUMP_END)
    {
        if (evt->u.hump.peak_mm > st_peak_mm)
        {
            st_peak_mm = evt->u.hump.peak_mm;
        }
    }
    else
    {
        /* nothing to cache for the rest */
    }

    (void)sub_telemetry_publish_event(evt);
}

/* Turn one "noteworthy" event into its own message on its own topic. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    char buf[PAYLOAD_MAX];

    if (evt == NULL)
    {
        return RC_ERR_PARAM;
    }

    switch (evt->id)
    {
        case RC_EVT_BARCODE_DECODED:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"sym\":\"%c\",\"cmd\":%d,\"rev\":%d}",
                evt->u.barcode.symbol,
                (int)evt->u.barcode.command,
                evt->u.barcode.reversed ? 1 : 0);
            return send("barcode", buf);

        case RC_EVT_BARCODE_SCAN:
        {
            /* The last unmatched pattern, first element first, as w (wide)
             * / n (narrow), e.g. "nnwwnnnnw"; "-" if every 3-wide window
             * matched. */
            const rc_pl_bar_scan_t *s = &evt->u.bar_scan;
            char     pat[10];
            uint32_t b;

            if (s->f_nomatch == 0U)
            {
                pat[0] = '-';
                pat[1] = '\0';
            }
            else
            {
                for (b = 0U; b < 9U; b++)
                {
                    pat[b] = ((s->last_nomatch & (1U << (8U - b))) != 0U)
                             ? 'w' : 'n';
                }
                pat[9] = '\0';
            }
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"res\":\"%s\",\"txt\":\"%s\",\"ok\":%u,\"edges\":%u,"
                "\"f_ratio\":%u,\"f_wide\":%u,\"f_nomatch\":%u,"
                "\"f_glitch\":%u,\"pat\":\"%s\",\"min_us\":%u,\"max_us\":%u}",
                (s->ok != 0U) ? "ok" : "fail",
                s->text,
                (unsigned int)s->ok,
                (unsigned int)s->edges,
                (unsigned int)s->f_ratio,
                (unsigned int)s->f_wide,
                (unsigned int)s->f_nomatch,
                (unsigned int)s->f_glitch,
                pat,
                (s->min_10us == 0xFFFFU) ? 0U
                                         : (unsigned int)s->min_10us * 10U,
                (unsigned int)s->max_10us * 10U);
            return send("bcscan", buf);
        }

        
        case RC_EVT_SPIN_EDGES:
        {
            /* {"face":277,"n":45,"l0":"229/203","l1":"193/215",
             *  "r1":"-287/222","r2":"-"}: each pass's edge as heading
             * (0.1 deg, > 0 left) / range (mm), "-" when not found */
            static const char *const key[4] = { "l0", "l1", "r1", "r2" };
            const rc_pl_spin_t *sp = &evt->u.spin;
            char     pass[4][16];
            uint32_t i;

            for (i = 0U; i < 4U; i++)
            {
                if ((sp->found & (1U << i)) != 0U)
                {
                    (void)rc_snprintf(pass[i], sizeof(pass[i]), "%d/%u",
                                      (int)sp->ddeg[i],
                                      (unsigned int)sp->r_2mm[i] * 2U);
                }
                else
                {
                    (void)rc_snprintf(pass[i], sizeof(pass[i]), "-");
                }
            }
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"face\":%u,\"n\":%u,\"%s\":\"%s\",\"%s\":\"%s\","
                "\"%s\":\"%s\",\"%s\":\"%s\"}",
                (unsigned int)sp->face_mm, (unsigned int)sp->n_points,
                key[0], pass[0], key[1], pass[1],
                key[2], pass[2], key[3], pass[3]);
            return send("spin", buf);
        }

        case RC_EVT_BARCODE_RAW:
        {
            /* "010010100 0 011010000 0 010010100": 1 = wide, 0 = narrow,
             * in the order the sensor met them. A Code 39 character is 9
             * elements and then a narrow gap, so after every 9 a space,
             * the gap's bit, and a space. Lines up with the characters
             * when the first element is the first bar of the start '*'. */
            const rc_pl_bar_raw_t *r = &evt->u.bar_raw;
            char     bits[RC_BAR_RAW_MAX + (2U * (RC_BAR_RAW_MAX / 10U)) + 3U];
            uint32_t i;
            uint32_t k = 0U;

            for (i = 0U; i < (uint32_t)r->n; i++)
            {
                if ((i % 10U) == 9U)
                {
                    bits[k++] = ' ';
                }
                bits[k++] = ((r->bits[i / 8U] & (1U << (i % 8U))) != 0U)
                            ? '1' : '0';
                if (((i % 10U) == 9U) && ((i + 1U) < (uint32_t)r->n))
                {
                    bits[k++] = ' ';
                }
            }
            bits[k] = '\0';
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"n\":%u,\"first\":\"%s\",\"bits\":\"%s\","
                "\"nar_us\":%u,\"wid_us\":%u,\"more\":%u}",
                (unsigned int)r->n,
                (r->first_bar != 0U) ? "bar" : "space",
                bits,
                (unsigned int)r->nar_10us * 10U,
                (unsigned int)r->wid_10us * 10U,
                (unsigned int)r->more);
            return send("bcbits", buf);
        }

        case RC_EVT_HUMP_END:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"peak_mm\":%u,\"ms\":%lu,\"pitch\":%d}",
                (unsigned int)evt->u.hump.peak_mm,
                (unsigned long)evt->u.hump.duration_ms,
                (int)evt->u.hump.max_pitch_deg);
            return send("hump", buf);

        case RC_EVT_OBSTACLE_PROFILE:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"n\":%u,\"near_mm\":%u,\"at_deg\":%d,\"w_mm\":%u,"
                "\"cl_l\":%u,\"cl_r\":%u}",
                (unsigned int)evt->u.profile.n_points,
                (unsigned int)evt->u.profile.closest_mm,
                (int)evt->u.profile.closest_angle_deg,
                (unsigned int)evt->u.profile.width_mm,
                (unsigned int)evt->u.profile.clearance_left_mm,
                (unsigned int)evt->u.profile.clearance_right_mm);
            return send("obstacle", buf);

        case RC_EVT_AVOIDANCE_PLAN:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"act\":%d,\"lat_mm\":%u,\"fwd_mm\":%u}",
                (int)evt->u.plan.action,
                (unsigned int)evt->u.plan.lateral_mm,
                (unsigned int)evt->u.plan.forward_mm);
            return send("avoid", buf);

        case RC_EVT_IMPACT:
            return send("impact", "{\"impact\":1}");

        case RC_EVT_MOTION_CLASS:
            (void)rc_snprintf(buf, sizeof(buf), "{\"cls\":%d}",
                              (int)evt->u.motion_class.cls);
            return send("motion", buf);

        case RC_EVT_LINE_LOST:
            return send("line", "{\"lost\":1}");

        case RC_EVT_LINE_REACQUIRED:
            return send("line", "{\"lost\":0}");

        case RC_EVT_MOTION_DONE:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"id\":%lu,\"ok\":%d,\"mm\":%lu}",
                (unsigned long)evt->u.motion_done.move_id,
                evt->u.motion_done.completed ? 1 : 0,
                (unsigned long)evt->u.motion_done.travelled_mm);
            return send("move", buf);

        default:
            return RC_ERR_PARAM;
    }
}

/* --------------------------------------------------------------------
 * The link (design section 6.5)
 * -------------------------------------------------------------------- */

static void link_read(rc_pl_link_t *out)
{
#if APP_TEST
    if (link_fake_on)
    {
        *out = link_fake;
        return;
    }
#endif
    sub_telemetry_mqtt_link(&out->connected, &out->broker_up);
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
    link_fake.connected = connected;
    link_fake.broker_up = broker_up;
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
    int32_t cls = st_motion_cls;

    if (cls == sent_cls)
    {
        return;
    }

    evt.id = RC_EVT_MOTION_CLASS;
    evt.u.motion_class.cls = (rc_motion_class_t)cls;
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

        publish_state();
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
 * car/01/ack: {"rx":"L","cmd":1}, or "cmd":0 if it was ignored. A quote,
 * a backslash or a control byte would break the JSON, so those show as
 * '?'. */
rc_nav_cmd_t sub_telemetry_receive_command(char c)
{
    char buf[ACK_MAX];
    char shown = c;
    rc_nav_cmd_t cmd = sub_telemetry_parse_cmd(c);

    count_inc(&rx_cmds);
    if (cmd != RC_CMD_NONE)
    {
        (void)sub_telemetry_deliver_command(cmd, 0);
    }

    if ((shown <= ' ') || (shown > '~') || (shown == '"') || (shown == '\\'))
    {
        shown = '?';
    }
    (void)rc_snprintf(buf, sizeof(buf), "{\"rx\":\"%c\",\"cmd\":%d}",
                      shown, (int)cmd);
    (void)send("ack", buf);

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
    evt.u.command.command = cmd;
    evt.u.command.arg = arg;
    res = rc_event_publish(&evt);

    if (cmd_cb != NULL)
    {
        cmd_cb(cmd, arg, cmd_ctx);
    }
    return res;
}
