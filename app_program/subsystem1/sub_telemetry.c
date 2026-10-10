/*
 * sub_telemetry.c  --  Buddy 1 / subsystem 1
 *
 * Builds the car's telemetry messages and ships them through whatever sink
 * is installed (console by default, MQTT in a WiFi build). Also hands any
 * inbound command off to sub_nav.
 *
 * Template migration, in short:
 *   - includes reach one folder up for the shared rc_* headers;
 *   - every field we report comes off the event bus and is cached here, so
 *     we don't reach into other buddies' folders -- the one exception is
 *     drv_motor_get(), which Buddy 2 publishes in drv_motor.h on purpose
 *     for telemetry (there is no motor-duty event to subscribe to);
 *   - init() only makes objects; start_task() starts the task (priority 12);
 *   - the task drains inbound commands each tick;
 *   - the tx counters are locked because two tasks touch them.
 * The framing, topics, 250 ms schedule and heartbeat are unchanged.
 */
#include "../rc_prelude.h"
#include <tm/tmonitor.h>

#include "sub_telemetry.h"
#include "subsystem1.h"             /* S1_PRI_TELEMETRY, app_start_task() */
#include "../rc_fmt.h"              /* rc_snprintf(), rc_strlen() */
#include "../rc_config.h"
#include "../rc_event.h"
#include "../rc_time.h"
#include "../subsystem2/drv_motor.h"    /* drv_motor_get(): last duty, Buddy 2 */

/* Topic buffer holds "car/01/<leaf>"; payload buffer holds one JSON body.
 * Bump PAYLOAD_MAX first if a message ever looks cut off. */
#define TOPIC_MAX       (48)
#define PAYLOAD_MAX     (192)

/* One prefix for every topic (rc_config.h), so the publish topics and the
 * command topic can't drift apart. */
#define TOPIC_BASE      RC_TOPIC_BASE

/* The transport we're currently sending through. */
static const sub_telemetry_sink_t *sink;

static uint32_t seq;            /* stamped into each state message */
static uint32_t tx_count;       /* messages sent OK (in the heartbeat) */
static uint32_t tx_fail;        /* messages that failed (in the heartbeat) */

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
static volatile char     st_last_bc;        /* most recent barcode symbol */

/* --------------------------------------------------------------------
 * Counter lock (dual-core)
 *
 * send() runs on two tasks -- the telemetry task and the slow event
 * dispatcher (via on_notable) -- so a bare tx_count++ can drop a count
 * when both cores hit it at once. Same lock pattern as rc_event.c:
 * a spinlock on SMP, interrupt masking on single core. NOT a mutex: the
 * dispatcher is never allowed to wait.
 * -------------------------------------------------------------------- */
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK tx_lock;
/* NOTE: ISpinLock wants a UINT*, so sts must be UINT, not uint32_t. */
#define TX_LOCK(sts)      ((void)ISpinLock(&tx_lock, &(sts)))
#define TX_UNLOCK(sts)    ((void)ISpinUnlock(&tx_lock, (sts)))
#else
#define TX_LOCK(sts)      DI(sts)
#define TX_UNLOCK(sts)    EI(sts)
#endif

static void tx_inc(uint32_t *ctr)
{
    UINT sts;

    TX_LOCK(sts);
    (*ctr)++;
    TX_UNLOCK(sts);
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
        tx_inc(&tx_fail);
        return RC_ERR_STATE;
    }

    (void)rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf);

    res = sink->publish(topic, payload, (uint16_t)rc_strlen(payload));
    if (res == RC_OK)
    {
        tx_inc(&tx_count);
    }
    else
    {
        tx_inc(&tx_fail);
    }
    return res;
}

/* The per-tick status line: speed, distance, motor duty, line sensors,
 * motion class, tallest hump, last barcode. Every field is a cached value
 * updated by the event callbacks above, except the motor duty, which
 * Buddy 2 reports live through drv_motor_get(). */
static void publish_state(void)
{
    char buf[PAYLOAD_MAX];

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"t\":%lu,\"spd_l\":%ld,\"spd_r\":%ld,"
        "\"dist\":%lu,\"m_l\":%d,\"m_r\":%d,\"line\":\"%c%c\","
        "\"cls\":%d,\"peak\":%u,\"bc\":\"%c\"}",
        (unsigned long)seq,
        (unsigned long)rc_time_ms(),
        (long)st_speed_l,
        (long)st_speed_r,
        (unsigned long)st_dist_mm,
        (int)drv_motor_get(RC_SIDE_LEFT),
        (int)drv_motor_get(RC_SIDE_RIGHT),
        st_line_l ? '1' : '0',
        st_line_r ? '1' : '0',
        (int)st_motion_cls,
        (unsigned int)st_peak_mm,
        (st_last_bc != '\0') ? st_last_bc : '-');

    seq++;
    (void)send("state", buf);
}

/* The slower "is the link healthy" line: uptime, send tallies, event-bus
 * drops, and which sink is active. A non-zero drop count is a real warning. */
static void publish_heartbeat(void)
{
    char buf[PAYLOAD_MAX];

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"up\":%lu,\"tx\":%lu,\"fail\":%lu,"
        "\"drop_fast\":%lu,\"drop_slow\":%lu,\"sink\":\"%s\"}",
        (unsigned long)rc_time_ms(),
        (unsigned long)tx_count,
        (unsigned long)tx_fail,
        (unsigned long)rc_event_dropped(RC_LANE_FAST),
        (unsigned long)rc_event_dropped(RC_LANE_SLOW),
        (sink != NULL) ? sink->name : "none");

    (void)send("status", buf);
}

/* --------------------------------------------------------------------
 * Event callbacks. All on the slow lane, so telemetry never jumps ahead
 * of the control path. The cache-and-return ones stay short on purpose --
 * the event bus calls them directly.
 * -------------------------------------------------------------------- */

static void on_odometry(const rc_event_t *evt, void *ctx)
{
    uint32_t l = evt->u.odometry.dist_l_mm;
    uint32_t r = evt->u.odometry.dist_r_mm;

    (void)ctx;
    st_speed_l = evt->u.odometry.speed_l_mm_s;
    st_speed_r = evt->u.odometry.speed_r_mm_s;

    /* Average of the two wheels. Halve each one first: (l + r) / 2 would
     * overflow 32 bits once the sum passes ~4,295 km. */
    st_dist_mm = (l / 2U) + (r / 2U) + (l & r & 1U);
}

static void on_line(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_line_l = evt->u.line.on_line_l;
    st_line_r = evt->u.line.on_line_r;
}

/* Terrain reports the motion class on its own event; cache it for the
 * per-tick line, the same way odometry and the line sensors are cached. */
static void on_motion_class(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_motion_cls = (int32_t)evt->u.motion_class.cls;
}

/* Barcode / hump / obstacle / impact: send straight away rather than wait
 * for the next tick. Two of them also feed the per-tick line, so cache the
 * fields it shows on the way past. */
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

    (void)sub_telemetry_publish_event(evt);
}

/* Turn one "noteworthy" event into its own message on its own topic. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    char buf[PAYLOAD_MAX];

    switch (evt->id)
    {
        case RC_EVT_BARCODE_DECODED:
            (void)rc_snprintf(buf, sizeof(buf),
                "{\"sym\":\"%c\",\"cmd\":%d,\"rev\":%d}",
                evt->u.barcode.symbol,
                (int)evt->u.barcode.command,
                evt->u.barcode.reversed ? 1 : 0);
            return send("barcode", buf);

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

        case RC_EVT_IMPACT:
            return send("impact", "{\"impact\":1}");

        default:
            return RC_ERR_PARAM;
    }
}

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
        tk_dly_tsk(RC_PERIOD_TELEM_MS);

#if TM_MQTT
        /* Pick up any commands the kernel forwarded and hand them to nav. */
        sub_telemetry_mqtt_poll();
#endif

        /* Skip this tick if the link is down -- no point hammering a dead
         * sink; send() would just count another failure. */
        if (!telemetry_link_ok())
        {
            continue;
        }

        publish_state();

        ticks++;
        if ((ticks % 8U) == 0U)         /* heartbeat every ~2 s */
        {
            publish_heartbeat();
        }
    }
}

/* --------------------------------------------------------------------
 * Start-up (two passes: init then start).
 * -------------------------------------------------------------------- */

/* Pass one: subscribe to the events we report on and, in a WiFi build,
 * make the message buffers and switch the sink to MQTT. No task yet. */
rc_result_t sub_telemetry_init(void)
{
    sink = &console_sink;

    (void)rc_event_subscribe(RC_EVT_ODOMETRY, RC_LANE_SLOW, on_odometry, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW, on_line, NULL);
    (void)rc_event_subscribe(RC_EVT_MOTION_CLASS, RC_LANE_SLOW,
                             on_motion_class, NULL);
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_OBSTACLE_PROFILE, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_SLOW, on_notable, NULL);

#if TM_MQTT
    if (sub_telemetry_mqtt_init() == RC_OK)
    {
        sink = sub_telemetry_mqtt_sink();
    }
#endif

    return RC_OK;
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

rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx)
{
    cmd_cb = cb;
    cmd_ctx = ctx;
    return RC_OK;
}

/* One command character -> a driving command. Takes the mnemonic letters
 * (L/R/S/U/X) and the barcode letters (A/B/C/D), either case; P also means
 * stop. Anything else is RC_CMD_NONE and gets ignored. Lives here rather
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
        default:                                return RC_CMD_NONE;
    }
}

/* Called once a command has been decoded. Prefer the registered callback;
 * otherwise drop it on the event bus for sub_nav to pick up on the fast
 * lane. Either way we don't touch nav state from here. */
rc_result_t sub_telemetry_deliver_command(rc_nav_cmd_t cmd, int32_t arg)
{
    rc_event_t evt;

    if (cmd_cb != NULL)
    {
        cmd_cb(cmd, arg, cmd_ctx);
        return RC_OK;
    }

    evt.id = RC_EVT_COMMAND_RX;
    evt.u.command.command = cmd;
    evt.u.command.arg = arg;
    return rc_event_publish(&evt);
}
