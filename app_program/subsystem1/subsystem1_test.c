/*
 * subsystem1_test.c
 * On-target tests for subsystem 1 (Buddy 1: telemetry + commands).
 * Built only with: make TEST=1   (add NET=lwip MQTT=1 for the S1-W tests)
 *
 * Buddy 1 sits in the middle of everyone else: it listens to the other
 * buddies' events and reports them, and it takes commands in for sub_nav.
 * So these tests don't need the other buddies' code to be finished. Where
 * Buddy 1 depends on someone, the test plays that buddy itself: it
 * publishes the exact event that buddy would publish, on the real event
 * bus, and checks Buddy 1 does the right thing with it.
 *
 *   S1-U01..U22  unit: sinks, command decoding, framing, timing, the link
 *   S1-I01..I12  integration: event-bus contracts with Buddies 2-5 + nav
 *   S1-W01..W09  WiFi/MQTT: only in an MQTT=1 build, SKIP otherwise
 *
 * What each Buddy 1 task in the project brief is proven by:
 *
 *   Establish WiFi connectivity        W04
 *   Implement MQTT communication       W01 W02 W06 W09
 *   Design communication topics        U17 W03
 *   Define telemetry message structs   U11-U14 U22 I10
 *   Implement telemetry publishing     U15 U21 I02-I08
 *   Implement command subscription     U03-U06 I09 W07 W08 W09
 *   Implement connection recovery      U18 U19 U20
 *   Implement heartbeat/status         U16 I11
 *   Telemetry API is safe to run       U01 U02 U07-U10 I12
 *
 * Every check prints one "[TEST] <id> PASS/FAIL" line. Save the console to
 * testing/subsystem1/logs/ and record the run in testing/subsystem1/
 * TEST_LOG.md (the IDs here match that file).
 *
 * NOTE: the integration tests need the event bus running (rc_event_init()
 * called before subsystem1_init). If S1-I01 fails, the bus isn't up and
 * the rest of the S1-I tests will fail for the same reason.
 *
 * NOTE: the tests leave their values in telemetry's cache (e.g. the last
 * barcode reads 'Z' afterwards). Fine for a TEST image; don't ship one.
 */

#include "../rc_prelude.h"
#include <string.h>
#include <tm/tmonitor.h>

#include "subsystem1.h"
#include "sub_telemetry.h"
#include "../rc_config.h"
#include "../rc_event.h"
#include "../test_report.h"

#if TM_MQTT
#include "wifi_secrets.h"           /* MQTT_TOPIC: what the kernel subscribes to */
#endif

#if APP_TEST

/* Stops the compiler moving buffer copies across the seq updates below. */
#define BARRIER()           __asm volatile ("" ::: "memory")

#define CAP_TOPIC_MAX       (48)
#define CAP_PAYLOAD_MAX     (240)
#define STATE_PAYLOAD_MAX   (224)   /* PAYLOAD_MAX in sub_telemetry.c */

/* --------------------------------------------------------------------
 * Capture sink
 *
 * A fake transport that keeps the last message it was given, so a test
 * can read back exactly what telemetry would have put on the wire. State,
 * status and event messages go into separate slots because the telemetry
 * task keeps writing state/status in the background while a test runs.
 *
 * seq is odd while a slot is being written. A reader copies the slot and
 * only trusts the copy if seq was even and didn't change (see cap_read).
 * -------------------------------------------------------------------- */
typedef struct
{
    volatile uint32_t seq;
    char topic[CAP_TOPIC_MAX];
    char payload[CAP_PAYLOAD_MAX];
} cap_slot_t;

static cap_slot_t cap_state;
static cap_slot_t cap_status;
static cap_slot_t cap_event;

static volatile bool     cap_up = true;     /* flip to fake a dead link */
static volatile uint32_t cap_opens;
static volatile uint32_t cap_publishes;

/* Scratch copies for the checks. Static, not on the stack: the initial
 * task that runs the tests doesn't have much stack to spare. */
static char snap_topic[CAP_TOPIC_MAX];
static char snap_payload[CAP_PAYLOAD_MAX];

static bool ends_with(const char *s, const char *tail)
{
    size_t ls = strlen(s);
    size_t lt = strlen(tail);

    return (ls >= lt) && (strcmp(s + (ls - lt), tail) == 0);
}

static rc_result_t cap_open(void)
{
    cap_opens++;
    return RC_OK;
}

static rc_result_t cap_close(void)
{
    return RC_OK;
}

static bool cap_is_up(void)
{
    return cap_up;
}

static rc_result_t cap_publish(const char *topic, const char *payload,
                               uint16_t len)
{
    cap_slot_t *s;
    uint32_t n = len;

    if (ends_with(topic, "/state"))
    {
        s = &cap_state;
    }
    else if (ends_with(topic, "/status"))
    {
        s = &cap_status;
    }
    else
    {
        s = &cap_event;
    }

    if (n >= CAP_PAYLOAD_MAX)
    {
        n = CAP_PAYLOAD_MAX - 1U;
    }

    s->seq++;                                   /* odd: writing */
    BARRIER();
    (void)strncpy(s->topic, topic, CAP_TOPIC_MAX - 1U);
    s->topic[CAP_TOPIC_MAX - 1U] = '\0';
    (void)memcpy(s->payload, payload, n);
    s->payload[n] = '\0';
    BARRIER();
    s->seq++;                                   /* even: done */

    cap_publishes++;
    return RC_OK;
}

static const sub_telemetry_sink_t cap_sink =
{
    "capture", cap_open, cap_publish, cap_close, cap_is_up
};

/* How many complete messages a slot has seen. */
static uint32_t cap_count(const cap_slot_t *s)
{
    return s->seq / 2U;
}

/* Copy a slot into snap_topic/snap_payload without tearing. If the writer
 * got in while we were copying, give it a tick to finish and try again. */
static void cap_read(const cap_slot_t *s)
{
    uint32_t before;

    for (;;)
    {
        before = s->seq;
        if ((before & 1U) == 0U)
        {
            BARRIER();
            (void)memcpy(snap_topic, s->topic, CAP_TOPIC_MAX);
            (void)memcpy(snap_payload, s->payload, CAP_PAYLOAD_MAX);
            BARRIER();
            if (s->seq == before)
            {
                return;
            }
        }
        tk_dly_tsk(1);
    }
}

/* Wait until a slot has seen more than `count` messages. */
static bool cap_wait(const cap_slot_t *s, uint32_t count, INT timeout_ms)
{
    INT waited = 0;

    while (cap_count(s) <= count)
    {
        if (waited >= timeout_ms)
        {
            return false;
        }
        tk_dly_tsk(10);
        waited += 10;
    }
    return true;
}

/* Grab a state message that was built after this call. Waits for two new
 * ones, not one: the first may have been half-built when we asked, so it
 * could still hold the old cached values. */
static bool fresh_state(void)
{
    uint32_t c = cap_count(&cap_state);

    if (!cap_wait(&cap_state, c + 1U, 1500))
    {
        return false;
    }
    cap_read(&cap_state);
    return true;
}

/* Send one event straight through telemetry (no bus) and read back the
 * message it produced. */
static bool frame_event(const rc_event_t *evt)
{
    uint32_t c = cap_count(&cap_event);

    if (sub_telemetry_publish_event(evt) != RC_OK)
    {
        return false;
    }
    if (!cap_wait(&cap_event, c, 200))
    {
        return false;
    }
    cap_read(&cap_event);
    return true;
}

/* Publish an event on the real bus, the way another buddy would, and read
 * back the message telemetry sent for it. */
static bool bus_event(rc_event_t *evt)
{
    uint32_t c = cap_count(&cap_event);

    if (rc_event_publish(evt) != RC_OK)
    {
        return false;
    }
    if (!cap_wait(&cap_event, c, 1000))
    {
        return false;
    }
    cap_read(&cap_event);
    return true;
}

static bool snap_is(const char *topic, const char *payload)
{
    return (strcmp(snap_topic, topic) == 0) &&
           (strcmp(snap_payload, payload) == 0);
}

static bool snap_has(const char *needle)
{
    return strstr(snap_payload, needle) != NULL;
}

/* Read the unsigned number after `key` in the JSON in snap_payload, e.g.
 * key "\"seq\":". Returns 0xFFFFFFFF if the key isn't there. */
static uint32_t snap_u32(const char *key)
{
    const char *p = strstr(snap_payload, key);
    uint32_t v = 0U;

    if (p == NULL)
    {
        return 0xFFFFFFFFU;
    }
    for (p += strlen(key); (*p >= '0') && (*p <= '9'); p++)
    {
        v = (v * 10U) + (uint32_t)(*p - '0');
    }
    return v;
}

/* --------------------------------------------------------------------
 * Listeners (for the bus tests)
 *
 * Subscribed once and left in place. rc_event_unsubscribe() isn't safe to
 * call while the dispatcher on the other core is running: it clears
 * in_use and cb as two separate writes, so the dispatcher can see the
 * slot still in use and call a NULL callback.
 * -------------------------------------------------------------------- */
static volatile uint32_t     probe_hits;
static volatile uint32_t     cmdrx_hits;
static volatile rc_nav_cmd_t cmdrx_cmd;
static volatile int32_t      cmdrx_arg;
static bool                  listeners_on;

static void on_probe(const rc_event_t *evt, void *ctx)
{
    (void)evt;
    (void)ctx;
    probe_hits++;
}

/* Stands in for sub_nav, which is who really listens for commands. */
static void on_cmdrx(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    cmdrx_cmd = evt->u.command.command;
    cmdrx_arg = evt->u.command.arg;
    cmdrx_hits++;
}

static void listeners_attach(void)
{
    if (!listeners_on)
    {
        /* RC_EVT_LINK_STATE is Buddy 1's own id and nobody else uses it,
         * so it's a safe event to probe the bus with. */
        (void)rc_event_subscribe(RC_EVT_LINK_STATE, RC_LANE_SLOW,
                                 on_probe, NULL);
        (void)rc_event_subscribe(RC_EVT_COMMAND_RX, RC_LANE_SLOW,
                                 on_cmdrx, NULL);
        listeners_on = true;
    }
}

static bool wait_hits(volatile uint32_t *hits, uint32_t want, INT timeout_ms)
{
    INT waited = 0;

    while (*hits < want)
    {
        if (waited >= timeout_ms)
        {
            return false;
        }
        tk_dly_tsk(10);
        waited += 10;
    }
    return true;
}

/* --------------------------------------------------------------------
 * Command callback (records what deliver_command() handed over)
 * -------------------------------------------------------------------- */
static volatile rc_nav_cmd_t cb_cmd;
static volatile int32_t      cb_arg;
static volatile uint32_t     cb_hits;

static void test_cmd_cb(rc_nav_cmd_t cmd, int32_t arg, void *ctx)
{
    (void)ctx;
    cb_cmd = cmd;
    cb_arg = arg;
    cb_hits++;
}

/* --------------------------------------------------------------------
 * Unit checks
 * -------------------------------------------------------------------- */

typedef struct
{
    char         c;
    rc_nav_cmd_t want;
} cmd_case_t;

static bool decodes(const cmd_case_t *cases, uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        if (sub_telemetry_parse_cmd(cases[i].c) != cases[i].want)
        {
            tm_printf((UB *)"[TEST]   '%c' decoded wrong\n", cases[i].c);
            return false;
        }
    }
    return true;
}

static bool check_mnemonics(void)
{
    static const cmd_case_t cases[] =
    {
        {'L', RC_CMD_TURN_LEFT},   {'l', RC_CMD_TURN_LEFT},
        {'R', RC_CMD_TURN_RIGHT},  {'r', RC_CMD_TURN_RIGHT},
        {'S', RC_CMD_GO_STRAIGHT}, {'s', RC_CMD_GO_STRAIGHT},
        {'U', RC_CMD_U_TURN},      {'u', RC_CMD_U_TURN},
        {'X', RC_CMD_STOP},        {'x', RC_CMD_STOP}
    };

    return decodes(cases, sizeof(cases) / sizeof(cases[0]));
}

/* The barcode alphabet: the same letters Buddy 3 decodes off the track. */
static bool check_barcode_letters(void)
{
    static const cmd_case_t cases[] =
    {
        {'A', RC_CMD_TURN_LEFT},   {'a', RC_CMD_TURN_LEFT},
        {'B', RC_CMD_TURN_RIGHT},  {'b', RC_CMD_TURN_RIGHT},
        {'C', RC_CMD_GO_STRAIGHT}, {'c', RC_CMD_GO_STRAIGHT},
        {'D', RC_CMD_U_TURN},      {'d', RC_CMD_U_TURN},
        {'P', RC_CMD_STOP},        {'p', RC_CMD_STOP}
    };

    return decodes(cases, sizeof(cases) / sizeof(cases[0]));
}

static bool check_garbage_ignored(void)
{
    static const cmd_case_t cases[] =
    {
        {'Z', RC_CMD_NONE}, {'0', RC_CMD_NONE}, {' ', RC_CMD_NONE},
        {'\0', RC_CMD_NONE}, {'{', RC_CMD_NONE}, {'E', RC_CMD_NONE}
    };

    return decodes(cases, sizeof(cases) / sizeof(cases[0]));
}

static bool check_callback_routing(void)
{
    bool ok;

    cb_hits = 0U;
    (void)sub_telemetry_on_command(test_cmd_cb, NULL);
    (void)sub_telemetry_deliver_command(RC_CMD_U_TURN, 42);
    ok = (cb_hits == 1U) && (cb_cmd == RC_CMD_U_TURN) && (cb_arg == 42);
    (void)sub_telemetry_on_command(NULL, NULL);
    return ok;
}

static bool check_task_priority(void)
{
    T_RTSK rtsk;

    return (sub_telemetry_task_id() > 0) &&
           (tk_ref_tsk((ID)sub_telemetry_task_id(), &rtsk) == E_OK) &&
           (rtsk.tskpri == S1_PRI_TELEMETRY);
}

/* Look at the task 10 times over ~1 s. Between ticks it should be asleep
 * in tk_dly_tsk, not spinning. */
static bool check_task_sleeps(void)
{
    T_RTSK rtsk;
    INT i;
    INT waiting = 0;

    for (i = 0; i < 10; i++)
    {
        tk_dly_tsk(97);                         /* not a multiple of 250 */
        if ((tk_ref_tsk((ID)sub_telemetry_task_id(), &rtsk) == E_OK) &&
            ((rtsk.tskstat & TTS_WAI) != 0U))
        {
            waiting++;
        }
    }
    return waiting >= 8;
}

/* --------------------------------------------------------------------
 * Framing checks (direct, through the capture sink)
 * -------------------------------------------------------------------- */

static void make_barcode(rc_event_t *e, char sym, rc_nav_cmd_t cmd, bool rev)
{
    (void)memset(e, 0, sizeof(*e));
    e->id = RC_EVT_BARCODE_DECODED;
    e->u.barcode.symbol   = sym;
    e->u.barcode.command  = cmd;
    e->u.barcode.reversed = rev;
}

static void make_hump(rc_event_t *e, uint16_t peak, uint32_t ms, int16_t p)
{
    (void)memset(e, 0, sizeof(*e));
    e->id = RC_EVT_HUMP_END;
    e->u.hump.peak_mm       = peak;
    e->u.hump.duration_ms   = ms;
    e->u.hump.max_pitch_deg = p;
}

static void make_obstacle(rc_event_t *e, uint8_t n, uint16_t near_mm,
                          int16_t deg, uint16_t w, uint16_t cl_l,
                          uint16_t cl_r)
{
    (void)memset(e, 0, sizeof(*e));
    e->id = RC_EVT_OBSTACLE_PROFILE;
    e->u.profile.n_points           = n;
    e->u.profile.closest_mm         = near_mm;
    e->u.profile.closest_angle_deg  = deg;
    e->u.profile.width_mm           = w;
    e->u.profile.clearance_left_mm  = cl_l;
    e->u.profile.clearance_right_mm = cl_r;
}

static void make_simple(rc_event_t *e, rc_evt_id_t id)
{
    (void)memset(e, 0, sizeof(*e));
    e->id = id;
}

static bool check_events_accepted(void)
{
    rc_event_t e;
    bool ok = true;

    make_barcode(&e, 'A', RC_CMD_TURN_LEFT, false);
    ok = ok && (sub_telemetry_publish_event(&e) == RC_OK);
    make_hump(&e, 37U, 420U, 12);
    ok = ok && (sub_telemetry_publish_event(&e) == RC_OK);
    make_obstacle(&e, 7U, 250U, -15, 120U, 300U, 80U);
    ok = ok && (sub_telemetry_publish_event(&e) == RC_OK);
    make_simple(&e, RC_EVT_IMPACT);
    ok = ok && (sub_telemetry_publish_event(&e) == RC_OK);
    return ok;
}

static bool check_unrelated_rejected(void)
{
    rc_event_t e;
    uint32_t c = cap_count(&cap_event);

    make_simple(&e, RC_EVT_TICK);
    return (sub_telemetry_publish_event(&e) == RC_ERR_PARAM) &&
           (cap_count(&cap_event) == c);
}

static bool check_barcode_frame(void)
{
    rc_event_t e;

    make_barcode(&e, 'A', RC_CMD_TURN_LEFT, false);
    return frame_event(&e) &&
           snap_is(RC_TOPIC_BASE "/barcode", "{\"sym\":\"A\",\"cmd\":1,\"rev\":0}");
}

static bool check_hump_frame(void)
{
    rc_event_t e;

    make_hump(&e, 37U, 420U, 12);
    return frame_event(&e) &&
           snap_is(RC_TOPIC_BASE "/hump",
                   "{\"peak_mm\":37,\"ms\":420,\"pitch\":12}");
}

static bool check_obstacle_frame(void)
{
    rc_event_t e;

    make_obstacle(&e, 7U, 250U, -15, 120U, 300U, 80U);
    return frame_event(&e) &&
           snap_is(RC_TOPIC_BASE "/obstacle",
                   "{\"n\":7,\"near_mm\":250,\"at_deg\":-15,\"w_mm\":120,"
                   "\"cl_l\":300,\"cl_r\":80}");
}

static bool check_impact_frame(void)
{
    rc_event_t e;

    make_simple(&e, RC_EVT_IMPACT);
    return frame_event(&e) &&
           snap_is(RC_TOPIC_BASE "/impact", "{\"impact\":1}");
}

/* One measurement, used by S1-U15 and S1-U21: eight state messages timed
 * two ways. The kernel clock (tk_get_tim) is the reference; the "t" field
 * is what telemetry stamps into each message (rc_time_ms). Keeping them
 * separate means a wrong clock can't hide behind a correct schedule. */
#define SCHED_MSGS      (8U)

static bool     sched_ok;
static uint32_t sched_msgs;         /* messages that arrived */
static uint32_t sched_seq;          /* how far "seq" moved over them */
static uint32_t sched_kernel_ms;    /* the span, by the kernel clock */
static uint32_t sched_car_ms;       /* the span, by telemetry's own "t" */

static void measure_schedule(void)
{
    SYSTIM   t0;
    SYSTIM   t1;
    uint32_t n_a;
    uint32_t seq_a;
    uint32_t car_a;

    sched_ok = false;
    if (!fresh_state())
    {
        return;
    }
    (void)tk_get_tim(&t0);
    n_a   = cap_count(&cap_state);
    seq_a = snap_u32("\"seq\":");
    car_a = snap_u32("\"t\":");

    if (!cap_wait(&cap_state, n_a + SCHED_MSGS - 1U, 4000))
    {
        return;
    }
    (void)tk_get_tim(&t1);
    cap_read(&cap_state);

    sched_msgs      = cap_count(&cap_state) - n_a;
    sched_seq       = snap_u32("\"seq\":") - seq_a;
    sched_kernel_ms = t1.lo - t0.lo;
    sched_car_ms    = snap_u32("\"t\":") - car_a;
    sched_ok        = (sched_seq > 0U);

    tm_printf((UB *)"[TEST]   %lu state msgs: %lu ms by kernel clock, "
              "%lu ms by telemetry's t\n", (unsigned long)sched_msgs,
              (unsigned long)sched_kernel_ms, (unsigned long)sched_car_ms);
}

/* A state message every 250 ms (rc_config.h; a few ms over is normal:
 * the work itself plus the 1 ms tick), and "seq" accounts for every one
 * of them, so none were skipped. Timed by the kernel clock. */
static bool check_state_schedule(void)
{
    uint32_t period;

    measure_schedule();
    if (!sched_ok || (sched_seq != sched_msgs))
    {
        return false;
    }
    period = sched_kernel_ms / SCHED_MSGS;
    return (period >= (uint32_t)RC_PERIOD_TELEM_MS) &&
           (period <= ((uint32_t)RC_PERIOD_TELEM_MS + 25U));
}

/* The "t" timestamps in the messages move at the same speed as real time
 * (within 5 %). A dashboard plots against "t", so a slow clock would
 * squash every graph. This is what caught the 42.7x slow TIMER after a
 * cold power-on (fixed in rc_time_init()). */
static bool check_timestamps_real_time(void)
{
    uint32_t car_per;
    uint32_t ker_per;
    uint32_t diff;

    if (!sched_ok)
    {
        return false;
    }
    car_per = sched_car_ms / sched_seq;
    ker_per = sched_kernel_ms / SCHED_MSGS;
    diff    = (car_per > ker_per) ? (car_per - ker_per) : (ker_per - car_per);
    return (diff * 20U) <= ker_per;
}

/* Every telemetry item the project brief lists has a field in the state
 * message: state (cls), speeds, encoder counts, motor duty, line sensors,
 * barcode, hump peak, distance -- plus seq and t to order them. */
static bool check_state_fields(void)
{
    static const char *const keys[] =
    {
        "\"seq\":", "\"t\":", "\"spd_l\":", "\"spd_r\":", "\"dist\":",
        "\"enc_l\":", "\"enc_r\":", "\"m_l\":", "\"m_r\":", "\"line\":",
        "\"cls\":", "\"peak\":", "\"bc\":"
    };
    uint32_t i;

    cap_read(&cap_state);
    for (i = 0U; i < (sizeof(keys) / sizeof(keys[0])); i++)
    {
        if (!snap_has(keys[i]))
        {
            tm_printf((UB *)"[TEST]   state has no %s\n", keys[i]);
            return false;
        }
    }
    return true;
}

/* Heartbeat comes every 8 ticks (~2 s) and says which sink is active. */
static bool check_heartbeat(void)
{
    uint32_t c = cap_count(&cap_status);

    if (!cap_wait(&cap_status, c, 3000))
    {
        return false;
    }
    cap_read(&cap_status);
    return snap_has("\"sink\":\"capture\"") && snap_has("\"tx\":") &&
           snap_has("\"fail\":") && snap_has("\"drop_fast\":") &&
           snap_has("\"drop_slow\":");
}

/* Every topic hangs off RC_TOPIC_BASE, so the broker side can subscribe to
 * car/01/# and get everything. */
static bool check_topics(void)
{
    bool ok;

    cap_read(&cap_state);
    ok = (strcmp(snap_topic, RC_TOPIC_BASE "/state") == 0);
    cap_read(&cap_status);
    ok = ok && (strcmp(snap_topic, RC_TOPIC_BASE "/status") == 0);
    cap_read(&cap_event);
    ok = ok && (strncmp(snap_topic, RC_TOPIC_BASE "/",
                        strlen(RC_TOPIC_BASE "/")) == 0);
    return ok;
}

/* --------------------------------------------------------------------
 * Integration checks (the test plays the other buddies on the bus)
 * -------------------------------------------------------------------- */

static bool check_bus_running(void)
{
    rc_event_t e;

    probe_hits = 0U;
    make_simple(&e, RC_EVT_LINK_STATE);
    (void)rc_event_publish(&e);
    return wait_hits(&probe_hits, 1U, 500);
}

/* Buddy 2 (odometry), Buddy 3 (line) and Buddy 4 (motion class) all feed
 * the per-tick state line, so publish all three and read one state back. */
static void play_sensor_buddies(int32_t spd_l, int32_t spd_r,
                                uint32_t dist_l, uint32_t dist_r,
                                bool line_l, bool line_r,
                                rc_motion_class_t cls)
{
    rc_event_t e;

    make_simple(&e, RC_EVT_ODOMETRY);
    e.u.odometry.speed_l_mm_s = spd_l;
    e.u.odometry.speed_r_mm_s = spd_r;
    e.u.odometry.dist_l_mm    = dist_l;
    e.u.odometry.dist_r_mm    = dist_r;
    (void)rc_event_publish(&e);

    make_simple(&e, RC_EVT_LINE_SAMPLE);
    e.u.line.on_line_l = line_l;
    e.u.line.on_line_r = line_r;
    (void)rc_event_publish(&e);

    make_simple(&e, RC_EVT_MOTION_CLASS);
    e.u.motion_class.cls = cls;
    (void)rc_event_publish(&e);
}

static bool check_bus_barcode(void)
{
    rc_event_t e;

    make_barcode(&e, 'C', RC_CMD_GO_STRAIGHT, true);
    if (!bus_event(&e) ||
        !snap_is(RC_TOPIC_BASE "/barcode", "{\"sym\":\"C\",\"cmd\":3,\"rev\":1}"))
    {
        return false;
    }
    return fresh_state() && snap_has("\"bc\":\"C\"");
}

static bool check_bus_hump(void)
{
    rc_event_t e;

    make_hump(&e, 55U, 300U, 9);
    if (!bus_event(&e) ||
        !snap_is(RC_TOPIC_BASE "/hump", "{\"peak_mm\":55,\"ms\":300,\"pitch\":9}"))
    {
        return false;
    }
    return fresh_state() && snap_has("\"peak\":55");
}

static bool check_bus_impact(void)
{
    rc_event_t e;

    make_simple(&e, RC_EVT_IMPACT);
    return bus_event(&e) &&
           snap_is(RC_TOPIC_BASE "/impact", "{\"impact\":1}");
}

static bool check_bus_obstacle(void)
{
    rc_event_t e;

    make_obstacle(&e, 3U, 410U, 30, 90U, 150U, 220U);
    return bus_event(&e) &&
           snap_is(RC_TOPIC_BASE "/obstacle",
                   "{\"n\":3,\"near_mm\":410,\"at_deg\":30,\"w_mm\":90,"
                   "\"cl_l\":150,\"cl_r\":220}");
}

/* With no callback registered, a command must go out on the bus as
 * RC_EVT_COMMAND_RX for sub_nav, with the command and argument intact. */
static bool check_command_to_nav(void)
{
    (void)sub_telemetry_on_command(NULL, NULL);
    cmdrx_hits = 0U;
    if (sub_telemetry_deliver_command(RC_CMD_STOP, 7) != RC_OK)
    {
        return false;
    }
    return wait_hits(&cmdrx_hits, 1U, 1000) &&
           (cmdrx_cmd == RC_CMD_STOP) && (cmdrx_arg == 7);
}

/* How many characters the value after `key` takes up in snap_payload
 * (up to the next ',' or '}'). */
static uint32_t field_len(const char *key)
{
    const char *p = strstr(snap_payload, key);
    uint32_t n = 0U;

    if (p == NULL)
    {
        return 0U;
    }
    for (p += strlen(key); (*p != ',') && (*p != '}') && (*p != '\0'); p++)
    {
        n++;
    }
    return n;
}

/* The longest state line possible. The test drives everything it can to
 * its longest: most negative speeds, 4,000 km on both wheels, both line
 * sensors on, the highest motion class, the tallest hump. Some numbers it
 * can't drive (seq, t, encoder counts, motor duty), so it adds the digits
 * those would need at their longest. The result must fit the telemetry
 * buffer and one MQTT message, and the real message must end in '}' and
 * report the right distance (no overflow in the average). */
static bool check_worst_case_state(void)
{
    rc_event_t e;
    size_t len;
    size_t worst;
    bool ok;

    make_hump(&e, 65535U, 1U, 0);
    (void)rc_event_publish(&e);
    play_sensor_buddies(INT32_MIN, INT32_MIN, 4000000000U, 4000000000U,
                        true, true, RC_MOTION_IMPACT);
    if (!fresh_state())
    {
        return false;
    }

    len   = strlen(snap_payload);
    worst = len
          + (10U - field_len("\"seq\":"))   + (10U - field_len("\"t\":"))
          + (10U - field_len("\"enc_l\":")) + (10U - field_len("\"enc_r\":"))
          + (5U - field_len("\"m_l\":"))    + (5U - field_len("\"m_r\":"));
    tm_printf((UB *)"[TEST]   state is %lu bytes now, %lu at worst "
              "(limit %d)\n", (unsigned long)len, (unsigned long)worst,
              STATE_PAYLOAD_MAX - 1);
    ok = (len > 0U) && (worst < STATE_PAYLOAD_MAX) &&
         (snap_payload[len - 1U] == '}') &&
         ((strlen(snap_topic) + 1U + worst) <= S1_MSG_MAX) &&
         snap_has("\"spd_l\":-2147483648") &&
         snap_has("\"dist\":4000000000") &&
         snap_has("\"line\":\"11\"") && snap_has("\"cls\":7") &&
         snap_has("\"peak\":65535");

    /* Put the sensors back to something sane for whatever runs next. */
    play_sensor_buddies(0, 0, 0U, 0U, false, false, RC_MOTION_STATIONARY);
    return ok;
}

static bool check_no_drops(void)
{
    return (rc_event_dropped(RC_LANE_FAST) == 0U) &&
           (rc_event_dropped(RC_LANE_SLOW) == 0U);
}

/* Somebody else calling rc_event_init() a second time (subsystem5_test.c
 * does) must not cut Buddy 1 off the bus. Re-init it on purpose, then check
 * both directions still work: an event from "Buddy 2" still reaches the
 * state message, and a command still reaches sub_nav's listener. Runs last,
 * because on a bus that isn't safe to re-init it does exactly that damage. */
static bool check_survives_reinit(void)
{
    rc_event_t e;
    bool events_ok;
    bool cmds_ok;

    (void)rc_event_init();

    (void)sub_telemetry_set_sink(&cap_sink);
    make_simple(&e, RC_EVT_ODOMETRY);
    e.u.odometry.speed_l_mm_s = 77;
    (void)rc_event_publish(&e);
    events_ok = fresh_state() && snap_has("\"spd_l\":77,");

    (void)sub_telemetry_on_command(NULL, NULL);
    cmdrx_hits = 0U;
    cmds_ok = (sub_telemetry_deliver_command(RC_CMD_TURN_RIGHT, 3) == RC_OK) &&
              wait_hits(&cmdrx_hits, 1U, 1000) &&
              (cmdrx_cmd == RC_CMD_TURN_RIGHT) && (cmdrx_arg == 3);

    if (!events_ok)
    {
        tm_printf((UB *)"[TEST]   telemetry lost its subscriptions\n");
    }
    if (!cmds_ok)
    {
        tm_printf((UB *)"[TEST]   commands no longer reach the bus\n");
    }
    return events_ok && cmds_ok;
}

/* --------------------------------------------------------------------
 * Link robustness
 * -------------------------------------------------------------------- */

/* With the link down, telemetry must stop sending (no hammering a dead
 * link), fail sends straight away, and keep trying to reopen. */
static bool check_dead_link(void)
{
    rc_event_t e;
    uint32_t pubs;
    uint32_t opens;
    bool fast_fail;

    cap_up = false;
    tk_dly_tsk(300);                    /* let any send in flight finish */
    pubs  = cap_publishes;
    opens = cap_opens;

    make_simple(&e, RC_EVT_IMPACT);
    fast_fail = (sub_telemetry_publish_event(&e) == RC_ERR_STATE);

    tk_dly_tsk(1500);
    return fast_fail && (cap_publishes == pubs) && (cap_opens > opens);
}

/* Bring it back up: telemetry should pick up again on its own. */
static bool check_link_recovers(void)
{
    uint32_t c = cap_count(&cap_state);

    cap_up = true;
    return cap_wait(&cap_state, c, 3000);
}

/* set_sink(NULL) means "go back to the console". The capture sink should
 * stop getting anything. */
static bool check_console_fallback(void)
{
    uint32_t pubs;

    if (sub_telemetry_set_sink(NULL) != RC_OK)
    {
        return false;
    }
    tk_dly_tsk(100);
    pubs = cap_publishes;
    tk_dly_tsk(800);
    return cap_publishes == pubs;
}

/* --------------------------------------------------------------------
 * WiFi / MQTT (MQTT=1 builds only)
 * -------------------------------------------------------------------- */
#if TM_MQTT

static bool check_mqtt_sink(void)
{
    const sub_telemetry_sink_t *m = sub_telemetry_mqtt_sink();

    return (m != NULL) && (strcmp(m->name, "mqtt") == 0);
}

/* The two kernel message buffers exist with the sizes subsystem1.h
 * promises mqtt_utk.c. */
static bool check_kernel_buffers(void)
{
    T_RMBF tx;
    T_RMBF rx;

    return (tk_ref_mbf(s1_mbf_tx, &tx) == E_OK) &&
           (tk_ref_mbf(s1_mbf_rx, &rx) == E_OK) &&
           (tx.maxmsz == S1_MSG_MAX) && (rx.maxmsz == S1_CMD_MAX);
}

/* The kernel subscribes to whatever wifi_secrets.h says. If that drifts
 * from RC_TOPIC_CMD, commands from the dashboard silently never arrive. */
static bool check_cmd_topic_matches(void)
{
    return strcmp(MQTT_TOPIC, RC_TOPIC_CMD) == 0;
}

static bool check_link_up(INT timeout_ms)
{
    const sub_telemetry_sink_t *m = sub_telemetry_mqtt_sink();
    INT waited = 0;

    while (!m->is_up())
    {
        if (waited >= timeout_ms)
        {
            return false;
        }
        tk_dly_tsk(100);
        waited += 100;
    }
    tm_printf((UB *)"[TEST]   WiFi + broker up after %d ms of waiting\n",
              waited);
    return true;
}

/* Too big for one kernel message: refuse it rather than send half. */
static bool check_oversize_rejected(void)
{
    static char big[300];
    const sub_telemetry_sink_t *m = sub_telemetry_mqtt_sink();

    (void)memset(big, 'x', sizeof(big) - 1U);
    big[sizeof(big) - 1U] = '\0';
    return m->publish(RC_TOPIC_BASE "/test", big,
                      (uint16_t)(sizeof(big) - 1U)) == RC_ERR_PARAM;
}

/* Hand the kernel one marker message. The PC-side script watching
 * car/01/# should see car/01/test {"test":"S1-W06"} arrive at the broker. */
static bool check_queued_for_broker(void)
{
    static const char marker[] = "{\"test\":\"S1-W06\"}";
    const sub_telemetry_sink_t *m = sub_telemetry_mqtt_sink();

    return m->publish(RC_TOPIC_BASE "/test", marker,
                      (uint16_t)(sizeof(marker) - 1U)) == RC_OK;
}

/* Put a command into s1_mbf_rx exactly the way mqtt_utk.c does when one
 * arrives from the broker, then check Buddy 1 picks it up and decodes it. */
static bool inbound(const char *payload, INT timeout_ms)
{
    if (tk_snd_mbf(s1_mbf_rx, (void *)payload, (INT)strlen(payload),
                   TMO_POL) != E_OK)
    {
        return false;
    }
    return wait_hits(&cb_hits, 1U, timeout_ms);
}

static bool check_inbound_command(void)
{
    bool ok;

    cb_hits = 0U;
    (void)sub_telemetry_on_command(test_cmd_cb, NULL);
    ok = inbound("R", 3000) && (cb_cmd == RC_CMD_TURN_RIGHT) && (cb_arg == 0);
    (void)sub_telemetry_on_command(NULL, NULL);
    return ok;
}

static bool check_inbound_garbage(void)
{
    bool delivered;

    cb_hits = 0U;
    (void)sub_telemetry_on_command(test_cmd_cb, NULL);
    delivered = inbound("Z", 800);
    (void)sub_telemetry_on_command(NULL, NULL);
    return !delivered;
}

/* The whole trip over the real network: the car publishes "L" to its own
 * command topic, the broker sends it straight back (the car is subscribed
 * to that topic), mqtt_utk.c drops it into s1_mbf_rx, and Buddy 1 decodes
 * it as turn-left. Car -> WiFi -> broker -> WiFi -> car, no PC involved.
 * Retries once, because a public broker can be slow on the first try. */
static bool check_broker_round_trip(void)
{
    static const char cmd[] = "L";
    const sub_telemetry_sink_t *m = sub_telemetry_mqtt_sink();
    INT attempt;
    bool ok = false;

    (void)sub_telemetry_on_command(test_cmd_cb, NULL);
    for (attempt = 0; (attempt < 2) && !ok; attempt++)
    {
        cb_hits = 0U;
        if (m->publish(RC_TOPIC_CMD, cmd, (uint16_t)(sizeof(cmd) - 1U))
            == RC_OK)
        {
            ok = wait_hits(&cb_hits, 1U, 5000) &&
                 (cb_cmd == RC_CMD_TURN_LEFT);
        }
    }
    (void)sub_telemetry_on_command(NULL, NULL);
    return ok;
}

#else /* console build: the S1-W tests can't run, say so */

static void skip(const char *id, const char *why)
{
    tm_printf((UB *)"[TEST] %s SKIP  %s\n", id, why);
}

#endif /* TM_MQTT */

/* -------------------------------------------------------------------- */

EXPORT void subsystem1_test(void)
{
    const sub_telemetry_sink_t *console = sub_telemetry_console_sink();

    TEST_INIT();

    tm_printf((UB *)"[TEST] subsystem1: telemetry + commands\n");

    /* --- unit: sinks, decoding, routing, the task ------------------- */
    TEST_CHECK("S1-U01", (console != NULL) && console->is_up(),
               "console sink present and up");
    TEST_CHECK("S1-U02",
               console->publish(RC_TOPIC_BASE "/test", "{\"t\":1}", 7U) == RC_OK,
               "console sink publishes");
    TEST_CHECK("S1-U03", check_mnemonics(),
               "L/R/S/U/X decode, either case");
    TEST_CHECK("S1-U04", check_barcode_letters(),
               "barcode letters A/B/C/D (and P) decode");
    TEST_CHECK("S1-U05", check_garbage_ignored(),
               "anything else decodes to no command");
    TEST_CHECK("S1-U06", check_callback_routing(),
               "command reaches a registered callback unchanged");
    TEST_CHECK("S1-U07", check_task_priority(),
               "telemetry task at its TEAM.md priority (12)");
    TEST_CHECK("S1-U08", check_task_sleeps(),
               "telemetry task sleeps between ticks");

    /* --- framing, through the capture sink -------------------------- */
    cap_up = true;
    (void)sub_telemetry_set_sink(&cap_sink);

    TEST_CHECK("S1-U09", check_events_accepted(),
               "barcode/hump/obstacle/impact accepted");
    TEST_CHECK("S1-U10", check_unrelated_rejected(),
               "unrelated event rejected, nothing sent");
    TEST_CHECK("S1-U11", check_barcode_frame(),
               "barcode message: exact topic + JSON");
    TEST_CHECK("S1-U12", check_hump_frame(),
               "hump message: exact topic + JSON");
    TEST_CHECK("S1-U13", check_obstacle_frame(),
               "obstacle message: exact topic + JSON");
    TEST_CHECK("S1-U14", check_impact_frame(),
               "impact message: exact topic + JSON");
    TEST_CHECK("S1-U15", check_state_schedule(),
               "state every 250 ms, no seq gaps");
    TEST_CHECK("S1-U16", check_heartbeat(),
               "heartbeat ~2 s with sink + counters");
    TEST_CHECK("S1-U17", check_topics(),
               "every topic under " RC_TOPIC_BASE "/");
    TEST_CHECK("S1-U21", check_timestamps_real_time(),
               "message timestamps keep real time (5 %)");
    TEST_CHECK("S1-U22", check_state_fields(),
               "state carries every telemetry item in the brief");

    /* --- integration: the test plays Buddies 2-5 and sub_nav -------- */
    listeners_attach();
    TEST_CHECK("S1-I01", check_bus_running(),
               "event bus running (event delivered)");

    play_sensor_buddies(123, -45, 1000U, 2000U, true, false,
                        RC_MOTION_CRUISING);
    {
        bool got = fresh_state();

        TEST_CHECK("S1-I02",
                   got && snap_has("\"spd_l\":123,\"spd_r\":-45,\"dist\":1500"),
                   "B2 odometry -> speed + distance in state");
        TEST_CHECK("S1-I03", got && snap_has("\"line\":\"10\""),
                   "B3 line sample -> line field");
        TEST_CHECK("S1-I04", got && snap_has("\"cls\":2"),
                   "B4 motion class -> cls field");
    }
    TEST_CHECK("S1-I05", check_bus_barcode(),
               "B3 barcode -> barcode message + bc field");
    TEST_CHECK("S1-I06", check_bus_hump(),
               "B4 hump end -> hump message + peak field");
    TEST_CHECK("S1-I07", check_bus_impact(),
               "B4 impact -> impact message");
    TEST_CHECK("S1-I08", check_bus_obstacle(),
               "B5 obstacle profile -> obstacle message");
    TEST_CHECK("S1-I09", check_command_to_nav(),
               "command -> RC_EVT_COMMAND_RX for sub_nav");
    TEST_CHECK("S1-I10", check_worst_case_state(),
               "worst-case inputs: state fits, no overflow");
    TEST_CHECK("S1-I11", check_no_drops(),
               "no event-bus drops on either lane");

    /* --- link robustness -------------------------------------------- */
    TEST_CHECK("S1-U18", check_dead_link(),
               "dead link: stop sending, fail fast, keep retrying");
    TEST_CHECK("S1-U19", check_link_recovers(),
               "link back: telemetry resumes on its own");
    TEST_CHECK("S1-U20", check_console_fallback(),
               "set_sink(NULL) falls back to console");

    /* --- WiFi / MQTT ------------------------------------------------- */
#if TM_MQTT
    (void)sub_telemetry_set_sink(sub_telemetry_mqtt_sink());

    TEST_CHECK("S1-W01", check_mqtt_sink(), "MQTT sink present");
    TEST_CHECK("S1-W02", check_kernel_buffers(),
               "kernel message buffers exist, agreed sizes");
    TEST_CHECK("S1-W03", check_cmd_topic_matches(),
               "kernel subscribes to " RC_TOPIC_CMD);
    TEST_CHECK("S1-W04", check_link_up(30000),
               "WiFi joined + broker connected (30 s)");
    TEST_CHECK("S1-W05", check_oversize_rejected(),
               "oversize message rejected, not cut short");
    TEST_CHECK("S1-W06", check_queued_for_broker(),
               "message handed to the kernel for the broker");
    TEST_CHECK("S1-W07", check_inbound_command(),
               "inbound 'R' from kernel buffer -> turn right");
    TEST_CHECK("S1-W08", check_inbound_garbage(),
               "inbound garbage ignored");
    TEST_CHECK("S1-W09", check_broker_round_trip(),
               "command round trip car -> broker -> car");
#else
    skip("S1-W01", "MQTT sink present (needs NET=lwip MQTT=1)");
    skip("S1-W02", "kernel message buffers (needs MQTT=1)");
    skip("S1-W03", "kernel command topic (needs MQTT=1)");
    skip("S1-W04", "WiFi + broker (needs MQTT=1)");
    skip("S1-W05", "oversize rejected (needs MQTT=1)");
    skip("S1-W06", "handed to kernel (needs MQTT=1)");
    skip("S1-W07", "inbound command (needs MQTT=1)");
    skip("S1-W08", "inbound garbage (needs MQTT=1)");
    skip("S1-W09", "broker round trip (needs MQTT=1)");
#endif

    /* --- last: must survive someone re-initialising the bus --------- */
    TEST_CHECK("S1-I12", check_survives_reinit(),
               "a 2nd rc_event_init() doesn't cut Buddy 1 off");

    /* Leave things as start-up had them: no callback, and MQTT if this
     * build has it (NULL -> console otherwise). */
    (void)sub_telemetry_on_command(NULL, NULL);
    (void)sub_telemetry_set_sink(sub_telemetry_mqtt_sink());

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
