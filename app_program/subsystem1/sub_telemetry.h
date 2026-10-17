/*
 * sub_telemetry.h  --  Buddy 1 / subsystem 1
 *
 * Telemetry: package up what the car is doing and ship it out, and take
 * driving commands back in. The transport sits behind a small "sink"
 * interface so the rest of this module never cares whether messages go to
 * the USB console or out over MQTT.
 *
 * NOTE: the course template won't let a student task call cyw43/lwIP/MQTT
 * directly -- the kernel owns the radio. So the MQTT sink doesn't talk to
 * the network; it just drops messages into a kernel message buffer that
 * mqtt_utk.c drains and publishes, and reads commands back from a second
 * buffer.
 *
 * Other buddies don't need anything in here to link to Buddy 1: they
 * publish their events and Buddy 1 picks them up (subsystem1.h has the
 * full list). The calls under "for other subsystems" are optional extras.
 */
#ifndef SUB_TELEMETRY_H
#define SUB_TELEMETRY_H

#include "../rc_types.h"        /* rc_result_t, rc_event_t, rc_nav_cmd_t */

/*
 * A telemetry transport. Fill in the five fields, hand it to
 * sub_telemetry_set_sink(), and the rest of the module sends through it
 * without knowing what it is. publish() runs on the telemetry task and on
 * the slow dispatcher, so it may wait briefly but must never block.
 */
typedef struct
{
    const char *name;                   /* "console", "mqtt", ... */
    rc_result_t (*open)(void);          /* start up (no-op for MQTT) */
    rc_result_t (*publish)(const char *topic, const char *payload,
                           uint16_t len);
    rc_result_t (*close)(void);         /* shut down (no-op for MQTT) */
    bool (*is_up)(void);                /* usable right now? */
} sub_telemetry_sink_t;

/* Callback shape for a command that arrives from outside the car. */
typedef void (*sub_telemetry_cmd_cb_t)(rc_nav_cmd_t cmd, int32_t arg,
                                       void *ctx);

/* --------------------------------------------------------------------
 * Start-up. The template splits it into two passes: everyone's _init()
 * runs first, then everyone's _start(). So we subscribe and create
 * objects in one call and start the task in the other.
 * -------------------------------------------------------------------- */
rc_result_t sub_telemetry_init(void);          /* subscribe + make objects */
rc_result_t sub_telemetry_start_task(void);    /* start the telemetry task */

/* --------------------------------------------------------------------
 * For other subsystems (all optional -- the event bus covers the rest)
 * -------------------------------------------------------------------- */

/* Also call cb for every command from the dashboard. The command still
 * goes out as RC_EVT_COMMAND_RX either way; this is an extra, like
 * Buddy 3's sub_barcode_on_decode(). cb runs on the telemetry task, so it
 * must be short and must not block. NULL removes it. */
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);

/* The network link as last announced on RC_EVT_LINK_STATE. Handy for a
 * subsystem that wants to know now rather than wait for the next change.
 * In a build without MQTT both fields are always false. */
void sub_telemetry_get_link(rc_pl_link_t *out);

/* Send one event's message now, instead of waiting for the next tick.
 * Takes the events Buddy 1 reports on their own topics (barcode, hump,
 * obstacle, impact, motion class, line lost/found, move done, avoidance
 * plan); anything else is RC_ERR_PARAM. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);

/* --------------------------------------------------------------------
 * Transports
 * -------------------------------------------------------------------- */

/* Swap the active transport. NULL falls back to the console sink. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);

/* The always-available console sink (prints over USB, needs no radio). */
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);

/* The MQTT sink (sub_telemetry_mqtt.c). Returns NULL in a plain build, so
 * sub_telemetry_set_sink(NULL) just keeps the console sink. */
const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void);

/* MQTT helpers (sub_telemetry_mqtt.c). In a build without MQTT, init()
 * fails, wait() is a plain sleep and link() reports both down. */
rc_result_t sub_telemetry_mqtt_init(void);     /* make the two buffers */
void sub_telemetry_mqtt_wait(uint32_t ms);     /* sleep, taking commands */
void sub_telemetry_mqtt_link(bool *wifi_up, bool *broker_up);

/* --------------------------------------------------------------------
 * The command path (used by sub_telemetry_mqtt.c and the tests)
 * -------------------------------------------------------------------- */

/* One command character -> a driving command: L/R/S/U/X or barcode letters
 * A/B/C/D, either case (P is also stop). Anything else is RC_CMD_NONE. */
rc_nav_cmd_t sub_telemetry_parse_cmd(char c);

/* A command character has arrived from the dashboard: count it, decode
 * it, hand a real one on (deliver_command below) and echo what we made of
 * it on car/01/ack. Returns the decoded command (RC_CMD_NONE = ignored). */
rc_nav_cmd_t sub_telemetry_receive_command(char c);

/* Hand a decoded command to the vehicle controller: publish
 * RC_EVT_COMMAND_RX, then call the on_command callback if there is one.
 * Keeps the network side out of navigation's state. */
rc_result_t sub_telemetry_deliver_command(rc_nav_cmd_t cmd, int32_t arg);

/* The telemetry task's ID, or 0 before sub_telemetry_start_task(). It's a
 * kernel ID; int32_t keeps this header free of kernel types. */
int32_t sub_telemetry_task_id(void);

#if APP_TEST
/* Test builds only: make the link tracker see this link instead of the
 * real one (on = false goes back to the real one). Lets subsystem1_test.c
 * check RC_EVT_LINK_STATE without pulling the WiFi out. */
void sub_telemetry_test_fake_link(bool on, bool connected, bool broker_up);
#endif

#endif /* SUB_TELEMETRY_H */
