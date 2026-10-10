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
 * buffer. Framing, topics and the schedule below didn't change -- only the
 * transport moved behind those buffers.
 */
#ifndef SUB_TELEMETRY_H
#define SUB_TELEMETRY_H

#include "../rc_types.h"        /* rc_result_t, rc_event_t, rc_nav_cmd_t */

/*
 * A telemetry transport. Fill in the five fields, hand it to
 * sub_telemetry_set_sink(), and the rest of the module sends through it
 * without knowing what it is. publish() runs on the telemetry task, so it
 * may wait briefly but must never block forever.
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

/*
 * The template splits start-up into two passes: everyone's _init() runs
 * first, then everyone's _start(). So we create objects in one call and
 * start the task in another.
 */
rc_result_t sub_telemetry_init(void);          /* subscribe + make objects */
rc_result_t sub_telemetry_start_task(void);    /* start the telemetry task */

/* Swap the active transport. NULL falls back to the console sink. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *sink);

/* Register a handler for inbound commands. Optional: with none registered,
 * commands are published as RC_EVT_COMMAND_RX for sub_nav instead. */
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx);

/* Send one event's message now, instead of waiting for the next tick. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt);

/* The always-available console sink (prints over USB, needs no radio). */
const sub_telemetry_sink_t *sub_telemetry_console_sink(void);

/* The MQTT sink (sub_telemetry_mqtt.c). Returns NULL in a plain build, so
 * sub_telemetry_set_sink(NULL) just keeps the console sink. */
const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void);

/* MQTT helpers, built only when TM_MQTT is set (sub_telemetry_mqtt.c).
 * init() creates the two message buffers; poll() drains inbound commands. */
rc_result_t sub_telemetry_mqtt_init(void);
void sub_telemetry_mqtt_poll(void);

/* Hand a decoded command to sub_nav (via the registered callback, or the
 * event bus). Keeps the caller out of nav's state. */
rc_result_t sub_telemetry_deliver_command(rc_nav_cmd_t cmd, int32_t arg);

#endif /* SUB_TELEMETRY_H */
