/*
 * sub_telemetry_mqtt.c  --  Buddy 1 / subsystem 1
 *
 * The MQTT sink, rewritten for the course template. We're not allowed to
 * touch cyw43/lwIP/MQTT from a student task -- the kernel's own task owns
 * the radio and the broker. So this sink never talks to the network. It
 * just hands messages to the kernel through two message buffers:
 *
 *   publish()  packs "<topic>\0<payload>" into s1_mbf_tx (IF-12).
 *              mqtt_utk.c drains it and does the real publish.
 *              Non-blocking: a full buffer is a failed send, not a wait.
 *   wait()     the telemetry task's sleep. It waits on s1_mbf_rx (IF-13),
 *              so a command the kernel drops in is handled straight away
 *              instead of at the end of the tick.
 *   link()     reads link + broker status (reading status is fine from any
 *              task, TEAM.md section 3).
 *
 * In a build without MQTT the bottom of this file stands in: no sink, a
 * plain sleep, and a link that's always down.
 */
#include "../rc_prelude.h"
#include "../rc_fmt.h"             /* rc_strlen() */
#include "subsystem1.h"           /* S1_* sizes + the message-buffer IDs */

#if TM_MQTT
#include "cyw43_utk.h"            /* on the include path in WIFI=cyw43 builds */
#include "mqtt_utk.h"             /* on the include path in NET=lwip builds   */

/* cyw43_tcpip_link_status() == CYW43_LINK_UP: joined and has an IP. */
#define S1_LINK_UP  (3)

/* These two live here; subsystem1.h and mqtt_utk.c see them as IMPORT. */
EXPORT ID s1_mbf_tx;
EXPORT ID s1_mbf_rx;

/* Make the two message buffers. Called from sub_telemetry_init(). */
rc_result_t sub_telemetry_mqtt_init(void)
{
    T_CMBF cmbf = {0};

    /* Outgoing telemetry: room for ~8-10 state messages of backlog. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = 2048;
    cmbf.maxmsz = S1_MSG_MAX;
    s1_mbf_tx = tk_cre_mbf(&cmbf);

    /* Inbound command payloads: much smaller. */
    cmbf.bufsz = 64;
    cmbf.maxmsz = S1_CMD_MAX;
    s1_mbf_rx = tk_cre_mbf(&cmbf);

    return ((s1_mbf_tx > 0) && (s1_mbf_rx > 0)) ? RC_OK : RC_ERR_NOSPACE;
}

/* The kernel handles joining and the broker, so there's nothing to do. */
static rc_result_t mqtt_open(void)
{
    return RC_OK;
}

static rc_result_t mqtt_close(void)
{
    return RC_OK;
}

/* WiFi is up when the radio has joined and got an IP; the broker is up
 * when mqtt_utk.c's connection has been accepted (only possible with
 * WiFi up). This is what RC_EVT_LINK_STATE carries. */
void sub_telemetry_mqtt_link(bool *wifi_up, bool *broker_up)
{
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    *wifi_up = (st.link_status == S1_LINK_UP);
    *broker_up = *wifi_up && (mqtt_utk_is_connected() != 0);
}

/* The sink is usable only when the broker connection is live. */
static bool mqtt_up(void)
{
    bool wifi;
    bool broker;

    sub_telemetry_mqtt_link(&wifi, &broker);
    return broker;
}

/* Pack the message as "<topic>\0<payload>" and drop it in the TX buffer.
 * Runs on two tasks (the telemetry task and the slow dispatcher), so it
 * must not wait: TMO_POL, and a full buffer just fails the send. No libc
 * string calls here -- rc_strlen() plus copy loops. */
static rc_result_t mqtt_publish_fn(const char *topic, const char *payload,
                                   uint16_t len)
{
    UB msg[S1_MSG_MAX];
    uint32_t tlen = rc_strlen(topic) + 1U;      /* keep the '\0' */
    uint32_t i;

    if ((tlen + len) > sizeof(msg))
    {
        return RC_ERR_PARAM;
    }
    for (i = 0U; i < tlen; i++)
    {
        msg[i] = (UB)topic[i];
    }
    for (i = 0U; i < len; i++)
    {
        msg[tlen + i] = (UB)payload[i];
    }

    return (tk_snd_mbf(s1_mbf_tx, msg, (INT)(tlen + len), TMO_POL) == E_OK)
           ? RC_OK : RC_ERR_NOSPACE;
}

/* The telemetry task's sleep. Rather than tk_dly_tsk(ms), wait on the
 * command buffer for whatever is left of the period, so a command is
 * passed on the moment it arrives. Only the first character of the
 * payload matters (see sub_telemetry_parse_cmd). The period still ends
 * on time: each wait is only for what's left of it. */
void sub_telemetry_mqtt_wait(uint32_t ms)
{
    UB buf[S1_CMD_MAX];
    SYSTIM start;
    SYSTIM now;
    uint32_t gone = 0U;

    (void)tk_get_tim(&start);
    while (gone < ms)
    {
        if (tk_rcv_mbf(s1_mbf_rx, buf, (TMO)(ms - gone)) > 0)
        {
            (void)sub_telemetry_receive_command((char)buf[0]);
        }
        (void)tk_get_tim(&now);
        gone = (uint32_t)(now.lo - start.lo);
    }
}

static const sub_telemetry_sink_t mqtt_sink =
{
    "mqtt", mqtt_open, mqtt_publish_fn, mqtt_close, mqtt_up
};

const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void)
{
    return &mqtt_sink;
}

#else   /* plain build: console only, no network */

rc_result_t sub_telemetry_mqtt_init(void)
{
    return RC_ERR_STATE;
}

void sub_telemetry_mqtt_wait(uint32_t ms)
{
    tk_dly_tsk((TMO)ms);
}

void sub_telemetry_mqtt_link(bool *wifi_up, bool *broker_up)
{
    *wifi_up = false;
    *broker_up = false;
}

const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void)
{
    return NULL;
}

#endif
