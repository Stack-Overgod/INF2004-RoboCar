/*
 * sub_telemetry_mqtt.c  --  Buddy 1 / subsystem 1
 *
 * The MQTT sink, rewritten for the course template. We're not allowed to
 * touch cyw43/lwIP/MQTT from a student task -- the kernel's own task owns
 * the radio and the broker. So this sink never talks to the network. It
 * just hands messages to the kernel through two message buffers:
 *
 *   publish()  packs "<topic>\0<payload>" into s1_mbf_tx. mqtt_utk.c drains
 *              it and does the real publish. Non-blocking: a full buffer is
 *              a failed send, not a wait.
 *   poll()     reads commands the kernel dropped into s1_mbf_rx and passes
 *              them to sub_nav.
 *   is_up()    reads link + broker status (reading status is fine from any
 *              task).
 *
 * Built only when TM_MQTT is set; otherwise sub_telemetry_mqtt_sink()
 * returns NULL and the console sink stays in charge.
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

/* Up only when the WiFi link is up and the broker connection is live. */
static bool mqtt_up(void)
{
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    return (st.link_status == S1_LINK_UP) && (mqtt_utk_is_connected() != 0);
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

/* First character of the payload -> a driving command. Accepts the mnemonic
 * letters (L/R/S/U/X) and the barcode letters (A/B/C/D). Unchanged from the
 * old sink. */
static rc_nav_cmd_t parse_cmd(char c)
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

/* Called each telemetry tick: drain every pending command and run it. */
void sub_telemetry_mqtt_poll(void)
{
    UB buf[S1_CMD_MAX];
    rc_nav_cmd_t cmd;

    while (tk_rcv_mbf(s1_mbf_rx, buf, TMO_POL) > 0)
    {
        cmd = parse_cmd((char)buf[0]);
        if (cmd != RC_CMD_NONE)
        {
            (void)sub_telemetry_deliver_command(cmd, 0);
        }
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

#else   /* plain build: console only */

const sub_telemetry_sink_t *sub_telemetry_mqtt_sink(void)
{
    return NULL;
}

#endif
