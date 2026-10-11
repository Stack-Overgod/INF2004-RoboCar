/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the drain below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	Buddy 1 (subsystem 1) edit: this file now DRAINS subsystem 1's outgoing
 *	telemetry from the message buffer s1_mbf_tx and publishes it, and
 *	forwards inbound command payloads into s1_mbf_rx. That needs the kernel
 *	message-buffer API (tk_rcv_mbf/tk_snd_mbf), so this file now includes
 *	tk/tkernel.h -- and therefore applies the same size_t workaround
 *	sys_arch.c uses (PROHIBIT_DEF_SIZE_T, below) before any libc header.
 */

/* --- Buddy 1 edit (a): tk/tkernel.h for the message buffers, with the
 * size_t workaround, BEFORE every other include. --- */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

/* --- Buddy 1 edit (a), continued --- */
#include <string.h>

IMPORT ID s1_mbf_tx;              /* IF-12: outgoing telemetry, drained here */
IMPORT ID s1_mbf_rx;              /* IF-13: inbound commands, written here   */

#define MQTT_UTK_DRAIN_MS   50
#define MQTT_UTK_MSG_MAX    240   /* must equal S1_MSG_MAX in subsystem1.h */

static volatile int connected_flag;
int mqtt_utk_is_connected(void) { return connected_flag; }

#define MQTT_UTK_HEARTBEAT_MS   5000   /* unused after the drain change */
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past the
 * commands this expects; anything longer is truncated and flagged rather than
 * being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t		rx_len;
static u8_t		rx_truncated;

/* --- Buddy 1 edit (b): renamed from heartbeat_timeout. --- */
static void drain_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			/* The console is a terminal -- keep raw bytes from a
			 * payload this demo did not publish from scrambling it. */
			UB c = data[i];
			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
			  rx_truncated? " (truncated)": "");

		/* --- Buddy 1 edit (d): forward the payload to subsystem 1's RX
		 * buffer. sub_telemetry_mqtt_poll() parses it (first byte -> a
		 * navigation command). Non-blocking; a full buffer just drops it. */
		if((s1_mbf_rx > 0) && (rx_len > 0U)) {
			(void)tk_snd_mbf(s1_mbf_rx, rx_data,
					 (INT)((rx_len < 16U)? rx_len: 16U),
					 TMO_POL);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* --- Buddy 1 edit (b): replaces heartbeat_timeout(). Publishes at most one
 * message per tick, drained from s1_mbf_tx. lwIP's MQTT output buffer is 256
 * bytes (default) and our largest message is ~245 bytes, so a message that
 * doesn't fit yet (ERR_MEM) stays pending and is retried on the next tick --
 * a busy link never loses it. --- */
static void drain_timeout(void *arg)
{
	static UB	msg[MQTT_UTK_MSG_MAX];
	static INT	pending = 0;

	(void)arg;

	if((pending == 0) && (s1_mbf_tx > 0)) {
		INT n = tk_rcv_mbf(s1_mbf_tx, msg, TMO_POL);
		pending = (n > 0)? n: 0;
	}
	if((pending > 0) && mqtt_client_is_connected(client)) {
		size_t	tlen = strlen((const char *)msg) + 1U;
		err_t	err  = mqtt_publish(client, (const char *)msg, &msg[tlen],
					    (u16_t)(pending - (INT)tlen),
					    0 /* qos */, 0 /* retain */,
					    publish_done_cb, NULL);
		if(err != ERR_MEM) {
			pending = 0;	/* sent, or failed for good: move on */
		}
	}

	sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		connected_flag = 1;		/* Buddy 1 edit (c) */

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to MQTT_TOPIC (= car/01/cmd). Every message the
		 * broker delivers on it arrives in incoming_data_cb(), which now
		 * forwards it to subsystem 1. Callbacks must be registered before
		 * subscribing, or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating timer, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(drain_timeout, NULL);		/* Buddy 1 edit (c) */
		sys_timeout(MQTT_UTK_DRAIN_MS, drain_timeout, NULL);
	} else {
		connected_flag = 0;		/* Buddy 1 edit (c) */

		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
