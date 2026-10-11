/*
 * subsystem1.h -- Subsystem 1: telemetry and commands. ID: <your ID>.
 *
 * Subsystem 1's public interface -- only what other parts of the build need
 * to see. The telemetry API itself lives in sub_telemetry.h.
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "sub_telemetry.h"

#define S1_PRI_TELEMETRY  12    /* band 12-13 (TEAM.md) */
#define S1_MSG_MAX        240   /* longest topic + '\0' + PAYLOAD_MAX 224 */
#define S1_CMD_MAX        16

/* The two message buffers that carry data to/from the kernel's MQTT task.
 * Defined in sub_telemetry_mqtt.c; mqtt_utk.c sees them as IMPORT too.
 *   IF-12  s1_mbf_tx  outgoing "<topic>\0<payload>", drained by mqtt_utk.c
 *   IF-13  s1_mbf_rx  raw inbound command payloads, written by mqtt_utk.c */
IMPORT ID s1_mbf_tx;
IMPORT ID s1_mbf_rx;

#endif /* SUBSYSTEM1_H */
