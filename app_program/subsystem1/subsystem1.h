/*
 * subsystem1.h -- Subsystem 1: WiFi, commands and telemetry (Buddy 1).
 * Student ID: <student ID>.
 *
 * Subsystem 1's public interface: only what other parts of the build need
 * to see. The telemetry calls themselves are in sub_telemetry.h. The long
 * version, with examples, is docs/buddy1-telemetry/buddy1-telemetry.md.
 *
 * WHAT BUDDY 1 NEEDS FROM YOU
 * ---------------------------
 * Nothing but your events. Publish them with rc_event_publish() as you
 * already do; Buddy 1 subscribes to each one on the SLOW lane in
 * sub_telemetry_init() and reports it. You never call Buddy 1 to report
 * something, and you never need to include this file to be reported.
 *
 *   From      Event                    Fields Buddy 1 reads    Shows up as
 *   --------  -----------------------  ----------------------  ---------------
 *   Buddy 2   RC_EVT_ODOMETRY          speed_l/r_mm_s,         state: spd_l,
 *             (every 20 ms)            dist_l/r_mm             spd_r, dist
 *   Buddy 2   RC_EVT_MOTION_DONE       move_id, completed,     car/01/move
 *                                      travelled_mm
 *   Buddy 2   drv_encoder_count(),     (read-backs, no event)  state: enc_l,
 *             drv_motor_get()                                  enc_r, m_l, m_r
 *   Buddy 3   RC_EVT_LINE_SAMPLE       on_line_l, on_line_r    state: line
 *   Buddy 3   RC_EVT_LINE_LOST,        (none)                  car/01/line
 *             RC_EVT_LINE_REACQUIRED
 *   Buddy 3   RC_EVT_BARCODE_DECODED   symbol, command,        car/01/barcode,
 *                                      reversed                state: bc
 *   Buddy 4   RC_EVT_IMU_SAMPLE        (only that it arrives)  status: live
 *   Buddy 4   RC_EVT_MOTION_CLASS      cls (send on change)    state: cls,
 *                                                              car/01/motion
 *   Buddy 4   RC_EVT_HUMP_END          peak_mm, duration_ms,   car/01/hump,
 *                                      max_pitch_deg           state: peak
 *   Buddy 4   RC_EVT_IMPACT            (none)                  car/01/impact
 *   Buddy 5   RC_EVT_ULTRA_RESULT      angle_deg, range_mm,    state: rng
 *                                      valid (90 +/- 5 only)
 *   Buddy 5   RC_EVT_OBSTACLE_PROFILE  all six fields          car/01/obstacle
 *   Buddy 5   RC_EVT_AVOIDANCE_PLAN    action, lateral_mm,     car/01/avoid
 *                                      forward_mm
 *
 * Fill in every field you see listed: Buddy 1 sends what it's given, so a
 * field left unset goes out as garbage. The status message's "live" flags
 * show which of the four streams are arriving (Buddy 2, 3, 4, 5).
 *
 * WHAT YOU CAN GET FROM BUDDY 1
 * -----------------------------
 *   RC_EVT_COMMAND_RX   a command from the dashboard (car/01/cmd), already
 *                       decoded: u.command.command is an rc_nav_cmd_t
 *                       (RC_CMD_STOP for X or P), u.command.arg is 0.
 *                       The vehicle controller subscribes on the FAST lane.
 *   RC_EVT_LINK_STATE   the network link changed: u.link.connected (WiFi
 *                       joined) and u.link.broker_up (MQTT session live).
 *                       Sent once at start-up too, so a subscriber made in
 *                       your _init() always learns the starting state.
 *   sub_telemetry_on_command(), sub_telemetry_get_link(),
 *   sub_telemetry_publish_event()   optional calls, see sub_telemetry.h.
 *
 * WHAT BUDDY 1 OWNS (TEAM.md)
 * ---------------------------
 *   Task      telemetry, priority 12 (band 12-13), any core
 *   Objects   1 task; 2 message buffers in MQTT builds (below)
 *   Pins      none. WiFi belongs to the kernel's cyw43_utk_task.
 *   Topics    car/01/... (RC_TOPIC_BASE in rc_config.h)
 *   IF-12     S1 -> kernel  s1_mbf_tx  "<topic>\0<payload>", max 240 bytes
 *   IF-13     kernel -> S1  s1_mbf_rx  raw car/01/cmd payload, max 16 bytes
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
