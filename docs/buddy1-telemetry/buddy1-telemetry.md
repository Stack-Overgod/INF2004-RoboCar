# Buddy 1 — WiFi, Commands & Telemetry

> How Buddy 1 works in this repo, and **how the other buddies link to it**.
> Code: `app_program/subsystem1/` · Tests: `testing/subsystem1/` ·
> Every topic and message: [`MQTT.md`](MQTT.md).

## The short version

You don't call Buddy 1, and you don't need to include anything from it, to
have your data reported. **Publish your events on the bus and Buddy 1 picks
them up.** The other buddies' code on this branch already publishes every
event Buddy 1 listens to. So the moment your `subsystemN_init()` /
`subsystemN_start()` actually start your code, it shows up at the broker,
with no changes on either side.

Going the other way, Buddy 1 puts two things on the bus for you:
`RC_EVT_COMMAND_RX` (a command from the dashboard) and `RC_EVT_LINK_STATE`
(WiFi/broker up or down).

```
Buddy 2 ─┐ ODOMETRY, MOTION_DONE                      ┌─> car/01/state   every 250 ms
Buddy 3 ─┤ LINE_SAMPLE, LINE_LOST/REACQUIRED, BARCODE ├─> car/01/status  every 2 s
Buddy 4 ─┼ IMU_SAMPLE, MOTION_CLASS, HUMP/RAMP, LIFTED├─> car/01/barcode, hump, ...  on the event
Buddy 5 ─┘ ULTRA_RESULT, OBSTACLE_PROFILE, AVOID_PLAN │
             (rc_event, SLOW lane)  ──>  Buddy 1  ─────┘   (s1_mbf_tx -> kernel MQTT -> broker)

dashboard -> car/01/cmd -> kernel -> s1_mbf_rx -> Buddy 1 -> RC_EVT_COMMAND_RX -> sub_nav
                                                           -> car/01/ack (echo)
```

## What Buddy 1 is responsible for

From the project brief and the design review (FR10a–FR11b):

| Brief task | Where it lives |
|---|---|
| Establish WiFi and MQTT | the kernel's `cyw43_utk_task` + `lib/liblwip/.../mqtt_utk.c` (Buddy 1's patch, IF-12/IF-13); only that task may touch the radio (TEAM.md §3) |
| Design topics and message structures | `sub_telemetry.c`, `publish_state()` / `publish_heartbeat()` / `sub_telemetry_publish_event()`; written up in [`MQTT.md`](MQTT.md) |
| Publish telemetry (FR10a) | the telemetry task, every 250 ms, plus a message the moment something notable happens |
| Subscribe to commands (FR10b) | `sub_telemetry_mqtt_wait()` takes each command the moment the kernel hands it over → `RC_EVT_COMMAND_RX` |
| Recover connections (FR11b) | `telemetry_link_ok()`: back-off 500 → 8000 ms, never a busy loop |
| Report heartbeat (FR11a) | `car/01/status` every 2 s: link state, counters, which buddies are "live" |

## How Buddy 1 is wired into the template (the pattern to copy)

`usermain()` in `app_main.c` runs in three phases, and every subsystem fits
into them the same way:

| Phase | Who | What Buddy 1 does there |
|---|---|---|
| 0 | `app_main.c` | starts the shared layer: `rc_defer_init()`, `rc_event_init()`, `rc_gpioirq_init()`, `rc_fault_init()`. The bus is running before anyone subscribes. |
| 1 | `subsystem1_init()` | `sub_telemetry_init()`: subscribes to the 18 events in `subs[]` and, in an MQTT build, creates the two message buffers. **No tasks yet.** |
| 2 | `subsystem1_start()` | `sub_telemetry_start_task()`: starts the `telemetry` task at priority 12 (`S1_PRI_TELEMETRY` = `RC_PRI_TELEMETRY`, the lowest application priority). |

`subsystem1.c` is just those two calls. The work lives in `sub_*.c` files,
and `rc_result_t` is turned into the `ER` the template wants:

```c
EXPORT ER subsystem1_init(void)
{
    return (sub_telemetry_init() == RC_OK) ? E_OK : E_SYS;
}

EXPORT ER subsystem1_start(void)
{
    return (sub_telemetry_start_task() == RC_OK) ? E_OK : E_SYS;
}
```

Your subsystem has the same shape. `subsystemN_init()` calls your `drv_*_init()` /
`sub_*_init()` (they create objects and **subscribe**), and `subsystemN_start()`
starts your tasks with `app_start_task(entry, RC_PRI_..., core, "name")` (priorities from `rc_config.h`).
Subscribe in `_init`, never in `_start`. On the dual-core build another task
can run the moment you start one, so every subscription has to exist first.

The other files follow the template's rules:

- **`subsystem1.h`** is the public interface. Its top comment is the whole
  linking contract as a table: what Buddy 1 reads from whom, and what it gives back.
- **`subsystem1_test.c`** plays every other buddy on the real bus (S1-I tests).
  Buddy 1 is therefore tested without anyone else's code, and those tests are
  the contract.
- **`testing/subsystem1/TEST_LOG.md`** maps every test to a brief task and FR
  and records every run.

## Linking to Buddy 1: what you publish

All of these are subscribed on the **SLOW** lane, so reporting never delays
your control path. Buddy 1's callbacks only copy a few numbers, or send one
message, and return.

| You | Event | When | Fields Buddy 1 reads | Where it shows up |
|---|---|---|---|---|
| **Buddy 2** | `RC_EVT_ODOMETRY` | every 20 ms | `speed_l_mm_s`, `speed_r_mm_s`, `dist_l_mm`, `dist_r_mm` | `state`: `spd_l`, `spd_r`, `dist` (average of both wheels) · `status`: live flag 1 |
| | `RC_EVT_MOTION_DONE` | a queued move finishes or aborts | `move_id`, `completed`, `travelled_mm` | `car/01/move` |
| | `drv_encoder_count()`, `drv_motor_get()` | read every 250 ms | (direct calls, no event exists) | `state`: `enc_l`, `enc_r`, `m_l`, `m_r` |
| **Buddy 3** | `RC_EVT_LINE_SAMPLE` | every 10 ms | `on_line_l`, `on_line_r` | `state`: `line` ("10" = left on black) · live flag 2 |
| | `RC_EVT_LINE_LOST` / `RC_EVT_LINE_REACQUIRED` | on the change | (none) | `car/01/line` `{"lost":1}` / `{"lost":0}` |
| | `RC_EVT_BARCODE_DECODED` | a character decodes | `sym`, `command`, `reversed` | `car/01/barcode` · `state`: `bc` |
| | `RC_EVT_BARCODE_SCAN`, `RC_EVT_BARCODE_RAW` | each decode attempt | all fields | `car/01/bcscan`, `car/01/bcbits` |
| **Buddy 4** | `RC_EVT_IMU_SAMPLE` | every 10 ms | (only that it arrives) | `status`: live flag 3. A `0` here is how "IMU unavailable" shows (FR15). |
| | `RC_EVT_MOTION_CLASS` | **only when the class changes** | `cls` | `state`: `cls` · `car/01/motion` (FR14) |
| | `RC_EVT_HUMP_END` | a hump finishes | `peak_mm`, `duration_ms`, `max_pitch_deg` | `car/01/hump` · `state`: `peak` (highest so far) |
| | `RC_EVT_RAMP_END` | a slope finishes | all four fields | `car/01/ramp` |
| | `RC_EVT_LIFTED` | the car is picked up, tilted or overturned | `why`, `h_mm`, `v_cm_s`, `ref_mg` | `car/01/lifted` |
| | `sub_terrain_height_mm()` | read every 250 ms | (direct call, no event) | `state`: `h` |
| **Buddy 5** | `RC_EVT_ULTRA_RESULT` | each ping | `angle_deg`, `range_mm` | `state`: `rng`, **only readings at 90° ± 5°** (straight ahead); range 0 = no echo · live flag 4 |
| | `RC_EVT_OBSTACLE_PROFILE` | a scan completes | all six fields | `car/01/obstacle` |
| | `RC_EVT_AVOIDANCE_PLAN` | right after the profile | `action`, `lateral_mm`, `forward_mm` | `car/01/avoid` |
| | `RC_EVT_SPIN_EDGES` | right after the plan | all fields | `car/01/spin` |
| **nav** | `sub_nav_state()`, `rc_fault_get()` | read every 250 ms | (direct calls, no event) | `state`: `nav`, `flt` |

Publishing is what you already do:

```c
rc_event_t evt = {0};                 /* zero it: unset fields go out as 0, not garbage */

evt.id = RC_EVT_HUMP_END;
evt.u.hump.peak_mm       = final_height;
evt.u.hump.duration_ms   = duration;
evt.u.hump.max_pitch_deg = (int16_t)(peak_pitch / 10);
(void)rc_event_publish(&evt);         /* task context; rc_event_publish_i() from an ISR */
```

### Things that break the link (please check yours)

1. **Your code isn't started.** If `subsystemN_init/start` don't start your
   code, nothing you publish exists. The status message's `"live"` flags
   show it straight away (`"live":"1000"` = only Buddy 2's odometry is arriving).
2. **A field left unset.** Buddy 1 sends what it's given. Start from
   `rc_event_t evt = {0};`.
3. **Motion class sent every sample.** Buddy 4 already only publishes on a
   change. Keep it that way: it's also what `car/01/motion` streams.
4. **Wrong angle on an ultrasonic reading.** `rng` only takes readings tagged
   90° ± 5°, so tag each reading with the angle the servo was really at.
5. **Changing Buddy 2's read-backs.** Buddy 1 calls
   `int16_t drv_motor_get(rc_side_t)` and `uint32_t drv_encoder_count(rc_side_t)`
   directly. Keep those two signatures, or tell Buddy 1 first.
6. **Calling `rc_event_init()` again** (a test did). It's harmless now (the
   second call does nothing), but don't rely on it.
7. **Touching the radio.** No task may call `cyw43_*`, lwIP or MQTT. Use
   `RC_EVT_LINK_STATE` (below) to find out whether the network is up.

## Getting things from Buddy 1

### Commands from the dashboard: `RC_EVT_COMMAND_RX`

The vehicle controller, `sub_nav.c` (`on_command_rx`), subscribes on the
**FAST** lane. It takes STOP from any running state, START (`G`) from IDLE or
STOPPED, and turn / straight / U-turn only while following the line
(FOLLOWING or READING_BARCODE). The pattern, for any subscriber:

```c
#include "../rc_event.h"

static void on_remote_cmd(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    switch (evt->u.command.command)
    {
        case RC_CMD_STOP:        /* X or P: honour from ANY state (FR10b, FR12) */
            (void)sub_motion_stop(true);      /* Buddy 2: brake */
            break;
        case RC_CMD_TURN_LEFT:   /* L or A */
        case RC_CMD_TURN_RIGHT:  /* R or B */
        case RC_CMD_GO_STRAIGHT: /* S or C */
        case RC_CMD_U_TURN:      /* U or D */
            /* same meaning as Buddy 3's barcodes, act while line following */
            break;
        default:
            break;
    }
}

/* in your subsystemN_init(): */
if (rc_event_subscribe(RC_EVT_COMMAND_RX, RC_LANE_FAST, on_remote_cmd, NULL) < 0)
{
    return E_SYS;
}
```

- The letters map to the same `rc_nav_cmd_t` values as Buddy 3's barcodes, so
  one handler can serve both.
- `u.command.arg` is always 0 for now.
- Buddy 1 passes the command on first and acks it after, so a STOP isn't held
  up. The dashboard sees `car/01/ack` `{"rx":"X","cmd":5}`.
- Optional extra: `sub_telemetry_on_command(cb, ctx)` calls `cb` on the
  telemetry task for every command, as well as the event. It is an addition,
  not a replacement. Use the event if you can.

### Network up or down: `RC_EVT_LINK_STATE`

```c
static void on_link(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    remote_ok = evt->u.link.broker_up;     /* connected = WiFi joined, broker_up = MQTT live */
}
/* in _init(): */ (void)rc_event_subscribe(RC_EVT_LINK_STATE, RC_LANE_SLOW, on_link, NULL);
```

Buddy 1 sends it once on its first tick, then only when something changes.
A subscriber made in `_init()` therefore always learns the starting state. Need it
right now instead? `sub_telemetry_get_link(&link)`. In a build without MQTT both
flags are always false.

## Checking your link

1. **On your own, before your code is ready:** `make TEST=1 -j8`, flash, and
   look for `[TEST] subsystem1 SUMMARY`. The S1-I tests publish *your* event
   exactly as the table says and check Buddy 1's output. If one fails after
   you change `rc_types.h`, the contract changed.
2. **For real, on the car:** `make NET=lwip MQTT=1 -j8` (needs
   `config/wifi_secrets.h`, see [`MQTT.md`](MQTT.md)). Then watch the broker:
   ```
   python -I dashboard/dashboard.py --start-broker
   ```
   (the broker log and the dashboard show each binary frame decoded by
   `scripts/tm_frames.py`).
   - `car/01/status` → `"live":"1111"` means all four streams arrive.
   - Push the car or wave at a sensor and watch your field in `car/01/state` change.
   - Publish `X` to `car/01/cmd` → `car/01/ack` `{"rx":"X","cmd":5}` comes back,
     and your `RC_EVT_COMMAND_RX` handler runs.

## TEAM.md rows for subsystem 1

These are what TEAM.md should say for S1. They're kept here rather than edited
into TEAM.md, so the team can agree them in one go:

| Section | Row |
|---|---|
| §1 Members | `1` · WiFi communication, commands and telemetry · 2503331 · Rexzer · `app_program/subsystem1/` · `testing/subsystem1/` |
| §2 Pins | none (the radio's GP23/24/25/29 are already *reserved*) |
| §3 Peripherals | none. WiFi/TCP-IP/MQTT stay with the kernel's `cyw43_utk_task`. |
| §4 Priorities | `telemetry` – `RC_PRI_TELEMETRY` 12 – any |
| §5 Interfaces | `IF-12` · S1 → kernel · message buffer (owned by S1) · `s1_mbf_tx` · `"<topic>\0<payload>"`, ≤ 240 bytes, `TMO_POL` both sides |
| | `IF-13` · kernel → S1 · message buffer (owned by S1) · `s1_mbf_rx` · raw `car/01/cmd` payload, ≤ 16 bytes |
| | S2–S5 → S1 · `rc_event`, SLOW lane · the 18 events in `subsystem1.h` · fields as in the table above |
| | S1 → vehicle controller · `rc_event` · `RC_EVT_COMMAND_RX`, `RC_EVT_LINK_STATE` |
| §6 Objects | S1: Task 1, Message buffer 2 (MQTT builds only) |

## Build and test

```
cd build_make
make -j8                          # console telemetry, no network
make TEST=1 -j8                   # + all S1-U / S1-I tests (S1-W print SKIP)
make NET=lwip MQTT=1 TEST=1 -j8   # + WiFi/MQTT tests S1-W01..W10
```

Results go in `testing/subsystem1/TEST_LOG.md`, and the console logs go in
`testing/subsystem1/logs/`.

## Not Buddy 1's (yet), so you know

- **Commands are one character.** Only the first byte counts, and `arg` is 0.
