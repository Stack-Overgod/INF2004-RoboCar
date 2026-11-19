# MQTT — topics and messages

Every topic the car uses, what's in each message, and how to watch and
drive it from a laptop. The code that builds them is
`app_program/subsystem1/sub_telemetry.c`. How the other subsystems feed it is
in [`buddy1-telemetry.md`](buddy1-telemetry.md).

## Topics

All topics hang off one base, `car/01` (`RC_TOPIC_BASE` in `rc_config.h`).
Subscribe to `car/01/#` to get everything from this car, or to `car/+/state`
to get every car's state on a shared broker.

| Topic | Direction | When | Built from |
|---|---|---|---|
| `car/01/state` | car → | every 250 ms (FR10a) | cached values from Buddies 2–5 |
| `car/01/status` | car → | every 2 s (FR11a) | Buddy 1's own counters + link |
| `car/01/barcode` | car → | a barcode character decodes | Buddy 3 `RC_EVT_BARCODE_DECODED` |
| `car/01/bcscan`, `car/01/bcbits` | car → | each barcode decode attempt (diagnostics) | Buddy 3 `RC_EVT_BARCODE_SCAN`, `RC_EVT_BARCODE_RAW` |
| `car/01/line` | car → | the line is lost / found again | Buddy 3 `RC_EVT_LINE_LOST` / `_REACQUIRED` |
| `car/01/hump` | car → | a hump finishes | Buddy 4 `RC_EVT_HUMP_END` |
| `car/01/motion` | car → | the motion class changes (at most once per 250 ms, FR14) | Buddy 4 `RC_EVT_MOTION_CLASS` |
| `car/01/ramp` | car → | a slope finishes | Buddy 4 `RC_EVT_RAMP_END` |
| `car/01/lifted` | car → | the car is picked up, tilted or overturned (it stops) | Buddy 4 `RC_EVT_LIFTED` |
| `car/01/obstacle` | car → | a scan completes | Buddy 5 `RC_EVT_OBSTACLE_PROFILE` |
| `car/01/avoid` | car → | an avoidance plan is chosen | Buddy 5 `RC_EVT_AVOIDANCE_PLAN` |
| `car/01/spin` | car → | a step scan completes (edge diagnostics) | Buddy 5 `RC_EVT_SPIN_EDGES` |
| `car/01/move` | car → | a queued move finishes or aborts | Buddy 2 `RC_EVT_MOTION_DONE` |
| `car/01/ack` | car → | each command character received | Buddy 1 |
| `car/01/cmd` | → car | whenever you send one | you (dashboard / laptop) |

All publishes are QoS 0, not retained. A lost telemetry sample is stale a
tick later anyway. The car subscribes to `car/01/cmd` at QoS 0. The kernel
sends at most one message every 50 ms (20 a second). Telemetry needs about 4.5
a second plus events, so there's plenty of room.

## Messages

Every payload the car sends is a compact binary frame (`subsystem1/tm_frame.h`:
a 2-byte header, `[ver:3 | type:5]` and an 8-bit seq, then each field in only
the bits its range needs; `TM_VER` 4). On the laptop, `scripts/tm_frames.py`
decodes a frame into the JSON shown below; the broker (`scripts/mqtt_broker.py`)
logs that JSON and the dashboard reads it. A decoder refuses a frame with the
wrong version or length, so the car image, `tm_frames.py` and the dashboard
must have the same `TM_VER`. Numbers are integers (the RP2040 has no FPU), so
units are in the names: `_mm`, `_mm_s`, `ms`.

### `car/01/state`

```json
{"seq":44,"t":203416,"spd_l":182,"spd_r":176,"dist":5310,"enc_l":1043,"enc_r":1031,
 "m_l":420,"m_r":415,"line":"10","cls":2,"peak":28,"rng":640,"bc":"B","nav":1,"flt":0,"h":0}
```

| Field | Meaning | From |
|---|---|---|
| `seq` | counts up by one per state message (8 bits, wraps). A gap means one was lost. | Buddy 1 |
| `t` | ms since the car booted | Buddy 1 |
| `spd_l`, `spd_r` | wheel speed, mm/s, signed (negative = backwards) | Buddy 2 odometry |
| `dist` | distance since the current move began, mm (average of both wheels) | Buddy 2 odometry |
| `enc_l`, `enc_r` | encoder edges since boot (only ever go up); in every 4th state message only | Buddy 2 `drv_encoder_count()` |
| `m_l`, `m_r` | last motor duty, per mille (−1000 … 1000) | Buddy 2 `drv_motor_get()` |
| `line` | left/right line sensor, `1` = on black | Buddy 3 line sample |
| `cls` | motion class, see the table below | Buddy 4 |
| `peak` | tallest hump so far this run, mm | Buddy 4 hump end |
| `rng` | ultrasonic range straight ahead, mm. `0` = no echo / nothing in range | Buddy 5, readings at 90° ± 5° only |
| `bc` | last barcode character, `-` = none yet | Buddy 3 barcode |
| `nav` | mission state: 0 IDLE, 1 FOLLOWING, 2 READING_BARCODE, 3 EXECUTING_CMD, 4 AVOIDING, 5 RECOVERING, 6 STOPPED | `sub_nav_state()` |
| `flt` | fault bitmask (`rc_fault.h`): bit 0 MOTOR_STALL, 1 ENCODER, 2 IMU, 3 NO_ECHO, 4 WIFI_LOSS, 5 LIFTED, 6 BARCODE_ADC, 7 WDOG_RESET | `rc_fault_get()` |
| `h` | height above the start from the ramp integrator, mm, signed | Buddy 4 `sub_terrain_height_mm()` |

A state frame is 20 bytes, 28 with the encoder counts.

**Motion classes (`cls`):** 0 stationary, 1 accelerating, 2 cruising,
3 decelerating, 4 turning, 5 climbing, 6 descending, 7 lifted.

### `car/01/status` (the heartbeat)

```json
{"up":203500,"tx":3911,"fail":0,"drop_fast":0,"drop_slow":0,"sink":"mqtt",
 "wifi":1,"mqtt":1,"cmds":4,"retries":0,"live":"1111"}
```

| Field | Meaning |
|---|---|
| `up` | ms since boot |
| `tx`, `fail` | messages handed to the transport OK / failed (link down or queue full) |
| `drop_fast`, `drop_slow` | events the bus had to drop per lane. **Should stay 0.** If not, a subscriber is too slow. |
| `sink` | where telemetry is going: `mqtt`, or `console` (USB) |
| `wifi`, `mqtt` | WiFi joined with an IP / broker session live |
| `cmds` | command characters received since boot |
| `retries` | times telemetry found the link down and backed off |
| `live` | four flags, one per stream, `1` = heard in the last second: Buddy 2 odometry, Buddy 3 line, Buddy 4 IMU, Buddy 5 ultrasonic. `"1101"` = Buddy 4's IMU has gone quiet. |

### Event messages

| Topic | Example | Fields |
|---|---|---|
| `barcode` | `{"sym":"A","cmd":1,"rev":0}` | character, command it maps to (below), `rev` 1 = read back to front |
| `line` | `{"lost":1}` / `{"lost":0}` | 1 = line lost, 0 = found again |
| `hump` | `{"peak_mm":37,"ms":420,"pitch":12}` | peak height, how long it took, steepest pitch in degrees |
| `motion` | `{"cls":4}` | the new motion class |
| `lifted` | `{"why":1,"h_mm":24,"v_cm_s":15,"ref_mg":0}` | `why` 1 picked up, 2 tilted, 3 overturned; the estimated rise and rising speed, and the learnt 1 g reference offset |
| `ramp` | `{"rise_mm":50,"net_mm":0,"deg":15,"len_mm":600}` | highest point above where the slope began, where it ended, steepest angle, length |
| `obstacle` | `{"n":7,"near_mm":190,"at_deg":90,"w_mm":120,"cl_l":60,"cl_r":60}` | stops pinged in the step scan, closest range and its servo angle (90 = ahead), length between the two corners (0 if one was not found), each corner's distance from the line (0 = not found) |
| `spin` | `{"face":190,"n":7,"l0":"…","l1":"-","r1":"…","r2":"-"}` | step-scan diagnostics: face range and the edge bearing/range found on each side (`-` = none) |
| `bcscan`, `bcbits` | see `scripts/tm_frames.py` | barcode decode diagnostics (result, failure counters, bar/space bits) |
| `avoid` | `{"act":1,"lat_mm":150,"fwd_mm":300}` | action (command values below), how far to step aside, how far to go past |
| `move` | `{"id":7,"ok":1,"mm":86}` | move id, 1 = finished / 0 = aborted, distance travelled |
| `ack` | `{"rx":"L","cmd":1}` | the character received and what it decoded to. `"cmd":0` = ignored. Quotes and control bytes show as `?`. |

**Command values** (`cmd`, `act`): 0 none, 1 turn left, 2 turn right,
3 straight, 4 U-turn, 5 stop, 6 start.

## Commands: `car/01/cmd`

Send a single character. Only the first byte counts, and either case works.

| Send | Command | Same as barcode |
|---|---|---|
| `L` or `A` | turn left | A |
| `R` or `B` | turn right | B |
| `S` or `C` | go straight | C |
| `U` or `D` | U-turn | D |
| `X` or `P` | **stop**, honoured from any running state | — |
| `G` | **start**: from IDLE (the MQTT image waits there after boot) or STOPPED | — |

Turn, straight and U-turn commands are carried out only while the car is
following the line (FOLLOWING or READING_BARCODE); in any other state they are
acked but ignored (`sub_nav.c` `on_command_rx`). Anything else is acked with
`"cmd":0` and ignored. A command is handed to the
car's controller (`RC_EVT_COMMAND_RX`) the moment the kernel delivers it, not
at the next telemetry tick.

## Setting up the broker

The kernel has no DNS, so the broker is given as an IPv4 address. Copy
`config/wifi_secrets.example.h` to `config/wifi_secrets.h` (it's gitignored,
so never committed) and fill it in:

```c
#define WIFI_SSID        "your-hotspot"
#define WIFI_PASSWORD    "your-password"
#define MQTT_BROKER_IP   "172.20.10.3"      /* the laptop running the broker */
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "robocar-01"       /* unique per car on a shared broker */
#define MQTT_TOPIC       "car/01/cmd"       /* must match RC_TOPIC_CMD (test S1-W03 checks it) */
```

The broker can be `scripts/mqtt_broker.py` on a laptop on the same hotspot
(the dashboard can start it, see `dashboard/README.md`), `mosquitto -v`, or a
public broker given by its IP. Put that IP in `MQTT_BROKER_IP`. Build with
`make NET=lwip MQTT=1 -j8`.

iPhone hotspot: turn on **Maximize Compatibility**, and keep the Personal
Hotspot screen open while the car joins. Otherwise the car sees no network
(`link_status` −2).

## Watching and driving from a laptop

```
python -I dashboard/dashboard.py --start-broker           # broker + dashboard, http://127.0.0.1:8080
python -I scripts/send_cmd.py X                           # stop (broker on this laptop)
python -I scripts/send_cmd.py G                           # start
```

`scripts/mqtt_broker.py` logs every frame as its decoded JSON plus the raw hex.
A generic client such as `mosquitto_sub` shows the raw binary frames; decode
one with `python -I scripts/tm_frames.py <hex>`. On the car's USB console the kernel also prints
`mqtt: RX [car/01/cmd] "X"` for each command.

## When it doesn't work

| You see | Likely cause |
|---|---|
| `status` has `"sink":"console"` | the MQTT buffers weren't created: not an `MQTT=1` build |
| nothing at the broker, console says `mqtt: connection status …` | wrong broker IP or port, or the hotspot has no internet |
| `"wifi":0` | wrong SSID/password, or the iPhone hotspot isn't visible (see above) |
| `fail` climbing while `"mqtt":1` | the queue to the kernel is full: something is sending far too often |
| `drop_slow` climbing | a SLOW-lane subscriber is blocking. Callbacks must return quickly. |
| a field never changes | that buddy's code isn't started, or isn't filling that field (`live` flags tell you which) |
| commands acked but the car does nothing | the command is not taken in the current state (see Commands), or the car is still in IDLE waiting for `G` |
| the dashboard shows nothing although the broker logs messages | the car image and `tm_frames.py` / the dashboard have different `TM_VER` |
