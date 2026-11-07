# Change Requirement 1: Minimum-Bit Data Representation (IS07)

This is the Week 13 evidence for Tribunal 1's Change Requirement 1, on branch `cr1-minbits-adc-barcode`. Nothing on the branch is committed yet.

The same branch also moves the barcode sensor from the digital DO pin (GP27) to the analogue AO pin (GP26) through the ADC (§7).

Evidence status:

| Item | Status |
|---|---|
| Build: 4 variants + MQTT | Clean on 2026-11-06 |
| Host tests | Run and passing |
| On-car tests | **Not run yet.** No car on 2026-11-06. §6 lists what is still owed. |

## 1. What changed

| Area | Change | Files (reviewer) |
|---|---|---|
| Shared payloads | Every event payload field is a C bit-field of its minimum width; values are clamped (`RC_SAT_U` / `RC_SAT_S`) before they are stored | `app_program/rc_types.h` (team) |
| Fault bitmask | New shared bitmask in a µT-Kernel **event flag**: one bit per fault, set and cleared atomically | `app_program/rc_fault.c/.h`, `rc_fault_test.c` (team; Buddy 5 integration) |
| MQTT payloads | Every topic is a compact binary frame: a 2-byte header, then each field in its exact bit width. Commands in stay one ASCII byte | `subsystem1/tm_frame.c/.h`, `sub_telemetry.c`, `subsystem1_test.c` (Buddy 1) |
| Laptop tools | Broker logs each frame as decoded JSON, then a tab, then the raw hex. Dashboard decodes; adds nav state, faults and frame size | `scripts/tm_frames.py`, `scripts/mqtt_broker.py`, `dashboard/dashboard.py`, `dashboard/index.html` (Buddy 1 / Buddy 5) |
| Motion | Speeds 11-bit, odometer 16-bit, move id 15-bit; stall and encoder fault bits | `subsystem2/sub_motion.c`, `drv_encoder.c` (Buddy 2) |
| Line, barcode | Line sample 2 bits (position derived); barcode symbol 5-bit code; ADC barcode | `subsystem3/drv_ir.c/.h`, `sub_line.c`, `sub_barcode.c`, `subsystem3_test.c` (Buddy 3) |
| IMU, terrain | Accelerometer 12-bit; hump 7/13/8 bits; IMU and impact fault bits; magnetometer off the bus | `subsystem4/drv_imu.c`, `sub_terrain.c` (Buddy 4) |
| Obstacle | Range 12-bit with 0 meaning no echo (`valid` flag removed); angle 8-bit; profile and plan packed; no-echo fault bit | `subsystem5/drv_ultrasonic.c`, `sub_scan.c`, `subsystem5_test.c`, `sub_nav.c` (Buddy 5) |
| Config | `DEVCNF_USE_ADC 1`; barcode ADC pins and threshold settings | `config/config_device.h`, `rc_config.h`, `rc_sense.c/.h`, `TEAM.md` (team) |

## 2. Data representation table

The table covers every application field in the CR1 scope:
- **Width** is the number of bits stored.
- **Before** is the C type it had at `dba7f92`.

| Field | Valid range | Width (bits) | Implementation | Before |
|---|---|---|---|---|
| Wheel speed L/R | ±1023 mm/s (car max ≈ 530) | 11 signed | `rc_pl_odometry_t` bit-field, `RC_SAT_S` in `sub_motion.c` | `int32_t` |
| Odometer L/R | 0..65535 mm, wraps (per move) | 16 | `uint16_t` (exact type fits) | `uint32_t` |
| Encoder count L/R | 0..2³²−1 | 32 | Unchanged by team decision: single-writer ISR words, read without a lock | `uint32_t` |
| Encoder edge gap | 0..131071 µs (stall timeout 100 ms) | 17 | `rc_pl_encoder_t` bit-field | `uint32_t` |
| Motor duty L/R | −1000..1000 ‰ | 11 signed | Telemetry frame field | `int16_t` |
| Move id | 1..32767, wraps | 15 | `rc_pl_motion_done_t`; `sub_motion` wraps ids at 2¹⁵ | `uint32_t` |
| Move completed | 0/1 | 1 | Bit-field | `bool` (8) |
| Move distance | 0..65535 mm | 16 | Bit-field, clamped | `uint32_t` |
| Line sensors L/R | 0/1 each | 2 | `rc_pl_line_t` `on_line_l:1, on_line_r:1`; position from `rc_line_position()` | 2×`bool` + `int16_t` position |
| Barcode symbol | none, `*`, A..Z (28 values) | 5 | `sym:5`, `rc_sym_code()` / `rc_sym_char()` | `char` (8) |
| Barcode command | `rc_nav_cmd_t`, 7 values | 3 | `command:3` | enum (8) |
| Barcode reversed | 0/1 | 1 | Bit-field | `bool` (8) |
| Bar/space width | 0..2³¹−1 µs | 31 | `rc_pl_bar_edge_t` `width_us:31`, `level_high:1` | `uint32_t` + `bool` |
| Accel x/y/z | ±2047 mg (LSM303D ±2 g) | 12 signed | `rc_pl_imu_t` bit-fields, clamped in `drv_imu.c` | `int16_t` |
| Magnetometer x/y/z | (no subscriber) | 0 | Removed from the event | 3×`int16_t` |
| Hump peak | 0..127 mm | 7 | `rc_pl_hump_t`, clamped in `sub_terrain.c` | `uint16_t` |
| Hump duration | 0..8191 ms (`HUMP_MAX_MS` 5000) | 13 | Bit-field | `uint32_t` |
| Hump max pitch | ±127° | 8 signed | Bit-field | `int16_t` |
| Height above start (live, ramp) | ±511 mm | 10 signed | state frame `h`, `sub_terrain_height_mm()` | (new, 6 Nov) |
| Ramp rise / net / steepest / length | 0..511 mm / ±511 mm / 0..63° / 0..4095 mm | 9 / 10 s / 6 / 12 | `rc_pl_ramp_t`, clamped in `sub_terrain.c` | (new, 6 Nov) |
| Motion class | 8 classes | 3 | `cls:3` | enum (8) |
| Ultrasonic range | 1..4095 mm, 0 = no echo | 12 | `rc_pl_ultra_t` `range_mm:12`, `RC_ULTRA_VALID()` | `uint16_t` + `bool valid` |
| Servo angle | 0..180° | 8 | `angle_deg:8` | `int16_t` |
| Scan points | 0..63 (saturates) | 6 | `rc_pl_profile_t` | `uint8_t` |
| Closest / width | 0..4095 mm | 12 each | Bit-fields, clamped in `sub_scan.c` | `uint16_t` |
| Side clearance L/R | 0..1023 mm (more counts as clear) | 10 each | Bit-fields, clamped | `uint16_t` |
| Plan action | `rc_nav_cmd_t` | 3 | `rc_pl_plan_t` | enum (8) |
| Plan lateral / forward | 0..4095 mm | 12 each | Bit-fields, clamped | `uint16_t` |
| Remote command | `rc_nav_cmd_t` | 3 | `rc_pl_command_t` `command:3` | enum (8) |
| Command arg | ±2²⁸ (always 0 today) | 29 signed | Bit-field | `int32_t` |
| Link state | WiFi 0/1, broker 0/1 | 2 | `rc_pl_link_t` 1-bit each | 2×`bool` |
| Nav state | 7 states | 3 | Telemetry field `nav` (`sub_nav_state()`) | (not sent) |
| Fault bitmask | 7 faults | 8 (7 used) | Event flag (`rc_fault.h`); telemetry field `flt` | (none) |
| Barcode diagnostics (`bcscan`, `bcbits`, `spin`) | 8-bit counters (saturate at 255), 16-bit 10 µs widths | As listed | Already byte-packed; on MQTT as exact-width frames | (same) |

### 2.1 Fault bitmask (`rc_fault.h`)

| Bit | Fault | Owner | Set when | Cleared when |
|---|---|---|---|---|
| 0 | MOTOR_STALL | Buddy 2 `sub_motion.c` | A wheel pushed at ≥ 500 ‰ shows no movement for 0.6 s, and the other wheel is not turning either | The next move starts |
| 1 | ENCODER | Buddy 2 `sub_motion.c` | The same, but the other wheel **is** turning, so this encoder is most likely dead | The next move starts |
| 2 | IMU | Buddy 4 `drv_imu.c` | IMU not ready, or a read failed | A read succeeds |
| 3 | NO_ECHO | Buddy 5 `drv_ultrasonic.c` | The latest ping timed out or was out of range | A valid echo |
| 4 | WIFI_LOSS | Buddy 1 `sub_telemetry.c` | WiFi or broker down (MQTT sink) | Link back up |
| 5 | IMPACT | Buddy 4 `sub_terrain.c` | Impact detected (FR13, 1500 mg unchanged) | START command (`sub_nav.c`) |
| 6 | BARCODE_ADC | Buddy 3 `drv_ir.c` | ADC open failed at boot | Never |

## 3. Race safety (packed shared data)

1. **Payload bit-fields are never shared live.** A publisher fills a private `rc_event_t` and `rc_event_publish()` copies it whole into the ring under the ring's lock (`rc_event.c`). Subscribers get their own copy. No two tasks or cores ever do a read-modify-write on the same packed word.
2. **The fault bitmask is a kernel event flag, not a C variable.** `tk_set_flg` ORs and `tk_clr_flg` ANDs inside the kernel's own lock, so every bit is changed atomically.
   - This matters on the RP2040: its Cortex-M0+ has no LDREX/STREX, so a C `mask |= bit` from two cores or an ISR could lose an update.
   - Setting a bit from an ISR is allowed; clearing is task-only.
   - CR1-T2 (§5) stresses this from two cores plus a handler.
3. **Telemetry's cached state has one writer per variable.** Each variable is written by one callback and read by the telemetry task, and each is a byte or halfword (one store, never torn on the M0+).
   - The two line bits share **one** byte, written by the one `on_line` callback.
   - The frame seq for events is taken under the existing count lock.
4. **MQTT frames are packed field by field with shifts**, never by casting a struct onto the buffer. The bytes do not depend on the compiler's bit-field order, and there are no unaligned accesses (risk R5).

## 4. MQTT telemetry

### 4.1 Frame format (`subsystem1/tm_frame.h`)

| Bytes | Content |
|---|---|
| 0 | `[ver:3 \| type:5]`: `TM_VER` = 1; type 1..15 |
| 1 | seq, 8 bits, wraps. State frames count state ticks; all other frames share one event counter. |
| 2.. | The type's fields, back to back. Least significant bit first: body bit *i* is bit *i % 8* of body byte *i / 8*. Signed fields are two's complement in their own width. Out-of-range values are clamped. |

- **The schema** (field names, widths, order) is one table in `tm_frame.c`, with an identical table in `scripts/tm_frames.py`.
  - A decoder refuses a frame whose version or length is wrong instead of misreading it (risk R4).
  - The host cross-check (§5) proves the two tables agree.
- **Topics** are unchanged (`car/01/<leaf>`).
- **Commands** stay one ASCII byte, because the kernel's receive path is text-only.

### 4.2 Message size, before vs after (payload bytes)

**Before:** JSON, measured from 15,694 messages in `test/wifi/mqtt_run_20261031_1638.log` (31 Oct track session).

**After:** fixed frame sizes from the schema. The car only sends whole frames, so every message of a type is exactly that size.

| Topic | Messages (31 Oct) | Before, avg B | After, B | Saved |
|---|---|---|---|---|
| `state` (4 Hz) | 12884 | 145.2 | 20 / 28 (avg 22) | 85% |
| `status` (0.5 Hz) | 1605 | 126.5 | 20 | 84% |
| `motion` | 722 | 9.0 | 3 | 67% |
| `move` | 117 | 23.9 | 6 | 75% |
| `cmd` (inbound) | 70 | 1.0 | 1 | 0% |
| `ack` | 69 | 18.0 | 4 | 78% |
| `hump` | 61 | 33.3 | 6 | 82% |
| `line` | 58 | 10.0 | 3 | 70% |
| `obstacle` | 16 | 63.0 | 10 | 84% |
| `avoid` | 16 | 34.3 | 6 | 83% |
| `spin` | 16 | 63.0 | 15 | 76% |
| `bcbits` | 14 | 86.8 | 16 | 82% |
| `bcscan` | 14 | 132.5 | 15 | 89% |
| `barcode` | 10 | 27.0 | 4 | 85% |
| `impact` | 2 | 12.0 | 3 | 75% |
| **Whole session** | 15694 | **2,092,936 B** | **320,278 B** | **84.7%** |

Notes on the `state` stream:
- It drops from ~581 B/s to ~88 B/s of payload. Every 4th frame (1 Hz) is the 28 B form that carries the two 32-bit encoder counts.
- Since 6 Nov it also carries `h`, the live height (10 bits, §8); that added 1 B per state frame (session total was 307,394 B / 85.3% without it). The `ramp` frame (7 B) has no 31 Oct count because it did not exist then.
- It now also carries the nav state and the fault mask, which the JSON never had.
- The MQTT topic string and header (~16 B per message) are the same before and after.

### 4.3 State frame layout (type 1, 20 B)

| Field | Bits | Field | Bits |
|---|---|---|---|
| t (ms since boot) | 32 | line L/R | 2 |
| spd_l, spd_r | 11 + 11 signed | cls | 3 |
| dist | 16 | peak | 7 |
| m_l, m_r | 11 + 11 signed | rng | 12 |
| bc | 5 | nav | 3 |
| flt | 8 | h (height mm) | 10 signed |
|  |  | **body** | **142 bits → 18 B** |

The other frame layouts are in the `tm_frame.c` table.

## 5. Correctness and performance evidence

| Test | What it shows | Where | Result |
|---|---|---|---|
| T1 host | Each of the 24 T1 cases round-trips through `tm_pack`/`tm_unpack`. Covers all 16 frame types, every field at its min and max, and one past each end (which must come back clamped). Each frame is the documented size; a bad version or length is refused. | `test/cr1/host/t1_host.c` (built from the table in `subsystem1_test.c`) | **24/24 pass** (2026-11-06, incl. ramp + `h`) |
| C ↔ Python cross-check | 2000 random frames of all 16 types are packed by the car's C code; every field the C side renders must match what `tm_frames.py` decodes | `test/cr1/host/harness.c`, `crosscheck.py` | **2000 frames, 0 mismatches** |
| Python round trip | `decode(encode(d)) == d` for a sample of every topic | `test/cr1/py_roundtrip.py` | **16/16** |
| Laptop end to end | Broker plus fake car send binary frames; the broker logs JSON plus hex; the dashboard parser rebuilds the same dicts | `test/cr1/e2e_broker.log` | **Pass** |
| T1 on car (S1-U30) | Same 24 cases on the RP2040 | `subsystem1_test.c` | Owed (TEST build) |
| S1 suite (S1-U/I) | The capture sink decodes every frame back to the pre-CR1 JSON, so the existing exact-text checks keep running. S1-I10 now checks clamping: speeds → −1024, hump 127, range 4095. | `subsystem1_test.c` | Owed |
| T2 (CR1-T2) | Task A on core 0 and task B on core 1 each set and clear their own fault bit 10,000×, checking every read-back. A cyclic handler sets a third bit at the same time, and every one of its sets must be seen. | `rc_fault_test.c` | Owed |
| T3 (CR1-T3) | Sense task over 3 s with the ADC read added: 0 missed 10 ms periods, ADC read + threshold < 1 ms. Also compare line/IMU timing against the 18 Oct numbers. | `rc_fault_test.c` | Owed |
| T4 | STOP over MQTT while driving; IMPACT shows `flt` bit 5 on the dashboard; START clears it | Track | Owed |

### 5.1 RAM before vs after

Measured with `test/cr1/measure.sh` (`arm-none-eabi-size` plus a per-symbol `nm` diff):

| Image | Static RAM before (data+bss) | After | Change |
|---|---|---|---|
| `smp0_usb_cdc` | 156 + 26848 = 27,004 B | 168 + 26864 = 27,032 B | +28 B |
| `smp1 … lwip_mqtt` (track image) | 228 + 89792 = 90,020 B | 240 + 89816 = 90,056 B | +36 B |

Per symbol, in the track image (`test/cr1/ram_diff.txt`):
- **The CR1 packing saved 22 B net** of application state: 25 B removed, 3 B added.
  - Removed: telemetry's cached readings (4 B → 1-2 B each), the 32-bit `seq`, the nav `plan` copy (6 → 4 B) and `link_now`.
  - Added: the two 8-bit frame seqs and the packed line byte.
- **The ADC and the fault module added 47 B:**
  - the kernel ADC driver's control blocks (24 B);
  - the barcode threshold state (17 B);
  - the fault flag id (4 B);
  - two 1-byte change trackers.
- Net: +25 B.

Static RAM is dominated by task stacks and the event rings, so the effect there is small. **The event slot stays 24 B** because the largest union members are the three 16 B diagnostic payloads (`bcscan`, `bcbits`, `spin`). Every control payload is now ≤ 8 B, so dropping or shrinking those three would make the slot 16 B and save 1 KB across the two 64-slot rings. Event-ring packing was deferred in the team plan; this is the measurement for that decision.

What did shrink, per copy:

| Payload | Before | After |
|---|---|---|
| odometry | 16 B | 8 B |
| hump | 12 B | 4 B |
| imu | 12 B | 8 B |
| profile | 12 B | 8 B |
| command | 8 B | 4 B |
| plan | 6 B | 4 B |
| ultra | 6 B | 4 B |
| bar_edge | 8 B | 4 B |
| line | 4 B | 1 B |
| barcode | 3 B | 2 B |
| link | 2 B | 1 B |
| motion_done | 12 B | 4 B |
| encoder | 12 B | 8 B |

Telemetry stack per send went from a 224 B JSON buffer to a 32 B frame plus a 60 B value array. The MQTT TX message buffer (`s1_mbf_tx`) now holds about 5× as many state messages.

Flash grew by 4,968 B (SMP0) and 5,332 B (track image), from the frame codec, the fault module, the ADC driver and the tests.

## 6. Still owed (needs the car)

1. Flash the **TEST** image and log COM17:
   - S1-U30, S1 suite, CR1-T2, CR1-T3;
   - S3 (barcode sensor hand test now on the ADC);
   - S5.

   Commit the log under `testing/`.
2. Flash the **MQTT** image and run the track with the dashboard:
   - state frames are 20/28 B (dashboard "State frame" row);
   - ramp height (§8): drive over a ramp, compare the dashboard's Rise with a ruler;
   - nav state and fault names show;
   - T4: STOP, IMPACT → START.
3. ADC barcode on the car (§7): sensor at 2 mm; the peak/valley detector and the `*`-first rule still need a track run.
4. Fill the T-rows in §5 with the log results, and add the report's data representation table (Week 13 doc) from §2.

## 7. Barcode sensor on the ADC

Barcode reading now uses the analogue output instead of the comparator output:
- **Pins:** the AO pin (GP26, ADC0) is read by the Sense task every 10 ms. GP27 (DO) is no longer configured and has no IRQ, so its trim pot does not matter.
- **Edge detector (peaks and valleys, not one threshold):**
  - A bar starts once the reading rises `RC_BAR_ADC_PROM` (400 counts) above the lowest point of the space before it; a space starts once it falls 400 below the bar's peak.
  - A bar only counts if its peak is `RC_BAR_ADC_MIN_BAR` (800) above the white level (running minimum, creeping up 1 count per sample), so mat texture is not read as bars.
  - Why: the sensor spot is about as wide as a 3 mm element, so a narrow space between two bars only dips to ~2200-2900 while bars read ~3600-3950 and paper ~1050-1700. A midpoint threshold (~2500 ± 375) caught those dips on some passes and merged bars on others.
- **Edge timing:** each edge is timed where the reading crosses halfway between the peak and the valley either side of it, interpolated between two samples from a 64-sample history. Edges are therefore sent one element late; a bar's falling edge is sent after 300 ms of space at the latest.
  - The old 10 ms level-debounce now rejects only a one-sample speck (15 ms).
  - Edges go through the same ring → defer task → `RC_EVT_BARCODE_EDGE`, so `sub_barcode.c`'s decoder is unchanged.
- **Timing margin:** a narrow element is about 95 ms at 31 mm/s, so 10 ms sampling gives roughly 10 samples per element.
- **Verified on the car (6 Nov, track speed ~33 mm/s):** black reads high (`RC_BAR_ADC_BLACK_HIGH` 1). With the sensor **2 mm** above the mat and the midpoint threshold, small (3 mm / 7 mm bars) `*H*` and large (4 mm / 14 mm) `*E` decoded at 19:10-19:12, but the 20:40-20:50 run read mostly wrong letters (E, A, G, X, S, C, Q, I, N, W, L), one of them acted on as a left turn.
- **Replay of the logged samples** (`test/bartrace/host_peak.py` builds the real `drv_ir.c` sampler on the PC; `sim_car.py` follows `sub_barcode.c`):

  | Passes | Midpoint, any letter | Peak/valley, `*` first |
  |---|---|---|
  | 19:10-19:12 (2 passes) | `*H*`, `*E*` | `*H*`, `*E*` |
  | 20:40-20:50 (all passes) | 1 full read, ~15 false letters | `*E*`, `*E`, `*Y`, `*`; no false letters |

  Not yet run on the car.
- **Decoder:** `sub_barcode.c` no longer reports a letter before a `*` has been read (forwards or backwards); before this, the first letter-shaped window of a pass, often one straddling two characters, was reported and acted on.
- **Sensor height matters more than the threshold.** Raw ADC traces (a temporary `car/01/bctrace` topic, `test/bartrace/`) showed:

  | Height | Mat | Paper | 3 mm bar | 4 mm bar | 7 mm bar | 14 mm bar |
  |---|---|---|---|---|---|---|
  | 5 mm | ~220 | ~220 | not seen | +100…+650 | ~2000 | ~2700 |
  | 2 mm | ~250 | ~1050-1700 | ~3400 | — | ~3950 | ~3950 |

  At 5 mm the narrow bars never reached the midpoint, so passes decoded as wide bars only (`000101010`). A threshold just above white caught the 4 mm bars but not the 3 mm ones, and at 2 mm it read the barcode paper (darker than the mat) as one long bar. Keep the sensor at 2 mm. The DO trim pot does not affect any of this (AO is read directly).

## 8. Ramp height from the accelerometer (added 2026-11-06)

**Formula.** Each 10 ms IMU sample, the wheels report a ground distance ds. On a slope of angle θ that piece rises ds·sin θ, so

    h = Σ ds_k · sin θ_k,     sin θ = g_x / √(g_x² + a_z²),     g_x = a_x − a_fwd

- `a_fwd` is the car's own forward acceleration, from the encoders: the distance in the last 100 ms minus the 100 ms before, over (100 ms)². Without it, speeding up or braking would look like a slope.
- Dividing by √(g_x² + a_z²) instead of 1000 mg cancels any accelerometer scale error, and needs no trig (integer square root only; the M0+ has no FPU).
- This integrates once, over distance, so it does not drift with time. `drv_imu.h` warns against double-integrating acceleration; this is not that.
- **Level zero:** while driving on the level, the slope the car still reads (IMU mounting, the chassis squatting under drive torque) is learned and taken off. It is learned only within ±2° of the current zero and with a 256-sample (~2.6 s) time constant, so the foot of a ramp is not learned as level.
- **Start:** a slope starts at |θ| ≥ 4° but only counts once it has lasted 25 mm (`RAMP_CONFIRM_MM`). If |θ| drops back first, it was a rock or a bump: its height is undone.
- **End:** the slope ends after 60 mm with |θ| < 4°. Then an `RC_EVT_RAMP_END` is sent with rise (highest point above where the slope began), net (where it ended), steepest angle and length. A slope that ends within 10 mm of the start level re-zeroes the height. A slope still open after 60 s is dropped.
- **Why these rules (6 Nov track run):** the first version started a slope on the chassis rock after START, then kept integrating a steady −1.7° reading on flat track (−7 to −9 mm per run), because it only treated |θ| ≤ 2° as level. Its 10 s timeout was also shorter than a long slope takes at 33 mm/s.

**Where.** `subsystem4/terrain_ramp.c/.h` (pure C, Buddy 4), called from `sub_terrain.c` `on_sample()`. `sub_terrain_height_mm()` → state frame `h` (10 bits). `car/01/ramp` frame (type 16, 7 B). Dashboard IMU card: "Height now", "Highest ramp", a 60 s height chart and a ramp table. TM_VER is now 2.

**Host simulation** (`test/ramp/ramp_host.c`: 80 mm wheelbase, 100 Hz, 333 µm/tick encoders, ±15 mg noise; "track" rows: 33 mm/s, −30 mg tilt reading, 6° rock after START, ±80 mg noise; output in `test/ramp/ramp_host_after.txt`):

| Profile | True | First version | Now |
|---|---|---|---|
| Flat, start/stop and 530 mm/s, noise | 0 | no report, h 0 | no report, h 0 |
| 15° ramp up, 300 mm plateau, down | 51.8 mm | rise 50, back to 0 | rise 50, back to 0 |
| Same, car slows 40% on the climb | 51.8 mm | rise 50 | rise 50 |
| 10° ramp up and stay | 52.1 mm | rise 50 | rise 50 |
| 10° ramp up and stay, 33 mm/s, clean | 52.1 mm | (not run) | rise 49 |
| 30 mm triangular hump, 120 mm sides | 29.9 mm | rise 22 | rise 22 |
| Track: flat | 0 | h sinks to −7 | no report, h 0 |
| Track: 30 mm hump | 30.0 mm | no report (timeout), h −11 | rise 22, back to 0 |
| Track: 15° ramp up/down | 51.8 mm | no report (timeout), h −60 | rise 50, back to 0 |
| Track: 10° up and stay | 52.1 mm | no report (timeout) | rise 47 |

Short humps read low because the chassis bridges the peak (80 mm between axles) and the 100 ms window smooths the tilt. The hump event keeps its own `pitch_to_height_mm()` estimate. On the car, compare both on the same hump.

**Cost:** +224 B static RAM (the 21-sample window `ramp`, 216 B) and ~1.1 KB flash (`ramp_step` 988 B), track image. Static RAM is now 90,280 B (§5.1 numbers are before this).

**Owed (car):** ramp polarity (nose-up must read +a_x; `drv_imu_pitch_ddeg()` already assumes that), a ruler check of rise on the track ramp, and the S1-I21 check in the TEST run.
