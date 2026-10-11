# Subsystem 1 — test log

**Subsystem:** WiFi communication, command and telemetry (Buddy 1)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## How to run

```
make TEST=1 -j8                                         # S1-U and S1-I tests (S1-W print SKIP)
make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=1  # everything, incl. S1-W (WiFi + broker)
```

The S1-W tests need `config/wifi_secrets.h` (copy from `wifi_secrets.example.h`;
`MQTT_TOPIC` must be `car/01/cmd`) and the WiFi hotspot on. Flash, open the USB
serial port, and look for `[TEST] subsystem1 SUMMARY`.

The S1-I tests need the event bus running (`rc_event_init()` before
`subsystem1_init()`). The other buddies' code does **not** need to be finished:
where Buddy 1 depends on someone, the test publishes that buddy's event itself.

## Test cases

Define a test here before running it. Test IDs never change.

**Brief task** = the Buddy 1 task in the Project Write-up that the test proves.

| ID | What is tested | Brief task / interface | Method | Expected result |
|---|---|---|---|---|
| S1-U01 | Console sink exists and reports itself up | Telemetry API | auto | `[TEST] S1-U01 PASS` |
| S1-U02 | Console sink accepts a publish | Telemetry API | auto | `[TEST] S1-U02 PASS` |
| S1-U03 | L/R/S/U/X decode to the right command, either case | Command subscription | auto | `[TEST] S1-U03 PASS` |
| S1-U04 | Barcode letters A/B/C/D (and P) decode like Buddy 3's barcodes | Command subscription | auto | `[TEST] S1-U04 PASS` |
| S1-U05 | Any other character decodes to "no command" | Command subscription | auto | `[TEST] S1-U05 PASS` |
| S1-U06 | A command reaches a registered callback unchanged | Command subscription | auto | `[TEST] S1-U06 PASS` |
| S1-U07 | Telemetry task runs at its TEAM.md priority (12) | TEAM.md §4 | auto | `[TEST] S1-U07 PASS` |
| S1-U08 | Telemetry task sleeps between ticks (no busy loop) | Telemetry API | auto | `[TEST] S1-U08 PASS` |
| S1-U09 | Barcode / hump / obstacle / impact events are accepted | Telemetry publishing | auto | `[TEST] S1-U09 PASS` |
| S1-U10 | An unrelated event is rejected and nothing is sent | Telemetry API | auto | `[TEST] S1-U10 PASS` |
| S1-U11 | Barcode message: exact topic + JSON | Message structures | auto | `[TEST] S1-U11 PASS` |
| S1-U12 | Hump message: exact topic + JSON | Message structures | auto | `[TEST] S1-U12 PASS` |
| S1-U13 | Obstacle message: exact topic + JSON | Message structures | auto | `[TEST] S1-U13 PASS` |
| S1-U14 | Impact message: exact topic + JSON | Message structures | auto | `[TEST] S1-U14 PASS` |
| S1-U15 | State message every 250 ms, no gaps in `seq` (timed by the kernel clock) | Telemetry publishing | auto | `[TEST] S1-U15 PASS` |
| S1-U16 | Heartbeat every ~2 s with active sink, tx/fail and drop counters | Heartbeat/status | auto | `[TEST] S1-U16 PASS` |
| S1-U17 | Every topic is under `car/01/` | Topics | auto | `[TEST] S1-U17 PASS` |
| S1-U21 | Message timestamps `t` keep real time (within 5 %) | Telemetry publishing | auto | `[TEST] S1-U21 PASS` |
| S1-U22 | State message carries every telemetry item in the brief (speed, encoder counts, line, state, barcode, hump, distance) | Message structures | auto | `[TEST] S1-U22 PASS` |
| S1-I01 | Event bus is running (a published event is delivered) | Shared layer | auto | `[TEST] S1-I01 PASS` |
| S1-I02 | Buddy 2 odometry event -> speed + distance in state | Telemetry publishing / B2 | auto | `[TEST] S1-I02 PASS` |
| S1-I03 | Buddy 3 line sample -> `line` field | Telemetry publishing / B3 | auto | `[TEST] S1-I03 PASS` |
| S1-I04 | Buddy 4 motion class -> `cls` field | Telemetry publishing / B4 | auto | `[TEST] S1-I04 PASS` |
| S1-I05 | Buddy 3 barcode decoded -> barcode message + `bc` field | Telemetry publishing / B3 | auto | `[TEST] S1-I05 PASS` |
| S1-I06 | Buddy 4 hump end -> hump message + highest `peak` | Telemetry publishing / B4 | auto | `[TEST] S1-I06 PASS` |
| S1-I07 | Buddy 4 impact -> impact message | Telemetry publishing / B4 | auto | `[TEST] S1-I07 PASS` |
| S1-I08 | Buddy 5 obstacle profile -> obstacle message | Telemetry publishing / B5 | auto | `[TEST] S1-I08 PASS` |
| S1-I09 | With no callback, a command goes out as `RC_EVT_COMMAND_RX` for sub_nav | Command subscription / nav | auto | `[TEST] S1-I09 PASS` |
| S1-I10 | Worst-case inputs: state fits its buffer and one MQTT message; no distance overflow | Message structures | auto | `[TEST] S1-I10 PASS` |
| S1-I11 | No event-bus drops on either lane | Heartbeat/status | auto | `[TEST] S1-I11 PASS` |
| S1-I12 | A second `rc_event_init()` doesn't cut Buddy 1 off the bus | Shared layer | auto | `[TEST] S1-I12 PASS` |
| S1-U18 | Dead link: stop sending, fail fast, keep retrying | Connection recovery | auto | `[TEST] S1-U18 PASS` |
| S1-U19 | Link back: telemetry resumes on its own | Connection recovery | auto | `[TEST] S1-U19 PASS` |
| S1-U20 | `set_sink(NULL)` falls back to the console | Connection recovery | auto | `[TEST] S1-U20 PASS` |
| S1-W01 | MQTT sink exists | MQTT communication | auto (MQTT=1) | `[TEST] S1-W01 PASS` |
| S1-W02 | Kernel message buffers exist with the agreed sizes (IF-12/IF-13) | MQTT communication | auto (MQTT=1) | `[TEST] S1-W02 PASS` |
| S1-W03 | Kernel subscribes to `car/01/cmd` (config matches `RC_TOPIC_CMD`) | Topics | auto (MQTT=1) | `[TEST] S1-W03 PASS` |
| S1-W04 | WiFi joins and broker connects within 30 s | WiFi connectivity | auto (MQTT=1) | `[TEST] S1-W04 PASS` |
| S1-W05 | Oversize message is rejected, not cut short | MQTT communication | auto (MQTT=1) | `[TEST] S1-W05 PASS` |
| S1-W06 | A message is handed to the kernel; `car/01/test {"test":"S1-W06"}` reaches the broker | MQTT communication | auto (MQTT=1) | `[TEST] S1-W06 PASS` |
| S1-W07 | Inbound `R` from the kernel buffer -> turn right | Command subscription | auto (MQTT=1) | `[TEST] S1-W07 PASS` |
| S1-W08 | Inbound garbage is ignored | Command subscription | auto (MQTT=1) | `[TEST] S1-W08 PASS` |
| S1-W09 | Command round trip car -> WiFi -> broker -> WiFi -> car | MQTT + command subscription | auto (MQTT=1) | `[TEST] S1-W09 PASS` |
| S1-M01 | Telemetry demonstration: a PC subscribed to `car/01/#` receives `state` every ~250 ms and `status` every ~2 s | Telemetry demonstration | manual (PC MQTT client) | state + status arrive continuously |
| S1-M02 | Dashboard commands: a PC publishes `L`/`X`/`B` to `car/01/cmd` | Command subscription | manual (PC MQTT client) | console shows `COMMAND_RX cmd=1/5/2` |

*Method:* **auto** = `TEST_CHECK` in `subsystem1_test.c`, result printed on the console;
**manual** = observed from outside (here a PC MQTT client on `broker.hivemq.com`).

## Results

Add a new row for every run. Never edit or delete old rows.

All runs: Robo Pico (Pico W), `make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=1`,
iPhone hotspot + `broker.hivemq.com`. Commit = `a7bbe74` (main) plus the local,
uncommitted changes listed under the table.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S1-U01–U22, S1-I01–I11, S1-W01–W09 | `a7bbe74`+local | Rexzer | 41 PASS, **1 FAIL** (S1-I10) | `logs/2026-10-11_S1_mqtt_test_run1_41of42.txt` | I10: worst-case state 195 chars > 191 buffer after adding encoder counts → raised `PAYLOAD_MAX` to 224. Also found PC commands stopped reaching the bus after the other subsystems' tests (Buddy 5's test re-runs `rc_event_init()`) → added S1-I12 |
| 2026-10-11 | S1-U01–U22, S1-I01–I12, S1-W01–W09 | `a7bbe74`+local | Rexzer | **43/43 PASS** | `logs/2026-10-11_S1_mqtt_test_after-flash.txt` | Run straight after flashing. Worst-case state 195 / 223 |
| 2026-10-11 | S1-U01–U22, S1-I01–I12, S1-W01–W09 | `a7bbe74`+local | Rexzer | **43/43 PASS** | `logs/2026-10-11_S1_mqtt_test_cold-boot.txt` | Run after a cold power-on (USB replug). S1-U21: 2080 ms kernel vs 2080 ms `t` — proves the `rc_time` fix |
| 2026-10-11 | S1-M01 | `a7bbe74`+local | Rexzer | PASS | `logs/2026-10-11_S1_mqtt_test_cold-boot.txt` (MQTT lines) | 408 `car/01/state` + `status` every ~2 s at the broker; car clock 106,340 ms over 106.1 s real |
| 2026-10-11 | S1-M02 | `a7bbe74`+local | Rexzer | PASS | `logs/2026-10-11_S1_mqtt_test_cold-boot.txt` | `L`→`COMMAND_RX cmd=1`, `X`→`cmd=5`, `B`→`cmd=2` |
| 2026-10-11 | S1-U01–U22, S1-I01–I12, S1-W01–W09 | `buddy1-fixes-with-other-buddies` (on `8954a06`) | Rexzer | **43/43 PASS** | `logs/2026-10-11_S1_pr-branch_buddy1-integration.txt` | PR branch: the changes below are now in the branch itself (no local test harness). Built in WSL with `make NET=lwip MQTT=1 TEST=1 -j8`. Other subsystems in the same run: S2 1/1, S3 1/1, S4 1/1, S5 10/10 |

**Local changes these runs depend on** (not on main yet — need the owners / team):

- `app_main.c` + `integration_test.c`: start the event bus (`rc_defer_init`, `rc_event_init`) before Phase 1.
- `rc_time.c`: set the watchdog tick to 12 so the µs timer is right after a cold power-on (was 42.7× slow).
- `rc_defer.c`, `rc_event.c`, `rc_gpioirq.c`: init safe to call twice.
- `subsystem2/3/4.c`: stop starting the deleted demo tasks (`4a05792`); restore `s2_make_value`, `s4_lines_received` for their tests.
- `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h`: Buddy 1's kernel MQTT patch (IF-12/IF-13).
