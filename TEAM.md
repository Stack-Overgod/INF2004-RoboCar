# Team and subsystems

Filled from the Week 6 Design Review (`Week_6_Design_Review_IS07.docx`):
§0 team table, §2 pin table, §3 task table/priority rationale, §4 interfaces
(event bus) and §6.5 mission state machine. GitHub usernames are left blank
for each owner to fill. This is a **shared file** — change it by team
agreement / PR review.

It is the single place that says **who owns what**, so that five people can
work in one firmware image without clashing over pins, priorities, or kernel
objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi comms, command & telemetry (binary frames `tm_frame.c`, 4 Hz state publish, console/MQTT sinks, command subscription, link-state heartbeat & reconnect) | 2503331 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control (PWM motor driver, encoder odometry, 50 Hz PID, queued move/turn API) | 2501302 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following & Code 39 barcode decoding (calibration, P-steer, junction detect, A/B/C/D nav commands) | 2500336 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU terrain & motion monitoring (LSM303D, pitch, hump FSM & peak height, lift/overturn detect, 8-state motion class) | 2503302 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic scanning & obstacle avoidance (non-blocking HC-SR04, forward watch, step scan with servo and chassis, obstacle profile, avoidance plan, line recovery). Also integration owner for `sub_nav` (`app_program/sub_nav.*`) + shared core. | 2500347 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`,
`app_program/rc_config.h`, the shared core `rc_event/rc_defer/rc_gpioirq`,
`sub_nav`, this file) are changed only by agreement of the team. The
exception is claiming an empty row in this file (a pin, a peripheral, an
interface ID), which anyone may do (section 2).

Pin numbers and tuning constants have a single source of truth in
`app_program/rc_config.h` (NFR10); this table mirrors it, it does not
replace it.

## 2. Pins

One owner per pin. Source of truth: report §2 (TABLE 7) and `rc_config.h`.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio (CYW43439) |
| GP0 | S2 (Buddy 2) | `ENC_L_A` left encoder A (Grove 1). Also UART0 TX — build must be `CONSOLE=usb_cdc` |
| GP1 | S2 (Buddy 2) | `ENC_L_B` left encoder B (Grove 1). Also UART0 RX |
| GP2 | S5 (Buddy 5) | `ULTRA_TRIG` HC-SR04 trigger (Grove 2), GPIO out + TIMER alarm 1 |
| GP3 | S5 (Buddy 5) | `ULTRA_ECHO` HC-SR04 echo (Grove 2), both-edge IRQ + TIMER alarm 2. **5 V — through 1k/2k divider** |
| GP4 | S4 (Buddy 4) | `I2C0_SDA` IMU (Grove 3), I2C0 @ 400 kHz |
| GP5 | S4 (Buddy 4) | `I2C0_SCL` IMU (Grove 3) |
| GP6 | S3 (Buddy 3) | `IR_LINE_R` right line sensor DO (Grove 5). Leave Grove-5 AO (GP26) unconnected |
| GP7 | S2 (Buddy 2) | `ENC_R_A` right encoder A (Grove 7) |
| GP8 | S2 (Buddy 2) | `MOTOR_R_A` M1A, PWM slice 4 ch A (Robo Pico MOTOR 1) |
| GP9 | S2 (Buddy 2) | `MOTOR_R_B` M1B, PWM slice 4 ch B |
| GP10 | S2 (Buddy 2) | `MOTOR_L_A` M2A, PWM slice 5 ch A (Robo Pico MOTOR 2) |
| GP11 | S2 (Buddy 2) | `MOTOR_L_B` M2B, PWM slice 5 ch B |
| GP12 | | free (servo port 1) |
| GP13 | | free (servo port 2) |
| GP14 | | free (servo port 3) |
| GP15 | S5 (Buddy 5) | `SERVO_SCAN` scan servo (Robo Pico servo port 4), PWM slice 7 ch B, 50 Hz |
| GP16 | S3 (Buddy 3) | `IR_LINE_L` left line sensor DO (Grove 4) |
| GP17 | | free |
| GP18 | | free |
| GP19 | | free |
| GP20 | | free |
| GP21 | | free |
| GP22 | | free |
| GP26 | S3 (Buddy 3) | `IR_BARCODE_A` barcode analogue AO (Grove 6), ADC0. With `RC_BAR_ADC_FIFO` 1 the ADC free-runs at `RC_BAR_ADC_HZ` × `RC_BAR_ADC_OVERSAMPLE` into its FIFO (`drv_ir.c`); peak/valley edge detector, no trim pot |
| GP27 | S3 (Buddy 3) | Barcode digital DO (Grove 6) is wired but **deliberately not configured, no IRQ** since the ADC change. Keep it free of other signals |
| GP28 | S2 (Buddy 2) | `ENC_R_B` right encoder B (Grove 7) |

## 3. Peripherals

Each peripheral is driven by exactly one task, its *owning task*. Source:
report §2/§9 (TABLE 7, TABLE 8).

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 | S4 (Buddy 4) | Sense task | GP4/GP5 (re-pinned from GP8/GP9) |
| PWM slice 4 (motors R) | S2 (Buddy 2) | Motion task | GP8/GP9 |
| PWM slice 5 (motors L) | S2 (Buddy 2) | Motion task | GP10/GP11 |
| PWM slice 7 ch B (servo) | S5 (Buddy 5) | Scan task | GP15 |
| TIMER alarms 1 & 2 | S5 (Buddy 5) | drv_ultrasonic (ISR) | — (HC-SR04 trig end + echo timeout) |
| ADC (channel 0) | S3 (Buddy 3) | ADC FIFO interrupt (`drv_ir.c`), edge detector in the Defer task (`bar_samples_drain`); Sense task only when `RC_BAR_ADC_FIFO` is 0 | GP26 |
| IO_BANK0 GPIO IRQ (shared) | *shared core* `rc_gpioirq` | demuxed to S2 encoders (channel A), S5 echo | GP0/GP7/GP3 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

PWM slice 6 is used by the kernel physical-timer option when `USE_PTMR=1`;
this build keeps `USE_PTMR=0`. The PWM block is taken out of reset on first
use by `rc_pwm_init_pin()` (shared `rc_pwm.c`), used by both motors (S2) and
the servo (S5).

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions.** Read connection state
with `cyw43_utk_get_status()`; send/receive by having the MQTT service task
(`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`) drain a student-owned
message buffer with `TMO_POL`. That shared-library change (IF-12/IF-13 below)
needs team **and instructor** agreement and the `NET=lwip MQTT=1` build.

## 4. Task priorities

This project does **not** use per-subsystem priority bands. Priorities are a
single shared scheme ordered by what must never wait on what (report §3,
TABLE 8/9), defined once in `rc_config.h`. 1 is highest. The kernel's USB
console task (3) sits above all application tasks; its WiFi task
(`CYW43_TASK_PRIORITY` 4) has the same priority as Defer.

| Task | Owner subsystem | Priority (`rc_config.h`) | Core | Period / trigger |
|---|---|---|---|---|
| Defer (bottom halves) | shared core (`rc_defer`) | `RC_PRI_DEFER` = 4 | any | ISR-posted flag |
| FAST dispatcher | shared core (`rc_event`) | `RC_PRI_DISPATCH_FAST` = 5 | any | event flag on publish |
| Sense (line + IMU) | shared (`rc_sense.c`, Buddy 5 integration); polls S3's `drv_ir` + S4's `drv_imu` | `RC_PRI_SENSE` = 6 | — | periodic 10 ms (line and IMU) |
| Motion | S2 (Buddy 2) | `RC_PRI_MOTION` = 7 | — | periodic 20 ms (50 Hz) |
| Scan | S5 (Buddy 5) | `RC_PRI_SCAN` = 8 | any | request semaphore + result flag; one ping per 60 ms |
| SLOW dispatcher | shared core (`rc_event`) | `RC_PRI_DISPATCH_SLOW` = 9 | any | event flag on publish |
| Telemetry | S1 (Buddy 1) | `RC_PRI_TELEMETRY` = 12 | — | periodic 250 ms + out-of-band |
| Initial task (`usermain`) | shared (`app_main.c`) | port default | — | boot, then `tk_slp_tsk(TMO_FEVR)` |

Rationale (report §3): fixed-priority preemptive scheduling, priorities
assigned deadline-monotonically (shorter deadline = higher priority). Defer
above all (finishes ISR work, incl. the barcode ADC samples); FAST pre-empts
the periodic loops so a steer never queues behind a PID cycle; Sense (10 ms)
above Motion (20 ms); the motion mutex is `TA_INHERIT`, so Motion holding it
is raised above Sense while FAST waits (no priority inversion); Scan below
both (it mostly waits on echoes); SLOW + Telemetry are last to get the CPU
(NFR6b).

## 5. Interfaces between subsystems

Almost everything crosses boundaries through the **event bus**
(`rc_event`): a producer fills an `rc_event_t` and publishes; the bus copies
it onto a per-lane 64-entry ring and the lane's dispatcher calls subscribers
(report §4). Subsystems otherwise never call each other, with the few direct
API calls noted below (sub_nav/sub_line → sub_motion; sub_nav → sub_scan).

| ID | From → To | Mechanism | Object / event name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → all | event (FAST) | `RC_EVT_ODOMETRY` | speed, encoder counts, distance; every 20 ms (NFR2) | team |
| IF-02 | S3 → all | event (FAST) | `RC_EVT_LINE_SAMPLE` | line position / status; every 10 ms (`RC_PERIOD_LINE_MS`) | team |
| IF-03 | S3 → all | event (FAST) | `RC_EVT_BARCODE_DECODED` | decoded A/B/C/D nav command | team |
| IF-04 | S4 → all | event (FAST) | `RC_EVT_IMU_SAMPLE`, `HUMP_BEGIN/END`, `RC_EVT_LIFTED` | pitch/accel; hump + lifted/overturned safety stop | team |
| IF-05 | S5 → all | event (FAST) | `RC_EVT_ULTRA_RESULT` | range (mm) + valid, tagged with servo angle | team |
| IF-06 | S5 → all | event (FAST) | `RC_EVT_OBSTACLE_PROFILE`, `RC_EVT_AVOIDANCE_PLAN` | bearing/width/closest/clearance; chosen action | team |
| IF-07 | sub_nav/sub_line → S2 | direct API + callback | `sub_motion_drive/move/turn`, `MOTION_DONE` | queued move; completion callback | team |
| IF-08 | sub_nav → S5 | direct API + event (FAST) | `sub_scan_start_measure()`, `sub_scan_abort()`, `sub_scan_busy()`; result as `RC_EVT_AVOIDANCE_PLAN` | request the step scan (servo and chassis measure the obstacle); profile and plan come back as events | team |
| IF-09 | sub_nav → S5 | direct API | `sub_scan_set_watch(on, trigger_mm)` | forward watch on in FOLLOWING / READING_BARCODE (and, outside TEST builds, in IDLE/STOPPED, where nav ignores it); off while executing a command or avoiding | team |
| IF-10 | S1 ← all | subsystem getters + subscribed events | — | telemetry snapshot of every subsystem's latest state | team |
| IF-11 | S1 → sub_nav | event (FAST for sub_nav) | `COMMAND_RX` | remote command: STOP from any running state (FR10b); START from IDLE/STOPPED; turn / straight / U-turn only in FOLLOWING or READING_BARCODE. FAST so a STOP never queues behind telemetry | team |
| IF-12 | S1 → MQTT service | message buffer, `TMO_POL` | `s1_mbf_tx` (in `subsystem1.h`) | outbound binary frames (`subsystem1/tm_frame.h`, 3-28 B) drained by `mqtt_utk.c` every 50 ms. **Instructor-approval item** | team + instructor |
| IF-13 | MQTT service → S1 | message buffer, `TMO_POL` | `s1_mbf_rx` (in `subsystem1.h`) | inbound `car/01/cmd` payload | team + instructor |
| IF-14 | sub_nav → S3 | direct API | `sub_line_enable()`, `sub_line_find()`, `sub_line_reading()`, `sub_barcode_arm()` | line steering on in FOLLOWING (kept on through READING_BARCODE), off in every other state (nav steers the rejoin itself); `sub_line_find()` after a junction turn and when the START line-up finds no line; barcode armed during FOLLOWING / READING_BARCODE | team |
| IF-15 | S3 → sub_nav | — | `RC_EVT_LINE_REACQUIRED` | **retired**: sub_nav no longer subscribes; RECOVERING ends with nav's own line scan (FR9). `sub_line` still publishes the event, and telemetry reports it on `car/01/line` | team |
| IF-16 | S3 → sub_nav | event (FAST) | `RC_EVT_BARCODE_DECODED` | `'*'` while FOLLOWING → READING_BARCODE; then `command` A/B/C/D → EXECUTING_CMD (FR3). sub_nav relies on `sym` (5-bit code, `rc_sym_char()`; CR1) and `command` as filled by `sub_barcode.c` | team |
| IF-17 | sub_nav, S3 → all | event (FAST) | `RC_EVT_RECOVERY_FAILED` | sub_nav: line not reached or a rejoin step timed out after a bypass (limit from the bypass length, `recover_timeout_ms()`: 150 % of leg 1 at the detour speed + 2 s); sub_line: its lost-line search found nothing. Nav goes to STOPPED (FR9, FR12). Also reported by telemetry | team |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits in `config/config.h`. Counts below are the objects the application
creates (`tk_cre_*` calls in `app_program/`, TEST-only objects not counted).

| Object | Limit | S1 | S2 | S3 | S4 | S5 | shared core |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 (Telemetry) | 1 (Motion) | — | — | 1 (Scan) | 4 (Defer, FAST, SLOW, Sense) + `usb_irq_prio` (runs once at boot and exits) |
| Semaphore | 16 | | | | | 1 (scan requests) | |
| Event flag | 16 | | | | | 1 (scan results / abort) | 4 (Defer, FAST and SLOW lanes, fault bitmask) |
| Mailbox | 12 | | | | | | |
| Mutex | 12 | | 1 (motion, `TA_INHERIT`) | | | | |
| Message buffer | 12 | 2 (`s1_mbf_tx/rx`, MQTT build) | | | | | |
| Fixed memory pool | 12 | | | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | | 1 (Motion period, `rc_period`) | | | | 2 (Sense period `rc_period`, watchdog supervisor `rc_wdog`) |
| Alarm handler | 12 | | | | | | 1 (sub_nav state timer) |
| TIMER alarm (HW) | 4 | | | | | 2 (ultrasonic) | |

(The kernel's USB console and WiFi tasks and the device drivers, e.g. I2C
and ADC, also use a few of these.)
