# Change Requirement 2: safe response to failed or invalid data

> If any part of the system fails or provides invalid data, the system must
> respond in a safe and predictable way.

This note lists, for every input the car depends on, how a failure is
**detected**, what the car **does**, and **why** that response was chosen.
Code: `app_program/sub_nav.c` section "Fault response" (one table,
`fault_rules[]`), plus the detectors in each subsystem.

## Principle

Each subsystem only *detects* bad data and raises its bit in the shared fault
mask (`rc_fault.h`, a µT-Kernel event flag, sent in every telemetry state
frame). `sub_nav` alone *decides* the response, so the response to a fault does
not depend on which task noticed it, and there is exactly one place to read
or change the policy. The check runs on every line sample (10 ms, FAST lane),
so a response follows within one sample.

Three responses, chosen by what the lost data is needed for:

| Response | When | What the car does |
|---|---|---|
| **STOP** | the car cannot move safely, or has lost what it steers by | `STOPPED`: motors braked, line/barcode/watch off. A person checks the car and sends START. |
| **HOLD** | the car cannot see ahead, but the fault may clear by itself | `HOLDING` (new state): braked on the line, watch still pinging. Resumes `FOLLOWING` on the first good answer; `STOPPED` after 5 s. |
| **CARRY ON** | the mission does not need the lost function | the run continues without it; the fault shows on the dashboard. |

## Per input

| Input / part | Failure and how it is detected | Response | Justification |
|---|---|---|---|
| Wheel motors (Buddy 2) | `MOTOR_STALL`: both wheels driven at ≥ stall duty with no encoder ticks for the stall time (`sub_motion` stall watch) | motion brakes at once; nav **STOP** | the car is blocked; pushing on would burn the motors or shove the obstacle |
| Encoders (Buddy 2) | `ENCODER`: one wheel silent while the other turns | motion brakes; nav **STOP** | speed control, distances and turns all come from the encoders; without one the PID would drive that wheel to full power and every turn angle is unknown |
| Ultrasonic (Buddy 5) | `NO_ECHO`: 3 pings in a row where ECHO never rose (`drv_ultrasonic`). A ping where ECHO rose but nothing reflected in range is a valid "clear ahead", not a fault | **HOLD** | the car is blind to obstacles; driving on would treat "no reading" as "no obstacle", the exact unsafe case in the requirement. The echo is marginal at the 3.3 V supply (~2.2 V), so one missed ping should not stop the car: 3 are needed; stopping in place costs nothing and the sensor often recovers |
| Ultrasonic chain (driver, defer task, scan task) | no forward-watch result at all for 500 ms while following (`sub_nav`, data age) | **HOLD** | a stopped chain leaves the last fault bit unchanged, so the bit alone cannot show it; the age of the last result can |
| Line sensors (Buddy 3) | both black for 2 s while following (`sub_nav`): the pair straddles the line, so 1/1 is only a crossing or a barcode bar (< 1 s at ~30 mm/s). 2 s means no floor under them (lifted: air reads black) or a stuck/unplugged sensor | **STOP** | the car cannot steer by the line; also stops a lifted car when the IMU is not working |
| Line sensors | both white for 2 s (line lost, `sub_line`) | search sweep, then `RECOVERY_FAILED` → **STOP** (existing) | attempt recovery once, then require intervention |
| IMU (Buddy 4) | `IMU`: not ready, I²C read failed, or the same reading 50 times running (frozen chip) (`drv_imu`); no samples are published while it is set | **CARRY ON** | hump and ramp measurements are reported, not needed to follow the line; the lift stop is lost, but the both-black rule still stops a lifted car and STOP still works over WiFi |
| Lift / overturn (Buddy 4) | `LIFTED` + `RC_EVT_LIFTED` (`sub_terrain`) | **STOP** (existing; the bit is now also a backstop if the event is lost) | wheels spinning in the air or upside down |
| Barcode ADC (Buddy 3) | `BARCODE_ADC`: ADC set-up failed at boot | **CARRY ON** | no barcode commands; the car keeps following the line. A barcode that does not decode times out back to FOLLOWING (existing) |
| WiFi / broker (Buddy 1) | `WIFI_LOSS` (`sub_telemetry`) | **CARRY ON**, reconnects with backoff (existing) | the run is autonomous; stopping on every reconnect would end runs for no safety gain. Hand stop still works (lift) |
| Remote commands | turn commands outside FOLLOWING / READING_BARCODE | ignored (existing, C1) | a turn in the middle of a bypass would leave the car off the line |
| Tasks (Sense, Motion, FAST dispatcher) | stop checking in: hardware watchdog 2 s (`rc_wdog`) | reset, `WDOG_RESET`, wait in IDLE for START. **New:** the build without MQTT no longer starts driving again after a watchdog reset | a reset can land mid-manoeuvre, off the line; restarting on its own would be unpredictable |
| Lost answers inside nav | plan or MOTION_DONE never arrives in AVOIDING (**new** 30 s backstop per step), EXECUTING_CMD (5 s), RECOVERING (computed) | **STOP** | nav never waits forever in a moving state |

## Changes made (branch `cr2-fail-safe`)

| File | Owner (reviewer) | Change |
|---|---|---|
| `sub_nav.c/.h` | Buddy 5 (shared, team) | `fault_rules[]` table + `fault_check()` on every line sample; new state `RC_NAV_HOLDING` (value 7, still fits the 3-bit telemetry field); watch data age check; both-black stop (off in TEST builds: the stand reads air as black); AVOIDING timeout per step |
| `subsystem5/drv_ultrasonic.c` | Buddy 5 | `NO_ECHO` = sensor not answering 3 times in a row; "nothing in range" is no longer a fault |
| `subsystem2/sub_motion.c` | Buddy 2 | a fresh speed-mode drive clears the old stall/encoder bits (as a queued move already did), so a repeat stall raises the bit again |
| `app_main.c` | team | no automatic START after a watchdog reset (build without MQTT) |
| `rc_fault.h` | team | NO_ECHO row; points to the response table |
| `scripts/tm_frames.py`, `dashboard/index.html` | Buddy 1/5 | state name HOLDING |

The message format does not change (TM_VER stays 4); an old dashboard shows state 7 as a number.

## Still to verify on the car

1. Unplug the ultrasonic during a run: HOLDING within ~0.2 s, plug back in →
   FOLLOWING; left unplugged → STOPPED after 5 s.
2. Lift the car in a run with the IMU working (lift stop) and with the IMU
   unplugged (both-black stop after 2 s).
3. Hold one wheel: STOPPED with MOTOR_STALL; START works afterwards.
4. A normal track run: no HOLDING from open space ahead (the "nothing in
   range" case), no both-black stop on crossings or barcodes.
5. TEST suite on the stand: S5 nav tests still pass.
