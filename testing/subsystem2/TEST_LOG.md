# Subsystem 2 — test log

**Subsystem:** Motion (motors, encoders, wheel PID, moves/turns)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Motion is idle after boot (not busy, both targets 0) | — | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | Motor driver rejects an invalid wheel | — | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | Every encoder reader returns 0 for an invalid wheel | — | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | µm per encoder tick matches the hand calculation | — | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | `RC_EVT_ODOMETRY` published at 50 Hz | NFR2, IF-01 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | `drv_motor_set` clamps to ±1000 ‰ | — | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | `drv_motor_stop` sets both wheels to 0 | — | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | `sub_motion_drive` applies base ± steer at once, clamped | IF-07 | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | Both encoders count while the wheels turn | — | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U09 PASS` |
| S2-U10 | Both wheels read a positive speed going forward | — | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U10 PASS` |
| S2-U11 | Speed reads 0 once the wheels have stopped | — | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U11 PASS` |
| S2-U12 | `drv_encoder_reset` zeroes both distances | — | auto | `subsystem2_test.c` | `[TEST] S2-U12 PASS` |
| S2-U13 | A forward move runs and reports completed | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U13 PASS` |
| S2-U14 | Forward distance within tolerance | NFR12 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U14 PASS` |
| S2-U15 | A backward move reverses both wheels and completes | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U15 PASS` |
| S2-U16 | Turn right 90°: left wheel forward, right back, completes | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U16 PASS` |
| S2-U17 | Turn left 90°: left wheel back, right forward, completes | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U17 PASS` |
| S2-U18 | 90° turn arc within tolerance | FR3 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U18 PASS` |
| S2-U19 | A new move pre-empts the running one, which reports not done | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U19 PASS` |
| S2-U20 | `drive` / `drive_speed` are refused during a queued move | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U20 PASS` |
| S2-U21 | `sub_motion_stop` cancels the move (reported not completed) | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U21 PASS` |
| S2-U22 | `drive_speed` targets base ± steer mm/s | IF-07 | auto | `subsystem2_test.c`; the wheels turn: car on a stand | `[TEST] S2-U22 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
