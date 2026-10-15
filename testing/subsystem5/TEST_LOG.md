# Subsystem 5 — test log

**Subsystem:** Scan and avoidance  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

Bench setup (no Robo Pico needed): servo signal → GP15, + → VBUS (pin 40),
− → GND. HC-SR04 optional: TRIG → GP2, ECHO → GP3 through the 1 kΩ/2 kΩ
divider, VCC → VBUS, GND → GND. Build with `make SMP=0 WIFI=none TEST=1`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Shared core (defer list, event bus, GPIO IRQ) starts | — | auto | `subsystem5_test.c` (only while `S5_TEST_STANDALONE` is 1) | `[TEST] S5-U01 PASS` |
| S5-U02 | Servo PWM starts on GP15 and centres at 90° | FR7b | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | Servo clamps commands outside 0–180° | FR7b | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | Settle estimate is positive, symmetric, grows with travel | FR7b | auto | `subsystem5_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | Ultrasonic driver claims TRIG/ECHO, IRQ and timer alarms | NFR4 | auto | `subsystem5_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | One ping ends in a result event within 100 ms, tagged with its angle | FR6, NFR4 | auto | `subsystem5_test.c` | `[TEST] S5-U06 PASS` (works with or without a sensor) |
| S5-U07 | A second ping while one is in flight is refused | NFR4 | auto | `subsystem5_test.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | Scan task starts | FR7 | auto | `subsystem5_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | Full scan finishes within 5 s with all 5 coarse bearings | FR7b, FR7c | auto | `subsystem5_test.c` | `[TEST] S5-U09 PASS`, then `[INFO] S5 profile …` and `[INFO] S5 plan …` |
| S5-U10 | Servo physically visits each scan angle | FR7b | manual | 1. Flash the TEST=1 image. 2. Watch the servo horn during S5-U09. | Stops at 30, 60, 90, 120, 150°; with an object closer than the obstacle threshold, a 6° fine sweep around it follows |
| S5-U11 | Range accuracy straight ahead | FR6, FR7c | manual | 1. Wire the HC-SR04. 2. Put a flat box at 100, 300 and 1000 mm on a tape. 3. Reset and read `[INFO] S5 range straight ahead`. | Each reading within the FR7c range tolerance; record the actual error |
| S5-U12 | GP15 is really outputting PWM: PWM block out of reset, pin routed to PWM, slice counter running | FR7b | auto | `subsystem5_test.c` | `[TEST] S5-U12 PASS` |
| S5-N01 | `sub_nav` stays IDLE (ignores other tests' fake events) until `sub_nav_start()` | §6.5 | auto | `subsystem5_test.c` | `[TEST] S5-N01 PASS` |
| S5-N02 | `sub_nav_start()` goes IDLE → FOLLOWING (line, barcode, forward watch on) | §6.5, FR6 | auto | `subsystem5_test.c` | `[TEST] S5-N02 PASS` |
| S5-N03 | A 200 mm watch reading while FOLLOWING enters AVOIDING and starts a scan | FR6, FR7a | auto | `subsystem5_test.c` (fake `RC_EVT_ULTRA_RESULT`) | `[TEST] S5-N03 PASS`, servo sweeps |
| S5-N04 | The scan's plan is acted on: nav leaves AVOIDING within 5 s | FR8a | auto | `subsystem5_test.c` | `[TEST] S5-N04 PASS`, then `[INFO] S5 nav after plan: …` (STOPPED until Buddy 2's `sub_motion` is wired) |
| S5-N05 | `sub_nav_stop()` goes FOLLOWING → STOPPED with no scan left running | FR10b | auto | `subsystem5_test.c` | `[TEST] S5-N05 PASS` |
| S5-N06 | A close reading while STOPPED does not restart avoidance | §6.5 | auto | `subsystem5_test.c` | `[TEST] S5-N06 PASS` |
| S5-N07 | Real obstacle path on a bare board | FR6–FR8a | manual | 1. Normal build (`make SMP=0 WIFI=none`), HC-SR04 wired. 2. Put a hand 200 mm in front. | Console: `[nav] obstacle at … mm`, `FOLLOWING -> AVOIDING`, servo sweeps, then `[nav] bypass …` + `refused by sub_motion` (or `no safe gap`), ending `AVOIDING -> STOPPED` (until S2 is wired) |
| S5-N08 | A Code 39 start character `*` while FOLLOWING enters READING_BARCODE | §6.5, FR3 | auto | `subsystem5_test.c` (fake `RC_EVT_BARCODE_DECODED`) | `[TEST] S5-N08 PASS` |
| S5-N09 | Data character `A` while READING_BARCODE is executed as a left turn | FR3 | auto | `subsystem5_test.c` | `[TEST] S5-N09 PASS`, then `[INFO] S5 nav after barcode A: …` (STOPPED until S2 is wired, EXECUTING_CMD after) |
| S5-N10 | A start character with no data times out back to FOLLOWING (2 s) | G10 | auto | `subsystem5_test.c` | `[TEST] S5-N10 PASS`, console `[nav] barcode not decoded, carrying on` |
| S5-N11 | RECOVERING with no line found → STOPPED and one `RC_EVT_RECOVERY_FAILED` within 3 s | FR9, FR12 | auto | `subsystem5_test.c` (`sub_nav_test_force`). **Both line sensors off the line** (car lifted or over white) | `[TEST] S5-N11 PASS`, console `[nav] line not found in 3000 ms` |
| S5-N12 | `LINE_REACQUIRED` in RECOVERING → FOLLOWING, and the 3 s timer is cancelled | FR9 | auto | `subsystem5_test.c`. Nothing within 300 mm of the sensor | `[TEST] S5-N12 PASS` |
| S5-N13 | A remote turn runs from FOLLOWING and is ignored once STOPPED | FR10b | auto | `subsystem5_test.c` (`sub_nav_inject_command`) | `[TEST] S5-N13 PASS` |

Once Buddy 2's motion is wired, S5-N04, N09, N11 and N13 really drive the
wheels: run the TEST=1 image with the car on a stand.
*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
