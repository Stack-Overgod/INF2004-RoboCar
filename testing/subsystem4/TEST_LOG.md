# Subsystem 4 — test log

**Subsystem:** IMU terrain (hump, lift/overturn, motion class)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | IMU read functions reject NULL pointers | — | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | `drv_imu_calibrate(0)` is rejected | — | auto | `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | `sub_terrain_init()` resets to stationary, peak 0 mm | — | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | Held upward acceleration gives one `RC_EVT_LIFTED` (raised) and motion class LIFTED | lift stop, IF-04 | auto | `subsystem4_test.c` (fake IMU samples, real ones held) | `[TEST] S4-U04 PASS` |
| S4-U05 | Nose-up pitch enters the hump climbing state | FR5 | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | Nose-down pitch moves to the descending state | FR5 | auto | `subsystem4_test.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | Levelling out records a non-zero hump peak and calls the hump callback once | FR5 | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | Flat-ground start-up surge, cruise and braking are not reported as a hump | FR5 (risk G6) | auto | `subsystem4_test.c` | `[TEST] S4-U08 PASS` |
| S4-U09 | `sub_terrain` FAST callback worst case < 10 % of the 10 ms IMU period | NFR1 | auto | `subsystem4_test.c` | `[TEST] S4-U09 PASS`, `[INFO] S4 on_sample WCET = … us` |
| S4-U10 | The lift episode ends once the car is calm on the ground; a single vertical spike is not a lift | lift stop | auto | `subsystem4_test.c` | `[TEST] S4-U10 PASS` |
| S4-U11 | Upside down gives one `RC_EVT_LIFTED` (overturned) | lift stop, IF-04 | auto | `subsystem4_test.c` | `[TEST] S4-U11 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
