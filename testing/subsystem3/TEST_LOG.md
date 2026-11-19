# Subsystem 3 — test log

**Subsystem:** IR line following and Code 39 barcode  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | *Removed.* Template example test, not part of this project. ID not reused | — | — | — | — |
| S3-U02 | Line sensor L (GP16): off on white, 3 line crossings | — | manual + auto | `subsystem3_test.c`: hold the sensor over white until it reads off, then move it across the black line and back three times | `[TEST] S3-U02 PASS` |
| S3-U03 | Line sensor R (GP6): off on white, 3 line crossings | — | manual + auto | `subsystem3_test.c`: hold the sensor over white until it reads off, then move it across the black line and back three times | `[TEST] S3-U03 PASS` |
| S3-U04 | Barcode sensor AO (GP26, ADC): off on white, 3 crossings | FR16 | manual + auto | `subsystem3_test.c`: hold the sensor over white until it reads off, then move it across the black line and back three times | `[TEST] S3-U04 PASS` |
| S3-U05 | The Sense task publishes line samples at ~100 Hz | NFR1, IF-02 | auto | `subsystem3_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | `*A*` forwards at 150 and 350 mm/s → 'A', TURN_LEFT | FR3, IF-03 | auto | `subsystem3_test.c` (bar widths fed straight to the decoder, no hardware) | `[TEST] S3-U06 PASS` |
| S3-U07 | `*A*` backwards → 'A', TURN_LEFT, reversed | FR3 | auto | `subsystem3_test.c` (bar widths fed straight to the decoder, no hardware) | `[TEST] S3-U07 PASS` |
| S3-U08 | `*B*` `*C*` `*D*` `*Z*` both ways → the right letter | FR3 | auto | `subsystem3_test.c` (bar widths fed straight to the decoder, no hardware) | `[TEST] S3-U08 PASS` |
| S3-U09 | A bar longer than 200 ms is rejected, no letter | FR3 | auto | `subsystem3_test.c` (bar widths fed straight to the decoder, no hardware) | `[TEST] S3-U09 PASS` |
| S3-U10 | Live: the barcode sensor gave edges | FR16 | manual + auto | `subsystem3_test.c`: sweep the barcode sensor across a printed barcode | `[TEST] S3-U10 PASS` |
| S3-U11 | Live: at least one letter decoded | FR3 | manual + auto | `subsystem3_test.c`: sweep the barcode sensor across a printed barcode | `[TEST] S3-U11 PASS` |
| S3-U12 | Live: no edge or result lost | FR3 | manual + auto | `subsystem3_test.c`: sweep the barcode sensor across a printed barcode | `[TEST] S3-U12 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
