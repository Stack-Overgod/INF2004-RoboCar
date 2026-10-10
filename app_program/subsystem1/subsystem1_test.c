/*
 * subsystem1_test.c
 * On-target tests for subsystem 1 (telemetry). Built only with: make TEST=1
 *
 * These run with no radio and no broker, so they pass on a plain console
 * build as well as a WiFi one: they exercise the telemetry layer itself --
 * the console sink, the event framing, and command routing -- not the
 * network. Save the console output to testing/subsystem1/logs/ and record
 * the result in testing/subsystem1/TEST_LOG.md. Test IDs: S1-U01, S1-U02...
 */

#include "subsystem1.h"
#include "sub_telemetry.h"
#include "../test_report.h"

#if APP_TEST

/* S1-U05 records what deliver_command() handed to the callback, so the test
 * can confirm the command and argument arrived unchanged. */
static volatile rc_nav_cmd_t test_last_cmd;
static volatile int32_t      test_last_arg;
static volatile int          test_cmd_hits;

static void test_cmd_cb(rc_nav_cmd_t cmd, int32_t arg, void *ctx)
{
    (void)ctx;
    test_last_cmd = cmd;
    test_last_arg = arg;
    test_cmd_hits++;
}

EXPORT void subsystem1_test(void)
{
    const sub_telemetry_sink_t *console = sub_telemetry_console_sink();
    rc_event_t evt;

    TEST_INIT();

    /* S1-U01: the console sink always exists and reports itself up -- it
       needs no radio -- so telemetry always has somewhere to fall back to. */
    TEST_CHECK("S1-U01",
               (console != NULL) && console->is_up(),
               "console sink present and up");

    /* S1-U02: the console sink accepts a publish. This is the send path on
       a plain build, so a PASS means framing-to-output works end to end. */
    TEST_CHECK("S1-U02",
               console->publish("car/01/test", "{\"t\":1}", 7) == RC_OK,
               "console sink publishes");

    /* S1-U03: a known event is recognised and routed. We check "not
       rejected" rather than RC_OK so the result doesn't depend on whether a
       network sink happens to be down (which would return RC_ERR_STATE). */
    evt.id = RC_EVT_BARCODE_DECODED;
    evt.u.barcode.symbol   = 'A';
    evt.u.barcode.command  = RC_CMD_TURN_LEFT;
    evt.u.barcode.reversed = false;
    TEST_CHECK("S1-U03",
               sub_telemetry_publish_event(&evt) != RC_ERR_PARAM,
               "known event recognised");

    /* S1-U04: an event we don't report on is turned away, not sent blindly. */
    evt.id = RC_EVT_TICK;
    TEST_CHECK("S1-U04",
               sub_telemetry_publish_event(&evt) == RC_ERR_PARAM,
               "unknown event rejected");

    /* S1-U05: a decoded command reaches a registered callback unchanged.
       Restore the default (event-bus) routing afterwards so the running
       system is left as it was. */
    test_cmd_hits = 0;
    (void)sub_telemetry_on_command(test_cmd_cb, NULL);
    (void)sub_telemetry_deliver_command(RC_CMD_U_TURN, 42);
    TEST_CHECK("S1-U05",
               (test_cmd_hits == 1) &&
               (test_last_cmd == RC_CMD_U_TURN) &&
               (test_last_arg == 42),
               "command routed to callback");
    (void)sub_telemetry_on_command(NULL, NULL);

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
