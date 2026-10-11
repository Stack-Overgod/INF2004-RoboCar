/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01 ...
 */

#include "../rc_prelude.h"
#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	TEST_INIT();

	TEST_CHECK("S2-U01", TRUE, "placeholder: replace with real tests");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */