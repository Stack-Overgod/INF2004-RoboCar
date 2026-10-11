/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01 ...
 */

#include "../rc_prelude.h"
#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	TEST_INIT();

	TEST_CHECK("S4-U01", TRUE, "placeholder: replace with real tests");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */