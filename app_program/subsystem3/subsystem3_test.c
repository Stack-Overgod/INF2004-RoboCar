/*!
 * @file  subsystem3_test.c
 * @brief On-target tests for subsystem 3. Built only with make TEST=1.
 *        Record results in testing/subsystem3/TEST_LOG.md (IDs: S3-U01 ...).
 */

#include "../rc_prelude.h"
#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

/*!
 * @brief  Run subsystem 3's on-target tests.
 */
EXPORT void subsystem3_test(void)
{
    TEST_INIT();

    TEST_CHECK("S3-U01", TRUE, "placeholder: replace with real tests");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/