/*
 * subsystem1.c -- Subsystem 1: telemetry and commands. ID: <your ID>.
 *
 * The template's two entry points. usermain() runs every _init() first,
 * then every _start(), so all kernel objects exist before any task runs.
 *
 * NOTE: sub_telemetry returns rc_result_t (RC_OK == 0); the template wants
 * ER. RC_OK becomes E_OK, anything else E_SYS so usermain()'s CHECK() flags
 * it. (There's no RC_ER helper in the shared headers, so convert by hand.)
 */
#include "subsystem1.h"

EXPORT ER subsystem1_init(void)
{
    return (sub_telemetry_init() == RC_OK) ? E_OK : E_SYS;
}

EXPORT ER subsystem1_start(void)
{
    return (sub_telemetry_start_task() == RC_OK) ? E_OK : E_SYS;
}
