/*
 * subsystem1.c -- Subsystem 1: WiFi, commands and telemetry (Buddy 1).
 *
 * The template's two entry points, and the pattern every subsystem
 * follows. usermain() starts the shared layer first (Phase 0: the event
 * bus is running before anyone subscribes), then calls every _init(), then
 * every _start(), so all kernel objects and subscriptions exist before
 * any task runs:
 *
 *   subsystem1_init()   subscribe to the other buddies' events and, in an
 *                       MQTT build, create the two message buffers.
 *                       No task yet.
 *   subsystem1_start()  start the telemetry task (priority 12).
 *
 * NOTE: sub_telemetry returns rc_result_t (RC_OK == 0); the template wants
 * ER. RC_OK becomes E_OK, anything else E_SYS so usermain()'s CHECK()
 * prints it. (There's no RC_ER helper in the shared headers, so convert
 * by hand.)
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
