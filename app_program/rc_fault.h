/*
 *  rc_fault.h  -  the shared fault bitmask
 *
 *  One bit per fault, kept in a micro T-Kernel event flag rather than a
 *  plain variable. tk_set_flg / tk_clr_flg change only the bits they are
 *  given, inside the kernel's own lock, so two cores and the encoder / echo
 *  ISRs can all set and clear bits without a read-modify-write race (the
 *  RP2040's Cortex-M0+ has no LDREX/STREX, so a C `mask |= bit` would lose
 *  updates). Each bit has one owner who sets and clears it:
 *
 *    bit  name                owner      set when                  cleared when
 *    0    MOTOR_STALL         Buddy 2    both wheels pushed, no     next move starts
 *                                        movement (sub_motion)
 *    1    ENCODER             Buddy 2    one wheel silent while     next move starts
 *                                        the other turns
 *    2    IMU                 Buddy 4    IMU open or read failed    a read succeeds
 *    3    NO_ECHO             Buddy 5    ultrasonic timeout / out   a valid echo
 *                                        of range
 *    4    WIFI_LOSS           Buddy 1    WiFi or broker down        link back up
 *    5    IMPACT              Buddy 4    impact detected (FR13)     START command
 *    6    BARCODE_ADC         Buddy 3    ADC open / read failed     never (boot)
 *
 *  7 bits used; telemetry sends the low 8 (subsystem1/tm_frame.c).
 *  Set from an ISR is fine (tk_set_flg is ISR-callable); clear only from
 *  task context.
 */
#ifndef RC_FAULT_H
#define RC_FAULT_H

#include "rc_types.h"

#define RC_FAULT_MOTOR_STALL    (1U << 0)
#define RC_FAULT_ENCODER        (1U << 1)
#define RC_FAULT_IMU            (1U << 2)
#define RC_FAULT_NO_ECHO        (1U << 3)
#define RC_FAULT_WIFI_LOSS      (1U << 4)
#define RC_FAULT_IMPACT         (1U << 5)
#define RC_FAULT_BARCODE_ADC    (1U << 6)

#define RC_FAULT_BITS           (8U)        /* width in telemetry */
#define RC_FAULT_ALL            (0x7FU)

/* Create the flag. Call once before any subsystem init (app_main phase 0).
 * Safe to call again. Until it has run, set/clear do nothing and get
 * returns 0. */
rc_result_t rc_fault_init(void);

void     rc_fault_set(uint32_t bits);       /* task or ISR */
void     rc_fault_clear(uint32_t bits);     /* task only   */
uint32_t rc_fault_get(void);                /* current mask, one kernel read */

#if APP_TEST
uint32_t rc_fault_get_raw(void);            /* all 32 bits, for rc_fault_test.c */
void     rc_fault_test(void);               /* fault bitmask and Sense timing tests (rc_fault_test.c) */
#endif

#endif /* RC_FAULT_H */
