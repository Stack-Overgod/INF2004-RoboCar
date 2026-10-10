/*
 *  sub_barcode.h  -  Buddy 3, barcode decoding
 *
 *  Consumes RC_EVT_BARCODE_EDGE, which carries the width in microseconds
 *  of each bar and space as the car drives over them, and produces a
 *  decoded character plus the navigation command it maps to.
 */
#ifndef SUB_BARCODE_H
#define SUB_BARCODE_H

#include "../rc_types.h"

/* This declares "sub_barcode_cb_t" as a name for a FUNCTION POINTER type:
 * any function that takes (char, rc_nav_cmd_t, void*) and returns nothing
 * can be assigned to a variable of this type and called through it later.*/
typedef void (*sub_barcode_cb_t)(char symbol, rc_nav_cmd_t cmd, void *ctx);

rc_result_t sub_barcode_init(void);

/* Arm or disarm decoding. Leave it disarmed while the car is not
 * expecting a barcode, so track noise does not produce phantom commands.
 * sub_nav arms this when sub_line reports a junction. */
rc_result_t sub_barcode_arm(bool on);

rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx);

/* Last successful decode, for telemetry. */
char sub_barcode_last(void);

#endif /* SUB_BARCODE_H */
/*** end of file ***/