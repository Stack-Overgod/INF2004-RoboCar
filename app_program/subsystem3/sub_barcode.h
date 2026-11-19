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

rc_result_t sub_barcode_init(void);

/* Arm or disarm decoding. Leave it disarmed while the car is not
 * expecting a barcode, so track noise does not produce phantom commands.
 * sub_nav arms it for the whole of FOLLOWING. */
rc_result_t sub_barcode_arm(bool on);

#endif /* SUB_BARCODE_H */
/*** end of file ***/