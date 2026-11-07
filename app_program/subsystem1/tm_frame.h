/*
 * tm_frame.h  --  Buddy 1 / subsystem 1
 *
 * Compact binary MQTT payloads. Every message is
 *
 *     byte 0   [ver:3 | type:5]     ver = TM_VER, type = tm_type_t
 *     byte 1   seq (8 bits, wraps)  state frames: one per state tick;
 *                                   all others: one per event frame
 *     byte 2.. the fields of that type, packed back to back, least
 *              significant bit first (bit i of the body is bit (i % 8) of
 *              body byte i / 8), each field in exactly the bits its range
 *              needs; signed fields are two's complement in that width.
 *
 * Fields are written one by one with shifts, never by casting a struct
 * onto the buffer, so the layout does not depend on the compiler's
 * bit-field order and there is no unaligned access on the Cortex-M0+.
 * Values are clamped into their width (saturate, never wrap).
 *
 * The schema (field names, widths, order) lives in ONE table in
 * tm_frame.c. The laptop side carries the same table in
 * scripts/tm_frames.py (used by the broker and the dashboard); bump TM_VER
 * on any change so a mismatched decoder refuses the frames. docs/cr1/
 * lists every frame.
 *
 * Commands from the dashboard stay one ASCII byte (car/01/cmd): the
 * kernel's receive path is text-only.
 */
#ifndef TM_FRAME_H
#define TM_FRAME_H

#include "../rc_types.h"
#include <stddef.h>

#define TM_VER          (2U)    /* bump on any schema change */
#define TM_HDR_BYTES    (2U)
#define TM_FRAME_MAX    (32U)   /* longest is STATE_ENC at 28 bytes */
#define TM_FIELDS_MAX   (24U)

typedef enum
{
    TM_T_STATE = 1,     /* car/01/state, 20 B                          */
    TM_T_STATE_ENC,     /* car/01/state + encoder counts, 28 B          */
    TM_T_STATUS,        /* car/01/status heartbeat, 20 B                */
    TM_T_BARCODE,       /* car/01/barcode, 4 B                          */
    TM_T_BCSCAN,        /* car/01/bcscan, 15 B                          */
    TM_T_BCBITS,        /* car/01/bcbits, 16 B                          */
    TM_T_SPIN,          /* car/01/spin, 15 B                            */
    TM_T_HUMP,          /* car/01/hump, 6 B                             */
    TM_T_OBSTACLE,      /* car/01/obstacle, 10 B                        */
    TM_T_AVOID,         /* car/01/avoid, 6 B                            */
    TM_T_IMPACT,        /* car/01/impact, 3 B                           */
    TM_T_MOTION,        /* car/01/motion, 3 B                           */
    TM_T_LINE,          /* car/01/line, 3 B                             */
    TM_T_MOVE,          /* car/01/move, 6 B                             */
    TM_T_ACK,           /* car/01/ack, 4 B                              */
    TM_T_RAMP,          /* car/01/ramp, 7 B                             */
    TM_T_COUNT
} tm_type_t;

/* Pack one frame. vals[] holds the type's fields in schema order (see
 * tm_frame.c). Returns the frame length in bytes, 0 on a bad type or a
 * buffer that is too small. */
uint16_t tm_pack(tm_type_t type, uint8_t seq, const int32_t *vals,
                 uint8_t *out, uint16_t cap);

/* Unpack a frame back into vals[] (schema order). Returns the number of
 * fields, or -1 if the header or length is wrong. *type and *seq are
 * filled from the header. */
int32_t tm_unpack(const uint8_t *buf, uint16_t len, tm_type_t *type,
                  uint8_t *seq, int32_t *vals);

/* Render a frame as JSON text, e.g.
 * {"peak_mm":40,"ms":900,"pitch":12}. Used by the console sink and the
 * self-tests; the laptop has its own decoder. false on a bad frame. */
bool tm_to_json(const uint8_t *buf, uint16_t len, char *out, size_t cap);

/* The topic leaf of a type ("state", "hump", ...), NULL if unknown. */
const char *tm_leaf(tm_type_t type);

#endif /* TM_FRAME_H */
