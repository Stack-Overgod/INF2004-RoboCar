/*
 * tm_frame.c  --  Buddy 1 / subsystem 1
 *
 * The binary telemetry frames. See tm_frame.h for the wire format; the
 * table below is the whole schema. A row is { JSON key, width in bits,
 * how to show it }. Change a row here and in scripts/tm_frames.py FRAMES
 * together, and bump TM_VER.
 */
#include "../rc_prelude.h"

#include "tm_frame.h"
#include "../rc_fmt.h"              /* rc_snprintf() */

typedef enum
{
    K_U = 0,        /* unsigned number                                  */
    K_S,            /* signed number (two's complement in `bits`)       */
    K_LINE2,        /* 2 bits: bit 0 left, bit 1 right -> "10"          */
    K_SYM,          /* 5-bit barcode symbol code -> "C", "-" for none   */
    K_SYM3,         /* 3 x 5-bit symbol codes -> "AB"                   */
    K_CHAR,         /* one ASCII byte -> "L", "?" if not printable      */
    K_FLAGS4,       /* 4 bits, bit 0 first -> "1011"                    */
    K_SINK          /* 2 bits: 0 console, 1 mqtt, 2 other               */
} kind_t;

typedef struct
{
    const char *key;
    uint8_t     bits;
    uint8_t     kind;
} field_t;

typedef struct
{
    const char    *leaf;
    const field_t *f;
    uint8_t        n;
    uint8_t        show_seq;    /* put "seq" first in the JSON */
} schema_t;

#define N(a)    ((uint8_t)(sizeof(a) / sizeof((a)[0])))

/* car/01/state, 4 Hz. 142 bits -> 18 B body. FR10a. */
static const field_t f_state[] =
{
    { "t",     32, K_U     },   /* ms since boot                        */
    { "spd_l", RC_BITS_SPEED, K_S },
    { "spd_r", RC_BITS_SPEED, K_S },
    { "dist",  RC_BITS_DIST,  K_U },    /* odometer mm, wraps at 65.5 m */
    { "m_l",   11, K_S     },   /* motor duty, permille -1000..1000     */
    { "m_r",   11, K_S     },
    { "line",  2,  K_LINE2 },
    { "cls",   RC_BITS_CLS,  K_U },
    { "peak",  RC_BITS_HUMP_PEAK, K_U },
    { "rng",   RC_BITS_MM,   K_U },     /* straight ahead, 0 = no echo  */
    { "bc",    RC_BITS_SYM,  K_SYM },
    { "nav",   3,  K_U     },   /* rc_nav_state_t                       */
    { "flt",   8,  K_U     },   /* rc_fault.h bitmask                   */
    { "h",     RC_BITS_HEIGHT, K_S }    /* mm above start level (ramp)  */
};

/* The same plus the two encoder counts (32-bit, single-writer words),
 * sent once a second. 206 bits -> 26 B body. */
static const field_t f_state_enc[] =
{
    { "t",     32, K_U     },
    { "spd_l", RC_BITS_SPEED, K_S },
    { "spd_r", RC_BITS_SPEED, K_S },
    { "dist",  RC_BITS_DIST,  K_U },
    { "m_l",   11, K_S     },
    { "m_r",   11, K_S     },
    { "line",  2,  K_LINE2 },
    { "cls",   RC_BITS_CLS,  K_U },
    { "peak",  RC_BITS_HUMP_PEAK, K_U },
    { "rng",   RC_BITS_MM,   K_U },
    { "bc",    RC_BITS_SYM,  K_SYM },
    { "nav",   3,  K_U     },
    { "flt",   8,  K_U     },
    { "h",     RC_BITS_HEIGHT, K_S },
    { "enc_l", 32, K_U     },
    { "enc_r", 32, K_U     }
};

/* car/01/status, every 2 s. Counters saturate at their width. 144 bits. */
static const field_t f_status[] =
{
    { "up",        32, K_U      },
    { "tx",        24, K_U      },
    { "fail",      16, K_U      },
    { "drop_fast", 16, K_U      },
    { "drop_slow", 16, K_U      },
    { "sink",      2,  K_SINK   },
    { "wifi",      1,  K_U      },
    { "mqtt",      1,  K_U      },
    { "cmds",      16, K_U      },
    { "retries",   16, K_U      },
    { "live",      4,  K_FLAGS4 }
};

static const field_t f_barcode[] =
{
    { "sym", RC_BITS_SYM, K_SYM },
    { "cmd", RC_BITS_CMD, K_U   },
    { "rev", 1,           K_U   }
};

/* Diagnostics: counters are 8-bit (they stop at 255 in sub_barcode). */
static const field_t f_bcscan[] =
{
    { "txt",       15, K_SYM3 },
    { "ok",        8,  K_U    },
    { "edges",     8,  K_U    },
    { "f_ratio",   8,  K_U    },
    { "f_wide",    8,  K_U    },
    { "f_nomatch", 8,  K_U    },
    { "f_glitch",  8,  K_U    },
    { "pat",       9,  K_U    },    /* last unmatched pattern, bit 8 first */
    { "min_10us",  16, K_U    },
    { "max_10us",  16, K_U    }
};

static const field_t f_bcbits[] =
{
    { "n",        7,  K_U },
    { "first",    1,  K_U },        /* 1: element 0 is a bar             */
    { "b0", 8, K_U }, { "b1", 8, K_U }, { "b2", 8, K_U }, { "b3", 8, K_U },
    { "b4", 8, K_U }, { "b5", 8, K_U }, { "b6", 8, K_U }, { "b7", 8, K_U },
    { "nar_10us", 16, K_U },
    { "wid_10us", 16, K_U },
    { "more",     8,  K_U }
};

static const field_t f_spin[] =
{
    { "face",  RC_BITS_MM, K_U },
    { "n",     8,  K_U },
    { "found", 4,  K_U },           /* bit i: pass i found its edge      */
    { "d0", 12, K_S }, { "r0", 8, K_U },    /* 0.1 deg, 2 mm units       */
    { "d1", 12, K_S }, { "r1", 8, K_U },
    { "d2", 12, K_S }, { "r2", 8, K_U },
    { "d3", 12, K_S }, { "r3", 8, K_U }
};

static const field_t f_hump[] =
{
    { "peak_mm", RC_BITS_HUMP_PEAK, K_U },
    { "ms",      RC_BITS_HUMP_MS,   K_U },
    { "pitch",   RC_BITS_PITCH,     K_S }
};

/* One finished slope (sub_terrain / terrain_ramp.c). 37 bits. */
static const field_t f_ramp[] =
{
    { "rise_mm", RC_BITS_RAMP_RISE, K_U },
    { "net_mm",  RC_BITS_HEIGHT,    K_S },
    { "deg",     RC_BITS_RAMP_DEG,  K_U },
    { "len_mm",  RC_BITS_RAMP_LEN,  K_U }
};

static const field_t f_obstacle[] =
{
    { "n",       RC_BITS_NPOINTS, K_U },
    { "near_mm", RC_BITS_MM,      K_U },
    { "at_deg",  RC_BITS_ANGLE,   K_U },
    { "w_mm",    RC_BITS_MM,      K_U },
    { "cl_l",    RC_BITS_CLEAR,   K_U },
    { "cl_r",    RC_BITS_CLEAR,   K_U }
};

static const field_t f_avoid[] =
{
    { "act",    RC_BITS_CMD, K_U },
    { "lat_mm", RC_BITS_MM,  K_U },
    { "fwd_mm", RC_BITS_MM,  K_U }
};

static const field_t f_impact[] = { { "impact", 1, K_U } };
static const field_t f_motion[] = { { "cls", RC_BITS_CLS, K_U } };
static const field_t f_line[]   = { { "lost", 1, K_U } };

static const field_t f_move[] =
{
    { "id", RC_BITS_MOVE_ID, K_U },
    { "ok", 1,               K_U },
    { "mm", RC_BITS_MOVE_MM, K_U }
};

static const field_t f_ack[] =
{
    { "rx",  8,           K_CHAR },
    { "cmd", RC_BITS_CMD, K_U    }
};

static const schema_t schema[TM_T_COUNT] =
{
    [TM_T_STATE]     = { "state",    f_state,     N(f_state),     1U },
    [TM_T_STATE_ENC] = { "state",    f_state_enc, N(f_state_enc), 1U },
    [TM_T_STATUS]    = { "status",   f_status,    N(f_status),    0U },
    [TM_T_BARCODE]   = { "barcode",  f_barcode,   N(f_barcode),   0U },
    [TM_T_BCSCAN]    = { "bcscan",   f_bcscan,    N(f_bcscan),    0U },
    [TM_T_BCBITS]    = { "bcbits",   f_bcbits,    N(f_bcbits),    0U },
    [TM_T_SPIN]      = { "spin",     f_spin,      N(f_spin),      0U },
    [TM_T_HUMP]      = { "hump",     f_hump,      N(f_hump),      0U },
    [TM_T_OBSTACLE]  = { "obstacle", f_obstacle,  N(f_obstacle),  0U },
    [TM_T_AVOID]     = { "avoid",    f_avoid,     N(f_avoid),     0U },
    [TM_T_IMPACT]    = { "impact",   f_impact,    N(f_impact),    0U },
    [TM_T_MOTION]    = { "motion",   f_motion,    N(f_motion),    0U },
    [TM_T_LINE]      = { "line",     f_line,      N(f_line),      0U },
    [TM_T_MOVE]      = { "move",     f_move,      N(f_move),      0U },
    [TM_T_ACK]       = { "ack",      f_ack,       N(f_ack),       0U },
    [TM_T_RAMP]      = { "ramp",     f_ramp,      N(f_ramp),      0U }
};

static const schema_t *schema_of(uint32_t type)
{
    if ((type == 0U) || (type >= (uint32_t)TM_T_COUNT) ||
        (schema[type].f == NULL))
    {
        return NULL;
    }
    return &schema[type];
}

const char *tm_leaf(tm_type_t type)
{
    const schema_t *s = schema_of((uint32_t)type);

    return (s != NULL) ? s->leaf : NULL;
}

static uint32_t body_bits(const schema_t *s)
{
    uint32_t b = 0U;
    uint32_t i;

    for (i = 0U; i < s->n; i++)
    {
        b += s->f[i].bits;
    }
    return b;
}

/* --------------------------------------------------------------------
 * Bit packing, least significant bit first.
 * -------------------------------------------------------------------- */

static void put_bits(uint8_t *body, uint32_t *pos, uint32_t v, uint32_t bits)
{
    uint32_t i;

    for (i = 0U; i < bits; i++)
    {
        if (((v >> i) & 1U) != 0U)
        {
            body[*pos / 8U] |= (uint8_t)(1U << (*pos % 8U));
        }
        (*pos)++;
    }
}

static uint32_t get_bits(const uint8_t *body, uint32_t *pos, uint32_t bits)
{
    uint32_t v = 0U;
    uint32_t i;

    for (i = 0U; i < bits; i++)
    {
        if ((body[*pos / 8U] & (1U << (*pos % 8U))) != 0U)
        {
            v |= (1UL << i);
        }
        (*pos)++;
    }
    return v;
}

uint16_t tm_pack(tm_type_t type, uint8_t seq, const int32_t *vals,
                 uint8_t *out, uint16_t cap)
{
    const schema_t *s = schema_of((uint32_t)type);
    uint32_t len;
    uint32_t pos = 0U;
    uint32_t i;

    if ((s == NULL) || (vals == NULL) || (out == NULL))
    {
        return 0U;
    }
    len = TM_HDR_BYTES + ((body_bits(s) + 7U) / 8U);
    if (len > cap)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        out[i] = 0U;
    }
    out[0] = (uint8_t)((TM_VER << 5) | ((uint32_t)type & 0x1FU));
    out[1] = seq;

    for (i = 0U; i < s->n; i++)
    {
        const field_t *f = &s->f[i];
        uint32_t v;

        if (f->kind == K_S)
        {
            /* clamp, then keep the low `bits` of the two's complement */
            v = (uint32_t)rc_sat_s(vals[i], f->bits) & RC_FIELD_MAX(f->bits);
        }
        else if (f->bits >= 32U)
        {
            v = (uint32_t)vals[i];
        }
        else
        {
            v = rc_sat_u((vals[i] < 0) ? 0U : (uint32_t)vals[i], f->bits);
        }
        put_bits(&out[TM_HDR_BYTES], &pos, v, f->bits);
    }
    return (uint16_t)len;
}

int32_t tm_unpack(const uint8_t *buf, uint16_t len, tm_type_t *type,
                  uint8_t *seq, int32_t *vals)
{
    const schema_t *s;
    uint32_t pos = 0U;
    uint32_t i;

    if ((buf == NULL) || (len < TM_HDR_BYTES) || ((buf[0] >> 5) != TM_VER))
    {
        return -1;
    }
    s = schema_of(buf[0] & 0x1FU);
    if ((s == NULL) ||
        (len != (TM_HDR_BYTES + ((body_bits(s) + 7U) / 8U))))
    {
        return -1;
    }
    *type = (tm_type_t)(buf[0] & 0x1FU);
    *seq  = buf[1];

    for (i = 0U; i < s->n; i++)
    {
        const field_t *f = &s->f[i];
        uint32_t v = get_bits(&buf[TM_HDR_BYTES], &pos, f->bits);

        if ((f->kind == K_S) && (f->bits < 32U) &&
            ((v & (1UL << (f->bits - 1U))) != 0U))
        {
            v |= ~RC_FIELD_MAX(f->bits);            /* sign-extend */
        }
        vals[i] = (int32_t)v;
    }
    return (int32_t)s->n;
}

/* --------------------------------------------------------------------
 * JSON rendering (console sink, self-tests).
 * -------------------------------------------------------------------- */

static char sym_or_dash(uint32_t code)
{
    char c = rc_sym_char(code);

    return (c != '\0') ? c : '-';
}

bool tm_to_json(const uint8_t *buf, uint16_t len, char *out, size_t cap)
{
    static const char *const sinks[4] = { "console", "mqtt", "other", "other" };
    int32_t   vals[TM_FIELDS_MAX];
    tm_type_t type;
    uint8_t   seq;
    int32_t   n;
    int32_t   i;
    size_t    k = 0U;

    n = tm_unpack(buf, len, &type, &seq, vals);
    if ((n < 0) || (out == NULL) || (cap < 3U))
    {
        return false;
    }

#define ADD(...)                                                        \
    do {                                                                \
        if (k < cap) {                                                  \
            k += (size_t)rc_snprintf(&out[k], cap - k, __VA_ARGS__);    \
        }                                                               \
    } while (0)

    ADD("{");
    if (schema[type].show_seq != 0U)
    {
        ADD("\"seq\":%u,", (unsigned int)seq);
    }
    for (i = 0; i < n; i++)
    {
        const field_t *f = &schema[type].f[i];
        uint32_t v = (uint32_t)vals[i];

        ADD("%s\"%s\":", (i > 0) ? "," : "", f->key);
        switch (f->kind)
        {
            case K_S:
                ADD("%ld", (long)vals[i]);
                break;
            case K_LINE2:
                ADD("\"%c%c\"", ((v & 1U) != 0U) ? '1' : '0',
                    ((v & 2U) != 0U) ? '1' : '0');
                break;
            case K_SYM:
                ADD("\"%c\"", sym_or_dash(v));
                break;
            case K_SYM3:
            {
                char t[4];
                uint32_t j;
                uint32_t m = 0U;

                for (j = 0U; j < 3U; j++)
                {
                    char c = rc_sym_char((v >> (5U * j)) & 0x1FU);

                    if (c == '\0')
                    {
                        break;
                    }
                    t[m++] = c;
                }
                t[m] = '\0';
                ADD("\"%s\"", t);
                break;
            }
            case K_CHAR:
                ADD("\"%c\"", ((v > 32U) && (v < 127U) && (v != 34U) &&
                               (v != 92U)) ? (char)v : '?');
                break;
            case K_FLAGS4:
                ADD("\"%c%c%c%c\"", ((v & 1U) != 0U) ? '1' : '0',
                    ((v & 2U) != 0U) ? '1' : '0',
                    ((v & 4U) != 0U) ? '1' : '0',
                    ((v & 8U) != 0U) ? '1' : '0');
                break;
            case K_SINK:
                ADD("\"%s\"", sinks[v & 3U]);
                break;
            default:
                ADD("%lu", (unsigned long)v);
                break;
        }
    }
    ADD("}");
#undef ADD

    return k < cap;
}
