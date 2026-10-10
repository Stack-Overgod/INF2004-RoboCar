/*
 *  rc_fmt.c
 */

#include "rc_fmt.h"

#define DIGITS_MAX      (12U)   /* -2147483648 plus terminator */

typedef struct {
    char    *buf;
    uint32_t size;
    uint32_t pos;
} sink_t;

static void put(sink_t *s, char c)
{
    /* Always leave room for the terminator. Truncate rather than overflow. */
    if ((s->size > 0U) && (s->pos < (s->size - 1U))) {
        s->buf[s->pos] = c;
    }
    s->pos++;
}

static void put_str(sink_t *s, const char *str)
{
    if (str == NULL) {
        str = "(null)";
    }
    while (*str != '\0') {
        put(s, *str);
        str++;
    }
}

static void put_uint(sink_t *s, uint32_t v)
{
    char     tmp[DIGITS_MAX];
    uint32_t n = 0U;

    if (v == 0U) {
        put(s, '0');
        return;
    }
    while ((v > 0U) && (n < DIGITS_MAX)) {
        tmp[n] = (char)('0' + (char)(v % 10U));
        n++;
        v /= 10U;
    }
    while (n > 0U) {
        n--;
        put(s, tmp[n]);
    }
}

static void put_int(sink_t *s, int32_t v)
{
    uint32_t mag;

    if (v < 0) {
        put(s, '-');
        /* Negating INT32_MIN overflows, so build the magnitude in unsigned
         * space instead of writing -v. */
        mag = (uint32_t)(-(v + 1)) + 1U;
    } else {
        mag = (uint32_t)v;
    }
    put_uint(s, mag);
}

int32_t rc_vsnprintf(char *buf, uint32_t size, const char *fmt, va_list ap)
{
    sink_t s;
    bool   is_long;

    s.buf  = buf;
    s.size = size;
    s.pos  = 0U;

    if ((buf == NULL) || (fmt == NULL)) {
        return 0;
    }

    while (*fmt != '\0') {
        if (*fmt != '%') {
            put(&s, *fmt);
            fmt++;
            continue;
        }

        fmt++;                      /* step past '%' */
        is_long = false;

        if (*fmt == 'l') {
            is_long = true;
            fmt++;
        }

        switch (*fmt) {
        case 'd':
        case 'i':
            if (is_long) {
                put_int(&s, (int32_t)va_arg(ap, long));
            } else {
                put_int(&s, (int32_t)va_arg(ap, int));
            }
            break;
        case 'u':
            if (is_long) {
                put_uint(&s, (uint32_t)va_arg(ap, unsigned long));
            } else {
                put_uint(&s, (uint32_t)va_arg(ap, unsigned int));
            }
            break;
        case 'c':
            /* char promotes to int through varargs. */
            put(&s, (char)va_arg(ap, int));
            break;
        case 's':
            put_str(&s, va_arg(ap, const char *));
            break;
        case '%':
            put(&s, '%');
            break;
        case '\0':
            /* Trailing '%' with nothing after it. Stop rather than run off
             * the end of the string. */
            continue;
        default:
            /* Unknown conversion: emit it verbatim so the bug is visible in
             * the output instead of silently swallowed. */
            put(&s, '%');
            put(&s, *fmt);
            break;
        }
        fmt++;
    }

    if (size > 0U) {
        buf[(s.pos < (size - 1U)) ? s.pos : (size - 1U)] = '\0';
    }

    return (int32_t)s.pos;
}

int32_t rc_snprintf(char *buf, uint32_t size, const char *fmt, ...)
{
    va_list ap;
    int32_t n;

    va_start(ap, fmt);
    n = rc_vsnprintf(buf, size, fmt, ap);
    va_end(ap);

    return n;
}

uint32_t rc_strlen(const char *s)
{
    uint32_t n = 0U;

    if (s == NULL) {
        return 0U;
    }
    while (s[n] != '\0') {
        n++;
    }
    return n;
}
