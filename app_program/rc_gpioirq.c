/*
 *  rc_gpioirq.c
 *
 *  IO_BANK0 interrupt demultiplexer.
 *
 *  Register layout, from the RP2040 datasheet section 2.19.6. The port's
 *  sysdef.h stops at GPIO_CTRL, so the interrupt registers are defined
 *  here rather than patched into the kernel tree.
 *
 *  Each GPIO gets a 4 bit field: bit 0 level low, bit 1 level high,
 *  bit 2 edge low, bit 3 edge high. Eight pins per 32 bit register.
 */

#include "rc_prelude.h"
#include "rc_gpioirq.h"
#include "rc_config.h"
#include "rc_time.h"

/* IO_BANK0 interrupt registers, offsets from IO_BANK0_BASE. */
#define IO_INTR(n)          (IO_BANK0_BASE + 0x0F0U + ((n) * 4U))
#define IO_PROC0_INTE(n)    (IO_BANK0_BASE + 0x100U + ((n) * 4U))
#define IO_PROC0_INTF(n)    (IO_BANK0_BASE + 0x110U + ((n) * 4U))
#define IO_PROC0_INTS(n)    (IO_BANK0_BASE + 0x120U + ((n) * 4U))

#define EDGE_LOW_BIT        (2U)
#define EDGE_HIGH_BIT       (3U)

/* IO_IRQ_BANK0 is RP2040 interrupt 13. */
#define INTNO_IO_BANK0      (13U)
#define INTPRI_IO_BANK0     (2)

#define PIN_MAX             (30U)

typedef struct {
    rc_gpio_isr_t cb;
    void         *ctx;
    uint8_t       edges;
    bool          in_use;
} pin_entry_t;

static pin_entry_t pins[PIN_MAX];

/* One bit per INTS register that has at least one registered pin, so the
 * handler can skip registers nobody is using. */
static volatile uint8_t reg_used[4];

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

static uint32_t pin_field_shift(uint32_t pin)
{
    return (pin % 8U) * 4U;
}

static uint32_t pin_reg_index(uint32_t pin)
{
    return pin / 8U;
}

static uint32_t edge_bits(rc_edge_t edges)
{
    uint32_t bits = 0U;

    if (((uint32_t)edges & (uint32_t)RC_EDGE_FALL) != 0U) {
        bits |= (1U << EDGE_LOW_BIT);
    }
    if (((uint32_t)edges & (uint32_t)RC_EDGE_RISE) != 0U) {
        bits |= (1U << EDGE_HIGH_BIT);
    }
    return bits;
}

/* ------------------------------------------------------------------ *
 *  The handler
 *
 *  One pass over the four INTS registers. The timestamp is taken once at
 *  entry so every pin that fired in the same batch shares a consistent
 *  time, and so the cost of the read is paid once.
 * ------------------------------------------------------------------ */

static void gpio_bank0_handler(UINT intno)
{
    uint32_t t_us = rc_time_us();
    uint32_t reg;
    uint32_t status;
    uint32_t pin;
    uint32_t shift;
    uint32_t fired;
    uint32_t pending;

    (void)intno;

    /*
     *  Only the registers that actually have a registered pin are read,
     *  and within one register only the bits that fired are visited. The
     *  earlier version walked all 8 pin slots of all 4 registers on every
     *  interrupt, which is 32 iterations to service one encoder edge.
     *
     *  The timestamp is taken once at entry so every pin in the same batch
     *  shares a consistent time.
     */
    for (reg = 0U; reg < 4U; reg++) {
        if (reg_used[reg] == 0U) {
            continue;
        }
        status = in_w(IO_PROC0_INTS(reg));
        if (status == 0U) {
            continue;
        }

        /* Collapse the 4-bit-per-pin edge fields to one bit per pin. */
        pending = 0U;
        for (pin = 0U; pin < 8U; pin++) {
            if (((status >> (pin * 4U)) & 0xCU) != 0U) {
                pending |= (1U << pin);
            }
        }

        while (pending != 0U) {
            pin   = (uint32_t)__builtin_ctz(pending);
            pending &= ~(1U << pin);

            shift = pin * 4U;
            fired = (status >> shift) & 0xCU;

            /* Acknowledge before calling out, so an edge arriving during
             * the handler is not lost. */
            out_w(IO_INTR(reg), fired << shift);

            {
                uint32_t gp = (reg * 8U) + pin;

                if ((gp < PIN_MAX) && pins[gp].in_use
                    && (pins[gp].cb != NULL)) {
                    bool level = (gpio_get_val(gp) != 0U);
                    pins[gp].cb(gp, level, t_us, pins[gp].ctx);
                }
            }
        }
    }

    EndOfInt(INTNO_IO_BANK0);
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

rc_result_t rc_gpioirq_init(void)
{
    T_DINT dint;
    ER     er;
    uint32_t reg;

    for (reg = 0U; reg < 4U; reg++) {
        reg_used[reg] = 0U;
    }
    for (reg = 0U; reg < PIN_MAX; reg++) {
        pins[reg].in_use = false;
        pins[reg].cb     = NULL;
        pins[reg].ctx    = NULL;
    }

    /* Mask everything before claiming the vector. */
    for (reg = 0U; reg < 4U; reg++) {
        out_w(IO_PROC0_INTE(reg), 0U);
        out_w(IO_INTR(reg), 0xFFFFFFFFU);
    }

    dint.intatr = TA_HLNG;
    dint.inthdr = gpio_bank0_handler;

    er = tk_def_int(INTNO_IO_BANK0, &dint);
    if (er != E_OK) {
        return RC_ERR_HARDWARE;
    }

    ClearInt(INTNO_IO_BANK0);
    EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);

    return RC_OK;
}

rc_result_t rc_gpioirq_attach(uint32_t pin,
                              rc_edge_t edges,
                              int8_t pull,
                              rc_gpio_isr_t cb,
                              void *ctx)
{
    uint32_t pad;
    uint32_t sts;

    if ((pin >= PIN_MAX) || (cb == NULL)) {
        return RC_ERR_PARAM;
    }

    (void)gpio_set_pin(pin, GPIO_MODE_IN);

    /* gpio_set_pin leaves the pad at its reset pulls, so set them here.
     * Schmitt trigger is on by default and we want it: the LM393 output
     * and a divided HC-SR04 echo both have slow-ish edges. */
    pad = GPIO_IE | GPIO_SHEMITT;
    if (pull > 0) {
        pad |= GPIO_PUE;
    } else if (pull < 0) {
        pad |= GPIO_PDE;
    } else {
        /* no pull */
    }
    out_w(GPIO(pin), pad);

    DI(sts);
    pins[pin].cb     = cb;
    pins[pin].ctx    = ctx;
    pins[pin].edges  = (uint8_t)edges;
    pins[pin].in_use = true;
    reg_used[pin_reg_index(pin)] = 1U;
    EI(sts);

    return rc_gpioirq_enable(pin, true);
}

rc_result_t rc_gpioirq_detach(uint32_t pin)
{
    uint32_t sts;

    if (pin >= PIN_MAX) {
        return RC_ERR_PARAM;
    }

    (void)rc_gpioirq_enable(pin, false);

    DI(sts);
    pins[pin].in_use = false;
    pins[pin].cb     = NULL;
    EI(sts);

    return RC_OK;
}

rc_result_t rc_gpioirq_enable(uint32_t pin, bool on)
{
    uint32_t reg;
    uint32_t shift;
    uint32_t bits;
    uint32_t sts;

    if ((pin >= PIN_MAX) || !pins[pin].in_use) {
        return RC_ERR_PARAM;
    }

    reg   = pin_reg_index(pin);
    shift = pin_field_shift(pin);
    bits  = edge_bits((rc_edge_t)pins[pin].edges) << shift;

    DI(sts);
    if (on) {
        out_w(IO_INTR(reg), bits);          /* drop anything stale */
        set_w(IO_PROC0_INTE(reg), bits);
    } else {
        clr_w(IO_PROC0_INTE(reg), bits);
    }
    EI(sts);

    return RC_OK;
}
