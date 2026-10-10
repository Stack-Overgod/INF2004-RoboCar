/*
 *  rc_prelude.h
 *
 *  Include this FIRST in every .c file in this tree, before anything else.
 *
 *  Why it exists
 *  -------------
 *  include/tk/syslib.h line 125 does:
 *
 *      typedef SZ size_t;          and SZ is `typedef W SZ;`, i.e. SIGNED int
 *
 *  newlib's stddef.h declares size_t as UNSIGNED int. Pull in <string.h> or
 *  <stdio.h> in the same translation unit as a mtk3 header and the two
 *  collide:
 *
 *      error: conflicting types for 'size_t'; have 'unsigned int'
 *
 *  The port's own sources never hit this because none of them include libc
 *  string or stdio headers; the kernel uses its private knl_memset and
 *  friends from kernel/knlinc/tstdlib.h, which application code cannot see.
 *
 *  mtk3 leaves a documented escape hatch, PROHIBIT_DEF_SIZE_T, which
 *  suppresses its own typedef and lets newlib's win. Defining it here, once,
 *  before any tk header is reached, is what makes <string.h> usable.
 *
 *  ABI note: Kmalloc and Kcalloc are prototyped with size_t. The kernel was
 *  compiled with the signed version, this tree sees the unsigned one. Both
 *  are 4 bytes and pass in the same register on AAPCS, so the mismatch is
 *  harmless. It would stop being harmless if either side ever passed a
 *  negative size, which is already a bug.
 */

#ifndef RC_PRELUDE_H
#define RC_PRELUDE_H

/*
 *  Order matters. newlib's stddef.h must define size_t BEFORE syslib.h is
 *  reached, because syslib.h still uses size_t in the Kmalloc prototypes
 *  even when PROHIBIT_DEF_SIZE_T suppresses its own typedef.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>

#endif /* RC_PRELUDE_H */
