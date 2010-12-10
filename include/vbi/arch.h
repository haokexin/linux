/*
 * arch.h - vbi architecture specific definitions
 *
 * Copyright 2008 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#ifndef _VBI_ARCH_H
#define _VBI_ARCH_H

/* endian types selectable by the arch specific header files */

#define __VBI_LITTLE_ENDIAN 1234  /* least-significant byte first         */
#define __VBI_BIG_ENDIAN    4321  /* most-significant byte first          */
#define __VBI_PDP_ENDIAN    3412  /* LSB first in word, MSW first in long */

/* include the appropriate arch specific header file */
#include <asm/arch_vbi.h>

/* ensure the arch has specified a byte ordering */
#if !defined(__VBI_BYTE_ORDER)
# error: Architecture has not defined a byte order!
#endif

#endif /* _VBI_ARCH_H */
