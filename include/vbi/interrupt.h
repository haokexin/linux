/*
 * interrupt.h - utility functions to read interrupt configuration data
 *
 * Copyright 2007 Wind River Systems, Inc.
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

/*
DESCRIPTION

This header file declares the vbi API for utility functions for reading the
interrupts configuration data.

*/

#ifndef _VBI_INTERRUPT_H
#define _VBI_INTERRUPT_H

#ifndef	_ASMLANGUAGE

extern int32_t vbi_find_irq(char *irq_name, int32_t irq_dir);

#endif /* _ASMLANGUAGE */

#endif  /* _VBI_INTERRUPT_H */
