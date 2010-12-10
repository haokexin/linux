/*
 * interrupt.c - virtual board interrupt configuration utility
 *
 * Copyright (c) 2007-2009 Wind River Systems, Inc.
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

/*
 * This module contains the code for accessing the the interrupts configuration
 * of the virtual board.
 */

/*#define DEBUG 1*/
#ifdef DEBUG
#define DEBUGM(x) x
#else
#define DEBUGM(x)
#endif

#include <linux/string.h>
#include <vbi/vbi.h>
#include <linux/types.h>
#include <vbi/private.h>
#include <linux/module.h>

/*
 * vbi_find_irq - determine the irq for a specified name
 *
 * This function finds the irq number of a named interrupt from  the virtual
 * board configuration information.
 *
 * The <irq_dir> is either input or output. VB_INPUT_INT for input,
 * VB_OUTPUT_INT for output.
 *
 */

int32_t vbi_find_irq(char *irq_name, int32_t irq_dir)
{
	struct vb_config *config = VBI_CONFIG_ADDR_GET();
	struct vb_int_info *info = config->interruptConfiguration;
	int32_t num  = config->num_ints;
	int32_t i;

	for (i = 0; i < num; i++, info++) {
		if (irq_dir != (int32_t)(info->irq_dir))
			continue;

		if (!strncmp((char *)irq_name, (char *)info->irq_name,
					VB_NAMELEN)) {
			/* Found */
			return (int32_t)info->irq_num;
		}
	}
	return VBI_INVALID_IRQ; /* no match */
}

EXPORT_SYMBOL(vbi_find_irq);

