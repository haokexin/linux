/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Copyright (C) 2011 Wind River Systems, Inc.
 */

/*
 * exports.c - Export necessary VBIs for kernel modules
 */

#include <linux/module.h>
#include <vbi/vbi.h>

EXPORT_SYMBOL(vbi_vb_suspend);
EXPORT_SYMBOL(vbi_vb_resume);

EXPORT_SYMBOL(vbi_vb_read_mem);
EXPORT_SYMBOL(vbi_vb_write_mem);

EXPORT_SYMBOL(vbi_vb_read_reg);
EXPORT_SYMBOL(vbi_vb_write_reg);

EXPORT_SYMBOL(vbi_vb_find_board_config);
EXPORT_SYMBOL(vbi_find_shmem);

EXPORT_SYMBOL(vbi_unmask_vioapic_irq);
EXPORT_SYMBOL(vbi_send_vioapic_irq);
EXPORT_SYMBOL(vbi_find_irq);

EXPORT_SYMBOL(vbi_ns_lookup);

EXPORT_SYMBOL(vbi_send);
