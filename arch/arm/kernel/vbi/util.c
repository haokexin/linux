/*
 * util.c - utilities routines for guest OS para-virtualization
 *
 * Copyright (c) 2011 Wind River Systems, Inc.
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

#include <asm/page.h>
#include <linux/module.h>
#include <vbi/interface.h>
#include <vbi/vmmu.h>
#include <vbi/syscall.h>
#include <vbi/vbi.h>

extern struct vb_config *wr_config;
extern struct vb_status *wr_status;
extern struct vb_control *wr_control;

/*
 * vb_memsize_get should not be called before wr_config is initialized
 */
unsigned int vb_memsize_get(void)
{
	if (wr_config == (struct vb_config *)(-1))
		return 0;
	return VBI_MEM_SIZE_GET();
}

unsigned int vb_context_get(void)
{
	if (wr_config == (struct vb_config *)(-1))
		return 0xbadc0de0;
	return VBI_CONTEXT_ID_GET();
}

void vb__flush_dcache_icache(void *start)
{
	vbi_flush_icache(start, PAGE_SIZE);
	vbi_flush_dcache(start, PAGE_SIZE);
}

void vb_flush_dcache_range(unsigned long start, unsigned long stop)
{
	vbi_flush_dcache((void *) start, (stop - start + 1));
}

void vb__flush_icache_range(unsigned long start, unsigned long stop)
{
	vbi_update_text_cache((void *) start, (stop - start + 1));
}

void vb__flush_dcache_icache_phys(unsigned long physaddr)
{
	vbi_flush_icache((void *) physaddr, PAGE_SIZE);
	vbi_flush_dcache((void *) physaddr, PAGE_SIZE);
}
