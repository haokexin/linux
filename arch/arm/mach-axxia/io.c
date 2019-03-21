// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * Support for the INTEL Axxia boards based on ARM cores
 */

#include <linux/module.h>
#include <linux/types.h>
#include <asm/page.h>
#include <asm/io.h>

void __iomem *
__axxia_arch_ioremap(phys_addr_t physical_address, size_t size,
		     unsigned int flags)
{
	unsigned long pfn;
	unsigned long offset;

	pfn = (unsigned long)((physical_address >> PAGE_SHIFT) & 0xffffffffULL);
	offset = (unsigned long)(physical_address & (PAGE_SIZE - 1));

	return __arm_ioremap_pfn(pfn, offset, size, flags);
}
EXPORT_SYMBOL(__axxia_arch_ioremap);
