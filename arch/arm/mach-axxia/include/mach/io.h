/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2021 INTEL Corporation
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <linux/types.h>

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

/*
 * Make the first argument to ioremap() be phys_addr_t (64 bits in this
 * case) instead of unsigned long.  When __arch_ioremap is defiend,
 * __arch_iounmap must be defined also.  Just use the default for
 * iounmap().
 */

void __iomem *__axxia_arch_ioremap(phys_addr_t a, size_t b, unsigned int c);
#define __arch_ioremap __axxia_arch_ioremap
#define __arch_iounmap __arm_iounmap

#endif
