/*
 * Author: Andy Fleming <afleming@freescale.com>
 * 	   Kumar Gala <galak@kernel.crashing.org>
 *
 * Copyright 2006-2008 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/pci-bridge.h>
#include <asm/mpic.h>
#include <asm/cacheflush.h>

#include <sysdev/fsl_soc.h>

extern unsigned long __secondary_hold_acknowledge;
extern void __early_start(void);

#define BOOT_ENTRY_ADDR_UPPER	0
#define BOOT_ENTRY_ADDR_LOWER	1
#define BOOT_ENTRY_R3_UPPER	2
#define BOOT_ENTRY_R3_LOWER	3
#define BOOT_ENTRY_RESV		4
#define BOOT_ENTRY_PIR		5
#define BOOT_ENTRY_R6_UPPER	6
#define BOOT_ENTRY_R6_LOWER	7
#define NUM_BOOT_ENTRY		8
#define SIZE_BOOT_ENTRY		(NUM_BOOT_ENTRY * sizeof(u32))

void startcore(u64 entryPt, int nr)
{
	unsigned long flags;
	const u64 *cpu_rel_addr;
	struct device_node *np;
	__iomem u32 *bptr_vaddr;

	local_irq_save(flags);

	np = of_get_cpu_node(nr, NULL);
	cpu_rel_addr = of_get_property(np, "cpu-release-addr", NULL);

	if (cpu_rel_addr == NULL) {
		printk(KERN_ERR "No cpu-release-addr for cpu %d\n", nr);
		return;
	}

	printk(KERN_DEBUG "CPU%d RELEASE at %llx\n", nr, *cpu_rel_addr);

	/* Map the spin table */
	bptr_vaddr = ioremap(*cpu_rel_addr, SIZE_BOOT_ENTRY);
	if (bptr_vaddr == NULL)
		return;

	out_be32(bptr_vaddr + BOOT_ENTRY_PIR, nr);
#ifdef CONFIG_PHYS_64BIT
	out_be32(bptr_vaddr + BOOT_ENTRY_ADDR_UPPER, (u32)(entryPt >> 32));
#endif
	wmb();
	out_be32(bptr_vaddr + BOOT_ENTRY_ADDR_LOWER, (u32)entryPt);

	/* Wait a bit for the CPU to ack. */
	while (in_be32(bptr_vaddr + BOOT_ENTRY_ADDR_LOWER) != 3);

	out_be32(bptr_vaddr + BOOT_ENTRY_ADDR_LOWER, 1);

	iounmap(bptr_vaddr);

	local_irq_restore(flags);
	printk(KERN_DEBUG "CPU%d RELEASE at %llx\n", nr, *cpu_rel_addr);

}

EXPORT_SYMBOL(startcore);
