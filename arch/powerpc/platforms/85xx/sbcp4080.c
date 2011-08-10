/*
 * Wind River SBCP4080 setup and early boot code.
 *
 * Copyright 2011 Wind River Systems Inc.
 *
 * By Weiwei Wang <weiwei.wang@windriver.com>
 *
 * Based largely on the P4080 DS support - Copyright 2009-2010 Freescale Inc.
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/phy.h>

#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <mm/mmu_decl.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/mpic.h>

#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#include "corenet_ds.h"

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init sbcp4080_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(root, "SBCP4080");
}

define_machine(sbcp4080) {
	.name			= "SBCP4080",
	.probe			= sbcp4080_probe,
	.setup_arch		= corenet_ds_setup_arch,
	.init_IRQ		= corenet_ds_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
#endif
	.get_irq		= mpic_get_coreint_irq,
	.restart		= fsl_rstcr_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
	.power_save		= e500_idle,
	.init_early		= corenet_ds_init_early,
};

machine_device_initcall(sbcp4080, declare_of_platform_devices);

#ifdef CONFIG_SWIOTLB
machine_arch_initcall(sbcp4080, swiotlb_setup_bus_notifier);
#endif
