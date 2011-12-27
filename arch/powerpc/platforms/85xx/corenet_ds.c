/*
 * Corenet based SoC DS Setup
 *
 * Maintained by Kumar Gala (see MAINTAINERS for contact information)
 *
 * Copyright 2009-2011 Freescale Semiconductor Inc.
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
#include <linux/lmb.h>

#include <linux/fsl_usdpaa.h>

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

void __init corenet_ds_pic_init(void)
{
	struct mpic *mpic;
	struct resource r;
	struct device_node *np = NULL;
	unsigned int flags = MPIC_PRIMARY | MPIC_BIG_ENDIAN |
				MPIC_BROKEN_FRR_NIRQS | MPIC_SINGLE_DEST_CPU;

	np = of_find_node_by_type(np, "open-pic");

	if (np == NULL) {
		printk(KERN_ERR "Could not find open-pic node\n");
		return;
	}

	if (of_address_to_resource(np, 0, &r)) {
		printk(KERN_ERR "Failed to map mpic register space\n");
		of_node_put(np);
		return;
	}

	if (ppc_md.get_irq == mpic_get_coreint_irq)
		flags |= MPIC_ENABLE_COREINT;

	mpic = mpic_alloc(np, r.start, flags, 0, 256, " OpenPIC  ");
	BUG_ON(mpic == NULL);

	mpic_init(mpic);
}

/*
 * Setup the architecture
 */
#ifdef CONFIG_SMP
void __init mpc85xx_smp_init(void);
#endif

void __init corenet_ds_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
	struct pci_controller *hose;
#endif
	dma_addr_t max = 0xffffffff;

#ifdef CONFIG_SMP
	mpc85xx_smp_init();
#endif

#ifdef CONFIG_PCI
	for_each_node_by_type(np, "pci") {
		if (of_device_is_compatible(np, "fsl,p4080-pcie") ||
		    of_device_is_compatible(np, "fsl,qoriq-pcie-v2.2")) {
			fsl_add_bridge(np, 0);
			hose = pci_find_hose_for_OF_device(np);
			max = min(max, hose->dma_window_base_cur +
					hose->dma_window_size);
		}
	}
#endif

#ifdef CONFIG_SWIOTLB
	if (lmb_end_of_DRAM() > max) {
		ppc_swiotlb_enable = 1;
		set_pci_dma_ops(&swiotlb_dma_ops);
		ppc_md.pci_dma_dev_setup = pci_dma_dev_setup_swiotlb;
	}
#endif
	pr_info("%s board from Freescale Semiconductor\n", ppc_md.name);
}

static const struct of_device_id of_device_ids[] __devinitconst = {
	{
		.compatible	= "simple-bus"
	},
	{
		.compatible	= "fsl,dpaa"
	},
	{
		.compatible	= "fsl,rapidio-delta",
	},
	{
		.compatible     = "fsl,srio",
	},
	{
		.compatible	= "fsl,p4080-pcie",
	},
	{
		.compatible	= "fsl,qoriq-pcie-v2.2",
	},
	{}
};

int __init corenet_ds_publish_devices(void)
{
	return of_platform_bus_probe(NULL, of_device_ids, NULL);
}

int __init declare_of_platform_devices(void)
{
	struct device_node *np;
	struct of_device *dev;
	int err;

	err = of_platform_bus_probe(NULL, of_device_ids, NULL);
	if (err)
		return err;

	/* Now probe the fake MDIO buses */
	for_each_compatible_node(np, NULL, "fsl,p4080ds-mdio") {
		dev = of_platform_device_create(np, NULL, NULL);
		if (!dev) {
			of_node_put(np);
			return -ENOMEM;
		}
	}

	for_each_compatible_node(np, NULL, "fsl,p4080ds-xmdio") {
		dev = of_platform_device_create(np, NULL, NULL);
		if (!dev) {
			of_node_put(np);
			return -ENOMEM;
		}
	}

	for_each_compatible_node(np, NULL, "fsl,hydra-mdio") {
		dev = of_platform_device_create(np, NULL, NULL);
		if (!dev) {
			of_node_put(np);
			return -ENOMEM;
		}
	}

	return 0;
}

/* Early setup is required for large chunks of contiguous (and coarsely-aligned)
 * memory. The following shoe-horns Qman/Bman "init_early" calls into the
 * platform setup to let them parse their CCSR nodes early on. */
#ifdef CONFIG_FSL_FMAN
void __init fman_init_early(void);
#endif
#ifdef CONFIG_FSL_QMAN_CONFIG
void __init qman_init_early(void);
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
void __init bman_init_early(void);
#endif
#ifdef CONFIG_FSL_PME2_CTRL
void __init pme2_init_early(void);
#endif

__init void corenet_ds_init_early(void)
{
#ifdef CONFIG_FSL_FMAN
	fman_init_early();
#endif
#ifdef CONFIG_FSL_QMAN_CONFIG
	qman_init_early();
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
	bman_init_early();
#endif
#ifdef CONFIG_FSL_PME2_CTRL
	pme2_init_early();
#endif
#ifdef CONFIG_FSL_USDPAA_SHMEM
	fsl_usdpaa_shmem_init_early();
#endif
}

