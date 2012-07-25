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
#include <linux/memblock.h>
#include <linux/of_platform.h>

#include <linux/fsl_usdpaa.h>

#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/ppc-pci.h>
#include <mm/mmu_decl.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/mpic.h>

#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include "smp.h"

void __init corenet_ds_pic_init(void)
{
	struct mpic *mpic;
	unsigned int flags = MPIC_BIG_ENDIAN | MPIC_SINGLE_DEST_CPU |
		MPIC_NO_RESET;

	if (ppc_md.get_irq == mpic_get_coreint_irq)
		flags |= MPIC_ENABLE_COREINT;

	mpic = mpic_alloc(NULL, 0, flags, 0, 256, " OpenPIC  ");
	BUG_ON(mpic == NULL);

	mpic_init(mpic);
}

/*
 * Setup the architecture
 */
void __init corenet_ds_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
#endif

	mpc85xx_smp_init();

#ifdef CONFIG_PCI
	for_each_node_by_type(np, "pci")
		fsl_pci_setup(np);
#endif

#ifdef CONFIG_SWIOTLB
	if (memblock_end_of_DRAM() > 0xffffffff)
		ppc_swiotlb_enable = 1;
#endif
	pr_info("%s board from Freescale Semiconductor\n", ppc_md.name);
}

static struct of_device_id __initdata corenetds_pci_ids[] = {
	{ .compatible = "fsl,p4080-pcie", },
	{ .compatible = "fsl,qoriq-pcie-v2.2", },
	{ .compatible = "fsl,qoriq-pcie-v2.4", },
	{ .compatible = "fsl,qoriq-pcie-v2.3", },
	{ .compatible = "fsl,qoriq-pcie-v2.4", },
	{ .compatible = "fsl,qoriq-pcie-v3.0", },
	{},
};

int __init corenet_ds_publish_pci_device(void)
{
	struct device_node *np;
	int rc = 0;

	for_each_matching_node(np, corenetds_pci_ids) {
		rc = of_platform_bus_create(np, corenetds_pci_ids, NULL,
						NULL, false);
		if (rc)
			break;
	}

	return rc;
}

static const struct of_device_id of_device_ids[] __devinitconst = {
	{
		.compatible	= "simple-bus"
	},
	{
		.compatible	= "fsl,dpaa"
	},
	{
		.compatible	= "fsl,srio",
	},
	/* The following two are for the Freescale hypervisor */
	{
		.name		= "hypervisor",
	},
	{
		.name		= "handles",
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
	int err;

	err = of_platform_bus_probe(NULL, of_device_ids, NULL);
	if (err)
		return err;

	/* Now probe the fake MDIO buses */
	for_each_compatible_node(np, NULL, "fsl,p4080ds-mdio")
		of_platform_device_create(np, NULL, NULL);

	for_each_compatible_node(np, NULL, "fsl,p4080ds-xmdio")
		of_platform_device_create(np, NULL, NULL);

	for_each_compatible_node(np, NULL, "fsl,hydra-mdio")
		of_platform_device_create(np, NULL, NULL);

	for_each_compatible_node(np, NULL, "fsl,hydra-xmdio")
		of_platform_device_create(np, NULL, NULL);

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
#ifdef CONFIG_FSL_USDPAA
	fsl_usdpaa_init_early();
#endif
}

