/*
 * P4080 DS Setup
 *
 * Copyright (C) 2009-2010 Wind River Systems, Inc.
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
#include <linux/lmb.h>

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

#include <asm/wrhv.h>
#include <vbi/vbi.h>
#include "corenet_ds.h"

extern struct vb_config *wr_config;
extern struct vb_status *wr_status;
extern struct vb_control *wr_control;
extern int wrhv_set_law_base(int index, unsigned long long addr);
extern unsigned long long wrhv_get_law_base(int index);
extern int wrhv_set_law_attr(int index, unsigned int attr);
extern int wrhv_get_law_attr(int index);

static int __init p4080_is_svr_rev(int maj, int min)
{
	u32 svr = mfspr(SPRN_SVR);

	return ((svr >> 4) & 0xf) == maj && (svr & 0xf) == min;
}

static int __init p4080_errata_gen8(void)
{
	struct device_node *fp, *np;
	struct property *pp;
	int *p;

	if (!p4080_is_svr_rev(1, 0))
		return 0;

	/* Find 10GbE of FMAN2 */
	for_each_compatible_node(fp, NULL, "fsl,fman") {
		p = (int *)of_get_property(fp, "cell-index", NULL);
		if (!p || (*p != 1))
			continue;

		np = of_find_compatible_node(fp, NULL, "fsl,fman-10g-mac");
		if (np)
			break;
	}

	if (!np)
		return 0;

	pp = of_find_property(np, "phy-handle", NULL);
	if (pp)
		prom_remove_property(np, pp);

	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		pr_err("Alloc memory failed\n");
		return -1;
	}

	p = kmalloc(sizeof(int) * 5, GFP_KERNEL);
	if (!p) {
		pr_err("Alloc memory failed\n");
		return -1;
	}

	p[0] = 9;	/* phy id */
	p[1] = 1;	/* duplex */
	p[2] = 10000;	/* speed */
	p[3] = 0;	/* pause */
	p[4] = 0;	/* asym_pause */

	pp->name = "fixed-link";
	pp->value = p;
	pp->length = 5;

	prom_add_property(np, pp);
	of_node_put(np);

	pr_info("Workaround for Errata GEN8 enabled\n");
	return 0;
}
machine_postcore_initcall(p4080_ds, p4080_errata_gen8);

static int __init p4080_errata_serdes9(struct corenet_serdes *sd)
{
	int i;

	/* Fix for XAUI on FMAN2. We only support serdes protocol 0x10 now. */
	for (i = 16; i < 20; i++)
		clrsetbits_be32(&sd->lane[i].ttlcr0, SRDS_TTLCR0_FLT_SEL_MASK,
				0x03000000 | SRDS_TTLCR0_PM_DIS);


	clrbits32(&sd->bank[2].pllcr1, SRDS_PLLCR1_PLL_BWSEL);

	for (i = 16; i < 20; i++)
		clrbits32(&sd->lane[i].gcr0, SRDS_GCR0_RRST);

	mdelay(1);
	for (i = 16; i < 20; i++)
		setbits32(&sd->lane[i].gcr0, SRDS_GCR0_RRST);

	pr_info("Workaround for Errata SERDES9 enabled\n");
	return 0;
}

static int __init p4080_serdes_errata(void)
{
	struct device_node *np;
	struct corenet_serdes *sd;

	np = of_find_compatible_node(NULL, NULL, "fsl,p4080-serdes");
	if (!np) {
		pr_err("fsl,p4080-serdes device node not found\n");
		return -1;
	}

	sd = of_iomap(np, 0);
	if (!sd) {
		pr_err("%s ioremap failed\n", np->full_name);
		of_node_put(np);
		return -1;
	}

	p4080_errata_serdes9(sd);

	iounmap(sd);
	of_node_put(np);
	return 0;
}
machine_postcore_initcall(p4080_ds, p4080_serdes_errata);

static void __init wrhv_mpc85xx_pic_init(void)
{
	wrhv_init_irq();
}

#ifdef CONFIG_PCI
static int primary_phb_addr;
#endif

/*
 * Setup the architecture
 */
#ifdef CONFIG_SMP
extern void __init wrhv_smp_init(void);
#endif

/*
 * The hypervisor needs to know which FMAN and which DTSEC you
 * are trying to access via the MDIO as the access functions
 * need that information.
 * This information should be encoded into the "bus" element
 * of the MDIO_MSG struct.

 * fman 4 bits  0-1
 * dtsec 4 bits 0-3

 * 0x000000<fman><dtsec>
 */
static uint32_t p4080_mdio_bus[PHY_MAX_ADDR];

static int __init p4080_mdio_bus_init(void)
{
	struct device_node *np, *phy_np;
	int i, j;
	uint32_t bus = 0;
	char *mdio_bus[] = {
			"/soc@fe000000/fman@400000/ethernet@e0000",
			"/soc@fe000000/fman@400000/ethernet@e2000",
			"/soc@fe000000/fman@400000/ethernet@e4000",
			"/soc@fe000000/fman@400000/ethernet@e6000",
			"/soc@fe000000/fman@500000/ethernet@e0000",
			"/soc@fe000000/fman@500000/ethernet@e2000",
			"/soc@fe000000/fman@500000/ethernet@e4000",
			"/soc@fe000000/fman@500000/ethernet@e6000",
	};

	for (i = 0; i < PHY_MAX_ADDR; i++) {
		for_each_compatible_node(np, NULL, "fsl,p4080-fman-1g-mac") {
			const u32 *phandle;
			const u32 *reg;

			phandle = of_get_property(np, "phy-handle", NULL);
			phy_np = of_find_node_by_phandle(*phandle);
			reg = of_get_property(phy_np, "reg", NULL);
			if (reg && (*reg == i))
				break;
		}
		if (!np) {
			bus = 0;
			goto get_bus;
		}

		for (j = 0; j < ARRAY_SIZE(mdio_bus); j++) {
			if (!(strcmp(np->full_name,  mdio_bus[j]))) {
				bus = j;
				/*
				 * mask fman1
				*/
				if (j > 3) {
					bus -= 4;
					bus |= 1 << 4;
				}
				break;
			}
		}

get_bus:
		p4080_mdio_bus[i] = bus;
	}

	return 0;
}
subsys_initcall(p4080_mdio_bus_init);

uint32_t p4080_get_vb_mdio_bus(struct mii_bus *mii_bus, int addr)
{
	if (addr > PHY_MAX_ADDR)
		return -1;

	return p4080_mdio_bus[addr];
}

static void __init wrhv_p4080_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
	struct pci_controller *hose;
#endif
	dma_addr_t max = 0xffffffff;

	if (ppc_md.progress)
		ppc_md.progress("wrhv_p4080_setup_arch()", 0);

#ifdef CONFIG_SMP
	wrhv_smp_init();
#endif

	get_hv_bsp_server_handle();
	wrhv_cpu_freq = get_bsp_clock_freq();

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,p4080-pcie") {
		struct resource rsrc;
		of_address_to_resource(np, 0, &rsrc);
		if ((rsrc.start & 0xfffff) == primary_phb_addr)
			fsl_add_bridge(np, 1);
		else
			fsl_add_bridge(np, 0);

		hose = pci_find_hose_for_OF_device(np);
		max = min(max, hose->dma_window_base_cur +
				hose->dma_window_size);

		ppc_setup_pci_law(np);
	}
#endif

#ifdef CONFIG_SWIOTLB
	if (lmb_end_of_DRAM() > max) {
		ppc_swiotlb_enable = 1;
		set_pci_dma_ops(&swiotlb_dma_ops);
		ppc_md.pci_dma_dev_setup = pci_dma_dev_setup_swiotlb;
	}
#endif
	printk(KERN_INFO "P4080 DS board from Freescale Semiconductor\n");
}

extern int vsc824x_add_skew(struct phy_device *phydev);
#define PHY_ID_VSC8244	0x000fc6c0
static int __init board_fixups(void)
{
	phy_register_fixup_for_uid(PHY_ID_VSC8244, 0xfffff, vsc824x_add_skew);

	return 0;
}
machine_device_initcall(p4080_ds, board_fixups);

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
	{}
};

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

	return 0;
}
machine_device_initcall(p4080_ds, declare_of_platform_devices);

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init wrhv_p4080_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	/* wr_config should have been initialized in wrhv_init(),
	 * continue to complete the vbi initialization here.
	 */
	wrhv_mapping(); /* Map vb_config structure */
	vbi_init(wr_config);

	strncpy(cmd_line, VBI_BOOTLINE_ADDR_GET(), VB_MAX_BOOTLINE_LENGTH - 1);
	cmd_line[VB_MAX_BOOTLINE_LENGTH - 1] = 0;

	/* Save command line for /proc/cmdline */
	strlcpy(boot_command_line, cmd_line, COMMAND_LINE_SIZE);

	if (of_flat_dt_is_compatible(root, "fsl,P4080DS")) {
#ifdef CONFIG_PCI
		/* xxx - galak */
		primary_phb_addr = 0x8000;
#endif
		return 1;
	} else {
		return 0;
	}
}

/* Early setup is required for large chunks of contiguous (and coarsely-aligned)
 * memory. The following shoe-horns Qman/Bman "init_early" calls into the
 * platform setup to let them parse their CCSR nodes early on. */
#ifdef CONFIG_FSL_QMAN_CONFIG
void __init qman_init_early(void);
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
void __init bman_init_early(void);
#endif
#ifdef CONFIG_FSL_PME2_CTRL
void __init pme2_init_early(void);
#endif

static __init void p4080_init_early(void)
{
	if (system_state != SYSTEM_RUNNING) {
#ifdef CONFIG_FSL_QMAN_CONFIG
	qman_init_early();
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
	bman_init_early();
#endif
#ifdef CONFIG_FSL_PME2_CTRL
	pme2_init_early();
#endif
	}
}

void wrhv_setup_msr_for_ap(VBI_HREG_SET_CMPLX_QUALIFIED *regs)
{
	/*
	 * The MSR value depends on hypervisor's preparetion for GOS.
	 * Hypervisor has prepared 2 TLB entry for GOS, hence here we don't
	 * have to set IS | DS. But e500mc will use kernel space 1, aka
	 * MSR_IS | MSR_DS, the same as E500. Refer to head_wrhv_p4080.S to
	 * see e500mc GOS run space 1. And thanks to e500mc's vt mode, we
	 * don't need PR bit at the entry point of GOS and the interrupts
	 * are masked as well. MSR_GS will good enough for e500mc. And
	 * GOS will manage (MSR_CE | MSR_ME | MSR_EE) bits with MSR_KERNEL
	 * without hypervisor's involvement.
	 */
	regs->vbiRegSet.hreg32.msr = MSR_GS;
}

define_machine(p4080_ds) {
	.name			= "Wind River Hypervisor P4080 DS",
	.probe			= wrhv_p4080_probe,
	.setup_arch		= wrhv_p4080_setup_arch,
	.init_IRQ		= wrhv_mpc85xx_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
	.enable_pci_law 	= wrhv_enable_pci_law,
#endif
	.get_irq		= wrhv_vioapic_get_irq,
	.get_direct_irq		= wrhv_get_direct_irq,
	.restart		= wrhv_restart,
	.calibrate_decr		= wrhv_calibrate_decr,
	.progress		= udbg_progress,
	.init_early		= p4080_init_early,
	.power_save		= wrhv_power_save,
	.set_law_base		= wrhv_set_law_base,
	.get_law_base           = wrhv_get_law_base,
	.set_law_attr		= wrhv_set_law_attr,
	.get_law_attr		= wrhv_get_law_attr,
	.get_mdio_bus		= p4080_get_vb_mdio_bus,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= cpu_die,
#endif
};
