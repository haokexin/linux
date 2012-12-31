/*
 * MPC85xx/86xx PCI Express structure define
 *
 * Copyright 2007,2011,2012 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifdef __KERNEL__
#ifndef __POWERPC_FSL_PCI_H
#define __POWERPC_FSL_PCI_H

#include <asm/pci-bridge.h>

#define PCIE_LTSSM	0x0404		/* PCIE Link Training and Status */
#define PCIE_LTSSM_L0	0x16		/* L0 state */
#define PCIE_IP_REV_2_2		0x02080202 /* PCIE IP block version Rev2.2 */
#define PIWAR_EN		0x80000000	/* Enable */
#define PIWAR_PF		0x20000000	/* prefetch */
#define PIWAR_TGI_LOCAL		0x00f00000	/* target - local memory */
#define PIWAR_READ_SNOOP	0x00050000
#define PIWAR_WRITE_SNOOP	0x00005000
#define PIWAR_SZ_MASK          0x0000003f

/* PCI/PCI Express outbound window reg */
struct pci_outbound_window_regs {
	__be32	potar;	/* 0x.0 - Outbound translation address register */
	__be32	potear;	/* 0x.4 - Outbound translation extended address register */
	__be32	powbar;	/* 0x.8 - Outbound window base address register */
	u8	res1[4];
	__be32	powar;	/* 0x.10 - Outbound window attributes register */
	u8	res2[12];
};

/* PCI/PCI Express inbound window reg */
struct pci_inbound_window_regs {
	__be32	pitar;	/* 0x.0 - Inbound translation address register */
	u8	res1[4];
	__be32	piwbar;	/* 0x.8 - Inbound window base address register */
	__be32	piwbear;	/* 0x.c - Inbound window base extended address register */
	__be32	piwar;	/* 0x.10 - Inbound window attributes register */
	u8	res2[12];
};

/* PCI/PCI Express IO block registers for 85xx/86xx */
struct ccsr_pci {
	__be32	config_addr;		/* 0x.000 - PCI/PCIE Configuration Address Register */
	__be32	config_data;		/* 0x.004 - PCI/PCIE Configuration Data Register */
	__be32	int_ack;		/* 0x.008 - PCI Interrupt Acknowledge Register */
	__be32	pex_otb_cpl_tor;	/* 0x.00c - PCIE Outbound completion timeout register */
	__be32	pex_conf_tor;		/* 0x.010 - PCIE configuration timeout register */
	__be32	pex_config;		/* 0x.014 - PCIE CONFIG Register */
	__be32	pex_int_status;		/* 0x.018 - PCIE interrupt status */
	u8	res2[4];
	__be32	pex_pme_mes_dr;		/* 0x.020 - PCIE PME and message detect register */
	__be32	pex_pme_mes_disr;	/* 0x.024 - PCIE PME and message disable register */
	__be32	pex_pme_mes_ier;	/* 0x.028 - PCIE PME and message interrupt enable register */
	__be32	pex_pmcr;		/* 0x.02c - PCIE power management command register */
	u8	res3[3016];
	__be32	block_rev1;	/* 0x.bf8 - PCIE Block Revision register 1 */
	__be32	block_rev2;	/* 0x.bfc - PCIE Block Revision register 2 */

/* PCI/PCI Express outbound window 0-4
 * Window 0 is the default window and is the only window enabled upon reset.
 * The default outbound register set is used when a transaction misses
 * in all of the other outbound windows.
 */
	struct pci_outbound_window_regs pow[5];
	u8	res14[96];
	struct pci_inbound_window_regs	pmit;	/* 0xd00 - 0xd9c Inbound MSI */
	u8	res6[96];
/* PCI/PCI Express inbound window 3-0
 * inbound window 1 supports only a 32-bit base address and does not
 * define an inbound window base extended address register.
 */
	struct pci_inbound_window_regs piw[4];
/* Merge PCI/PCI Express error management registers */
	__be32	pex_err_dr;	  /* 0x.e00
				   * - PCI/PCIE error detect register
				   */
	__be32	pex_err_cap_dr;	  /* 0x.e04
				   * - PCI error capture disabled register
				   * - PCIE has no this register
				   */
	__be32	pex_err_en;	  /* 0x.e08
				   * - PCI/PCIE error interrupt enable register
				   */
	__be32	pex_err_attrib;	  /* 0x.e0c
				   * - PCI error attributes capture register
				   * - PCIE has no this register
				   */
	__be32	pex_err_disr;	  /* 0x.e10
				   * - PCI error address capture register
				   * - PCIE error disable register
				   */
	__be32	pex_err_ext_addr; /* 0x.e14
				   * - PCI error extended addr capture register
				   * - PCIE has no this register
				   */
	__be32	pex_err_dl;	  /* 0x.e18
				   * - PCI error data low capture register
				   * - PCIE has no this register
				   */
	__be32	pex_err_dh;	  /* 0x.e1c
				   * - PCI error data high capture register
				   * - PCIE has no this register
				   */
	__be32	pex_err_cap_stat; /* 0x.e20
				   * - PCI gasket timer register
				   * - PCIE error capture status register
				   */
	u8	res24[4];
	__be32	pex_err_cap_r0;		/* 0x.e28 - PCIE error capture register 0 */
	__be32	pex_err_cap_r1;		/* 0x.e2c - PCIE error capture register 0 */
	__be32	pex_err_cap_r2;		/* 0x.e30 - PCIE error capture register 0 */
	__be32	pex_err_cap_r3;		/* 0x.e34 - PCIE error capture register 0 */
};

extern int primary_phb_addr;
extern int fsl_add_bridge(struct device_node *dev, int is_primary);
extern int fsl_pci_setup(struct device_node *np);
extern void fsl_pcibios_fixup_bus(struct pci_bus *bus);
extern int mpc83xx_add_bridge(struct device_node *dev);
u64 fsl_pci_immrbar_base(struct pci_controller *hose);

#ifdef CONFIG_FSL_PCI
extern int fsl_pci_mcheck_exception(struct pt_regs *);
#else
static inline int fsl_pci_mcheck_exception(struct pt_regs *regs) {return 0; }
#endif

#ifdef CONFIG_EDAC_MPC85XX
extern int mpc85xx_pci_err_probe(struct platform_device *op);
#endif

#endif /* __POWERPC_FSL_PCI_H */
#endif /* __KERNEL__ */
