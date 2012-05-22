/*
 * arch/arm/mach-spear13xx/include/mach/dw_pcie.h
 *
 * Copyright (C) 2010-2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __DW_PCIE_H
#define __DW_PCIE_H

#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/clk.h>

#define MAX_LINK_UP_WAIT_MS	2
/* Max port defined can be changed if required */
#define MAX_PCIE_PORT_SUPPORTED	3
struct pcie_port;

struct pcie_port_info {
	u32	vendor;
	u8	is_host;
	u8	is_gen1;
	u32	mem_size;
	u32	io_size;
	u32	cfg0_size;
	u32	cfg1_size;
	u32	msg_size;
	u32	in_mem_size;
};

struct pcie_private_ops {
	void (*rd_own)(struct pcie_port *pp, int where, int size, u32 *val);
	void (*wr_own)(struct pcie_port *pp, int where, int size, u32 val);
	int (*rd_other)(struct pcie_port *pp, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 *val);
	int (*wr_other)(struct pcie_port *pp, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 val);
	int (*add_port)(struct pcie_port *pp, struct platform_device *pdev);
	int (*link_up)(void __iomem *va_app_base);
	void (*host_init)(struct pcie_port *pp);
	void (*host_exit)(struct pcie_port *pp);
	int (*clk_init)(struct pcie_port *pp);
	int (*clk_exit)(struct pcie_port *pp);
};

struct pcie_port {
	u8			port;
	u8			controller;
	u8			root_bus_nr;
	void __iomem		*base;
	void __iomem		*app_base;
	void __iomem		*cfg0_base;
	void __iomem		*cfg1_base;
	void __iomem		*mem_base;
	void __iomem		*io_base;
	void __iomem		*va_base;
	void __iomem		*va_app_base;
	void __iomem		*va_cfg0_base;
	void __iomem		*va_cfg1_base;
	spinlock_t		conf_lock;
	char			mem_space_name[16];
	char			io_space_name[16];
	struct resource		res[2];
	struct pcie_port_info	config;
	struct list_head	next;
	struct pcie_private_ops	ops;
	struct clk *clk;
	int	susp_state;
};

/* synopsis specific PCIE configuration registers*/
#define PCIE_PORT_LOGIC			0x80C
#define PORT_LOGIC_SPD_CHANGE_ID	17

#define PCIE_MSI_ADDR_LO		0x820
#define PCIE_MSI_ADDR_HI		0x824
#define PCIE_MSI_INTR0_ENABLE		0x828
#define PCIE_MSI_INTR0_MASK		0x82C
#define PCIE_MSI_INTR0_STATUS		0x830

#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND		(1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0 << 31)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_TYPE_MEM		0
#define PCIE_ATU_TYPE_IO		2
#define PCIE_ATU_TYPE_CFG0		4
#define PCIE_ATU_TYPE_CFG1		5
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_ENABLE			(1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(1 << 30)
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_UPPER_TARGET		0x91C

/*BAR MASK registers*/
#define PCIE_BAR0_MASK_REG		0x1010

#ifdef CONFIG_ARCH_SPEAR13XX
#define INTX0_BASE			SPEAR_INTX0_BASE
#define NUM_INTX_IRQS			SPEAR_NUM_INTX_IRQS
#define IRQ_PCIE0			SPEAR13XX_IRQ_PCIE0
#define NUM_MSI_IRQS			SPEAR_NUM_MSI_IRQS
#define MSI0_INT_BASE			SPEAR_MSI0_INT_BASE
#define MSI0_INT_END			SPEAR_MSI0_INT_END

int enable_pcie0_clk(void);
void spear_pcie_341_add_ops(struct pcie_port *pp);
void spear_pcie_370_add_ops(struct pcie_port *pp);
#endif

int pci_find_own_capability(struct pcie_port *pp, int cap);
struct pcie_port *portno_to_port(int port);
void handle_msi(struct pcie_port *pp);
#endif
