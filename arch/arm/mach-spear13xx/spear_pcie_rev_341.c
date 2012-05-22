/*
 * arch/arm/mach-spear13xx/spear_pcie_rev_341.c
 *
 * Supports SPEAr1300, SPEAr1310 RevA, SPEAr900
 *
 * Copyright (C) 2010-2011 ST Microelectronics
 * Pratyush Anand <pratyush.anand@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <asm/mach/irq.h>
#include <mach/misc_regs.h>
#include <mach/spear_pcie_rev_341.h>

static void enable_dbi_access(struct pcie_app_reg __iomem *app_reg)
{
	writel(readl(&app_reg->slv_armisc) | (1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_awmisc) | (1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_awmisc);
}

static void disable_dbi_access(struct pcie_app_reg __iomem *app_reg)
{
	writel(readl(&app_reg->slv_armisc) & ~(1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_armisc);
	writel(readl(&app_reg->slv_awmisc) & ~(1 << AXI_OP_DBI_ACCESS_ID),
			&app_reg->slv_awmisc);
}

static void pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
		u32 *val)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;
	u32 va_address;

	enable_dbi_access(app_reg);

	va_address = (u32)pp->va_base + (where & ~0x3);

	*val = readl(va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	disable_dbi_access(app_reg);
}

static void pcie_wr_own_conf(struct pcie_port *pp, int where, int size, u32 val)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;
	u32 va_address;

	enable_dbi_access(app_reg);

	va_address = (u32)pp->va_base + (where & ~0x3);

	if (size == 4)
		writel(val, va_address);
	else if (size == 2)
		writew(val, va_address + (where & 2));
	else if (size == 1)
		writeb(val, va_address + (where & 3));

	disable_dbi_access(app_reg);
}

static int pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;
	u32 address;
	u32 armisc;

	armisc = readl(&app_reg->slv_armisc);
	armisc &= ~(AXI_OP_TYPE_MASK);
	if (bus->parent->number == pp->root_bus_nr) {
		address = (u32)pp->va_cfg0_base | (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);
		writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
		armisc |= AXI_OP_TYPE_CONFIG_RDRW_TYPE0;
	} else {
		address = (u32)pp->va_cfg1_base | (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);
		writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg1_addr_start);
		armisc |= AXI_OP_TYPE_CONFIG_RDRW_TYPE1;
	}

	writel(armisc, &app_reg->slv_armisc);
	while (armisc != readl(&app_reg->slv_armisc))
		;

	*val = readl(address);
	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	armisc &= ~(AXI_OP_TYPE_MASK);
	writel(armisc, &app_reg->slv_armisc);

	return PCIBIOS_SUCCESSFUL;
}

static int pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;
	u32 address;
	u32 awmisc;

	awmisc = readl(&app_reg->slv_awmisc);
	awmisc &= ~(AXI_OP_TYPE_MASK);

	if (bus->parent->number == pp->root_bus_nr) {
		address = (u32)pp->va_cfg0_base | (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);
		writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg0_addr_start);
		awmisc |= AXI_OP_TYPE_CONFIG_RDRW_TYPE0;
	} else {
		address = (u32)pp->va_cfg1_base | (PCI_FUNC(devfn) << 16)
			| (where & 0xFFFC);
		writel((bus->number << 24) | (PCI_SLOT(devfn) << 19),
			&app_reg->pom_cfg1_addr_start);
		awmisc |= AXI_OP_TYPE_CONFIG_RDRW_TYPE1;
	}

	writel(awmisc, &app_reg->slv_awmisc);
	while (awmisc != readl(&app_reg->slv_awmisc))
		;

	if (size == 4)
		writel(val, address);
	else if (size == 2)
		writew(val, address + (where & 2));
	else if (size == 1)
		writeb(val, address + (where & 3));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	awmisc &= ~(AXI_OP_TYPE_MASK);
	writel(awmisc, &app_reg->slv_awmisc);
	return ret;
}

#ifdef CONFIG_PCI_MSI
extern unsigned int msi_data[];

static void msi_init(struct pcie_port *pp)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;

	pp->ops.wr_own(pp, PCIE_MSI_ADDR_LO, 4,
			__virt_to_phys((u32)(&msi_data[pp->port])));
	pp->ops.wr_own(pp, PCIE_MSI_ADDR_HI, 4, 0);
	/* Enbale MSI interrupt*/
	writel(readl(&app_reg->int_mask) | MSI_CTRL_INT,
			&app_reg->int_mask);
}
#endif

static void mask_intx_irq(struct irq_data *data)
{
	u32 irq = data->irq;
	int irq_offset = (irq - INTX0_BASE) % NUM_INTX_IRQS;
	int port = (irq - INTX0_BASE) / NUM_INTX_IRQS;
	struct pcie_port *pp = portno_to_port(port);
	struct pcie_app_reg __iomem *app_reg;
	u32 mask;

	if (!pp) {
		BUG();
		return;
	}

	app_reg = pp->va_app_base;

	switch (irq_offset) {
	case 0:
		mask = ~INTA_CTRL_INT;
		break;
	case 1:
		mask = ~INTB_CTRL_INT;
		break;
	case 2:
		mask = ~INTC_CTRL_INT;
		break;
	case 3:
		mask = ~INTD_CTRL_INT;
		break;
	}
	writel(readl(&app_reg->int_mask) & mask, &app_reg->int_mask);
}

static void unmask_intx_irq(struct irq_data *data)
{
	u32 irq = data->irq;
	int irq_offset = (irq - INTX0_BASE) % NUM_INTX_IRQS;
	int port = (irq - INTX0_BASE) / NUM_INTX_IRQS;
	struct pcie_port *pp = portno_to_port(port);
	struct pcie_app_reg __iomem *app_reg;
	u32 mask;

	if (!pp) {
		BUG();
		return;
	}

	app_reg = pp->va_app_base;

	switch (irq_offset) {
	case 0:
		mask = INTA_CTRL_INT;
		break;
	case 1:
		mask = INTB_CTRL_INT;
		break;
	case 2:
		mask = INTC_CTRL_INT;
		break;
	case 3:
		mask = INTD_CTRL_INT;
		break;
	}
	writel(readl(&app_reg->int_mask) | mask, &app_reg->int_mask);
}

static struct irq_chip intx_chip = {
	.name = "PCI-INTX",
	.irq_mask = mask_intx_irq,
	.irq_unmask = unmask_intx_irq,
};

static void pcie_int_handler(unsigned int irq, struct irq_desc *desc)
{
	struct pcie_port *pp  = portno_to_port(irq - IRQ_PCIE0);
	struct pcie_app_reg __iomem *app_reg;
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned int status;

	if (!pp) {
		BUG();
		return;
	}

	app_reg = pp->va_app_base;
	status = readl(&app_reg->int_sts);

	chained_irq_enter(irqchip, desc);

	if (status & MSI_CTRL_INT) {
#ifdef CONFIG_PCI_MSI
		handle_msi(pp);
#endif
		writel(MSI_CTRL_INT, &app_reg->int_clr);
	} else if (status & INTA_CTRL_INT)
		generic_handle_irq(INTX0_BASE
				+ pp->port * NUM_INTX_IRQS);
	else if (status & INTB_CTRL_INT)
		generic_handle_irq(INTX0_BASE
				+ pp->port * NUM_INTX_IRQS + 1);
	else if (status & INTC_CTRL_INT)
		generic_handle_irq(INTX0_BASE
				+ pp->port * NUM_INTX_IRQS + 2);
	else if (status & INTD_CTRL_INT)
		generic_handle_irq(INTX0_BASE
				+ pp->port * NUM_INTX_IRQS + 3);
	else
		writel(status, &app_reg->int_clr);

	chained_irq_exit(irqchip, desc);
}

static void pcie_int_init(struct pcie_port *pp)
{
	int i, irq;
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;

	irq_set_chained_handler(IRQ_PCIE0 + pp->port,
			pcie_int_handler);

#ifdef CONFIG_PCI_MSI
	msi_init(pp);
#endif
	/*
	 * initilize INTX chip here only. MSI chip will be
	 * initilized dynamically.
	 */
	irq = (INTX0_BASE + pp->port * NUM_INTX_IRQS);
	for (i = 0; i < NUM_INTX_IRQS; i++) {
		irq_set_chip_and_handler(irq + i, &intx_chip,
				handle_simple_irq);
		set_irq_flags(irq + i, IRQF_VALID);
	}

	/* Enbale INTX interrupt*/
	writel(readl(&app_reg->int_mask) | INTA_CTRL_INT
			| INTB_CTRL_INT	| INTC_CTRL_INT
			| INTD_CTRL_INT, &app_reg->int_mask);
}

static void pcie_host_init(struct pcie_port *pp)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;
	struct pcie_port_info *config = &pp->config;
	u32 cap, val;

	/*setup registers for outbound translation */

	writel(pp->base, &app_reg->in_io_addr_start);
	pp->io_base = pp->base;
	writel(readl(&app_reg->in_io_addr_start) + (config->io_size - 1),
			&app_reg->in_io_addr_limit);
	writel(readl(&app_reg->in_io_addr_limit) + 1,
			&app_reg->in0_mem_addr_start);
	pp->mem_base = pp->io_base + config->io_size;
	writel(readl(&app_reg->in0_mem_addr_start) + (config->mem_size - 1),
			&app_reg->in0_mem_addr_limit);
	writel(readl(&app_reg->in0_mem_addr_limit) + 1,
			&app_reg->in_cfg0_addr_start);
	pp->cfg0_base = pp->mem_base + config->mem_size;
	writel(readl(&app_reg->in_cfg0_addr_start) + (config->cfg0_size - 1),
			&app_reg->in_cfg0_addr_limit);
	writel(readl(&app_reg->in_cfg0_addr_limit) + 1,
			&app_reg->in_cfg1_addr_start);
	pp->cfg1_base = pp->cfg0_base + config->cfg0_size;
	writel(readl(&app_reg->in_cfg1_addr_start) + (config->cfg1_size - 1),
			&app_reg->in_cfg1_addr_limit);
	writel(readl(&app_reg->in_cfg1_addr_limit) + 1,
			&app_reg->in_msg_addr_start);
	writel(readl(&app_reg->in_msg_addr_start) + (config->msg_size - 1),
			&app_reg->in_msg_addr_limit);

	writel(readl(&app_reg->in0_mem_addr_start),
			&app_reg->pom0_mem_addr_start);
	writel(readl(&app_reg->in1_mem_addr_start),
			&app_reg->pom1_mem_addr_start);
	writel(readl(&app_reg->in_io_addr_start),
			&app_reg->pom_io_addr_start);

	/*setup registers for inbound translation */

	writel((u32)config->in_mem_size, &app_reg->mem0_addr_offset_limit);
	writel(0, &app_reg->pim0_mem_addr_start);
	writel(0, &app_reg->pim1_mem_addr_start);
	pcie_wr_own_conf(pp, PCIE_BAR0_MASK_REG, 4,
			(config->in_mem_size - 1));
	pcie_wr_own_conf(pp, PCI_BASE_ADDRESS_0, 4, 0);

	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_io_addr_start);
	writel(0x0, &app_reg->pim_rom_addr_start);

	cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);

	/*
	 * this controller support only 128 bytes read size, however its
	 * default value in capability register is 512 bytes. So force
	 * it to 128 here.
	 */

	pcie_rd_own_conf(pp, cap + PCI_EXP_DEVCTL, 4, &val);
	val &= ~PCI_EXP_DEVCTL_READRQ;
	pcie_wr_own_conf(pp, cap + PCI_EXP_DEVCTL, 4, val);

	/*program correct class for RC*/
	pcie_rd_own_conf(pp, PCI_CLASS_REVISION, 4, &val);
	val &= 0xFFFF;
	val |= (PCI_CLASS_BRIDGE_PCI << 16);
	pcie_wr_own_conf(pp, PCI_CLASS_REVISION, 4, val);
	/*program vid and did for RC*/
	pcie_wr_own_conf(pp, PCI_VENDOR_ID, 2, 0x104A);
	pcie_wr_own_conf(pp, PCI_DEVICE_ID, 2, 0xCD80);
	/*if is_gen1 is set then handle it*/
	if (pp->config.is_gen1) {
		cap = pci_find_own_capability(pp, PCI_CAP_ID_EXP);
		pcie_rd_own_conf(pp, cap + PCI_EXP_LNKCAP, 4, &val);
		if ((val & 0xF) != 1) {
			val &= ~((u32)0xF);
			val |= 1;
			pcie_wr_own_conf(pp, cap + PCI_EXP_LNKCAP, 4,
					val);
		}
		pcie_rd_own_conf(pp, cap + PCI_EXP_LNKCTL2, 4, &val);
		if ((val & 0xF) != 1) {
			val &= ~((u32)0xF);
			val |= 1;
			pcie_wr_own_conf(pp, cap + PCI_EXP_LNKCTL2, 4,
					val);
		}
	} else {
		pcie_rd_own_conf(pp, PCIE_PORT_LOGIC, 4, &val);
		val |= (1 << PORT_LOGIC_SPD_CHANGE_ID);
		pcie_wr_own_conf(pp, PCIE_PORT_LOGIC, 4, val);
	}

	writel(DEVICE_TYPE_RC | (1 << MISCTRL_EN_ID)
			| (1 << APP_LTSSM_ENABLE_ID)
			| ((u32)1 << REG_TRANSLATION_ENABLE),
			&app_reg->app_ctrl_0);

	pcie_int_init(pp);
}

static void pcie_host_exit(struct pcie_port *pp)
{
	struct pcie_app_reg __iomem *app_reg = pp->va_app_base;

	/* Remove link up */
	writel(0, &app_reg->app_ctrl_0);
}

static int pcie_link_up(void __iomem *va_app_base)
{
	struct pcie_app_reg __iomem *app_reg = va_app_base;
	int ucount = 0;

	do {
		if (readl(&app_reg->app_status_1) &
				((u32)1 << XMLH_LINK_UP_ID))
			return 1;
		ucount++;
		udelay(1);
	} while (ucount <= MAX_LINK_UP_WAIT_MS * 1000);

	return 0;
}

static int add_pcie_port(struct pcie_port *pp, struct platform_device *pdev)
{
	struct pcie_app_reg __iomem *app_reg;
	struct resource *base;
	struct resource *app_base;
	int err;

	app_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!app_base) {
		dev_err(&pdev->dev, "couldn't get app base resource\n");
		return -EINVAL;
	}
	if (!request_mem_region(app_base->start, resource_size(app_base),
				pdev->name)) {
		dev_err(&pdev->dev, "app base resource is busy\n");
		return -EBUSY;
	}
	pp->app_base = (void __iomem *)app_base->start;

	base = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!base) {
		dev_err(&pdev->dev, "couldn't get base resource\n");
		err = -EINVAL;
		goto free_app_res;
	}
	if (!request_mem_region(base->start, resource_size(base),
				pdev->name)) {
		dev_err(&pdev->dev, "app base resource is busy\n");
		err = -EBUSY;
		goto free_app_res;
	}
	pp->base = (void __iomem *)base->start;

	pp->va_app_base = ioremap(app_base->start, resource_size(app_base));
	if (!pp->va_app_base) {
		dev_err(&pdev->dev, "error with ioremap\n");
		err = -ENOMEM;
		goto free_base_res;
	}

	pp->va_base = ioremap(base->start, resource_size(base));
	if (!pp->va_base) {
		dev_err(&pdev->dev, "error with ioremap\n");
		err = -ENOMEM;
		goto unmap_app_res;
	}

	pp->port = pdev->id;
	pp->root_bus_nr = -1;
	spin_lock_init(&pp->conf_lock);
	if (pcie_link_up(pp->va_app_base)) {
		dev_info(&pdev->dev, "link up\n");
	} else {
		dev_info(&pdev->dev, "link down\n");
		pcie_host_init(pp);
		app_reg = pp->va_app_base;
		pp->va_cfg0_base =
			ioremap(readl(&app_reg->in_cfg0_addr_start),
					pp->config.cfg0_size);
		if (!pp->va_cfg0_base) {
			dev_err(&pdev->dev, "error with ioremap\n");
			err = -ENOMEM;
			goto unmap_base_res;
		}
		pp->va_cfg1_base =
			ioremap(readl(&app_reg->in_cfg1_addr_start),
					pp->config.cfg1_size);
		if (!pp->va_cfg1_base) {
			dev_err(&pdev->dev, "error with ioremap\n");
			err = -ENOMEM;
			goto unmap_cfg0;
		}
		return 0;
	}
unmap_cfg0:
	iounmap(pp->va_cfg0_base);
unmap_base_res:
	iounmap(pp->va_base);
unmap_app_res:
	iounmap(pp->va_app_base);
free_base_res:
	release_mem_region(base->start, resource_size(base));
free_app_res:
	release_mem_region(app_base->start, resource_size(app_base));
	return err;
}

static int pcie_clk_init(struct pcie_port *pp)
{
	/*
	 * Enable all CLK in CFG registers here only. Idealy only PCIE0
	 * should have been enabled. But Controler does not work
	 * properly if PCIE1 and PCIE2's CFG CLK is enabled in stages.
	 */
	writel(PCIE0_CFG_VAL | PCIE1_CFG_VAL | PCIE2_CFG_VAL, VA_PCIE_CFG);

	if (pp->clk == NULL) {
		pp->clk = clk_get_sys("dw_pcie.0", NULL);

		if (IS_ERR(pp->clk)) {
			pr_err("%s:couldn't get clk for pcie0\n", __func__);
			return -ENODEV;
		}
	}

	if (clk_enable(pp->clk)) {
		pr_err("%s:couldn't enable clk for pcie0\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int pcie_clk_exit(struct pcie_port *pp)
{
	writel(0, VA_PCIE_CFG);
	if (pp->clk)
		clk_disable(pp->clk);

	return 0;
}

void spear_pcie_341_add_ops(struct pcie_port *pp)
{
	struct pcie_private_ops	*ops = &pp->ops;

	ops->rd_own = pcie_rd_own_conf;
	ops->wr_own = pcie_wr_own_conf;
	ops->rd_other = pcie_rd_other_conf;
	ops->wr_other = pcie_wr_other_conf;
	ops->add_port = add_pcie_port;
	ops->link_up = pcie_link_up;
	ops->host_init = pcie_host_init;
	ops->host_exit = pcie_host_exit;
	ops->clk_init = pcie_clk_init;
	ops->clk_exit = pcie_clk_exit;
}
