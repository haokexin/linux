// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Synopsys DesignWare Core
 *
 * Copyright (C) 2015-2016 Synopsys, Inc. (www.synopsys.com)
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * Authors: Joao Pinto <Joao.Pinto@synopsys.com>
 *          Xuran Yang <xuran.yang@bst.ai>
 */
#define pr_fmt(fmt) KBUILD_MODNAME" " fmt
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/spinlock.h>

#include "pcie-bst.h"
#include "pcie-bst-phy.h"
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
#include "pcie-bst-diagnostic.h"
extern int send_dtc_to_safety_svc(u32 dtc);
#endif

#ifdef CONFIG_HAVE_DBI_MUTEX
struct mutex dbi_mutex;
#define BST_PCIE_DBI_MUTEX_LOCK() 			mutex_lock(&dbi_mutex)
#define BST_PCIE_DBI_MUTEX_UNLOCK() 		mutex_unlock(&dbi_mutex)
#else
#define BST_PCIE_DBI_MUTEX_LOCK()
#define BST_PCIE_DBI_MUTEX_UNLOCK()
#endif

#define to_bst_pcie(x)	dev_get_drvdata((x)->dev)

struct bst_ep_doorbell {
	u8			func_no;
	u8			vfunc_no;
	u32			irq_no;
	u32			bar_no;
	u32			offset;
	u32			msg;
	int			(*handler)(int irq, void *arg);
	void			*arg;
	struct list_head	list;
};



struct dw_plat_pcie_of_data {
	enum dw_pcie_device_mode	mode;
};

static unsigned int bst_pcie_ep_func_conf_select(struct dw_pcie_ep *ep,
						 u8 func_no, u8 vfunc_no)
{
	return (vfunc_no ? ((vfunc_no - 1) << 22) | (1 << 21) : 0) | (func_no << 16);
}

static irqreturn_t bst_pcie_ep_db_handler(int irq, void *arg)
{
	struct bst_pcie *bst_pcie = arg;
	struct bst_ep_doorbell *db;
	u32 ctrl_id = bst_pcie->ctrl_id;
	u32 status;
	int func, db_irq;

	for (func = 0; func < 8; func++) {
		status = pcie_phy_read(bst_pcie->phy, ctrl_id ?
				       X2_LBC_EXT_INT_STATUS_FUNC(func) :
				       X4_LBC_EXT_INT_STATUS_FUNC(func));
		for (db_irq = 0; db_irq < 32; db_irq++) {
			if (status & (1 << db_irq)) {
				list_for_each_entry(db, &bst_pcie->db_irq_list, list) {
					if (db->func_no == func &&
					    db->irq_no == db_irq &&
					    db->handler) {
						db->handler(db_irq, db->arg);
					}
				}
			}
		}
		pcie_phy_write(bst_pcie->phy, ctrl_id ? X2_LBC_EXT_INT_CLR_FUNC(func) :
							X4_LBC_EXT_INT_CLR_FUNC(func),
			       status);
	}
	return IRQ_HANDLED;
}

static void dw_plat_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct device *dev = pci->dev;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct platform_device *pdev = to_platform_device(dev);
	enum pci_barno bar;
	u8 func_no, funcs, reg;
	unsigned int func_offset = 0;
	int ret;

	funcs = ep->epc->max_functions;

	dw_pcie_dbi_ro_wr_en(pci);
	/* Indicates this is a multi-function device */
	reg = dw_pcie_readb_dbi(pci, PCI_HEADER_TYPE);
	dw_pcie_writeb_dbi(pci, PCI_HEADER_TYPE, reg | (1 << 7));
	for (func_no = 0; func_no < funcs; func_no++) {
		func_offset = bst_pcie_ep_func_conf_select(ep, func_no, 0);
		dw_pcie_writew_dbi(pci, func_offset + PCI_VENDOR_ID, 0xFFFF);
	}
	dw_pcie_dbi_ro_wr_dis(pci);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);

	bst_pcie->db_irq = platform_get_irq_byname_optional(pdev, "doorbell");
	if (bst_pcie->db_irq >= 0) {
		dev_info(dev, "EP doorbell IRQ %d\n", bst_pcie->db_irq);
		/* Mask PTM master stobe int */
		if (bst_pcie->ctrl_id == 0) {
			pcie_phy_cfg(bst_pcie->phy, PCIE_X4_PTM_CTRL2, 1, BIT(6));
			pcie_phy_cfg(bst_pcie->phy, PCIE_X4_PTM_CTRL1, 1, BIT(9));
		} else if (bst_pcie->ctrl_id == 1) {
			pcie_phy_cfg(bst_pcie->phy, PCIE_X2_PTM_CTRL2, 1, BIT(6));
			pcie_phy_cfg(bst_pcie->phy, PCIE_X2_PTM_CTRL1, 1, BIT(9));
		}
		ret = devm_request_irq(dev, bst_pcie->db_irq, bst_pcie_ep_db_handler,
				0, "pcie_ep_db", bst_pcie);
		if (ret) {
			dev_err(dev, "Failed to request EP doorbell IRQ %d\n",
				bst_pcie->db_irq);
			bst_pcie->db_irq = -1;
		}
		bst_pcie->db_irq_win = devm_kzalloc(dev, funcs * sizeof(*bst_pcie->db_irq_win), GFP_KERNEL);
		bst_pcie->db_irq_num = 32;
		INIT_LIST_HEAD(&bst_pcie->db_irq_list);
	}
}

int bst_pcie_ep_db_irq_alloc(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct bst_ep_doorbell *db;
	int free_irq;

	if (bst_pcie->db_irq < 0)
		return -EINVAL;

	free_irq = (int)find_first_zero_bit(&bst_pcie->db_irq_win[func_no], bst_pcie->db_irq_num);
	if (free_irq < 0) {
		// spin_unlock(&bst_pcie->db_lock);
		return -EINVAL;
	}

	set_bit(free_irq, &bst_pcie->db_irq_win[func_no]);
	// spin_unlock(&bst_pcie->db_lock);

	db = kzalloc(sizeof(*db), GFP_KERNEL);
	if (!db)
		return -ENOMEM;
	db->func_no = func_no;
	db->vfunc_no = vfunc_no;
	db->irq_no = free_irq;
	db->bar_no = BST_TRGT0_BAR;
	db->offset = BST_TRGT0_DOORBELL_BASE + BST_TRGT0_DOORBELL_OFF(func_no);
	db->msg = (1 << db->irq_no);
	list_add(&db->list, &bst_pcie->db_irq_list);

	return free_irq;
}
EXPORT_SYMBOL(bst_pcie_ep_db_irq_alloc);

int bst_pcie_ep_db_irq_request(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			       int (*handler)(int irq, void *arg), void *arg)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct bst_ep_doorbell *db;

	list_for_each_entry(db, &bst_pcie->db_irq_list, list) {
		if (db->func_no == func_no &&
		    db->vfunc_no == vfunc_no && db->irq_no == irq_no) {
			db->handler = handler;
			db->arg = arg;
			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(bst_pcie_ep_db_irq_request);

void bst_pcie_ep_db_irq_free(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct bst_ep_doorbell *db, *n;

	list_for_each_entry_safe(db, n, &bst_pcie->db_irq_list, list) {
		if (db->func_no == func_no &&
		    db->vfunc_no == vfunc_no && db->irq_no == irq_no) {
			list_del(&db->list);
			clear_bit(db->irq_no, &bst_pcie->db_irq_win[func_no]);
			kfree(db);
			return;
		}
	}
}
EXPORT_SYMBOL(bst_pcie_ep_db_irq_free);

int bst_pcie_ep_db_info_get(struct pci_epc *epc, u8 func_no, u8 vfunc_no, u32 irq_no,
			    u32 *bar_no, u32 *offset, u32 *msg)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct bst_ep_doorbell *db;

	list_for_each_entry(db, &bst_pcie->db_irq_list, list) {
		if (db->func_no == func_no &&
		    db->vfunc_no == vfunc_no && db->irq_no == irq_no) {
			*bar_no = db->bar_no;
			*offset = db->offset;
			*msg = db->msg;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(bst_pcie_ep_db_info_get);

static int bst_plat_pcie_ep_raise_msi_irq(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
					  u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;
	int ctrl_id = bst_pcie->ctrl_id;
	u32 reg;
	u64 pending;

	reg = VEN_MSI_VECTOR(interrupt_num) | VEN_MSI_TC(0) |
	      VEN_MSI_VF(vfunc_no) | VEN_MSI_PF(func_no);
	pcie_phy_write(phy, ctrl_id ? X2_VEN_MSI_INTX_CTRL : X4_VEN_MSI_INTX_CTRL, reg);
	if (vfunc_no) {
		pending = (func_no * 4 + (vfunc_no - 1)) * 32 + interrupt_num - 1;
		pcie_phy_write(phy, ctrl_id ? X2_VEN_VF_MSI_PENDING_0 : X4_VEN_VF_MSI_PENDING_0,
				lower_32_bits(pending));
		pcie_phy_write(phy, ctrl_id ? X2_VEN_VF_MSI_PENDING_1 : X4_VEN_VF_MSI_PENDING_1,
				upper_32_bits(pending));
	} else {
		pending = func_no * 32 + interrupt_num - 1;
		pcie_phy_write(phy, ctrl_id ? X2_VEN_MSI_PENDING_0 : X4_VEN_MSI_PENDING_0,
				lower_32_bits(pending));
		pcie_phy_write(phy, ctrl_id ? X2_VEN_MSI_PENDING_1 : X4_VEN_MSI_PENDING_1,
				upper_32_bits(pending));
	}
	pcie_phy_cfg(phy, ctrl_id ? X2_MISC_FUNC_CTRL1 : X4_MISC_FUNC_CTRL1, 1, BIT(2));

	return 0;
}

static int dw_plat_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		#ifdef CONFIG_ARCH_BSTC1200
		return bst_plat_pcie_ep_raise_msi_irq(ep, func_no, vfunc_no, interrupt_num);
		#endif
		return dw_pcie_ep_raise_msi_irq(ep, func_no, vfunc_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq_doorbell(ep, func_no, vfunc_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features bst_pcie_epc_features[] = {
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_2) | BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_2) | BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_2) | BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_2) | BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.align = SZ_1M,
	},
};

static const struct pci_epc_features bst_pcie_epc_vf_features[] = {
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.bar_fixed_64bit = BIT(BAR_2) | BIT(BAR_4),
		.align = SZ_1M,
	},
	{
		.linkup_notifier = false,
		.core_init_notifier = false,
		.msi_capable = true,
		.msix_capable = true,
		.align = SZ_1M,
	},
};

static const struct pci_epc_features*
bst_pcie_get_features(struct dw_pcie_ep *ep, u8 func_no, u8 vfunc_no)
{
	if (vfunc_no)
		return &bst_pcie_epc_vf_features[func_no];
	return &bst_pcie_epc_features[func_no];
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = dw_plat_pcie_ep_init,
	.raise_irq = dw_plat_pcie_ep_raise_irq,
	.get_features = bst_pcie_get_features,
	.func_conf_select = bst_pcie_ep_func_conf_select,
};

static void bst_pcie_legacy_irq_mask(struct irq_data *d)
{
	struct bst_pcie *bst_pcie = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&bst_pcie->legacy_irq_lock, flags);
	pcie_phy_cfg(bst_pcie->phy, X4_INT_PULSE_MASK_0, 1, BIT(d->hwirq));
	raw_spin_unlock_irqrestore(&bst_pcie->legacy_irq_lock, flags);
}

static void bst_pcie_legacy_irq_unmask(struct irq_data *d)
{
	struct bst_pcie *bst_pcie = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&bst_pcie->legacy_irq_lock, flags);
	pcie_phy_cfg(bst_pcie->phy, X4_INT_PULSE_MASK_0, 0, BIT(d->hwirq));
	raw_spin_unlock_irqrestore(&bst_pcie->legacy_irq_lock, flags);
}

static struct irq_chip bst_pcie_legacy_irq_chip = {
	.name		= "BST INTx",
	.irq_enable	= bst_pcie_legacy_irq_unmask,
	.irq_disable	= bst_pcie_legacy_irq_mask,
	.irq_mask	= bst_pcie_legacy_irq_mask,
	.irq_unmask	= bst_pcie_legacy_irq_unmask,
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_MASK_ON_SUSPEND,
};

static int bst_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			    irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &bst_pcie_legacy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = bst_pcie_intx_map,
};

static void bst_pcie_legacy_int_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct bst_pcie *bst_pcie = irq_desc_get_handler_data(desc);
	struct device *dev = bst_pcie->pci->dev;
	u32 reg;
	u32 hwirq;
	u32 virq;

	chained_irq_enter(chip, desc);

	switch ((bst_pcie->chip_type << 8) | bst_pcie->ctrl_id) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 X4 */
		reg = pcie_phy_read(bst_pcie->phy, X4_INT_PULSE_STATUS_0);
		pcie_phy_write(bst_pcie->phy, X4_MISC_INT_CLR0, 0);
		pcie_phy_write(bst_pcie->phy, X4_MISC_INT_CLR0, reg);
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 X2 */
		reg = pcie_phy_read(bst_pcie->phy, X2_INT_PULSE_STATUS_0);
		pcie_phy_write(bst_pcie->phy, X2_MISC_INT_CLR0, 0);
		pcie_phy_write(bst_pcie->phy, X2_MISC_INT_CLR0, reg);
		break;
	default:
		reg = 0;
		pr_err("Unknow PCIe phy mode, unable to clear the misc int\n");
		break;
	}

	while (reg) {
		hwirq = ffs(reg) - 1;
		reg &= ~BIT(hwirq);

		/* ignore intx deassert irq */
		if (bst_pcie->chip_type == PCIE_C1200_SERIES
		    && hwirq >= 4 && hwirq <= 7)
			continue;

		virq = irq_find_mapping(bst_pcie->legacy_irq_domain, hwirq);
		if (virq)
			generic_handle_irq(virq);
		else
			dev_err(dev, "unexpected Other IRQ:%d\n", hwirq);
	}

	chained_irq_exit(chip, desc);
}

static int bst_pcie_init_irq_domain(struct bst_pcie *bst_pcie)
{
	struct device *dev = bst_pcie->pci->dev;
	struct device_node *intc = of_get_next_child(dev->of_node, NULL);

	if (!intc) {
		dev_err(dev, "missing child interrupt-controller node\n");
		return -EINVAL;
	}

	raw_spin_lock_init(&bst_pcie->legacy_irq_lock);
	bst_pcie->legacy_irq_domain = irq_domain_add_linear(intc, MAX_PCIE_OTHER_INT_NUM,
						     &intx_domain_ops, bst_pcie);
	if (!bst_pcie->legacy_irq_domain) {
		dev_err(dev, "failed to get a INTx IRQ domain\n");
		return -EINVAL;
	}

	return 0;
}

int bst_rc_id_setup(struct dw_pcie *pci)
{
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi(pci, 0x0, (PCI_DEVICE_ID_BST_RC << 16) | PCI_VENDOR_ID_BST);
	dw_pcie_dbi_ro_wr_dis(pci);
	return 0;
}

int bst_set_perf_mps(struct dw_pcie *pci, int mps)
{
	u32 reg;
	u8 cap;

	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	if (cap) {
		cap += PCI_EXP_DEVCTL;
		dw_pcie_dbi_ro_wr_en(pci);
		reg = dw_pcie_readl_dbi(pci, cap);
		reg &= ~PCI_EXP_DEVCTL_PAYLOAD;
		reg |= (ffs(mps) - 8) << 5;
		dw_pcie_writel_dbi(pci, cap, reg);
		dw_pcie_dbi_ro_wr_dis(pci);
		return 0;
	}
	return -1;
}

static int bst_pcie_host_init(struct dw_pcie_rp *pp)
{
	int irq, ret;
	u32 reg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct device *dev = pci->dev;

	/* Set BST VendorID */
	bst_rc_id_setup(pci);

	/* Set MPS */
	bst_set_perf_mps(pci, 256);

	/* Allowed reassign bus */
	pci_add_flags(PCI_REASSIGN_ALL_BUS);

	/* Legacy interrupt */
	ret = bst_pcie_init_irq_domain(bst_pcie);
	if (!ret) {
		irq = of_irq_get_byname(dev->of_node, "other");
		if (irq >= 0) {
			bst_pcie->legacy_parent_irq = irq;
			irq_set_chained_handler_and_data(irq, bst_pcie_legacy_int_handler,
							 bst_pcie);
			/* Only unmask legacy interrupt*/
			switch ((bst_pcie->chip_type << 8) | bst_pcie->ctrl_id) {
			case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 X4 */
				reg = pcie_phy_read(bst_pcie->phy, X4_INT_PULSE_MASK_0);
				reg = ~0;
				/* Unmask Asserted Int */
				reg &= ~((1 << 0xC) | (1 << 0xD) | (1 << 0xE) | (1 << 0xF));
				pcie_phy_write(bst_pcie->phy, X4_INT_PULSE_MASK_0, reg);
				pcie_phy_write(bst_pcie->phy, X4_INT_LEVEL_MASK_1, 0x800000);
				break;
			case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 X2 */
				reg = pcie_phy_read(bst_pcie->phy, X2_INT_PULSE_MASK_0);
				reg = ~0;
				/* Unmask Asserted Int */
				reg &= ~((1 << 0xC) | (1 << 0xD) | (1 << 0xE) | (1 << 0xF));
				pcie_phy_write(bst_pcie->phy, X2_INT_PULSE_MASK_0, reg);
				pcie_phy_write(bst_pcie->phy, X2_INT_LEVEL_MASK_1, 0x800000);
				break;
			default:
				break;
			}
		} else
			dev_info(dev, "missing legacy IRQ resource\n");
	}

	return 0;
}

static const struct dw_pcie_host_ops bst_plat_pcie_host_ops = {
	.host_init = bst_pcie_host_init,
};

static int dw_plat_add_pcie_port(struct bst_pcie *bst_pcie,
				 struct platform_device *pdev)
{
	struct dw_pcie *pci = bst_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq = platform_get_irq(pdev, 1);
	if (pp->irq < 0)
		return pp->irq;

	pp->num_vectors = MAX_MSI_IRQS;
	pp->ops = &bst_plat_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int bst_pcie_link_up(struct dw_pcie *pci)
{
	u32 temp = 0, dev_info;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;
	int ret = 0;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL1, 1, BIT(1));
		temp = pcie_phy_read(phy, X4_MISC_COM_STATUS);
		ret = temp & BIT(6) ? (temp & BIT(7) ? 1 : 0) : 0;
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL1, 1, BIT(1));
		temp = pcie_phy_read(phy, X2_MISC_COM_STATUS);
		ret = temp & BIT(6) ? (temp & BIT(7) ? 1 : 0) : 0;
		break;
	default:
		pr_err("status: ctrl_id error\n");
		return 0;
	}

	phy->ltssm = temp;

	return ret;
}

int bst_pcie_start_link(struct dw_pcie *pci)
{
	u32 dev_info;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;

	if (gpio_is_valid(bst_pcie->reset_gpio))
		gpio_set_value_cansleep(bst_pcie->reset_gpio,
					bst_pcie->gpio_active_high);

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x1, BIT(1));
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x1, BIT(2));
		break;
	default:
		pr_err("status: ctrl_id error\n");
		return -1;
	}

	return 0;
}

void bst_pcie_stop_link(struct dw_pcie *pci)
{
	u32 dev_info;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x0, BIT(1));
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x0, BIT(2));
		break;
	default:
		pr_err("status: ctrl_id error\n");
		break;
	}
}

void c1200_axi2cfg(struct dw_pcie *pci)
{
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info = 0;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):
		spin_lock_irq(&bst_pcie->share_dbi_lock);
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL1, 1, BIT(21));
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL4, 1, BIT(21));
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1):
		spin_lock_irq(&bst_pcie->share_dbi_lock);
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL1, 1, BIT(21));
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL4, 1, BIT(21));
		break;
	default:
		break;
	}
}

void c1200_axi2mem(struct dw_pcie *pci)
{
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info = 0;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL1, 0, BIT(21));
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL4, 0, BIT(21));
		spin_unlock_irq(&bst_pcie->share_dbi_lock);
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1):
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL1, 0, BIT(21));
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL4, 0, BIT(21));
		spin_unlock_irq(&bst_pcie->share_dbi_lock);
		break;
	default:
		break;
	}
}

u32 bst_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size)
{
	int ret;
	u32 val;
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
#endif

	BST_PCIE_DBI_MUTEX_LOCK();
	c1200_axi2cfg(pci);
	ret = dw_pcie_read(base + reg, (int)size, &val);
	if (ret)
	{
		dev_err(pci->dev, "Read DBI address failed\n");
	#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
		if(bst_pcie->pcie_diagnostic_init_done)
		{
			if(bst_pcie->bst_pcie_diag->dbi_access_monitor_psm)
				bst_pcie->bst_pcie_diag->dbi_access_failed_count++;
		}
	#endif
	}

	c1200_axi2mem(pci);
	BST_PCIE_DBI_MUTEX_UNLOCK();

	return val;
}
EXPORT_SYMBOL(bst_pcie_read_dbi);

void bst_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size, u32 val)
{
	int ret;
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	u32 rd_val;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
#endif
	BST_PCIE_DBI_MUTEX_LOCK();
	c1200_axi2cfg(pci);
	ret = dw_pcie_write(base + reg, (int)size, val);
	if (ret)
	{
		dev_err(pci->dev, "Write DBI address failed\n");
	#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
		if(bst_pcie->pcie_diagnostic_init_done)
		{
			if(bst_pcie->bst_pcie_diag->dbi_access_monitor_psm)
				bst_pcie->bst_pcie_diag->dbi_access_failed_count++;
		}
	#endif
	}

#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->pcie_diagnostic_init_done)
	{
		if(bst_pcie->bst_pcie_diag->reg_rdback_monitor_psm 
			&& bst_pcie->bst_pcie_diag->reg_rdback_monitor_enable 
			&& reg < PCIE_CONFIGURE_REG_SPACE_SIZE) /* Limit the register address range extend configure space, Some configuration spaces are unreadable */
		{
			ret = dw_pcie_read(base + reg, (int)size, &rd_val);
			if(rd_val != val)
			{
				pr_err("[%s] register readback not match reg:[%x] wt_val:[%x] rd_val:[%x]\n", __FUNCTION__, reg, val, rd_val);
				send_dtc_to_safety_svc(PSM_ID_CONFIGURE_REG_READBACK_DTC);
			}
		}
	}
#endif

	c1200_axi2mem(pci);
	BST_PCIE_DBI_MUTEX_UNLOCK();

}
EXPORT_SYMBOL(bst_pcie_write_dbi);

void bst_pcie_write_dbi2(struct dw_pcie *pci, void __iomem *base, u32 reg, size_t size, u32 val)
{
	int ret;
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	u32 rd_val;
	struct bst_pcie *bst_pcie = to_bst_pcie(pci);
#endif
	BST_PCIE_DBI_MUTEX_LOCK();
	c1200_axi2cfg(pci);
	ret = dw_pcie_write(base + reg, (int)size, val);
	if (ret)
	{
		dev_err(pci->dev, "write DBI address failed\n");
	#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
		if(bst_pcie->pcie_diagnostic_init_done)
		{
			if(bst_pcie->bst_pcie_diag->dbi_access_monitor_psm)
				bst_pcie->bst_pcie_diag->dbi_access_failed_count++;
		}
	#endif
	}
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->pcie_diagnostic_init_done)
	{
		if(bst_pcie->bst_pcie_diag->reg_rdback_monitor_psm 
			&& bst_pcie->bst_pcie_diag->reg_rdback_monitor_enable)
		{
			ret = dw_pcie_read(base + reg, (int)size, &rd_val);
			if(rd_val != val)
			{
				pr_err("2 [%s] register readback not match reg:[%x] wt_val:[%x] rd_val:[%x]\n", __FUNCTION__, reg, val, rd_val);
				send_dtc_to_safety_svc(PSM_ID_CONFIGURE_REG_READBACK_DTC);
			}
		}
	}
#endif
	c1200_axi2mem(pci);
	BST_PCIE_DBI_MUTEX_UNLOCK();
}
EXPORT_SYMBOL(bst_pcie_write_dbi2);

static ssize_t ltssm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;
	int len;
	u32 tmp;
	int i, offset = 0;

	if (bst_pcie->ctrl_id == 0) {
		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 0, BIT(4));
		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 1, BIT(4));
		for (i = 0; i < 63; i++) {
			tmp = pcie_phy_read(phy, 0x1f04);
			tmp = pcie_phy_read(phy, X4_LTSSM_RD_DATA);

			if (!tmp)
				continue;
			offset += sprintf(buf + offset, "tr_cnt:%02X ", tmp >> 16);
			offset += sprintf(buf + offset, "mult_cnt:%02X ", (tmp >> 8) & 0xFF);
			offset += sprintf(buf + offset, "phy_rate:%X ", (tmp >> 6) & 0x3);
			offset += sprintf(buf + offset, "ltssm_state:%X\n", tmp & 0x3f);
		}
		pcie_phy_cfg(phy, X4_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(1));
		udelay(1000);
		pcie_phy_cfg(phy, X4_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(2));
		udelay(1000);
		pcie_phy_cfg(phy, X4_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(0));
		udelay(1000);
	} else if (bst_pcie->ctrl_id == 1) {
		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 1, BIT(4));
		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 0, BIT(4));
		for (i = 0; i < 63; i++) {
			tmp = pcie_phy_read(phy, 0x1f04);
			tmp = pcie_phy_read(phy, X2_LTSSM_RD_DATA);

			if (!tmp)
				continue;
			offset += sprintf(buf + offset, "tr_cnt:%02X ", tmp >> 16);
			offset += sprintf(buf + offset, "mult_cnt:%02X ", (tmp >> 8) & 0xFF);
			offset += sprintf(buf + offset, "phy_rate:%X ", (tmp >> 6) & 0x3);
			offset += sprintf(buf + offset, "ltssm_state:%X\n", tmp&0x3f);
		}
		pcie_phy_cfg(phy, X2_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(1));
		udelay(1000);
		pcie_phy_cfg(phy, X2_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(2));
		udelay(1000);
		pcie_phy_cfg(phy, X2_MISC_FUNC_CLEAR_CTRL0, 0x1, BIT(0));
		udelay(1000);
	} else {
		len = 0;
		return len;
	}

	len = offset;
	return len;
}
static DEVICE_ATTR_RO(ltssm_status);

static ssize_t ltssm_en_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	unsigned long val;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	if (val == 0)
		bst_pcie_stop_link(bst_pcie->pci);
	else if (val == 1)
		bst_pcie_start_link(bst_pcie->pci);
	else
		return -EINVAL;

	return count;
}

static ssize_t ltssm_en_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;
	unsigned long val;
	u32 dev_info;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0): /*C1200 ctrl0 */
		val = pcie_phy_read(phy, PCIE_MODE_CTRL);
		val = (val >> 1) & 1;
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		val = pcie_phy_read(phy, PCIE_MODE_CTRL);
		val = (val >> 2) & 1;
		break;
	default:
		return -EINVAL;
	}
	return sprintf(buf, "%lu\n", val);
}
static DEVICE_ATTR_RW(ltssm_en);

static ssize_t perst_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	unsigned long val;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	if (gpio_is_valid(bst_pcie->reset_gpio))
		gpio_set_value_cansleep(bst_pcie->reset_gpio,
					(int)(val & 1));
	else
		return -EINVAL;

	return count;
}

static ssize_t perst_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	unsigned long val;

	if (gpio_is_valid(bst_pcie->reset_gpio))
		val = gpio_get_value_cansleep(bst_pcie->reset_gpio);
	else
		return -EINVAL;

	return sprintf(buf, "%lu\n", val);
}
static DEVICE_ATTR_RW(perst);

#ifdef CONFIG_ARCH_BSTC1200
static ssize_t shared_dbi_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;
	unsigned long val;
	u32 dev_info = 0;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL1, !!val, BIT(21));
		pcie_phy_cfg(phy, X4_AXI_SIDEBAND_CTRL4, !!val, BIT(21));
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1):
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL1, !!val, BIT(21));
		pcie_phy_cfg(phy, X2_AXI_SIDEBAND_CTRL4, !!val, BIT(21));
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t shared_dbi_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;
	unsigned long val;
	u32 dev_info = 0;

	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;

	switch (dev_info) {
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):
		val = ((pcie_phy_read(phy, X4_AXI_SIDEBAND_CTRL1) >> 21) & 1) << 1;
		val |= (pcie_phy_read(phy, X4_AXI_SIDEBAND_CTRL4) >> 21) & 1;
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1):
		val = ((pcie_phy_read(phy, X2_AXI_SIDEBAND_CTRL1) >> 21) & 1) << 1;
		val |= (pcie_phy_read(phy, X2_AXI_SIDEBAND_CTRL4) >> 21) & 1;
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%lx\n", val);
}
static DEVICE_ATTR_RW(shared_dbi);
#endif

static ssize_t phy_cr_store(struct device *dev, struct device_attribute *attr, const char *buf,
			    size_t count)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;
	int rw;
	char *token, *cur, *tmp;
	char *delimiter = " ";
	u16 ctrl = 0, addr = 0, data = 0;
	long result;

	tmp = kmalloc(strlen(buf) + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	strscpy(tmp, buf, strlen(buf) + 1);
	cur = tmp;

	while ((token = strsep(&cur, delimiter)) != NULL) {
		if (token[0] == 'c') {
			if (kstrtol(token + 2, 16, &result) == 0) {
				ctrl = (uint16_t)result;
			} else {
				count = -EINVAL;
				goto out;
			}
		} else if (token[0] == 'w') {
			rw = 1;
			if (kstrtol(token + 2, 16, &result) == 0) {
				addr = (uint16_t)result;
			} else {
				count = -EINVAL;
				goto out;
			}
		} else if (token[0] == 'r') {
			rw = 0;
			if (kstrtol(token + 2, 16, &result) == 0) {
				addr = (uint16_t)result;
			} else {
				count = -EINVAL;
				goto out;
			}
		} else if (token[0] == 'd') {
			if (kstrtol(token + 2, 16, &result) == 0) {
				data = (uint16_t)result;
			} else {
				count = -EINVAL;
				goto out;
			}
		}
	}
	if (rw)
	{
		c1200_write_phy_cr(phy, ctrl, addr, data);
	}
	else
		c1200_read_phy_cr(phy, ctrl, addr, &phy->last_cr_value);
out:
	kfree(tmp);
	return count;
}

static ssize_t phy_cr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bst_pcie *bst_pcie = dev->driver_data;
	struct pcie_phy *phy = bst_pcie->phy;

	return sprintf(buf, "%x\n", phy->last_cr_value);
}
static DEVICE_ATTR_RW(phy_cr);

#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	DEVICE_ATTR_RW(diagnostic);
#endif

static struct attribute *bst_pcie_dev_attrs[] = {
	&dev_attr_ltssm_status.attr,
	&dev_attr_ltssm_en.attr,
	&dev_attr_perst.attr,
#ifdef CONFIG_ARCH_BSTC1200
	&dev_attr_shared_dbi.attr,
#endif
	&dev_attr_phy_cr.attr,
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	&dev_attr_diagnostic.attr,
#endif
	NULL,
};

static struct attribute_group bst_pcie_attr_group = {
	.attrs = bst_pcie_dev_attrs,
};

static int bst_pcie_suspend_noirq(struct device *dev)
{
	struct bst_pcie *bst_pcie = dev_get_drvdata(dev);
	bst_pcie_stop_link(bst_pcie->pci);

	return 0;
}
static int bst_pcie_suspend(struct device *dev)
{
	struct bst_pcie *bst_pcie = dev_get_drvdata(dev);
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_RUNNING_STATE)
		bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_SUSPEND_STATE; // suspend diagnostic
	pr_info("suspend diagnostic first\n");
#endif
	return 0;
}

static int bst_pcie_resume_noirq(struct device *dev)
{
	struct bst_pcie *bst_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = bst_pcie->pci;

	dw_pcie_setup_rc(&pci->pp);
	bst_pcie_start_link(pci);
	dw_pcie_wait_for_link(pci);

	return 0;
}
static int bst_pcie_resume(struct device *dev)
{
	struct bst_pcie *bst_pcie = dev_get_drvdata(dev);
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_SUSPEND_STATE)
		bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_RUNNING_STATE; // resume diagnostic
	pr_info("resume diagnostic\n");
#endif

	return 0;
}


static const struct dev_pm_ops bst_pcie_pm_ops = {
	.suspend_noirq = bst_pcie_suspend_noirq,
	.resume_noirq = bst_pcie_resume_noirq,
	.suspend = bst_pcie_suspend,
	.resume = bst_pcie_resume,
};

#ifdef CONFIG_PCIE_BST_RESET
static struct bst_pcie *rcdev_to_pcie_host(struct reset_controller_dev *rcd)
{
	return container_of(rcd, struct bst_pcie, rcdev);
}

static int bst_pcie_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct bst_pcie *bst_pcie = rcdev_to_pcie_host(rcdev);
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info;
	u32 button_rst = 0;
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	button_rst = pcie_phy_read(phy, CRM_CTRL);

	switch (dev_info)
	{
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):  /*C1200 ctrl0 */
		button_rst &= ~(1<<4); // warm reset
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		button_rst &= ~(1<<5); // warm reset
		break;
	default:
		pr_err("state: ctrl_id error\n");
		return -1;
	}
	//pr_info("bst_pcie_reset_assert btn rst:0x%x\n", button_rst);
	pcie_phy_write(phy, CRM_CTRL, button_rst);
	usleep_range(1000, 1100);
	return 0;
}

static int bst_pcie_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct bst_pcie *bst_pcie = rcdev_to_pcie_host(rcdev);
	struct pcie_phy *phy = bst_pcie->phy;
	u32 dev_info;
	u32 button_rst = 0;
	dev_info = (bst_pcie->chip_type << 8) | bst_pcie->ctrl_id;
	button_rst = pcie_phy_read(phy, CRM_CTRL);

	switch (dev_info)
	{
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):  /*C1200 ctrl0 */
		button_rst |= (1<<4); 		// // warm reset
		break;
	case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1): /*C1200 ctrl1 */
		button_rst |= (1<<5);       // // warm reset
		break;
	default:
		pr_err("state: ctrl_id error\n");
		return -1;
	}

	pcie_phy_write(phy, CRM_CTRL, button_rst);
	//button_rst = pcie_phy_read(phy, CRM_CTRL);
	//pr_info("bst_pcie_reset_deassert btn rst:0x%x\n", button_rst);
	usleep_range(1000, 1100);
	return 0;
}

static const struct reset_control_ops bst_pcie_reset_ops = {
	.assert = bst_pcie_reset_assert,
	.deassert = bst_pcie_reset_deassert,
};

static void bst_pcie_reset(struct reset_controller_dev *rcdev)
{
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	struct bst_pcie *bst_pcie = rcdev_to_pcie_host(rcdev);
	if(bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_RUNNING_STATE)
		bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_SUSPEND_STATE; // suspend diagnostic
	pr_info("suspend diagnostic\n");
#endif
	bst_pcie_reset_assert(rcdev, 0);
	bst_pcie_reset_deassert(rcdev, 0);
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_SUSPEND_STATE)
		bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_RUNNING_STATE; // resume diagnostic
	pr_info("resume diagnostic\n");
#endif
}
#endif
static void bst_pcie_shutdown(struct platform_device *pdev)
{
#ifdef CONFIG_PCIE_BST_RESET
	struct bst_pcie *bst_pcie = platform_get_drvdata(pdev);
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->bst_pcie_diag->diag_status == PCIE_DIAG_RUNNING_STATE)
		bst_pcie->bst_pcie_diag->diag_status = PCIE_DIAG_SHUTDOWN_STATE; // stop diagnostic
	pr_info("stop diagnostic first\n");
#endif
	bst_pcie_reset(&bst_pcie->rcdev);
#endif
	dev_info(&pdev->dev, "Shutdown\n");
}

static int dw_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_pcie *bst_pcie;
	struct dw_pcie *pci;
	int ret;
	const struct dw_plat_pcie_of_data *data;
	enum dw_pcie_device_mode mode;
	struct dw_pcie_ops *bst_pcie_ops;

#ifdef CONFIG_HAVE_DBI_MUTEX
	mutex_init(&dbi_mutex);
#endif

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	mode = (enum dw_pcie_device_mode)data->mode;

	bst_pcie = devm_kzalloc(dev, sizeof(*bst_pcie), GFP_KERNEL);
	if (!bst_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;

	bst_pcie->pci = pci;
	bst_pcie->mode = mode;

	// INIT SPIN LOCK
	spin_lock_init(&bst_pcie->share_dbi_lock);

	/* Creat phy struct */
	bst_pcie->phy = devm_kzalloc(dev, sizeof(*bst_pcie->phy), GFP_KERNEL);
	if (!bst_pcie->phy)
		return -ENOMEM;
	bst_pcie->phy->dev = dev;
	#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	bst_pcie->phy->bst_pcie = bst_pcie;
	#endif
	bst_pcie->phy->is_pre_init = &bst_pcie->is_pre_init;
	bst_pcie->phy->ctrl_id = &bst_pcie->ctrl_id;
	bst_pcie->phy->chip_type = &bst_pcie->chip_type;

	ret = of_property_read_u32(dev->of_node, "controller-id", &bst_pcie->ctrl_id);
	if (ret) {
		pr_err("controller-id Undefined. set to 0\n");
		bst_pcie->ctrl_id = 0;
	}

	bst_pcie->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	bst_pcie->gpio_active_high = of_property_read_bool(dev->of_node,
						"reset-gpio-active-high");
	if (gpio_is_valid(bst_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, bst_pcie->reset_gpio,
				bst_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	}

	ret = of_property_read_u32(dev->of_node, "chip-type", &bst_pcie->chip_type);
	if (ret) {
		bst_pcie->chip_type = 0;
		pr_info("chip-type undefined, use default:%#x\n", bst_pcie->chip_type);
	}

	pci->edma.reg_base = devm_platform_ioremap_resource_byname(pdev, "dma");

	bst_pcie->is_pre_init = of_property_read_bool(dev->of_node, "pre-init");
	pr_info("PCIe init by %s\n", bst_pcie->is_pre_init ? "PBC" : "Driver");

	platform_set_drvdata(pdev, bst_pcie);

	bst_pcie_ops = devm_kzalloc(dev, sizeof(*bst_pcie_ops), GFP_KERNEL);
	bst_pcie_ops->link_up = bst_pcie_link_up;
	bst_pcie_ops->start_link = bst_pcie_start_link;
	bst_pcie_ops->stop_link = bst_pcie_stop_link;
	pci->ops = bst_pcie_ops;

	ret = bst_pcie_phyinit(bst_pcie->phy);
	if (ret) {
		pr_info("bst pcie phy init failed ret:%d\n", ret);
		bst_pcie_phydeinit(bst_pcie->phy);
		return ret;
	}
#ifdef CONFIG_PCIE_BST_RESET
	/* Fire up the reset controller. Failure here is non-fatal. */
	bst_pcie->rcdev.of_node = dev->of_node;
	bst_pcie->rcdev.ops = &bst_pcie_reset_ops;
	bst_pcie->rcdev.owner = dev->driver->owner;
	bst_pcie->rcdev.nr_resets = 1;
	ret = devm_reset_controller_register(dev, &bst_pcie->rcdev);
	if (ret)
		pr_warn("pcie bst Failed to register reset controller\n");
	// bst_pcie_reset(&bst_pcie->rcdev);
#endif

	dma_set_mask_and_coherent(pci->dev, DMA_BIT_MASK(64));

	switch (bst_pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_PLAT_HOST))
			return -ENODEV;

#ifdef CONFIG_ARCH_BSTC1200
		bst_pcie_ops->read_dbi = bst_pcie_read_dbi,
		bst_pcie_ops->write_dbi = bst_pcie_write_dbi,
		bst_pcie_ops->write_dbi2 = bst_pcie_write_dbi2,
#endif
		ret = dw_plat_add_pcie_port(bst_pcie, pdev);
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_PLAT_EP))
			return -ENODEV;

#ifdef CONFIG_ARCH_BSTC1200
#ifndef CONFIG_PCIE_BST_EP_USE_OUTBOUND
		bst_pcie_ops->read_dbi = NULL;
		bst_pcie_ops->write_dbi = NULL;
		bst_pcie_ops->write_dbi2 = NULL;

		switch ((bst_pcie->chip_type << 8) | bst_pcie->ctrl_id) {
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL0):
			pcie_phy_cfg(bst_pcie->phy, X4_AXI_SIDEBAND_CTRL1, 1, BIT(21));
			pcie_phy_cfg(bst_pcie->phy, X4_AXI_SIDEBAND_CTRL4, 1, BIT(21));
			break;
		case ((PCIE_C1200_SERIES << 8) | PCIE_CTRL1):
			pcie_phy_cfg(bst_pcie->phy, X2_AXI_SIDEBAND_CTRL1, 1, BIT(21));
			pcie_phy_cfg(bst_pcie->phy, X2_AXI_SIDEBAND_CTRL4, 1, BIT(21));
			break;
		default:
			break;
		}
#else
		bst_pcie_ops->read_dbi = bst_pcie_read_dbi,
		bst_pcie_ops->write_dbi = bst_pcie_write_dbi,
		bst_pcie_ops->write_dbi2 = bst_pcie_write_dbi2,
#endif
#endif
		pci->ep.ops = &pcie_ep_ops;
		ret = dw_pcie_ep_init(&pci->ep);
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", bst_pcie->mode);
		ret = -EINVAL;
		break;
	}
	
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	bst_pcie->phy->bst_pcie = bst_pcie;
	pcie_bst_diag_init(bst_pcie);
#endif

	if (ret)
		bst_pcie_phydeinit(bst_pcie->phy);
	else
		if (sysfs_create_group(&dev->kobj, &bst_pcie_attr_group))
			pr_err("Failed to create sysfs files\n");

	return ret;
}

static int dw_plat_pcie_remove(struct platform_device *pdev)
{
	struct bst_pcie *bst_pcie = platform_get_drvdata(pdev);
	struct dw_pcie *pci = bst_pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct dw_pcie_ep *ep = &pci->ep;

	switch (bst_pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_PLAT_HOST))
			return -ENODEV;

		dw_pcie_host_deinit(pp);
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_BST_PLAT_EP))
			return -ENODEV;

		dw_pcie_ep_exit(ep);
		devm_pci_epc_destroy(pci->dev, ep->epc);
		break;
	default:
		break;
	}
	if (bst_pcie->legacy_irq_domain)
		irq_domain_remove(bst_pcie->legacy_irq_domain);
	bst_pcie_stop_link(pci);
	if (gpio_is_valid(bst_pcie->reset_gpio))
		gpio_set_value(bst_pcie->reset_gpio,
			       !bst_pcie->gpio_active_high);
	bst_pcie_phydeinit(bst_pcie->phy);
	pr_err("pcie remove completed.\n");

	return 0;
}

static const struct dw_plat_pcie_of_data dw_plat_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct dw_plat_pcie_of_data dw_plat_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id dw_plat_pcie_of_match[] = {
	{
		.compatible = "bst,dw-pcie",
		.data = &dw_plat_pcie_rc_of_data,
	},
	{
		.compatible = "bst,dw-pcie-ep",
		.data = &dw_plat_pcie_ep_of_data,
	},
	{},
};

static struct platform_driver dw_plat_pcie_driver = {
	.driver = {
		.name	= "dw-pcie",
		.of_match_table = dw_plat_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &bst_pcie_pm_ops,
	},
	.probe = dw_plat_pcie_probe,
	.remove = dw_plat_pcie_remove,
	.shutdown = bst_pcie_shutdown,
};
module_platform_driver(dw_plat_pcie_driver);

MODULE_LICENSE("GPL");
