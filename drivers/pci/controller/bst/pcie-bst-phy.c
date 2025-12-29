// SPDX-License-Identifier: GPL-2.0
/*
 * BST PCIe controller csr driver
 *
 * Copyright (C) 2024 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_gpio.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/list.h>

#include "../../pci.h"
#include "pcie-bst-phy.h"
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
#include "pcie-bst-diagnostic.h"
extern int send_dtc_to_safety_svc(u32 dtc);
#endif

struct phy_init_state {
	struct list_head list;
	char name[32];
	u32 count;
};

static LIST_HEAD(phy_state_list);

void c1200_write_phy_cr(struct pcie_phy *phy, int ctrl, u16 addr, u16 val)
{
	u32 cr = (addr << 16) | val;
	u32 tmp;
#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	u16 rd_data;
	struct bst_pcie *bst_pcie = phy->bst_pcie;
#endif
	int timeout = 100;

	if (ctrl == 0) {
		pcie_phy_write(phy, PHY0_CR_CTRL, cr);

		tmp = pcie_phy_read(phy, PHY_CR_CTRL);
		tmp |= (1 << 0);	/* csr_phy0_cr_para_sel */
		pcie_phy_write(phy, PHY_CR_CTRL, tmp);

		tmp = (1 << 1);	/* csr_phy0_cr_para_wr_en */
		pcie_phy_write(phy, PHY_CTRL10, tmp);

		/* wait for phy0_cr_para_ack */
		do {
			tmp = pcie_phy_read(phy, PCIE_PHY0_CR_PARA_STATE);
			timeout--;
			if (!timeout) {
				pr_err("write phy0 cr timeout\n");
				return;
			}
			udelay(100);
		} while (tmp & 1);
	} else {
		pcie_phy_write(phy, PHY1_CR_CTRL, cr);

		tmp = pcie_phy_read(phy, PHY_CR_CTRL);
		tmp |= (1 << 8);	/* csr_phy1_cr_para_sel */
		pcie_phy_write(phy, PHY_CR_CTRL, tmp);

		tmp = (1 << 4);	/* csr_phy1_cr_para_wr_en */
		pcie_phy_write(phy, PHY_CTRL10, tmp);

		/* wait for phy1_cr_para_ack */
		do {
			tmp = pcie_phy_read(phy, PCIE_PHY1_CR_PARA_STATE);
			timeout--;
			if (!timeout) {
				pr_err("write phy1 cr timeout\n");
				return;
			}
			udelay(100);
		} while (tmp & 1);
	}

#ifdef CONFIG_PCIE_BST_DIAGNOSTIC
	if(bst_pcie->pcie_diagnostic_init_done)
	{
		if(bst_pcie->bst_pcie_diag->cr_check_safety_monitor_psm 
			&& bst_pcie->bst_pcie_diag->cr_check_safety_monitor_enable
			&& addr < PHY_ADDR_SPACE_LIMIT_UPPER) // exclude rom/sram addr space <refer:dwc_ap_16g_lp_phyxxxx databoot table6-24>
			{
				c1200_read_phy_cr(phy, ctrl, addr, &rd_data);
				if(rd_data != val)
				{
					pr_err("phy r/w mismatch val:0x%x rd_data:0x%x\n", val, rd_data);
					send_dtc_to_safety_svc(PSM_ID_PHY_REG_CHECK_DTC);
				}
			}
	}
#endif
}

void c1200_read_phy_cr(struct pcie_phy *phy, int ctrl, u16 addr, u16 *val)
{
	u32 cr = (addr << 16);
	u32 tmp;
	int timeout = 100;

	if (ctrl == 0) {
		pcie_phy_write(phy, PHY0_CR_CTRL, cr);

		tmp = pcie_phy_read(phy, PHY_CR_CTRL);
		tmp |= (1 << 0);	/* csr_phy0_cr_para_sel */
		pcie_phy_write(phy, PHY_CR_CTRL, tmp);

		tmp = (1 << 0);	/* csr_phy0_cr_para_rd_en */
		pcie_phy_write(phy, PHY_CTRL10, tmp);
		/* wait for phy0_cr_para_ack */
		do {
			tmp = pcie_phy_read(phy, PCIE_PHY0_CR_PARA_STATE);
			timeout--;
			if (!timeout) {
				pr_err("read phy0 cr timeout\n");
				return;
			}
			udelay(100);
		} while (tmp & 1);
		*val = (tmp >> 16) & 0xFFFF;
	} else {
		pcie_phy_write(phy, PHY1_CR_CTRL, cr);

		tmp = pcie_phy_read(phy, PHY_CR_CTRL);
		tmp |= (1 << 8);	/* csr_phy1_cr_para_sel */
		pcie_phy_write(phy, PHY_CR_CTRL, tmp);

		tmp = (1 << 3);	/* csr_phy1_cr_para_rd_en */
		pcie_phy_write(phy, PHY_CTRL10, tmp);
		/* wait for phy0_cr_para_ack */
		do {
			tmp = pcie_phy_read(phy, PCIE_PHY1_CR_PARA_STATE);
			timeout--;
			if (!timeout) {
				pr_err("read phy0 cr timeout\n");
				return;
			}
			udelay(100);
		} while (tmp & 1);
		*val = (tmp >> 16) & 0xFFFF;
	}
}

static void bst_pcie_set_its_target(struct pcie_phy *phy)
{
	pcie_phy_cfg(phy, PCIE_RSV0, 0x2, GENMASK(2, 0));
	pcie_phy_cfg(phy, PCIE_RSV0, 0x2, GENMASK(6, 4));
	pcie_phy_cfg(phy, X2_ITS_CTRL, 0x3287, GENMASK(15, 0));
	pcie_phy_cfg(phy, X4_ITS_CTRL, 0x3285, GENMASK(15, 0));
}

static bool pcie_phy_is_inited(const char *name)
{
	struct phy_init_state *state;

	list_for_each_entry(state, &phy_state_list, list) {
		if (!strcmp(state->name, name)) {
			if (state->count++)
				return true;
		}
	}
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return false;
	strscpy(state->name, name, sizeof(state->name));
	state->count++;
	list_add(&state->list, &phy_state_list);
	return false;
}

static void phy_eq_quirk(struct pcie_phy *phy)
{
	int i;
	u16 tmp;

	for (i = 0; i < 2; i++) {
		if (i == 0 && !phy->pcie_ctl0)
			continue;
		if (i == 1 && !phy->pcie_ctl1)
			continue;
		c1200_read_phy_cr(phy, i, 0x4048, &tmp);
		tmp |= (1 << 12);
		c1200_write_phy_cr(phy, i, 0x4048, tmp);

		c1200_read_phy_cr(phy, i, 0x4148, &tmp);
		tmp |= (1 << 12);
		c1200_write_phy_cr(phy, i, 0x4148, tmp);
	}
}

static int c1200_pcie_phyinit(struct pcie_phy *phy)
{
	u32 reg_val, tmp;
	int timeout = 100;

	pr_info("PCIe bifurcate:%s, Ctrl0:%s, Ctrl1:%s\n",
		phy->dmc_lane ? "en" : "dis",
		phy->pcie_ctl0 ? (phy->dmc_mode & 0x2) ? "RC" : "EP" : "disable",
		phy->pcie_ctl1 ? (phy->dmc_mode & 0x1) ? "RC" : "EP" : "disable");

	/* Unlock PCIe Reg write protect */
	pcie_phy_write(phy, PCIE_REG_WR_PROTECT, 0xabcd1234);

	/* Hold ctrl rst */
	pcie_phy_cfg(phy, CRM_CTRL, 0, GENMASK(5, 0));
	pcie_phy_cfg(phy, CRM_CTRL, 3, GENMASK(8, 7));
	/* Hold csr_phy_rst */
	pcie_phy_cfg(phy, PHY_CTRL0, 1, BIT(9));

	/* Configure PCIe-PHY ref clk source */
	if (phy->pcie_ctl0 == 1) {
		if (phy->pcie_ref_clk0 == PCIE_USE_PAD_REFCLK) {
			pcie_phy_cfg(phy, PHY_CTRL0, 1, BIT(0));
			pr_info("Configure PCIe phy0 use pad ref clk");
		} else {
			pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(0));
			pr_info("Configure PCIe phy0 use on-chip clk");
		}
	}
	if (phy->pcie_ctl1 == 1 || phy->dmc_lane == 0) {
		if (phy->pcie_ref_clk1 == PCIE_USE_PAD_REFCLK) {
			pcie_phy_cfg(phy, PHY_CTRL0, 1, BIT(1));
			pr_info("Configure PCIe phy1 use pad ref clk");
		} else if (phy->pcie_ref_clk1 == PCIE_USE_REPEAT_REFCLK) {
			pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(1));
			pcie_phy_cfg(phy, PHY_CTRL0, 1, BIT(8));
			pr_info("Configure PCIe phy1 use repeat clk");
		} else {
			pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(1));
			pcie_phy_cfg(phy, PHY_CTRL0, 1, BIT(8));
			pr_info("Configure PCIe phy1 use on-chip clk");
		}
	}

	/* Configure bifurcation */
	pcie_phy_cfg(phy, PCIE_MODE_CTRL, phy->dmc_lane, BIT(0));

	/* Configure link number */
	if (phy->dmc_lane) {
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(11, 8));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(15, 12));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x01, GENMASK(19, 16));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x01, GENMASK(23, 20));
	} else {
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(11, 8));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(15, 12));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(19, 16));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(23, 20));
	}

	/* Configure phy source */
	if (phy->dmc_lane) {
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(1, 0));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(3, 2));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x01, GENMASK(5, 4));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x01, GENMASK(7, 6));
	} else {
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(1, 0));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(3, 2));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(5, 4));
		pcie_phy_cfg(phy, PIPE_MODE_CTRL, 0x00, GENMASK(7, 6));
	}

	/* Configure RC or EP mode */
	if (phy->dmc_mode & BIT(1))
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x04, GENMASK(11, 8));
	else
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x00, GENMASK(11, 8));

	if (phy->dmc_mode & BIT(0))
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x04, GENMASK(15, 12));
	else
		pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x00, GENMASK(15, 12));

	pcie_phy_cfg(phy, PHY_CTRL8, 0x1, BIT(0));

	/* Configure phy_sram bypass */
	pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(4));
	pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(5));

	/* App hold_phy reset */
	pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(7));
	pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(8));

	/* Release csr_phy_rst */
	pcie_phy_cfg(phy, PHY_CTRL0, 0, BIT(9));

	if (phy->dmc_lane == 1)
		reg_val = (phy->pcie_ctl1 << 1) | phy->pcie_ctl0;
	else
		reg_val = 0x3;

	do {
		tmp = pcie_phy_read(phy, PHY_STATUS0);
		tmp = (tmp >> 2) & 0x3;
		timeout--;
		if (!timeout) {
			pr_err("Wait PCIe Phy sram init done timeout: 0x%X\n", tmp);
			return -ETIMEDOUT;
		}
		udelay(100);
	} while ((tmp & reg_val) != reg_val);

	phy_eq_quirk(phy);

	/* Configure sram_ext_ld_done */
	pcie_phy_cfg(phy, PHY_CTRL0, reg_val, GENMASK(3, 2));

	if (phy->pcie_ctl0) {
		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL0, 0x3f, GENMASK(13, 8));

		/* Release button_rst power_up_rst perst_n */
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(4));
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(2));
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(0));

		/* Release x4_app_hold_phy_rst */
		pcie_phy_cfg(phy, CRM_CTRL, 0x0, BIT(8));

		pcie_phy_cfg(phy, X4_MISC_FUNC_CTRL1, 0x1, BIT(0));
	}

	if (phy->pcie_ctl1) {
		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL0, 0x3f, GENMASK(13, 8));

		/* Release button_rst power_up_rst perst_n */
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(5));
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(3));
		pcie_phy_cfg(phy, CRM_CTRL, 0x1, BIT(1));

		/* Release x2_app_hold_phy_rst */
		pcie_phy_cfg(phy, CRM_CTRL, 0x0, BIT(7));

		pcie_phy_cfg(phy, X2_MISC_FUNC_CTRL1, 0x1, BIT(0));
	}

	/* Disable LTSSM */
	pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x0, BIT(1));
	pcie_phy_cfg(phy, PCIE_MODE_CTRL, 0x0, BIT(2));

	bst_pcie_set_its_target(phy);

	return 0;
}

static void c1200_pcie_phydeinit(struct pcie_phy *phy)
{
	pr_info("disable PCIe phy:%s\n", phy->name);
}

int bst_pcie_phyinit(struct pcie_phy *phy)
{
	int ret = 0;
	struct device *dev = phy->dev;
	struct device_node *np = dev->of_node;
	struct device_node *phy_np = NULL;
	void * __iomem *phy_base = NULL;

	phy_np = of_parse_phandle(np, "pcie-phy", 0);
	if (!phy_np) {
		pr_err("dts pcie-phy undefined\n");
		return -ENODEV;
	}

	strscpy(phy->name, phy_np->full_name, sizeof(phy->name));

	phy_base = of_iomap(phy_np, 0);
	if (!phy_base)
		return -ENOMEM;
	phy->phy_base = phy_base;

	ret = of_property_read_u32(phy_np, "dmc-mode", &phy->dmc_mode);
	if (ret) {
		phy->dmc_mode = PCIE_DWC_RC_MODE;
		pr_info("dmc-mode undefined, use default:%#x\n", phy->dmc_mode);
	}

	ret = of_property_read_u32(phy_np, "dmc-lane", &phy->dmc_lane);
	if (ret) {
		phy->dmc_lane = PCIE_DWC_LINEX4;
		pr_info("dmc-lane undefined, use default:%#x\n", phy->dmc_lane);
	}

	ret = of_property_read_u32(phy_np, "pcie-ctl0", &phy->pcie_ctl0);
	if (ret) {
		phy->pcie_ctl0 = 1;
		pr_info("pcie-ctl0 undefined, use default:%#x\n", phy->pcie_ctl0);
	}

	ret = of_property_read_u32(phy_np, "pcie-ctl1", &phy->pcie_ctl1);
	if (ret) {
		phy->pcie_ctl1 = 1;
		pr_info("pcie-ctl1 undefined, use default:%#x\n", phy->pcie_ctl1);
	}

	ret = of_property_read_u32(phy_np, "pcie-ref-clk0", &phy->pcie_ref_clk0);
	if (ret) {
		phy->pcie_ref_clk0 = PCIE_USE_INCHIP_REFCLK;
		pr_info("pcie-ref-clk0 undefined, use default:%#x\n", phy->pcie_ref_clk0);
	}

	ret = of_property_read_u32(phy_np, "pcie-ref-clk1", &phy->pcie_ref_clk1);
	if (ret) {
		phy->pcie_ref_clk1 = PCIE_USE_INCHIP_REFCLK;
		pr_info("pcie-ref-clk1 undefined, use default:%#x\n", phy->pcie_ref_clk1);
	}


	/* bst pcie-phy init only do once */
	if (pcie_phy_is_inited(phy->name))
		return 0;

	if (!*phy->is_pre_init) {
		switch (*phy->chip_type) {
		case PCIE_C1200_SERIES:
			ret = c1200_pcie_phyinit(phy);
			break;
		default:
			pr_err("unkonw PCIe chip type\n");
			break;
		}
	}

	return ret;
}

void bst_pcie_phydeinit(struct pcie_phy *phy)
{
	struct phy_init_state *state, *tmp;

	list_for_each_entry_safe(state, tmp, &phy_state_list, list) {
		if (strcmp(state->name, phy->name))
			continue;
		if (--state->count)
			return;
		if (!*phy->is_pre_init) {
			switch (*phy->chip_type) {
			case PCIE_C1200_SERIES:
				c1200_pcie_phydeinit(phy);
				break;
			default:
				pr_err("unkonw PCIe chip type\n");
				break;
			}
		}
		list_del(&state->list);
		kfree(state);
	}
}
