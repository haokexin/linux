// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * phy driver for BST usb
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include "phy-bst-usb.h"
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#define DEBUG 1
static const char *const usb_modes[] = {
	[USB_MODE_UNKNOWN] = "",
	[USB_MODE_USB2] = "usb20",
	[USB_MODE_USB3] = "usb30",
	[USB_MODE_USB31] = "usb31",
};

#if defined(CONFIG_TYPEC) || defined(CONFIG_TYPEC_MODULE)
static int usb_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	//struct bst_usb *ctx = typec_mux_get_drvdata(mux);

	return 0;
}

int set_orientation(struct bst_usb *ctx, unsigned int orientation)
{
	if (ctx) {
		if (ctx->cur_orientation != orientation) {
			u32 reg = 0;

			reg = readl(ctx->phy_tca_base + 0x14);    //14.1.5 TCA_TCPC
			reg &= ~0x3; //USB -> NC
			reg |= BIT(4);
			writel(reg, ctx->phy_tca_base + 0x14);
			wait_reg_bit(ctx->phy_tca_base + 8, 0, 1, 1);//
			writel(0xffff, ctx->phy_tca_base + 8);

			reg = readl(ctx->phy_tca_base + 0x14);
			if (orientation == TYPEC_ORIENTATION_REVERSE)
				reg |= BIT(2);  // LANE0->LANE1
			else if (orientation == TYPEC_ORIENTATION_NORMAL)
				reg &= ~BIT(2);
			reg |= BIT(4);
			writel(reg, ctx->phy_tca_base + 0x14);
			wait_reg_bit(ctx->phy_tca_base + 8, 0, 1, 1);
			writel(0xffff, ctx->phy_tca_base + 8);

			reg = readl(ctx->phy_tca_base + 0x14);
			reg &= ~0x3;
			reg |= 0x1; //NC ->USB
			reg |= BIT(4);
			writel(reg, ctx->phy_tca_base + 0x14);
			wait_reg_bit(ctx->phy_tca_base + 8, 0, 1, 1);//
			writel(0xffff, ctx->phy_tca_base + 8);
			ctx->cur_orientation = orientation;
			dev_info(ctx->dev, "switch orientation to %x\n",
				readl(ctx->phy_tca_base + 0x14));
		}
	}
	return 0;
}

static int usb_set_orientation(struct typec_switch *sw,
			       enum typec_orientation orientation)
{
	struct bst_usb *ctx = typec_switch_get_drvdata((struct typec_switch_dev *)sw);

	set_orientation(ctx, orientation);
	return 0;
}

static int bst_typec_register_switch(struct bst_usb *ctx,
				     struct device *dev,
				     struct fwnode_handle *fwnode)
{
	struct typec_switch_desc sw_desc = { 0 };

	sw_desc.fwnode = fwnode;
	sw_desc.drvdata = ctx;
	sw_desc.name = fwnode_get_name(fwnode);
	sw_desc.set = (typec_switch_set_fn_t)usb_set_orientation;

	ctx->typec_switch = (struct typec_switch *)(typec_switch_register(dev, &sw_desc));
	if (IS_ERR(ctx->typec_switch)) {
		dev_err(dev, "switch register failed\n");
		return PTR_ERR(ctx->typec_switch);
	}
	return 0;
}

static int bst_typec_register_mux(struct bst_usb *ctx,
				  struct device *dev,
				  struct fwnode_handle *fwnode)
{
	struct typec_mux_desc mux_desc = { };

	mux_desc.fwnode = fwnode;
	mux_desc.drvdata = ctx;
	mux_desc.name = fwnode_get_name(fwnode);
	mux_desc.set = (typec_mux_set_fn_t)usb_mux_set;

	ctx->typec_mux = (struct typec_mux *)(typec_mux_register(dev, &mux_desc));
	if (IS_ERR(ctx->typec_mux)) {
		dev_err(dev, "mux register failed\n");
		return PTR_ERR(ctx->typec_mux);
	}
	return 0;
}

static void bst_typec_unregister_mux(struct bst_usb *ctx)
{
	if (ctx->typec_mux) {
		typec_mux_unregister((struct typec_mux_dev *)(ctx->typec_mux));
		ctx->typec_mux = NULL;
	}
}

static void bst_typec_unregister_switch(struct bst_usb *ctx)
{
	if (ctx->typec_switch) {
		typec_switch_unregister((struct typec_switch_dev *)(ctx->typec_switch));
		ctx->typec_switch = NULL;
	}
}

int usb_typec_phy_probe(struct bst_usb *phy)
{
	struct device *dev = phy->dev;

	if (phy->is_typec_phy) {
		phy->phy_tca_base = phy->phy_base + PHY_TCA_OFFSET;
		bst_typec_register_switch(phy, dev, dev_fwnode(dev));
		bst_typec_register_mux(phy, dev, dev_fwnode(dev));
	}
	return 0;
}

int usb_typec_phy_remove(struct bst_usb *phy)
{
	if (phy->is_typec_phy) {
		bst_typec_unregister_mux(phy);
		bst_typec_unregister_switch(phy);
	}
	return 0;
}

#endif
int usb_typec_phy_init(struct bst_usb *ctx)
{
	void __iomem *phy_base = NULL;

	ctx->phy_tca_base = ctx->phy_base + PHY_TCA_OFFSET;
	phy_base = ctx->phy_tca_base;

	writel(0xffff, phy_base + 8);
	writel(0x3, phy_base + 4);
	writel(0x11, phy_base + 0x14);
	wait_reg_bit(ctx->phy_tca_base + 8, 0, 1, 1);//
	writel(0xffff, ctx->phy_tca_base + 8);

	return 0;
}

static enum usb_mode_enum usb_get_usb_mode_from_string(const char *str)
{
	int ret;

	ret = match_string(usb_modes, ARRAY_SIZE(usb_modes), str);
	return (ret < 0) ? USB_MODE_UNKNOWN : ret;
}

enum usb_mode_enum usb_get_usb_mode(struct device *dev)
{
	const char *usb_mode;
	int err;

	err = device_property_read_string(dev, "usb_mode", &usb_mode);
	if (err < 0)
		return USB_MODE_UNKNOWN;

	return usb_get_usb_mode_from_string(usb_mode);
}

static int bst_usb_phy_power(struct bst_usb *phy, int on)
{
	return 0;
}

static int bst_usb_power_off(struct phy *x)
{
	struct bst_usb *phy = phy_get_drvdata(x);

	return bst_usb_phy_power(phy, false);
}

static int bst_usb_power_on(struct phy *x)
{
	struct bst_usb *phy = phy_get_drvdata(x);

	return bst_usb_phy_power(phy, true);
}

void bst_usb3_phy_init(struct bst_usb *phy)
{
	void __iomem *phy_base = NULL;
	u32 reg = 0;

	phy_base = phy->phy_base;
	//ssc ssp
	reg = readl(phy_base + USB3PHY_REG_2);
	reg |= SSCEN;
	reg |= REFSSPEN;
	writel(reg, phy_base + USB3PHY_REG_2);
	if (phy->external_clk) {
		/* USB3 PLL config */
		reg = readl(phy_base + USB3PHY_REG_1);
		reg &= ~FSEL_MSK;
		reg |= (0x27 << FSEL_POS);
		reg &= ~MPLLMULTIPLIER_MSK;
		reg &= ~SSCREFCLKSEL_MSK;
		writel(reg, phy_base + USB3PHY_REG_1);

		reg = readl(phy_base + USB3PHY_REG_4);
		reg &= (~(REFCLKDIV2 | REFCLKSEL_MSK));
		writel(reg, phy_base + USB3PHY_REG_4);

		reg = readl(phy_base + USB3PHY_REG_2);
		reg |= REFUSEPAD;
		writel(reg, phy_base + USB3PHY_REG_2);
	} else {
		reg = readl(phy_base + USB3PHY_REG_2);
		reg &= ~REFUSEPAD;
		writel(reg, phy_base + USB3PHY_REG_2);

		reg = readl(phy_base + USB3PHY_REG_5);
		reg |= (BIT(11) | BIT(14));
		writel(reg, phy_base + USB3PHY_REG_5);
	}
	//reset
	reg = readl(phy_base + USB3PHY_LOCAL_RST);
	reg |= (PHYSWRSTN | CTRLSWRSTN);
	writel(reg, phy_base + USB3PHY_LOCAL_RST);
}


void bst_usb31_phy_init(struct bst_usb *phy)
{
	void __iomem *phy_base = NULL;
	u32 reg = 0;

	phy_base = phy->phy_base;

	reg = readl(phy_base + USB31_CRM_CTRL);
	reg &= ~CSR_CTRL_SW_RST;	//ctrl reset
	writel(reg, phy_base + USB31_CRM_CTRL);

	reg = readl(phy_base + USB31_CRM_CTRL);
	reg &= ~CSR_DIV_SW_RST;	//div reset
	writel(reg, phy_base + USB31_CRM_CTRL);

	reg = readl(phy_base + USB31_CRM_CTRL);
	reg |= (0x3<<1); //bit1: csr_u3phy_rst_n  bit2: csr_u2phy_rst_n internal usb reset
	writel(reg, phy_base + USB31_CRM_CTRL);

	mdelay(1);

	if (phy->external_clk) {
		reg = readl(phy_base + USB31PHY_REG_1);
		reg |= CSR_PHY_REF_USE_PAD;
		writel(reg, phy_base + USB31PHY_REG_1);
	} else {
		reg = readl(phy_base + USB31PHY_REG_1);
		reg &= ~CSR_PHY_REF_USE_PAD;
		writel(reg, phy_base + USB31PHY_REG_1);
	}

	reg = readl(phy_base + 0x254);
	reg &= ~(1<<8 | 1<<9 | 1<<10 | 1<<11);
	writel(reg, phy_base + 0x254);

	//reg = readl(phy_base + USB31PHY_REG_1);
	//reg |= 1<<16;
	//writel(reg, phy_base + USB31PHY_REG_1);

	usb31_phy_use_sram_direct_init(phy_base);

	reg = readl(phy_base + USB31_CRM_CTRL);
	reg |= CSR_DIV_SW_RST;	//div reset
	writel(reg, phy_base + USB31_CRM_CTRL);

	reg = readl(phy_base + USB31_CRM_CTRL);
	reg |= CSR_CTRL_SW_RST;	//ctrl reset
	writel(reg, phy_base + USB31_CRM_CTRL);

	if (phy->host_force_gen1_speed) {
		reg = readl(phy_base + USB31_HOST_CTRL0);
		reg |= CSR_HOST_FORCE_GEN1_SPEED;
		writel(reg, phy_base + USB31_HOST_CTRL0);
	}
	mdelay(50);
	usb_typec_phy_init(phy);
}

void bst_usb2_phy_init(struct bst_usb *phy)
{
	writel((COMMONONN | PHYRESET), phy->phy_base + USB2PHY_PWR_CTRL);
	writel(COMMONONN, phy->phy_base + USB2PHY_PWR_CTRL);
}

static int bst_usb_init(struct phy *x)
{
	struct bst_usb *phy = phy_get_drvdata(x);

	if (phy) {
		if (phy->usb_mode == USB_MODE_USB2)
			bst_usb2_phy_init(phy);
		else if (phy->usb_mode == USB_MODE_USB3)
			bst_usb3_phy_init(phy);
		else
			bst_usb31_phy_init(phy);
	}
	return 0;
}


static int bst_usb_reset(struct phy *x)
{
	struct bst_usb *phy = phy_get_drvdata(x);

	if (phy) {
		if (phy->usb_mode == USB_MODE_USB2)
			bst_usb2_phy_init(phy);
		else if (phy->usb_mode == USB_MODE_USB3)
			bst_usb3_phy_init(phy);
		else
			bst_usb31_phy_init(phy);
	}
	return 0;
}

static int bst_usb_exit(struct phy *x)
{
	return 0;
}

static const struct phy_ops ops = {
	.init = bst_usb_init,
	.exit = bst_usb_exit,
	.power_on = bst_usb_power_on,
	.power_off = bst_usb_power_off,
	.reset = bst_usb_reset,
	.owner = THIS_MODULE,
};

//ret 0 internal,others external
static int usb_get_usb_pll_type(struct device *dev)
{
	const char *pll_type = NULL;
	int err = 0;

	err = device_property_read_string(dev, "pll_type", &pll_type);
	if (err < 0)
		return -1;
	if (pll_type && strncmp(pll_type, "internal", 5)) {	//size 5 "inter" valid
		return 1;
	}
	return 0;
}

static int bst_usb_probe(struct platform_device *pdev)
{
	struct bst_usb *phy;
	struct phy *generic_phy;
	struct resource *res;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	enum usb_mode_enum usb_mode;
	int ret = 0;

	usb_mode = usb_get_usb_mode(dev);
	if (usb_mode == USB_MODE_UNKNOWN) {
		dev_err(dev, "missing usb mode settting.\n");
		return -ENODEV;
	}

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;
	phy->reset = devm_reset_control_array_get_optional_shared(dev);
	if (IS_ERR(phy->reset))
		return PTR_ERR(phy->reset);
	phy->external_clk = usb_get_usb_pll_type(dev);
	phy->usb_mode = usb_mode;
	phy->dev = dev;
	phy->is_typec_phy = false;
	phy->allow_cr_test = true;
	ret = device_property_read_u32(dev, "vboost", &phy->vboost);
	if (ret)
		phy->vboost = 7;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy->phy_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(phy->phy_base)) {
		dev_err(dev, "devm_ioremap_resource err\n");
		return PTR_ERR(phy->phy_base);
	}
	platform_set_drvdata(pdev, phy);

	ret = bst_sysfs_cr_create(dev);
	if (ret)
		dev_err(dev, "Failed to create sysfs group\n");

	pm_runtime_enable(phy->dev);
	phy->host_force_gen1_speed = device_property_read_bool(dev, "host_force_gen1_speed");
#if defined(CONFIG_TYPEC) || defined(CONFIG_TYPEC_MODULE)
	phy->is_typec_phy = device_property_read_bool(dev, "is_typec_phy");
	usb_typec_phy_probe(phy);
#endif
	generic_phy = devm_phy_create(phy->dev, NULL, &ops);
	if (IS_ERR(generic_phy)) {
		dev_err(dev, "devm_phy_create err\n");
		return PTR_ERR(generic_phy);
	}

	phy_set_drvdata(generic_phy, phy);

	phy_provider = devm_of_phy_provider_register(phy->dev,
						     of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "devm_of_phy_provider_register err\n");
		return PTR_ERR(phy_provider);
	}
	ret = reset_control_deassert(phy->reset);
	if (ret) {
		dev_err(phy->dev, "reset_control_deassert fail.\n");
		return ret;
	}
	return 0;
}

static int bst_usb_remove(struct platform_device *pdev)
{

	bst_sysfs_cr_remove(&pdev->dev);
#if defined(CONFIG_TYPEC) || defined(CONFIG_TYPEC_MODULE)
{
	struct bst_usb *phy = (struct bst_usb *)platform_get_drvdata(pdev);

	usb_typec_phy_remove(phy);
}
#endif
	return 0;
}

static const struct of_device_id bst_phy_match[] = {
	{.compatible = "bst,dwc-usb-phy", },
	{ },
};

MODULE_DEVICE_TABLE(of, bst_phy_match);

static struct platform_driver bst_usb_driver = {
	.probe = bst_usb_probe,
	.remove = bst_usb_remove,
	.driver = {
		   .name = "bst-usb",
		   .of_match_table = bst_phy_match,
		    },
};

module_platform_driver(bst_usb_driver);

MODULE_ALIAS("platform:bst_usb");
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("Black Sesame Technologies USB phy driver");
MODULE_LICENSE("GPL v2");
