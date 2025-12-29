// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * DWC3 Specific Glue layer for BST usb
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/usb/otg.h>
#include <linux/of_gpio.h>
#include <linux/reset.h>

struct dwc3_bst {
	struct device *dev;
	enum usb_dr_mode dr_mode;
	struct clk_bulk_data *clks;
	int num_clks;
	int external_clk;
	struct reset_control *reset;
	int powerctrl_gpio;
	enum of_gpio_flags gpio_flags;
	struct phy *usb_generic_phy;
};

static int dwc3_bst_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

//ret 0 internal,others external
static int usb_get_usb_pll_type(struct device_node *np)
{
	const char *pll_type = NULL;
	int err = 0;
	struct device_node *phy_np = NULL;

	phy_np = of_parse_phandle(np, "phys", 0);
	if (!phy_np) {
		pr_err("Failed to find phy node\n");
		return -1;
	}
	err = of_property_read_string(phy_np, "pll_type", &pll_type);
	if (err < 0)
		return -1;
	if (pll_type && strncmp(pll_type, "internal", 5)) {	//size 5 "inter" valid
		return 1;
	}
	return 0;
}

void usb3_inter_clk_ref_alt_disable(struct dwc3_bst *bst)
{
	int i = 0;

	if (bst == NULL)
		return;
	for (i = 0; i < bst->num_clks; i++) {
		if (!strncmp(bst->clks[i].id, "ref", 3)) {
			clk_disable(bst->clks[i].clk);
			break;
		}
	}
}

void dwc3_bst_usb3_power_gpio(struct dwc3_bst *bst)
{
	struct device *dev = NULL;
	int ret = 0;
	int enable_gpio = 0;

	if (bst == NULL)
		return;
	dev = bst->dev;
	if (bst->powerctrl_gpio < 0) {
		/* some platform USB do not use GPIO to control power IC */
		dev_info(dev, "usb30 could not find power control gpio.\n");
	} else {
		if (gpio_is_valid(bst->powerctrl_gpio)) {
			ret = devm_gpio_request(dev, bst->powerctrl_gpio,
					      dev_name(dev));
			if (ret) {
				dev_err(dev,
					"usb30 could not request power control gpio.\n");
				return;
			}
			enable_gpio =
			    bst->gpio_flags & OF_GPIO_ACTIVE_LOW ? 0 : 1;
			if (bst->dr_mode == USB_DR_MODE_PERIPHERAL)
				gpio_direction_output(bst->powerctrl_gpio,
						      !enable_gpio);
			else if (bst->dr_mode == USB_DR_MODE_HOST)
				gpio_direction_output(bst->powerctrl_gpio,
						      enable_gpio);
			dev_dbg(dev,
				"usb3 power gpio %d flags %d enable_gpio %d\n",
				bst->powerctrl_gpio, bst->gpio_flags,
				enable_gpio);
		} else {
			dev_err(dev,
				"usb30 could not get valid power control gpio.\n");
		}
	}
}

static struct device_node *dwc3_bst_find_child(struct device *dev,
					       const char *compatible)
{
	struct device_node *np;

	np = of_get_compatible_child(dev->of_node, compatible);
	if (!np)
		return NULL;
	return np;
}

static void dwc3_bst_put_node(struct device_node *np)
{
	if (np)
		of_node_put(np);
}

static enum usb_dr_mode dwc3_bst_get_dr_mode(struct device_node *np)
{
	enum usb_dr_mode ret_dr_mode = USB_DR_MODE_UNKNOWN;

	if (np) {
		const char *dr_mode = NULL;
		int err = of_property_read_string(np, "dr_mode", &dr_mode);

		if (err == 0) {
			if (strncmp("host", dr_mode, strlen("host")) == 0)
				ret_dr_mode = USB_DR_MODE_HOST;
			else if (strncmp("peripheral", dr_mode,
					   strlen("peripheral")) == 0)
				ret_dr_mode = USB_DR_MODE_PERIPHERAL;
			else if (strncmp("otg", dr_mode, strlen("otg")) == 0)
				ret_dr_mode = USB_DR_MODE_OTG;
		}
	}
	return ret_dr_mode;
}

static int dwc3_bst_probe(struct platform_device *pdev)
{
	struct dwc3_bst *bst;
	struct device *dev = NULL;
	struct device_node *np = NULL, *child_np = NULL;
	int ret = 0;

	if (pdev == NULL)
		return -EINVAL;
	dev = &pdev->dev;
	np = dev->of_node;
	bst = devm_kzalloc(dev, sizeof(*bst), GFP_KERNEL);
	if (!bst) {
		dev_err_probe(dev, ret, "alloc dev fail\n");
		return -ENOMEM;
	}
	bst->dev = dev;
	bst->reset = devm_reset_control_array_get_optional_shared(dev);
	if (IS_ERR(bst->reset)) {
		dev_err(dev, "reset get fail\n");
		return PTR_ERR(bst->reset);
	}
	ret = devm_clk_bulk_get_all(dev, &bst->clks);
	if (ret == -EPROBE_DEFER) {
		dev_err_probe(dev, ret, "clk get fail\n");
		return ret;
	}
	if (ret < 0)
		bst->num_clks = 0;
	else
		bst->num_clks = ret;
	child_np = dwc3_bst_find_child(dev, "snps,dwc3");
	if (child_np) {
		bst->dr_mode = dwc3_bst_get_dr_mode(child_np);
		dwc3_bst_put_node(child_np);
	} else {
		dev_err(dev, "no dwc3 node\n");
		return -ENODEV;
	}
	bst->powerctrl_gpio =
	    of_get_named_gpio_flags(dev->of_node, "powerctl-gpios", 0,
				    &(bst->gpio_flags));
	bst->usb_generic_phy = devm_phy_get(dev, "usb-phy");
	if (IS_ERR(bst->usb_generic_phy)) {
		ret = PTR_ERR(bst->usb_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV)
			bst->usb_generic_phy = NULL;
		else
			return dev_err_probe(dev, ret,
					     "no usb2 phy configured\n");
	}

	bst->external_clk = usb_get_usb_pll_type(np);

	platform_set_drvdata(pdev, bst);

	ret = reset_control_deassert(bst->reset);
	if (ret) {
		dev_err_probe(dev, ret, "reset fail\n");
		return ret;
	}
	dwc3_bst_usb3_power_gpio(bst);

	ret = clk_bulk_prepare_enable(bst->num_clks, bst->clks);
	if (ret) {
		dev_err_probe(dev, ret, "clk enable failed\n");
		goto assert_reset;
	}
	if (bst->external_clk)
		usb3_inter_clk_ref_alt_disable(bst);

	ret = phy_init(bst->usb_generic_phy);
	if (ret < 0) {
		dev_err_probe(dev, ret, "usb phy init failed\n");
		goto err_clk_put;
	}

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		dev_err_probe(dev, ret, "of_platform_populate failed ret %d\n",
			      ret);
		goto err_clk_put;
	}
	return 0;

err_clk_put:
	clk_bulk_disable_unprepare(bst->num_clks, bst->clks);
	clk_bulk_put_all(bst->num_clks, bst->clks);

assert_reset:
	reset_control_assert(bst->reset);

	return ret;
}

static int dwc3_bst_remove(struct platform_device *pdev)
{
	struct dwc3_bst *bst = platform_get_drvdata(pdev);
	int i;

	if (bst == NULL)
		return -1;
	device_for_each_child(&pdev->dev, NULL, dwc3_bst_remove_child);

	for (i = bst->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(bst->clks[i].clk);

	return 0;
}

static int __maybe_unused dwc3_bst_suspend(struct device *dev)
{
	struct dwc3_bst *priv_data = dev_get_drvdata(dev);
	int ret;

	phy_exit(priv_data->usb_generic_phy);

	/* Disable the clocks */
	clk_bulk_disable(priv_data->num_clks, priv_data->clks);

	ret = reset_control_assert(priv_data->reset);
	if (ret) {
		dev_err_probe(dev, ret, "reset fail\n");
		return ret;
	}
	return 0;
}

static int __maybe_unused dwc3_bst_resume(struct device *dev)
{
	struct dwc3_bst *priv_data = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(priv_data->reset);
	if (ret) {
		dev_err_probe(dev, ret, "reset fail\n");
		return ret;
	}
	dwc3_bst_usb3_power_gpio(priv_data);

	ret = clk_bulk_enable(priv_data->num_clks, priv_data->clks);
	if (ret)
		return ret;
	if (priv_data->external_clk)
		usb3_inter_clk_ref_alt_disable(priv_data);

	ret = phy_init(priv_data->usb_generic_phy);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct dev_pm_ops dwc3_bst_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_bst_suspend, dwc3_bst_resume)
};

static const struct of_device_id bst_dwc3_match[] = {
	{
	 .compatible = "bst,dwc3usb",
	}, {	}
};

MODULE_DEVICE_TABLE(of, bst_dwc3_match);

static struct platform_driver dwc3_bst_driver = {
	.probe = dwc3_bst_probe,
	.remove = dwc3_bst_remove,
	.driver = {
		   .name = "bst-dwc3",
		   .of_match_table = bst_dwc3_match,
		   .pm = &dwc3_bst_dev_pm_ops,
		    },
};

module_platform_driver(dwc3_bst_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 bst Glue Layer");
