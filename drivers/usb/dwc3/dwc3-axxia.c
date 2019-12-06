// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/**
 * dwc3-axxia.c - Axxia Specific Glue layer
 *
 * Author: Sangeetha Rao <sangeetha.rao@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of_platform.h>

#define DWC3_AXXIA_MAX_CLOCKS	3

static u64 adwc3_dma_mask;

struct dwc3_axxia_driverdata {
	const char		*clk_names[DWC3_AXXIA_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
};

struct dwc3_axxia {
	struct device		*dev;

	const char		**clk_names;
	struct clk		*clks[DWC3_AXXIA_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;

};

static int axxia_dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*node = pdev->dev.of_node;
	struct dwc3_axxia	*adwc;
	const struct dwc3_axxia_driverdata *driver_data;
	int			error;
	int			i;

	adwc = devm_kzalloc(dev, sizeof(*adwc), GFP_KERNEL);
	if (!adwc)
		return -ENOMEM;

	driver_data = of_device_get_match_data(dev);
	if (!driver_data)
		return -ENODEV;

	adwc->dev = dev;
	adwc->num_clks = driver_data->num_clks;
	adwc->clk_names = (const char **)driver_data->clk_names;
	adwc->suspend_clk_idx = driver_data->suspend_clk_idx;

	platform_set_drvdata(pdev, adwc);

	for (i = 0; i < adwc->num_clks; i++) {
		adwc->clks[i] = devm_clk_get(dev, adwc->clk_names[i]);
		if (IS_ERR(adwc->clks[i])) {
			dev_err(dev, "failed to get clock: %s\n",
				adwc->clk_names[i]);
			return PTR_ERR(adwc->clks[i]);
		}
	}

	for (i = 0; i < adwc->num_clks; i++) {
		error = clk_prepare_enable(adwc->clks[i]);
		if (error) {
			while (i-- > 0)
				clk_disable_unprepare(adwc->clks[i]);
			return error;
		}
	}

	if (adwc->suspend_clk_idx >= 0)
		clk_prepare_enable(adwc->clks[adwc->suspend_clk_idx]);

	adwc3_dma_mask = dma_get_mask(dev);
	dev->dma_mask = &adwc3_dma_mask;

	if (node) {
		error = of_platform_populate(node, NULL, NULL, dev);
		if (error) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto populate_err;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		error = -ENODEV;
		goto populate_err;
	}

	return 0;

populate_err:
	for (i = adwc->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(adwc->clks[i]);

	if (adwc->suspend_clk_idx >= 0)
		clk_disable_unprepare(adwc->clks[adwc->suspend_clk_idx]);

	return error;
}

static int axxia_dwc3_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int axxia_dwc3_remove(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, axxia_dwc3_remove_core);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

void
arch_setup_pdev_archdata(struct platform_device *pdev)
{
	if (strncmp(pdev->name, "xhci-hcd", strlen("xhci-hcd")) == 0)
		arch_setup_dma_ops(&pdev->dev, 0, 0, NULL, 1);
}

static const struct dwc3_axxia_driverdata axxia_drvdata = {
	.clk_names = { "ref", "bus_early", "suspend" },
	.num_clks = 3,
	.suspend_clk_idx = 2,
};

static const struct of_device_id adwc3_of_match[] = {
	{ .compatible = "axxia,axxia-dwc3",
	  .data = &axxia_drvdata, },
	{},
};
MODULE_DEVICE_TABLE(of, adwc3_of_match);

static struct platform_driver adwc3_driver = {
	.probe		= axxia_dwc3_probe,
	.remove		= axxia_dwc3_remove,
	.driver		= {
		.name	= "axxia-dwc3",
		.of_match_table	= adwc3_of_match,
	},
};

module_platform_driver(adwc3_driver);

MODULE_ALIAS("platform:axxia-dwc3");
MODULE_AUTHOR("Sangeetha Rao <sangeetha.rao@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Intel's Axxia Glue Layer");
