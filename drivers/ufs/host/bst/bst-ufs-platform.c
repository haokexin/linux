// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys G210 Test Chip driver
 *
 * Copyright (C) 2015-2016 Synopsys, Inc. (www.synopsys.com)
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * 
 * Authors: Joao Pinto <jpinto@synopsys.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <ufs/ufshcd.h>

#include "../ufshcd-pltfrm.h"

/**
 * ufshcd_dwc_link_startup_notify()
 * UFS Host DWC specific link startup sequence
 * @hba: private structure pointer
 * @status: Callback notify status
 *
 * Returns 0 on success, non-zero value on failure
 */

static int tc_dwc_ufs_init(struct ufs_hba *hba)
{
    return 0;
}

static struct ufs_hba_variant_ops tc_dwc_g210_40bit_pltfm_hba_vops = {
	.name                   = "tc-dwc-g210-pltfm",
	.init					= tc_dwc_ufs_init,
};

static const struct of_device_id tc_dwc_g210_pltfm_match[] = {
	{
		.compatible = "snps,g210-tc-6.00-40bit",
		.data = &tc_dwc_g210_40bit_pltfm_hba_vops,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tc_dwc_g210_pltfm_match);

/**
 * tc_dwc_g210_pltfm_probe()
 * @pdev: pointer to platform device structure
 *
 */
static int tc_dwc_g210_pltfm_probe(struct platform_device *pdev)
{
	int err;
	const struct of_device_id *of_id;
	struct ufs_hba_variant_ops *vops;
	struct device *dev = &pdev->dev;

//	dev_err(dev, "ufs init start !!!\n");

	of_id = of_match_node(tc_dwc_g210_pltfm_match, dev->of_node);
	if (of_id->data) {
		vops = (struct ufs_hba_variant_ops *)of_id->data;
	} else {
		dev_err(dev, "of_match_node() failed\n");
		return 1;
	}

	/* Perform generic probe */
	err = ufshcd_pltfrm_init(pdev, vops);
	if (err)
		dev_err(dev, "ufshcd_pltfrm_init() failed %d\n", err);

	return err;
}

/**
 * tc_dwc_g210_pltfm_remove()
 * @pdev: pointer to platform device structure
 *
 */
static int tc_dwc_g210_pltfm_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	pm_runtime_get_sync(&(pdev)->dev);
	ufshcd_remove(hba);

	return 0;
}

static const struct dev_pm_ops tc_dwc_g210_pltfm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ufshcd_system_suspend, ufshcd_system_resume)
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
};

static struct platform_driver tc_dwc_g210_pltfm_driver = {
	.probe		= tc_dwc_g210_pltfm_probe,
	.remove		= tc_dwc_g210_pltfm_remove,
	.shutdown = ufshcd_pltfrm_shutdown,
	.driver		= {
		.name	= "tc-dwc-g210-pltfm",
		.pm	= &tc_dwc_g210_pltfm_pm_ops,
		.of_match_table	= of_match_ptr(tc_dwc_g210_pltfm_match),
	},
};

module_platform_driver(tc_dwc_g210_pltfm_driver);

MODULE_ALIAS("platform:tc-dwc-g210-pltfm");
MODULE_DESCRIPTION("Synopsys Test Chip G210 platform glue driver");
MODULE_AUTHOR("Joao Pinto <Joao.Pinto@synopsys.com>");
MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("Dual BSD/GPL");
