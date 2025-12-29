// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 * Copyright (C) 2007-2011  STMicroelectronics Ltd
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 */

#include <linux/of.h>
#include "bstvmac.h"

extern int bstvmac_get_platform_resources(struct platform_device *pdev,
				   struct bstvmac_resources *bstvmac_res);

static int bstvmac_pre_init(struct plat_vmacenet_data *plat_dat)
{
	return 0;
}

static int bstvmac_probe(struct platform_device *pdev)
{
	struct plat_vmacenet_data *plat_dat = NULL;
	struct bstvmac_resources bstvmac_res;
	int ret = 0;

	ret = bstvmac_get_platform_resources(pdev, &bstvmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = bstvmac_probe_config_dt(pdev, bstvmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}
	}

	ret = bstvmac_pre_init(plat_dat);
	if (ret < 0) {
		dev_err(&pdev->dev, "vmac pre-init failed\n");
		goto err_exit;
	}

	/* Custom initialisation (if needed) */
	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret) {
			goto err_exit;
		}
	}

	ret = bstvmac_dvr_probe(pdev, plat_dat, &bstvmac_res);
	if (ret) {
		goto err_exit;
	}

	return 0;

err_exit:
	if (plat_dat->exit)
		plat_dat->exit(pdev, plat_dat->bsp_priv);

	return ret;
}

static const struct of_device_id bst_vmac_match[] = {
	{ .compatible = "bst,vmac-hif"},
	{ }
};
MODULE_DEVICE_TABLE(of, bst_vmac_match);

static struct platform_driver vmac_generic_driver = {
	.probe  = bstvmac_probe,
	.remove = bstvmac_pltfr_remove,
	.shutdown = bstvmac_pltfr_shutdown,
	.driver = {
		.name	= BSTVMAC_RESOURCE_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm		= &bstvmac_pltfr_pm_ops,
#endif
		.of_match_table = of_match_ptr(bst_vmac_match),
	},
};
module_platform_driver(vmac_generic_driver);

MODULE_DESCRIPTION("Bst Vmac Driver");
MODULE_LICENSE("GPL v2");
