// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include "bst_hwcv_main.h"
#include "bst_hwcv_gwarp.h"
#include "bst_hwcv_scaler.h"
#include "bst_hwcv_common.h"

#define CV_SYS_CTRL_STATUS 0x51000000
#define GWARP0_CLK_EN_BIT 12
#define GWARP1_CLK_EN_BIT 13
#define SCLR_CLK_EN_BIT 14
#define DMA_CLK_EN_BIT 15
#define SOFT_RST_GWARP0_BIT 16
#define SOFT_RST_GWARP1_BIT 17
#define SOFT_RST_SCLR_BIT 18
#define SOFT_RST_DMA_BIT 19

#define CV_PARITY_CTRL 0x51000050
#define CV_INTERNAL_ECC_EN 0
#define CV_INTERNAL_PTY_EN 1

#define CV_INTR_EN 0x510000D4
#define CV_SCALER_FUNC_INTR_EN 4
#define CV_GWARP0_FUNC_INTR_EN 5
#define CV_GWARP1_FUNC_INTR_EN 6
#define CV_FUNC_INTR_OUTPUT_EN 29

static int bst_hwcv_clk_enable(struct device *dev)
{
	uint32_t reg;
	virt_addr_t cv_sys_ctrl_status;

	cv_sys_ctrl_status = bst_hwcv_map_reg(dev, CV_SYS_CTRL_STATUS);
	if (!cv_sys_ctrl_status) {
		dev_err(dev, "Failed to map reg[0x%08x]", CV_SYS_CTRL_STATUS);
		return -ENOMEM;
	}

	reg = readl_relaxed(cv_sys_ctrl_status);
	reg |= 1UL << GWARP0_CLK_EN_BIT;
	reg |= 1UL << GWARP1_CLK_EN_BIT;
	reg |= 1UL << SCLR_CLK_EN_BIT;
	reg |= 1UL << DMA_CLK_EN_BIT;
	reg |= 1UL << SOFT_RST_GWARP0_BIT;
	reg |= 1UL << SOFT_RST_GWARP1_BIT;
	reg |= 1UL << SOFT_RST_SCLR_BIT;
	reg |= 1UL << SOFT_RST_DMA_BIT;
	writel_relaxed(reg, cv_sys_ctrl_status);

	dev_dbg(dev, "CV_SYS_CTRL_STATUS: 0x%08x:0x%08x", CV_SYS_CTRL_STATUS,
		readl_relaxed(cv_sys_ctrl_status));
	bst_hwcv_unmap_reg(dev, cv_sys_ctrl_status);

	return 0;
}

static int bst_hwcv_ecc_pty_enable(struct device *dev, int status)
{
	uint32_t reg;
	virt_addr_t cv_parity_ctrl;

	cv_parity_ctrl = bst_hwcv_map_reg(dev, CV_PARITY_CTRL);
	if (!cv_parity_ctrl) {
		dev_err(dev, "Failed to map reg[0x%08x]", CV_PARITY_CTRL);
		return -ENOMEM;
	}

	reg = readl_relaxed(cv_parity_ctrl);
	if (status)
		reg |= 0x3;
	else
		reg &= ~0x3;

	writel_relaxed(reg, cv_parity_ctrl);

	dev_dbg(dev, "CV_PARITY_CTRL: 0x%08x:0x%08x", CV_PARITY_CTRL,
		readl_relaxed(cv_parity_ctrl));
	bst_hwcv_unmap_reg(dev, cv_parity_ctrl);

	return 0;
}

static int bst_hwcv_int_enable(struct device *dev)
{
	uint32_t reg;
	virt_addr_t cv_intr_en;

	cv_intr_en = bst_hwcv_map_reg(dev, CV_INTR_EN);
	if (!cv_intr_en) {
		dev_err(dev, "Failed to map reg[0x%08x]", CV_INTR_EN);
		return -ENOMEM;
	}

	reg = readl_relaxed(cv_intr_en);
	reg |= 1UL << CV_SCALER_FUNC_INTR_EN;
	reg |= 1UL << CV_GWARP0_FUNC_INTR_EN;
	reg |= 1UL << CV_GWARP1_FUNC_INTR_EN;
	reg |= 1UL << CV_FUNC_INTR_OUTPUT_EN;
	writel_relaxed(reg, cv_intr_en);

	dev_dbg(dev, "CV_INTR_EN: 0x%08x:0x%08x", CV_INTR_EN,
		readl_relaxed(cv_intr_en));
	bst_hwcv_unmap_reg(dev, cv_intr_en);

	return 0;
}

static int bst_hwcv_preinit(struct device *dev)
{
	int ret;

	ret = bst_hwcv_clk_enable(dev);
	if (ret < 0)
		return ret;

	ret = bst_hwcv_ecc_pty_enable(dev, 0);
	if (ret < 0)
		return ret;

	ret = bst_hwcv_int_enable(dev);
	if (ret < 0)
		return ret;

	ret = bst_gwarp_map_all_regs(dev);
	if (ret < 0)
		return ret;

	ret = bst_scaler_map_all_regs(dev);
	if (ret < 0)
		return ret;

	return 0;
}

/*----------------------------------------------------------------------*/

static int bst_hwcv_probe(struct platform_device *pdev)
{
	int ret;
	struct bst_hwcv_dev *cv_dev;
	struct device *dev = &pdev->dev;

	dev_info(dev, "Hwcv probe.");
	cv_dev = devm_kzalloc(dev, sizeof(struct bst_hwcv_dev), GFP_KERNEL);
	if (!cv_dev)
		return -ENOMEM;
	cv_dev->pdev = pdev;
	cv_dev->dev = dev;

	ret = bst_hwcv_miscdev_init(dev, &cv_dev->misc_dev);
	if (ret < 0) {
		devm_kfree(dev, cv_dev);
		return ret;
	}

	ret = bst_hwcv_preinit(dev);
	if (ret < 0) {
		devm_kfree(dev, cv_dev);
		bst_hwcv_miscdev_exit(&cv_dev->misc_dev);
		return ret;
	}

	platform_set_drvdata(pdev, cv_dev);

	return 0;
}

static int bst_hwcv_remove(struct platform_device *pdev)
{
	struct bst_hwcv_dev *cv_dev;
	struct device *dev = &pdev->dev;

	cv_dev = platform_get_drvdata(pdev);
	bst_hwcv_miscdev_exit(&cv_dev->misc_dev);
	dev_info(dev, "Hwcv exit");

	return 0;
}

static const struct of_device_id bst_hwcv_of_match[] = {
	{
		.compatible = "bst,c1200-hwcv",
	},
	{},
};

static struct platform_driver bst_hwcv_driver = {
	.probe   = bst_hwcv_probe,
	.remove  = bst_hwcv_remove,
	.driver  = {
		.name = BST_HWCV_DRIVER_NAME,
		.of_match_table = of_match_ptr(bst_hwcv_of_match),
	},
};

static int __init bst_hwcv_driver_init(void)
{
	return platform_driver_register(&bst_hwcv_driver);
}

late_initcall(bst_hwcv_driver_init);

static void __exit bst_hwcv_driver_exit(void)
{
	platform_driver_unregister(&bst_hwcv_driver);
}

module_exit(bst_hwcv_driver_exit);

MODULE_IMPORT_NS(DMA_BUF);
MODULE_VERSION(HWCV_DRIVER_VERSION);
MODULE_DESCRIPTION("BST C1200 HWCV driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
