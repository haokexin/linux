// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/platform_device.h>
#include <linux/dma-direct.h>
#include <linux/module.h>
#include <linux/mman.h>
#include <linux/of.h>
#include <linux/io.h>
#include "bst_cvsmm.h"

int has_cv_dsp2_iommu_map;
EXPORT_SYMBOL(has_cv_dsp2_iommu_map);

int has_cv_dsp3_iommu_map;
EXPORT_SYMBOL(has_cv_dsp3_iommu_map);

static int bst_cvsmm_probe(struct platform_device *pdev);
static int bst_cvsmm_remove(struct platform_device *pdev);

struct device *dev_cvsmm;
EXPORT_SYMBOL_GPL(dev_cvsmm);

struct bst_cvsmm {
	struct platform_device *pdev;
};

// globle data define
static const struct of_device_id bst_cvsmm_of_match[] = {
	{
		.compatible = "bst,bst-cvsmm",
	},
	{},
};

static struct platform_driver bst_cvsmm_driver = {
	.probe  = bst_cvsmm_probe,
	.remove = bst_cvsmm_remove,
	.driver = {
		.name = BST_CVSMM_DRIVER_NAME,
		.of_match_table = of_match_ptr(bst_cvsmm_of_match),
		},
};

static void __init bst_cvshareram_init(struct device *dev)
{
	u32 val;
	void __iomem *csr_dsp_base;
	void __iomem *csr_cirbuf0, *csr_cirbuf1, *csr_cirbuf2, *csr_cirbuf3;

	csr_dsp_base = devm_ioremap(dev, 0x51000000, 4);
	csr_cirbuf0 = devm_ioremap(dev, 0x51038100, 4);
	csr_cirbuf1 = devm_ioremap(dev, 0x51038400, 4);
	csr_cirbuf2 = devm_ioremap(dev, 0x51039100, 4);
	csr_cirbuf3 = devm_ioremap(dev, 0x51039400, 4);

	val = ioread32(csr_dsp_base);
	iowrite32((val | 0x30), csr_dsp_base);
	val = ioread32(csr_dsp_base);
	iowrite32((val | 0xc0), csr_dsp_base);

	val = ioread32(csr_cirbuf0);
	iowrite32((val | 0x20), csr_cirbuf0);

	val = ioread32(csr_cirbuf1);
	iowrite32((val | 0x20), csr_cirbuf1);

	val = ioread32(csr_cirbuf2);
	iowrite32((val | 0x20), csr_cirbuf2);

	val = ioread32(csr_cirbuf3);
	iowrite32((val | 0x20), csr_cirbuf3);

	devm_iounmap(dev, csr_dsp_base);
	devm_iounmap(dev, csr_cirbuf0);
	devm_iounmap(dev, csr_cirbuf1);
	devm_iounmap(dev, csr_cirbuf2);
	devm_iounmap(dev, csr_cirbuf3);

	pr_info("cvshareram_init success\n");
}

static void __init bst_cv_freq_monitor_en(struct device *dev)
{
	u32 val;
	void __iomem *csr_freq_monitor_CTRL0;
	void __iomem *csr_freq_monitor_CTRL1;

	csr_freq_monitor_CTRL0 = devm_ioremap(dev, 0x51000110, 4);
	csr_freq_monitor_CTRL1 = devm_ioremap(dev, 0x51000114, 4);

	val = ioread32(csr_freq_monitor_CTRL0);
	iowrite32((val | (56700 << 12)), csr_freq_monitor_CTRL0);
	val = ioread32(csr_freq_monitor_CTRL0);
	iowrite32((val | (63 << 4)), csr_freq_monitor_CTRL0);

	val = ioread32(csr_freq_monitor_CTRL1);
	iowrite32((val | 17), csr_freq_monitor_CTRL1);

	val = ioread32(csr_freq_monitor_CTRL0);
	iowrite32((val | 1), csr_freq_monitor_CTRL0);

	devm_iounmap(dev, csr_freq_monitor_CTRL0);
	devm_iounmap(dev, csr_freq_monitor_CTRL1);

	pr_info("%s success\n", __func__);
}

static int bst_cvsmm_probe(struct platform_device *pdev)
{
	int ret;
	struct bst_cvsmm *pbst_cvsmm;

	dev_cvsmm = &pdev->dev;
	pr_info("cvsmm device probe");

	ret = dma_set_mask_and_coherent(dev_cvsmm, DMA_BIT_MASK(32));
	if (ret) {
		pr_err("cvsmm dma_set_mask_and_coherent fail, ret %d", ret);
		return -ENODEV;
	}
	pr_info("cvsmm dma_set_mask_and_coherent OK.");

	pbst_cvsmm = devm_kzalloc(dev_cvsmm, sizeof(*pbst_cvsmm), GFP_KERNEL);
	if (pbst_cvsmm == NULL)
		return -ENOMEM;

	pbst_cvsmm->pdev = pdev;
	platform_set_drvdata(pdev, pbst_cvsmm);

	bst_cvshareram_init(dev_cvsmm);
	bst_cv_freq_monitor_en(dev_cvsmm);

	return 0;
}

/*!
 * @brief       This is the remove callback function of bst_cvsmm driver.
 * @param[in]   pdev The pointer to the platform device structure
 * @return      0
 */
static int bst_cvsmm_remove(struct platform_device *pdev)
{
	struct bst_cvsmm *pbst_cvsmm;

	pbst_cvsmm = platform_get_drvdata(pdev);
	devm_kfree(dev_cvsmm, pbst_cvsmm);
	dev_cvsmm = NULL;
	pr_info("cvsmm remove");
	return 0;
}

// register the bst_cvsmm driver on platform bus
/*!
 * @brief       This is the init function of bst_cvsmm driver.
 * @return      0 - success
 *              Error ode - failure
 */
static int __init bst_cvsmm_driver_init(void)
{
	return platform_driver_register(&bst_cvsmm_driver);
}

/*!
 * @brief       This is the exit function of bst_cvsmm driver.
 * @return      Void
 */
static void __exit bst_cvsmm_driver_exit(void)
{
	platform_driver_unregister(&bst_cvsmm_driver);
}

module_init(bst_cvsmm_driver_init);
module_exit(bst_cvsmm_driver_exit);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION(
	"BST_CVSMM: Linux device driver for Black Sesame Technologies Data Processing IP");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
