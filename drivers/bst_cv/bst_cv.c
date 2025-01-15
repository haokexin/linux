/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP
 * author: AI Tools Team, BST Ltd.
 *
 * @file    bst_cv.c
 * @brief   This file is the top source code file of bst_cv driver. It contains
 *          function definitions of driver setup and interface.
 */

#include "bst_cv.h"
#include "bst_cv_wdt.h"

/******************************************************/

static int bst_cv_probe(struct platform_device *pdev);
static int bst_cv_remove(struct platform_device *pdev);
static void bst_cv_shutdown(struct platform_device *pdev);

// globle data define
static const struct of_device_id bst_cv_of_match[] = {
	{.compatible = "bst,bst_cv,cma",},
	{},
};

static struct platform_driver bst_cv_driver = {
	.probe = bst_cv_probe,
	.remove = bst_cv_remove,
	.shutdown = bst_cv_shutdown,
	.driver = {
		   .name = BST_CV_DRIVER_NAME,
		   .of_match_table = of_match_ptr(bst_cv_of_match),
		   },
};

int bst_cv_dspcnt = 2;
module_param(bst_cv_dspcnt, int, S_IRUGO);
MODULE_PARM_DESC(bst_cv_dspcnt, "start dsp count");

int bst_cv_mem_usingsmmu = 1;
module_param(bst_cv_mem_usingsmmu, int, S_IRUGO);
MODULE_PARM_DESC(bst_cv_mem_usingsmmu,
	"bst_cv_mem_usingsmmu : 0 disable, 1 enable(default)");
/*******************************************************************************
 * BST CV Driver Interface
 ******************************************************************************/
/*
 * @func    bst_cv_probe
 * @brief   This is the probe callback function of bst_cv driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0 for success and error code otherwise
 */
extern struct device *dev_cvsmm;
static int bst_cv_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bst_cv *pbst_cv;
	struct device_node *node;
	struct device_node *node_iommu;
	const char *status;

	bst_cv_mem_usingsmmu = 0;
	if (dev_cvsmm) {
		node = dev_of_node(dev_cvsmm);
		if (!node) {
			BST_CV_DEV_ERR(dev_cvsmm, "no device tree node found");
			return -ENODEV;
		}
		node_iommu = of_parse_phandle(node, "iommus", 0);
		if (node_iommu) {
			status = of_get_property(node_iommu, "status", NULL);
			if (!status || strcmp(status, "okay") == 0) {
				bst_cv_mem_usingsmmu = 1;
			}
			of_node_put(node_iommu);
		}
	}

	BST_CV_STAGE_PRINTK("BST_CV ko driver initializing dspcnt %d ...",
			    bst_cv_dspcnt);
	BST_CV_STAGE_PRINTK("BST_CV ko driver initializing usingsmmu %d ...",
			    bst_cv_mem_usingsmmu);

	if (bst_cv_dspcnt <= 0) {
		return -EINVAL;
	}

	pbst_cv = devm_kzalloc(&pdev->dev, sizeof(*pbst_cv), GFP_KERNEL);
	if (pbst_cv == NULL) {
		return -ENOMEM;
	}
	//init bst_cv device
	pbst_cv->pdev = pdev;
	platform_set_drvdata(pdev, pbst_cv);
	mutex_init(&pbst_cv->mutex);

	//init sysfile
	ret = bst_cv_sysfile_init(pbst_cv);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "bst_cv_sysfile_init failed, ret %d", ret);
		goto err_sysfile_init;
	}
	BST_CV_STAGE_PRINTK("bst_sysfile_init OK");

	//init bst_cv memory manager
	pbst_cv->mem_manager.enable_smmu =
		(bst_cv_mem_usingsmmu == 1) ? true : false;
	ret = bst_cv_mem_manager_init(pbst_cv);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "bst_cv_mem_manager_init failed, ret %d", ret);
		goto err_mem_manager;
	}
	BST_CV_STAGE_PRINTK("bst_cv_mem_manager_init OK");

	//init bst_cv firmware manager
	ret = bst_cv_fw_manager_init(pbst_cv);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "bst_cv_fw_manager_init all failed");
		goto err_fw_manager;
	}
	BST_CV_STAGE_PRINTK("bst_cv_fw_manager_init OK");

	//init misc device
	ret = bst_cv_miscdev_init(pbst_cv);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "bst_cv_misc_init failed, ret %d", ret);
		goto err_miscdev_init;
	}
	BST_CV_STAGE_PRINTK("bst_cv_misc_init OK, /dev/%s registered",
			    pbst_cv->miscdev.name);

err_miscdev_init:
	bst_cv_fw_manager_cleanup(pbst_cv);
	if (ret == 0) {
		pbst_cv->state = BST_CV_INIT;
		BST_CV_STAGE_PRINTK("%s", "bst_cv probe completed!");
		return 0;
	}
err_fw_manager:
	bst_cv_mem_manager_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_mem_manager_exit OK");
err_mem_manager:
	bst_cv_sysfile_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_sysfile_exit OK");
err_sysfile_init:
	devm_kfree(&pdev->dev, pbst_cv);
	BST_CV_STAGE_PRINTK("%s", "probe exit");
	return ret;
}

/*
 * @func    bst_cv_remove
 * @brief   This is the remove callback function of bst_cv driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0
 */
static int bst_cv_remove(struct platform_device *pdev)
{
	struct bst_cv *pbst_cv = platform_get_drvdata(pdev);

	BST_CV_STAGE_PRINTK("%s", "remove enter");

	bst_cv_miscdev_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_misc_exit OK");
	bst_cv_sysfile_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_sysfile_exit OK");
	bst_cv_fw_rt_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_fw_rt_exit OK");
	bst_cv_msg_manager_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_msg_manager_exit OK");
	bst_cv_fw_manager_exit(pbst_cv);
	BST_CV_STAGE_PRINTK("bst_cv_fw_manager_exit OK");
	if (bst_cv_check_online(pbst_cv)) {
		bst_cv_mem_manager_exit(pbst_cv);
		BST_CV_STAGE_PRINTK("bst_cv_mem_manager_exit OK");
	}
	// wdt_cv_release();
	mutex_destroy(&pbst_cv->mutex);
	devm_kfree(&pdev->dev, pbst_cv);

	BST_CV_STAGE_PRINTK("%s", "remove completed");
	return 0;
}

static void bst_cv_shutdown(struct platform_device *pdev)
{
	BST_CV_STAGE_PRINTK("%s", "this is a shutdown test");
	return;
}

// register the BST_CV driver on platform bus
static int __init bst_cv_driver_init(void)
{
	return platform_driver_register(&bst_cv_driver);
}

// initialize the BST_CV driver after the BST_IPC driver which is in device_initcall_sync
late_initcall(bst_cv_driver_init);

static void __exit bst_cv_driver_exit(void)
{
	platform_driver_unregister(&bst_cv_driver);
	return;
}

module_exit(bst_cv_driver_exit);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION
    ("bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
