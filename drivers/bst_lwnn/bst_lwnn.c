// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn:    Linux device driver for Black Sesame Technologies Light Weighted
 *              Neural Network Processor
 * @author:     AI Tools Team, BST Ltd.
 *
 * @file        bst_lwnn.c
 * @brief       This file is the top source code file of bst_lwnn driver. It
 *              contains function definitions of kernel driver interface.
 */

#include "bst_lwnn.h"

/******************************************************/

static int bst_lwnn_probe(struct platform_device *pdev);
static int bst_lwnn_remove(struct platform_device *pdev);
static void bst_lwnn_shutdown(struct platform_device *pdev);

// globle data define
static const struct of_device_id bst_lwnn_of_match[] = {
	{
		.compatible = "bst,bst_lwnn,cma",
	},
	{},
};

static struct platform_driver bst_lwnn_driver = {
	.probe = bst_lwnn_probe,
	.remove = bst_lwnn_remove,
	.shutdown = bst_lwnn_shutdown,
	.driver = {
		   .name = BST_LWNN_DRIVER_NAME,
		   .of_match_table = of_match_ptr(bst_lwnn_of_match),
		   },
};

int bst_lwnn_dspcnt = 4;
module_param(bst_lwnn_dspcnt, int, S_IRUGO);
MODULE_PARM_DESC(bst_lwnn_dspcnt, "start dsp count");

int bst_lwnn_msg_interface = 1;
int bst_lwnn_mem_usingsmmu = 1;
module_param(bst_lwnn_msg_interface, int, S_IRUGO);
module_param(bst_lwnn_mem_usingsmmu, int, S_IRUGO);
MODULE_PARM_DESC(bst_lwnn_msg_interface,
		 "bst_lwnn_msg_interface : 0 is ipc, 1 is msgbox(default)");
MODULE_PARM_DESC(bst_lwnn_mem_usingsmmu,
		 "bst_lwnn_mem_usingsmmu : 0 disable, 1 enable(default)");

char *msg_interface_name[2] = { "ipc", "msgbox" };

/*******************************************************************************
 * BST LWNNP Driver Interface
 ******************************************************************************/
/*!
 * @brief       This is the probe callback function of bst_lwnn driver.
 * @param[in]   pdev The pointer to the platform device structure
 * @return      0 - success
 *              Error code - failure
 */
extern struct device *dev_cvsmm;
static int bst_lwnn_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bst_lwnn *pbst_lwnn;
	struct device_node *node;
	struct device_node *node_iommu;
	const char *status;

	bst_lwnn_mem_usingsmmu = 0;
	if (dev_cvsmm) {
		node = dev_of_node(dev_cvsmm);
		if (!node) {
			BST_LWNN_DEV_ERR(dev_cvsmm,
					 "no device tree node found");
			return -ENODEV;
		}

		node_iommu = of_parse_phandle(node, "iommus", 0);
		if (node_iommu) {
			status = of_get_property(node_iommu, "status", NULL);
			if (!status || strcmp(status, "okay") == 0) {
				bst_lwnn_mem_usingsmmu = 1;
			}
			of_node_put(node_iommu);
		}
	}

	if ((bst_lwnn_msg_interface != BST_LWNN_MSG_INTERFACE_IPC) &&
	    (bst_lwnn_msg_interface != BST_LWNN_MSG_INTERFACE_MSGBOX)) {
		BST_LWNN_STAGE_PRINTK(
			"bst_lwnn msg interface is error, it should be %d or %d",
			BST_LWNN_MSG_INTERFACE_IPC,
			BST_LWNN_MSG_INTERFACE_MSGBOX);
		return -EINVAL;
	}

	BST_LWNN_STAGE_PRINTK(
		"bst_lwnn driver is initializing, dsp number is %d, msg interface is %s ...",
		bst_lwnn_dspcnt, msg_interface_name[bst_lwnn_msg_interface]);

	BST_LWNN_STAGE_PRINTK(
		"BST_LWNN driver is initializing, bst_lwnn_mem_usingsmmu %d...",
		bst_lwnn_mem_usingsmmu);

	if ((bst_lwnn_dspcnt < 1) || (bst_lwnn_dspcnt > BST_LWNN_MAX_DSPNUM)) {
		BST_LWNN_STAGE_PRINTK(
			"the dsp number of bst_lwnn should be between 1 and %d",
			BST_LWNN_MAX_DSPNUM);
		return -EINVAL;
	}

	pbst_lwnn = devm_kzalloc(&pdev->dev, sizeof(*pbst_lwnn), GFP_KERNEL);
	if (pbst_lwnn == NULL) {
		return -ENOMEM;
	}
	//init bst_lwnn device
	pbst_lwnn->pdev = pdev;
	platform_set_drvdata(pdev, pbst_lwnn);
	mutex_init(&pbst_lwnn->mutex);

	//init sysfile
	ret = bst_lwnn_sysfile_init(pbst_lwnn);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "bst_lwnn_sysfile_init failed, ret %d", ret);
		goto err_sysfile_init;
	}
	BST_LWNN_STAGE_PRINTK("bst_sysfile_init OK, /sys/kernel/%s registered",
			      BST_LWNN_DRIVER_NAME);

	//init bst_lwnn memory manager
	pbst_lwnn->mem_manager.enable_smmu =
		(bst_lwnn_mem_usingsmmu == 1) ? true : false;
	ret = bst_lwnn_mem_manager_init(pbst_lwnn);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "bst_lwnn_mem_manager_init failed, ret %d",
				 ret);
		goto err_mem_manager;
	}
	BST_LWNN_STAGE_PRINTK("bst_lwnn_mem_manager_init OK");

	//init bst_lwnn firmware manager
	ret = bst_lwnn_fw_manager_init(pbst_lwnn);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "bst_lwnn_fw_manager_init all failed");
		goto err_fw_manager;
	}
	BST_LWNN_STAGE_PRINTK("bst_lwnn_fw_manager_init OK");

	//init misc device
	ret = bst_lwnn_miscdev_init(pbst_lwnn);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "bst_lwnn_misc_init failed, ret %d", ret);
		goto err_misc_init;
	}
	BST_LWNN_STAGE_PRINTK("bst_lwnn_misc_init OK, /dev/%s registered",
			      pbst_lwnn->miscdev.name);

	// init some msg manager variables
	bst_lwnn_msg_manager_probe_init(pbst_lwnn);

err_misc_init:
	bst_lwnn_fw_manager_cleanup(pbst_lwnn);
	if (ret == 0) {
		pbst_lwnn->state = BST_LWNN_INIT;
		BST_LWNN_STAGE_PRINTK("%s", "bst_lwnn probe completed!");
		return 0;
	}
err_fw_manager:
	bst_lwnn_mem_manager_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_mem_manager_exit OK");
err_mem_manager:
	bst_lwnn_sysfile_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_sysfile_exit OK");
err_sysfile_init:
	devm_kfree(&pdev->dev, pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("%s", "probe exit");
	return ret;
}

/*!
 * @brief       This is the remove callback function of bst_lwnn driver.
 * @param[in]   pdev The pointer to the platform device structure
 * @return      0
 */
static int bst_lwnn_remove(struct platform_device *pdev)
{
	struct bst_lwnn *pbst_lwnn = platform_get_drvdata(pdev);

	//BST_LWNN_STAGE_PRINTK("%s", "remove enter");
	BST_LWNN_STAGE_PRINTK("%s", "remove enter");

	bst_lwnn_miscdev_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_misc_exit OK");
	bst_lwnn_sysfile_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_sysfile_exit OK");
	bst_lwnn_fw_rt_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_fw_rt_exit OK");
	bst_lwnn_msg_manager_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_msg_manager_exit OK");
	bst_lwnn_fw_manager_exit(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("bst_lwnn_fw_manager_exit OK");
	if (bst_lwnn_check_online(pbst_lwnn)) {
		bst_lwnn_mem_manager_exit(pbst_lwnn);
		BST_LWNN_STAGE_PRINTK("bst_lwnn_mem_manager_exit OK");
	}
	mutex_destroy(&pbst_lwnn->mutex);
	devm_kfree(&pdev->dev, pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("%s", "remove completed");
	return 0;
}

/*!
 * @brief       This is the shutdown callback function of bst_lwnn driver. (just
                for personal exploration)
 * @return      Void
 */
static void bst_lwnn_shutdown(struct platform_device *pdev)
{
	int i;
	struct bst_lwnn *pbst_lwnn = platform_get_drvdata(pdev);
	for (i = 0; i < pbst_lwnn->dsp_num; ++i) {
		pbst_lwnn->dsp_online[i] = 0;
		pbst_lwnn->fw_manager.dsps[i].boot = 1;
	}
	bst_lwnn_fw_rt_cleanup(pbst_lwnn);
	BST_LWNN_STAGE_PRINTK("release lwnn dsp 0 ~ %d",
			      pbst_lwnn->dsp_num - 1);
	BST_LWNN_STAGE_PRINTK("lwnn shutdown");
	return;
}

// register the bst_lwnn driver on platform bus
/*!
 * @brief       This is the init function of bst_lwnn driver.
 * @return      0 - success
 *              Error ode - failure
 */
static int __init bst_lwnn_driver_init(void)
{
	BST_LWNN_STAGE_PRINTK("LWNN version:%s,%s", _GIT_MSG_, _GIT_DATE_);
	BST_LWNN_STAGE_PRINTK("LWNN build:  %s,%s", __DATE__, __TIME__);

	return platform_driver_register(&bst_lwnn_driver);
}

/*!
 * @brief       This is the exit function of bst_lwnn driver.
 * @return      Void
 */
static void __exit bst_lwnn_driver_exit(void)
{
	platform_driver_unregister(&bst_lwnn_driver);
	return;
}

module_init(bst_lwnn_driver_init);
module_exit(bst_lwnn_driver_exit);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION(
	"BST_LWNN: Linux device driver for Black Sesame Technologies Data Processing IP");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
