// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn.c
 * @brief   This file is the top source code file of BSTN driver. It contains
 *          function definitions of driver setup and interface.
 */

#include "bstn.h"
/******************************************************/

static int bstn_probe(struct platform_device *pdev);
static int bstn_remove(struct platform_device *pdev);
static void bstn_shutdown(struct platform_device *pdev);

// globle data define
static const struct of_device_id bstn_of_match[] = {
	{
		.compatible = "bst,bstn",
	},
	{},
};

static struct platform_driver bstn_driver = {
	.probe = bstn_probe,
	.remove = bstn_remove,
	.shutdown = bstn_shutdown,
.driver = {
		   .name = BSTN_DRIVER_NAME,
		   .of_match_table = of_match_ptr(bstn_of_match),
		   },
};

int bstn_msg_interface =
	1; /* 0: BSTN_MSG_INTERFACE_IPC, 1: BSTN_MSG_INTERFACE_MSGBOX */
int bstn_mem_usingsmmu =
	0; /* 0: disable smmu, 1: enable smmu (status same with dtb) */
module_param(bstn_msg_interface, int, S_IRUGO);
module_param(bstn_mem_usingsmmu, int, S_IRUGO);
MODULE_PARM_DESC(bstn_msg_interface,
		 "msg interface use 0(ipc) 1(msgbox,default)");
MODULE_PARM_DESC(bstn_mem_usingsmmu,
		 "smmu status: 0(disable), 1(enable,default)");

/*******************************************************************************
 * BSTN Driver Interface
 ******************************************************************************/
/*
 * @func    bstn_probe
 * @brief   This is the probe callback function of BSTN driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0 for success and error code otherwise
 */
static int bstn_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int id = 0;

	struct bstn_device *pbstn;
	struct device_node *node;
	struct device_node *node_iommu;
	const char *status;

	bstn_mem_usingsmmu = 0;
	node = dev_of_node(&pdev->dev);
	if (!node) {
		BSTN_DEV_ERR(&pdev->dev, "no device tree node found");
		return -ENODEV;
	}
	node_iommu = of_parse_phandle(node, "iommus", 0);
	if (node_iommu) {
		status = of_get_property(node_iommu, "status", NULL);
		if (!status || strcmp(status, "okay") == 0) {
			bstn_mem_usingsmmu = 1;
		}
		of_node_put(node_iommu);
	}

	BSTN_STAGE_PRINTK("BSTN driver initializing bstn_msg_interface %d ...",
			  bstn_msg_interface);
	BSTN_STAGE_PRINTK("BSTN driver initializing bstn_mem_usingsmmu %d ...",
			  bstn_mem_usingsmmu);
	BSTN_STAGE_PRINTK("timeout_jiffies: %d, timeout_ms %d",
			  BSTN_RSP_TIMEOUT_JIFFIES, BSTN_RSP_TIMEOUT_MS);

	//get device id from the device tree
	ret = device_property_read_u32_array(&pdev->dev, "id", &id, 1);
	if (ret < 0) {
		BSTN_DEV_ERR(&pdev->dev, "no id property, ret %d", ret);
		return ret;
	}

	pbstn = devm_kzalloc(&pdev->dev, sizeof(struct bstn_device),
			     GFP_KERNEL);
	if (pbstn == NULL) {
		return -ENOMEM;
	}
	//init bstn device
	pbstn->pdev = pdev;
	platform_set_drvdata(pdev, pbstn);
	mutex_init(&pbstn->mutex);

	//init bstn memory manager
	pbstn->mem_manager.enable_smmu = (bstn_mem_usingsmmu == 1) ? true :
								     false;
	ret = bstn_mem_manager_init(pbstn);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "bstn_mem_manager_init failed, ret %d", ret);
		goto err_mem_manager;
	}
	BSTN_STAGE_PRINTK("bstn_mem_manager_init OK");

	//init bstn firmware manager
	ret = bstn_fw_manager_init(pbstn);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "bstn_fw_manager_init failed, ret %d", ret);
		goto err_fw_manager;
	}
	BSTN_STAGE_PRINTK("bstn_fw_manager_init OK");

	//init sysfs files
	bstn_sysfile_init(pbstn);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "bstn_sysfile_init failed, ret %d", ret);
		goto err_sysfile_init;
	}
	BSTN_STAGE_PRINTK("bstn_sysfile_init OK");

	//init misc device
	ret = bstn_misc_init(pbstn);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "bstn_misc_init failed, ret %d",
			     ret);
		goto err_misc_init;
	}
	BSTN_STAGE_PRINTK("bstn_misc_init OK, device[%s] registered",
			  pbstn->miscdev.name);

	pbstn->msg_manager.ipc_session_id = -1;
	pbstn->msg_manager.msgbx_client = NULL;
	pbstn->msg_manager.req_bufs = NULL;
	pbstn->msg_manager.msg_receiver_task = NULL;
	pbstn->msg_manager.msg_sw_bister_task = NULL;

	pbstn->state = BSTN_INIT;
	BSTN_STAGE_PRINTK("BSTN v%d.%d.%d probe completed", BSTN_VER_MAJOR,
			  BSTN_VER_MINOR, BSTN_VER_PATCH);
	return ret;

err_misc_init:
	bstn_sysfile_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_msg_manager_exit OK");
err_sysfile_init:
	bstn_fw_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_fw_manager_exit OK");
err_fw_manager:
	bstn_mem_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_mem_manager_exit OK");
err_mem_manager:
	devm_kfree(&pdev->dev, pbstn);
	BSTN_STAGE_PRINTK("BSTN v%d.%d.%d probe exit", BSTN_VER_MAJOR,
			  BSTN_VER_MINOR, BSTN_VER_PATCH);
	return ret;
}

/*
 * @func    bstn_remove
 * @brief   This is the remove callback function of BSTN driver. However, it is
 *          never called when BSTN is built as a built-in driver.
 * @params  pdev - the pointer to the platform device structure
 * @return  0
 */
static int bstn_remove(struct platform_device *pdev)
{
	struct bstn_device *pbstn = platform_get_drvdata(pdev);

	BSTN_STAGE_PRINTK("remove enter");

	bstn_fw_rt_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_fw_rt_exit OK");
	bstn_msg_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_msg_manager_exit OK");
	bstn_misc_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_misc_exit OK");
	bstn_sysfile_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_sysfile_exit OK");
	bstn_fw_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_fw_manager_exit OK");
	bstn_mem_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_mem_manager_exit OK");

	mutex_destroy(&pbstn->mutex);
	devm_kfree(&pdev->dev, pbstn);
	BSTN_STAGE_PRINTK("%s", "remove completed");
	return 0;
}

/*!
 * @brief       This is the shutdown callback function of bstn driver. (just
                for personal exploration)
 * @return      Void
 */
static void bstn_shutdown(struct platform_device *pdev)
{
	int ret = bstn_remove(pdev);
	BSTN_STAGE_PRINTK("bstn shutdown:%d", ret);
	return;
}

// register the BSTN driver on platform bus
static int __init bstn_driver_init(void)
{
	BSTN_STAGE_PRINTK("BSTN version:%s,%s", _GIT_MSG_, _GIT_DATE_);
	BSTN_STAGE_PRINTK("BSTN build:  %s,%s", __DATE__, __TIME__);

	return (platform_driver_register(&bstn_driver));
}

// initialize the BSTN driver after the BST_IPC driver which is in device_initcall_sync

static void __exit bstn_driver_exit(void)
{
	platform_driver_unregister(&bstn_driver);
	return;
}

module_init(bstn_driver_init);
module_exit(bstn_driver_exit);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION(
	"BSTN: Linux device driver for Black Sesame Technologies Neural Network IP");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
