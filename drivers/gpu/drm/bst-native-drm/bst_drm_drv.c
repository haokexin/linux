// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_of.h>
#include "bst_drm_dev.h"
#include "bst_drm_kms.h"
#include "bst-dc/dc_dev.h"
#include "bst-dc/dc_io.h"

struct bst_drv {
	struct bst_dev *mdev;
	struct bst_kms_dev *kms;
};

struct bst_dev *dev_to_mdev(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);

	return mdrv ? mdrv->mdev : NULL;
}

static void bst_unbind(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);

	if (!mdrv)
		return;

	bst_kms_detach(mdrv->kms);

	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);
	else
		bst_dev_suspend(mdrv->mdev);

	bst_dev_destroy(mdrv->mdev);

	dev_set_drvdata(dev, NULL);
	devm_kfree(dev, mdrv);
}

static int bst_bind(struct device *dev)
{
	struct bst_drv *mdrv;
	int err;

	mdrv = devm_kzalloc(dev, sizeof(*mdrv), GFP_KERNEL);
	if (!mdrv)
		return -ENOMEM;

	mdrv->mdev = bst_dev_create(dev);
	if (IS_ERR(mdrv->mdev)) {
		err = PTR_ERR(mdrv->mdev);
		devm_kfree(dev, mdrv);
		return -EPROBE_DEFER;
	}

	dev_set_drvdata(dev, mdrv);
	pm_runtime_enable(dev);
	if (!pm_runtime_enabled(dev))
		bst_dev_resume(mdrv->mdev);

	mdrv->kms = bst_kms_attach(mdrv->mdev);
	if (IS_ERR(mdrv->kms)) {
		err = PTR_ERR(mdrv->kms);
		goto destroy_mdev;
	}

	drm_fbdev_generic_setup(&mdrv->kms->base, 32);

	return 0;

destroy_mdev:
	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);
	else
		bst_dev_suspend(mdrv->mdev);

	bst_dev_destroy(mdrv->mdev);
	devm_kfree(dev, mdrv);
	return err;
}

static const struct component_master_ops bst_master_ops = {
	.bind	= bst_bind,
	.unbind	= bst_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	struct device_node *np = data;

	if (of_node_name_eq(np, "lvds_channel")) {
		np = of_get_parent(np);
		of_node_put(np);
	}
	return dev->of_node == np;
}

static void bst_add_slave(struct device *master,
			     struct component_match **match,
			     struct device_node *np,
			     u32 port, u32 endpoint)
{
	struct device_node *remote;

	remote = of_graph_get_remote_node(np, port, endpoint);
	if (remote) {
		drm_of_component_match_add(master, match, compare_of, remote);
		of_node_put(remote);
	}
}

static int bst_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *child;

	if (!dev->of_node)
		return -ENODEV;

	for_each_available_child_of_node(dev->of_node, child) {
		if (of_node_cmp(child->name, "pipeline") != 0)
			continue;

		bst_add_slave(dev, &match, child, BST_DRM_OF_PORT_OUTPUT, 0);
		bst_add_slave(dev, &match, child, BST_DRM_OF_PORT_OUTPUT, 1);
	}

	return component_master_add_with_match(dev, &bst_master_ops, match);
}

static int bst_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &bst_master_ops);
	return 0;
}

static const struct of_device_id bst_of_match[] = {
	{ .compatible = "bst,c1200-native-display0", .data = dc_identify_display_0, },
	{ .compatible = "bst,c1200-native-display1", .data = dc_identify_display_1, },
	{ .compatible = "bst,c1200-native-display2", .data = dc_identify_display_2, },
	{},
};

MODULE_DEVICE_TABLE(of, bst_of_match);

static int __maybe_unused bst_rt_pm_suspend(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);

	return bst_dev_suspend(mdrv->mdev);
}

static int __maybe_unused bst_rt_pm_resume(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);

	return bst_dev_resume(mdrv->mdev);
}

static int __maybe_unused bst_pm_suspend(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);
	int res;

	res = drm_mode_config_helper_suspend(&mdrv->kms->base);

	if (!pm_runtime_status_suspended(dev))
		bst_dev_suspend(mdrv->mdev);

	return res;
}

static int __maybe_unused bst_pm_resume(struct device *dev)
{
	struct bst_drv *mdrv = dev_get_drvdata(dev);
	struct dc_dev *dc = (struct dc_dev *)(mdrv->mdev->chip_data);
	struct block_header blk;
	u32 __iomem *blk_base;
	u32 i = 1;
	u32 offset = DC_BLOCK_SIZE;
	int err;

	mdrv->mdev->resume = true;

	if (!pm_runtime_status_suspended(dev))
		bst_dev_resume(mdrv->mdev);

	while (i < dc->blocks_num) {
		blk_base = mdrv->mdev->reg_base + (offset >> 2);

		dc_read_block_header(blk_base, &blk);
		if (BST_BLK_INFO_BLK_TYPE(blk.block_info) == DC_BLK_TYPE_LPU) {
			err = dc_probe_block(dc, &blk, blk_base);
			if (err)
				DRM_ERROR("Fail to probe DC_BLK_TYPE_LPU.\n");
		} else if (BST_BLK_INFO_BLK_TYPE(blk.block_info) == DC_BLK_TYPE_LPU_LAYER) {
			bstdc_write32(blk_base, BST_LAYER_PIXALPHA, DC_PALPHA_DEF_MAP);
		}

		i++;
		offset += DC_BLOCK_SIZE;
	}

	err = drm_mode_config_helper_resume(&mdrv->kms->base);
	mdrv->mdev->resume = false;
	return err;
}

static const struct dev_pm_ops bst_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bst_pm_suspend, bst_pm_resume)
	SET_RUNTIME_PM_OPS(bst_rt_pm_suspend, bst_rt_pm_resume, NULL)
};

static void bst_platform_shutdown(struct platform_device *pdev)
{
	bst_pm_suspend(&pdev->dev);
	return;
}

static struct platform_driver bst_native_drm_platform_driver = {
	.probe	= bst_platform_probe,
	.remove	= bst_platform_remove,
	.shutdown = bst_platform_shutdown,
	.driver	= {
		.name = "bst-drm",
		.of_match_table	= bst_of_match,
		.pm = &bst_pm_ops,
	},
};

module_platform_driver(bst_native_drm_platform_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST Native DRM-KMS driver");
MODULE_LICENSE("GPL v2");
