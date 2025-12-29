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
#include <drm/drm_of.h>
#include "bst_virt_drm_device.h"
#include "bst_virt_drm_kms.h"

struct bst_virt_drm_drv {
	struct bst_super_device *super_dev;
	struct bst_kms_dev *kms;
};

static const u32 platforms_id[] = { BST_PLATFORM_ID/* C1200 */ ,0};

struct bst_super_device *dev_to_super_dev(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);

	return drv ? drv->super_dev : NULL;
}

static void bst_unbind(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);

	if (!drv)
		return;

	bst_kms_detach(drv->kms);

	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);
	else
		bst_virt_dev_suspend(drv->super_dev);

	bst_virt_dev_destroy(drv->super_dev);

	dev_set_drvdata(dev, NULL);
}

static int bst_bind(struct device *dev)
{
	struct bst_virt_drm_drv *drv;
	int err;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->super_dev = bst_virt_dev_create(dev);
	if (IS_ERR(drv->super_dev)) {
		err = PTR_ERR(drv->super_dev);
		goto done;
	}
	dev_set_drvdata(dev, drv);

	pm_runtime_enable(dev);
	if (!pm_runtime_enabled(dev))
		bst_virt_dev_resume(drv->super_dev);

	drv->kms = bst_kms_attach(drv->super_dev);
	if (IS_ERR(drv->kms)) {
		err = PTR_ERR(drv->kms);
		goto destroy_super_dev;
	}

	return 0;

destroy_super_dev:
	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);
	else
		bst_virt_dev_suspend(drv->super_dev);

	bst_virt_dev_destroy(drv->super_dev);

done:
	return err;
}

static const struct component_master_ops bst_master_ops = {
	.bind = bst_bind,
	.unbind = bst_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/*
 * This list determines the binding order of our components
 */
extern struct platform_driver bst_virt_connectors_driver;
static struct platform_driver *const component_drivers[] = {
	&bst_virt_connectors_driver,
};

static int bst_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *remote;
	struct device_node *child;

	for_each_available_child_of_node(dev->of_node, child) {
		if (!of_node_cmp(child->name, "pipeline")) { /* add connector */
			remote = of_graph_get_remote_node(child, 0, 0);
			if (remote) {
				drm_of_component_match_add(dev, &match,
							   compare_of, remote);
				of_node_put(remote);
			}
		}
	}
	if (!match) {
		DRM_ERROR("component match is NULL\n");
		return -EINVAL;
	}
	return component_master_add_with_match(dev, &bst_master_ops,
						       match);
}

static int bst_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &bst_master_ops);
	return 0;
}

static const struct of_device_id bst_of_match[] = {
	{
		.compatible = "bst,c1200-virtual-display0",
		.data = &platforms_id[0],
	},
	{
		.compatible = "bst,c1200-virtual-display1",
		.data = &platforms_id[0],
	},
	{
		.compatible = "bst,c1200-virtual-display2",
		.data = &platforms_id[0],
	},
	{
		.compatible = "bst,c1200-virtual-display3",
		.data = &platforms_id[0],
	},
	{
		.compatible = "bst,c1200-virtual-display4",
		.data = &platforms_id[0],
	},
	{},
};

MODULE_DEVICE_TABLE(of, bst_of_match);

static int __maybe_unused bst_rt_pm_suspend(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);

	bst_virt_connector_suspend(drv->super_dev);

	return bst_virt_dev_suspend(drv->super_dev);
}

static int __maybe_unused bst_rt_pm_resume(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);

	bst_virt_connector_resume(drv->super_dev);

	return bst_virt_dev_resume(drv->super_dev);
}

static int __maybe_unused bst_pm_suspend(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);
	int res;

	res = drm_mode_config_helper_suspend(&drv->kms->base);

	bst_virt_connector_suspend(drv->super_dev);

	if (!pm_runtime_status_suspended(dev))
		bst_virt_dev_suspend(drv->super_dev);

	return res;
}

static int __maybe_unused bst_pm_resume(struct device *dev)
{
	struct bst_virt_drm_drv *drv = dev_get_drvdata(dev);

	bst_virt_connector_resume(drv->super_dev);

	if (!pm_runtime_status_suspended(dev))
		bst_virt_dev_resume(drv->super_dev);

	return drm_mode_config_helper_resume(&drv->kms->base);
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

static struct platform_driver bst_virtual_drm_platform_driver = {
	.probe	= bst_platform_probe,
	.remove	= bst_platform_remove,
	.shutdown = bst_platform_shutdown,
	.driver	= {
		.name = "bst-vdrm",
		.of_match_table	= bst_of_match,
		.pm = &bst_pm_ops,
	},
};

static int __init bst_virt_drm_register(void)
{
	int ret;

	ret = platform_driver_register(&bst_virtual_drm_platform_driver);
	if (ret)
		return ret;

	ret = platform_register_drivers(component_drivers,
					ARRAY_SIZE(component_drivers));

	if (ret)
		platform_unregister_drivers(component_drivers,
					    ARRAY_SIZE(component_drivers));
	return ret;
}

static void __exit bst_virt_drm_unregister(void)
{
	platform_unregister_drivers(component_drivers,
				    ARRAY_SIZE(component_drivers));
	platform_driver_unregister(&bst_virtual_drm_platform_driver);
}

late_initcall(bst_virt_drm_register);
module_exit(bst_virt_drm_unregister);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST Virtual DRM-KMS driver");
MODULE_LICENSE("GPL v2");
