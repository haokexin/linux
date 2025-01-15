// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/mman.h>
#include "bst_hwcv_ioctl.h"
#include "bst_hwcv_sys_manager.h"

static ssize_t bst_hwcv_timer_attr_show(struct kobject *object,
					struct kobj_attribute *attr, char *buf)
{
	struct bst_hwcv_sys_manager *sys_manager;

	sys_manager = container_of(object, struct bst_hwcv_sys_manager, kobj);
	dev_info(sys_manager->dev, "Show hwcv timer: %d", sys_manager->timer);

	return sprintf(buf, "Hwcv timer: %d\n", sys_manager->timer);
}

static ssize_t bst_hwcv_timer_attr_store(struct kobject *object,
					 struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	int val;
	struct bst_hwcv_sys_manager *sys_manager;

	sys_manager = container_of(object, struct bst_hwcv_sys_manager, kobj);
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0 || val < 0) {
		dev_err(sys_manager->dev, "Store invalid hwcv timer: %d", val);
	} else {
		sys_manager->timer = val;
		dev_info(sys_manager->dev, "Store hwcv timer: %d",
			 sys_manager->timer);
	}
	return count;
}

static struct kobj_attribute bst_hwcv_timer_attr =
	__ATTR(bst_hwcv_timer, 0664, bst_hwcv_timer_attr_show,
	       bst_hwcv_timer_attr_store);

static struct attribute *bst_hwcv_kobj_attrs[] = {
	&bst_hwcv_timer_attr.attr,
	NULL,
};

static struct attribute_group bst_hwcv_kobj_attr_group = {
	.attrs = bst_hwcv_kobj_attrs,
};

static void dynamic_kobj_release(struct kobject *kobj)
{
	struct bst_hwcv_sys_manager *sys_manager;

	sys_manager = container_of(kobj, struct bst_hwcv_sys_manager, kobj);
	dev_info(sys_manager->dev, "dynamic_kobj_release");
}

static struct kobj_type dynamic_kobj_ktype = {
	.release = dynamic_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

int bst_hwcv_sys_manager_init(struct device *dev,
			      struct bst_hwcv_sys_manager *sys_manager)
{
	int ret;

	dev_info(dev, "Init hwcv sys manager.");
	sys_manager->dev = dev;

	ret = kobject_init_and_add(&sys_manager->kobj, &dynamic_kobj_ktype,
				   kernel_kobj, BST_HWCV_DRIVER_NAME);
	if (unlikely(ret != 0)) {
		dev_err(dev, "Init kobject failed.");
		kobject_put(&sys_manager->kobj);
		return ret;
	}

	ret = sysfs_create_group(&sys_manager->kobj, &bst_hwcv_kobj_attr_group);
	if (ret) {
		dev_err(dev, "Create sysfs group failed.");
		kobject_put(&sys_manager->kobj);
		return ret;
	}

	return ret;
}

void bst_hwcv_sys_manager_exit(struct bst_hwcv_sys_manager *sys_manager)
{
	sysfs_remove_group(&sys_manager->kobj, &bst_hwcv_kobj_attr_group);
	kobject_del(&sys_manager->kobj);
	kobject_put(&sys_manager->kobj);
	dev_info(sys_manager->dev, "Exit hwcv sysfile.");
}
