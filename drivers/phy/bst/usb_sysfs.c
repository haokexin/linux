// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * driver for BST usb  phy cr debug test
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/sysfs.h>
#include <linux/fs.h>
#include "phy-bst-usb.h"

static ssize_t cr_read_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct bst_usb *phy = dev_get_drvdata(dev);
	unsigned short addr, data;

	if (sscanf(buf, "0x%hx 0x%hx", &addr, &data) == 2) {
		dev_info(phy->dev, "read | addr=%04x, data=%04x\n", addr, data);
		data |= read_cr_reg(phy->phy_base, addr);
		write_cr_reg(phy->phy_base, addr, data);
		dev_info(phy->dev, "read | addr=%04x, data=%04x\n", addr,
			 read_cr_reg(phy->phy_base, addr));
	} else if (sscanf(buf, "0x%hx", &addr) == 1) {
		data = read_cr_reg(phy->phy_base, addr);
		dev_info(phy->dev, "read addr=%04x, data=%04x\n", addr, data);
	}
	return size;
}

static ssize_t cr_write_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct bst_usb *phy = dev_get_drvdata(dev);
	unsigned short addr, data;

	if (sscanf(buf, "0x%hx 0x%hx", &addr, &data) == 2) {
		dev_info(phy->dev, "write addr=%04x, data=%04x\n", addr, data);
		write_cr_reg(phy->phy_base, addr, data);
		data = read_cr_reg(phy->phy_base, addr);
		dev_info(phy->dev, "write addr=%04x, data=%04x back\n", addr, data);
	}
	return size;
}


static ssize_t orientation_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct bst_usb *phy = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", phy->cur_orientation);
}

static ssize_t orientation_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct bst_usb *phy = dev_get_drvdata(dev);
	unsigned int data;
	int ret;

	ret = kstrtouint(buf, 10, &data);
	if (ret < 0) {
		dev_err(dev, "Invalid input: %s\n", buf);
		return size;
	}

	if (data == 1 || data == 2)
		set_orientation(phy, data);
	else
		dev_warn(dev, "Invalid orientation value: %u. Only 1 or 2 are allowed.\n", data);
	return size;
}

static umode_t
cr_test_is_visible(struct kobject *kobj, struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct bst_usb *phy = dev_get_drvdata(dev);

	if (phy->allow_cr_test)
		return attr->mode;

	return 0;
}

// Declare device attributes for cr_write and cr_read
static struct device_attribute dev_attr_cr_write = __ATTR_WO(cr_write);
static struct device_attribute dev_attr_cr_read = __ATTR_WO(cr_read);
static struct device_attribute dev_attr_orientation = __ATTR_RW(orientation);

// Create a sysfs group for cr_write and cr_read attributes
static const struct attribute *bst_usb_attrs[] = {
	&dev_attr_cr_write.attr,
	&dev_attr_cr_read.attr,
	&dev_attr_orientation.attr,
	NULL,
};

static const struct attribute_group bst_usb_attr_group = {
	.attrs = (struct attribute **)bst_usb_attrs,
	.is_visible = cr_test_is_visible,
};

int bst_sysfs_cr_create(struct device *dev)
{
	// Create sysfs group for cr_write and cr_read attributes
	if (dev)
		return sysfs_create_group(&dev->kobj, &bst_usb_attr_group);
	else
		return -1;
}

void bst_sysfs_cr_remove(struct device *dev)
{
	if (dev)
		sysfs_remove_group(&dev->kobj, &bst_usb_attr_group);
}

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("Black Sesame Technologies USB phy driver");
MODULE_LICENSE("GPL v2");
