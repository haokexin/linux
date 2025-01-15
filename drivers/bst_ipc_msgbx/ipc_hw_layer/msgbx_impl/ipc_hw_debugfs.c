// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>


static int msgbox_debug;

bool msgbox_debug_has_en(void)
{
	return msgbox_debug == 1 ? true : false;
}
EXPORT_SYMBOL(msgbox_debug_has_en);

static int __init msgbox_debug_bootargs(char *str)
{
	return kstrtoint(str, 10, &msgbox_debug);
}

__setup("msgbox_debug=", msgbox_debug_bootargs);

static ssize_t msgbox_debug_show(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d\n", msgbox_debug);
}

static ssize_t msgbox_debug_store(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t count)
{
	int ret = 0;

	ret = kstrtoint(buf, 10, &msgbox_debug);
	if (ret < 0)
		return ret;

	return count;
}

static struct kobj_attribute msgbx_dbg_attr = __ATTR_RW(msgbox_debug);

static struct attribute *attrs[] = {
	&msgbx_dbg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int msgbx_dbg_sysfs_init(struct device *dev)
{
	int ret;

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if (ret)
		kobject_put(&dev->kobj);

	return ret;
}

void msgbx_dbg_sysfs_exit(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &attr_group);
}
