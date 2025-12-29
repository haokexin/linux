/* SPDX-License-Identifier: GPL-2.0 */
/* virt-touchscreen driver for BST C1200
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 */

 #define pr_fmt(fmt) KBUILD_BASENAME ": " fmt

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include <touch_virt_bst_cdev.h>

#define TOUCH_DEV_NAME "bst_touch"

/* It may not be used here */
#define TOUCH_MAX_CHAR_DEVICES	1024
#define TOUCH_FIRST_DYNAMIC_DEV	256
#define TOUCH_DEV_MINOR_BASE	64
#define TOUCH_DEV_MINORS		32
#define TOUCH_DEV_MINOR_MAX		(TOUCH_DEV_MINOR_BASE + TOUCH_DEV_MINORS)

static DEFINE_IDA(touch_ida);

static LIST_HEAD(ts_dev_list);
static DEFINE_SPINLOCK(ts_dev_list_lock);

static struct class *ts_dev_class;

static int major;		/* default to dynamic major */
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

struct ts_dev {
	struct list_head list;
	struct bst_ts_data *ts;
	struct device dev;
	struct cdev cdev;
};

/**
 * touch_dev_get_new_minor - allocates a new input minor number
 * @legacy_base: beginning of the legacy range to be searched
 * @legacy_num: size of legacy range
 * @allow_dynamic: whether we can also take ID from the dynamic range
 *
 * This function allocates a new device minor from the input major namespace.
 * Caller can request a legacy minor by specifying @legacy_base and @legacy_num
 * parameters and whether the ID can be allocated from the dynamic range if there are
 * no free IDs in the legacy range.
 */
static int touch_dev_get_new_minor(int legacy_base, unsigned int legacy_num,
			bool allow_dynamic)
{
	if (legacy_base >= 0) {
		int minor = ida_simple_get(&touch_ida,
					   legacy_base,
					   legacy_base + legacy_num,
					   GFP_KERNEL);
		if (minor >= 0 || !allow_dynamic)
			return minor;
	}

	return ida_simple_get(&touch_ida,
			      TOUCH_FIRST_DYNAMIC_DEV, TOUCH_MAX_CHAR_DEVICES,
			      GFP_KERNEL);
}

/**
 * touch_dev_free_minor - release previously allocated minor
 * @minor: minor to be released
 *
 * This function releases a previously allocated input minor so that it can be
 * reused later.
 */
static void touch_dev_free_minor(unsigned int minor)
{
	ida_simple_remove(&touch_ida, minor);
}

static struct ts_dev *ts_dev_get_by_devt(dev_t index)
{
	struct ts_dev *ts_dev;

	spin_lock(&ts_dev_list_lock);
	list_for_each_entry(ts_dev, &ts_dev_list, list) {
		if (ts_dev->ts->devt == index)
			goto found;
	}
	ts_dev = NULL;
found:
	spin_unlock(&ts_dev_list_lock);
	return ts_dev;
}

static void tsdev_dev_release(struct device *dev)
{
	struct ts_dev *ts_dev;

	ts_dev = container_of(dev, struct ts_dev, dev);
	kfree(ts_dev);
	ts_dev = NULL;
}

static struct ts_dev *get_free_ts_dev(struct bst_ts_data *ts)
{
	struct ts_dev *ts_dev;

	if (MINOR(ts->devt) >= TOUCH_DEV_MINOR_MAX) {
		printk(KERN_ERR "Touch-dev: Out of device minors (%d)\n",
		       MINOR(ts->devt));
		return ERR_PTR(-ENODEV);
	}

	ts_dev = kzalloc(sizeof(*ts_dev), GFP_KERNEL);
	if (!ts_dev)
		return ERR_PTR(-ENOMEM);
	ts_dev->ts = ts;

	spin_lock(&ts_dev_list_lock);
	list_add_tail(&ts_dev->list, &ts_dev_list);
	spin_unlock(&ts_dev_list_lock);
	return ts_dev;
}

static void put_ts_dev(struct ts_dev *ts_dev, bool del_cdev)
{
	spin_lock(&ts_dev_list_lock);
	list_del(&ts_dev->list);
	spin_unlock(&ts_dev_list_lock);
	if (del_cdev) {
		cdev_device_del(&ts_dev->cdev, &ts_dev->dev);
		touch_dev_free_minor(MINOR(ts_dev->dev.devt));
	}
	put_device(&ts_dev->dev);
}

#if 1
/*
 * static char *ts_devnode(struct device *dev, umode_t *mode)
 * {
 * 	return kasprintf(GFP_KERNEL, "ts-dev/%s", dev_name(dev)); // /dev/%s
 * }
 */
static struct class g_ts_dev_class = {
	.name		= "ts-dev",
	/* .devnode	= ts_devnode, */
};
#else
static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct ts_dev *ts_dev = ts_dev_get_by_devt(dev->devt);

	if (!ts_dev)
		return -ENODEV;
	return sprintf(buf, "%s\n", ts_dev->ts->uniq);
}
static DEVICE_ATTR_RO(name);

static struct attribute *ts_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ts);
#endif

/**
 * bst_touch_open - Open function for the touch device
 * @inp: Pointer to the inode structure
 * @filp: Pointer to the file structure
 *
 * This function is called when the touch device is opened.
 *
 * Return: 0 on success, negative error code on failure
 */
static int bst_touch_open(struct inode *inp, struct file *filp)
{
	struct ts_dev *ts_dev;

#if 0
	ts_dev = ts_dev_get_by_devt(inp->i_rdev);
	if (!ts_dev) {
		pr_err("%s: invalid device file open request (devt=0x%x).", __func__, inp->i_rdev);
		return -ENODEV;
	}
#else
	ts_dev = container_of(inp->i_cdev, struct ts_dev, cdev);
#endif
	filp->private_data = ts_dev->ts;
	/* pr_info("%s: device file open request (devt=0x%x).\n", __func__, ts_dev->ts->devt); */

	return 0;
}

/**
 * bst_touch_cdev_init - Initialize the character device for the touch screen
 * @ts: Pointer to the touch screen data structure
 * @fops: Pointer to the file operations structure
 *
 * Return: 0 on success, negative error code on failure
 */
int bst_touch_cdev_init(struct bst_ts_data *ts, struct file_operations *fops)
{
	struct ts_dev *ts_dev;
	int retval;
	int minor;
	int dev_no;
	dev_t devid;

	if (!ts || !ts->input_dev || !fops)
		return -EINVAL;

	mutex_init(&ts->mutex);

	minor = touch_dev_get_new_minor(TOUCH_DEV_MINOR_BASE, TOUCH_DEV_MINORS, true);
	if (minor < 0) {
		retval = minor;
		pr_err("Failed to reserve new minor: %d\n", retval);
		goto err_quit;
	}
	devid = MKDEV(major, minor);
	ts->devt = devid;

	dev_dbg(&ts->pdev->dev, "Devname: %s, major: %d, minor: %d devt: 0x%x\n", TOUCH_DEV_NAME, major, minor, devid);

	ts_dev = get_free_ts_dev(ts);
	if (IS_ERR(ts_dev)) {
		retval = PTR_ERR(ts_dev);
		goto err_free_minor;
	}

	dev_no = minor;
	/* Normalize device number if it falls into legacy range */
	if (dev_no < TOUCH_DEV_MINOR_MAX)
		dev_no -= TOUCH_DEV_MINOR_BASE;

	fops->open = bst_touch_open;
	cdev_init(&ts_dev->cdev, fops);
	ts_dev->cdev.owner = THIS_MODULE;

	device_initialize(&ts_dev->dev);
	ts_dev->dev.parent = &ts->input_dev->dev;
	ts_dev->dev.devt = ts->devt;
	ts_dev->dev.class = ts_dev_class;
	ts_dev->dev.release = tsdev_dev_release;
	dev_set_name(&ts_dev->dev, TOUCH_DEV_NAME"%d", dev_no);

	retval = cdev_device_add(&ts_dev->cdev, &ts_dev->dev);
	if (retval)
		goto err_put_ts_dev;

	pr_info("Touch dev: touchscreen [%s] registered as minor %d\n",
		 ts->uniq, MINOR(ts->devt));
    return 0;

err_put_ts_dev:
	put_ts_dev(ts_dev, false);
err_free_minor:
	touch_dev_free_minor(minor);
err_quit:
	pr_err ("Touch dev: Failed to initialize touchscreen [%s]\n", ts->uniq);
	return retval;
}
EXPORT_SYMBOL_GPL(bst_touch_cdev_init);

/**
 * bst_touch_cdev_remove - Remove the character device for the touch screen
 * @ts: Pointer to the touch screen data structure
 */
void bst_touch_cdev_remove(struct bst_ts_data *ts)
{
	struct ts_dev *ts_dev;

	if (!ts)
		return;

	ts_dev = ts_dev_get_by_devt(ts->devt);
	if (!ts_dev)
		return;

	put_ts_dev(ts_dev, true);
	pr_info("Touch dev: touchscreen [%s] unregistered\n", ts->uniq);
}
EXPORT_SYMBOL_GPL(bst_touch_cdev_remove);

/**
 * bst_ts_dev_init - Initialize the touch screen device subsystem
 *
 * Return: 0 on success, negative error code on failure
 */
static int __init bst_ts_dev_init(void)
{
	dev_t devid;
	int retval;
#if 1
	ts_dev_class = &g_ts_dev_class;
	retval = class_register(ts_dev_class);
	if (retval) {
		pr_err("Unable to register touchscreen_dev class\n");
		goto err_class_create;
	}
#else
	ts_dev_class = class_create(THIS_MODULE, "ts-dev");
	if (IS_ERR(ts_dev_class)) {
		retval = PTR_ERR(ts_dev_class);
		goto err_class_create;
	}
	ts_dev_class->dev_groups = ts_groups;
#endif

	if (major) {
		devid = MKDEV(major, TOUCH_DEV_MINOR_BASE);
		retval = register_chrdev_region(devid, TOUCH_DEV_MINORS, TOUCH_DEV_NAME);
	} else {
		retval = alloc_chrdev_region(&devid, TOUCH_DEV_MINOR_BASE, TOUCH_DEV_MINORS, TOUCH_DEV_NAME);
		major = MAJOR(devid);
	}
	if (retval < 0) {
		pr_err("Failed to register chrdev region: %d\n", retval);
		goto err_unreg_class;
	}

	pr_info("BST Virt Touchscreen cdev core initialized (major %d)\n", major);

	return 0;

err_unreg_class:
	/* class_destroy(ts_dev_class); */
	class_unregister(ts_dev_class);
err_class_create:
	return retval;
}

/**
 * bst_ts_dev_exit - Cleanup the touch screen device subsystem
 */
static void __exit bst_ts_dev_exit(void)
{
	unregister_chrdev_region(MKDEV(major, TOUCH_DEV_MINOR_BASE), TOUCH_DEV_MINORS);
	/* class_destroy(ts_dev_class); */
	class_unregister(ts_dev_class);
}

subsys_initcall(bst_ts_dev_init);
module_exit(bst_ts_dev_exit);

MODULE_AUTHOR("Pengcheng Xue");
MODULE_DESCRIPTION("BST Virt Touchscreen cdev core");
MODULE_LICENSE("GPL v2");
