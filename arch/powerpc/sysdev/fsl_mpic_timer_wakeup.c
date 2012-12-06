/*
 * MPIC timer wakeup driver
 *
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/sysdev.h>

#include <asm/mpic_timer.h>

struct fsl_mpic_timer_wakeup {
	struct mpic_timer *timer;
	struct work_struct free_work;
};

static struct fsl_mpic_timer_wakeup *fsl_wakeup;
static DEFINE_MUTEX(sysfs_lock);

static void fsl_free_resource(struct work_struct *ws)
{
	struct fsl_mpic_timer_wakeup *wakeup =
		container_of(ws, struct fsl_mpic_timer_wakeup, free_work);

	mpic_free_timer(wakeup->timer);
	wakeup->timer = NULL;
}

static irqreturn_t fsl_mpic_timer_irq(int irq, void *dev_id)
{
	struct fsl_mpic_timer_wakeup *wakeup = dev_id;

	schedule_work(&wakeup->free_work);
	return IRQ_HANDLED;
}

static ssize_t fsl_timer_wakeup_show(struct sysdev_class *kobj,
				struct sysdev_class_attribute *attr,
				char *buf)
{
	struct timeval interval;
	int val, status;

	mutex_lock(&sysfs_lock);

	if (fsl_wakeup->timer) {
		mpic_get_remain_time(fsl_wakeup->timer, &interval);
		val = interval.tv_sec + 1;
	} else {
		val = 0;
	}
	status = sprintf(buf, "%d\n", val);

	mutex_unlock(&sysfs_lock);

	return status;
}

static ssize_t fsl_timer_wakeup_store(struct sysdev_class *kobj,
				struct sysdev_class_attribute *attr,
				const char *buf, size_t count)
{
	struct timeval interval;

	if (fsl_wakeup->timer)
		return -EBUSY;

	interval.tv_usec = 0;
	if (kstrtol(buf, 0, &interval.tv_sec))
		return -EINVAL;

	mutex_lock(&sysfs_lock);

	fsl_wakeup->timer = mpic_request_timer(fsl_mpic_timer_irq,
						fsl_wakeup, &interval);
	if (!fsl_wakeup->timer) {
		mutex_unlock(&sysfs_lock);
		return -EINVAL;
	}

	mpic_start_timer(fsl_wakeup->timer);

	mutex_unlock(&sysfs_lock);

	return count;
}

static SYSDEV_CLASS_ATTR(timer_wakeup, 0644,
			fsl_timer_wakeup_show, fsl_timer_wakeup_store);

static struct sysdev_class mpic_sysdev_class = {
	.name = "mpic",
};

static int __init fsl_wakeup_sys_init(void)
{
	int ret = 0;

	fsl_wakeup = kzalloc(sizeof(struct fsl_mpic_timer_wakeup), GFP_KERNEL);
	if (!fsl_wakeup)
		return -ENOMEM;

	INIT_WORK(&fsl_wakeup->free_work, fsl_free_resource);

	ret = sysdev_class_register(&mpic_sysdev_class);
	if (ret)
		goto err;

	ret = sysdev_class_create_file(&mpic_sysdev_class, &attr_timer_wakeup);
	if (ret) {
		sysdev_class_unregister(&mpic_sysdev_class);
		goto err;
	}

	return ret;

err:
	kfree(fsl_wakeup);
	return ret;
}

static void __exit fsl_wakeup_sys_exit(void)
{
	sysdev_class_remove_file(&mpic_sysdev_class, &attr_timer_wakeup);

	sysdev_class_unregister(&mpic_sysdev_class);

	kfree(fsl_wakeup);
}

module_init(fsl_wakeup_sys_init);
module_exit(fsl_wakeup_sys_exit);

MODULE_DESCRIPTION("Freescale MPIC global timer wakeup driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wang Dongsheng <dongsheng.wang@freescale.com>");
