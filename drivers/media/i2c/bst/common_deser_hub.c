// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_gpio.h>
#include <media/media-entity.h>
#include <media/media-device.h>
#include <linux/kernel.h>
#include <linux/gfp.h>

#include "common_deser_hub.h"
#include "maxim_deser_hub.h"
#include "ti_deser_hub.h"
#ifdef C1200_ISP
#include "../../platform/bst-c1200/isp_fw_loader.h"
#endif

/*
 *	misc_deivce : deser
 */
static long deser_misc_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long args)
{
	int ret = 0;
	struct deser_trigger_info info;
	struct deser_hub_dev *hub =
		container_of(filp->private_data, struct deser_hub_dev, miscdev);

	dev_dbg(hub->miscdev.this_device, "%s : enter, cmd = 0x%08x\n", __func__, cmd);

	switch (cmd) {
	case DESERIOC_SET_TRIGGER_INFO:
		{
			dev_info(hub->miscdev.this_device, "DESERIOC_SET_TRIGGER_INFO");
			ret = copy_from_user(&info,
					(struct deser_trigger_info __user *)args,
					sizeof(struct deser_trigger_info));
			if (ret < 0) {
				dev_err(hub->miscdev.this_device, "copy_from_user failed!");
				return ret;
			}
			// dummp_trigger_info(hub->miscdev.this_device, &info);
			memcpy(&hub->trig_info, &info, sizeof(struct deser_trigger_info));
			return 0;
		}
	case DESERIOC_GET_TRIGGER_INFO:
		{
			dev_info(hub->miscdev.this_device, "DESERIOC_GET_TRIGGER_INFO");
			ret = copy_to_user((struct deser_trigger_info __user *)args,
								&hub->trig_info,
								sizeof(struct deser_trigger_info));
			// dummp_trigger_info(hub->miscdev.this_device, &hub->trig_info);
			if (ret < 0) {
				dev_err(hub->miscdev.this_device, "copy_to_user failed!");
				return ret;
			}
			return 0;
		}
	case DESERIOC_ENABLE_ISP_TRIGGER: //ISP
		{
			struct deser_trigger_info *trig_info;

			dev_info(hub->miscdev.this_device, "DESERIOC_ENABLE_TRIGGER");

			if (hub->ctl_mode == FAD_LIS_MODE) {
				dev_info(hub->miscdev.this_device, "LIS mode break");
				return 0;
			}
			trig_info = &hub->trig_info;

			return 0;
		}
	case DESERIOC_ENABLE_REPILICATE:
		{
			dev_info(hub->miscdev.this_device, "DESERIOC_ENABLE_RPILICATE");
			if (hub->type == DESER_TYPE_MAX96712)
				max967XX_replicate_mode(hub);
			else if (hub->type == DESER_TYPE_TI960)
				ti_deser_hub_enable_replicate(hub);
			return 0;
		}
	default:
		return -EINVAL;
	}
}

static const struct file_operations deser_misc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = deser_misc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = deser_misc_ioctl,
#endif
};

static void set_misc_name(char **name, const char *fmt, ...)
{
	va_list vargs;

	va_start(vargs, fmt);
	*name = kvasprintf(GFP_KERNEL, fmt, vargs);
	va_end(vargs);
}

int register_deser_miscdev(struct miscdevice *miscdev, int deser_index)
{
	int ret;

	if (miscdev == NULL) {
		pr_err("miscdev is NULL\n");
		return -EFAULT;
	}
	miscdev->minor = MISC_DYNAMIC_MINOR;
	set_misc_name((char **)&miscdev->name, "%s%d", "deser", deser_index);

	miscdev->fops = &deser_misc_fops;
	pr_debug("device name: %s", miscdev->name);
	// register as misc device
	ret = misc_register(miscdev);
	if (ret < 0) {
		pr_err("misc register failed\n");
		return -EFAULT;
	}
	return 0;
}

void dummp_trigger_info(struct device *dev, struct deser_trigger_info *info)
{
	int i;

	dev_info(dev, "%-10s:%d", "trigger_mode", info->trigger_mode);
	dev_info(dev, "%-10s:%d", "trigger_fps", info->trigger_fps);
	dev_info(dev, "%-10s:%d", "trigger_rx_gpio", info->trigger_rx_gpio);
	for (i = 0; i < 4; i++)
		dev_info(dev, "%-10s[%d]:%d", "trigger_tx_gpio", i, info->trigger_tx_gpio[i]);
}
