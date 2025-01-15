// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/dma-direct.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include "bst_hwcv_ioctl.h"
#include "bst_hwcv_miscdev.h"
#include "bst_hwcv_gwarp.h"
#include "bst_hwcv_scaler.h"

#define WAIT_TIME_OUT_MSEC 10000

static int bst_hwcv_ioctl_alloc(struct file *filp,
				struct bst_hwcv_misc_dev *misc_dev,
				struct hwcv_allocation_data __user *udata)
{
	int ret;
	struct hwcv_allocation_data req;
	struct bst_hwcv_buf *buf;
	struct device *dev = misc_dev->dev;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret < 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	buf = bst_hwcv_buf_alloc(&misc_dev->mem_manager, req.length);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	req.length = buf->length;
	req.paddr = buf->iova;
	req.fd = buf->fd;

	mutex_lock(&misc_dev->mem_manager.lock);
	ret = bst_hwcv_add_buf_to_ctx(filp, &misc_dev->mem_manager, buf);
	if (ret < 0) {
		mutex_unlock(&misc_dev->mem_manager.lock);
		bst_hwcv_buf_free(&misc_dev->mem_manager, buf);
		return ret;
	}
	mutex_unlock(&misc_dev->mem_manager.lock);

	ret = copy_to_user(udata, &req, sizeof(req));
	if (ret < 0) {
		dev_err(dev, "Failed to copy to user");
		bst_hwcv_buf_free(&misc_dev->mem_manager, buf);
		return ret;
	}

	return 0;
}

static int bst_hwcv_ioctl_free(struct file *filp,
			       struct bst_hwcv_misc_dev *misc_dev,
			       struct hwcv_handle_data __user *udata)
{
	int ret;
	struct hwcv_handle_data req;
	struct bst_hwcv_buf *buf;
	struct device *dev = misc_dev->dev;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret < 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	mutex_lock(&misc_dev->mem_manager.lock);
	buf = bst_hwcv_del_buf_from_ctx(filp, &misc_dev->mem_manager,
					req.paddr);
	if (IS_ERR(buf)) {
		mutex_unlock(&misc_dev->mem_manager.lock);
		return PTR_ERR(buf);
	}
	mutex_unlock(&misc_dev->mem_manager.lock);

	bst_hwcv_buf_free(&misc_dev->mem_manager, buf);

	return 0;
}

static int bst_hwcv_ioctl_import(struct file *filp,
				 struct bst_hwcv_misc_dev *misc_dev,
				 struct hwcv_fd_data __user *udata)
{
	int ret;
	struct hwcv_fd_data req;
	struct bst_hwcv_buf *buf;
	struct device *dev = misc_dev->dev;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret != 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	buf = bst_hwcv_buf_import(&misc_dev->mem_manager, req.fd, req.length);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	req.paddr = buf->iova;

	mutex_lock(&misc_dev->mem_manager.lock);
	ret = bst_hwcv_add_buf_to_ctx(filp, &misc_dev->mem_manager, buf);
	if (ret < 0) {
		bst_hwcv_buf_return(&misc_dev->mem_manager, buf);
		return ret;
	}
	mutex_unlock(&misc_dev->mem_manager.lock);

	ret = copy_to_user(udata, &req, sizeof(req));
	if (ret < 0) {
		dev_err(dev, "Failed to copy to user");
		bst_hwcv_buf_return(&misc_dev->mem_manager, buf);
		return ret;
	}

	return 0;
}

static int bst_hwcv_ioctl_return(struct file *filp,
				 struct bst_hwcv_misc_dev *misc_dev,
				 struct hwcv_handle_data __user *udata)
{
	int ret;
	struct hwcv_handle_data req;
	struct bst_hwcv_buf *buf;
	struct device *dev = misc_dev->dev;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret != 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	mutex_lock(&misc_dev->mem_manager.lock);
	buf = bst_hwcv_del_buf_from_ctx(filp, &misc_dev->mem_manager,
					req.paddr);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	mutex_unlock(&misc_dev->mem_manager.lock);

	bst_hwcv_buf_return(&misc_dev->mem_manager, buf);

	return 0;
}

static int bst_hwcv_ioctl_gwarp(struct file *filp,
				struct bst_hwcv_misc_dev *misc_dev,
				struct hwcv_gwarp_data __user *udata)
{
	int ret;
	struct hwcv_gwarp_data req;
	struct device *dev = misc_dev->dev;
	ktime_t start_time;
	ktime_t end_time;
	s64 delta_time;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret != 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	if (req.engine_id >= BST_HWCV_GWARP_ENGINE_NUM) {
		dev_err(dev, "Invalid engine id: %d", req.engine_id);
		return -EINVAL;
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	mutex_lock(&misc_dev->gwarp_mutex[req.engine_id]);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Gwarp[%d-%d][%d]: Get mutex cost %lluns",
			 current->tgid, current->pid, req.engine_id,
			 delta_time);
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	ret = bst_gwarp_start(misc_dev->dev, &req);
	if (ret < 0) {
		dev_err(dev, "Failed to start Gwarp[%d]", req.engine_id);
		mutex_unlock(&misc_dev->gwarp_mutex[req.engine_id]);
		return ret;
	}

	dev_dbg(dev, "Gwarp[%d]: Wait for IRQ", req.engine_id);
	ret = wait_for_completion_timeout(
		&misc_dev->irq_manager.gwarp_irq_complete[req.engine_id],
		msecs_to_jiffies(WAIT_TIME_OUT_MSEC));
	if (!ret) {
		dev_err(dev, "Gwarp[%d]: Timeout", req.engine_id);
		bst_gwarp_disable(dev, req.engine_id);
		mutex_unlock(&misc_dev->gwarp_mutex[req.engine_id]);
		return -ETIMEDOUT;
	}
	dev_dbg(dev, "Gwarp[%d]: Received IRQ", req.engine_id);

	bst_gwarp_disable(dev, req.engine_id);
	mutex_unlock(&misc_dev->gwarp_mutex[req.engine_id]);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Gwarp[%d-%d][%d]: Operation cost %lluns",
			 current->tgid, current->pid, req.engine_id,
			 delta_time);
	}

	return 0;
}

static int bst_hwcv_ioctl_sbs_gwarp(struct file *filp,
				    struct bst_hwcv_misc_dev *misc_dev,
				    struct hwcv_sbs_gwarp_data __user *udata)
{
	int ret;
	struct hwcv_sbs_gwarp_data req;
	struct device *dev = misc_dev->dev;
	ktime_t start_time;
	ktime_t end_time;
	s64 delta_time;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret != 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	if (req.engine_id >= BST_HWCV_GWARP_ENGINE_NUM) {
		dev_err(dev, "Invalid engine id: %d", req.engine_id);
		return -EINVAL;
	}

	if (req.engine_id >= BST_HWCV_GWARP_SNR_NUM) {
		dev_err(dev, "Invalid sensor id: %d", req.sensor_id);
		return -EINVAL;
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	mutex_lock(&misc_dev->sbs_gwarp_mutex[req.engine_id][req.sensor_id]);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Gwarp[%d][%d]: Get mutex cost %lluns",
			 req.engine_id, req.sensor_id, delta_time);
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	ret = bst_sbs_gwarp_start(misc_dev->dev, &req);
	if (ret < 0) {
		dev_err(dev, "Failed to start Gwarp[%d][%d]", req.engine_id,
			req.sensor_id);
		mutex_unlock(&misc_dev->sbs_gwarp_mutex[req.engine_id]
						       [req.sensor_id]);
		return ret;
	}

	dev_dbg(dev, "Gwarp[%d][%d]: Wait for IRQ", req.engine_id,
		req.sensor_id);
	ret = wait_for_completion_timeout(
		&misc_dev->irq_manager
			 .sbs_gwarp_irq_complete[req.engine_id][req.sensor_id],
		msecs_to_jiffies(WAIT_TIME_OUT_MSEC));
	if (!ret) {
		dev_err(dev, "Gwarp[%d][%d]: Timeout", req.engine_id,
			req.sensor_id);
		bst_sbs_gwarp_disable(dev, req.engine_id, req.sensor_id);
		mutex_unlock(&misc_dev->sbs_gwarp_mutex[req.engine_id]
						       [req.sensor_id]);
		return -ETIMEDOUT;
	}
	dev_dbg(dev, "Gwarp[%d][%d]: Received IRQ", req.engine_id,
		req.sensor_id);

	bst_sbs_gwarp_disable(dev, req.engine_id, req.sensor_id);
	mutex_unlock(&misc_dev->sbs_gwarp_mutex[req.engine_id][req.sensor_id]);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Gwarp[%d][%d]: Operation cost %lluns",
			 req.engine_id, req.sensor_id, delta_time);
	}

	return 0;
}

static int bst_hwcv_ioctl_scaler(struct file *filp,
				 struct bst_hwcv_misc_dev *misc_dev,
				 struct hwcv_scaler_data __user *udata)
{
	int ret;
	struct hwcv_scaler_data req;
	struct device *dev = misc_dev->dev;
	ktime_t start_time;
	ktime_t end_time;
	s64 delta_time;

	ret = copy_from_user(&req, udata, sizeof(req));
	if (ret != 0) {
		dev_err(dev, "Failed to copy from user");
		return ret;
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	mutex_lock(&misc_dev->scaler_mutex);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Scaler[%d-%d]: Get mutex cost %lluns",
			 current->tgid, current->pid, delta_time);
	}

	if (misc_dev->sys_manager.timer)
		start_time = ktime_get();
	ret = bst_scaler_start(misc_dev->dev, &req);
	if (ret < 0) {
		dev_err(dev, "Failed to start scaler");
		mutex_unlock(&misc_dev->scaler_mutex);
		return ret;
	}

	dev_dbg(dev, "Wait for scaler IRQ");
	ret = wait_for_completion_timeout(
		&misc_dev->irq_manager.scaler_irq_complete,
		msecs_to_jiffies(WAIT_TIME_OUT_MSEC));
	if (!ret) {
		dev_err(dev, "scaler timeout");
		bst_scaler_disable(dev);
		bst_scaler_soft_reset(dev);
		mutex_unlock(&misc_dev->scaler_mutex);
		return -ETIMEDOUT;
	}
	dev_dbg(dev, "Receive scaler IRQ");

	bst_scaler_disable(dev);
	bst_scaler_soft_reset(dev);
	mutex_unlock(&misc_dev->scaler_mutex);
	if (misc_dev->sys_manager.timer) {
		end_time = ktime_get();
		delta_time = ktime_to_ns(ktime_sub(end_time, start_time));
		dev_info(dev, "Scaler[%d-%d]: Get Operation cost %lluns",
			 current->tgid, current->pid, delta_time);
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static long bst_hwcv_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long args)
{
	int ret = 0;
	struct bst_hwcv_misc_dev *misc_dev;
	struct device *dev;

	misc_dev = container_of(filp->private_data, struct bst_hwcv_misc_dev,
				miscdev);
	if (!misc_dev) {
		pr_err("%s: Invalid data", __func__);
		return -EINVAL;
	}
	dev = misc_dev->dev;

	switch (cmd) {
	case HWCV_IOCTL_ALLOC:
		dev_dbg(dev, "HWCV_IOCTL_ALLOC");
		ret = bst_hwcv_ioctl_alloc(filp, misc_dev, (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to alloc, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_FREE:
		dev_dbg(dev, "HWCV_IOCTL_FREE");
		ret = bst_hwcv_ioctl_free(filp, misc_dev, (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to free, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_IMPORT:
		dev_dbg(dev, "HWCV_IOCTL_IMPORT");
		ret = bst_hwcv_ioctl_import(filp, misc_dev,
					    (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to import, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_IFREE:
		dev_dbg(dev, "HWCV_IOCTL_IFREE");
		ret = bst_hwcv_ioctl_return(filp, misc_dev,
					    (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to return, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_GWARP:
		dev_dbg(dev, "HWCV_IOCTL_GWARP");
		ret = bst_hwcv_ioctl_gwarp(filp, misc_dev, (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to gwarp, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_SBS_GWARP:
		dev_dbg(dev, "HWCV_IOCTL_SBS_GWARP");
		ret = bst_hwcv_ioctl_sbs_gwarp(filp, misc_dev,
					       (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to sbs gwarp, ret = %d", ret);
			return ret;
		}
		break;

	case HWCV_IOCTL_SCALER:
		dev_dbg(dev, "HWCV_IOCTL_SCALER");
		ret = bst_hwcv_ioctl_scaler(filp, misc_dev,
					    (void __user *)args);
		if (ret < 0) {
			dev_err(dev, "Failed to scaler, ret = %d", ret);
			return ret;
		}
		break;

	default:
		dev_err(dev, "Invalid cmd: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int bst_hwcv_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct bst_hwcv_misc_dev *misc_dev;
	struct device *dev;

	misc_dev = container_of(filp->private_data, struct bst_hwcv_misc_dev,
				miscdev);
	if (!misc_dev) {
		pr_err("%s: Invalid data", __func__);
		return -EINVAL;
	}
	dev = misc_dev->dev;
	dev_dbg(dev, "-----HWCV OPEN-----");

	mutex_lock(&misc_dev->mem_manager.lock);
	ret = bst_hwcv_add_ctx(filp, &misc_dev->mem_manager);
	if (ret < 0) {
		mutex_unlock(&misc_dev->mem_manager.lock);
		return ret;
	}
	mutex_unlock(&misc_dev->mem_manager.lock);

	if (!try_module_get(THIS_MODULE)) {
		dev_err(dev, "Failed to get module");
		return -ENONET;
	}

	return 0;
}

static int bst_hwcv_close(struct inode *inode, struct file *filp)
{
	int ret;
	struct bst_hwcv_misc_dev *misc_dev;
	struct device *dev;

	misc_dev = container_of(filp->private_data, struct bst_hwcv_misc_dev,
				miscdev);
	if (!misc_dev) {
		pr_err("%s: Invalid data", __func__);
		return -EINVAL;
	}
	dev = misc_dev->dev;
	dev_dbg(dev, "-----HWCV CLOSE-----");

	mutex_lock(&misc_dev->mem_manager.lock);
	ret = bst_hwcv_del_ctx(filp, &misc_dev->mem_manager);
	if (ret < 0) {
		mutex_unlock(&misc_dev->mem_manager.lock);
		return ret;
	}
	mutex_unlock(&misc_dev->mem_manager.lock);

	module_put(THIS_MODULE);
	return 0;
}

/*--------------------------------------------------------------------------*/

static const struct file_operations bst_hwcv_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = bst_hwcv_ioctl,
	.open = bst_hwcv_open,
	.release = bst_hwcv_close,
};

int bst_hwcv_miscdev_init(struct device *dev,
			  struct bst_hwcv_misc_dev *misc_dev)
{
	int i, j;
	int ret;

	dev_info(dev, "Init hwcv misc dev.");
	mutex_init(&misc_dev->mem_manager.lock);
	mutex_init(&misc_dev->scaler_mutex);
	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++)
		mutex_init(&misc_dev->gwarp_mutex[i]);
	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++)
		for (j = 0; j < BST_HWCV_GWARP_SNR_NUM; j++)
			mutex_init(&misc_dev->sbs_gwarp_mutex[i][j]);

	misc_dev->dev = dev;
	misc_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	misc_dev->miscdev.fops = &bst_hwcv_fops;
	misc_dev->miscdev.name =
		devm_kstrdup(dev, BST_HWCV_DRIVER_NAME, GFP_KERNEL);
	misc_dev->miscdev.nodename =
		devm_kstrdup(dev, BST_HWCV_DRIVER_NAME, GFP_KERNEL);

	ret = misc_register(&misc_dev->miscdev);
	if (ret) {
		dev_err(dev, "Failed to register misc dev");
		return ret;
	}

	ret = bst_hwcv_sys_manager_init(dev, &misc_dev->sys_manager);
	if (ret < 0) {
		misc_deregister(&misc_dev->miscdev);
		return ret;
	}

	ret = bst_hwcv_mem_manager_init(dev, &misc_dev->mem_manager);
	if (ret < 0) {
		misc_deregister(&misc_dev->miscdev);
		bst_hwcv_sys_manager_exit(&misc_dev->sys_manager);
		return ret;
	}

	ret = bst_hwcv_irq_manager_init(dev, &misc_dev->irq_manager);
	if (ret < 0) {
		misc_deregister(&misc_dev->miscdev);
		bst_hwcv_sys_manager_exit(&misc_dev->sys_manager);
		bst_hwcv_mem_manager_exit(&misc_dev->mem_manager);
		return ret;
	}

	return 0;
}

void bst_hwcv_miscdev_exit(struct bst_hwcv_misc_dev *misc_dev)
{
	struct device *dev = misc_dev->dev;

	dev_info(dev, "Exit hwcv misc dev.");
	bst_hwcv_irq_manager_exit(&misc_dev->irq_manager);
	bst_hwcv_mem_manager_exit(&misc_dev->mem_manager);
	bst_hwcv_sys_manager_exit(&misc_dev->sys_manager);
	misc_deregister(&misc_dev->miscdev);
}
