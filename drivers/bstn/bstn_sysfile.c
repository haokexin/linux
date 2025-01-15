/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_misc.c
 * @brief   This file is the source code file of sysfs file interface of BSTN
 *          driver. It contains function definitions of ioctl callbacks and
 *          initialization of the sysfs files.
 */

#include "bstn.h"

int bstn_print_level = BSTN_LOG_PRINT;
int bstn_soft_reset = 0;
int bstn_fw_debug = 0;
int bstn_fw_profiling = 0;

static ssize_t bstn_print_level_attr_show(struct kobject *object,
					  struct kobj_attribute *attr,
					  char *buf)
{
	switch (bstn_print_level) {
	case BSTN_DEBUG_PRINT:
		return sprintf(buf,
			       "Current BSTN Print Level: BSTN_DEBUG_PRINT(2)\nSet 0 "
			       "to BSTN_NO_PIRNT or 1 to BSTN_LOG_PRINT\n");
	case BSTN_LOG_PRINT:
		return sprintf(buf,
			       "Current BSTN Print Level: BSTN_LOG_PRINT(1)\nSet 0 to "
			       "BSTN_NO_PIRNT or 2 to BSTN_DEBUG_PRINT\n");
	case BSTN_NO_PRINT:
		return sprintf(buf,
			       "Current BSTN Print Level: BSTN_NO_PRINT(0)\nSet 1 to "
			       "BSTN_LOG_PIRNT or 2 to BSTN_DEBUG_PRINT\n");
	default:{
			struct bstn_device *pbstn;

			pbstn =
			    container_of((void *)object, struct bstn_device,
					 kobj);
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "fatal error! invalid print level");
			return 0;
		}
	}
}

static ssize_t bstn_print_level_attr_store(struct kobject *object,
					   struct kobj_attribute *attr,
					   const char *buf, size_t count)
{
	int new_print_level;
	int ret;

	ret = kstrtoint(buf, 0, &new_print_level);
	if (ret < 0 || new_print_level < 0
	    || new_print_level > BSTN_DEBUG_PRINT) {
		struct bstn_device *pbstn;

		pbstn = container_of((void *)object, struct bstn_device, kobj);
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "tried to set an invalid print level");
	} else {
		bstn_print_level = new_print_level;
	}
	return count;
}

static ssize_t bstn_soft_reset_attr_show(struct kobject *object,
					 struct kobj_attribute *attr, char *buf)
{
	if (bstn_soft_reset == 0) {
		return sprintf(buf,
			       "Current BSTN DSP Soft Reset: 0\nSet 1 to DO Soft "
			       "Reset for the next time driver gets opened\n");
	}

	return sprintf(buf,
		       "Current BSTN DSP Soft Reset: 1\nSet 0 to NOT DO soft "
		       "Reset for the next time driver gets opened\n");
}

static ssize_t bstn_soft_reset_attr_store(struct kobject *object,
					  struct kobj_attribute *attr,
					  const char *buf, size_t count)
{
	int new_soft_reset;
	int ret;

	ret = kstrtoint(buf, 0, &new_soft_reset);
	if (ret < 0) {
		struct bstn_device *pbstn;

		pbstn = container_of((void *)object, struct bstn_device, kobj);
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "tried to set an invalid soft reset value");
	} else {
		bstn_soft_reset = new_soft_reset;
	}
	return count;
}

static ssize_t bstn_fw_debug_attr_show(struct kobject *object,
				       struct kobj_attribute *attr, char *buf)
{
	if (bstn_fw_debug == 0) {
		return sprintf(buf,
			       "Current BSTN FW debug mode: 0\nSet 1 to enable FW "
			       "debugging mode for the next time driver gets "
			       "opened\n");
	}

	return sprintf(buf,
		       "Current BSTN FW debug mode: 1\nSet 0 to disable FW "
		       "debugging mode for the next time driver gets opened\n");
}

static ssize_t bstn_fw_debug_attr_store(struct kobject *object,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int new_fw_debug;
	int ret;
	struct bstn_device *pbstn;

	ret = kstrtoint(buf, 0, &new_fw_debug);

	pbstn = container_of((void *)object, struct bstn_device, kobj);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "tried to set an invalid soft reset value");
	} else {
		bstn_fw_debug = new_fw_debug;
		if (pbstn->state == BSTN_ONLINE) {
			struct bsnn_msg_exchange exchange_msg;

			exchange_msg.req.opcode = RT_CMD_DEBUG;
			exchange_msg.req.pdata = bstn_fw_debug;

			ret = bstn_msg_exchange(pbstn, &exchange_msg);
			// error code
			if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "bstn_msg_exchange error: %d",
					     ret);
				return ret;
			}
			// time out
			else if (ret == 0) {
				BSTN_STAGE_PRINTK
				    ("bstn_msg_exchange time out, it'll reset NET DSP "
				     "when next time driver gets opened");
				bstn_soft_reset = 1;
				return -ENOMSG;
			}
		} else {
			BSTN_STAGE_PRINTK
			    ("BSTN driver not online yet, it'll %s FW debugging mode "
			     "when next time driver gets opened",
			     bstn_fw_debug ? "enable" : "disable");
		}
	}
	return count;
}

static ssize_t bstn_fw_profiling_attr_show(struct kobject *object,
					   struct kobj_attribute *attr,
					   char *buf)
{
	if (bstn_fw_profiling == 0) {
		return sprintf(buf,
			       "Current BSTN FW profiling mode: 0\nSet 1 to enable FW "
			       "profiling mode for the next time driver gets "
			       "opened\n");
	}

	return sprintf(buf,
		       "Current BSTN FW profiling mode: 1\nSet 0 to disable FW "
		       "profiling mode for the next time driver gets opened\n");
}

static ssize_t bstn_fw_profiling_attr_store(struct kobject *object,
					    struct kobj_attribute *attr,
					    const char *buf, size_t count)
{
	int new_fw_profiling;
	int ret;
	struct bstn_device *pbstn;

	ret = kstrtoint(buf, 0, &new_fw_profiling);

	pbstn = container_of((void *)object, struct bstn_device, kobj);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "tried to set an invalid soft reset value");
	} else {
		bstn_fw_profiling = new_fw_profiling;
		if (pbstn->state == BSTN_ONLINE) {
			struct bsnn_msg_exchange exchange_msg;

			exchange_msg.req.opcode = RT_CMD_PROFILING;
			exchange_msg.req.pdata = bstn_fw_profiling;

			ret = bstn_msg_exchange(pbstn, &exchange_msg);
			// error code
			if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "bstn_msg_exchange error: %d",
					     ret);
				return ret;
			}
			// time out
			else if (ret == 0) {
				BSTN_STAGE_PRINTK
				    ("bstn_msg_exchange time out, it'll reset NET DSP "
				     "when next time driver gets opened");
				bstn_soft_reset = 1;
				return -ENOMSG;
			}
		} else {
			BSTN_STAGE_PRINTK
			    ("BSTN driver not online yet, it'll %s FW profiling "
			     "mode when next time driver gets opened",
			     bstn_fw_profiling ? "enable" : "disable");
		}
	}
	return count;
}

static ssize_t bstn_fw_msginfo_attr_show(struct kobject *object,
					 struct kobj_attribute *attr, char *buf)
{
	struct bstn_device *pbstn;

	pbstn = container_of((void *)object, struct bstn_device, kobj);
	if (pbstn->state == BSTN_ONLINE) {
		struct bstn_fw_msginfo *info = NULL;

		if (pbstn->msg_manager.msg_info == NULL) {
			return 0;
		}

		info = pbstn->msg_manager.msg_info->kern_addr;

		return sprintf(buf, "a78_irq_recved:%d,a78_irq_send:%d\n"
			       "other_irq_recved:%d,other_irq_send:%d\n"
			       "msgbx_recved:%d,msgbx_send:%d\n",
			       info->a78_irq_recved, info->a78_irq_send,
			       info->other_irq_recved, info->other_irq_send,
			       info->msgbx_recved, info->msgbx_send);
	} else {
		BSTN_STAGE_PRINTK("BSTN driver not online yet.");
		return 0;
	}
}

static ssize_t bstn_fw_msginfo_attr_store(struct kobject *object,
					  struct kobj_attribute *attr,
					  const char *buf, size_t count)
{
	return 0;
}

/* print level attribute */
static struct kobj_attribute bstn_print_level_attr = __ATTR(bstn_print_level,
							    0664,
							    bstn_print_level_attr_show,
							    bstn_print_level_attr_store);

/* software reset attribute */
static struct kobj_attribute bstn_soft_reset_attr = __ATTR(bstn_soft_reset,
							   0664,
							   bstn_soft_reset_attr_show,
							   bstn_soft_reset_attr_store);

/* debug firmware attribute */
static struct kobj_attribute bstn_fw_debug_attr = __ATTR(bstn_fw_debug,
							 0664,
							 bstn_fw_debug_attr_show,
							 bstn_fw_debug_attr_store);

/* get firmware profiling attribute */
static struct kobj_attribute bstn_fw_profiling_attr = __ATTR(bstn_fw_profiling,
							     0664,
							     bstn_fw_profiling_attr_show,
							     bstn_fw_profiling_attr_store);

/* get firmware information attribute */
static struct kobj_attribute bstn_fw_msginfo_attr = __ATTR(bstn_fw_msginfo,
							   0664,
							   bstn_fw_msginfo_attr_show,
							   bstn_fw_msginfo_attr_store);

static struct attribute *bstn_kobj_attrs[] = {
	&bstn_print_level_attr.attr,
	&bstn_soft_reset_attr.attr,
	&bstn_fw_debug_attr.attr,
	&bstn_fw_profiling_attr.attr,
	&bstn_fw_msginfo_attr.attr,
	NULL,			/* need to NULL terminate the list of attributes */
};

static struct attribute_group bstn_kobj_attr_group = {
	.attrs = bstn_kobj_attrs,
};

static void dynamic_kobj_release(struct kobject *kobj)
{
	BSTN_TRACE_PRINTK("kobject: (0x%px): %s\n", kobj, __func__);
}

static struct kobj_type dynamic_kobj_ktype = {
	.release = dynamic_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

int bstn_sysfile_init(struct bstn_device *pbstn)
{
	int ret;

	ret =
	    kobject_init_and_add(&pbstn->kobj, &dynamic_kobj_ktype, kernel_kobj,
				 "bstn");
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "failed to create bstn kobject");
		return ret;
	}

	ret = sysfs_create_group(&pbstn->kobj, &bstn_kobj_attr_group);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "failed to create bstn kobject attribute group");
		kobject_put(&pbstn->kobj);
	}
	return ret;
}

void bstn_sysfile_exit(struct bstn_device *pbstn)
{
	sysfs_remove_group(&pbstn->kobj, &bstn_kobj_attr_group);

	kobject_del(&pbstn->kobj);
	kobject_put(&pbstn->kobj);

	return;
}
