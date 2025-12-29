// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * BST_LWNN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file   bst_lwnn_misc.c
 * @brief  This file is the source code file of sysfs file interface of BST_LWNN
 *         driver. It contains function definitions of ioctl callbacks and
 *         initialization of the sysfs files.
 */

#include "bst_lwnn.h"

int bst_lwnn_print_level = BST_LWNN_LOG_PRINT;

/*!
 * @brief       This is the show callback function of the LWNN sysfile,
 *              bst_lwnn_refcnt.
 * @param[in]   object The pointer the sysfile kobject
 * @param[in]   attr The pointer to the sysfile kobject attribute
 * @param[in]   buf The showed string buffer
 * @return      The size of showed string - success
 *              0 - failure
 */
static ssize_t bst_lwnn_refcnt_attr_show(struct kobject *object,
					 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", *(int *)(&THIS_MODULE->refcnt));
}

/*!
 * @brief       This is the show callback function of the LWNN sysfile,
 *              bst_lwnn_print_level.
 * @param[in]   object The pointer the sysfile kobject
 * @param[in]   attr The pointer to the sysfile kobject attribute
 * @param[in]   buf The showed string buffer
 * @return      The size of showed string - success
 *              0 - failure
 */
static ssize_t bst_lwnn_print_level_attr_show(struct kobject *object,
					      struct kobj_attribute *attr,
					      char *buf)
{
	switch (bst_lwnn_print_level) {
	case BST_LWNN_DEBUG_PRINT:
		return sprintf(
			buf,
			"Current BST_LWNN Print Level: BST_LWNN_DEBUG_PRINT(2)\nSet 0 to BST_LWNN_NO_PIRNT or 1 to BST_LWNN_LOG_PRINT\n");
	case BST_LWNN_LOG_PRINT:
		return sprintf(
			buf,
			"Current BST_LWNN Print Level: BST_LWNN_LOG_PRINT(1)\nSet 0 to BST_LWNN_NO_PIRNT or 2 to BST_LWNN_DEBUG_PRINT\n");
	case BST_LWNN_NO_PRINT:
		return sprintf(
			buf,
			"Current BST_LWNN Print Level: BST_LWNN_NO_PRINT(0)\nSet 1 to BST_LWNN_LOG_PIRNT or 2 to BST_LWNN_DEBUG_PRINT\n");
	default: {
		struct bst_lwnn *pbst_lwnn;

		pbst_lwnn = container_of((void *)object, struct bst_lwnn, kobj);
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "fatal error! invalid print level");
		return 0;
	}
	}
}

/*!
 * @brief       This is the store callback function of the LWNN sysfile,
 *              bst_lwnn_print_level.
 * @param[in]   object The pointer the sysfile kobject
 * @param[in]   attr The pointer to the sysfile kobject attribute
 * @param[in]   buf The stored string buffer
 * @param[in]   count The size of the stored string
 * @return      The size of the stored string
 */
static ssize_t bst_lwnn_print_level_attr_store(struct kobject *object,
					       struct kobj_attribute *attr,
					       const char *buf, size_t count)
{
	int new_print_level;
	int ret;

	ret = kstrtoint(buf, 0, &new_print_level);
	if (ret < 0 || new_print_level < 0 ||
	    new_print_level > BST_LWNN_DEBUG_PRINT) {
		struct bst_lwnn *pbst_lwnn;

		pbst_lwnn = container_of((void *)object, struct bst_lwnn, kobj);
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "tried to set an invalid print level");
	} else {
		bst_lwnn_print_level = new_print_level;
	}
	return count;
}

static struct kobj_attribute bst_lwnn_refcnt_attr =
	__ATTR(bst_lwnn_refcnt, 0664, bst_lwnn_refcnt_attr_show, NULL);

static struct kobj_attribute bst_lwnn_print_level_attr =
	__ATTR(bst_lwnn_print_level, 0664, bst_lwnn_print_level_attr_show,
	       bst_lwnn_print_level_attr_store);

static struct attribute *bst_lwnn_kobj_attrs[] = {
	&bst_lwnn_refcnt_attr.attr, &bst_lwnn_print_level_attr.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group bst_lwnn_kobj_attr_group = {
	.attrs = bst_lwnn_kobj_attrs,
};

static void dynamic_kobj_release(struct kobject *kobj)
{
	BST_LWNN_TRACE_PRINTK("kobject: (%p): %s\n", kobj, __func__);
}

static struct kobj_type dynamic_kobj_ktype = {
	.release = dynamic_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

/*!
 * @brief       This is the initialization function of the LWNN sysfile(s).
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_sysfile_init(struct bst_lwnn *pbst_lwnn)
{
	int ret;

	ret = kobject_init_and_add(&pbst_lwnn->kobj, &dynamic_kobj_ktype,
				   kernel_kobj, BST_LWNN_DRIVER_NAME);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "failed to create bst_lwnn kobject");
		return ret;
	}

	ret = sysfs_create_group(&pbst_lwnn->kobj, &bst_lwnn_kobj_attr_group);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(
			&pbst_lwnn->pdev->dev,
			"failed to create bst_lwnn kobject attribute group");
		kobject_put(&pbst_lwnn->kobj);
	}
	return ret;
}

/*!
 * @brief       This is the exit function of the LWNN sysfile(s).
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_sysfile_exit(struct bst_lwnn *pbst_lwnn)
{
	sysfs_remove_group(&pbst_lwnn->kobj, &bst_lwnn_kobj_attr_group);

	kobject_del(&pbst_lwnn->kobj);
	kobject_put(&pbst_lwnn->kobj);

	return;
}
