
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

#ifndef __TOUCH_VIRT_BST_CDEV_H__
#define __TOUCH_VIRT_BST_CDEV_H__
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <touch_virt_bst.h>

int bst_touch_cdev_init(struct bst_ts_data *ts, struct file_operations *fops);
void bst_touch_cdev_remove(struct bst_ts_data *ts);

#endif /* __TOUCH_VIRT_BST_CDEV_H__ */