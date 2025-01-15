/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_MISCDEV_H__
#define __BST_HWCV_MISCDEV_H__

#include <linux/miscdevice.h>
#include "bst_hwcv_sys_manager.h"
#include "bst_hwcv_mem_manager.h"
#include "bst_hwcv_irq_manager.h"

struct bst_hwcv_misc_dev {
	struct device *dev;
	struct mutex gwarp_mutex[BST_HWCV_GWARP_ENGINE_NUM];
	struct mutex sbs_gwarp_mutex[BST_HWCV_GWARP_ENGINE_NUM]
				    [BST_HWCV_GWARP_SNR_NUM];
	struct mutex scaler_mutex;
	struct miscdevice miscdev;
	struct bst_hwcv_irq_manager irq_manager;
	struct bst_hwcv_mem_manager mem_manager;
	struct bst_hwcv_sys_manager sys_manager;
};

int bst_hwcv_miscdev_init(struct device *dev,
			  struct bst_hwcv_misc_dev *misc_dev);
void bst_hwcv_miscdev_exit(struct bst_hwcv_misc_dev *misc_dev);

#endif
