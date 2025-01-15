/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_SYS_MANAGER_H__
#define __BST_HWCV_SYS_MANAGER_H__
#include <linux/device.h>
#include <linux/kobject.h>

struct bst_hwcv_sys_manager {
	struct device *dev;
	struct kobject kobj;
	int timer;
};

int bst_hwcv_sys_manager_init(struct device *dev,
			      struct bst_hwcv_sys_manager *sys_manager);
void bst_hwcv_sys_manager_exit(struct bst_hwcv_sys_manager *sys_manager);

#endif
