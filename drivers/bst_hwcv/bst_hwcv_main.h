/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_MAIN_H__
#define __BST_HWCV_MAIN_H__
#include "bst_hwcv_miscdev.h"

#define HWCV_DRIVER_VERSION "0.0.1"

struct bst_hwcv_dev {
	struct platform_device *pdev;
	struct device *dev;
	struct bst_hwcv_misc_dev misc_dev;
};

#endif
