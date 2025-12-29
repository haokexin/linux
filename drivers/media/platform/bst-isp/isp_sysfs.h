/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_ISP_SYSFS_H__
#define __BST_ISP_SYSFS_H__

#include "isp_core.h"

int isp_sysfs_init(struct isp_device *isp);
void isp_sysfs_exit(struct isp_device *isp);

#endif /* __BST_ISP_SYSFS_H__ */
