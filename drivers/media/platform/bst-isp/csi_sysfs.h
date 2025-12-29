/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_SYSFS_H__
#define __BST_CSI_SYSFS_H__

struct csi_device;

int csi_sysfs_init(struct csi_device *csi);
void csi_sysfs_exit(struct csi_device *csi);

#endif /* __BST_CSI_SYSFS_H__ */
