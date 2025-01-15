// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __BST_VIRT_DRM_DEBUGFS_H__
#define __BST_VIRT_DRM_DEBUGFS_H__

#include "bst_virt_dc/virt_dc_dev.h"
#include "bst_virt_dp/virt_dp_dev.h"
#include "bst_virt_lvds/virt_lvds_dev.h"
#include "bst_virt_mipi/virt_mipi_dev.h"

void bst_virt_drm_debugfs_init(struct bst_virt_device *virt_dev);
int bst_virt_subdev_dump_info(struct seq_file *s, uint8_t subdev);
#endif /* __BST_VIRT_DRM_DEBUGFS_H__ */