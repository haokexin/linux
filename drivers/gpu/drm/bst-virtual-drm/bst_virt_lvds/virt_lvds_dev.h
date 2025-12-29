// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _VIRT_LVDS_DEV_H_
#define _VIRT_LVDS_DEV_H_

#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst_virt_drm_device.h"

struct virt_lvds_dev {
	struct bst_virt_device *base_dev;
	int output_mode;   /* o channel  / e channel  / dual channel */
	int color_mapping; /* vesa or jeida format */
};

#define to_virt_lvds_connector(x) container_of(x, struct bst_virt_connector, base)

struct bst_virt_device *bst_virt_lvds_create(struct device *dev,
					   struct bst_virt_platform_info *plat_info,
					   struct bst_virt_pipe *pipe);
int virt_lvds_init_submodule(struct virt_lvds_dev *lvds,
		       struct bst_display_submodule_header *submodule);

#endif /* !_VIRT_LVDS_DEV_H_ */
