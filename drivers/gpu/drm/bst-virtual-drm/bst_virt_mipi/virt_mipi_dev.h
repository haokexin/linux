// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _VIRT_MIPI_DEV_H_
#define _VIRT_MIPI_DEV_H_

#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst_virt_drm_device.h"

struct virt_mipi_dev {
	struct bst_virt_device *base_dev;
	u32 lanes;
	u32 format;
	unsigned long flags;
};

#define to_virt_mipi_connector(x) container_of(x, struct bst_virt_connector, base)

struct bst_virt_device *bst_virt_dsi_create(struct device *dev,
					   struct bst_virt_platform_info *plat_info,
					   struct bst_virt_pipe *pipe);
int virt_mipi_init_submodule(struct virt_mipi_dev *mipi,
		       struct bst_display_submodule_header *submodule);

#endif /* !_VIRT_MIPI_DEV_H_ */
