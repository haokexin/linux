// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _VIRT_DP_DEV_H_
#define _VIRT_DP_DEV_H_

#include <drm/display/drm_dp_helper.h>
#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst_virt_drm_device.h"

struct virt_dp_dev {
	struct bst_virt_device *base_dev;
	u8 *edid;
	bool trained;
	u8 colorimetry;
	u8 dynamic_range;
	bool audio_support;
};

#define to_virt_connector(x) container_of(x, struct bst_virt_connector, base)

static inline u8 firmware_dp_rate_from_drm(u32 rate)
{
	switch (rate / 1000) {
	case 162:
		return DPTX_PHYIF_CTRL_RATE_RBR;
	case 270:
		return DPTX_PHYIF_CTRL_RATE_HBR;
	case 540:
		return DPTX_PHYIF_CTRL_RATE_HBR2;
	case 810:
		return DPTX_PHYIF_CTRL_RATE_HBR3;
	default:
		return DPTX_PHYIF_CTRL_RATE_RBR;
	}
	return DPTX_PHYIF_CTRL_RATE_RBR;
}

struct bst_virt_device *bst_virt_dp_create(struct device *dev,
					   struct bst_virt_platform_info *plat_info,
					   struct bst_virt_pipe *pipe);
int virt_dp_init_submodule(struct virt_dp_dev *dp,
		       struct bst_display_submodule_header *submodule);
#endif /* !_VIRT_DP_DEV_H_ */
