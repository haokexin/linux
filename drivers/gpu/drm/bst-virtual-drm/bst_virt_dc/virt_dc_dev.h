// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _VIRT_DC_DEV_H_
#define _VIRT_DC_DEV_H_

#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst_display_dc_cmdset.h"
#include "bst_virt_drm_device.h"
#include "virt_dc_dev.h"
#include <linux/hrtimer.h>

#define DC_MAX_GLB_SCL_COEFF 4
struct bst_crtc;

struct virt_dc_dev {
	struct bst_virt_device *base_dev;
	int num_submodules;
	int num_rich_layers;
	u32 enabled_layers_map;
	u32 max_line_size;
	u32 max_vsize;
	u32 support_dual_link : 1;
	u32 support_smmu_stage_1 : 1;
	uint8_t min_fw_layer_id;
#ifdef DISPLAY_SUPPORT_SCALE
	uint8_t scaler_num;
#endif
	u32 *dou_ft_coeff_addr;
	u32 *glb_scl_coeff_addr[DC_MAX_GLB_SCL_COEFF];
#ifndef __DISPLAY_EVENTS_MGR__
	struct hrtimer vblank_hrtimer;
#endif
	bool new_flush;
	struct bst_crtc* bcrtc;
	bool test_mode;
	struct hrtimer flip_hrtimer;
	ktime_t framedur_ns;
	bool timer_inited;
	u64 flush_count;
	u64 vsync_count;
};

struct bst_virt_pipe;
int virt_dc_init_submodule(struct virt_dc_dev *dc,
		       struct bst_display_submodule_header *submodule);
struct bst_virt_device *bst_virt_dc_create(struct device *dev,
					   struct bst_virt_platform_info *plat_info,
					   struct bst_virt_pipe *pipe);
void bst_virt_dc_destroy(struct bst_virt_device *vdev);

#endif /* !_VIRT_DC_DEV_H_ */
