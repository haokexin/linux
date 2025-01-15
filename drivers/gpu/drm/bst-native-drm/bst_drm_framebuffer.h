/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#ifndef _BST_DRM_FRAMEBUFFER_H_
#define _BST_DRM_FRAMEBUFFER_H_

#include <drm/drm_framebuffer.h>
#include "bst_format_color.h"


struct bst_fb {
	struct drm_framebuffer base;
	const struct bst_format_caps *format_caps;
	bool is_va;
	u32 aligned_w;
	u32 aligned_h;
	u32 afbc_size;
	u32 offset_payload;
};

#define to_kfb(dfb)	container_of(dfb, struct bst_fb, base)

struct drm_framebuffer *
bst_fb_create(struct drm_device *dev, struct drm_file *file,
		const struct drm_mode_fb_cmd2 *mode_cmd);
int bst_fb_check_src_coords(const struct bst_fb *kfb,
			       u32 src_x, u32 src_y, u32 src_w, u32 src_h);
dma_addr_t
bst_fb_get_pixel_addr(struct bst_fb *kfb, int x, int y, int plane);
bool bst_fb_is_layer_supported(struct bst_fb *kfb, u32 layer_type,
		u32 rot);

#endif
