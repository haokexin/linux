/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef _BST_DRM_COLOR_MGMT_H_
#define _BST_DRM_COLOR_MGMT_H_

#include <linux/types.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
#include <uapi/drm/drm_fourcc.h>

#define BST_DRM_N_YUV2RGB_COEFFS		12
#define BST_DRM_N_RGB2YUV_COEFFS		12
#define BST_DRM_COLOR_PRECISION		12
#define BST_DRM_N_GAMMA_COEFFS		65
#define BST_DRM_COLOR_LUT_SIZE		BIT(BST_DRM_COLOR_PRECISION)
#define BST_DRM_N_CTM_COEFFS		9

void drm_lut_to_fgamma_coeffs(struct drm_property_blob *lut_blob, u32 *coeffs);
void drm_ctm_to_coeffs(struct drm_property_blob *ctm_blob, u32 *coeffs);

const s32 *bst_select_yuv2rgb_coeffs(u32 color_encoding, u32 color_range);


#define AFBC(x)		DRM_FORMAT_MOD_ARM_AFBC(x)


#define AFBC_16x16(x)	AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 | (x))
#define AFBC_32x8(x)	AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_32x8 | (x))
#define _YTR		AFBC_FORMAT_MOD_YTR
#define _SPLIT		AFBC_FORMAT_MOD_SPLIT
#define _SPARSE		AFBC_FORMAT_MOD_SPARSE
#define _CBR		AFBC_FORMAT_MOD_CBR
#define _TILED		AFBC_FORMAT_MOD_TILED
#define _SC		AFBC_FORMAT_MOD_SC

#define BST_DRM_FMT_RICH_LAYER		BIT(0)
#define BST_DRM_FMT_SIMPLE_LAYER		BIT(1)
#define BST_DRM_FMT_WB_LAYER		BIT(2)

#define AFBC_TH_LAYOUT_ALIGNMENT	8
#define AFBC_HEADER_SIZE		16
#define AFBC_SUPERBLK_ALIGNMENT		128
#define AFBC_SUPERBLK_PIXELS		256
#define AFBC_BODY_START_ALIGNMENT	1024
#define AFBC_TH_BODY_START_ALIGNMENT	4096

struct bst_format_caps {
	u32 hw_id;
	u32 fourcc;
	u32 supported_layer_types;
	u32 supported_rots;
	u32 supported_afbc_layouts;
	u64 supported_afbc_features;
};

struct bst_format_caps_table {
	u32 n_formats;
	const struct bst_format_caps *format_caps;
	bool (*format_mod_supported)(const struct bst_format_caps *caps,
				     u32 layer_type, u64 modifier, u32 rot);
};

extern u64 bst_supported_modifiers[];

const struct bst_format_caps *
bst_get_format_caps(struct bst_format_caps_table *table,
		       u32 fourcc, u64 modifier);

u32 bst_get_afbc_format_bpp(const struct drm_format_info *info,
			       u64 modifier);

u32 *bst_get_layer_fourcc_list(struct bst_format_caps_table *table,
				  u32 layer_type, u32 *n_fmts);

void bst_put_fourcc_list(u32 *fourcc_list);

bool bst_format_mod_supported(struct bst_format_caps_table *table,
				 u32 layer_type, u32 fourcc, u64 modifier,
				 u32 rot);

#endif /*_BST_DRM_COLOR_MGMT_H_*/
