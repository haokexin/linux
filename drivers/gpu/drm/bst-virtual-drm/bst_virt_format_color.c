// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/slab.h>
#include "bst_virt_utils.h"
#include "bst_virt_format_color.h"
#include "bst-fw-msg/firmware_cmdsets/bst_display_dc_cmdset.h"
#include "bst_virt_drm_device.h"

static const s32 yuv2rgb_bt601_narrow[BST_DRM_N_YUV2RGB_COEFFS] = {
	1192,    0, 1634,
	1192, -401, -832,
	1192, 2066,    0,
	  64,  512,  512
};

static const s32 yuv2rgb_bt601_wide[BST_DRM_N_YUV2RGB_COEFFS] = {
	1024,    0, 1436,
	1024, -352, -731,
	1024, 1815,    0,
	   0,  512,  512
};

static const s32 yuv2rgb_bt709_narrow[BST_DRM_N_YUV2RGB_COEFFS] = {
	1192,    0, 1836,
	1192, -218, -546,
	1192, 2163,    0,
	  64,  512,  512
};

static const s32 yuv2rgb_bt709_wide[BST_DRM_N_YUV2RGB_COEFFS] = {
	1024,    0, 1613,
	1024, -192, -479,
	1024, 1900,    0,
	   0,  512,  512
};

static const s32 yuv2rgb_bt2020[BST_DRM_N_YUV2RGB_COEFFS] = {
	1024,    0, 1476,
	1024, -165, -572,
	1024, 1884,    0,
	   0,  512,  512
};

const s32 *bst_select_yuv2rgb_coeffs(u32 color_encoding, u32 color_range)
{
	bool narrow = color_range == DRM_COLOR_YCBCR_LIMITED_RANGE;
	const s32 *coeffs;

	switch (color_encoding) {
	case DRM_COLOR_YCBCR_BT709:
		coeffs = narrow ? yuv2rgb_bt709_narrow : yuv2rgb_bt709_wide;
		break;
	case DRM_COLOR_YCBCR_BT601:
		coeffs = narrow ? yuv2rgb_bt601_narrow : yuv2rgb_bt601_wide;
		break;
	case DRM_COLOR_YCBCR_BT2020:
		coeffs = yuv2rgb_bt2020;
		break;
	default:
		coeffs = NULL;
		break;
	}

	return coeffs;
}

struct gamma_curve_sector {
	u32 boundary_start;
	u32 num_of_segments;
	u32 segment_width;
};

struct gamma_curve_segment {
	u32 start;
	u32 end;
};

static struct gamma_curve_sector sector_tbl[] = {
	{ 0,    4,  4   },
	{ 16,   4,  4   },
	{ 32,   4,  8   },
	{ 64,   4,  16  },
	{ 128,  4,  32  },
	{ 256,  4,  64  },
	{ 512,  16, 32  },
	{ 1024, 24, 128 },
};

static void drm_lut_to_coeffs(struct drm_property_blob *lut_blob, u32 *coeffs,
			      struct gamma_curve_sector *sector_tbl, u32 num_sectors)
{
	struct drm_color_lut *lut;
	u32 i, j, in, num = 0;

	if (!lut_blob)
		return;

	lut = lut_blob->data;

	for (i = 0; i < num_sectors; i++) {
		for (j = 0; j < sector_tbl[i].num_of_segments; j++) {
			in = sector_tbl[i].boundary_start +
			     j * sector_tbl[i].segment_width;

			coeffs[num++] = drm_color_lut_extract(
				lut[in].red, BST_DRM_COLOR_PRECISION);
		}
	}

	coeffs[num] = BIT(BST_DRM_COLOR_PRECISION);
}

void drm_lut_to_fgamma_coeffs(struct drm_property_blob *lut_blob, u32 *coeffs)
{
	drm_lut_to_coeffs(lut_blob, coeffs, sector_tbl, ARRAY_SIZE(sector_tbl));
}

void drm_ctm_to_coeffs(struct drm_property_blob *ctm_blob, u32 *coeffs)
{
	struct drm_color_ctm *ctm;
	u32 i;

	if (!ctm_blob)
		return;

	ctm = ctm_blob->data;

	for (i = 0; i < BST_DRM_N_CTM_COEFFS; i++)
		coeffs[i] = drm_color_ctm_s31_32_to_qm_n(ctm->matrix[i], 3, 12);
}

const struct bst_format_caps *
bst_get_format_caps(struct bst_format_caps_table *table,
		       u32 fourcc, u64 modifier)
{
	const struct bst_format_caps *caps;
	u64 afbc_features = modifier & ~(AFBC_FORMAT_MOD_BLOCK_SIZE_MASK);
	u32 afbc_layout = modifier & AFBC_FORMAT_MOD_BLOCK_SIZE_MASK;
	int id;

	for (id = 0; id < table->n_formats; id++) {
		caps = &table->format_caps[id];

		if (fourcc != caps->fourcc)
			continue;

		if ((modifier == 0ULL) && (caps->supported_afbc_layouts == 0))
			return caps;

		if (has_bits(afbc_features, caps->supported_afbc_features) &&
		    has_bit(afbc_layout, caps->supported_afbc_layouts))
			return caps;
	}

	return NULL;
}

u32 bst_get_afbc_format_bpp(const struct drm_format_info *info, u64 modifier)
{
	u32 bpp;

	switch (info->format) {
	case DRM_FORMAT_YUV420_8BIT:
		bpp = 12;
		break;
	case DRM_FORMAT_YUV420_10BIT:
		bpp = 15;
		break;
	default:
		bpp = info->cpp[0] * 8;
		break;
	}

	return bpp;
}

u64 bst_supported_modifiers[] = {
	AFBC_16x16(0),
	AFBC_16x16(_SPARSE),
	AFBC_16x16(_YTR | _SPARSE),
	AFBC_16x16(_YTR),
	AFBC_16x16(_SPLIT | _SPARSE | _YTR),
	AFBC_16x16(_TILED | _SPARSE),
	AFBC_16x16(_TILED),
	AFBC_16x16(_TILED | _SC | _SPLIT | _SPARSE | _YTR),
	AFBC_16x16(_TILED | _SC | _SPARSE | _YTR),
	AFBC_16x16(_TILED | _SC | _YTR),
	AFBC_32x8(_YTR | _SPARSE),
	AFBC_32x8(_YTR),
	AFBC_32x8(_SPLIT | _SPARSE | _YTR),
	AFBC_32x8(_TILED | _SC | _SPLIT | _SPARSE | _YTR),
	AFBC_32x8(_TILED | _SC | _SPARSE | _YTR),
	AFBC_32x8(_TILED | _SC | _YTR),
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

bool bst_format_mod_supported(struct bst_format_caps_table *table,
				 u32 layer_type, u32 fourcc, u64 modifier,
				 u32 rot)
{
	const struct bst_format_caps *caps;

	caps = bst_get_format_caps(table, fourcc, modifier);
	if (!caps)
		return false;

	if (!(caps->supported_layer_types & layer_type))
		return false;

	if (table->format_mod_supported)
		return table->format_mod_supported(caps, layer_type, modifier,
						   rot);

	return true;
}

u32 *bst_get_layer_fourcc_list(struct bst_format_caps_table *table,
			       u32 layer_type, u32 *n_fmts)
{
	const struct bst_format_caps *cap;
	u32 *fmts;
	int i, j, n = 0;

	fmts = kcalloc(table->n_formats, sizeof(u32), GFP_KERNEL);
	if (!fmts)
		return NULL;

	for (i = 0; i < table->n_formats; i++) {
		cap = &table->format_caps[i];
		if (!(layer_type & cap->supported_layer_types) ||
		    (cap->fourcc == 0))
			continue;

		for (j = n - 1; j >= 0; j--)
			if (fmts[j] == cap->fourcc)
				break;

		if (j < 0)
			fmts[n++] = cap->fourcc;
	}

	if (n_fmts)
		*n_fmts = n;

	return fmts;
}

void bst_put_fourcc_list(u32 *fourcc_list)
{
	kfree(fourcc_list);
}