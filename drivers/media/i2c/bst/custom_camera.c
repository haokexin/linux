// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>

#include <bst/media-dev.h>

static struct camera_cfg dt_yuv420_8b_cfg = {
	.data_type = DT_YUV420_8B,
};

static struct camera_cfg dt_yuv420_10b_cfg = {
	.data_type = DT_YUV420_10B,
};

static struct camera_cfg dt_yuv420_8b_legacy_cfg = {
	.data_type = DT_YUV420_8B_LEGACY,
};

static struct camera_cfg dt_yuv420_8b_cs_cfg = {
	.data_type = DT_YUV420_8B_CS,
};

static struct camera_cfg dt_yuv420_10b_cs_cfg = {
	.data_type = DT_YUV420_10B_CS,
};

static struct camera_cfg dt_yuv422_8b_cfg = {
	.data_type = DT_YUV422_8B,
};

static struct camera_cfg dt_yuv422_10b_cfg = {
	.data_type = DT_YUV422_10B,
};

static struct camera_cfg dt_rgb444_cfg = {
	.data_type = DT_RGB444,
};

static struct camera_cfg dt_rgb555_cfg = {
	.data_type = DT_RGB555,
};

static struct camera_cfg dt_rgb565_cfg = {
	.data_type = DT_RGB565,
};

static struct camera_cfg dt_rgb666_cfg = {
	.data_type = DT_RGB666,
};

static struct camera_cfg dt_rgb888_cfg = {
	.data_type = DT_RGB888,
};

static struct camera_cfg dt_raw28_cfg = {
	.data_type = DT_RAW28,
};

static struct camera_cfg dt_raw24_cfg = {
	.data_type = DT_RAW24,
};

static struct camera_cfg dt_raw6_cfg = {
	.data_type = DT_RAW6,
};

static struct camera_cfg dt_raw7_cfg = {
	.data_type = DT_RAW7,
};

static struct camera_cfg dt_raw8_cfg = {
	.data_type = DT_RAW8,
};

static struct camera_cfg dt_raw10_cfg = {
	.data_type = DT_RAW10,
};

static struct camera_cfg dt_raw12_cfg = {
	.data_type = DT_RAW12,
};

static struct camera_cfg dt_raw14_cfg = {
	.data_type = DT_RAW14,
};

static struct camera_cfg dt_raw16_cfg = {
	.data_type = DT_RAW16,
};

static struct camera_cfg dt_raw20_cfg = {
	.data_type = DT_RAW20,
};

static struct camera_cfg dt_yuyv_cfg = {
	.data_type = DT_YUYV,
};

static struct camera_cfg *cfgs[] = {
	// clang-format off
	&dt_yuv420_8b_cfg,
	&dt_yuv420_10b_cfg,
	&dt_yuv420_8b_legacy_cfg,
	&dt_yuv420_8b_cs_cfg,
	&dt_yuv420_10b_cs_cfg,
	&dt_yuv422_8b_cfg,
	&dt_yuv422_10b_cfg,
	&dt_rgb444_cfg,
	&dt_rgb555_cfg,
	&dt_rgb565_cfg,
	&dt_rgb666_cfg,
	&dt_rgb888_cfg,
	&dt_raw28_cfg,
	&dt_raw24_cfg,
	&dt_raw6_cfg,
	&dt_raw7_cfg,
	&dt_raw8_cfg,
	&dt_raw10_cfg,
	&dt_raw12_cfg,
	&dt_raw14_cfg,
	&dt_raw16_cfg,
	&dt_raw20_cfg,
	&dt_yuyv_cfg,
	// clang-format on
};

// clang-format off
const struct camera_cfg_set custom_cfg_set = {
	.num = ARRAY_SIZE(cfgs),
	.cfgs = cfgs,
};
// clang-format on
