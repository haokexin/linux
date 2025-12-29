// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>

#include <bst/media-dev.h>

static const struct reg_cfg ox03f_hil_yuv_ser_init[] = {
	// clang-format off
	{ 0x02D3, 0x00 },
	{ 0x0002, 0xF3 },
	{ 0x0383, 0x00 },
	{ 0x0318, 0x5E },
	{ 0x03F1, 0x09 },
	{ 0x03F0, 0x51 },
	{ 0x0570, 0x1C },
	{ 0x0570, 0x0C },
	{ 0x0006, 0xB0 },
	{ 0x0041, 0x76 },
	{ 0x02BF, 0x60 },
	{ 0x02BE, 0x90 },
	{ 0x02D6, 0x84 },
	{ 0x02D3, 0x90 },
	{ 0x02CD, 0x12 },
	{ 0x02BE, 0x84 },
	// clang-format on
};

static struct camera_cfg ox03f_hil_yuv_cfg = {
	.data_type = DT_YUV422_8B,
	.ser_init = __REG_CFGS(ox03f_hil_yuv_ser_init),
};

static struct camera_cfg *cfgs[] = {
	&ox03f_hil_yuv_cfg,
};

// clang-format off
const struct camera_cfg_set ox03f_hil_yuv_cfg_set = {
	.num = ARRAY_SIZE(cfgs),
	.cfgs = cfgs,
};
// clang-format on
