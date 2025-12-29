// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>

#include <bst/media-dev.h>

static const struct reg_cfg ox08b_hil_yuv_ser_init[] = {
	// clang-format off
	{ 0x0002, 0x43 },
	{ 0x0330, 0x00 },
	{ 0x0331, 0x33 },
	{ 0x0332, 0xEE },
	{ 0x0333, 0xE4 },
	{ 0x0308, 0x64 },
	{ 0x0311, 0x40 },
	{ 0x0318, 0x5E },
	{ 0x02D3, 0x84 },
	// clang-format on
};

static struct camera_cfg ox08b_hil_yuv_cfg = {
	.data_type = DT_YUV422_8B,
	.ser_init = __REG_CFGS(ox08b_hil_yuv_ser_init),
};

static struct camera_cfg *cfgs[] = {
	&ox08b_hil_yuv_cfg,
};

// clang-format off
const struct camera_cfg_set ox08b_hil_yuv_cfg_set = {
	.num = ARRAY_SIZE(cfgs),
	.cfgs = cfgs,
};
// clang-format on
