// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>

#include <bst/media-dev.h>

static const struct reg_cfg ox03c_hil_yuv_ser_init[] = {
	// clang-format off
	{ 0x0318, 0x5E },
	// clang-format on
};

static struct camera_cfg ox03c_hil_yuv_cfg = {
	.data_type = DT_YUV422_8B,
	.ser_init = __REG_CFGS(ox03c_hil_yuv_ser_init),
};

static struct camera_cfg *cfgs[] = {
	&ox03c_hil_yuv_cfg,
};

// clang-format off
const struct camera_cfg_set ox03c_hil_yuv_cfg_set = {
	.num = ARRAY_SIZE(cfgs),
	.cfgs = cfgs,
};
// clang-format on
