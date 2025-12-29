// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/kernel.h>

#include <bst/media-dev.h>

static const struct reg_cfg imx623_ds_ser_init[] = {
	// clang-format off
	{ 0x0302, 0x10 },
	{ 0x0383, 0x00 },
	{ 0x1417, 0x00 },
	{ 0x1432, 0x7F },
	{ 0x0331, 0x33 },
	{ 0x0318, 0x6C },
	{ 0x0570, 0x00 },
	{ 0x03F0, 0x59 },
	{ 0x03F1, 0x89 },
	{ 0x0003, 0x03 },
	{ 0x0006, 0xB0 },
	{ 0x02D3, 0x00 },
	{ 0x02D3, 0x10 },
	{ 0x02D6, 0x84 },
	{ 0x02D7, 0x27 },
	{ 0x02D8, 0x47 },
	// clang-format on
};

static struct camera_cfg imx623_raw12_cfg = {
	.data_type = DT_RAW12,
	.ser_init = __REG_CFGS(imx623_ds_ser_init),
};

static struct camera_cfg *cfgs[] = {
	&imx623_raw12_cfg,
};

// clang-format off
const struct camera_cfg_set imx623_raw_cfg_set = {
	.num = ARRAY_SIZE(cfgs),
	.cfgs = cfgs,
};
// clang-format on
