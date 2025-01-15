// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_print.h>

#include "../bst_disp_conn.h"
#include "../bst_dpu_csr.h"
#include "lvds_conn.h"

struct bst_lvds;

#define connector_to_lvds_ch(c) \
		container_of(c, struct bst_lvds_channel, connector)

#define encoder_to_lvds_ch(c) \
		container_of(c, struct bst_lvds_channel, encoder)

#define LVDS_CHANNEL_1  0
#define LVDS_CHANNEL_2  1

static u8 lvds0_edid[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x0A, 0x74, 0x29, 0x09,
	0xB1, 0x04, 0x00, 0x00, 0x1E, 0x21, 0x01, 0x04, 0xB5, 0x35, 0x1E, 0x78,
	0x23, 0xA6, 0x31, 0xA8, 0x55, 0x51, 0x9D, 0x25, 0x0F, 0x50, 0x54, 0x00,
	0x00, 0x00, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x30, 0x4C, 0x2D, 0x55, 0x0F, 0x01, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x4C,
	0x56, 0x44, 0x53, 0x2D, 0x30, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49,
};

static u8 lvds1_edid[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x0A, 0x74, 0x29, 0x09,
	0xB2, 0x04, 0x00, 0x00, 0x1E, 0x21, 0x01, 0x04, 0xB5, 0x35, 0x1E, 0x78,
	0x23, 0xA6, 0x31, 0xA8, 0x55, 0x51, 0x9D, 0x25, 0x0F, 0x50, 0x54, 0x00,
	0x00, 0x00, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,
	0x00, 0x00, 0x00, 0xFD, 0x00, 0x30, 0x4C, 0x2D, 0x55, 0x0F, 0x01, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x4C,
	0x56, 0x44, 0x53, 0x2D, 0x31, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48,
};

struct bst_lvds_channel {
	struct bst_lvds *lvds;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	int output;  /* o channel  / e channel  / dual channel */
	int format;		/* vesa or jeida format */
	int width;
	int height;
	int chno;
	struct device *host_dpu;
};


struct bst_lvds {
	struct device *dev;
	void __iomem *regs;
	struct drm_device *drm_dev;
	struct bst_lvds_channel channel[2];
	struct lvds_formats lvds_fmt;
	unsigned int lvds_arrary[LVDS_RGB_MAX_LANES][LVDS_LANE_BIT];
	int high_bit;
};

static int lvds_formats_init(struct bst_lvds *lvds)
{
	// R
	lvds->lvds_fmt.r[0] = 0x00;
	lvds->lvds_fmt.r[1] = 0x01;
	lvds->lvds_fmt.r[2] = 0x02;
	lvds->lvds_fmt.r[3] = 0x03;
	lvds->lvds_fmt.r[4] = 0x04;
	lvds->lvds_fmt.r[5] = 0x05;
	lvds->lvds_fmt.r[6] = 0x06;
	lvds->lvds_fmt.r[7] = 0x07;
	lvds->lvds_fmt.r[8] = 0x08;
	lvds->lvds_fmt.r[9] = 0x09;
	// G
	lvds->lvds_fmt.g[0] = 0x0a;
	lvds->lvds_fmt.g[1] = 0x0b;
	lvds->lvds_fmt.g[2] = 0x0c;
	lvds->lvds_fmt.g[3] = 0x0d;
	lvds->lvds_fmt.g[4] = 0x0e;
	lvds->lvds_fmt.g[5] = 0x0f;
	lvds->lvds_fmt.g[6] = 0x10;
	lvds->lvds_fmt.g[7] = 0x11;
	lvds->lvds_fmt.g[8] = 0x12;
	lvds->lvds_fmt.g[9] = 0x13;
	// B
	lvds->lvds_fmt.b[0] = 0x14;
	lvds->lvds_fmt.b[1] = 0x15;
	lvds->lvds_fmt.b[2] = 0x16;
	lvds->lvds_fmt.b[3] = 0x17;
	lvds->lvds_fmt.b[4] = 0x18;
	lvds->lvds_fmt.b[5] = 0x19;
	lvds->lvds_fmt.b[6] = 0x1a;
	lvds->lvds_fmt.b[7] = 0x1b;
	lvds->lvds_fmt.b[8] = 0x1c;
	lvds->lvds_fmt.b[9] = 0x1d;
	// data_en
	lvds->lvds_fmt.vsync = 0x1e;
	lvds->lvds_fmt.hsync = 0x1f;
	lvds->lvds_fmt.data_en = 0x20;
	lvds->lvds_fmt.res0 = 0x21;
	lvds->lvds_fmt.res1 = 0x22;
	lvds->lvds_fmt.defaults = 0x23;
	return 0;
}

static int combine_rgb(struct bst_lvds *lvds, int format)
{
	DRM_INFO("choose format:%d high_bit:%d!\n",format,lvds->high_bit);
	switch (format) {
	case LVDS_NOLINEAR_12:
		// Non Linear step size
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[2 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[0 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[2 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.r[2 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.r[0 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.r[3 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[1 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[0 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.res0;
		break;
	case LVDS_LINEAR_12:
		// Linear step size
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.res1;
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[2 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[0 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.res1;

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.res1;
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[3 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[1 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[0 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.res0;

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[1 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[0 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		break;
	case LVDS_JEIDA_18:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[7 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[6 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[5 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[4 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[2 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[7 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[6 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[5 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[4 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[3 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[7 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[6 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[5 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[4 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		break;
	case LVDS_VESA_18:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[0 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[5 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[4 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[2 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[0 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[1 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[0 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[5 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[4 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[3 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[1 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[5 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[4 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[2 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		break;
	case LVDS_JEIDA_24:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[7 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[6 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[5 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[4 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[2 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[7 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[6 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[5 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[4 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[3 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[7 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[6 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[5 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[4 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.defaults;
		lvds->lvds_arrary[3][1] = lvds->lvds_fmt.b[1 + lvds->high_bit];
		lvds->lvds_arrary[3][2] = lvds->lvds_fmt.b[0 + lvds->high_bit];
		lvds->lvds_arrary[3][3] = lvds->lvds_fmt.g[1 + lvds->high_bit];
		lvds->lvds_arrary[3][4] = lvds->lvds_fmt.g[0 + lvds->high_bit];
		lvds->lvds_arrary[3][5] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[3][6] = lvds->lvds_fmt.r[0 + lvds->high_bit];
		break;
	case LVDS_VESA_24:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[0 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[5 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[4 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[2 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[1 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[0 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[1 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[0 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[5 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[4 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[3 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[1 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[5 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[4 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[2 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.defaults;
		lvds->lvds_arrary[3][1] = lvds->lvds_fmt.b[7 + lvds->high_bit];
		lvds->lvds_arrary[3][2] = lvds->lvds_fmt.b[6 + lvds->high_bit];
		lvds->lvds_arrary[3][3] = lvds->lvds_fmt.g[7 + lvds->high_bit];
		lvds->lvds_arrary[3][4] = lvds->lvds_fmt.g[6 + lvds->high_bit];
		lvds->lvds_arrary[3][5] = lvds->lvds_fmt.r[7 + lvds->high_bit];
		lvds->lvds_arrary[3][6] = lvds->lvds_fmt.r[6 + lvds->high_bit];
		break;
	case LVDS_FORMAT3_24:

		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[2 + lvds->high_bit];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[7 + lvds->high_bit];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[6 + lvds->high_bit];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[5 + lvds->high_bit];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[4 + lvds->high_bit];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[3 + lvds->high_bit];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[2 + lvds->high_bit];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[3 + lvds->high_bit];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[2 + lvds->high_bit];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[7 + lvds->high_bit];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[6 + lvds->high_bit];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[5 + lvds->high_bit];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[4 + lvds->high_bit];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[3 + lvds->high_bit];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[7 + lvds->high_bit];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[6 + lvds->high_bit];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[5 + lvds->high_bit];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[4 + lvds->high_bit];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;

		break;
	case LVDS_JEIDA_30:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[4];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[9];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[8];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[7];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[6];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[5];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[4];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[5];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[4];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[9];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[8];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[7];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[6];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[5];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[9];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[8];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[7];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[6];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[3][1] = lvds->lvds_fmt.b[3];
		lvds->lvds_arrary[3][2] = lvds->lvds_fmt.b[2];
		lvds->lvds_arrary[3][3] = lvds->lvds_fmt.g[3];
		lvds->lvds_arrary[3][4] = lvds->lvds_fmt.g[2];
		lvds->lvds_arrary[3][5] = lvds->lvds_fmt.r[3];
		lvds->lvds_arrary[3][6] = lvds->lvds_fmt.r[2];

		lvds->lvds_arrary[4][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[4][1] = lvds->lvds_fmt.b[1];
		lvds->lvds_arrary[4][2] = lvds->lvds_fmt.b[0];
		lvds->lvds_arrary[4][3] = lvds->lvds_fmt.g[1];
		lvds->lvds_arrary[4][4] = lvds->lvds_fmt.g[0];
		lvds->lvds_arrary[4][5] = lvds->lvds_fmt.r[1];
		lvds->lvds_arrary[4][6] = lvds->lvds_fmt.r[0];
		break;
	case LVDS_VESA_30:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[0];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[5];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[4];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[3];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[2];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[1];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[0];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[1];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[0];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[5];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[4];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[3];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[2];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[1];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[5];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[4];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[3];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[2];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[3][1] = lvds->lvds_fmt.b[7];
		lvds->lvds_arrary[3][2] = lvds->lvds_fmt.b[6];
		lvds->lvds_arrary[3][3] = lvds->lvds_fmt.g[7];
		lvds->lvds_arrary[3][4] = lvds->lvds_fmt.g[6];
		lvds->lvds_arrary[3][5] = lvds->lvds_fmt.r[7];
		lvds->lvds_arrary[3][6] = lvds->lvds_fmt.r[6];

		lvds->lvds_arrary[4][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[4][1] = lvds->lvds_fmt.b[9];
		lvds->lvds_arrary[4][2] = lvds->lvds_fmt.b[8];
		lvds->lvds_arrary[4][3] = lvds->lvds_fmt.g[9];
		lvds->lvds_arrary[4][4] = lvds->lvds_fmt.g[8];
		lvds->lvds_arrary[4][5] = lvds->lvds_fmt.r[9];
		lvds->lvds_arrary[4][6] = lvds->lvds_fmt.r[8];
		break;
	case LVDS_FORMAT3_30:
		lvds->lvds_arrary[0][0] = lvds->lvds_fmt.g[2];
		lvds->lvds_arrary[0][1] = lvds->lvds_fmt.r[7];
		lvds->lvds_arrary[0][2] = lvds->lvds_fmt.r[6];
		lvds->lvds_arrary[0][3] = lvds->lvds_fmt.r[5];
		lvds->lvds_arrary[0][4] = lvds->lvds_fmt.r[4];
		lvds->lvds_arrary[0][5] = lvds->lvds_fmt.r[3];
		lvds->lvds_arrary[0][6] = lvds->lvds_fmt.r[2];

		lvds->lvds_arrary[1][0] = lvds->lvds_fmt.b[3];
		lvds->lvds_arrary[1][1] = lvds->lvds_fmt.b[2];
		lvds->lvds_arrary[1][2] = lvds->lvds_fmt.g[7];
		lvds->lvds_arrary[1][3] = lvds->lvds_fmt.g[6];
		lvds->lvds_arrary[1][4] = lvds->lvds_fmt.g[5];
		lvds->lvds_arrary[1][5] = lvds->lvds_fmt.g[4];
		lvds->lvds_arrary[1][6] = lvds->lvds_fmt.g[3];

		lvds->lvds_arrary[2][0] = lvds->lvds_fmt.data_en;
		lvds->lvds_arrary[2][1] = lvds->lvds_fmt.vsync;
		lvds->lvds_arrary[2][2] = lvds->lvds_fmt.hsync;
		lvds->lvds_arrary[2][3] = lvds->lvds_fmt.b[7];
		lvds->lvds_arrary[2][4] = lvds->lvds_fmt.b[6];
		lvds->lvds_arrary[2][5] = lvds->lvds_fmt.b[5];
		lvds->lvds_arrary[2][6] = lvds->lvds_fmt.b[4];

		lvds->lvds_arrary[3][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[3][1] = lvds->lvds_fmt.b[9];
		lvds->lvds_arrary[3][2] = lvds->lvds_fmt.b[8];
		lvds->lvds_arrary[3][3] = lvds->lvds_fmt.g[9];
		lvds->lvds_arrary[3][4] = lvds->lvds_fmt.g[8];
		lvds->lvds_arrary[3][5] = lvds->lvds_fmt.r[9];
		lvds->lvds_arrary[3][6] = lvds->lvds_fmt.r[8];

		lvds->lvds_arrary[4][0] = lvds->lvds_fmt.res0;
		lvds->lvds_arrary[4][1] = lvds->lvds_fmt.b[1];
		lvds->lvds_arrary[4][2] = lvds->lvds_fmt.b[0];
		lvds->lvds_arrary[4][3] = lvds->lvds_fmt.g[1];
		lvds->lvds_arrary[4][4] = lvds->lvds_fmt.g[0];
		lvds->lvds_arrary[4][5] = lvds->lvds_fmt.r[1];
		lvds->lvds_arrary[4][6] = lvds->lvds_fmt.r[0];
		break;
	default:
		DRM_DEV_ERROR(lvds->dev, "err lvds fmt\n");
		break;
	}
	return 0;
}

static inline int bst_lvds_name_to_format(const char *s)
{
	if (s == NULL)
		return -EINVAL;
	if (strncmp(s, "jeida-30", 8) == 0)
		return LVDS_JEIDA_30;
	else if (strncmp(s, "vesa-30", 7) == 0)
		return LVDS_VESA_30;
	else if (strncmp(s, "jeida-24", 8) == 0)
		return LVDS_JEIDA_24;
	else if (strncmp(s, "vesa-24", 7) == 0)
		return LVDS_VESA_24;
	else if (strncmp(s, "jeida-18", 8) == 0)
		return LVDS_JEIDA_18;
	else if (strncmp(s, "vesa-18", 8) == 0)
		return LVDS_VESA_18;
	else if (strncmp(s, "linear-12", 9) == 0)
		return LVDS_LINEAR_12;
	else if (strncmp(s, "nolinear-12", 11) == 0)
		return LVDS_NOLINEAR_12;
	else if (strncmp(s, "format3-24", 10) == 0)
		return LVDS_FORMAT3_24;
	else if (strncmp(s, "format3-30", 10) == 0)
		return LVDS_FORMAT3_30;
	return -EINVAL;
}

static const struct drm_connector_funcs bst_lvds_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int bst_panel_timing_to_edid(u8 *edid, struct drm_connector *connector)
{
	struct drm_display_mode *pmode, *pt;
	uint8_t *preferred_vic = &edid[0x36];
	list_for_each_entry_safe(pmode, pt, &connector->probed_modes, head) {
		if (DRM_MODE_TYPE_PREFERRED & pmode->type) {
			break;
		}
	}

	bst_conn_edid_byte_gen(&preferred_vic[1], 0, 8, &preferred_vic[0], 0, 8,
			       pmode->clock * 1000 / 10000);

	bst_conn_edid_byte_gen(&preferred_vic[4], 4, 4, &preferred_vic[2], 0, 8,
			       pmode->hdisplay);

	bst_conn_edid_byte_gen(&preferred_vic[4], 0, 4, &preferred_vic[3], 0, 8,
			       pmode->htotal - pmode->hdisplay);
	bst_conn_edid_byte_gen(&preferred_vic[11], 6, 2, &preferred_vic[8], 0,
			       8, pmode->hsync_start - pmode->hdisplay);
	bst_conn_edid_byte_gen(&preferred_vic[11], 4, 2, &preferred_vic[9], 0,
			       8, pmode->hsync_end - pmode->hsync_start);
	bst_conn_edid_byte_gen(&preferred_vic[14], 4, 4, &preferred_vic[12], 0,
			       8, pmode->width_mm);

	bst_conn_edid_byte_gen(&preferred_vic[7], 4, 4, &preferred_vic[5], 0, 8,
			       pmode->vdisplay);
	bst_conn_edid_byte_gen(&preferred_vic[7], 0, 4, &preferred_vic[6], 0, 8,
			       pmode->vtotal - pmode->vdisplay);
	bst_conn_edid_byte_gen(&preferred_vic[11], 2, 2, &preferred_vic[10], 4,
			       4, pmode->vsync_start - pmode->vdisplay);
	bst_conn_edid_byte_gen(&preferred_vic[11], 0, 2, &preferred_vic[10], 0,
			       4, pmode->vsync_end - pmode->vsync_start);
	bst_conn_edid_byte_gen(&preferred_vic[14], 0, 4, &preferred_vic[13], 0,
			       8, pmode->height_mm);

	bst_conn_edid_bit_change(&preferred_vic[17], 7, 1,
				 pmode->flags & DRM_MODE_FLAG_INTERLACE);
	bst_conn_edid_bit_change(&preferred_vic[17], 2, 1,
				 pmode->flags & DRM_MODE_FLAG_PHSYNC);
	bst_conn_edid_bit_change(&preferred_vic[17], 1, 1,
				 pmode->flags & DRM_MODE_FLAG_PVSYNC);
	edid[127] = bst_edid_block_checksum(edid);

	return 0;
}

static int bst_lvds_connector_get_modes(struct drm_connector *connector)
{
	struct bst_lvds_channel *channel = connector_to_lvds_ch(connector);
	struct drm_panel *panel = channel->panel;
	u8 *edid_data = channel->chno ? lvds1_edid : lvds0_edid;
	int mode_num;

	mode_num = drm_panel_get_modes(panel, connector);

	if (mode_num) {
		bst_panel_timing_to_edid(edid_data, connector);
		drm_connector_update_edid_property(connector,  (struct edid *)edid_data);
	}

	return mode_num;
}

static const
struct drm_connector_helper_funcs bst_lvds_connector_helper_funcs = {
	.get_modes = bst_lvds_connector_get_modes,
};

static int
bst_lvds_encoder_atomic_check(struct drm_encoder *encoder,
			      struct drm_crtc_state *crtc_state,
			      struct drm_connector_state *conn_state)
{
	return 0;
}

void bst_lvds_control_enable(struct bst_lvds_channel *channel, struct drm_display_mode *mode)
{
	unsigned int ctl_reg = 0;
	unsigned int ch_mux_reg = 0;
	struct bst_lvds *lvds = channel->lvds;

	WRITEL(0xabcd1234, lvds->regs + LVDS_REG_WR_PROTECT);
	ctl_reg = readl(lvds->regs + LVDS_CONTROL_REG);
	ch_mux_reg = readl(lvds->regs + LVDS_CHANNEL_MUX);

	if (channel->chno == LVDS_CHANNEL_1) {
		switch (channel->output) {
		case DISPLAY_OUTPUT_O_LVDS:
			ctl_reg |= BIT(10);//ch_switch
			ctl_reg &= (~BIT(7));//ch_mode_sel
			ctl_reg |= BIT(8);//oclk
			ch_mux_reg |= BIT(7);	//single_ch_2_o
			break;
		case DISPLAY_OUTPUT_E_LVDS:
			ctl_reg |= BIT(10);	//ch_switch
			ctl_reg &= (~BIT(7));	//ch_mode_sel
			ctl_reg |= BIT(9);//eclk
			ch_mux_reg &= (~BIT(7));	//single_ch_2_o
			break;
		case DISPLAY_OUTPUT_DUAL_LVDS:
			ctl_reg |= BIT(10);	//ch_switch
			ctl_reg |= BIT(7);	//ch_mode_sel
			ctl_reg |= BIT(9);//eclk
			ctl_reg |= BIT(8);//oclk
			break;
		default:
			ctl_reg &= (~BIT(10));	//CLOSE channel 1
			//ctl_reg &=(~BIT(9));//CLOSE eclk
			//ctl_reg &=(~BIT(8));//CLOSE oclk
		}
		if (mode->htotal && mode->vtotal)
			WRITEL(mode->htotal * mode->vtotal, lvds->regs + LVDS_PIXEL_NUMBER);
	}
	if (channel->chno == LVDS_CHANNEL_2) {
		switch (channel->output) {
		case DISPLAY_OUTPUT_O_LVDS:
			ctl_reg |= BIT(22);	//ch_switch
			ctl_reg &= (~BIT(19));	//ch_mode_sel
			ctl_reg |= BIT(20);//oclk
			ch_mux_reg |= BIT(6);	//single_ch_2_o
			break;
		case DISPLAY_OUTPUT_E_LVDS:
			ctl_reg |= BIT(22);	//ch_switch
			ctl_reg &= (~BIT(19));	//ch_mode_sel
			ctl_reg |= BIT(21);//eclk
			ch_mux_reg &= (~BIT(6));	//single_ch_2_o
			break;
		case DISPLAY_OUTPUT_DUAL_LVDS:
			ctl_reg |= BIT(22);	//ch_switch
			ctl_reg |= BIT(19);	//ch_mode_sel
			ctl_reg |= BIT(21);//eclk
			ctl_reg |= BIT(20);//oclk
			break;
		default:
			ctl_reg &= (~BIT(22));	//CLOSE channel 2
			//ctl_reg &=(~BIT(21));//CLOSE eclk
			//ctl_reg &=(~BIT(20));//CLOSE oclk
		}
		if (mode->htotal && mode->vtotal)
			WRITEL(mode->htotal * mode->vtotal, lvds->regs + LVDS_PIXEL_NUMBER_CH2);
	}
	WRITEL(ctl_reg, lvds->regs + LVDS_CONTROL_REG);
	WRITEL(ch_mux_reg, lvds->regs + LVDS_CHANNEL_MUX);
}

void bst_lvds_control_config(struct bst_lvds_channel *channel)
{
	struct bst_lvds *lvds = channel->lvds;

	if (channel->format <= LVDS_FORMAT3_30)
		lvds->high_bit = 0;
	else
		lvds->high_bit = 2;
	if (channel->chno == LVDS_CHANNEL_1) {
		if (channel->output != DISPLAY_OUTPUT_LVDS_INVALID  && channel->output  != DISPLAY_OUTPUT_DUAL_LVDS) {
			DRM_INFO("channel1 format:%d!\n",channel->format);
			combine_rgb(lvds, channel->format);	//format1
			WRITEL(CH1_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A0_SEL_CFG);
			WRITEL(CH1_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A1_SEL_CFG);
			WRITEL(CH1_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A2_SEL_CFG);
			WRITEL(CH1_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A3_SEL_CFG);
			WRITEL(CH1_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A4_SEL_CFG);
			WRITEL(CH1_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A5_SEL_CFG);
			WRITEL(CH1_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A6_SEL_CFG);
		}else if(DISPLAY_OUTPUT_DUAL_LVDS == DISPLAY_OUTPUT_DUAL_LVDS){
			DRM_INFO("channel1 DUAL lvds config!\n");
			combine_rgb(lvds, channel->format);	//format1
			WRITEL(CH1_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A0_SEL_CFG);
			WRITEL(CH1_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A1_SEL_CFG);
			WRITEL(CH1_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A2_SEL_CFG);
			WRITEL(CH1_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A3_SEL_CFG);
			WRITEL(CH1_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A4_SEL_CFG);
			WRITEL(CH1_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A5_SEL_CFG);
			WRITEL(CH1_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A6_SEL_CFG);

			WRITEL(CH2_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A0_SEL_CFG);
			WRITEL(CH2_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A1_SEL_CFG);
			WRITEL(CH2_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A2_SEL_CFG);
			WRITEL(CH2_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A3_SEL_CFG);
			WRITEL(CH2_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A4_SEL_CFG);
			WRITEL(CH2_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A5_SEL_CFG);
			WRITEL(CH2_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A6_SEL_CFG);
		}
	}
	if (channel->chno == LVDS_CHANNEL_2) {
		if (channel->output != DISPLAY_OUTPUT_LVDS_INVALID  && channel->output  != DISPLAY_OUTPUT_DUAL_LVDS) {
			DRM_INFO("channel2 format:%d!\n",channel->format);
			combine_rgb(lvds, channel->format);	//format2
			WRITEL(CH2_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A0_SEL_CFG);
			WRITEL(CH2_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A1_SEL_CFG);
			WRITEL(CH2_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A2_SEL_CFG);
			WRITEL(CH2_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A3_SEL_CFG);
			WRITEL(CH2_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A4_SEL_CFG);
			WRITEL(CH2_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A5_SEL_CFG);
			WRITEL(CH2_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A6_SEL_CFG);
		}else if(DISPLAY_OUTPUT_DUAL_LVDS == DISPLAY_OUTPUT_DUAL_LVDS){
			DRM_INFO("channel2 DUAL lvds config!\n");
			combine_rgb(lvds, channel->format);	//format1
			WRITEL(CH1_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A0_SEL_CFG);
			WRITEL(CH1_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A1_SEL_CFG);
			WRITEL(CH1_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A2_SEL_CFG);
			WRITEL(CH1_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A3_SEL_CFG);
			WRITEL(CH1_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A4_SEL_CFG);
			WRITEL(CH1_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A5_SEL_CFG);
			WRITEL(CH1_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH1_A6_SEL_CFG);

			WRITEL(CH2_A0_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A0_SEL_CFG);
			WRITEL(CH2_A1_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A1_SEL_CFG);
			WRITEL(CH2_A2_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A2_SEL_CFG);
			WRITEL(CH2_A3_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A3_SEL_CFG);
			WRITEL(CH2_A4_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A4_SEL_CFG);
			WRITEL(CH2_A5_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A5_SEL_CFG);
			WRITEL(CH2_A6_SEL_CFG(lvds->lvds_arrary),
				lvds->regs + LVDS_CH2_A6_SEL_CFG);
		}
	}

}
void bst_lvds_control_reset(struct bst_lvds_channel *channel)
{
	struct bst_lvds *lvds = channel->lvds;
	WRITEL((unsigned int)apb_local_resetn|ch2_local_reset_n|ch1_local_reset_n, 	lvds->regs + LVDS_CHANNEL_RESET);
}

void bst_lvds_control_disable(struct bst_lvds_channel *channel)
{
	unsigned int ctl_reg = 0;
	unsigned int ch_mux_reg = 0;
	struct bst_lvds *lvds = channel->lvds;

	ctl_reg = readl(lvds->regs + LVDS_CONTROL_REG);
	ch_mux_reg = readl(lvds->regs + LVDS_CHANNEL_MUX);

	ctl_reg &= (~BIT(10));	//CLOSE channel 1
	//ctl_reg &=(~BIT(9));//CLOSE eclk
	//ctl_reg &=(~BIT(8));//CLOSE oclk

	ctl_reg &= (~BIT(22));	//CLOSE channel 2
	//ctl_reg &=(~BIT(21));//CLOSE eclk
	//ctl_reg &=(~BIT(20));//CLOSE oclk
	WRITEL(ctl_reg, lvds->regs + LVDS_CONTROL_REG);
	WRITEL(ch_mux_reg, lvds->regs + LVDS_CHANNEL_MUX);

}

void display_lvds_link_cfg(struct bst_lvds *lvds)
{
	struct bst_dpu_connection conn;
	struct device_node *child, *endpoint;
	unsigned int i;
	int ret;

	for_each_child_of_node(lvds->dev->of_node, child) {

		ret = of_property_read_u32(child, "reg", &i);
		if ( ret || (i != LVDS_CHANNEL_1 && i != LVDS_CHANNEL_2))
			continue;

		endpoint = of_graph_get_endpoint_by_regs(child, 1, 0);
		if (!endpoint)
			continue;

		ret = bst_get_remote_dpu_connection(endpoint, &conn);
		of_node_put(endpoint);
		if (ret)
			continue;

		if (conn.host){
			bst_select_dpu_output_to_lvds(&conn, i);
			lvds->channel[i].host_dpu = conn.host;
			bst_dpu_check_and_release(conn.host);
		}

		DRM_DEV_INFO(lvds->dev, "lvds%d connect to dpu:%d,pipe:%d,link:%d\n",
			i, conn.port.dpu_id, conn.port.pipeline_id, conn.port.link_id);
	}
}

static void bst_lvds_encoder_enable(struct drm_encoder *encoder)
{
	struct bst_lvds_channel *channel = encoder_to_lvds_ch(encoder);
	struct drm_display_mode *mode = &encoder->crtc->state->adjusted_mode;
	struct bst_dev *mdev = encoder->dev->dev_private;

	if(mdev->resume) {
		display_lvds_link_cfg(channel->lvds);
	}

	drm_panel_prepare(channel->panel);
	bst_lvds_control_enable(channel, mode);
	bst_lvds_control_config(channel);
	bst_lvds_control_reset(channel);
	drm_panel_enable(channel->panel);
}

static void bst_lvds_encoder_disable(struct drm_encoder *encoder)
{
	struct bst_lvds_channel *channel = encoder_to_lvds_ch(encoder);

	drm_panel_disable(channel->panel);
	bst_lvds_control_disable(channel);
	drm_panel_unprepare(channel->panel);
}

static const struct drm_encoder_helper_funcs bst_lvds_encoder_helper_funcs = {
	.enable = bst_lvds_encoder_enable,
	.disable = bst_lvds_encoder_disable,
	.atomic_check = bst_lvds_encoder_atomic_check,
};

static int bst_lvds_register(struct drm_device *drm_dev,
	struct bst_lvds_channel *channel, struct device_node *child)
{
		int ret = 0;
		struct drm_encoder *encoder = NULL;
		struct drm_connector *connector = NULL;

		if (drm_dev == NULL || channel == NULL || child == NULL) {
			DRM_ERROR("lvds register fail null!\n");
			return -EINVAL;
		}
		encoder = &channel->encoder;
		encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
									child);
		encoder->possible_clones = 0;

		ret = drm_simple_encoder_init(drm_dev, encoder, DRM_MODE_ENCODER_LVDS);
		if (ret < 0) {
			DRM_DEV_ERROR(drm_dev->dev,
					"failed to initialize encoder: %d\n", ret);
			return ret;
		}

		drm_encoder_helper_add(encoder, &bst_lvds_encoder_helper_funcs);
		if (channel->panel) {
			connector = &channel->connector;
			ret = drm_connector_init(drm_dev, connector,
						&bst_lvds_connector_funcs,
						DRM_MODE_CONNECTOR_LVDS);
			if (ret < 0) {
				DRM_DEV_ERROR(drm_dev->dev,
						"failed to initialize connector: %d\n",
						ret);
				goto err_free_encoder;
			}
			drm_connector_helper_add(connector,
						&bst_lvds_connector_helper_funcs);

			ret = drm_connector_attach_encoder(connector, encoder);
			if (ret < 0) {
				DRM_DEV_ERROR(drm_dev->dev,
						"failed to attach encoder: %d\n", ret);
				goto err_free_connector;
			}
		} else {
			ret = drm_bridge_attach(encoder, channel->bridge, NULL, 0);
			if (ret) {
				DRM_DEV_ERROR(drm_dev->dev,
						"failed to attach bridge: %d\n", ret);
				goto err_free_encoder;
			}
		}

		return 0;

err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static int bst_lvds_bind(struct device *dev, struct device *master, void *data)
{
	struct bst_lvds *lvds = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret = 0, i = 0;
	struct device_node *child;
	const char *name;
	static int channel_cnt;

	lvds->drm_dev = drm_dev;
	for_each_child_of_node(dev->of_node, child) {
		struct bst_lvds_channel *channel;
		struct device_node *remote = NULL;

		ret = of_property_read_u32(child, "reg", &i);
		if (ret || (i != LVDS_CHANNEL_1 && i != LVDS_CHANNEL_2)) {
			ret = -EINVAL;
			DRM_DEV_ERROR(dev, "invalid channel no [%d]\n", i);
			goto free_child;
		}

		if (!of_device_is_available(child))
			continue;

		channel = &lvds->channel[i];
		channel->lvds = lvds;
		channel->chno = i;

		ret = of_property_read_u32(child, "output", &channel->output);
		if (ret) {
			channel->output = DISPLAY_OUTPUT_LVDS_INVALID;
			continue;
		}
		if (channel->output == DISPLAY_OUTPUT_LVDS_INVALID) //ignore others
			continue;
		channel_cnt++;

		if(channel_cnt>=2){
			if (lvds->channel[LVDS_CHANNEL_1].output == lvds->channel[LVDS_CHANNEL_2].output ||
				lvds->channel[LVDS_CHANNEL_1].output == DISPLAY_OUTPUT_DUAL_LVDS ||
				lvds->channel[LVDS_CHANNEL_2].output == DISPLAY_OUTPUT_DUAL_LVDS) {
				DRM_DEV_ERROR(dev, "lvds two channel can't select the same output channel ch1 output:%d ch2 output:%d\n",lvds->channel[LVDS_CHANNEL_1].output,lvds->channel[LVDS_CHANNEL_2].output);
				goto free_child;
			}
		}

		ret = drm_of_find_panel_or_bridge(child,
						  0, 0,
						  &channel->panel, &channel->bridge);
		if (ret) {
			DRM_DEV_ERROR(dev, "not find panel or bridge [%d]\n", ret);
			goto free_child;
		}

		if (channel->panel)
			remote = channel->panel->dev->of_node;
		else
			remote = channel->bridge->of_node;

		ret = of_property_read_u32(remote, "width-mm", &channel->width);
		if (ret) {
			DRM_DEV_ERROR(dev, "not find width-mm[%d]\n", ret);
		}

		ret = of_property_read_u32(remote, "height-mm", &channel->height);
		if (ret) {
			DRM_DEV_ERROR(dev, "not find height-mm[%d]\n", ret);
		}

		if (!of_property_read_string(remote, "data-mapping", &name))
			channel->format = bst_lvds_name_to_format(name);

		if (!of_property_read_string(child, "data-mapping", &name))
			channel->format = bst_lvds_name_to_format(name);

		if (channel->format < 0) {
			DRM_DEV_ERROR(dev, "invalid data-mapping format [%s]\n", name);
			ret = channel->format;
			goto free_child;
		}

		ret = bst_lvds_register(drm_dev, channel, child);
		if (ret) {
			DRM_DEV_ERROR(dev, "bst_lvds_register fail [%d]\n", ret);
			goto free_child;
		}
	}

	pm_runtime_enable(dev);

	display_lvds_link_cfg(lvds);
	return 0;
free_child:

	of_node_put(child);
	return ret;
}

static void bst_lvds_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct bst_lvds *lvds = dev_get_drvdata(dev);
	const struct drm_encoder_helper_funcs *encoder_funcs;
	int i = 0;

	for (i = 0; i < 2; i++) {
		struct bst_lvds_channel *channel = &lvds->channel[i];

		encoder_funcs = channel->encoder.helper_private;
		encoder_funcs->disable(&channel->encoder);
		drm_connector_cleanup(&channel->connector);
		drm_encoder_cleanup(&channel->encoder);
		if (channel->host_dpu)
			put_device(channel->host_dpu);
	}
	pm_runtime_disable(dev);

}

static const struct component_ops bst_lvds_component_ops = {
	.bind = bst_lvds_bind,
	.unbind = bst_lvds_unbind,
};

static int bst_lvds_get_dtb_param(struct bst_lvds *lvds,
				  struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lvds->regs = devm_ioremap_resource(lvds->dev, res);
	if (IS_ERR(lvds->regs))
		return PTR_ERR(lvds->regs);

	return 0;
}

static int bst_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_lvds *lvds;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	lvds = devm_kzalloc(&pdev->dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;

	lvds->dev = dev;
	ret = bst_lvds_get_dtb_param(lvds, pdev);
	if (ret)
		return ret;
	lvds_formats_init(lvds);

	dev_set_drvdata(dev, lvds);

	ret = component_add(&pdev->dev, &bst_lvds_component_ops);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "failed to add component\n");
	}

	return ret;
}

static int bst_lvds_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &bst_lvds_component_ops);
	return 0;
}

static const struct of_device_id bst_lvds_dt_ids[] = {
	{.compatible = "bst,bst-lvds-conn", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bst_lvds_dt_ids);

struct platform_driver bst_lvds_driver = {
       .probe = bst_lvds_probe,
       .remove = bst_lvds_remove,
       .driver = {
                  .name = "bst-lvds",
                  .of_match_table = of_match_ptr(bst_lvds_dt_ids),
                   },
};
module_platform_driver(bst_lvds_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST LVDS controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bst-lvds");
