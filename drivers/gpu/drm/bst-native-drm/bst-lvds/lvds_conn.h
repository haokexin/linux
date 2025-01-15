// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_LVDS_CONN_H_
#define _BST_LVDS_CONN_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <drm/drm_device.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_fixed.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <../bst_drm_dev.h>

#define LVDS_VESA_30				0
#define LVDS_JEIDA_30				1
#define LVDS_FORMAT3_30				2
#define LVDS_VESA_24				3
#define LVDS_JEIDA_24				4
#define LVDS_FORMAT3_24				5
#define LVDS_VESA_18				6
#define LVDS_JEIDA_18				7
#define LVDS_LINEAR_12				8
#define LVDS_NOLINEAR_12			9

#define DISPLAY_OUTPUT_LVDS_INVALID	0
#define DISPLAY_OUTPUT_O_LVDS		1
#define DISPLAY_OUTPUT_E_LVDS		2
#define DISPLAY_OUTPUT_DUAL_LVDS	3

#define LVDS_LANE_BIT 7
#define LVDS_RGB_MAX_LANES 5

#define LVDS_CONTROL_REG			0x00000000
#define ODD_LVDS_DUTY_IRQ			0x00000004
#define EVEN_LVDS_DUTY_IRQ			0x00000008
#define LVDS_PIXEL_NUMBER			0x0000000C
#define LVDS_PIXEL_NUMBER_CH2		0x00000010
#define LVDS_PARITY_CTL				0x00000014
#define LVDS_PARITY_STS				0x00000018
#define LVDS_CHANNEL_RESET			0x0000001C
#define apb_local_resetn			(unsigned int)BIT(2)
#define ch1_local_reset_n			(unsigned int)BIT(1)
#define ch2_local_reset_n			(unsigned int)BIT(0)
#define LVDS_CHANNEL_MUX			0x00000020
#define LVDS_VERSION_REG			0x00000024
#define LVDS_REG_WR_PROTECT			0x00000034
/* VESA/JEIDA RGB Configuration */
#define LVDS_CH1_A0_SEL_CFG 0x0048
#define LVDS_CH1_A1_SEL_CFG 0x004c
#define LVDS_CH1_A2_SEL_CFG 0x0050
#define LVDS_CH1_A3_SEL_CFG 0x0054
#define LVDS_CH1_A4_SEL_CFG 0x0058
#define LVDS_CH1_A5_SEL_CFG 0x005C
#define LVDS_CH1_A6_SEL_CFG 0x0060
#define LVDS_CH2_A0_SEL_CFG 0x0064
#define LVDS_CH2_A1_SEL_CFG 0x0068
#define LVDS_CH2_A2_SEL_CFG 0x006C
#define LVDS_CH2_A3_SEL_CFG 0x0070
#define LVDS_CH2_A4_SEL_CFG 0x0074
#define LVDS_CH2_A5_SEL_CFG 0x0078
#define LVDS_CH2_A6_SEL_CFG 0x007C

/**
 *  lvds_formats - LVDS RGB arrangement according to different protocols
 *  @r[10]:      R component(6bit)
 *  @g[10]:      G component(6bit)
 *  @b[10]:      B component(6bit)
 *  @data_en:    data_en component(6bit)
 *  @vsync:      vsync component(6bit)
 *  @hsync:      hsync component(6bit)
 *  @res0:       res component(6bit)
 *  @res1:       res component(6bit)
 *  @defaults:    defaults component(6bit)
 */
struct lvds_formats {
	unsigned char r[10];
	unsigned char g[10];
	unsigned char b[10];
	unsigned char data_en;
	unsigned char vsync;
	unsigned char hsync;
	unsigned char res0;
	unsigned char res1;
	unsigned char defaults;
};

// Channel 1
#define CH1_A0_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][6] | lvds_arrary[1][6] << 6 |                          \
	 lvds_arrary[2][6] << 12 | lvds_arrary[3][6] << 18 |                   \
	 lvds_arrary[4][6] << 24)

#define CH1_A1_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][5] | lvds_arrary[1][5] << 6 |                          \
	 lvds_arrary[2][5] << 12 | lvds_arrary[3][5] << 18 |                   \
	 lvds_arrary[4][5] << 24)
#define CH1_A2_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][4] | lvds_arrary[1][4] << 6 |                          \
	 lvds_arrary[2][4] << 12 | lvds_arrary[3][4] << 18 |                   \
	 lvds_arrary[4][4] << 24)
#define CH1_A3_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][3] | lvds_arrary[1][3] << 6 |                          \
	 lvds_arrary[2][3] << 12 | lvds_arrary[3][3] << 18 |                   \
	 lvds_arrary[4][3] << 24)
#define CH1_A4_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][2] | lvds_arrary[1][2] << 6 |                          \
	 lvds_arrary[2][2] << 12 | lvds_arrary[3][2] << 18 |                   \
	 lvds_arrary[4][2] << 24)
#define CH1_A5_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][1] | lvds_arrary[1][1] << 6 |                          \
	 lvds_arrary[2][1] << 12 | lvds_arrary[3][1] << 18 |                   \
	 lvds_arrary[4][1] << 24)
#define CH1_A6_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][0] | lvds_arrary[1][0] << 6 |                          \
	 lvds_arrary[2][0] << 12 | lvds_arrary[3][0] << 18 |                   \
	 lvds_arrary[4][0] << 24)
// Channel 2
#define CH2_A0_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][6] | lvds_arrary[1][6] << 6 |                          \
	 lvds_arrary[2][6] << 12 | lvds_arrary[3][6] << 18 |                   \
	 lvds_arrary[4][6] << 24)
#define CH2_A1_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][5] | lvds_arrary[1][5] << 6 |                          \
	 lvds_arrary[2][5] << 12 | lvds_arrary[3][5] << 18 |                   \
	 lvds_arrary[4][5] << 24)
#define CH2_A2_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][4] | lvds_arrary[1][4] << 6 |                          \
	 lvds_arrary[2][4] << 12 | lvds_arrary[3][4] << 18 |                   \
	 lvds_arrary[4][4] << 24)
#define CH2_A3_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][3] | lvds_arrary[1][3] << 6 |                          \
	 lvds_arrary[2][3] << 12 | lvds_arrary[3][3] << 18 |                   \
	 lvds_arrary[4][3] << 24)
#define CH2_A4_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][2] | lvds_arrary[1][2] << 6 |                          \
	 lvds_arrary[2][2] << 12 | lvds_arrary[3][2] << 18 |                   \
	 lvds_arrary[4][2] << 24)
#define CH2_A5_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][1] | lvds_arrary[1][1] << 6 |                          \
	 lvds_arrary[2][1] << 12 | lvds_arrary[3][1] << 18 |                   \
	 lvds_arrary[4][1] << 24)
#define CH2_A6_SEL_CFG(lvds_arrary)                                            \
	(lvds_arrary[0][0] | lvds_arrary[1][0] << 6 |                          \
	 lvds_arrary[2][0] << 12 | lvds_arrary[3][0] << 18 |                   \
	 lvds_arrary[4][0] << 24)

#define WRITEL(v, r) do { \
	printk("%s: %08x => %px\n", __func__, (unsigned int)(v), (r)); \
	writel((v), (r)); \
} while (0)

#endif
