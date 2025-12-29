// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __MIPI_DSI_BST_H_
#define __MIPI_DSI_BST_H_
#include "mipi_dsi_hal.h"
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <video/mipi_display.h>
#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_probe_helper.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <drm/drm_bridge_connector.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <../bst_drm_dev.h>

#define MIPI_BASE_ADDR	  (void *)(0x24600000)
#define MIPI_CSITX 		  0x00000000
#define MIPI_DSI1  		  0x00004000
#define MIPI_DSI0  		  0x00008000
#define MIPI_CSR	  	  0x0000C000

#define MIPI_CSITX_ADDR (MIPI_BASE_ADDR + MIPI_CSITX)
#define MIPI_DSI1_ADDR	(MIPI_BASE_ADDR + MIPI_DSI1)
#define MIPI_DSI0_ADDR	(MIPI_BASE_ADDR + MIPI_DSI0)
#define MIPI_CSR_ADDR	(MIPI_BASE_ADDR + MIPI_CSR)

#define SYS_CTRL_BASE		 (void *)(0x30000000)
#define SYSNOC_PTY_INTR_2	 0x40
#define SYSNOC_PTY_INTR_INJECT_1 0x34
#define SYSNOC_PTY_EN_1		 0x24
#define SYSNOC_PTY_CLEAR_1	 0x2C
#define SOC_TO_DB_INTR_SEL0  0x164
#define SOC_TO_DB_INTR_SEL1  0x168
#define SOC_TO_DB_INTR_SEL2  0x16C
#define SOC_TO_DB_INTR_SEL3  0x170
#define SOC_TO_DB_INTR_SEL4  0x174
#define SOC_TO_DB_INTR_SEL5  0x178
#define SOC_TO_DB_INTR_SEL6  0x17C
#define SOC_TO_SAFE_INTR_SEL0 0x1d4
#define SOC_TO_RT_INTR_SEL0  0x274
#define SYS_REG_WR_PROTECT       0x300
#define SYSNOC_PTY_INTR_MASK_1   0x420

#define MIPI_DSI_DEBUG 0x8
#define MIPI_DSI_INFO  0x4
#define MIPI_DSI_WARN  0x2
#define MIPI_DSI_ERROR 0x1

enum{
	MIPI_DSI0_INST,
	MIPI_DSI1_INST,
	MIPI_DSI_MAX,
};

enum{
	DUAL_PIPE_DISPLAY0,
	DUAL_PIPE_DISPLAY1,
	SINGL_PIPE_DISPLAY,
	DISPLAY_INST_MAX,
};

enum{
	DISPLAY_PIPE0,
	DISPLAY_PIPE1,
};

enum{
	DISPLAY_LINKE0,
	DISPLAY_LINKE1,
};

#define DSI_CSITX_CFG_CLK	  0x00
#define DSI_CSITX_CTRL_DSI01_REG0 0x1c
#define DSI_DPI_SEL(id)           (BIT(id) << (12))
#define DSI_TE_REQ(id)            ((id) ? BIT(5): BIT(11))
#define LOCAL_CTRL_CSITX_REG0	  0x20
#define CSITX_DPHY		  BIT(7)
#define DSI_CSITX_IPI_RSTN	  ~BIT(0)

#define DSI_CSITX_ADDR_START0	  0x30
#define DSI_CSITX_ADDR_START1	  0x34
#define DSI_CSITX_ADDR_START2	  0x38
#define DSI_CSITX_ADDR_END0	  0x40
#define DSI_CSITX_ADDR_END1	  0x44
#define DSI_CSITX_ADDR_END2	  0x48

#define DSI_CSITX_DIAG_FLT	  0x50
#define DSI_CSITX_INT_STAT	  0x54
#define DSI_CSITX_APB_PARITY_TYPE 0x58
#define DSI_CSITX_DPI_PARITY	  0x5C

#define IPTEST_DPHY0_REG0 0x70
#define IPTEST_DPHY0_REG1 0x74
#define IPTEST_DPHY0_REG2 0x78
#define IPTEST_DPHY0_REG3 0x7C
#define IPTEST_DPHY0_REG4 0x80
#define IPTEST_DPHY0_REG5 0x84

#define IPTEST_DPHY1_REG0 0x90
#define IPTEST_DPHY1_REG1 0x94
#define IPTEST_DPHY1_REG2 0x98
#define IPTEST_DPHY1_REG3 0x9C
#define IPTEST_DPHY1_REG4 0xA0
#define IPTEST_DPHY1_REG5 0xA4

#define DISPLAY2MIPI_MAP	    0xa8
#define DSI0_REMAP_RGB24_EN	    BIT(15)
#define DSI0_REMAP_RGB30_EN	    BIT(14)
#define DSI1_REMAP_RGB24_EN	    BIT(13)
#define DSI1_REMAP_RGB30_EN	    BIT(12)
#define DSI0_REMAP_YUV422_8_EN	    BIT(11)
#define DSI0_REMAP_YUV422_10_EN	    BIT(10)
#define DSI0_REMAP_YUV420_TOGGLE_EN BIT(9)
#define DSI0_REMAP_YUV420_HALF_EN   BIT(8)
#define DSI1_REMAP_YUV422_8_EN	    BIT(7)
#define DSI1_REMAP_YUV422_10_EN	    BIT(6)
#define DSI1_REMAP_YUV420_TOGGLE_EN BIT(5)
#define DSI1_REMAP_YUV420_HALF_EN   BIT(4)

#define DSI_CSITX_DISLAY2DSI_PARITY 0xac
#define DSI_CSITX_CLK_SEL	    0xb0
#define IPI_CLK_SEL		    (BIT(14) | BIT(15))
#define IDI_CLK_SEL		    (BIT(13) | BIT(12))
#define FIFO_CLK_SEL		    (BIT(10) | BIT(11))
#define CSITX_ALL_RST_N				BIT(9)
#define LOCAL_ISP_RST_N				BIT(8)
#define LOCAL_DSI0_DPIPCLK_MUX_RSTN	BIT(6)
#define LOCAL_DSI1_DPIPCLK_MUX_RSTN	BIT(5)
#define LOCAL_DSI_CSITX_DMAC_RST_N	BIT(4)
#define LOCAL_DSI_CSITX_HRESETN		BIT(3)
#define LOCAL_DSI0_PRESETN			BIT(2)
#define LOCAL_DSI1_PRESETN			BIT(1)
#define LOCAL_CSITX_PRESETN			BIT(0)

#define DSI_CSITX_APB_PARITY_CTL    0xb4
#define DSI_CSITX_DSI_FUN_CFG	    0xB8
#define REG_DSI_CSITX_INT_MASK	    0xBC
#define IDI0_DATA_PARITY	    0xC0
#define IDI1_DATA_PARITY	    0xC4
#define IDI2_DATA_PARITY	    0xC8
#define DSICSITX_RESERVD0_ECO	    0xCC
#define DSICSITX_RESERVD1_ECO	    0xD0
#define DSICSITX_RESERVD2_ECO	    0xD4
#define DSICSITX_RESERVD3_ECO	    0xD8
#define DSI0_DATA_VID_EN_ECO		BIT(5)
#define DSI1_DATA_VID_EN_ECO		BIT(11)
#define DSICSITX_RESERVD4_ECO	    0xDC
#define DSICSITX_RESERVD5_ECO	    0xE0
#define DSI0_MEM_PARITY		    0xE4
#define DSI1_MEM_PARITY		    0xE8
#define CSITX_MEM_PARITY12	    0xEC
#define CSITX_MEM_PARITY34	    0xF0

#define pixel_data_clk 400
#define phy_hstx_clk   300		//lane model clk
#define proportion (phy_hstx_clk/pixel_data_clk)

#define DSI_CFG_POL(disp)	  (((disp)->hpol == 0 ? BIT(2) : 0) | ((disp)->vpol == 0 ? BIT(1) : 0))
#define DSI_CFG_POL_INV(disp) (((disp)->hpol == 1 ? BIT(2) : 0) | ((disp)->vpol == 1 ? BIT(1) : 0))

#define HSTT(_maxfreq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)	\
{					\
	.maxfreq = _maxfreq,		\
	.timing = {			\
		.clk_lp2hs = _c_lp2hs,	\
		.clk_hs2lp = _c_hs2lp,	\
		.data_lp2hs = _d_lp2hs,	\
		.data_hs2lp = _d_hs2lp,	\
	}				\
}
enum mipi_test_mode {
	MIPI_DSI_WRITE_TEST,
	MIPI_DSI_READ_TEST,
	MIPI_VIDEO_MODE,
	MIPI_CMD_MODE,
	MIPI_VPG_TEST,
	MIPI_DUAL_TEST,
	MIPI_DBG_ON,
	MIPI_PARITY_TEST,
	MIPI_INST_MAX,
};

enum {
	MIPI_DSI_16BIT_CONFIGURATION1 = 0x0,
	MIPI_DSI_16BIT_CONFIGURATION2,
	MIPI_DSI_16BIT_CONFIGURATION3,
	MIPI_DSI_18BIT_CONFIGURATION1,
	MIPI_DSI_18BIT_CONFIGURATION2,
	MIPI_DSI_24BIT,
	MIPI_DSI_20BIT_YUV422,
	MIPI_DSI_24BIT_YUV422,
	MIPI_DSI_16BIT_YUV422,
	MIPI_DSI_30BIT,
	MIPI_DSI_36BIT = 0XA,
	MIPI_DSI_12BIT_YUV420,
	MIPI_DSI_DSC_24BIT,
};

struct bst_crtc_state {
	struct drm_crtc_state base;
	int output_type;
	int output_mode;
	int output_bpc;
	int output_flags;
	bool enable_afbc;
};
#define to_bst_crtc_state(s) \
		container_of(s, struct bst_crtc_state, base)
#define to_dsi(nm)	container_of(nm, struct dw_mipi_dsi_bst, nm)


struct bst_dsi_chip_data {
	u32 reg;

	u32 lcdsel_grf_reg;
	u32 lcdsel_big;
	u32 lcdsel_lit;

	u32 enable_grf_reg;
	u32 enable;

	u32 lanecfg1_grf_reg;
	u32 lanecfg1;
	u32 lanecfg2_grf_reg;
	u32 lanecfg2;

	unsigned int flags;
	unsigned int dsi_id;
	unsigned int max_data_lanes;
};

struct dw_mipi_dsi_bst {
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *dsi_base;
	void __iomem *csr_base;
	struct regmap *csr_regmap;

	/* dual-channel */
	bool is_slave;
	struct dw_mipi_dsi_bst *dsi0;
	struct dw_mipi_dsi_bst *dsi1;

	/* remote dpu device */
	struct device *host_dpu;

	unsigned int lane_mbps; /* per lane */
	u16 input_div;
	u16 feedback_div;
	u32 format;
	u32 mode_flags;
	u32 lanes;
	u32 channel;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
	struct debugfs_entries *debugfs_vpg;
	struct {
		bool vpg;
		bool vpg_horizontal;
		bool vpg_ber_pattern;
	} vpg_defs;
#endif /* CONFIG_DEBUG_FS */
	struct videomode *vm;
	struct device_node *panel_node;
	struct dw_mipi_dsi *dw_dsi;
	const struct bst_dsi_chip_data *cdata;
	struct dw_mipi_dsi_plat_data pdata;
};

/* MIPI DCS pixel formats */
#define MIPI_DCS_PIXEL_FMT_24BIT 7
#define MIPI_DCS_PIXEL_FMT_18BIT 6
#define MIPI_DCS_PIXEL_FMT_16BIT 5
#define MIPI_DCS_PIXEL_FMT_12BIT 3
#define MIPI_DCS_PIXEL_FMT_8BIT	 2
#define MIPI_DCS_PIXEL_FMT_3BIT	 1

void bst_mipi_dsi_video_mode_config(struct dw_mipi_dsi_bst *dsi);
void bst_dsi_set(struct dw_mipi_dsi_bst *dsi, u32 reg, u32 mask);
u32 bst_dsi_read(struct dw_mipi_dsi_bst *dsi, u32 reg);
int get_dsi_version(struct dw_mipi_dsi_bst *dsi);
void bst_dsi_write(struct dw_mipi_dsi_bst *dsi, u32 reg, u32 val);
void bst_dsi_csr_write(struct dw_mipi_dsi_bst *dsi,u32 reg, u32 val);
void bst_dsi_debugfs_init(struct dw_mipi_dsi_bst *dev);
void cfg_dsi_video_mode(struct dw_mipi_dsi_bst *dsi);
void mipi_dsi_channel(struct dw_mipi_dsi_bst *dsi,u8 dsi_id);
void bst_mipi_dsi_reset(struct dw_mipi_dsi_bst *dsi);

#endif