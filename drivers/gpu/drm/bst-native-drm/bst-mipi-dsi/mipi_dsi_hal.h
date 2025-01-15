// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef __MIPI_DSI_HAL_H_
#define __MIPI_DSI_HAL_H_
#include "mipi-dsi-bst.h"
#define DSI_VERSION	            0x00000000
#define DSI_PWR_UP	            0x00000004
#define RESET				    0
#define POWERUP				    BIT(0)
#define DSI_CLKMGR_CFG	        0x00000008
#define TO_CLK_DIVISION(div)		(((div) & 0xff) << 8)
#define TX_ESC_CLK_DIVISION(div)	((div) & 0xff)
#define DSI_DPI_VCID	        0x0000000C
#define DPI_VCID(vcid)			((vcid) & 0x3)
#define DSI_DPI_COLOR_CODING    0x00000010
#define LOOSELY18_EN			BIT(8)
#define DPI_COLOR_CODING_16BIT_1	0x0
#define DPI_COLOR_CODING_16BIT_2	0x1
#define DPI_COLOR_CODING_16BIT_3	0x2
#define DPI_COLOR_CODING_18BIT_1	0x3
#define DPI_COLOR_CODING_18BIT_2	0x4
#define DPI_COLOR_CODING_24BIT		0x5
#define DSI_DPI_CFG_POL	        0x00000014
#define COLORM_ACTIVE_LOW		BIT(4)
#define SHUTD_ACTIVE_LOW		BIT(3)
#define HSYNC_ACTIVE_LOW		BIT(2)
#define VSYNC_ACTIVE_LOW		BIT(1)
#define DATAEN_ACTIVE_LOW		BIT(0)
#define DSI_DPI_LP_CMD_TIM      0x00000018
#define DSI_DPI_LP_CMD_TIM      0x00000018
#define DSI_DBI_VCID	        0x0000001C
#define DSI_DBI_CFG	            0x00000020
#define DSI_DBI_PARTITIONING_EN 0x00000024
#define DSI_DBI_CMDSIZE	        0x00000028
#define DSI_PCKHDL_CFG	        0x0000002C
#define CRC_RX_EN		        BIT(4)
#define ECC_RX_EN		        BIT(3)
#define BTA_EN			        BIT(2)
#define EOTP_RX_EN		        BIT(1)
#define EOTP_TX_EN		        BIT(0)

#define DSI_GEN_VCID	        0x00000030
#define DSI_MODE_CFG	        0x00000034
#define ENABLE_VIDEO_MODE		0
#define ENABLE_CMD_MODE			BIT(0)
#define DSI_VID_MODE_CFG	    0x00000038
#define ENABLE_LOW_POWER		    (0x3f << 8)
#define ENABLE_LOW_POWER_MASK		(0x3f << 8)
#define VID_MODE_TYPE_NON_BURST_SYNC_PULSES	0x0
#define VID_MODE_TYPE_NON_BURST_SYNC_EVENTS	0x1
#define VID_MODE_TYPE_BURST			0x2
#define VID_MODE_TYPE_MASK			0x3
#define ENABLE_LOW_POWER_CMD		BIT(15)
#define VID_MODE_VPG_ENABLE		    BIT(16)
#define VID_MODE_VPG_MODE		    BIT(20)
#define VID_MODE_VPG_HORIZONTAL		BIT(24)
#define DSI_VID_PKT_SIZE	    0x0000003C
#define VID_PKT_SIZE(p)			((p) & 0x3fff)
#define DSI_VID_NUM_CHUNKS      0x00000040
#define DSI_VID_NULL_SIZE       0x00000044
#define DSI_VID_HSA_TIME	    0x00000048
#define DSI_VID_HBP_TIME	    0x0000004C
#define DSI_VID_HLINE_TIME      0x00000050
#define DSI_VID_VSA_LINES       0x00000054
#define DSI_VID_VBP_LINES       0x00000058
#define DSI_VID_VFP_LINES       0x0000005C
#define DSI_VID_VACTIVE_LINES   0x00000060
#define DSI_EDPI_CMD_SIZE       0x00000064
#define DSI_CMD_MODE_CFG	    0x00000068
#define DSI_GEN_HDR	            0x0000006C
#define DSI_GEN_PLD_DATA	    0x00000070

#define DSI_CMD_PKT_STATUS      0x74
#define GEN_PLD_BUF_FULL        BIT(19)
#define GEN_CMD_BUF_FULL        BIT(17)
#define GEN_RD_CMD_BUSY		    BIT(6)
#define GEN_PLD_R_FULL		    BIT(5)
#define GEN_PLD_R_EMPTY		    BIT(4)
#define GEN_PLD_W_FULL		    BIT(3)
#define GEN_PLD_W_EMPTY		    BIT(2)
#define GEN_CMD_FULL		    BIT(1)
#define GEN_CMD_EMPTY		    BIT(0)
#define DSI_TO_CNT_CFG	        0x78
#define HSTX_TO_CNT(p)			(((p) & 0xffff) << 16)
#define LPRX_TO_CNT(p)			((p) & 0xffff)
#define DSI_HS_RD_TO_CNT	    0x7c
#define DSI_LP_RD_TO_CNT	    0x80
#define DSI_HS_WR_TO_CNT	    0x84
#define DSI_LP_WR_TO_CNT	    0x88
#define DSI_BTA_TO_CNT	        0x8c
#define DSI_SDF_3D	            0x90
#define DSI_LPCLK_CTRL			0x94
#define AUTO_CLKLANE_CTRL		BIT(1)
#define PHY_TXREQUESTCLKHS		BIT(0)
#define DSI_PHY_TMR_LPCLK_CFG   0x98
#define DSI_PHY_TMR_CFG	        0x9c
#define DSI_PHY_RSTZ	        0xa0
#define PHY_DISFORCEPLL			0
#define PHY_ENFORCEPLL			BIT(3)
#define PHY_DISABLECLK			0
#define PHY_ENABLECLK			BIT(2)
#define PHY_RSTZ			    0
#define PHY_UNRSTZ			    BIT(1)
#define PHY_SHUTDOWNZ			0
#define PHY_UNSHUTDOWNZ			BIT(0)

#define DSI_PHY_IF_CFG	        0xa4
#define DSI_PHY_ULPS_CTRL       0xa8
#define DSI_PHY_TX_TRIGGERS     0x0a
#define DSI_PHY_STATUS	        0xb0
#define DSI_PHY_TST_CTRL0       0xb4
#define DSI_PHY_TST_CTRL1       0xb8
#define DSI_INT_ST0	            0xbc

#define DSI_INT_ST1		        0xc0
#define DSI_INT_MSK0		    0xc4
#define DSI_INT_MSK1		    0xc8
#define DSI_PHY_CAL		        0xcc
#define DSI_INT_FORCE0		    0xd8
#define DSI_INT_FORCE1		    0xdc
#define DSI_AUTO_ULPS_MODE	    0xe0
#define DSI_AUTO_ULPS_ENTRY_DELAY   0xe4
#define DSI_AUTO_ULPS_WAKEUP_TIME   0xe8
#define DSI_DSC_PARAMETER	        0xf0
#define DSI_PHY_TMR_RD_CFG	        0xf4
#define DSI_AUTO_ULPS_MIN_TIME	    0xf8
#define DSI_PHY_MODE		        0xfc
#define DSI_VID_SHADOW_CTRL	        0x100
#define DSI_DPI_VCID_ACT		    0x10c
#define DSI_DPI_COLOR_CODING_ACT	0x110
#define DSI_DPI_LP_CMD_TIM_ACT	    0x118
#define DSI_EDPI_TE_HW_CFG	        0x11c
#define DSI_VID_MODE_CFG_ACT	    0x138
#define DSI_VID_PKT_SIZE_ACT	    0x13c
#define DSI_VID_NUM_CHUNKS_ACT	    0x140

#define DSI_VID_NULL_SIZE_ACT		    0x144
#define DSI_VID_HSA_TIME_ACT		    0x148
#define DSI_VID_HBP_TIME_ACT		    0x14c
#define DSI_VID_HLINE_TIME_ACT		    0x150
#define DSI_VID_VSA_LINES_ACT		    0x154
#define DSI_VID_VFP_LINES_ACT		    0x15c
#define DSI_VID_VACTIVE_LINES_ACT	    0x160
#define DSI_VID_PKT_STATUS		        0x168
#define DSI_SDF_3D_ACT			        0x190
#define DSI_DSC_ENC_COREID		        0x200
#define DSI_DSC_ENC_VERSION		        0x204
#define DSI_DSC_ENC_FLATNESS_DET_THRES	    0x208
#define DSI_DSC_ENC_DELAY		            0x20c
#define DSI_DSC_ENC_COMPRESSED_LINE_SIZE    0x210
#define DSI_DSC_ENC_LINES_IN_EXCESS	        0x214
#define DSI_DSC_ENC_RBUF_ADDR_LAST_LINE_ADJ 0x218
#define DSI_DSC_MODE			            0x21c
#define DSI_DSC_ENC_INT_ST		            0x220
#define DSI_DSC_ENC_INT_MSK		            0x224

#define DSI_DSC_ENC_INT_FORCE	    0x228
#define DSI_DSC_FIFO_STATUS_SELECT  0x22c
#define DSI_DSC_FIFO_STATUS	        0x230
#define DSI_DSC_FIFO_STATUS2	    0x234
#define DSI_DSC_FIFO_WORD_COUNT	    0x238
#define DSI_DSC_FIFO_WORD_COUNT2	0x23c
#define DSI_DSC_ENC_PPS_0_3	        0x260
#define DSI_DSC_ENC_PPS_4_7	        0x264
#define DSI_DSC_ENC_PPS_8_11	  0x268
#define DSI_DSC_ENC_PPS_12_15	  0x26c
#define DSI_DSC_ENC_PPS_16_19	  0x270
#define DSI_DSC_ENC_PPS_20_23	  0x274
#define DSI_DSC_ENC_PPS_24_27	  0x278
#define DSI_DSC_ENC_PPS_28_31	  0x27c
#define DSI_DSC_ENC_PPS_32_35	  0x280
#define DSI_DSC_ENC_PPS_36_39	  0x284
#define DSI_DSC_ENC_PPS_40_43	  0x288
#define DSI_DSC_ENC_PPS_44_47	  0x28c
#define DSI_DSC_ENC_PPS_48_51	  0x290
#define DSI_DSC_ENC_PPS_52_55	  0x294

#define DSI_DSC_ENC_PPS_56_59 0x298
#define DSI_DSC_ENC_PPS_60_63 0x29c
#define DSI_DSC_ENC_PPS_64_67 0x2a0
#define DSI_DSC_ENC_PPS_68_71 0x2a4
#define DSI_DSC_ENC_PPS_72_75 0x2a8
#define DSI_DSC_ENC_PPS_76_79 0x2ac
#define DSI_DSC_ENC_PPS_80_83 0x2b0
#define DSI_DSC_ENC_PPS_84_87 0x2b4
#define DSI_INT_ST0_AP	      0x300
#define DSI_INT_MSK0_AP	      0x304
#define DSI_INT_FORCE0_AP     0x308
#define DSI_INT_ST1_AP	      0x310
#define DSI_INT_MSK1_AP	      0x314
#define DSI_INT_FORCE1_AP     0x318
#define DSI_INT_ST2_AP	      0x320
#define DSI_INT_MSK2_AP	      0x324
#define DSI_INT_FORCE2_AP     0x328
#define DSI_TO_HSTXRDY_CFG_AP 0x340
#define DSI_TO_LPTXRDY_CFG_AP 0x344

#define DSI_TO_LPTXTRIG_CFG_AP  0x348
#define DSI_TO_LPTXULPS_CFG_AP  0x34c
#define DSI_TO_HSTX_CFG_AP      0x350
#define DSI_TO_LPRX_CFG_AP      0x354
#define DSI_TO_BTA_CFG_AP       0x358
#define DSI_TO_CLK_DIV_AP       0x35c
#define DSI_ERR_INJ_CTRL_AP     0x380
#define DSI_ERR_INJ_CHK_MSK_AP  0x384
#define DSI_ERR_INJ_DATA_MSK_AP 0x388
#define DSI_ERR_INJ_ST_AP       0x38c

enum{
    DPHY_DATA_RATE_800M,
    DPHY_DATA_RATE_850M,
    DPHY_DATA_RATE_900M,
	DPHY_DATA_RATE_950M,
    DPHY_DATA_RATE_1000M,
	DPHY_DATA_RATE_1200M,
	DPHY_DATA_RATE_1250M,
    DPHY_DATA_RATE_1500M,
    DPHY_DATA_RATE_2000M,
    DPHY_DATA_RATE_2500M,
};
struct dw_mipi_dsi_bst;
void bst_mipi_csr_set(struct dw_mipi_dsi_bst *dsi, unsigned int reg, unsigned int mask);
int cfg_dphy_signals(struct dw_mipi_dsi_bst *dsi, int dphy_sel, int lane_mbps);
int dphy_rate_swtch(struct dw_mipi_dsi_bst *dsi, int lane_mbps);
#endif