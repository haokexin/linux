/* SPDX-License-Identifier: GPL-2.0 */
/* reset driver for BST C1200
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 */

#ifndef _DT_BINDINGS_BST_C1200_RESETS_H_
#define _DT_BINDINGS_BST_C1200_RESETS_H_

/* BLOCK_SW_REG0 */
#define RST_BLOCK_SW_REG0_START     (0)
#define RST_CMN_FMU_SW              (0)
#define RST_XGMAC_SW                (1)
#define RST_CLK_MONITOR             (2)
#define RST_CPU0_MP4_SW             (3)
#define RST_CPU1_MP4_SW             (4)
#define RST_CPU_PERIP_SW            (5)
#define RST_EDP_SW                  (6)
#define RST_HIFI_DSP_SW             (7)
#define RST_MEDIA_DMA_SW            (8)
#define RST_MIPI_DSI_CSITX_SW       (9)
#define RST_BLOCK_SW_REG0_END       (9)

/* BLOCK_SW_REG1 */
#define RST_BLOCK_SW_REG1_START     (10)
#define RST_MATRIX_SW               (10)
#define RST_SOC_DMA_SW              (11)
#define RST_PCIE_SW                 (12)
#define RST_UFS_SW                  (13)
#define RST_PLL_SW                  (14)
#define RST_CPU_MP2_SW              (15)
#define RST_GPU_G78AE_SP_SW         (16)
#define RST_DB_DMA_SW               (17)
#define RST_ISP_SW                  (18)
#define RST_MIPI0_CSIRX_SW          (19)
#define RST_MIPI1_CSIRX_SW          (20)
#define RST_MIPI2_CSIRX_SW          (21)
#define RST_CV_SW                   (22)              
#define RST_NET_SW                  (23)
#define RST_CS_DMA_SW               (24)
#define RST_GPU_G78AE_SW            (25)
#define RST_DISPLAY_0_SW            (26)
#define RST_DISPLAY_1_SW            (27)
#define RST_DISPLAY_2_SW            (28)
#define RST_CODEC_0_SW              (29)
#define RST_CODEC_1_SW              (30)
#define RST_LVDS_0_SW               (31)
#define RST_JIAYU_SW                (32)
#define RST_USB3_0_SW               (33)
#define RST_USB3_1_SW               (34)
#define RST_SDEMMC_0_SW             (35)
#define RST_SDEMMC_1_SW             (36)
#define RST_SOC_LSP_0_SW            (37)            
#define RST_SOC_LSP_1_SW            (38)
#define RST_LPDDR5_0_SW             (39)
#define RST_LPDDR5_1_SW             (40)
#define RST_BLOCK_SW_REG1_END       (40)

/* IST_CTRL_REG0 */
#define RST_IST_CTRL_REG0_START     (41)
#define RST_TOP_LB_CORESIGHT_CRM_SW (41)
#define RST_IST_CTRL_REG0_END       (41)

/* FRACDIV_CTRL_SW_RST_CTRL_0 */
#define RST_IFRACDIV_CTRL_SW_CTRL_0_START  (42)
#define RST_SOC_LSP1_FLEXRAY_HCLK_SW       (42)
#define RST_SOC_LSP0_FLEXRAY_HCLK_SW       (43)
#define RST_TOP_USB_U20_PHY_REF_CLK_SW     (44)
#define RST_GTC_DIV_WCLK_SW                (45)
#define RST_BD_EXTERNAL_CLK7_SW            (46)    
#define RST_BD_EXTERNAL_CLK6_SW            (47)
#define RST_BD_EXTERNAL_CLK5_SW            (48)
#define RST_BD_EXTERNAL_CLK4_SW            (49)
#define RST_BD_EXTERNAL_CLK3_SW            (50)
#define RST_BD_EXTERNAL_CLK2_SW            (51)
#define RST_BD_EXTERNAL_CLK1_SW            (52)
#define RST_BD_EXTERNAL_CLK0_SW            (53)
#define RST_REF_ALT_CLK_26M_SW             (54)
#define RST_CLK_DISPLAY2_CH0_594_SW        (55)
#define RST_CLK_DISPLAY1_CH1_594_SW        (56)
#define RST_CLK_DISPLAY1_CH0_594_SW        (57)
#define RST_CLK_DISPLAY0_CH1_594_SW        (58)
#define RST_CLK_DISPLAY0_CH0_594_SW        (59)
#define RST_FRACDIV_CTRL_SW_RST_CTRL_0_END (59)

#endif /* _DT_BINDINGS_BST_C1200_RESETS_H_ */