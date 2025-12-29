// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef __BST_MD_CSR_H_
#define __BST_MD_CSR_H_

#include <linux/device.h>

#define MD_CSR_DISPLAY0_CFG   0x10
#define MD_CSR_DISPLAY1_CFG   0x14

/* display mux */
#define MD_CSR_VOUT_CFG   0x18
#define MD_CSR_LVDS0_CFG   0x1C
#define MD_CSR_LVDS1_CFG   0x20
#define MD_CSR_LVDS0_PIX_NUM_CFG   0x24
#define MD_CSR_LVDS1_PIX_NUM_CFG   0x28
#define MD_CSR_EDP_CFG   0x30
#define MD_CSR_DSI0_CFG   0x34
#define MD_CSR_DSI1_CFG   0x38
#define MD_CSR_DSI2_CFG   0x3C

#define MD_CSR_DISP_SEL_MASK (0xF)
#define MD_CSR_DISP_SEL_SHIFT (0)

#define DISP_SEL_BUILD(_disp_id, _pipeline_id, _link_id) \
    ((((_disp_id) & 3) << 2) | (((_pipeline_id) & 1) << 1) | ((_link_id) & 1))

#define DISP_SEL_DISP0_CH0_LINK0   (0x0000)
#define DISP_SEL_DISP0_CH0_LINK1   (0x0001)
#define DISP_SEL_DISP0_CH1_LINK0   (0x0002)
#define DISP_SEL_DISP0_CH1_LINK1   (0x0003)
#define DISP_SEL_DISP1_CH0_LINK0   (0x0004)
#define DISP_SEL_DISP1_CH0_LINK1   (0x0005)
#define DISP_SEL_DISP1_CH1_LINK0   (0x0006)
#define DISP_SEL_DISP1_CH1_LINK1   (0x0007)
#define DISP_SEL_DISP2_CH0_LINK0   (0x0008)
#define DISP_SEL_DISP2_CH0_LINK1   (0x0009)

#define MD_CSR_MDNOC_WDT_EN   0x40
#define MD_CSR_MDNOC_WDT_TIMEOUT   0x44
#define MD_CSR_MDNOC_PTY_EN   0x48
#define MD_CSR_MDNOC_PTY_CLEAR   0x4C
#define MD_CSR_MDNOC_PTY_ERR_INJECT   0x50
#define MD_CSR_MDNOC_PTY_INTR0   0x54
#define MD_CSR_MDNOC_PTY_INTR1   0x58
#define MD_CSR_MDNOC_PTY_INTR2   0x5C
#define MD_CSR_CODEC0_REG   0x60
#define MD_CSR_CODEC1_REG   0x64
#define MD_CSR_CODEC0_TBU_ARMMUSSID_S   0x68
#define MD_CSR_CODEC1_TBU_ARMMUSSID_S   0x6C
#define MD_CSR_CODEC0_TBU_ARMMUSID_S   0x70
#define MD_CSR_CODEC1_TBU_ARMMUSID_S   0x74
#define MD_CSR_CODEC0_TBU_AWMMUSSID_S   0x78
#define MD_CSR_CODEC1_TBU_AWMMUSSID_S   0x7C
#define MD_CSR_CODEC0_TBU_AWMMUSID_S   0x80
#define MD_CSR_CODEC1_TBU_AWMMUSID_S   0x84
#define MD_CSR_CODEC0_TBU_MMU   0x88
#define MD_CSR_CODEC1_TBU_MMU   0x8C
#define MD_CSR_CODEC0_TBU_SID   0x90
#define MD_CSR_CODEC1_TBU_SID   0x94
#define MD_CSR_CODEC_TBU_CMO   0x98
#define MD_CSR_CODEC_TBU_TOK   0x9C
#define MD_CSR_CODEC_TBU_UTLB   0x100
#define MD_CSR_CODEC_TBU_ECOREVNUM1   0x104
#define MD_CSR_CODEC_TBU_SEC_OVERRIDE   0x108
#define MD_CSR_CODEC_TBU_IRPT   0x10C
#define MD_CSR_CODEC_TBU_PMU_REQ   0x110
#define MD_CSR_CODEC_TBU_PMU_ACK   0x114
#define MD_CSR_CODEC_TCU_PARITY   0x118
#define MD_CSR_CODEC_TCU_PARITY_INTR1   0x11C
#define MD_CSR_CODEC_TCU_SUP   0x120
#define MD_CSR_CODEC_TCU_IRPT   0x124
#define MD_CSR_CODEC_TCU_APB_BASEADDR1   0x128
#define MD_CSR_CODEC_TCU_FMU_INT   0x12C
#define MD_CSR_CODEC_TCU_ECOREVNUM1   0x130
#define MD_CSR_CODEC_TCU_SEC_OVERRIDE1   0x134
#define MD_CSR_CODEC_TCU_FMU_REQ   0x138
#define MD_CSR_CODEC_TCU_FMU_ACK   0x13C
#define MD_CSR_CODEC_TCU_SYSCO_REQ1   0x140
#define MD_CSR_CODEC_TCU_SYSCO_ACK1   0x144
#define MD_CSR_HIFI_APB_REG   0x148
#define MD_CSR_HIFI_INT_MUX_SEL1   0x14C
#define MD_CSR_HIFI_INT_MUX_SEL2   0x150
#define MD_CSR_HIFI_INT_MUX_SEL3   0x154
#define MD_CSR_HIFI_INT_MUX_SEL4   0x158
#define MD_CSR_HIFI_INT_MUX_SEL5   0x15C
#define MD_CSR_CODEC_CLK_QA_OUT   0x160
#define MD_CSR_CODEC_CLK_QA_IN   0x164
#define MD_CSR_MEDIA_APB_PAR1   0x168
#define MD_CSR_MEDIA_APB_PAR2   0x16C
#define MD_CSR_HIFI_PDEBUG1   0x170
#define MD_CSR_HIFI_PDEBUG2   0x174
#define MD_CSR_HIFI_PDEBUG3   0x178
#define MD_CSR_HIFI_PDEBUG4   0x17C
#define MD_CSR_HIFI_PDEBUG5   0x180
#define MD_CSR_HIFI_PDEBUG6   0x184
#define MD_CSR_HIFI_PDEBUG7   0x188
#define MD_CSR_HIFI_PDEBUG8   0x18C
#define MD_CSR_HIFI_PDEBUG9   0x190
#define MD_CSR_HIFI_PDEBUG10   0x194
#define MD_CSR_HIFI_PDEBUG11   0x198
#define MD_CSR_EDP_LOCAL_SEL   0x19C
#define MD_CSR_CODEC_TBU_MBISTREG   0x200

#define MD_CSR_DISPLAY0_SMMU_TBU_NSAID_CFG   0x204
#define MD_CSR_DISPLAY0_SMMU_TCU_NSAID_CFG   0x208
#define MD_CSR_DISPLAY1_SMMU_TBU_NSAID_CFG   0x20C
#define MD_CSR_DISPLAY1_SMMU_TCU_NSAID_CFG   0x210
#define MD_CSR_DISPLAY_SP_SMMU_TBU_NSAID_CFG   0x214
#define MD_CSR_DISPLAY_SP_SMMU_TCU_NSAID_CFG   0x218

#define MD_CSR_CODEC_SMMU_NSAID_CFG   0x21C

#define MD_CSR_DISPLAY_CODEC_FIF0PWRSTALL   0x220
#define MD_CSR_DISPLAY_CODEC_FIFOPWRACTIVE   0x224
#define MD_CSR_MD_POWER_IDLEREQ   0x228
#define MD_CSR_MD_POWER_IDEL   0x22C
#define MD_CSR_REG_WR_PROTECT   0x230
#define MD_CSR_MDNOC_PTY_INTR0_MASK   0x234
#define MD_CSR_MDNOC_PTY_INTR1_MASK   0x238
#define MD_CSR_MDNOC_PTY_INTR2_MASK   0x23C
#define MD_CSR_MDNOC_WDT_TIMEOUT_MASK   0x240
#define MD_CSR_MD_PARITY_MASK   0x244
#define MD_CSR_MEDIANOC_COMBINE_INTR   0x248
#define MD_CSR_MEDIANOC_COMBINE_INTR_MASK   0x24C
#define MD_CSR_CODEC_TCU_IRPT_MASK   0x250
#define MD_CSR_CODEC_TCU_FMU_INT_MASK   0x254
#define MD_CSR_MEDIA_BLOCK_SAFETY_FUNC   0x258
#define MD_CSR_MEDIA_BLOCK_SAFETY_FUNC_MASK   0x25C
#define MEDIA_DMA_MIPI_NASID	0x260

#define MEDIA_DISPLAY_NASID_SEL 0x264

#define MDNOC_PTY_ERR_INJECT1 0x268
#define ADDR_E_EDP_HIFI 0x26C


u32 bst_md_csr_read(struct device *dev, u32 reg);
void bst_md_csr_write(struct device *dev, u32 reg, u32 val);

void bst_disp_vout_mux_sel(struct device *dev, u32 disp_id, u32 pipeline_id, u32 link_id);
void bst_disp_edp_mux_sel(struct device *dev, u32 disp_id, u32 pipeline_id, u32 link_id);
void bst_disp_lvds_mux_sel(struct device *dev, u32 lvds_n, u32 disp_id, u32 pipeline_id, u32 link_id);
void bst_disp_dsi_mux_sel(struct device *dev, u32 dsi_n, u32 disp_id, u32 pipeline_id, u32 link_id);

#endif