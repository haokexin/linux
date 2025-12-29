/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_HW_V2_H__
#define __BST_HWCV_HW_V2_H__

#define HWCV_GWARP_NUM 2
#define HWCV_GWARP_SBS_SENSOR_NUM 4

/* Core top register definitions */
#define CV_SYS_CTRL_STATUS (0x00)
#define CV_PARITY_CTRL_REG0 (0x50)
#define NOC_S_PORT_CHK_PTY_INTR_REG0 (0x78)
#define CV_INTR_EN_REG (0xD4)
#define CV_SUBMODULE_INTR_REG (0xD8)

/* CV_SYS_CTRL_STATUS */
#define BIT_GWARP0_CLK_EN BIT(12)
#define BIT_GWARP1_CLK_EN BIT(13)
#define BIT_SCLR_CLK_EN BIT(14)
#define BIT_DMA_CLK_EN BIT(15)
#define BIT_SOFT_RST_GWARP0 BIT(16)
#define BIT_SOFT_RST_GWARP1 BIT(17)
#define BIT_SOFT_RST_SCLR BIT(18)
#define BIT_SOFT_RST_DMA BIT(19)

/* CV_PARITY_CTRL_REG0 */
#define BIT_INTERNAL_ECC_EN BIT(0)
#define BIT_INTERNAL_PTY_EN BIT(1)

/* CV_INTR_EN_REG */
#define BIT_SCALER_FUNC_INTR_EN BIT(4)
#define BIT_GWARP0_FUNC_INTR_EN BIT(5)
#define BIT_GWARP1_FUNC_INTR_EN BIT(6)
#define BIT_FUNC_INTR_OUTPUT_EN BIT(29)

/* Scaler register definitions */
#define SCLR_ENABLE (0x00)
#define SCLR_SYS_CTRL (0x04)
#define SCLR_MEMCTRL_INTR_STATUS (0x08)
#define SCLR_ALGRTHM_INTR_STATUS (0X0c)
#define SCLR_SRC_Y_ADDR (0x10)
#define SCLR_SRC_U_ADDR (0x14)
#define SCLR_SRC_V_ADDR (0x18)
#define SCLR_SRC_RESOLUTION (0x1c)
#define SCLR_DST_Y_ADDR (0x20)
#define SCLR_DST_U_ADDR (0x24)
#define SCLR_DST_V_ADDR (0x28)
#define SCLR_DST_RESOLUTION (0x2c)
#define SCLR_X_RATIO (0x30)
#define SCLR_X_INIT_PHASE (0x34)
#define SCLR_Y_RATIO (0x38)
#define SCLR_Y_INIT_PHASE (0x3c)
#define SCLR_AXI_PARA (0x40)
#define SCLR_AXI_STRIDE (0x44)
#define SCLR_COEFF_ADDR (0x48)
#define SCLR_COEFF_SIZE (0x4c)
#define SCLR_SCLR_DMA_PARA (0x50)
#define SCLR_CHO_BANK_REMAP (0x54)
#define SCLR_CH1_BANK_REMAP (0x58)
#define SCLR_DST_LAYER1_Y_ADDR (0x60)
#define SCLR_DST_LAYER1_U_ADDR (0x64)
#define SCLR_DST_LAYER1_V_ADDR (0x68)
#define SCLR_DST_LAYER2_Y_ADDR (0x6c)
#define SCLR_DST_LAYER2_U_ADDR (0x70)
#define SCLR_DST_LAYER2_V_ADDR (0x74)
#define SCLR_FRAME_CYCLE_CNT (0x100)

/* SCLR_SYS_CTRL */
#define BIT_FRAME_DONE_INTR_CLEAN BIT(16)

/* SCLR_MEMCTRL_INTR_STATUS */
#define BIT_CSR_FRAME_DONE_INTR BIT(0)

/* Gwarp register definitions */
#define GWC_ENABLE (0x00)
#define GWC_SYS_CTRL (0x04)
#define GWC_SRC_RESOLUTION (0x08)
#define GWC_DST_RESOLUTION (0x0c)
#define GWC_SRC_BASE_CH0 (0x10)
#define GWC_SRC_BASE_CH1 (0x14)
#define GWC_SRC_BASE_CH2 (0x18)
#define GWC_DST_BASE_CH0 (0x1c)
#define GWC_DST_BASE_CH1 (0x20)
#define GWC_DST_BASE_CH2 (0x24)
#define GWC_LUT_BASE (0x28)
#define GWC_SRC_STRIDE (0x2c)
#define GWC_DST_STRIDE (0x30)
#define GWC_LUT_STRIDE (0x34)
#define GWC_AXI_PARAMETER (0x38)
#define GWC_INTR (0x3c)
#define GWC_INTR_EN (0x40)
#define GWC_SAFETY_ERROR (0x44)
#define GWC_SAFETY_MASK (0x48)
#define GWC_SAFETY_INJECT (0x4c)
/* #define GWC_CRC_GRPX	(0x50~0x6c)*/
#define GWC_CRC_GRP0 (0x50)
#define GWC_CFG_VIOLATION_INT (0x70)
#define GWC_CFG_VIOLATION_MASK (0x74)
/* #define GWC_DEBUGX	(0x78~0xd8) */
#define CSR_FRAME_CYCLE_CNT (0x80)

#define SNR0_GWC_ENABLE (0x100)
#define SNR0_GWC_SYS_CTRL (0x104)
#define SNR0_SRC_RESOLUTION (0x108)
#define SNR0_DST_RESOLUTION (0x10c)
#define SNR0_GWC_SRC_BASE_CH0 (0x110)
#define SNR0_GWC_SRC_BASE_CH1 (0x114)
#define SNR0_GWC_SRC_BASE_CH2 (0x118)
#define SNR0_GWC_DST_BASE_CH0 (0x11c)
#define SNR0_GWC_DST_BASE_CH1 (0x120)
#define SNR0_GWC_DST_BASE_CH2 (0x124)
#define SNR0_GWC_LUT_BASE (0x128)
#define SNR0_GWC_SRC_STRIDE (0x12c)
#define SNR0_GWC_DST_STRIDE (0x130)
#define SNR0_GWC_LUT_STRIDE (0x134)
#define SNR0_GWC_LUT_DISTRIBUTION (0x138)
#define SNR0_GWC_UPDATE_SYNC (0x13c)
#define SNR0_GWC_CIRBUF_RID (0x140)
#define SNR0_GWC_CIRBUF_WID (0x144)
#define SNR0_GWC_ROWNUM_OFFSET (0x148)

#define SNR_GWC_OFFSET (0x100)

/* GWC_INTR */
#define BIT_CSR_GWC_INTR BIT(0)
#define BIT_CSR_SNR0_INTR BIT(8)
#define BIT_CSR_SNR1_INTR BIT(9)
#define BIT_CSR_SNR2_INTR BIT(10)
#define BIT_CSR_SNR3_INTR BIT(11)

extern const struct hwcv_backend_ops hwcv_v2_ops;
extern const struct hwcv_hw_data hwcv_v2_data;

#endif
