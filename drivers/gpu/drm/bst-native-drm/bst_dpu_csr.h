// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef __BST_DPU_CSR_H_
#define __BST_DPU_CSR_H_

#include  <linux/device.h>

#define BIT_SRST                (16)
#define BUS_WIDTH_16_BYTES      (16)
#define ARCH_ID 	        (0x0000)
#define CORE_ID	            (0x0004)
#define CORE_INFO	        (0x0008)
#define DISP_APB_REG_OFFSET (0x40000UL)

#define LOCAL_CTRL (DISP_APB_REG_OFFSET + 0x00)
#define PARITY_CTRL (DISP_APB_REG_OFFSET + 0x04)
#define PARITY_INTR (DISP_APB_REG_OFFSET + 0x08)
#define COUNTER_CFG (DISP_APB_REG_OFFSET + 0x10)

#define FRAME_COUTER_CH0_UPDATE	(1 << 25)
#define FRAME_COUTER_CH1_UPDATE	(1 << 24)
#define VSYNC_COUNTER_CH0_LN0_UPDATE	(1 << 23)
#define VSYNC_COUNTER_CH0_LN1_UPDATE	(1 << 22)
#define VSYNC_COUNTER_CH1_LN0_UPDATE	(1 << 21)
#define VSYNC_COUNTER_CH1_LN1_UPDATE	(1 << 20)
#define HSYNC_COUNTER_CH0_LN0_UPDATE	(1 << 19)
#define HSYNC_COUNTER_CH0_LN1_UPDATE	(1 << 18)
#define HSYNC_COUNTER_CH1_LN0_UPDATE	(1 << 17)
#define HSYNC_COUNTER_CH1_LN1_UPDATE	(1 << 16)
#define FRAME_COUTER_CH0_CLR	(1 << 9)
#define FRAME_COUTER_CH1_CLR	(1 << 8)
#define VSYNC_COUNTER_CH0_LN0_CLR	(1 << 7)
#define VSYNC_COUNTER_CH0_LN1_CLR	(1 << 6)
#define VSYNC_COUNTER_CH1_LN0_CLR	(1 << 5)
#define VSYNC_COUNTER_CH1_LN1_CLR	(1 << 4)
#define HSYNC_COUNTER_CH0_LN0_CLR	(1 << 3)
#define HSYNC_COUNTER_CH0_LN1_CLR	(1 << 2)
#define HSYNC_COUNTER_CH1_LN0_CLR	(1 << 1)
#define HSYNC_COUNTER_CH1_LN1_CLR	(1 << 0)

#define FRAME_COUNTER_CH0 (DISP_APB_REG_OFFSET + 0x14)
#define FRAME_COUNTER_CH1 (DISP_APB_REG_OFFSET + 0x18)
#define VSYNC_COUNTER_CH0_LN0 (DISP_APB_REG_OFFSET + 0x1c)
#define VSYNC_COUNTER_CH0_LN1 (DISP_APB_REG_OFFSET + 0x20)
#define VSYNC_COUNTER_CH1_LN0 (DISP_APB_REG_OFFSET + 0x24)
#define VSYNC_COUNTER_CH1_LN1 (DISP_APB_REG_OFFSET + 0x28)
#define HSYNC_COUNTER_CH0_LN0 (DISP_APB_REG_OFFSET + 0x2c)
#define HSYNC_COUNTER_CH0_LN1 (DISP_APB_REG_OFFSET + 0x30)
#define HSYNC_COUNTER_CH1_LN0 (DISP_APB_REG_OFFSET + 0x34)
#define HSYNC_COUNTER_CH1_LN1 (DISP_APB_REG_OFFSET + 0x38)
#define READY_BYPASS          (DISP_APB_REG_OFFSET + 0x3c)
#define STAGE1_TBU_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x50)
#define STAGE1_TBU_TIE_OFF1 (DISP_APB_REG_OFFSET + 0x54)
#define STAGE1_TCU_TIE_OFF  (DISP_APB_REG_OFFSET + 0x58)
#define STAGE2_TBU0_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x60)
#define STAGE2_TBU0_TIE_OFF1 (DISP_APB_REG_OFFSET + 0x64)
#define STAGE2_TBU1_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x68)
#define STAGE2_TBU1_TIE_OFF1 (DISP_APB_REG_OFFSET + 0x6c)
#define STAGE2_TBU2_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x70)
#define STAGE2_TBU2_TIE_OFF1 (DISP_APB_REG_OFFSET + 0x74)
#define STAGE2_TBU3_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x78)
#define STAGE2_TBU3_TIE_OFF1 (DISP_APB_REG_OFFSET + 0x7c)
#define STAGE2_TBU23_TIE_OFF0 (DISP_APB_REG_OFFSET + 0x80)
#define STAGE2_TCU_TIE_OFF   (DISP_APB_REG_OFFSET + 0x84)
#define SMMU_INTR_CTRL (DISP_APB_REG_OFFSET + 0x88)
#define SMMU_INTR_FLG0 (DISP_APB_REG_OFFSET + 0x8c)
#define SMMU_INTR_FLG1 (DISP_APB_REG_OFFSET + 0x90)
#define SMMU_INTR_FLG2 (DISP_APB_REG_OFFSET + 0x94)
#define PMU_SNAPSHOT   (DISP_APB_REG_OFFSET + 0x98)

#define SAFETY_INTR_MASK (DISP_APB_REG_OFFSET + 0x9c)
#define SMMU_INTR_S_MASK (DISP_APB_REG_OFFSET + 0xa0)
#define SMMU_INTR_NS_MASK (DISP_APB_REG_OFFSET + 0xa4)

#define REG_WR_PROTECT (DISP_APB_REG_OFFSET + 0xa8)

#define SMMU_STAGE1_TCU_BASE_ADDR (DISP_APB_REG_OFFSET + 0xac)
#define SMMU_STAGE2_TCU_BASE_ADDR (DISP_APB_REG_OFFSET + 0xb0)

#define PINMUX_BASE_ADDR ((void *)0x30001000)

void bst_csr_clear_frame_counter(struct device *dev, int ch);
void bst_csr_update_frame_counter(struct device *dev, int ch);
u32 bst_csr_read_frame_counter(struct device *dev, int ch);

u32 bst_csr_update_read_frame_counter(struct device *dev, int ch);

void bst_csr_update_vsync_counter(struct device *dev, int ch, int link);
void bst_csr_clear_vsync_counter(struct device *dev, int ch, int link);
void bst_csr_read_vsync_counter(struct device *dev, int ch, int link);

void bst_csr_update_hsync_counter(struct device *dev, int ch, int link);
void bst_csr_clear_hsync_counter(struct device *dev, int ch, int link);
void bst_csr_read_hsync_counter(struct device *dev, int ch, int link);

u32 bst_dpu_reg_read(struct device *dev, u32 reg);
void bst_dpu_reg_write(struct device *dev, u32 reg, u32 val);

void bst_dpu_software_reset(struct device *dev);
void bst_dpu_check_and_release(struct device *dev);



#endif