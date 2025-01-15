// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/io.h>
#include <linux/device.h>

#include "bst_drm_dev.h"
#include "bst_dpu_csr.h"


static inline void __iomem * dev_to_reg_base(struct device *dev)
{
    struct bst_dev *mdev = dev_to_mdev(dev);
    return mdev->reg_base;
}

static inline u32
dpu_csr_read(u32 __iomem *base, u32 offset)
{
	return readl((base + (offset >> 2)));
}

static inline void
dpu_csr_write(u32 __iomem *base, u32 offset, u32 v)
{
	writel(v, (base + (offset >> 2)));
}

#define VPG_CSR_CNT_INVALID		 (0xFFFFFFFF)
#define VPG_CSR_OFFSET			  (0x40000UL)
#define VPG_CSR_WR_PROTECT		  (VPG_CSR_OFFSET + 0xa8)
#define VPG_CSR_READY_BYPASS		(VPG_CSR_OFFSET + 0x3c)
#define VPG_CSR_CNT_CFG			 (VPG_CSR_OFFSET + 0x10)
#define VPG_CSR_CNT_FRAME0		  (VPG_CSR_OFFSET + 0x14)
#define VPG_CSR_CNT_FRAME1		  (VPG_CSR_OFFSET + 0x18)
#define VPG_CSR_CNT_VSYNC_CH0_LN0   (VPG_CSR_OFFSET + 0x1C)
#define VPG_CSR_CNT_VSYNC_CH0_LN1   (VPG_CSR_OFFSET + 0x20)
#define VPG_CSR_CNT_VSYNC_CH1_LN0   (VPG_CSR_OFFSET + 0x24)
#define VPG_CSR_CNT_VSYNC_CH1_LN1   (VPG_CSR_OFFSET + 0x28)
#define VPG_CSR_CNT_HSYNC_CH0_LN0   (VPG_CSR_OFFSET + 0x2C)
#define VPG_CSR_CNT_HSYNC_CH0_LN1   (VPG_CSR_OFFSET + 0x30)
#define VPG_CSR_CNT_HSYNC_CH1_LN0   (VPG_CSR_OFFSET + 0x34)
#define VPG_CSR_CNT_HSYNC_CH1_LN1   (VPG_CSR_OFFSET + 0x38)

void bst_csr_clear_frame_counter(struct device *dev, int ch)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
	dpu_csr_write(reg_base, VPG_CSR_CNT_CFG, 0x200 >> ch);
}


u32 bst_csr_update_read_frame_counter(struct device *dev, int ch)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
	u32 reg;
	u32 offset_addr[2] = {VPG_CSR_CNT_FRAME0, VPG_CSR_CNT_FRAME1};

	reg = dpu_csr_read(reg_base, VPG_CSR_CNT_CFG);
	reg &= ~(0x2000000 >> ch);
	dpu_csr_write(reg_base, VPG_CSR_CNT_CFG, reg);
	reg |= 0x2000000 >> ch;
	reg &= ~(0x3FF);
	dpu_csr_write(reg_base, VPG_CSR_CNT_CFG, reg);

	return dpu_csr_read(reg_base, offset_addr[ch]);
}

void bst_csr_update_vsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_write(reg_base, COUNTER_CFG, VSYNC_COUNTER_CH0_LN0_UPDATE >> offset);
}

void bst_csr_update_hsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_write(reg_base, COUNTER_CFG, HSYNC_COUNTER_CH0_LN0_UPDATE >> offset);
}

void bst_csr_clear_vsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_write(reg_base, COUNTER_CFG, VSYNC_COUNTER_CH0_LN0_CLR >> offset);
}

void bst_csr_clear_hsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_write(reg_base, COUNTER_CFG, VSYNC_COUNTER_CH0_LN0_UPDATE >> offset);
}

void bst_csr_read_vsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_read(reg_base, VSYNC_COUNTER_CH0_LN0 + offset *sizeof(u32));
}

void bst_csr_read_hsync_counter(struct device *dev, int ch, int link)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 offset = (ch << 1) | link;

    dpu_csr_read(reg_base, HSYNC_COUNTER_CH0_LN0 + offset *sizeof(u32));
}

u32 bst_dpu_reg_read(struct device *dev, u32 reg)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    return readl(reg_base + reg);
}
EXPORT_SYMBOL(bst_dpu_reg_read);

void bst_dpu_reg_write(struct device *dev, u32 reg, u32 val)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    writel(val, reg_base + reg);
}
EXPORT_SYMBOL(bst_dpu_reg_write);

void bst_dpu_software_reset(struct device *dev)
{
    void __iomem *reg_base = dev_to_reg_base(dev);
    u32 reg;

    reg = readl(reg_base + 0x00D0);
    reg |= (1 << BIT_SRST);
    writel(reg, reg_base + 0x00D0);
    reg &= (~(1 << BIT_SRST));
    writel(reg, reg_base + 0x00D0);
}

void bst_dpu_check_and_release(struct device *dev)
{
    void __iomem *dpu_base = dev_to_reg_base(dev);
    u32 arch_id, core_id, core_info, ready_bypass;

    arch_id = readl(dpu_base + ARCH_ID);
    core_id = readl(dpu_base + CORE_ID);
    core_info = readl(dpu_base + CORE_INFO);

    writel(0xabcd1234, dpu_base + REG_WR_PROTECT);
    writel(0xFFFF, dpu_base + READY_BYPASS);
    ready_bypass = readl(dpu_base + READY_BYPASS);

    if (!(arch_id == 0x44501000 && core_id == 0x711000)) {
        bst_dpu_software_reset(dev);
    }
}
EXPORT_SYMBOL(bst_dpu_check_and_release);
