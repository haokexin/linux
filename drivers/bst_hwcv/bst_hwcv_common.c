// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/io.h>
#include "bst_hwcv_common.h"

int bst_hwcv_get_bpp_by_format(enum format format)
{
	int bpp = 0;

	switch (format) {
	case RGB888:
		bpp = 3;
		break;
	case YUV422_YUYV:
	case YUV422_UYVY:
		bpp = 2;
		break;
	case NV12:
	case NV21:
	case RGB888_PLANAR:
	case YUV420_PLANAR:
	case YUV422_PLANAR:
		bpp = 1;
		break;
	default:
		bpp = 0;
	}

	return bpp;
}

int bst_hwcv_get_planar_num_by_format(enum format format)
{
	int planar_num;

	switch (format) {
	case RGB888:
	case YUV422_YUYV:
	case YUV422_UYVY:
		planar_num = 1;
		break;
	case NV12:
	case NV21:
	case YUV422_PLANAR:
		planar_num = 2;
		break;
	case RGB888_PLANAR:
	case YUV420_PLANAR:
		planar_num = 3;
		break;
	default:
		planar_num = 0;
	}

	return planar_num;
}

virt_addr_t bst_hwcv_map_reg(struct device *dev, uint32_t reg_addr)
{
	return devm_ioremap(dev, reg_addr, BST_HWCV_REG_BYTE);
}

void bst_hwcv_unmap_reg(struct device *dev, virt_addr_t vaddr)
{
	return devm_iounmap(dev, vaddr);
}

int bst_hwcv_map_regs(struct device *dev, uint32_t base_reg_addr,
		      uint32_t reg_cnt, virt_addr_t *dst)
{
	int i = 0;
	uint32_t reg_addr;

	dev_dbg(dev, "Map regs, reg count = %d, reg range = (0x%08x - 0x%08x)",
		reg_cnt, base_reg_addr,
		base_reg_addr + (reg_cnt - 1) * BST_HWCV_REG_BYTE);

	for (i = 0; i < reg_cnt; i++) {
		reg_addr = base_reg_addr + i * BST_HWCV_REG_BYTE;
		dst[i] = bst_hwcv_map_reg(dev, reg_addr);
		if (!dst[i]) {
			dev_err(dev, "Failed tp map reg[0x%08x]", reg_addr);
			return -ENOMEM;
		}
	}

	return 0;
}

void bst_hwcv_unmap_regs(struct device *dev, uint32_t reg_cnt, virt_addr_t *dst)
{
	int i = 0;

	for (i = 0; i < reg_cnt; i++) {
		if (dst[i])
			bst_hwcv_unmap_reg(dev, dst[i]);
	}
}
