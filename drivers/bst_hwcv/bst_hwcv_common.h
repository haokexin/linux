/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_COMMON_H__
#define __BST_HWCV_COMMON_H__
#include <linux/device.h>

#define BST_HWCV_GWARP_ENGINE_NUM 2
#define BST_HWCV_GWARP_SNR_NUM 4
#define MAX_HWCV_PLANAR_NUM 3
#define BST_HWCV_REG_BYTE 4

#define SET_BIT(x, val, bit) \
	((val) ? ((x) |= (1 << (bit))) : ((x) &= ~(1 << (bit))))
#define SET_BITS(x, val, sbit, len)                   \
	do {                                          \
		(x) &= ~(((1 << len) - 1) << (sbit)); \
		(x) |= ((val) << (sbit));             \
	} while (0)

typedef void __iomem *virt_addr_t;

enum format {
	RGB888,
	RGB888_PLANAR,
	YUV422_YUYV,
	YUV422_UYVY,
	NV12,
	NV21,
	YUV420_PLANAR,
	YUV422_PLANAR,
};

int bst_hwcv_get_bpp_by_format(enum format format);
int bst_hwcv_get_planar_num_by_format(enum format format);

virt_addr_t bst_hwcv_map_reg(struct device *dev, uint32_t reg_addr);
void bst_hwcv_unmap_reg(struct device *dev, virt_addr_t vaddr);
int bst_hwcv_map_regs(struct device *dev, uint32_t base_reg_addr,
		      uint32_t reg_cnt, virt_addr_t *dst);
void bst_hwcv_unmap_regs(struct device *dev, uint32_t reg_cnt,
			 virt_addr_t *dst);

#endif
