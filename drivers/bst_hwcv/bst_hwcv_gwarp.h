/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_GWARP_H__
#define __BST_HWCV_GWARP_H__
#include "bst_hwcv_ioctl.h"
#include "bst_hwcv_common.h"

enum gwarp_mode {
	BICUBIC,
	BILINEAR
};

struct gwarp_regs {
	virt_addr_t enable;
	virt_addr_t sys_ctl;
	virt_addr_t src_resolution;
	virt_addr_t dst_resolution;
	virt_addr_t src_base_ch[3];
	virt_addr_t dst_base_ch[3];
	virt_addr_t lut_base;
	virt_addr_t src_stride;
	virt_addr_t dst_stride;
	virt_addr_t lut_stride;
	virt_addr_t axi_parameter;
	virt_addr_t intr;
	virt_addr_t intr_en;
	virt_addr_t safety_error;
	virt_addr_t safety_mask;
	virt_addr_t safety_inject;
	virt_addr_t crc_grp[8];
};

struct sbs_gwarp_regs {
	virt_addr_t enable;
	virt_addr_t sys_ctl;
	virt_addr_t src_resolution;
	virt_addr_t dst_resolution;
	virt_addr_t src_base_ch[3];
	virt_addr_t dst_base_ch[3];
	virt_addr_t lut_base;
	virt_addr_t src_stride;
	virt_addr_t dst_stride;
	virt_addr_t lut_stride;
	virt_addr_t lut_distribution;
	virt_addr_t update_sync;
	virt_addr_t cirbuf_rid;
	virt_addr_t cirbuf_wid;
	virt_addr_t rownum_offset;
};

int bst_gwarp_map_all_regs(struct device *dev);
uint32_t bst_gwarp_read_intr(struct device *dev, uint8_t engine_id);
void bst_gwarp_clear_intr(struct device *dev, uint8_t engine_id);
void bst_gwarp_enable(struct device *dev, uint8_t engine_id);
void bst_gwarp_disable(struct device *dev, uint8_t engine_id);
int bst_gwarp_start(struct device *dev, const struct hwcv_gwarp_data *data);

void bst_sbs_gwarp_clear_intr(struct device *dev, uint8_t engine_id,
			      uint8_t sensor_id);
void bst_sbs_gwarp_enable(struct device *dev, uint8_t engine_id,
			  uint8_t sensor_id);
void bst_sbs_gwarp_disable(struct device *dev, uint8_t engine_id,
			   uint8_t sensor_id);
int bst_sbs_gwarp_start(struct device *dev,
			const struct hwcv_sbs_gwarp_data *data);

#endif
