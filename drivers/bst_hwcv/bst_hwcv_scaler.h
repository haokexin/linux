/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef BST_HWCV_SCALER_H
#define BST_HWCV_SCALER_H
#include "bst_hwcv_ioctl.h"
#include "bst_hwcv_common.h"

struct scaler_regs {
	virt_addr_t enable;
	virt_addr_t sys_ctrl;
	virt_addr_t memctrl_intr_status;
	virt_addr_t algorithm_intr_status;
	virt_addr_t src_base_ch[3];
	virt_addr_t src_resolution;
	virt_addr_t dst_base_ch[3];
	virt_addr_t dst_resolution;
	virt_addr_t x_ratio;
	virt_addr_t x_init_phase;
	virt_addr_t y_ratio;
	virt_addr_t y_init_phase;
	virt_addr_t axi_param;
	virt_addr_t axi_stride;
	virt_addr_t coeff_addr;
	virt_addr_t coeff_size;
	virt_addr_t dma_para;
	virt_addr_t bank_remap_ch[2];
	virt_addr_t reserved;
	virt_addr_t dst_layer1_base_ch[3];
	virt_addr_t dst_layer2_base_ch[3];
};

int bst_scaler_map_all_regs(struct device *dev);
int bst_scaler_read_intr(struct device *dev);
void bst_scaler_clear_intr(struct device *dev);
void bst_scaler_enable(struct device *dev);
void bst_scaler_disable(struct device *dev);
void bst_scaler_soft_reset(struct device *dev);

int bst_scaler_start(struct device *dev, struct hwcv_scaler_data *data);

#endif
