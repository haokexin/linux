// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/io.h>
#include "bst_hwcv_scaler.h"

#define CV_SYS_CTRL_STATUS 0x51000000
#define SOFT_RST_SCLR_BIT 18
#define BST_HWCV_SCALER_BASE_PHY 0x51020000

/* 0x51020004 */
#define BIT_SYS_CTRL_FRAME_DONE_INTR_CLR 16
#define BIT_SYS_CTRL_FRAME_DONE_INTR_EN 15
#define BIT_SYS_CTRL_LEFTEDGE_SPLIT_FLAG 9
#define BITS_SYS_CTRL_PRAMID_MAX_LAYERNUM 7
#define LENGTH_SYS_CTRL_PRAMID_MAX_LAYERNUM 2
#define BIT_SYS_CTRL_GAUSS_EN 6
#define BITS_SYS_CTRL_DATA_FORMAT 3
#define LENGTH_SYS_CTRL_DATA_FORMAT 3
#define BIT_SYS_CTRL_SCALER_MODE 2

/* 0x51020008 */
#define BIT_FRAME_DONE_INTR 0

/* 0x5102001c */
#define BITS_SRC_RES_HEIGHT 16
#define LENGTH_SRC_RES_HEIGHT 13
#define BITS_SRC_RES_WIDTH 0
#define LENGTH_SRC_RES_WIDTH 13

/* 0x5102002c */
#define BITS_DST_RES_HEIGHT 16
#define LENGTH_DST_RES_HEIGHT 13
#define BITS_DST_RES_WIDTH 0
#define LENGTH_DST_RES_WIDTH 13

/* 0x51020044 */
#define BITS_DST_STRIDE 16
#define LENGTH_DST_STRIDE 16
#define BITS_SRC_STRIDE 0
#define LENGTH_SRC_STRIDE 16

static struct scaler_regs global_scaler_regs;
static virt_addr_t cv_sys_ctrl_status;

int bst_scaler_map_all_regs(struct device *dev)
{
	uint32_t base_reg_addr;
	uint32_t reg_cnt;

	cv_sys_ctrl_status = bst_hwcv_map_reg(dev, CV_SYS_CTRL_STATUS);

	base_reg_addr = BST_HWCV_SCALER_BASE_PHY;
	reg_cnt = sizeof(struct scaler_regs) / sizeof(virt_addr_t);

	return bst_hwcv_map_regs(dev, base_reg_addr, reg_cnt,
				 (virt_addr_t *)&global_scaler_regs);
}

int bst_scaler_read_intr(struct device *dev)
{
	uint32_t reg;

	reg = readl_relaxed(global_scaler_regs.memctrl_intr_status) &
	      (1 << BIT_FRAME_DONE_INTR);
	dev_dbg(dev, "Scaler Read Interrupt: %d", reg);

	return reg;
}

void bst_scaler_clear_intr(struct device *dev)
{
	uint32_t reg;

	dev_dbg(dev, "Scaler Clear Interrupt");
	reg = readl_relaxed(global_scaler_regs.sys_ctrl);
	SET_BIT(reg, 1, BIT_SYS_CTRL_FRAME_DONE_INTR_CLR);
	writel_relaxed(reg, global_scaler_regs.sys_ctrl);
	SET_BIT(reg, 0, BIT_SYS_CTRL_FRAME_DONE_INTR_CLR);
	writel_relaxed(reg, global_scaler_regs.sys_ctrl);
}

void bst_scaler_enable(struct device *dev)
{
	dev_dbg(dev, "Scaler Enable");
	writel_relaxed(1, global_scaler_regs.enable);
}

void bst_scaler_disable(struct device *dev)
{
	dev_dbg(dev, "Scaler Disable");
	writel_relaxed(0, global_scaler_regs.enable);
}

void bst_scaler_soft_reset(struct device *dev)
{
	uint32_t reg;

	dev_dbg(dev, "Scaler Reset");
	reg = readl_relaxed(cv_sys_ctrl_status);
	SET_BIT(reg, 0, SOFT_RST_SCLR_BIT);
	writel_relaxed(reg, cv_sys_ctrl_status);
	SET_BIT(reg, 1, SOFT_RST_SCLR_BIT);
	writel_relaxed(reg, cv_sys_ctrl_status);
}

/*---------------------------------------------------------------------------------------------*/

int _bst_scaler_dump_input(struct device *dev, struct hwcv_scaler_data *data)
{
	int i, j;
	int planar_num = bst_hwcv_get_planar_num_by_format(data->format);

	dev_dbg(dev, "--- Dump Input Scaler Params ---");
	dev_dbg(dev, "mode: %d", data->mode);
	dev_dbg(dev, "gauss: %d, leftedge: %d, layer: %d", data->gauss_enable,
		data->leftedge_split_flag, data->layer_num);
	dev_dbg(dev, "format: %d, planar_num: %d", data->format, planar_num);
	dev_dbg(dev, "resolution: src=%dx%d, dst=%dx%d", data->src_width,
		data->src_height, data->dst_width, data->dst_height);
	dev_dbg(dev, "stride: src=%d, dst=%d", data->src_stride,
		data->dst_stride);
	for (i = 0; i < MAX_HWCV_PLANAR_NUM; i++) {
		dev_dbg(dev, "src_phy_addr[%d]: 0x%08x", i,
			data->src_phy_addr[i]);
	}
	for (i = 0; i < MAX_HWCV_LAYER_NUM; i++) {
		for (j = 0; j < MAX_HWCV_PLANAR_NUM; j++) {
			dev_dbg(dev, "dst_phy_addr[%d][%d]: 0x%08x", i, j,
				data->dst_phy_addr[i][j]);
		}
	}
	dev_dbg(dev, "coeff_phy_addr: 0x%08x", data->coeff_phy_addr);
	dev_dbg(dev, "coeff_size: %d", data->coeff_size);
	dev_dbg(dev, "ratio: x=%d, y=%d", data->x_ratio, data->y_ratio);
	dev_dbg(dev, "phase: x=%d, y=%d", data->x_init_phase,
		data->y_init_phase);

	if (data->mode > 1) {
		dev_err(dev, "Invalid mode: %d", data->mode);
		return -EINVAL;
	}

	if (data->gauss_enable > 1 || data->leftedge_split_flag > 1 ||
	    data->layer_num > 3) {
		dev_err(dev, "Invalid pyr params: %d %d %d", data->gauss_enable,
			data->leftedge_split_flag, data->layer_num);
		return -EINVAL;
	}

	if (data->format > 6) {
		dev_err(dev, "Invalid format: %d", data->format);
		return -EINVAL;
	}

	if (!data->src_width || !data->src_height) {
		dev_err(dev, "Invalid src resolution: %dx%d", data->src_width,
			data->src_height);
		return -EINVAL;
	}

	if (!data->mode && (!data->dst_width || !data->dst_height)) {
		dev_err(dev, "Invalid dst resolution: %dx%d", data->dst_width,
			data->dst_height);
		return -EINVAL;
	}

	if (!data->src_stride || data->src_stride % 16 != 0 ||
	    !data->dst_stride || data->dst_stride % 16 != 0) {
		dev_err(dev, "Invalid stride: %d, %d", data->src_stride,
			data->dst_stride);
		return -EINVAL;
	}

	for (i = 0; i < planar_num; i++) {
		if (!data->src_phy_addr[i]) {
			dev_err(dev, "Invalid src paddr[%d]: 0x%08x", i,
				data->src_phy_addr[i]);
			return -EINVAL;
		}
	}

	for (i = 0; i < (!data->mode ? 1 : data->layer_num); i++) {
		for (j = 0; j < planar_num; j++) {
			if (!data->dst_phy_addr[i][j] ||
			    data->dst_phy_addr[i][j] % 16 != 0) {
				dev_err(dev,
					"Invalid dst paddr[%d][%d]: 0x%08x", i,
					j, data->dst_phy_addr[i][j]);
				return -EINVAL;
			}
		}
	}

	if (!data->mode && (!data->coeff_phy_addr || !data->coeff_size)) {
		dev_err(dev, "Invalid poly table: %d, %d", data->coeff_phy_addr,
			data->coeff_size);
		return -EINVAL;
	}

	if (!data->mode && (!data->x_ratio || !data->y_ratio)) {
		dev_err(dev, "Invalid poly ratio: %d, %d", data->x_ratio,
			data->y_ratio);
		return -EINVAL;
	}

	return 0;
}

static void _scaler_config_sys_ctrl(const struct hwcv_scaler_data *data)
{
	uint32_t sys_ctrl;

	sys_ctrl = readl_relaxed(global_scaler_regs.sys_ctrl);
	SET_BIT(sys_ctrl, 1, BIT_SYS_CTRL_FRAME_DONE_INTR_EN);
	SET_BITS(sys_ctrl, data->format, BITS_SYS_CTRL_DATA_FORMAT,
		 LENGTH_SYS_CTRL_DATA_FORMAT);
	if (!data->mode) {
		SET_BIT(sys_ctrl, data->mode, BIT_SYS_CTRL_SCALER_MODE);
	} else {
		SET_BIT(sys_ctrl, data->mode, BIT_SYS_CTRL_SCALER_MODE);
		SET_BIT(sys_ctrl, data->gauss_enable, BIT_SYS_CTRL_GAUSS_EN);
		SET_BITS(sys_ctrl, data->layer_num,
			 BITS_SYS_CTRL_PRAMID_MAX_LAYERNUM,
			 LENGTH_SYS_CTRL_PRAMID_MAX_LAYERNUM);
		SET_BIT(sys_ctrl, data->leftedge_split_flag,
			BIT_SYS_CTRL_LEFTEDGE_SPLIT_FLAG);
	}
	writel_relaxed(sys_ctrl, global_scaler_regs.sys_ctrl);
}

static void _scaler_config_resolution(const struct hwcv_scaler_data *data)
{
	uint32_t src_res = 0;
	uint32_t dst_res = 0;

	SET_BITS(src_res, data->src_width, BITS_SRC_RES_WIDTH,
		 LENGTH_SRC_RES_WIDTH);
	SET_BITS(src_res, data->src_height, BITS_SRC_RES_HEIGHT,
		 LENGTH_SRC_RES_HEIGHT);
	writel_relaxed(src_res, global_scaler_regs.src_resolution);

	if (!data->mode) {
		SET_BITS(dst_res, data->dst_width, BITS_DST_RES_WIDTH,
			 LENGTH_DST_RES_WIDTH);
		SET_BITS(dst_res, data->dst_height, BITS_DST_RES_HEIGHT,
			 LENGTH_DST_RES_HEIGHT);
		writel_relaxed(dst_res, global_scaler_regs.dst_resolution);
	}
}

static void _scaler_config_ratio(const struct hwcv_scaler_data *data)
{
	if (!data->mode) {
		writel_relaxed(data->x_ratio, global_scaler_regs.x_ratio);
		writel_relaxed(data->y_ratio, global_scaler_regs.y_ratio);
	}
}

static void _scaler_config_phase(const struct hwcv_scaler_data *data)
{
	if (!data->mode) {
		writel_relaxed(data->x_init_phase,
			       global_scaler_regs.x_init_phase);
		writel_relaxed(data->y_init_phase,
			       global_scaler_regs.y_init_phase);
	}
}

static void _scaler_config_stride(const struct hwcv_scaler_data *data)
{
	uint32_t axi_stride = 0;

	SET_BITS(axi_stride, data->src_stride, BITS_SRC_STRIDE,
		 LENGTH_SRC_STRIDE);
	SET_BITS(axi_stride, data->dst_stride, BITS_DST_STRIDE,
		 LENGTH_DST_STRIDE);
	writel_relaxed(axi_stride, global_scaler_regs.axi_stride);
}

static void _scaler_config_addr(const struct hwcv_scaler_data *data)
{
	int i, j;
	int planar_num = bst_hwcv_get_planar_num_by_format(data->format);
	int layer_num = (!data->layer_num) ? 1 : data->layer_num;

	for (i = 0; i < planar_num; i++) {
		writel_relaxed(data->src_phy_addr[i],
			       global_scaler_regs.src_base_ch[i]);
	}

	for (i = 0; i < layer_num; i++) {
		for (j = 0; j < planar_num; j++) {
			if (i == 0)
				writel_relaxed(
					data->dst_phy_addr[i][j],
					global_scaler_regs.dst_base_ch[j]);
			else if (i == 1)
				writel_relaxed(data->dst_phy_addr[i][j],
					       global_scaler_regs
						       .dst_layer1_base_ch[j]);
			else
				writel_relaxed(data->dst_phy_addr[i][j],
					       global_scaler_regs
						       .dst_layer2_base_ch[j]);
		}
	}
	writel_relaxed(data->coeff_phy_addr, global_scaler_regs.coeff_addr);
	writel_relaxed(data->coeff_size, global_scaler_regs.coeff_size);
}

static void _scaler_dump_register(struct device *dev)
{
	int i;
	uint32_t base_addr;
	virt_addr_t *p_reg_virt;
	static const char *const reg_name[] = { "ENABLE",
						"SYS_CTRL",
						"MEMCTRL_INTR_STATUS",
						"ALGO_INTR_STATUS",
						"SRC_Y_ADDR",
						"SRC_U_ADDR",
						"SRC_V_ADDR",
						"SRC_RESOLUTION",
						"DST_Y_ADDR",
						"DST_U_ADDR",
						"DST_V_ADDR",
						"DST_RESOLUTION",
						"X_RATIO",
						"X_INIT_PHASE",
						"Y_RATIO",
						"Y_INIT_PHASE",
						"AXI_PARAM",
						"AXI_STRIDE",
						"COEFF_ADDR",
						"COEFF_SIZE",
						"DMA_PARA",
						"BANK_REMAK_CH1",
						"BANK_REMAK_CH2",
						"RESERVED",
						"DST_LAYER1_Y_ADDR",
						"DST_LAYER1_U_ADDR",
						"DST_LAYER1_V_ADDR",
						"DST_LAYER2_Y_ADDR",
						"DST_LAYER2_U_ADDR",
						"DST_LAYER2_V_ADDR" };

	base_addr = BST_HWCV_SCALER_BASE_PHY;
	p_reg_virt = (virt_addr_t *)&global_scaler_regs;

	dev_dbg(dev, "--- dump scaler registers ---");
	for (i = 0; i < ARRAY_SIZE(reg_name); i++) {
		dev_dbg(dev, "%-20s | 0x%08x | 0x%08x", reg_name[i],
			base_addr + i * BST_HWCV_REG_BYTE,
			readl_relaxed(p_reg_virt[i]));
	}
}

int bst_scaler_start(struct device *dev, struct hwcv_scaler_data *data)
{
	int ret;

	ret = _bst_scaler_dump_input(dev, data);
	if (ret < 0)
		return ret;

	_scaler_config_sys_ctrl(data);
	_scaler_config_resolution(data);
	_scaler_config_ratio(data);
	_scaler_config_phase(data);
	_scaler_config_stride(data);
	_scaler_config_addr(data);
	_scaler_dump_register(dev);
	bst_scaler_enable(dev);

	return 0;
}
