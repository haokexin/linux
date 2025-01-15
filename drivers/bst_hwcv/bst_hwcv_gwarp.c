// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/io.h>
#include "bst_hwcv_gwarp.h"

#define BST_HWCV_GWARP_BASE_PHY 0x51021000
#define BST_HWCV_GWARP_SNR_BASE_PHY (BST_HWCV_GWARP_BASE_PHY + 0x100)
#define BST_HWCV_GWARP_SNR_INTERVAL 0x100
#define BST_HWCV_GWARP_ENGINE_INTERVAL 0x1000

/* 0x51021004 */
#define BITS_SYS_CTRL_SRC_FORMAT 0
#define LENGTH_SYS_CTRL_SRC_FORMAT 4
#define BITS_SYS_CTRL_DST_FORMAT 8
#define LENGTH_SYS_CTRL_DST_FORMAT 4
#define BIT_SYS_CTRL_IS_BILNEAR 16
#define BITS_SYS_CTRL_GWC_MODE 24
#define LENGTH_SYS_CTRL_GWC_MODE 2

/* 0x51021008 */
#define BITS_SRC_RES_WIDTH 0
#define LENGTH_SRC_RES_WIDTH 13
#define BITS_SRC_RES_HEIGHT 16
#define LENGTH_SRC_RES_HEIGHT 13

/* 0x5102100c */
#define BITS_DST_RES_WIDTH 0
#define LENGTH_DST_RES_WIDTH 13
#define BITS_DST_RES_HEIGHT 16
#define LENGTH_DST_RES_HEIGHT 13

/* 0x5102103c */
#define BIT_GWC_INTR 0
#define BIT_SBS_GWC_INTR 8

/* 0x51021040 */
#define BIT_GWC_INTR_EN 0
#define BIT_SBS_GWC_INTR_EN 8

static struct gwarp_regs global_gwarp_regs[BST_HWCV_GWARP_ENGINE_NUM];
static struct sbs_gwarp_regs global_sbs_gwarp_regs[BST_HWCV_GWARP_ENGINE_NUM]
						  [BST_HWCV_GWARP_SNR_NUM];

int _bst_gwarp_map_normal_regs(struct device *dev, uint8_t engine_id)
{
	uint32_t base_reg_addr;
	uint32_t reg_cnt;

	base_reg_addr = BST_HWCV_GWARP_BASE_PHY +
			BST_HWCV_GWARP_ENGINE_INTERVAL * engine_id;
	reg_cnt = sizeof(struct gwarp_regs) / sizeof(virt_addr_t);

	return bst_hwcv_map_regs(dev, base_reg_addr, reg_cnt,
				 (virt_addr_t *)&global_gwarp_regs[engine_id]);
}

int _bst_gwarp_map_snr_regs(struct device *dev, uint8_t engine_id,
			    uint8_t sensor_id)
{
	uint32_t base_reg_addr;
	uint32_t reg_cnt;

	base_reg_addr = BST_HWCV_GWARP_SNR_BASE_PHY +
			BST_HWCV_GWARP_ENGINE_INTERVAL * engine_id +
			BST_HWCV_GWARP_SNR_INTERVAL * sensor_id;
	reg_cnt = sizeof(struct sbs_gwarp_regs) / sizeof(virt_addr_t);

	return bst_hwcv_map_regs(
		dev, base_reg_addr, reg_cnt,
		(virt_addr_t *)&global_sbs_gwarp_regs[engine_id][sensor_id]);
}

int bst_gwarp_map_all_regs(struct device *dev)
{
	int i, j;
	int ret = 0;

	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		ret = _bst_gwarp_map_normal_regs(dev, i);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < BST_HWCV_GWARP_ENGINE_NUM; i++) {
		for (j = 0; j < BST_HWCV_GWARP_SNR_NUM; j++) {
			ret = _bst_gwarp_map_snr_regs(dev, i, j);
			if (ret < 0)
				return ret;
		}
	}

	return ret;
}

uint32_t bst_gwarp_read_intr(struct device *dev, uint8_t engine_id)
{
	uint32_t reg;

	reg = readl_relaxed(global_gwarp_regs[engine_id].intr);
	dev_dbg(dev, "Gwarp[%d] Read Interrupt: 0x%08x", engine_id, reg);

	return reg;
}

void bst_gwarp_clear_intr(struct device *dev, uint8_t engine_id)
{
	uint32_t reg;

	dev_dbg(dev, "Gwarp[%d] Clear Interrupt", engine_id);
	reg = readl_relaxed(global_gwarp_regs[engine_id].intr);
	SET_BIT(reg, 1, BIT_GWC_INTR);
	writel_relaxed(reg, global_gwarp_regs[engine_id].intr);
}

void bst_gwarp_enable(struct device *dev, uint8_t engine_id)
{
	dev_dbg(dev, "Gwarp[%d] Enable", engine_id);
	writel_relaxed(1, global_gwarp_regs[engine_id].enable);
}

void bst_gwarp_disable(struct device *dev, uint8_t engine_id)
{
	dev_dbg(dev, "Gwarp[%d] Disable", engine_id);
	writel_relaxed(0, global_gwarp_regs[engine_id].enable);
}

void bst_sbs_gwarp_clear_intr(struct device *dev, uint8_t engine_id,
			      uint8_t sensor_id)
{
	uint32_t reg;

	dev_dbg(dev, "Gwarp[%d][%d] Clear Interrupt", engine_id, sensor_id);
	reg = readl_relaxed(global_gwarp_regs[engine_id].intr);
	SET_BIT(reg, 1, BIT_SBS_GWC_INTR + sensor_id);
	writel_relaxed(reg, global_gwarp_regs[engine_id].intr);
}

void bst_sbs_gwarp_enable(struct device *dev, uint8_t engine_id,
			  uint8_t sensor_id)
{
	dev_dbg(dev, "Gwarp[%d][%d] Enable", engine_id, sensor_id);
	writel_relaxed(1, global_sbs_gwarp_regs[engine_id][sensor_id].enable);
}

void bst_sbs_gwarp_disable(struct device *dev, uint8_t engine_id,
			   uint8_t sensor_id)
{
	dev_dbg(dev, "Gwarp[%d][%d] Disable", engine_id, sensor_id);
	writel_relaxed(0, global_sbs_gwarp_regs[engine_id][sensor_id].enable);
}

/*---------------------------------------------------------------------------------------------*/

int _bst_gwarp_dump_check_input(struct device *dev,
				const struct hwcv_gwarp_data *data)
{
	int i;
	int in_planar_num = bst_hwcv_get_planar_num_by_format(data->src_format);
	int out_planar_num =
		bst_hwcv_get_planar_num_by_format(data->dst_format);

	dev_dbg(dev, "--- Gwarp[%d]:Dump Input Params ---", data->engine_id);
	dev_dbg(dev, "Gwarp[%d]:interp: %d", data->engine_id,
		data->interpolation);
	dev_dbg(dev, "Gwarp[%d]:format: src=%d, dst=%d", data->engine_id,
		data->src_format, data->dst_format);
	dev_dbg(dev, "Gwarp[%d]:resolution: src=%dx%d, dst=%dx%d",
		data->engine_id, data->src_width, data->src_height,
		data->dst_width, data->dst_height);
	dev_dbg(dev, "Gwarp[%d]:stride: src=%d, dst=%d, lut =%d",
		data->engine_id, data->src_stride, data->dst_stride,
		data->lut_stride);
	for (i = 0; i < MAX_HWCV_PLANAR_NUM; i++)
		dev_dbg(dev, "Gwarp[%d]:src_phy_addr[%d]: 0x%08x",
			data->engine_id, i, data->src_phy_addr[i]);
	for (i = 0; i < MAX_HWCV_PLANAR_NUM; i++)
		dev_dbg(dev, "Gwarp[%d]:dst_phy_addr[%d]: 0x%08x",
			data->engine_id, i, data->dst_phy_addr[i]);
	dev_dbg(dev, "Gwarp[%d]:lut_phy_addr: 0x%08x", data->engine_id,
		data->lut_phy_addr);

	if (data->interpolation > 1) {
		dev_err(dev, "Gwarp[%d]:Invalid interpolation: %d",
			data->engine_id, data->interpolation);
		return -EINVAL;
	}

	if (data->src_format > 7 || data->dst_format > 7) {
		dev_err(dev, "Gwarp[%d]:Invalid format: %d %d", data->engine_id,
			data->src_format, data->dst_format);
		return -EINVAL;
	}

	if (!data->src_width || !data->src_height || !data->dst_width ||
	    !data->dst_height) {
		dev_err(dev,
			"Gwarp[%d]:Invalid resolution: src=%dx%d, dst=%dx%d",
			data->engine_id, data->src_width, data->src_height,
			data->dst_width, data->dst_height);
		return -EINVAL;
	}

	if (!data->src_stride || data->src_stride % 16 != 0 ||
	    !data->dst_stride || data->dst_stride % 16 != 0) {
		dev_err(dev, "Gwarp[%d]:Invalid stride: %d, %d",
			data->engine_id, data->src_stride, data->dst_stride);
		return -EINVAL;
	}

	for (i = 0; i < in_planar_num; i++) {
		if (!data->src_phy_addr[i]) {
			dev_err(dev, "Gwarp[%d]:Invalid src paddr[%d]: 0x%08x",
				data->engine_id, i, data->src_phy_addr[i]);
			return -EINVAL;
		}
	}

	for (i = 0; i < out_planar_num; i++) {
		if (!data->dst_phy_addr[i] || data->dst_phy_addr[i] % 16 != 0) {
			dev_err(dev, "Gwarp[%d]:Invalid dst paddr[%d]: 0x%08x",
				data->engine_id, i, data->dst_phy_addr[i]);
			return -EINVAL;
		}
	}

	if (!data->lut_phy_addr || !data->lut_stride) {
		dev_err(dev, "Gwarp[%d]:Invalid lut param: 0x%08x, %d",
			data->engine_id, data->lut_phy_addr, data->lut_stride);
		return -EINVAL;
	}

	return 0;
}

static void _gwarp_config_sys_ctrl(const struct hwcv_gwarp_data *data)
{
	uint32_t sys_ctrl;

	sys_ctrl = readl_relaxed(global_gwarp_regs[data->engine_id].sys_ctl);
	SET_BITS(sys_ctrl, data->src_format, BITS_SYS_CTRL_SRC_FORMAT,
		 LENGTH_SYS_CTRL_SRC_FORMAT);
	SET_BITS(sys_ctrl, data->dst_format, BITS_SYS_CTRL_DST_FORMAT,
		 LENGTH_SYS_CTRL_DST_FORMAT);
	SET_BIT(sys_ctrl, data->interpolation, BIT_SYS_CTRL_IS_BILNEAR);
	SET_BITS(sys_ctrl, 0x0, BITS_SYS_CTRL_GWC_MODE, 2);
	writel_relaxed(sys_ctrl, global_gwarp_regs[data->engine_id].sys_ctl);
}

static void _gwarp_config_resolution(const struct hwcv_gwarp_data *data)
{
	uint32_t src_res = 0;
	uint32_t dst_res = 0;

	SET_BITS(src_res, data->src_width, BITS_SRC_RES_WIDTH,
		 LENGTH_SRC_RES_WIDTH);
	SET_BITS(src_res, data->src_height, BITS_SRC_RES_HEIGHT,
		 LENGTH_SRC_RES_HEIGHT);
	SET_BITS(dst_res, data->dst_width, BITS_DST_RES_WIDTH,
		 LENGTH_DST_RES_WIDTH);
	SET_BITS(dst_res, data->dst_height, BITS_DST_RES_HEIGHT,
		 LENGTH_DST_RES_HEIGHT);
	writel_relaxed(src_res,
		       global_gwarp_regs[data->engine_id].src_resolution);
	writel_relaxed(dst_res,
		       global_gwarp_regs[data->engine_id].dst_resolution);
}

static void _gwarp_config_stride(const struct hwcv_gwarp_data *data)
{
	writel_relaxed(data->src_stride,
		       global_gwarp_regs[data->engine_id].src_stride);
	writel_relaxed(data->dst_stride,
		       global_gwarp_regs[data->engine_id].dst_stride);
	writel_relaxed(data->lut_stride,
		       global_gwarp_regs[data->engine_id].lut_stride);
}

static void _gwarp_enable_intr(const struct hwcv_gwarp_data *data)
{
	uint32_t reg;

	reg = readl_relaxed(global_gwarp_regs[data->engine_id].intr_en);
	SET_BIT(reg, 1, BIT_GWC_INTR_EN);
	writel_relaxed(reg, global_gwarp_regs[data->engine_id].intr_en);
}

static void _gwarp_config_addr(const struct hwcv_gwarp_data *data)
{
	int i;
	int in_planar_num = bst_hwcv_get_planar_num_by_format(data->src_format);
	int out_planar_num =
		bst_hwcv_get_planar_num_by_format(data->dst_format);

	for (i = 0; i < in_planar_num; i++) {
		writel_relaxed(
			data->src_phy_addr[i],
			global_gwarp_regs[data->engine_id].src_base_ch[i]);
	}
	for (i = 0; i < out_planar_num; i++) {
		writel_relaxed(
			data->dst_phy_addr[i],
			global_gwarp_regs[data->engine_id].dst_base_ch[i]);
	}
	writel_relaxed(data->lut_phy_addr,
		       global_gwarp_regs[data->engine_id].lut_base);
}

static void _gwarp_dump_register(struct device *dev,
				 const struct hwcv_gwarp_data *data)
{
	int i;
	uint32_t base_addr;
	virt_addr_t *p_reg_virt;
	static const char *const reg_name[] = {
		"ENABLE",     "SYS_CTRL",   "SRC_RESOLUTION", "DST_RESOLUTION",
		"SRC_Y_ADDR", "SRC_U_ADDR", "SRC_V_ADDR",     "DST_Y_ADDR",
		"DST_U_ADDR", "DST_V_ADDR", "LUT_ADDR",	      "SRC_STRIDE",
		"DST_STRIDE", "LUT_STRIDE", "AXI_PARAM",      "INTR",
		"INTR_EN"
	};

	base_addr = BST_HWCV_GWARP_BASE_PHY +
		    BST_HWCV_GWARP_ENGINE_INTERVAL * data->engine_id;
	p_reg_virt = (virt_addr_t *)&global_gwarp_regs[data->engine_id];

	dev_dbg(dev, "--- Gwarp[%d]:Dump Gwarp Regs ---", data->engine_id);
	for (i = 0; i < ARRAY_SIZE(reg_name); i++) {
		dev_dbg(dev, "Gwarp[%d]:%-20s | 0x%08x | 0x%08x",
			data->engine_id, reg_name[i],
			base_addr + i * BST_HWCV_REG_BYTE,
			readl_relaxed(p_reg_virt[i]));
	}
}

int bst_gwarp_start(struct device *dev, const struct hwcv_gwarp_data *data)
{
	int ret;

	ret = _bst_gwarp_dump_check_input(dev, data);
	if (ret < 0)
		return ret;

	_gwarp_config_sys_ctrl(data);
	_gwarp_config_resolution(data);
	_gwarp_config_stride(data);
	_gwarp_enable_intr(data);
	_gwarp_config_addr(data);
	_gwarp_dump_register(dev, data);
	bst_gwarp_enable(dev, data->engine_id);

	return ret;
}

/*---------------------------------------------------------------------------------------------*/

int _bst_sbs_gwarp_dump_input(struct device *dev,
			      const struct hwcv_sbs_gwarp_data *data)
{
	int i;
	int in_planar_num = bst_hwcv_get_planar_num_by_format(data->src_format);
	int out_planar_num =
		bst_hwcv_get_planar_num_by_format(data->dst_format);
	int eid = data->engine_id;
	int sid = data->sensor_id;

	dev_dbg(dev, "--- Gwarp[%d][%d]:Dump Input Params ---", eid, sid);
	dev_dbg(dev, "Gwarp[%d][%d]:interp: %d", eid, sid, data->interpolation);
	dev_dbg(dev, "Gwarp[%d][%d]:format: src=%d, dst=%d", eid, sid,
		data->src_format, data->dst_format);
	dev_dbg(dev, "Gwarp[%d][%d]:resolution: src=%dx%d, dst=%dx%d", eid, sid,
		data->src_width, data->src_height, data->dst_width,
		data->dst_height);
	dev_dbg(dev, "Gwarp[%d][%d]:stride: src=%d, dst=%d, lut =%d", eid, sid,
		data->src_stride, data->dst_stride, data->lut_stride);
	for (i = 0; i < MAX_HWCV_PLANAR_NUM; i++)
		dev_dbg(dev, "Gwarp[%d][%d]:src_phy_addr[%d]: 0x%08x", eid, sid,
			i, data->src_phy_addr[i]);
	for (i = 0; i < MAX_HWCV_PLANAR_NUM; i++)
		dev_dbg(dev, "Gwarp[%d][%d]:dst_phy_addr[%d]: 0x%08x", eid, sid,
			i, data->dst_phy_addr[i]);
	dev_dbg(dev, "Gwarp[%d][%d]:lut_phy_addr: 0x%08x", eid, sid,
		data->lut_phy_addr);

	if (data->interpolation > 1) {
		dev_err(dev, "Gwarp[%d][%d]:Invalid interpolation: %d", eid,
			sid, data->interpolation);
		return -EINVAL;
	}

	if (data->src_format > 7 || data->dst_format > 7) {
		dev_err(dev, "Gwarp[%d][%d]:Invalid format: %d %d", eid, sid,
			data->src_format, data->dst_format);
		return -EINVAL;
	}

	if (!data->src_width || !data->src_height || !data->dst_width ||
	    !data->dst_height) {
		dev_err(dev,
			"Gwarp[%d][%d]:Invalid resolution: src=%dx%d, dst=%dx%d",
			eid, sid, data->src_width, data->src_height,
			data->dst_width, data->dst_height);
		return -EINVAL;
	}

	if (!data->src_stride || data->src_stride % 16 != 0 ||
	    !data->dst_stride || data->dst_stride % 16 != 0) {
		dev_err(dev, "Gwarp[%d][%d]:Invalid stride: %d, %d", eid, sid,
			data->src_stride, data->dst_stride);
		return -EINVAL;
	}

	for (i = 0; i < in_planar_num; i++) {
		if (!data->src_phy_addr[i]) {
			dev_err(dev,
				"Gwarp[%d][%d]:Invalid src paddr[%d]: 0x%08x",
				eid, sid, i, data->src_phy_addr[i]);
			return -EINVAL;
		}
	}

	for (i = 0; i < out_planar_num; i++) {
		if (!data->dst_phy_addr[i] || data->dst_phy_addr[i] % 16 != 0) {
			dev_err(dev,
				"Gwarp[%d][%d]:Invalid dst paddr[%d]: 0x%08x",
				eid, sid, i, data->dst_phy_addr[i]);
			return -EINVAL;
		}
	}

	if (!data->lut_phy_addr || !data->lut_stride) {
		dev_err(dev, "Gwarp[%d][%d]:Invalid lut param: 0x%08x, %d", eid,
			sid, data->lut_phy_addr, data->lut_stride);
		return -EINVAL;
	}

	return 0;
}

static void _sbs_gwarp_config_sys_ctrl(const struct hwcv_sbs_gwarp_data *data)
{
	uint32_t sys_ctrl;

	sys_ctrl = readl_relaxed(global_gwarp_regs[data->engine_id].sys_ctl);
	if (((sys_ctrl >> BITS_SYS_CTRL_GWC_MODE) & 0x3) != 0x1) {
		SET_BITS(sys_ctrl, 0x1, BITS_SYS_CTRL_GWC_MODE, 2);
		writel_relaxed(sys_ctrl,
			       global_gwarp_regs[data->engine_id].sys_ctl);
	}

	sys_ctrl = readl_relaxed(
		global_sbs_gwarp_regs[data->engine_id][data->sensor_id].sys_ctl);
	SET_BITS(sys_ctrl, data->src_format, BITS_SYS_CTRL_SRC_FORMAT,
		 LENGTH_SYS_CTRL_SRC_FORMAT);
	SET_BITS(sys_ctrl, data->dst_format, BITS_SYS_CTRL_DST_FORMAT,
		 LENGTH_SYS_CTRL_DST_FORMAT);
	SET_BIT(sys_ctrl, data->interpolation, BIT_SYS_CTRL_IS_BILNEAR);
	writel_relaxed(
		sys_ctrl,
		global_sbs_gwarp_regs[data->engine_id][data->sensor_id].sys_ctl);
}

static void _sbs_gwarp_config_resolution(const struct hwcv_sbs_gwarp_data *data)
{
	uint32_t src_res = 0;
	uint32_t dst_res = 0;

	SET_BITS(src_res, data->src_width, BITS_SRC_RES_WIDTH,
		 LENGTH_SRC_RES_WIDTH);
	SET_BITS(src_res, data->src_height, BITS_SRC_RES_HEIGHT,
		 LENGTH_SRC_RES_HEIGHT);
	SET_BITS(dst_res, data->dst_width, BITS_DST_RES_WIDTH,
		 LENGTH_DST_RES_WIDTH);
	SET_BITS(dst_res, data->dst_height, BITS_DST_RES_HEIGHT,
		 LENGTH_DST_RES_HEIGHT);
	writel_relaxed(src_res,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .src_resolution);
	writel_relaxed(dst_res,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .dst_resolution);
}

static void _sbs_gwarp_config_stride(const struct hwcv_sbs_gwarp_data *data)
{
	writel_relaxed(data->src_stride,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .src_stride);
	writel_relaxed(data->dst_stride,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .dst_stride);
	writel_relaxed(data->lut_stride,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .lut_stride);
}

static void _sbs_gwarp_enable_intr(const struct hwcv_sbs_gwarp_data *data)
{
	uint32_t reg;

	reg = readl_relaxed(global_gwarp_regs[data->engine_id].intr_en);
	SET_BIT(reg, 1, BIT_SBS_GWC_INTR_EN + data->sensor_id);
	writel_relaxed(reg, global_gwarp_regs[data->engine_id].intr_en);
}

static void _sbs_gwarp_config_addr(const struct hwcv_sbs_gwarp_data *data)
{
	int i;
	int in_planar_num = bst_hwcv_get_planar_num_by_format(data->src_format);
	int out_planar_num =
		bst_hwcv_get_planar_num_by_format(data->dst_format);

	for (i = 0; i < in_planar_num; i++) {
		writel_relaxed(
			data->src_phy_addr[i],
			global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
				.src_base_ch[i]);
	}
	for (i = 0; i < out_planar_num; i++) {
		writel_relaxed(
			data->dst_phy_addr[i],
			global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
				.dst_base_ch[i]);
	}
	writel_relaxed(data->lut_phy_addr,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .lut_base);
}

static void
_sbs_gwarp_config_update_sync(const struct hwcv_sbs_gwarp_data *data)
{
	writel_relaxed(0x00010300,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .update_sync);
	writel_relaxed(0x00000300,
		       global_sbs_gwarp_regs[data->engine_id][data->sensor_id]
			       .update_sync);
}

static void _sbs_gwarp_dump_register(struct device *dev,
				     const struct hwcv_sbs_gwarp_data *data)
{
	int i;
	uint32_t base_addr;
	virt_addr_t *p_reg_virt;
	static const char *const reg_name[] = {
		"ENABLE",	  "SYS_CTRL",	"SRC_RESOLUTION",
		"DST_RESOLUTION", "SRC_Y_ADDR", "SRC_U_ADDR",
		"SRC_V_ADDR",	  "DST_Y_ADDR", "DST_U_ADDR",
		"DST_V_ADDR",	  "LUT_ADDR",	"SRC_STRIDE",
		"DST_STRIDE",	  "LUT_STRIDE", "LUT_DISTRIBUTION",
		"UPDATE_SYNC"
	};

	base_addr = BST_HWCV_GWARP_SNR_BASE_PHY +
		    BST_HWCV_GWARP_SNR_INTERVAL * data->sensor_id +
		    BST_HWCV_GWARP_ENGINE_INTERVAL * data->engine_id;
	p_reg_virt = (virt_addr_t *)&global_sbs_gwarp_regs[data->engine_id]
							  [data->sensor_id];

	dev_dbg(dev, "--- Gwarp[%d][%d]:Dump  Regs ---", data->engine_id,
		data->sensor_id);
	for (i = 0; i < ARRAY_SIZE(reg_name); i++) {
		dev_dbg(dev, "Gwarp[%d][%d]:%-20s | 0x%08x | 0x%08x",
			data->engine_id, data->sensor_id, reg_name[i],
			base_addr + i * BST_HWCV_REG_BYTE,
			readl_relaxed(p_reg_virt[i]));
	}
}

int bst_sbs_gwarp_start(struct device *dev,
			const struct hwcv_sbs_gwarp_data *data)
{
	int ret;

	ret = _bst_sbs_gwarp_dump_input(dev, data);
	if (ret < 0)
		return ret;

	_sbs_gwarp_config_sys_ctrl(data);
	_sbs_gwarp_config_resolution(data);
	_sbs_gwarp_config_stride(data);
	_sbs_gwarp_enable_intr(data);
	_sbs_gwarp_config_addr(data);
	_sbs_gwarp_dump_register(dev, data);
	bst_sbs_gwarp_enable(dev, data->engine_id, data->sensor_id);
	_sbs_gwarp_config_update_sync(data);

	return ret;
}
