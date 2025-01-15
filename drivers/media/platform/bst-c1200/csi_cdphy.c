// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

#include "cam_entity.h"
#include "csi_cdphy.h"
#include "isp_core.h"

#define u32		     unsigned int
#define MAX_ITERATION_DESKEW 150
#define WINDOW_SIZE_DESKEW   3
#define FILTER_SIZE_DESKEW   16
#define DESKEW_OPT	     0
#define VT_TRACKING_EN	     0
//#define CSI_REG_READ_WRITE_DEBUG_LOG_EN
static const u32 deskew_regs[] = {
	// clang-format off
	0x0404, 0x040C, 0x0414, 0x041C, 0x0423, 0x0429, 0x0430, 0x043A,
	0x0445, 0x044A, 0x0450, 0x045A, 0x0465, 0x0469, 0x0472, 0x047A,
	0x0485, 0x0489, 0x0490, 0x049A, 0x04A4, 0x04AC, 0x04B4, 0x04BC,
	0x04C4, 0x04CC, 0x04D4, 0x04DC, 0x04E4, 0x04EC, 0x04F4, 0x04FC,
	0x0504, 0x050C, 0x0514, 0x051C, 0x0523, 0x0529, 0x0530, 0x053A,
	0x0545, 0x054A, 0x0550, 0x055A, 0x0565, 0x0569, 0x0572, 0x057A,
	0x0585, 0x0589, 0x0590, 0x059A, 0x05A4, 0x05AC, 0x05B4, 0x05BC,
	0x05C4, 0x05CC, 0x05D4, 0x05DC, 0x05E4, 0x05EC, 0x05F4, 0x05FC,
	0x0604, 0x060C, 0x0614, 0x061C, 0x0623, 0x0629, 0x0632, 0x063A,
	0x0645, 0x064A, 0x0650, 0x065A, 0x0665, 0x0669, 0x0672, 0x067A,
	0x0685, 0x0689, 0x0690, 0x069A, 0x06A4, 0x06AC, 0x06B4, 0x06BC,
	0x06C4, 0x06CC, 0x06D4, 0x06DC, 0x06E4, 0x06EC, 0x06F4, 0x06FC,
	0x0704, 0x070C, 0x0714, 0x071C, 0x0723, 0x072A, 0x0730, 0x073A,
	0x0745, 0x074A, 0x0750, 0x075A, 0x0765, 0x0769, 0x0772, 0x077A,
	0x0785, 0x0789, 0x0790, 0x079A, 0x07A4, 0x07AC, 0x07B4, 0x07BC,
	0x07C4, 0x07CC, 0x07D4, 0x07DC, 0x07E4, 0x07EC, 0x07F4, 0x07FC
	// clang-format on
};

static inline void csi_write_top_reg(struct bst_csi_device *csi_dev, u32 reg,
				     u32 val)
{
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	u32 val_tmp = 0;

	val_tmp = ioread32(csi_dev->top_base + reg);
	dev_info(
		csi_dev->dev,
		"MIPI CSI top write: reg-%08lx offset-%08x read value-%08x write value-%08x\n",
		(unsigned long)csi_dev->top_base + reg, reg, val_tmp, val);
#endif
	iowrite32(val, csi_dev->top_base + reg);
}

static inline u32 csi_read_top_reg(struct bst_csi_device *csi_dev, u32 reg)
{
	u32 val_tmp = 0;

	val_tmp = ioread32(csi_dev->top_base + reg);
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	dev_info(csi_dev->dev,
		 "MIPI CSI top read: reg-%08lx offset-%08x value-%08x\n",
		 (unsigned long)(csi_dev->top_base + reg), reg, val_tmp);
#endif

	return val_tmp;
}

static inline void csi_cdphy_iowrite32(struct bst_csi_device *csi_dev, u32 reg,
				       u32 val)
{
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	u32 val_tmp = 0;

	val_tmp = ioread32(csi_dev->ctrl_base + (reg << 2));
	dev_info(
		csi_dev->dev,
		"MIPI CSI dphy write: reg-%08lx offset-%08x read value-%08x write value-%08x\n",
		(unsigned long)csi_dev->ctrl_base + (reg << 2), reg, val_tmp, val);
#endif
	iowrite32(val, csi_dev->ctrl_base + (reg << 2));
}

static inline u32 csi_cdphy_ioread32(struct bst_csi_device *csi_dev, u32 reg)
{
	u32 val_tmp = 0;

	val_tmp = ioread32(csi_dev->ctrl_base + (reg << 2));
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	dev_info(csi_dev->dev,
		 "MIPI CSI dphy read: reg-%08lx offset-%08x value-%08x\n",
		 (unsigned long)(csi_dev->ctrl_base + (reg << 2)), reg, val_tmp);
#endif
	return val_tmp;
}

static inline void csi_write_ctrl_reg(struct bst_csi_device *csi_dev, u32 reg,
				      u32 val)
{
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	u32 val_tmp = 0;

	val_tmp = ioread32(csi_dev->ctrl_base + reg);
	dev_info(
		csi_dev->dev,
		"MIPI CSI ctrl write: reg-%08lx offset-%08x read value-%08x write value-%08x\n",
		(unsigned long)csi_dev->ctrl_base + reg, reg, val_tmp, val);
#endif
	iowrite32(val, csi_dev->ctrl_base + reg);
}

static inline u32 csi_read_ctrl_reg(struct bst_csi_device *csi_dev, u32 reg)
{
	u32 read_val = 0;

	read_val = ioread32(csi_dev->ctrl_base + reg);
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	dev_info(csi_dev->dev,
		 "MIPI CSI ctrl read: reg-%08lx offset-%08x value-%08x\n",
		 (unsigned long)(csi_dev->ctrl_base + reg), reg, read_val);
#endif
	return read_val;
}

void csi_write_cdphy_reg(struct bst_csi_device *csi_dev, u32 reg, u32 bit_off,
			 u32 mask, u32 value)
{
	u32 val;

	val = csi_cdphy_ioread32(csi_dev, reg);
	val = (val & (~(mask << bit_off))) | (value << bit_off);
	csi_cdphy_iowrite32(csi_dev, reg, val);
}

void csi_cdphy_config_common(struct bst_csi_device *csi_dev)
{
	u32 vt_tracking_en = VT_TRACKING_EN;

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_10, 0,
			    0xff, 48);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0, 2,
			    0x3f, 63);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_STARTUP_1_1,
			    0, 0xfff, 563);
	// PoR FSM RCAL state control
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_2, 0,
			    0xff, 0x5);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_6, 0,
			    0xff, 39);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_CALIBCTRL_RW_COMMON_BG_0, 0,
			    0x1ff, 500);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_TERMCAL_CFG_0, 0, 0x7f,
			    csi_dev->mipi_cfg.reg_val);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_OFFSETCAL_CFG_0, 0, 0x1f,
			    csi_dev->mipi_cfg.reg_val_offsetcal_wait_thresh);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_TIMEBASE, 0, 0x3ff,
			    csi_dev->mipi_cfg.reg_val_lpdcocal_timebase);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_NREF, 0, 0x7ff,
			    800);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_NREF_RANGE, 0,
			    0x1f, 0x1B);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_TWAIT_CONFIG, 9,
			    0x7f, 127);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_TWAIT_CONFIG, 0,
			    0x1ff, csi_dev->mipi_cfg.reg_val_twait_coarse_fine);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_VT_CONFIG, 7,
			    0x1ff, csi_dev->mipi_cfg.reg_val_twait_coarse_fine);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_VT_CONFIG, 2, 0x1f,
			    27);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_VT_CONFIG, 1, 0x1,
			    1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_VT_CONFIG, 0, 0x1,
			    vt_tracking_en);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_LPDCOCAL_COARSE_CFG, 0, 0x3,
			    1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_COMMON_CFG, 0, 0x3, 3);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0,
			    10, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1,
			    10, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1,
			    15, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_3,
			    8, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0,
			    15, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_3,
			    9, 0x1, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    13, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			    9, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    12, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			    8, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    14, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			    10, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_5,
			    8, 0x1, 0);
}

static int csi_int_round_up(int num, int div)
{
	int ret_val = 0;

	ret_val = num / div;
	if (ret_val * div < num)
		ret_val += 1;
	return ret_val;
}

static int csi_int_round_off(int num, int div)
{
	int dividend;
	int remainder;

	dividend = num / div;
	remainder = num % div;

	if (remainder >= (div + 1) / 2)
		dividend += 1;
	return dividend;
}

static int csi_int_round_down(int num, int div)
{
	int dividend;

	dividend = num / div;

	return dividend;
}

static int csi_long_round_up(long num, int div)
{
	int ret_val = 0;

	ret_val = num / div;
	if (ret_val * div < num)
		ret_val += 1;
	return ret_val;
}

void csi_calcul_cfg(struct bst_csi_device *csi_dev)
{
	int timebase, itminrx;
	int des_div_en_deass_th;
	long t_hs_settle_ns;
	long t_hs_settle_ui;
	int ths_settle_target;
	long hs_clk_freq;
	int i = 0;
	int reg_val_det_dly;
	int reg_val_post_rcvd_rst_val;
	long t_hs_temp_ns = 0;
	uint32_t hsdcocal_div;
	uint32_t csi_calcu_dividend;
	uint32_t csi_calcu_divisor;
	uint64_t csi_calcu_dividend_l;
	int speed_array[14][6] = {
		{ 4500, 4500, 1, 63, 7, 0 },   { 4000, 4499, 1, 71, 7, 1 },
		{ 3600, 3999, 1, 79, 9, 1 },   { 3230, 3599, 1, 87, 9, 1 },
		{ 3000, 3229, 0, 71, 7, 1 },   { 2700, 2999, 0, 79, 9, 1 },
		{ 2455, 2699, 0, 87, 9, 1 },   { 2250, 2454, 0, 95, 11, 1 },
		{ 2077, 2249, 0, 103, 11, 2 }, { 1929, 2076, 0, 111, 13, 2 },
		{ 1800, 1928, 0, 119, 13, 2 }, { 1688, 1799, 0, 127, 15, 2 },
		{ 1588, 1687, 0, 135, 15, 3 }, { 1500, 1587, 0, 143, 17, 3 },
	};

	int phase_bound_arry[14] = { 71,  103, 103, 103, 103, 103, 103,
				     103, 135, 135, 135, 135, 167, 167 };

	int osc_array[7][10] = {
		{ 76, 23, 1, 95, 19, 9, 76, 23, 191, 96 },
		{ 79, 24, 1, 99, 19, 9, 79, 24, 199, 99 },
		{ 95, 29, 2, 119, 23, 11, 95, 29, 239, 119 },
		{ 103, 32, 2, 129, 25, 12, 103, 32, 259, 129 },
		{ 107, 33, 2, 134, 26, 13, 107, 33, 269, 134 },
		{ 153, 47, 3, 191, 38, 19, 153, 47, 383, 192 },
		{ 207, 64, 5, 259, 51, 25, 207, 64, 519, 259 },
	};

	int speed_opt[20];
	int k2 = 0;

	for (k2 = 2500; k2 < 4500; k2 = k2 + 100) {
		if (k2 == 2500 || k2 == 3000 || k2 == 3100 || k2 == 3300 ||
		    k2 == 4000 || k2 == 4100) {
			speed_opt[k2 / 100 - 25] = 1;
		} else if (k2 == 2600 || k2 == 3500 || k2 == 4300 ||
			   k2 == 4400) {
			speed_opt[k2 / 100 - 25] = -1;
		} else {
			speed_opt[k2 / 100 - 25] = 0;
		}
	}

	hs_clk_freq = (long)(csi_dev->lane_speed * 1000 / 2);
	timebase = 5000; // 5us
	itminrx = 4;
	des_div_en_deass_th = 1;

	csi_dev->mipi_cfg.reg_val_ddlcal_counter_ref =
		csi_long_round_up((timebase * hs_clk_freq), 32000000);
	csi_dev->mipi_cfg.phase_bound_reg = 71;
	for (i = 0; i < 14; i++) {
		if ((csi_dev->lane_speed >= speed_array[i][0]) &&
		    (csi_dev->lane_speed <= speed_array[i][1])) {
			if ((csi_dev->lane_speed >= 2500 &&
			     csi_dev->lane_speed < 4500) &&
			    DESKEW_OPT) {
				int k1 = csi_dev->lane_speed / 100 - 25;
				int index_opt = speed_opt[k1];

				csi_dev->mipi_cfg
					.reg_val_lanex_hsrx_cdphy_sel_fast =
					speed_array[i + index_opt][2];
				csi_dev->mipi_cfg.max_phase =
					speed_array[i + index_opt][3];
				csi_dev->mipi_cfg.reg_val_ddlcal_dll_fbk =
					speed_array[i + index_opt][4];
				csi_dev->mipi_cfg
					.reg_val_ddlcal_ddl_coarse_bank =
					speed_array[i + index_opt][5];
			} else {
				csi_dev->mipi_cfg
					.reg_val_lanex_hsrx_cdphy_sel_fast =
					speed_array[i][2];
				csi_dev->mipi_cfg.max_phase = speed_array[i][3];
				csi_dev->mipi_cfg.reg_val_ddlcal_dll_fbk =
					speed_array[i][4];
				csi_dev->mipi_cfg
					.reg_val_ddlcal_ddl_coarse_bank =
					speed_array[i][5];
				csi_dev->mipi_cfg.phase_bound_reg =
					phase_bound_arry[i];
			}
			break;
		}
	}

	if (hs_clk_freq * 2 < 160000)
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 1;
	else if (hs_clk_freq * 2 < 320000)
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 2;
	else if (hs_clk_freq * 2 < 640000)
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 3;
	else if (hs_clk_freq * 2 < 1280000)
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 4;
	else if (hs_clk_freq * 2 < 2560000)
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 5;
	else
		csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div = 6;

	t_hs_settle_ns = MIN_T_HS_SETTLE_NS + MAX_T_HS_SETTLE_NS;
	t_hs_settle_ui = MIN_T_HS_SETTLE_UI + MAX_T_HS_SETTLE_UI;

	t_hs_temp_ns = t_hs_settle_ui * 1000000;

	ths_settle_target = csi_long_round_up(
		(t_hs_settle_ns * hs_clk_freq * 2 + t_hs_temp_ns),
		hs_clk_freq * T_DCO * 4);

	csi_dev->mipi_cfg.reg_val_hs_rx_thssettle =
		ths_settle_target - itminrx - 7;
	csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew =
		csi_int_round_up(csi_dev->mipi_cfg.max_phase, 40);
	csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew =
		csi_int_round_up(csi_dev->mipi_cfg.max_phase, 5);

	if (hs_clk_freq * 2 >= 900000) {
		csi_calcu_dividend_l = timebase * hs_clk_freq * 2 * 2;
		csi_calcu_divisor = 32 * 1000000 * 5;
		csi_dev->mipi_cfg.reg_val_coarse_target_reg =
			csi_long_round_up(csi_calcu_dividend_l,
					  csi_calcu_divisor) -
			1;
	} else {
		csi_calcu_dividend_l = timebase * 900000.0 * 2;
		csi_calcu_divisor = 32 * 1000000 * 5;
		csi_dev->mipi_cfg.reg_val_coarse_target_reg =
			csi_long_round_up(csi_calcu_dividend_l,
					  csi_calcu_divisor) -
			1;
	}

#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	dev_info(csi_dev->dev, "%s %d reg_val_hs_rx_thssettle:%d\n", __func__,
		 __LINE__, csi_dev->mipi_cfg.reg_val_hs_rx_thssettle);
	dev_info(csi_dev->dev, "%s %d reg_val_hs_rx_fjump_deskew:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	dev_info(csi_dev->dev,
		 "%s %d reg_val_hs_rx_min_eye_opening_deskew:%d\n", __func__,
		 __LINE__,
		 csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew);
	dev_info(csi_dev->dev, "%s %d reg_val_coarse_target_reg:%d\n", __func__,
		 __LINE__, csi_dev->mipi_cfg.reg_val_coarse_target_reg);
#endif

	csi_calcu_dividend = 1000000 * 7 * 6;
	csi_calcu_divisor = hs_clk_freq * 2 * T_DCO;
	csi_dev->mipi_cfg.reg_val_delay_deass_thresh_reg =
		csi_int_round_up(csi_calcu_dividend, csi_calcu_divisor) +
		des_div_en_deass_th;

	if (csi_dev->lane_speed >= 320) {
		hsdcocal_div = 16;
	} else if ((csi_dev->lane_speed >= 160) &&
		   (csi_dev->lane_speed < 320)) {
		hsdcocal_div = 8;
	} else {
		hsdcocal_div = 4;
	}
	csi_calcu_dividend = csi_dev->lane_speed * (51 + 1);
	csi_calcu_divisor = hsdcocal_div * 26;
	csi_dev->mipi_cfg.reg_val_hsdcocal_nref =
		csi_int_round_off(csi_calcu_dividend, csi_calcu_divisor);
	if (csi_dev->lane_speed >= 400) {
		csi_calcu_dividend = csi_dev->lane_speed * (51 + 1);
		csi_calcu_divisor = hsdcocal_div * 26 * 20;
		csi_dev->mipi_cfg.reg_val_hsdcocal_nref_range =
			csi_int_round_off(csi_calcu_dividend,
					  csi_calcu_divisor);
	} else {
		csi_calcu_dividend = csi_dev->lane_speed * (51 + 1);
		csi_calcu_divisor = hsdcocal_div * 26 * 10;
		csi_dev->mipi_cfg.reg_val_hsdcocal_nref_range =
			csi_int_round_off(csi_calcu_dividend,
					  csi_calcu_divisor);
	}

	csi_calcu_dividend = 161.0 * 1000;
	csi_calcu_divisor = (csi_dev->lane_speed) * T_DCO;
	reg_val_det_dly =
		csi_int_round_down(csi_calcu_dividend, csi_calcu_divisor) - 7;
	csi_dev->mipi_cfg.reg_val_det_dly_thresh_val =
		(reg_val_det_dly > 1.0) ? reg_val_det_dly : 1;

	csi_calcu_dividend = 2 * (hs_clk_freq)*T_DCO * 2;
	csi_calcu_divisor = 7 * 1000000;
	reg_val_post_rcvd_rst_val =
		csi_int_round_off(csi_calcu_dividend, csi_calcu_divisor) - 1;
	csi_dev->mipi_cfg.reg_val_post_rcvd_rst_val_thresh_reg =
		(reg_val_post_rcvd_rst_val > 0) ? reg_val_post_rcvd_rst_val : 1;

	csi_dev->mipi_cfg.reg_val_lpdcocal_timebase = osc_array[3][0];
	csi_dev->mipi_cfg.reg_val_twait_coarse_fine = osc_array[3][1];

	csi_dev->mipi_cfg.reg_val = (CFG_CLK - 1) / 1;
	csi_dev->mipi_cfg.reg_val_offsetcal_wait_thresh =
		csi_int_round_up(CFG_CLK, 5) - 1;
	csi_calcu_dividend = timebase * CFG_CLK;
	csi_calcu_divisor = 1000;
	csi_dev->mipi_cfg.reg_val_ddlcal_timebase_target =
		csi_int_round_up(csi_calcu_dividend, csi_calcu_divisor) - 1;
	csi_dev->mipi_cfg.reg_val_ddlcal_start_delay =
		csi_int_round_up(CFG_CLK, 2) - 1;
#ifdef CSI_REG_READ_WRITE_DEBUG_LOG_EN
	dev_info(csi_dev->dev, "%s %d reg_val:%d\n", __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val);
	dev_info(csi_dev->dev, "%s %d reg_val_offsetcal_wait_thresh:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_offsetcal_wait_thresh);
	dev_info(csi_dev->dev, "%s %d reg_val_ddlcal_timebase_target:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_ddlcal_timebase_target);
	dev_info(csi_dev->dev, "%s %d reg_val_ddlcal_start_delay:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_ddlcal_start_delay);
	dev_info(csi_dev->dev,
		 "%s %d reg_val_post_rcvd_rst_val_thresh_reg:%d\n", __func__,
		 __LINE__,
		 csi_dev->mipi_cfg.reg_val_post_rcvd_rst_val_thresh_reg);
	dev_info(csi_dev->dev, "%s %d reg_val_det_dly_thresh_val:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_det_dly_thresh_val);
	dev_info(csi_dev->dev, "%s %d reg_val_delay_deass_thresh_reg:%d\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_delay_deass_thresh_reg);
	dev_info(csi_dev->dev, "%s %d reg_val_hsdcocal_nref:%ld\n", __func__,
		 __LINE__, csi_dev->mipi_cfg.reg_val_hsdcocal_nref);
	dev_info(csi_dev->dev, "%s %d reg_val_hsdcocal_nref_range:%ld\n",
		 __func__, __LINE__,
		 csi_dev->mipi_cfg.reg_val_hsdcocal_nref_range);
#endif
}

void csi_cphy_spec_cfg(struct bst_csi_device *csi_dev)
{
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 0, 0x3, 0x1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 2, 0x3, 0x1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 4, 0x3, 0x1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 6, 0x3, 0x1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 8, 0x3, 0x1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_7, 0,
			    0xff, 104);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_8, 0,
			    0xff, 16);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_0, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_0, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_0, 0,
			    0x1, 1);
	if (csi_dev->lane_speed > 900) {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_0, 1, 0x1,
				    1);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_0, 1, 0x1,
				    1);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_0, 1, 0x1,
				    1);
	} else {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_0, 1, 0x1,
				    0);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_0, 1, 0x1,
				    0);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_0, 1, 0x1,
				    0);
	}

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_1, 0,
			    0xffff, 25);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_1, 0,
			    0xffff, 25);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_1, 0,
			    0xffff, 25);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_5, 0,
			    0xffff, csi_dev->mipi_cfg.reg_val);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_5, 0,
			    0xffff, csi_dev->mipi_cfg.reg_val);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_5, 0,
			    0xffff, csi_dev->mipi_cfg.reg_val);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_6, 0,
			    0xffff, 10);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_6, 0,
			    0xffff, 10);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_6, 0,
			    0xffff, 10);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_2, 0,
			    0xffff,
			    csi_dev->mipi_cfg.reg_val_coarse_target_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_2, 0,
			    0xffff,
			    csi_dev->mipi_cfg.reg_val_coarse_target_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_2, 0,
			    0xffff,
			    csi_dev->mipi_cfg.reg_val_coarse_target_reg);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2, 0,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2, 0,
			    0x1, 0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_0, 6, 0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_0, 6, 0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_0, 6, 0x7, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_0, 3, 0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_0, 3, 0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_0, 3, 0x7, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_2, 0, 0xff,
			    csi_dev->mipi_cfg.reg_val_delay_deass_thresh_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_2, 0, 0xff,
			    csi_dev->mipi_cfg.reg_val_delay_deass_thresh_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_2, 0, 0xff,
			    csi_dev->mipi_cfg.reg_val_delay_deass_thresh_reg);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_0, 0, 0x7, 2);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_0, 0, 0x7, 2);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_0, 0, 0x7, 2);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_1, 0, 0xffff,
			    csi_dev->mipi_cfg.reg_val_det_dly_thresh_val);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_1, 0, 0xffff,
			    csi_dev->mipi_cfg.reg_val_det_dly_thresh_val);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_1, 0, 0xffff,
			    csi_dev->mipi_cfg.reg_val_det_dly_thresh_val);

	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_RW_TRIO0_0, 9, 0x7,
		csi_dev->mipi_cfg.reg_val_post_rcvd_rst_val_thresh_reg);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_RW_TRIO1_0, 9, 0x7,
		csi_dev->mipi_cfg.reg_val_post_rcvd_rst_val_thresh_reg);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_RW_TRIO2_0, 9, 0x7,
		csi_dev->mipi_cfg.reg_val_post_rcvd_rst_val_thresh_reg);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_LP_0, 12, 0xf,
			    1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_LP_0, 12, 0xf,
			    1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_LP_0, 12, 0xf,
			    1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_LP_2, 0, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_LP_2, 0, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_LP_2, 0, 0x1,
			    0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_HS_RX_0, 2,
			    0x1f, 12);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_HS_RX_0, 2,
			    0x1f, 12);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_HS_RX_0, 2,
			    0x1f, 12);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_5, 14,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_3_5, 14,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_3_5, 14,
			    0x1, 0);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_6, 3,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_3_6, 3,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_3_6, 3,
			    0x1, 0);
}

void csi_cphy_option_cfg(struct bst_csi_device *csi_dev)
{
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_CFG_0, 0, 0x7,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_CFG_0, 0, 0x7,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_CFG_0, 0, 0x7,
			    0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_CFG_0, 3, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_1_RW_CFG_0, 3, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_2_RW_CFG_0, 3, 0x1,
			    0);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 8,
			    0x1, 0);
}

void csi_cdphy_equalizer_cfg(struct bst_csi_device *csi_dev)
{
	u32 CPHY_EQUALIZER = 4;
	u32 CPHY_GMODE = csi_dev->lane_speed <= 2500 ? 0 : 2;
	u32 CPHY_CAP_PROG = csi_dev->lane_speed <= 920	? 0 :
			    csi_dev->lane_speed <= 1030 ? 1 :
			    csi_dev->lane_speed <= 1170 ? 2 :
			    csi_dev->lane_speed <= 1350 ? 3 :
			    csi_dev->lane_speed <= 1590 ? 4 :
			    csi_dev->lane_speed <= 1950 ? 5 :
			    csi_dev->lane_speed <= 2500 ? 6 :
							  7;

	u32 DPHY_EQUALIZER = csi_dev->lane_speed <= 1500 ? 0 : 4;
	u32 DPHY_GMODE = csi_dev->lane_speed <= 2500 ? 0 : 2;
	u32 DPHY_CP_PROG = csi_dev->lane_speed <= 1500 ? 0 : 3;

	if (csi_dev->phy_mode_cfg) {
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			0, 0x7, CPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			0, 0x7, CPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			0, 0x7, CPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
			0, 0x7, CPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
			0, 0x7, CPHY_EQUALIZER);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			3, 0x3, CPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			3, 0x3, CPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			3, 0x3, CPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
			3, 0x3, CPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
			3, 0x3, CPHY_GMODE);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_7,
			10, 0x7, CPHY_CAP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_7,
			10, 0x7, CPHY_CAP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_7,
			10, 0x7, CPHY_CAP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_7,
			10, 0x7, CPHY_CAP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_7,
			10, 0x7, CPHY_CAP_PROG);
	} else {
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			0, 0x7, DPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			0, 0x7, DPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			0, 0x7, DPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
			0, 0x7, DPHY_EQUALIZER);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
			0, 0x7, DPHY_EQUALIZER);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			3, 0x3, DPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			3, 0x3, DPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			3, 0x3, DPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
			3, 0x3, DPHY_GMODE);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
			3, 0x3, DPHY_GMODE);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15,
			0, 0x7, DPHY_CP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15,
			0, 0x7, DPHY_CP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15,
			0, 0x7, DPHY_CP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_15,
			0, 0x7, DPHY_CP_PROG);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_15,
			0, 0x7, DPHY_CP_PROG);
	}

	if (csi_dev->phy_mode_cfg == 0) {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_A, 0,
				    0xff, 48);

		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_0, 2,
				    0xfff, 61);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_0, 1, 1,
				    0);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_10,
			2, 1, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_10,
			2, 1, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_10,
			2, 1, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_10,
			2, 1, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_10,
			2, 1, 0);

		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15,
			3, 0x3, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15,
			3, 0x3, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15,
			3, 0x3, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_15,
			3, 0x3, 0);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_15,
			3, 0x3, 0);

		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_LP_2,
				    1, 1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_2,
				    1, 1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_2,
				    1, 1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_LP_2,
				    1, 1, 1);

		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_LP_2,
				    1, 1, 1);
	} else {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_A, 0,
				    0xff, 48);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_CLANE_0_RW_LP_2,
				    1, 1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_2,
				    1, 1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_2,
				    1, 1, 1);
	}
	csi_write_cdphy_reg(csi_dev, 0x1c25, 8, 0x1, 1);
}

void csi_dphy_spec_cfg(struct bst_csi_device *csi_dev)
{
	int i;
	unsigned int lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg;
	unsigned int lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg;
	unsigned int lanex_hsrx_dphy_dll_en_ovr_val_reg;
	unsigned int lanex_hsrx_dphy_dll_en_ovr_en_reg;
	unsigned int dlane_x_sot_detection_reg;
	unsigned int dlane_x_ignore_alterncal_reg;
	unsigned int dlane_x_deskew_supported_reg;
	int en_check_prbs9 = 0;

	if (csi_dev->lane_speed <= 1500) {
		lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg = 1;
		lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg = 1;
		lanex_hsrx_dphy_dll_en_ovr_val_reg = 0;
		lanex_hsrx_dphy_dll_en_ovr_en_reg = 1;
		dlane_x_deskew_supported_reg = 0;
		dlane_x_sot_detection_reg = 1;
	} else if (csi_dev->lane_speed > 1500) {
		lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg = 0;
		lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg = 0;
		lanex_hsrx_dphy_dll_en_ovr_val_reg = 0;
		lanex_hsrx_dphy_dll_en_ovr_en_reg = 0;
		dlane_x_deskew_supported_reg = 1;
		dlane_x_sot_detection_reg = 0;
	}

	if (csi_dev->lane_speed <= 2500)
		dlane_x_ignore_alterncal_reg = 1;
	else
		dlane_x_ignore_alterncal_reg = 0;

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 0, 0x3, 0x0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 2, 0x3, 0x0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 4, 0x3, 0x0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 6, 0x3, 0x0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_7, 8, 0x3, 0x0);

	if (csi_dev->lane_speed >= 1500) {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_7, 0,
				    0xff, 40);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_8, 0,
				    0xff, 80);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_0, 0, 0x3ff,
			csi_dev->mipi_cfg.reg_val_ddlcal_timebase_target);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_7, 7,
				    0x3f, 34);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_1, 8,
				    0xff, csi_dev->mipi_cfg.reg_val);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_2, 12,
				    0xf, 4);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_2, 10,
				    0x3, 2);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_2, 8,
				    0x1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_2, 0,
				    0xff, csi_dev->mipi_cfg.reg_val);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_2, 9,
				    0x1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_4, 0,
				    0x3ff, 10);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_6, 0,
				    0x3ff, 20);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_7, 0, 0x7f,
			csi_dev->mipi_cfg.reg_val_ddlcal_start_delay);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_3, 0, 0x3ff,
			csi_dev->mipi_cfg.reg_val_ddlcal_counter_ref);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_1, 0,
				    0xff, csi_dev->mipi_cfg.max_phase);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_5, 4,
				    0x3f,
				    csi_dev->mipi_cfg.reg_val_ddlcal_dll_fbk);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_PPI_RW_DDLCAL_CFG_5, 0, 0xf,
			csi_dev->mipi_cfg.reg_val_ddlcal_ddl_coarse_bank);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8,
			12, 0x1,
			csi_dev->mipi_cfg.reg_val_lanex_hsrx_cdphy_sel_fast);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_8,
			12, 0x1,
			csi_dev->mipi_cfg.reg_val_lanex_hsrx_cdphy_sel_fast);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_8,
			12, 0x1,
			csi_dev->mipi_cfg.reg_val_lanex_hsrx_cdphy_sel_fast);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_8,
			12, 0x1,
			csi_dev->mipi_cfg.reg_val_lanex_hsrx_cdphy_sel_fast);
		csi_write_cdphy_reg(
			csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_8,
			12, 0x1,
			csi_dev->mipi_cfg.reg_val_lanex_hsrx_cdphy_sel_fast);
	} else {
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_7, 0,
				    0xff, 104);
		csi_write_cdphy_reg(csi_dev,
				    CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_8, 0,
				    0xff, 80);
	}

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2, 0,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2, 0,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2, 0,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2, 0,
			    0x1, 0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_6, 3, 0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_RW_COMMON_6, 0, 0x7, 1);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_12_oa_lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_val_reg);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_13_oa_lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13, 1,
			    0x1, lanex_hsrx_dphy_ddl_bypass_en_ovr_en_reg);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_12_oa_lanex_hsrx_dphy_dll_en_ovr_val_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_val_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_val_reg);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_13_oa_lanex_hsrx_dphy_dll_en_ovr_en_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_en_reg);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13, 3,
			    0x1, lanex_hsrx_dphy_dll_en_ovr_en_reg);

	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_9_oa_lanex_hsrx_hs_clk_div_reg is determined
	// by a case statement that depends on the operating data rate in basic_tasks.v file (csi_calcul_cfg() task)
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 5,
			    0x7,
			    csi_dev->mipi_cfg.reg_val_lanex_hsrx_hs_clk_div);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_HS_RX_0, 0,
			    0xff, 28);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_HS_RX_7, 0,
			    0xff, 6);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_0, 8,
			    0xff, csi_dev->mipi_cfg.reg_val_hs_rx_thssettle);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_0, 8,
			    0xff, csi_dev->mipi_cfg.reg_val_hs_rx_thssettle);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_0, 8,
			    0xff, csi_dev->mipi_cfg.reg_val_hs_rx_thssettle);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_0, 8,
			    0xff, csi_dev->mipi_cfg.reg_val_hs_rx_thssettle);

	// The value of core_dig_dlane_x_rw_cfg_1_cfg_1_deskew_supported_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_CFG_1, 2, 0x1,
			    dlane_x_deskew_supported_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_CFG_1, 2, 0x1,
			    dlane_x_deskew_supported_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_CFG_1, 2, 0x1,
			    dlane_x_deskew_supported_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_CFG_1, 2, 0x1,
			    dlane_x_deskew_supported_reg);

	// The value of core_dig_dlane_x_rw_cfg_1_cfg_1_sot_detection_reg is determined
	// by a case statement that depends on the operating data rate in this csi_dphy_spec_cfg() task
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_CFG_1, 3, 0x1,
			    dlane_x_sot_detection_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_CFG_1, 3, 0x1,
			    dlane_x_sot_detection_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_CFG_1, 3, 0x1,
			    dlane_x_sot_detection_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_CFG_1, 3, 0x1,
			    dlane_x_sot_detection_reg);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_2, 15,
			    0x1, dlane_x_ignore_alterncal_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_2, 15,
			    0x1, dlane_x_ignore_alterncal_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_2, 15,
			    0x1, dlane_x_ignore_alterncal_reg);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_2, 15,
			    0x1, dlane_x_ignore_alterncal_reg);

	if (en_check_prbs9) {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_LP_0,
				    12, 0xf, 3);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_0,
				    12, 0xf, 3);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_0,
				    12, 0xf, 3);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_LP_0,
				    12, 0xf, 3);
	} else {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_LP_0,
				    12, 0xf, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_0,
				    12, 0xf, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_0,
				    12, 0xf, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_LP_0,
				    12, 0xf, 1);
	}

	if (en_check_prbs9) {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_LP_2,
				    0, 0x1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_2,
				    0, 0x1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_2,
				    0, 0x1, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_LP_2,
				    0, 0x1, 1);
	} else {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_LP_2,
				    0, 0x1, 0);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_LP_2,
				    0, 0x1, 0);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_LP_2,
				    0, 0x1, 0);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_LP_2,
				    0, 0x1, 0);
	}

	if (en_check_prbs9) {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_LP_0,
				    12, 0xf, 3);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_LP_2,
				    0, 0x1, 1);
	} else {
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_LP_0,
				    12, 0xf, 1);
		csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_LP_2,
				    0, 0x1, 0);
	}

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_2, 13,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_2, 13,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_2, 13,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_2, 13,
			    0x1, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_1, 0,
			    0xff, FILTER_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_1, 0,
			    0xff, FILTER_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_1, 0,
			    0xff, FILTER_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_1, 0,
			    0xff, FILTER_SIZE_DESKEW);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_2, 9,
			    0xf, WINDOW_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_2, 9,
			    0xf, WINDOW_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_2, 9,
			    0xf, WINDOW_SIZE_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_2, 9,
			    0xf, WINDOW_SIZE_DESKEW);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_3, 0,
			    0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_3, 0,
			    0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_3, 0,
			    0x7, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_3, 0,
			    0x7, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_4, 0,
			    0xffff, MAX_ITERATION_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_4, 0,
			    0xffff, MAX_ITERATION_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_4, 0,
			    0xffff, MAX_ITERATION_DESKEW);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_4, 0,
			    0xffff, MAX_ITERATION_DESKEW);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_5, 0,
			    0xff, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_5, 0,
			    0xff, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_5, 0,
			    0xff, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_5, 0,
			    0xff, 0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_5, 8,
			    0xff, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_5, 8,
			    0xff, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_5, 8,
			    0xff, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_5, 8,
			    0xff, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_6, 0,
			    0xff, 2);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_6, 0,
			    0xff, 2);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_6, 0,
			    0xff, 2);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_6, 0,
			    0xff, 2);

	// startup_sequence_02p05 version #1.3
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_7, 13,
			    0x1, 0); //FIXME from 1 to 0
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_7, 13,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_7, 13,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_7, 13,
			    0x1, 0);

	// startup_sequence_02p05 version #1.8
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_7, 15,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_7, 15,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_7, 15,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_7, 15,
			    0x1, 0);

	// The value of max_phase is determined by a case statement that depends on the operating
	// data rate in basic_tasks.v file (csi_calcul_cfg() task)
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_3, 3,
			    0x3f, csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_3, 3,
			    0x3f, csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_3, 3,
			    0x3f, csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_3, 3,
			    0x3f, csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);

	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_9, 0, 0xff,
		csi_dev->mipi_cfg.phase_bound_reg -
			csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_9, 0, 0xff,
		csi_dev->mipi_cfg.phase_bound_reg -
			csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_9, 0, 0xff,
		csi_dev->mipi_cfg.phase_bound_reg -
			csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_9, 0, 0xff,
		csi_dev->mipi_cfg.phase_bound_reg -
			csi_dev->mipi_cfg.reg_val_hs_rx_fjump_deskew);

	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_HS_RX_6, 8, 0xff,
		csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_HS_RX_6, 8, 0xff,
		csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_HS_RX_6, 8, 0xff,
		csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew);
	csi_write_cdphy_reg(
		csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_HS_RX_6, 8, 0xff,
		csi_dev->mipi_cfg.reg_val_hs_rx_min_eye_opening_deskew);

	for (i = 0; i < 128; i++) {
		csi_cdphy_iowrite32(csi_dev,
				    CSI_PHY_CORE_DIG_COMMON_RW_DESKEW_FINE_MEM,
				    deskew_regs[i]);
	}
}

void csi_dphy_option_cfg(struct bst_csi_device *csi_dev)
{
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_CFG_0, 0,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_CLK_RW_CFG_0, 1,
			    0x1, 0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_CFG_0, 0, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_CFG_0, 0, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_CFG_0, 0, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_CFG_0, 0, 0x1,
			    0);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_0_RW_CFG_0, 1, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_1_RW_CFG_0, 1, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_2_RW_CFG_0, 1, 0x1,
			    0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_DLANE_3_RW_CFG_0, 1, 0x1,
			    0);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 8,
			    0x1, 0);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 8,
			    0x1, 0);
}

void add_speed_reg(struct bst_csi_device *csi_dev)
{
	csi_write_cdphy_reg(csi_dev, CSI_PHY_PPI_STARTUP_RW_COMMON_DPHY_2, 0,
			    0xff, 16);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    3, 0x1, 0);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    4, 0x7f, 85);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			    3, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			    4, 0x1, 1);

	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    3, 0x1, 1);
	csi_write_cdphy_reg(csi_dev, CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			    3, 0x1, 0);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 0,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_14, 0,
			    0xf, 15);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 11,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_14, 4,
			    0xf, 15);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15, 6,
			    0x1, 1);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15, 7,
			    0x1, 1);

	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_0, 0,
			    0xff, 255);
	csi_write_cdphy_reg(csi_dev,
			    CSI_PHY_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_0, 8,
			    0xff, 255);
}

void all_interupt_mask(struct bst_csi_device *csi_dev, unsigned short on_off)
{
	u32 reg_mask, reg_read;

	if (on_off == 0)
		reg_mask = 0xffffffff;
	else
		reg_mask = 0;

	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_PHY_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_PKT_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_PHY, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_LINE, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_BNDRY_FRAME_FATAL,
			   reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_SEQ_FRAME_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_CRC_FRAME_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_PLD_CRC_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_DATA_ID, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_ECC_CORRECT, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_AP_GENERIC, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_PHY_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_PKT_FATAL, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_PHY, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_LINE, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_BNDRY_FRAME_FATAL,
			   reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_SEQ_FRAME_FATAL,
			   reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_CRC_FRAME_FATAL,
			   reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_PLD_CRC_FATAL,
			   reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_DATA_ID, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_ECC_CORRECT, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_LOGGER_ERR, reg_mask);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_INT_MSK_FAP_BNDRY_FRAME_FATAL,
			   reg_mask);

	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_PHY_FATAL);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_PKT_FATAL);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_PLD_CRC_FATAL);
	reg_read =
		csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_BNDRY_FRAME_FATAL);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_SEQ_FRAME_FATAL);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_CRC_FRAME_FATAL);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_PHY);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_DATA_ID);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_ECC_CORRECT);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_LINE);
	reg_read = csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_MAIN);
	csi_read_ctrl_reg(csi_dev, CSI_CTRL_INT_ST_AP_MAIN);

	//some error int reg clear after read
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_GENERIC);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI2_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI3_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI4_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI5_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI6_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI7_FATAL);
	csi_read_ctrl_reg(csi_dev, INT_ST_AP_IPI8_FATAL);

	reg_read = csi_read_top_reg(csi_dev, CSI_TOP_INT_F_INTR_ENABLE);
	reg_read = 0xfffffff7;
	csi_write_top_reg(csi_dev, CSI_TOP_INT_F_INTR_ENABLE, reg_read);

	reg_read = csi_read_top_reg(csi_dev, CSI_TOP_INT_E_INTR_ENABLE);
	reg_read = 0xffffffff;
	csi_write_top_reg(csi_dev, CSI_TOP_INT_E_INTR_ENABLE, reg_read);

	reg_read = csi_read_top_reg(csi_dev, CSI_TOP_INT_D_INTR_ENABLE);
	reg_read = 0xffffffff;
	csi_write_top_reg(csi_dev, CSI_TOP_INT_D_INTR_ENABLE, reg_read);

	csi_write_top_reg(csi_dev, 0xa0, 0xffffffff);
	csi_write_top_reg(csi_dev, 0xb0, 0xffffffff);
	csi_write_top_reg(csi_dev, 0xc0, 0xffffffff);
}

void csi_cdphy_config_lanes(struct bst_csi_device *csi_dev)
{
	u32 read_val = 0;
	int retrytime = 0;
	int phy_cfg = 0; //0 for dphy
	u32 tempd = 0;

	mdelay(10);

	//release host preset
	csi_write_top_reg(csi_dev, CSI_TOP_CONTRL, 0x00400001);
	mdelay(10);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_PHY_SHUTDOWNZ, 0);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_DPHY_RSTZ, 0);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_CSI2_RESETN, 0);
	mdelay(10);
	if (csi_dev->phy_mode_cfg == 1) {
		phy_cfg = 1;
		if (csi_dev->num_lanes >= 3)
			csi_dev->num_lanes = 3;
	}
	tempd = (phy_cfg == 0) ? 0x1 : 0x5;

	csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE0, tempd);
	csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE1, tempd);
	csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE2, tempd);
	csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE3, tempd);
	csi_write_top_reg(csi_dev, CSI_TOP_PHY_CLK, 0x00000005);
	mdelay(10);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_PHY_MODE,
			   csi_dev->phy_mode_cfg); //phy_mode 0:DPHY 1:CPHY
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_PHY_CFG, phy_cfg); //O-ppi8 1-ppi16
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_N_LANES, csi_dev->num_lanes - 1);
	mdelay(10);
	csi_write_top_reg(csi_dev, CSI_TOP_CONTRL, 0x00400009);
	mdelay(10);
	csi_calcul_cfg(csi_dev);
	//csi_cdphy_config_common
	csi_cdphy_config_common(csi_dev);

	if (phy_cfg == 0) {
		csi_dphy_spec_cfg(csi_dev);
		csi_dphy_option_cfg(csi_dev);
		csi_cdphy_equalizer_cfg(csi_dev);
	} else {
		csi_cphy_spec_cfg(csi_dev);
		csi_cphy_option_cfg(csi_dev);
		csi_cdphy_equalizer_cfg(csi_dev);
	}

	mdelay(10);
	all_interupt_mask(csi_dev, 0);
	mdelay(10);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_PHY_SHUTDOWNZ, 0xffffffff);
	mdelay(10);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_DPHY_RSTZ, 0xffffffff);
	mdelay(10);
	csi_write_ctrl_reg(csi_dev, CSI_CTRL_CSI2_RESETN, 0xffffffff);

	mdelay(50);

	while ((!(read_val & 0x80000000)) && (retrytime < 30)) {
		retrytime++;
		read_val = csi_read_top_reg(csi_dev, CSI_TOP_SYS_RD);
		mdelay(10);
	}
	read_val = 0;
	if (csi_dev->num_lanes == 1) {
		while ((!((read_val & 0x1f0) == 0x110)) && (retrytime < 30)) {
			retrytime++;
			mdelay(10);
			read_val = csi_read_top_reg(csi_dev, CSI_TOP_IPTEST0);
		}
		if ((read_val & 0x1f0) == 0x110)
			dev_info(csi_dev->dev, "check lane sts PASS\n");
		else
			dev_err(csi_dev->dev, "check lane sts FAILED\n");
	}

	if (csi_dev->num_lanes == 2) {
		while ((!((read_val & 0x1f0) == 0x130)) && (retrytime < 30)) {
			retrytime++;
			mdelay(10);
			read_val = csi_read_top_reg(csi_dev, CSI_TOP_IPTEST0);
		}
		if ((read_val & 0x1f0) == 0x130)
			dev_info(csi_dev->dev, "check lane sts PASS\n");
		else
			dev_err(csi_dev->dev, "check lane sts FAILED\n");
	}

	if (csi_dev->num_lanes == 3) {
		while ((!((read_val & 0x1f0) == 0x170)) && (retrytime < 30)) {
			retrytime++;
			mdelay(10);
			read_val = csi_read_top_reg(csi_dev, CSI_TOP_IPTEST0);
		}
		if ((read_val & 0x1f0) == 0x170)
			dev_info(csi_dev->dev, "check lane sts PASS\n");
		else
			dev_err(csi_dev->dev, "check lane sts FAILED\n");
	}

	if (csi_dev->num_lanes == 4) {
		while ((!((read_val & 0x1f0) == 0x1f0)) && (retrytime < 30)) {
			retrytime++;
			mdelay(10);
			read_val = csi_read_top_reg(csi_dev, CSI_TOP_IPTEST0);
		}

		if ((read_val & 0x1f0) == 0x1f0)
			dev_info(csi_dev->dev, "check lane sts PASS\n");
		else
			dev_err(csi_dev->dev, "check lane sts FAILED\n");
	}
	mdelay(10);

	csi_write_top_reg(csi_dev, 0x3C, 0x80000000);
	if (phy_cfg == 0) {
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE0, 0x0);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE1, 0x0);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE2, 0x0);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE3, 0x0);
	} else {
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE0, 0x4);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE1, 0x4);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE2, 0x4);
		csi_write_top_reg(csi_dev, CSI_TOP_PHY_LANE3, 0x4);
	}
	csi_write_top_reg(csi_dev, CSI_TOP_PHY_CLK, 0x1);
}
