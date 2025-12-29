// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math.h>

#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "csi_cdphy.h"

#include "csi_controller.h"
#include "csi_hw.h"
#include "csi_safety.h"

static const u32 deskew_settings[] = {
	0x0404, 0x040C, 0x0414, 0x041C, 0x0423, 0x0429, 0x0430, 0x043A, 0x0445,
	0x044A, 0x0450, 0x045A, 0x0465, 0x0469, 0x0472, 0x047A, 0x0485, 0x0489,
	0x0490, 0x049A, 0x04A4, 0x04AC, 0x04B4, 0x04BC, 0x04C4, 0x04CC, 0x04D4,
	0x04DC, 0x04E4, 0x04EC, 0x04F4, 0x04FC, 0x0504, 0x050C, 0x0514, 0x051C,
	0x0523, 0x0529, 0x0530, 0x053A, 0x0545, 0x054A, 0x0550, 0x055A, 0x0565,
	0x0569, 0x0572, 0x057A, 0x0585, 0x0589, 0x0590, 0x059A, 0x05A4, 0x05AC,
	0x05B4, 0x05BC, 0x05C4, 0x05CC, 0x05D4, 0x05DC, 0x05E4, 0x05EC, 0x05F4,
	0x05FC, 0x0604, 0x060C, 0x0614, 0x061C, 0x0623, 0x0629, 0x0632, 0x063A,
	0x0645, 0x064A, 0x0650, 0x065A, 0x0665, 0x0669, 0x0672, 0x067A, 0x0685,
	0x0689, 0x0690, 0x069A, 0x06A4, 0x06AC, 0x06B4, 0x06BC, 0x06C4, 0x06CC,
	0x06D4, 0x06DC, 0x06E4, 0x06EC, 0x06F4, 0x06FC, 0x0704, 0x070C, 0x0714,
	0x071C, 0x0723, 0x072A, 0x0730, 0x073A, 0x0745, 0x074A, 0x0750, 0x075A,
	0x0765, 0x0769, 0x0772, 0x077A, 0x0785, 0x0789, 0x0790, 0x079A, 0x07A4,
	0x07AC, 0x07B4, 0x07BC, 0x07C4, 0x07CC, 0x07D4, 0x07DC, 0x07E4, 0x07EC,
	0x07F4, 0x07FC
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void phy_update_once(struct csi_device *csi, u32 reg, u32 off, u32 mask,
			    u32 val)
{
	u32 rval;
	u32 wval;

	rval = csi_phy_get(csi, reg);
	wval = (rval & (~(mask << off))) | ((val & mask) << off);
	dev_dbg(csi->dev,
		"CPUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X\n",
		reg, val, mask, rval, wval);
	csi_phy_set(csi, reg, rval);
}

#pragma GCC diagnostic pop

static int phy_update(struct csi_device *csi, u32 reg, u32 off, u32 mask,
		      u32 val)
{
	u32 rval;
	u32 wval;
	int i;

	rval = csi_phy_get(csi, reg);
	wval = (rval & (~(mask << off))) | ((val & mask) << off);
	dev_dbg(csi->dev,
		"CPUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X\n",
		reg, val, mask, rval, wval);
	i = 0;
	do {
		csi_phy_set(csi, reg, wval);
		rval = csi_phy_get(csi, reg);
		if (rval == wval)
			return 0;
	} while (++i <= csi->phy_access_retries);

#ifdef CONFIG_BST_HEALTH_MONITOR
	if (csi->psm.phy_access_confirm)
		send_dtc_to_safety_svc(MKDTC(csi->id, PSM_ID_PHY_REG_ACCESS));
#endif
	dev_err(csi->dev,
		"CPUP: 0x%08X -> 0x%08X & 0x%08X, rval: 0x%08X, wval: 0x%08X, FAILED\n",
		reg, val, mask, rval, wval);

	return -EIO;
}

static void config_common(struct csi_device *csi, struct phy_cfg *cfg)
{
	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_10, 0, 0xFF, 48);

	phy_update(csi, R_CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2, 13, 0x1, 0);
	phy_update(csi, R_CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2, 12, 0x1, 1);
	phy_update(csi, R_CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0, 2, 0x3F, 63);
	phy_update(csi, R_PPI_STARTUP_RW_COMMON_STARTUP_1_1, 0, 0xFFF, 563);
	// PoR FSM RCAL state control
	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_2, 0, 0xFF, 0x5);
	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_6, 0, 0xFF, 39);
	phy_update(csi, R_PPI_CALIBCTRL_RW_COMMON_BG_0, 0, 0x1FF, 500);
	phy_update(csi, R_PPI_RW_TERMCAL_CFG_0, 0, 0x7F, cfg->termcal_timer);
	phy_update(csi, R_PPI_RW_OFFSETCAL_CFG_0, 0, 0x1F,
		   cfg->offsetcal_wait_thresh);

	phy_update(csi, R_PPI_RW_LPDCOCAL_TIMEBASE, 0, 0x3FF,
		   cfg->lpdcocal_timebase);
	phy_update(csi, R_PPI_RW_LPDCOCAL_NREF, 0, 0x7FF, 800);
	phy_update(csi, R_PPI_RW_LPDCOCAL_NREF_RANGE, 0, 0x1F, 0x1B);
	phy_update(csi, R_PPI_RW_LPDCOCAL_TWAIT_CONFIG, 9, 0x7F, 127);
	phy_update(csi, R_PPI_RW_LPDCOCAL_TWAIT_CONFIG, 0, 0x1FF,
		   cfg->lpdcocal_twait_coarse);
	phy_update(csi, R_PPI_RW_LPDCOCAL_VT_CONFIG, 7, 0x1FF,
		   cfg->lpdcocal_twait_coarse);
	phy_update(csi, R_PPI_RW_LPDCOCAL_VT_CONFIG, 2, 0x1F, 27);
	phy_update(csi, R_PPI_RW_LPDCOCAL_VT_CONFIG, 1, 0x1, 1);
	phy_update(csi, R_PPI_RW_LPDCOCAL_VT_CONFIG, 0, 0x1, VT_TRACKING_EN);
	phy_update(csi, R_PPI_RW_LPDCOCAL_COARSE_CFG, 0, 0x3, 1);

	phy_update(csi, R_PPI_RW_COMMON_CFG, 0, 0x3, 3);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0, 10, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1, 10, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1, 15, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_3, 8, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0, 15, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_3, 9, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6, 13, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7, 9, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6, 12, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6, 14, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7, 10, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_5, 8, 0x1, 0);
}

/* See CDPHY Databook 7.08a A.1 Support Formulas */
static void calc_cfg(struct csi_device *csi, struct phy_cfg *cfg)
{
	int timebase, itminrx;
	int des_ds_th;
	long t_hs_settle_ns;
	long t_hs_settle_ui;
	int th_stl_trgt;
	long hs_clk_frq;
	int i = 0;
	int det_dly;
	int post_rcvd_rst;
	long t_hs_temp_ns = 0;
	u32 hsdcocal_div;
	u32 calcu_dvd;
	u32 calcu_dvs;
	u64 calcu_dvd_l;
	u32 lane_speed = csi->lane_speed;

	int algo_setting[][6] = {
		{ 4500, 4500, 1, 63, 7, 0 },   { 4000, 4499, 1, 71, 7, 1 },
		{ 3600, 3999, 1, 79, 9, 1 },   { 3230, 3599, 1, 87, 9, 1 },
		{ 3000, 3229, 0, 71, 7, 1 },   { 2700, 2999, 0, 79, 9, 1 },
		{ 2455, 2699, 0, 87, 9, 1 },   { 2250, 2454, 0, 95, 11, 1 },
		{ 2077, 2249, 0, 103, 11, 2 }, { 1929, 2076, 0, 111, 13, 2 },
		{ 1800, 1928, 0, 119, 13, 2 }, { 1688, 1799, 0, 127, 15, 2 },
		{ 1588, 1687, 0, 135, 15, 3 }, { 1500, 1587, 0, 143, 17, 3 },
	};

	int phase_bound[] = { 71,  103, 103, 103, 103, 103, 103,
			      103, 135, 135, 135, 135, 167, 167 };

	int dif_osc_para[][10] = {
		{ 76, 23, 1, 95, 19, 9, 76, 23, 191, 96 },
		{ 79, 24, 1, 99, 19, 9, 79, 24, 199, 99 },
		{ 95, 29, 2, 119, 23, 11, 95, 29, 239, 119 },
		{ 103, 32, 2, 129, 25, 12, 103, 32, 259, 129 },
		{ 107, 33, 2, 134, 26, 13, 107, 33, 269, 134 },
		{ 153, 47, 3, 191, 38, 19, 153, 47, 383, 192 },
		{ 207, 64, 5, 259, 51, 25, 207, 64, 519, 259 },
	};

	int algo_opt[20];
	int k2 = 0;

	for (k2 = 2500; k2 < 4500; k2 = k2 + 100) {
		if (k2 == 2500 || k2 == 3000 || k2 == 3100 || k2 == 3300 ||
		    k2 == 4000 || k2 == 4100)
			algo_opt[k2 / 100 - 25] = 1;
		else if (k2 == 2600 || k2 == 3500 || k2 == 4300 || k2 == 4400)
			algo_opt[k2 / 100 - 25] = -1;
		else
			algo_opt[k2 / 100 - 25] = 0;
	}

	hs_clk_frq = (long)(lane_speed * 1000 / 2);
	timebase = 5000;
	itminrx = 4;
	des_ds_th = 1;

	cfg->ddlcal_counter_ref =
		(u32)DIV_ROUND_UP_ULL((timebase * hs_clk_frq), 32000000);
	cfg->phase_bound = 71;
	for (i = 0; i < ARRAY_SIZE(algo_setting); i++) {
		if ((lane_speed >= algo_setting[i][0]) &&
		    (lane_speed <= algo_setting[i][1])) {
			if ((lane_speed >= 2500 && lane_speed < 4500) &&
			    DESKEW_OPT) {
				int k1 = lane_speed / 100 - 25;
				int index_opt = algo_opt[k1];

				cfg->hsrx_cdphy_sel_fast =
					algo_setting[i + index_opt][2];
				cfg->max_phase = algo_setting[i + index_opt][3];
				cfg->ddlcal_dll_fbk =
					algo_setting[i + index_opt][4];
				cfg->ddlcal_ddl_coarse_bank =
					algo_setting[i + index_opt][5];
			} else {
				cfg->hsrx_cdphy_sel_fast = algo_setting[i][2];
				cfg->max_phase = algo_setting[i][3];
				cfg->ddlcal_dll_fbk = algo_setting[i][4];
				cfg->ddlcal_ddl_coarse_bank =
					algo_setting[i][5];
				cfg->phase_bound = phase_bound[i];
			}
			break;
		}
	}

	if (hs_clk_frq * 2 < 160000)
		cfg->hsrx_hs_clk_div = 1;
	else if (hs_clk_frq * 2 < 320000)
		cfg->hsrx_hs_clk_div = 2;
	else if (hs_clk_frq * 2 < 640000)
		cfg->hsrx_hs_clk_div = 3;
	else if (hs_clk_frq * 2 < 1280000)
		cfg->hsrx_hs_clk_div = 4;
	else if (hs_clk_frq * 2 < 2560000)
		cfg->hsrx_hs_clk_div = 5;
	else
		cfg->hsrx_hs_clk_div = 6;

	t_hs_settle_ns = MIN_T_HS_SETTLE_NS + MAX_T_HS_SETTLE_NS;
	t_hs_settle_ui = MIN_T_HS_SETTLE_UI + MAX_T_HS_SETTLE_UI;

	t_hs_temp_ns = t_hs_settle_ui * 1000000;

	th_stl_trgt = (u32)DIV_ROUND_UP_ULL(
		(t_hs_settle_ns * hs_clk_frq * 2 + t_hs_temp_ns),
		hs_clk_frq * T_DCO * 4);

	cfg->thssettle = th_stl_trgt - itminrx - 7;
	cfg->fjump_deskew = DIV_ROUND_UP(cfg->max_phase, 40);
	cfg->min_eye_opening_deskew = DIV_ROUND_UP(cfg->max_phase, 5);

	if (hs_clk_frq * 2 >= 900000) {
		calcu_dvd_l = timebase * hs_clk_frq * 2 * 2;
		calcu_dvs = 32 * 1000000 * 5;
		cfg->coarse_target =
			(u32)DIV_ROUND_UP_ULL(calcu_dvd_l, calcu_dvs) - 1;
	} else {
		calcu_dvd_l = timebase * 900000.0 * 2;
		calcu_dvs = 32 * 1000000 * 5;
		cfg->coarse_target =
			(u32)DIV_ROUND_UP_ULL(calcu_dvd_l, calcu_dvs) - 1;
	}

	calcu_dvd = 1000000 * 7 * 6;
	calcu_dvs = hs_clk_frq * 2 * T_DCO;
	cfg->delay_deass_thresh =
		(u32)(DIV_ROUND_UP(calcu_dvd, calcu_dvs) + des_ds_th);

	if (lane_speed >= 320)
		hsdcocal_div = 16;
	else if ((lane_speed >= 160) && (lane_speed < 320))
		hsdcocal_div = 8;
	else
		hsdcocal_div = 4;

	calcu_dvd = lane_speed * (51 + 1);
	calcu_dvs = hsdcocal_div * 26;
	cfg->lpcdcocal_nref = DIV_ROUND_CLOSEST(calcu_dvd, calcu_dvs);
	if (lane_speed >= 400) {
		calcu_dvd = lane_speed * (51 + 1);
		calcu_dvs = hsdcocal_div * 26 * 20;
		cfg->lpcdcocal_nref_range =
			DIV_ROUND_CLOSEST(calcu_dvd, calcu_dvs);
	} else {
		calcu_dvd = lane_speed * (51 + 1);
		calcu_dvs = hsdcocal_div * 26 * 10;
		cfg->lpcdcocal_nref_range =
			DIV_ROUND_CLOSEST(calcu_dvd, calcu_dvs);
	}

	calcu_dvd = 161 * 1000;
	calcu_dvs = lane_speed * T_DCO;
	det_dly = calcu_dvd / calcu_dvs - 7;
	cfg->post_det_delay_thresh = (det_dly > 1) ? det_dly : 1;

	calcu_dvd = 2 * (hs_clk_frq)*T_DCO * 2;
	calcu_dvs = 7 * 1000000;
	post_rcvd_rst = DIV_ROUND_CLOSEST(calcu_dvd, calcu_dvs) - 1;
	cfg->post_received_reset_thresh = (post_rcvd_rst > 0) ? post_rcvd_rst :
								1;

	cfg->lpdcocal_timebase = dif_osc_para[3][0];
	cfg->lpdcocal_twait_coarse = dif_osc_para[3][1];

	cfg->termcal_timer = (CFG_CLK - 1) / 1;
	cfg->offsetcal_wait_thresh = DIV_ROUND_UP(CFG_CLK, 5) - 1;
	calcu_dvd = timebase * CFG_CLK;
	calcu_dvs = 1000;
	cfg->ddlcal_timebase_target = DIV_ROUND_UP(calcu_dvd, calcu_dvs) - 1;
	cfg->ddlcal_start_delay = DIV_ROUND_UP(CFG_CLK, 2) - 1;
}

static void config_cphy_specific(struct csi_device *csi, struct phy_cfg *cfg)
{
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 0, 0x3, 0x1);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 2, 0x3, 0x1);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 4, 0x3, 0x1);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 6, 0x3, 0x1);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 8, 0x3, 0x1);

	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_7, 0, 0xFF, 104);
	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_8, 0, 0xFF, 16);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_0, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_0, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_0, 0, 0x1, 1);
	if (csi->lane_speed > 900) {
		phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_0, 1, 0x1, 1);
		phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_0, 1, 0x1, 1);
		phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_0, 1, 0x1, 1);
	} else {
		phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_0, 1, 0x1, 0);
		phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_0, 1, 0x1, 0);
		phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_0, 1, 0x1, 0);
	}

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_1, 0, 0xFFFF, 25);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_1, 0, 0xFFFF, 25);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_1, 0, 0xFFFF, 25);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_5, 0, 0xFFFF,
		   cfg->termcal_timer);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_5, 0, 0xFFFF,
		   cfg->termcal_timer);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_5, 0, 0xFFFF,
		   cfg->termcal_timer);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_6, 0, 0xFFFF, 10);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_6, 0, 0xFFFF, 10);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_6, 0, 0xFFFF, 10);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_2, 0, 0xFFFF,
		   cfg->coarse_target);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_2, 0, 0xFFFF,
		   cfg->coarse_target);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_2, 0, 0xFFFF,
		   cfg->coarse_target);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2, 0, 0x1, 0);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_0, 6, 0x7, 1);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_0, 6, 0x7, 1);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_0, 6, 0x7, 1);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_0, 3, 0x7, 1);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_0, 3, 0x7, 1);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_0, 3, 0x7, 1);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_2, 0, 0xFF,
		   cfg->delay_deass_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_2, 0, 0xFF,
		   cfg->delay_deass_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_2, 0, 0xFF,
		   cfg->delay_deass_thresh);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_0, 0, 0x7, 2);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_0, 0, 0x7, 2);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_0, 0, 0x7, 2);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_1, 0, 0xFFFF,
		   cfg->post_det_delay_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_1, 0, 0xFFFF,
		   cfg->post_det_delay_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_1, 0, 0xFFFF,
		   cfg->post_det_delay_thresh);

	phy_update(csi, R_CORE_DIG_RW_TRIO0_0, 9, 0x7,
		   cfg->post_received_reset_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO1_0, 9, 0x7,
		   cfg->post_received_reset_thresh);
	phy_update(csi, R_CORE_DIG_RW_TRIO2_0, 9, 0x7,
		   cfg->post_received_reset_thresh);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_LP_0, 12, 0xF, 1);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_LP_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_LP_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_LP_2, 0, 0x1, 0);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_HS_RX_0, 2, 0x1F, 12);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_HS_RX_0, 2, 0x1F, 12);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_HS_RX_0, 2, 0x1F, 12);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_5, 14, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_3_5, 14, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_3_5, 14, 0x1, 0);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_3_6, 3, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_3_6, 3, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_3_6, 3, 0x1, 0);
}

static void config_cphy_optional(struct csi_device *csi)
{
	phy_update(csi, R_CORE_DIG_CLANE_0_RW_CFG_0, 0, 0x7, 0);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_CFG_0, 0, 0x7, 0);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_CFG_0, 0, 0x7, 0);

	phy_update(csi, R_CORE_DIG_CLANE_0_RW_CFG_0, 3, 0x1, 0);
	phy_update(csi, R_CORE_DIG_CLANE_1_RW_CFG_0, 3, 0x1, 0);
	phy_update(csi, R_CORE_DIG_CLANE_2_RW_CFG_0, 3, 0x1, 0);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 8, 0x1, 0);
}

static void config_cphy_equalizer(struct csi_device *csi)
{
	u32 eq = csi->eq;
	u32 gmode = csi->lane_speed <= 2500 ? 0 : 2;
	u32 cap_prog = csi->lane_speed <= 920  ? 0 :
		       csi->lane_speed <= 1030 ? 1 :
		       csi->lane_speed <= 1170 ? 2 :
		       csi->lane_speed <= 1350 ? 3 :
		       csi->lane_speed <= 1590 ? 4 :
		       csi->lane_speed <= 1950 ? 5 :
		       csi->lane_speed <= 2500 ? 6 :
						 7;
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 0, 0x7, eq);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 3, 0x3, gmode);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_7, 10, 0x7,
		   cap_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_7, 10, 0x7,
		   cap_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_7, 10, 0x7,
		   cap_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_7, 10, 0x7,
		   cap_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_7, 10, 0x7,
		   cap_prog);

	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_A, 0, 0xFF, 48);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_5, 8, 0x1, 1);
}

void csi_phy_config_cphy(struct csi_device *csi)
{
	struct phy_cfg cfg;

	calc_cfg(csi, &cfg);
	config_common(csi, &cfg);

	config_cphy_specific(csi, &cfg);
	config_cphy_optional(csi);
	config_cphy_equalizer(csi);
}

static void config_dphy_specific(struct csi_device *csi, struct phy_cfg *cfg)
{
	int i;
	unsigned int hsrx_ddl_byps_val;
	unsigned int hsrx_ddl_byps_en;
	unsigned int hsrx_ddl_val;
	unsigned int lanex_hsrx_dphy_dll_en_ovr_en_reg;
	unsigned int dlane_x_sot_detection_reg;
	unsigned int dlane_x_ignore_alterncal_reg;
	unsigned int dlane_x_deskew_supported_reg;

	if (csi->lane_speed <= 1500) {
		hsrx_ddl_byps_val = 1;
		hsrx_ddl_byps_en = 1;
		hsrx_ddl_val = 0;
		lanex_hsrx_dphy_dll_en_ovr_en_reg = 1;
		dlane_x_deskew_supported_reg = 0;
		dlane_x_sot_detection_reg = 1;
	} else {
		hsrx_ddl_byps_val = 0;
		hsrx_ddl_byps_en = 0;
		hsrx_ddl_val = 0;
		lanex_hsrx_dphy_dll_en_ovr_en_reg = 0;
		dlane_x_deskew_supported_reg = 1;
		dlane_x_sot_detection_reg = 0;
	}

	if (csi->lane_speed <= 2500)
		dlane_x_ignore_alterncal_reg = 1;
	else
		dlane_x_ignore_alterncal_reg = 0;

	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 0, 0x3, 0x0);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 2, 0x3, 0x0);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 4, 0x3, 0x0);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 6, 0x3, 0x0);
	phy_update(csi, R_CORE_DIG_RW_COMMON_7, 8, 0x3, 0x0);

	if (csi->lane_speed >= 1500) {
		phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_7, 0, 0xFF, 40);
		phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_8, 0, 0xFF, 80);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_0, 0, 0x3FF,
			   cfg->ddlcal_timebase_target);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_7, 7, 0x3F, 34);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_1, 8, 0xFF,
			   cfg->termcal_timer);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_2, 12, 0xF, 4);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_2, 10, 0x3, 2);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_2, 8, 0x1, 1);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_2, 0, 0xFF,
			   cfg->termcal_timer);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_2, 9, 0x1, 1);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_4, 0, 0x3FF, 10);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_6, 0, 0x3FF, 20);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_7, 0, 0x7F,
			   cfg->ddlcal_start_delay);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_3, 0, 0x3FF,
			   cfg->ddlcal_counter_ref);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_1, 0, 0xFF, cfg->max_phase);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_5, 4, 0x3F,
			   cfg->ddlcal_dll_fbk);
		phy_update(csi, R_PPI_RW_DDLCAL_CFG_5, 0, 0xF,
			   cfg->ddlcal_ddl_coarse_bank);
		phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8, 12,
			   0x1, cfg->hsrx_cdphy_sel_fast);
		phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_8, 12,
			   0x1, cfg->hsrx_cdphy_sel_fast);
		phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_8, 12,
			   0x1, cfg->hsrx_cdphy_sel_fast);
		phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_8, 12,
			   0x1, cfg->hsrx_cdphy_sel_fast);
		phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_8, 12,
			   0x1, cfg->hsrx_cdphy_sel_fast);
	} else {
		phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_7, 0, 0xFF, 104);
		phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_8, 0, 0xFF, 80);
	}

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2, 0, 0x1, 0);

	phy_update(csi, R_CORE_DIG_RW_COMMON_6, 3, 0x7, 1);
	phy_update(csi, R_CORE_DIG_RW_COMMON_6, 0, 0x7, 1);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_12_oa_hsrx_ddl_ovr_val is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12, 1, 0x1,
		   hsrx_ddl_byps_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12, 1, 0x1,
		   hsrx_ddl_byps_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12, 1, 0x1,
		   hsrx_ddl_byps_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12, 1, 0x1,
		   hsrx_ddl_byps_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12, 1, 0x1,
		   hsrx_ddl_byps_val);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_13_oa_hsrx_ddl_ovr_en is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 1, 0x1,
		   hsrx_ddl_byps_en);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13, 1, 0x1,
		   hsrx_ddl_byps_en);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13, 1, 0x1,
		   hsrx_ddl_byps_en);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13, 1, 0x1,
		   hsrx_ddl_byps_en);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13, 1, 0x1,
		   hsrx_ddl_byps_en);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_12_oa_lanex_hsrx_dphy_dll_en_ovr_val_reg is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12, 3, 0x1,
		   hsrx_ddl_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_12, 3, 0x1,
		   hsrx_ddl_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_12, 3, 0x1,
		   hsrx_ddl_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_12, 3, 0x1,
		   hsrx_ddl_val);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_12, 3, 0x1,
		   hsrx_ddl_val);
	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_13_oa_lanex_hsrx_dphy_dll_en_ovr_en_reg is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13, 3, 0x1,
		   lanex_hsrx_dphy_dll_en_ovr_en_reg);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_13, 3, 0x1,
		   lanex_hsrx_dphy_dll_en_ovr_en_reg);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_13, 3, 0x1,
		   lanex_hsrx_dphy_dll_en_ovr_en_reg);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_13, 3, 0x1,
		   lanex_hsrx_dphy_dll_en_ovr_en_reg);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_13, 3, 0x1,
		   lanex_hsrx_dphy_dll_en_ovr_en_reg);

	// The value of core_dig_ioctrl_rw_afe_lanex_ctrl_2_9_oa_lanex_hsrx_hs_clk_div_reg is determined
	// by a case statement that depends on the operating data rate in basic_tasks.v file (calcul_cfg() task)
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 5, 0x7,
		   cfg->hsrx_hs_clk_div);

	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_HS_RX_0, 0, 0xFF, 28);
	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_HS_RX_7, 0, 0xFF, 6);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_0, 8, 0xFF, cfg->thssettle);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_0, 8, 0xFF, cfg->thssettle);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_0, 8, 0xFF, cfg->thssettle);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_0, 8, 0xFF, cfg->thssettle);

	// The value of core_dig_dlane_x_rw_cfg_1_cfg_1_deskew_supported_reg is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_CFG_1, 2, 0x1,
		   dlane_x_deskew_supported_reg);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_CFG_1, 2, 0x1,
		   dlane_x_deskew_supported_reg);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_CFG_1, 2, 0x1,
		   dlane_x_deskew_supported_reg);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_CFG_1, 2, 0x1,
		   dlane_x_deskew_supported_reg);

	// The value of core_dig_dlane_x_rw_cfg_1_cfg_1_sot_detection_reg is determined
	// by a case statement that depends on the operating data rate in this dphy_spec_cfg() task
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_CFG_1, 3, 0x1,
		   dlane_x_sot_detection_reg);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_CFG_1, 3, 0x1,
		   dlane_x_sot_detection_reg);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_CFG_1, 3, 0x1,
		   dlane_x_sot_detection_reg);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_CFG_1, 3, 0x1,
		   dlane_x_sot_detection_reg);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_2, 15, 0x1,
		   dlane_x_ignore_alterncal_reg);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_2, 15, 0x1,
		   dlane_x_ignore_alterncal_reg);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_2, 15, 0x1,
		   dlane_x_ignore_alterncal_reg);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_2, 15, 0x1,
		   dlane_x_ignore_alterncal_reg);

#ifdef PRBS9_CHECK_EN
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_LP_0, 12, 0xF, 3);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_LP_0, 12, 0xF, 3);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_LP_0, 12, 0xF, 3);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_LP_0, 12, 0xF, 3);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_LP_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_LP_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_LP_2, 0, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_LP_2, 0, 0x1, 1);

	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_LP_0, 12, 0xF, 3);
	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_LP_2, 0, 0x1, 1);
#else
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_LP_0, 12, 0xF, 1);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_LP_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_LP_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_LP_2, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_LP_2, 0, 0x1, 0);

	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_LP_0, 12, 0xF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_LP_2, 0, 0x1, 0);

#endif

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_2, 13, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_2, 13, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_2, 13, 0x1, 1);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_2, 13, 0x1, 1);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_1, 0, 0xFF,
		   FILTER_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_1, 0, 0xFF,
		   FILTER_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_1, 0, 0xFF,
		   FILTER_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_1, 0, 0xFF,
		   FILTER_SIZE_DESKEW);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_2, 9, 0xF,
		   WINDOW_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_2, 9, 0xF,
		   WINDOW_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_2, 9, 0xF,
		   WINDOW_SIZE_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_2, 9, 0xF,
		   WINDOW_SIZE_DESKEW);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_3, 0, 0x7, 1);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_3, 0, 0x7, 1);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_3, 0, 0x7, 1);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_3, 0, 0x7, 1);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_4, 0, 0xFFFF,
		   MAX_ITERATIONS_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_4, 0, 0xFFFF,
		   MAX_ITERATIONS_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_4, 0, 0xFFFF,
		   MAX_ITERATIONS_DESKEW);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_4, 0, 0xFFFF,
		   MAX_ITERATIONS_DESKEW);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_5, 0, 0xFF, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_5, 0, 0xFF, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_5, 0, 0xFF, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_5, 0, 0xFF, 0);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_5, 8, 0xFF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_5, 8, 0xFF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_5, 8, 0xFF, 1);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_5, 8, 0xFF, 1);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_6, 0, 0xFF, 2);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_6, 0, 0xFF, 2);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_6, 0, 0xFF, 2);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_6, 0, 0xFF, 2);

	// startup_sequence_02p05 version #1.3
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_7, 13, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_7, 13, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_7, 13, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_7, 13, 0x1, 0);

	// startup_sequence_02p05 version #1.8
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_7, 15, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_7, 15, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_7, 15, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_7, 15, 0x1, 0);

	// The value of max_phase is determined by a case statement that depends on the operating
	// data rate in basic_tasks.v file (calcul_cfg() task)
	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_3, 3, 0x3F,
		   cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_3, 3, 0x3F,
		   cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_3, 3, 0x3F,
		   cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_3, 3, 0x3F,
		   cfg->fjump_deskew);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_9, 0, 0xFF,
		   cfg->phase_bound - cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_9, 0, 0xFF,
		   cfg->phase_bound - cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_9, 0, 0xFF,
		   cfg->phase_bound - cfg->fjump_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_9, 0, 0xFF,
		   cfg->phase_bound - cfg->fjump_deskew);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_HS_RX_6, 8, 0xFF,
		   cfg->min_eye_opening_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_HS_RX_6, 8, 0xFF,
		   cfg->min_eye_opening_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_HS_RX_6, 8, 0xFF,
		   cfg->min_eye_opening_deskew);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_HS_RX_6, 8, 0xFF,
		   cfg->min_eye_opening_deskew);

	for (i = 0; i < ARRAY_SIZE(deskew_settings); ++i)
		phy_update(csi, R_CORE_DIG_COMMON_RW_DESKEW_FINE_MEM, 0, 0x7FF,
			   deskew_settings[i]);
}

static void config_dphy_optional(struct csi_device *csi)
{
	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_CFG_0, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_CLK_RW_CFG_0, 1, 0x1, 0);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_CFG_0, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_CFG_0, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_CFG_0, 0, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_CFG_0, 0, 0x1, 0);

	phy_update(csi, R_CORE_DIG_DLANE_0_RW_CFG_0, 1, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_1_RW_CFG_0, 1, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_2_RW_CFG_0, 1, 0x1, 0);
	phy_update(csi, R_CORE_DIG_DLANE_3_RW_CFG_0, 1, 0x1, 0);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 8, 0x1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 8, 0x1, 0);
}

static void config_dphy_equalizer(struct csi_device *csi)
{
	u32 eq = csi->eq;
	u32 gmode = csi->lane_speed <= 2500 ? 0 : 2;
	u32 cp_prog = csi->lane_speed <= 1500 ? 0 : 3;

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 0, 0x7, eq);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 0, 0x7, eq);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9, 3, 0x3, gmode);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9, 3, 0x3, gmode);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15, 0, 0x7,
		   cp_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15, 0, 0x7,
		   cp_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15, 0, 0x7,
		   cp_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_15, 0, 0x7,
		   cp_prog);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_15, 0, 0x7,
		   cp_prog);

	phy_update(csi, R_PPI_STARTUP_RW_COMMON_DPHY_A, 0, 0xFF, 48);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_10, 2, 1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_10, 2, 1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_10, 2, 1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_10, 2, 1, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_10, 2, 1, 0);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15, 3, 0x3, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15, 3, 0x3, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15, 3, 0x3, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_15, 3, 0x3, 0);
	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_15, 3, 0x3, 0);

	phy_update(csi, R_CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_5, 8, 0x1, 1);
}

void csi_phy_config_dphy(struct csi_device *csi)
{
	struct phy_cfg cfg;

	calc_cfg(csi, &cfg);
	config_common(csi, &cfg);
	config_dphy_specific(csi, &cfg);
	config_dphy_optional(csi);
	config_dphy_equalizer(csi);
}
