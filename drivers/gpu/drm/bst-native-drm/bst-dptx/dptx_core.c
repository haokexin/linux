// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "dptx_csr.h"

/*
 * Core Access Layer
 *
 * Provides low-level register access to the DPTX core.
 */

/**
 * dptx_intr_en() - Enables interrupts
 * @dptx: The dptx struct
 * @bits: The interrupts to enable
 *
 * This function enables (unmasks) all interrupts in the INTERRUPT
 * register specified by @bits.
 */
static void dptx_intr_en(struct dptx *dptx, u32 bits)
{
	u32 ien;

	ien = dptx_read_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT_ENABLE);
	ien |= bits;
	dptx_write_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT_ENABLE, ien);
}

/**
 * dptx_intr_dis() - Disables interrupts
 * @dptx: The dptx struct
 * @bits: The interrupts to disable
 *
 * This function disables (masks) all interrupts in the INTERRUPT
 * register specified by @bits.
 */
static void dptx_intr_dis(struct dptx *dptx, u32 bits)
{
	u32 ien;

	ien = dptx_read_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT_ENABLE);
	ien &= ~bits;
	dptx_write_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT_ENABLE, ien);
}

/**
 * dptx_global_intr_en() - Enables top-level interrupts
 * @dptx: The dptx struct
 *
 * Enables (unmasks) all top-level interrupts.
 */
void dptx_global_intr_en(struct dptx *dptx)
{
	dptx_intr_en(dptx, DPTX_IEN_ALL_INTR &
		     ~(DPTX_ISTS_AUX_REPLY | DPTX_ISTS_AUX_CMD_INVALID));
}

/**
 * dptx_global_intr_dis() - Disables top-level interrupts
 * @dptx: The dptx struct
 *
 * Disables (masks) all top-level interrupts.
 */
void dptx_global_intr_dis(struct dptx *dptx)
{
	dptx_intr_dis(dptx, DPTX_IEN_ALL_INTR);
}

/**
 * dptx_soft_reset() - Performs a core soft reset
 * @dptx: The dptx struct
 * @bits: The components to reset
 *
 * Resets specified parts of the core by writing @bits into the core
 * soft reset control register and clearing them 10-20 microseconds
 * later.
 */
void dptx_soft_reset(struct dptx *dptx, u32 bits)
{
	u32 rst;

	bits &= (DPTX_SRST_CTRL_ALL);

	/* Set reset bits */
	rst = dptx_read_reg(dptx, dptx->regs[DPTX], SOFT_RESET_CTRL);
	rst |= bits;
	dptx_write_reg(dptx, dptx->regs[DPTX], SOFT_RESET_CTRL, rst);

	usleep_range(10, 20);

	/* Clear reset bits */
	rst = dptx_read_reg(dptx, dptx->regs[DPTX], SOFT_RESET_CTRL);
	rst &= ~bits;
	dptx_write_reg(dptx, dptx->regs[DPTX], SOFT_RESET_CTRL, rst);
}

/**
 * dptx_soft_reset_all() - Reset all core modules
 * @dptx: The dptx struct
 */
void dptx_soft_reset_all(struct dptx *dptx)
{
	dptx_soft_reset(dptx, DPTX_SRST_CTRL_ALL);
}

void dptx_phy_soft_reset(struct dptx *dptx)
{
	dptx_soft_reset(dptx, DPTX_SRST_CTRL_PHY);
}

/**
 * dptx_core_init_phy() - Initializes the DP TX PHY module
 * @dptx: The dptx struct
 *
 * Initializes the PHY layer of the core. This needs to be called
 * whenever the PHY layer is reset.
 */
void dptx_core_init_phy(struct dptx *dptx)
{
	dptx_write_regfield(dptx, dptx->field_phy_width, 0);
}

/**
 * dptx_sink_enabled_ssc() - Returns true, if sink is enabled ssc
 * @dptx: The dptx struct
 *
 */
bool dptx_sink_enabled_ssc(struct dptx *dptx)
{
	u8 byte;

	dptx_read_dpcd(dptx, DP_MAX_DOWNSPREAD, &byte);

	return byte & 1;
}

/**
 * dptx_core_program_ssc() - Move phy to P3 state and programs SSC
 * @dptx: The dptx struct
 *
 * Enables SSC should be called during hot plug.
 *
 */
int dptx_core_program_ssc(struct dptx *dptx, bool sink_ssc)
{
	u8  retval;

	dptx_phy_set_lanes(dptx, 4);
	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 3);

	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	if (dptx->ssc_en && sink_ssc)
		dptx_write_regfield(dptx, dptx->field_ssc_dis, 0);
	else
		dptx_write_regfield(dptx, dptx->field_ssc_dis, 1);

	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	return 0;
}

/**
 * dptx_check_dptx_id() - Check value of DPTX_ID register
 * @dptx: The dptx struct
 *
 * Returns True if DPTX core correctly identifyed.
 */
bool dptx_check_dptx_id(struct dptx *dptx)
{
	u32 dptx_id;

	dptx_id = dptx_read_reg(dptx, dptx->regs[DPTX], DPTX_ID);
	if (dptx_id != ((DPTX_ID_DEVICE_ID << DPTX_ID_DEVICE_ID_SHIFT) |
			DPTX_ID_VENDOR_ID))
		return false;

	return true;
}

/**
 * dptx_enable_ssc() - Enables SSC based on automation request,
 *		      if DPTX controller enables ssc
 * @dptx: The dptx struct
 *
 */
void dptx_enable_ssc(struct dptx *dptx)
{
	bool sink_ssc = dptx_sink_enabled_ssc(dptx);

	if (sink_ssc)
		dev_dbg(dptx->dev, "%s: SSC enable on the sink side\n", __func__);
	else
		dev_dbg(dptx->dev, "%s: SSC disabled on the sink side\n", __func__);
	dptx_core_program_ssc(dptx, sink_ssc);
}

void dptx_init_hwparams(struct dptx *dptx)
{
	dptx->streams = dptx_read_regfield(dptx, dptx->field_num_streams);
	dptx->hwparams.gen2phy = dptx_read_regfield(dptx, dptx->field_gen2_phy);
	dptx->hwparams.dsc = dptx_read_regfield(dptx, dptx->field_dsc_en);
	switch (dptx_read_regfield(dptx, dptx->field_mp_mode)) {
	default:
	case DPTX_CONFIG1_MP_MODE_SINGLE:
		dptx->hwparams.multipixel = DPTX_MP_SINGLE_PIXEL;
		break;
	case DPTX_CONFIG1_MP_MODE_DUAL:
		dptx->hwparams.multipixel = DPTX_MP_DUAL_PIXEL;
		break;
	case DPTX_CONFIG1_MP_MODE_QUAD:
		dptx->hwparams.multipixel = DPTX_MP_QUAD_PIXEL;
		break;
	}
}

/**
 * dptx_core_init() - Initializes the DP TX core
 * @dptx: The dptx struct
 *
 * Initialize the DP TX core and put it in a known state.
 */
int dptx_core_init(struct dptx *dptx)
{
	char str[15];
	u32 hpd_ien;

	dptx_soft_reset_all(dptx);
	dptx_write_reg(dptx, dptx->regs[DPTX], CCTL,
			DPTX_CCTL_SCALE_DOWN_MODE_EN
			| (dptx->enhance_frame_en ? DPTX_CCTL_ENH_FRAME_EN : 0)
			| (dptx->mst ? DPTX_CCTL_ENABLE_MST_MODE : 0) | BIT(9) | BIT(8) | (dptx->edp ? BIT(27) : 0));
	memset(str, 0, sizeof(str));
	dptx_core_init_phy(dptx);

	dptx_write_reg(dptx, dptx->regs[DPTX], AUX_250US_CNT_LIMIT, 0xf8);
	dptx_write_reg(dptx, dptx->regs[DPTX], AUX_2000US_CNT_LIMIT, 0x7C2);
	dptx_write_reg(dptx, dptx->regs[DPTX], AUX_100000US_CNT_LIMIT, 0x182b8);
	hpd_ien = dptx_read_reg(dptx, dptx->regs[DPTX], HPD_INTERRUPT_ENABLE);
	hpd_ien |= (DPTX_HPD_UNPLUG_ERR_EN | DPTX_HPD_IEN_IRQ_EN |
		    DPTX_HPD_IEN_HOT_PLUG_EN |
		    DPTX_HPD_IEN_HOT_UNPLUG_EN);
	dptx_write_reg(dptx, dptx->regs[DPTX], HPD_INTERRUPT_ENABLE, hpd_ien);

	dptx_global_intr_en(dptx);

	if (dptx->force_hpd) {
		dptx_csr_force_hpd(dptx, true);
	}

	return 0;
}

/**
 * dptx_core_deinit() - Deinitialize the core
 * @dptx: The dptx struct
 *
 * Disable the core in preparation for module shutdown.
 */
int dptx_core_deinit(struct dptx *dptx)
{
	dptx_global_intr_dis(dptx);
	dptx_soft_reset_all(dptx);
	return 0;
}

/*
 * PHYIF core access functions
 */

unsigned int dptx_phy_get_lanes(struct dptx *dptx)
{
	u32 val;

	val = dptx_read_regfield(dptx, dptx->field_phy_lanes);

	return (1 << val);
}

void dptx_phy_set_lanes(struct dptx *dptx, unsigned int lanes)
{
	u32 val;

	dptx_dbg(dptx, "%s: lanes=%d\n", __func__, lanes);

	switch (lanes) {
	case 1:
		val = 0;
		break;
	case 2:
		val = 1;
		break;
	case 4:
		val = 2;
		break;
	default:
		WARN(1, "Invalid number of lanes %d\n", lanes);
		return;
	}

	dptx_write_regfield(dptx, dptx->field_phy_lanes, val);
}

void dptx_phy_set_rate(struct dptx *dptx, unsigned int rate)
{
	dptx_dbg(dptx, "%s: rate=%d\n", __func__, rate);
	dptx_write_regfield(dptx, dptx->field_phyrate, rate);
}

unsigned int bst_phy_get_rate(struct dptx *dptx)
{
	u32 rate;

	rate = dptx_read_regfield(dptx, dptx->field_phyrate);

	return rate;
}

int dptx_phy_wait_busy(struct dptx *dptx, unsigned int lanes)
{
	unsigned int count;
	u32 phyifctrl;
	u32 mask = 0;

	dptx_dbg(dptx, "%s: lanes=%d\n", __func__, lanes);

	switch (lanes) {
	case 4:
		mask |= DPTX_PHYIF_CTRL_BUSY(3);
		mask |= DPTX_PHYIF_CTRL_BUSY(2);
		fallthrough;
	case 2:
		mask |= DPTX_PHYIF_CTRL_BUSY(1);
		fallthrough;
	case 1:
		mask |= DPTX_PHYIF_CTRL_BUSY(0);
		break;
	default:
		WARN(1, "Invalid number of lanes %d\n", lanes);
		break;
	}

	count = 0;

	while (1) {
		phyifctrl = dptx_read_reg(dptx, dptx->regs[DPTX], PHYIF_CTRL);

		if (!(phyifctrl & mask))  // if 0 means no busy and return 0;
			break;

		count++;
		if (count > 10000) { // was 50
			dptx_warn(dptx, "%s: PHY BUSY timed out\n", __func__);
			return -EBUSY;
		}
		usleep_range(10, 20);
	}

	return 0;
}

void dptx_phy_set_pre_emphasis(struct dptx *dptx,
			       unsigned int lane,
			       unsigned int level)
{
	u32 phytxeq;

	dptx_dbg(dptx, "%s: lane=%d, level=0x%x\n", __func__, lane, level);

	if (WARN(lane > 3, "Invalid lane %d", lane))
		return;

	if (WARN(level > 3, "Invalid pre-emphasis level %d, using 3", level))
		level = 3;

	phytxeq = dptx_read_reg(dptx, dptx->regs[DPTX], PHY_TX_EQ);
	phytxeq &= ~DPTX_PHY_TX_EQ_PREEMP_MASK(lane);
	phytxeq |= (level << DPTX_PHY_TX_EQ_PREEMP_SHIFT(lane)) &
		DPTX_PHY_TX_EQ_PREEMP_MASK(lane);

	dptx_write_reg(dptx, dptx->regs[DPTX], PHY_TX_EQ, phytxeq);
}

void dptx_phy_set_vswing(struct dptx *dptx,
			 unsigned int lane,
			 unsigned int level)
{
	u32 phytxeq;

	dptx_dbg(dptx, "%s: lane=%d, level=0x%x\n", __func__, lane, level);

	if (WARN(lane > 3, "Invalid lane %d", lane))
		return;

	if (WARN(level > 3, "Invalid vswing level %d, using 3", level))
		level = 3;

	phytxeq = dptx_read_reg(dptx, dptx->regs[DPTX], PHY_TX_EQ);
	phytxeq &= ~DPTX_PHY_TX_EQ_VSWING_MASK(lane);
	phytxeq |= (level << DPTX_PHY_TX_EQ_VSWING_SHIFT(lane)) &
		DPTX_PHY_TX_EQ_VSWING_MASK(lane);

	dptx_write_reg(dptx, dptx->regs[DPTX], PHY_TX_EQ, phytxeq);
}

void dptx_phy_set_pattern(struct dptx *dptx,
			  unsigned int pattern)
{
	dptx_dbg(dptx, "%s: Setting PHY pattern=0x%x\n", __func__, pattern);

	dptx_write_regfield(dptx, dptx->field_tps_sel, pattern);
}

void dptx_phy_enable_xmit(struct dptx *dptx, unsigned int lanes, bool enable)
{
	u32 phyifctrl;
	u32 mask = 0;

	dptx_dbg(dptx, "%s: lanes=%d, enable=%d\n", __func__, lanes, enable);

	phyifctrl = dptx_read_reg(dptx, dptx->regs[DPTX], PHYIF_CTRL);

	switch (lanes) {
	case 4:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(3);
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(2);
		fallthrough;
	case 2:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(1);
		fallthrough;
	case 1:
		mask |= DPTX_PHYIF_CTRL_XMIT_EN(0);
		break;
	default:
		WARN(1, "Invalid number of lanes %d\n", lanes);
		break;
	}

	if (enable)
		phyifctrl |= mask;
	else
		phyifctrl &= ~mask;

	dptx_write_reg(dptx, dptx->regs[DPTX], PHYIF_CTRL, phyifctrl);
}

int dptx_phy_rate_to_bw(unsigned int rate)
{
	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		return DP_LINK_BW_1_62;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		return DP_LINK_BW_2_7;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		return DP_LINK_BW_5_4;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		return DP_LINK_BW_8_1;
	default:
		WARN(1, "Invalid rate 0x%x\n", rate);
		return -EINVAL;
	}
}

int dptx_bw_to_phy_rate(unsigned int bw)
{
	switch (bw) {
	case DP_LINK_BW_1_62:
		return DPTX_PHYIF_CTRL_RATE_RBR;
	case DP_LINK_BW_2_7:
		return DPTX_PHYIF_CTRL_RATE_HBR;
	case DP_LINK_BW_5_4:
		return DPTX_PHYIF_CTRL_RATE_HBR2;
	case DP_LINK_BW_8_1:
		return DPTX_PHYIF_CTRL_RATE_HBR3;
	default:
		WARN(1, "Invalid bw 0x%x\n", bw);
		return -EINVAL;
	}
}

int dptx_lanes_to_dpcd_lanes(unsigned int lanes, bool enhance_frame_en)
{
	if (enhance_frame_en) {
		switch (lanes) {
		case 1:
			return 0x81;
		case 2:
			return 0x82;
		case 4:
			return 0x84;
		default:
			WARN(1, "Invalid lanes 0x%x\n", lanes);
			return 0;
		}
	} else {
		switch (lanes) {
		case 1:
			return 0x1;
		case 2:
			return 0x2;
		case 4:
			return 0x4;
		default:
			WARN(1, "Invalid lanes 0x%x\n", lanes);
			return 0;
		}
	}
}