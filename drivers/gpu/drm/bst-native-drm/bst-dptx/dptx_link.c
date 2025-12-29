// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"

static int dptx_link_read_status(struct dptx *dptx)
{
	return dptx_read_bytes_from_dpcd(dptx, DP_LANE0_1_STATUS,
					 dptx->link.status,
					 DP_LINK_STATUS_SIZE);
}

static int dptx_link_check_cr_done(struct dptx *dptx, bool *out_done)
{
	int retval;
	u8 byte;
	u32 reg;

	if (WARN_ON(!out_done))
		return -EINVAL;

	*out_done = false;

	retval = dptx_read_dpcd(dptx, DP_TRAINING_AUX_RD_INTERVAL, &byte);
	if (retval)
		return retval;

	reg = min_t(u32, (byte & 0x7f), 4);
	reg *= 4000;
	if (!reg)
		reg = 400;

	usleep_range(reg, reg + 10);

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	*out_done = drm_dp_clock_recovery_ok(dptx->link.status,
					     dptx->link.lanes);

	dptx_dbg(dptx, "%s: CR_DONE = %d\n", __func__, *out_done);

	return 0;
}

static int dptx_link_check_ch_eq_done(struct dptx *dptx,
				      bool *out_cr_done,
				      bool *out_ch_eq_done)
{
	int retval;
	bool done;

	if (WARN_ON(!out_cr_done || !out_ch_eq_done))
		return -EINVAL;

	retval = dptx_link_check_cr_done(dptx, &done);
	if (retval)
		return retval;

	*out_cr_done = false;
	*out_ch_eq_done = false;

	if (!done)
		return 0;

	*out_cr_done = true;
	*out_ch_eq_done = drm_dp_channel_eq_ok(dptx->link.status,
					       dptx->link.lanes);

	dptx_dbg(dptx, "%s: CH_EQ_DONE = %d\n", __func__, *out_ch_eq_done);

	return 0;
}

void dptx_link_set_preemp_vswing(struct dptx *dptx)
{
	unsigned int i;

	for (i = 0; i < dptx->link.lanes; i++) {
		u8 pe;
		u8 vs;
#if defined(CONFIG_C1200_SLT)
		dptx->link.preemp_level[i] = 2;
		dptx->link.vswing_level[i] = 1;
#elif defined(CONFIG_C1200_MASS)
		dptx->link.preemp_level[i] = 1;
		dptx->link.vswing_level[i] = 2;
#endif
		pe = dptx->link.preemp_level[i];
		vs = dptx->link.vswing_level[i];

		dptx_phy_set_pre_emphasis(dptx, i, pe);
		dptx_phy_set_vswing(dptx, i, vs);
	}
}

int dptx_link_training_lanes_set(struct dptx *dptx)
{
	int retval;
	unsigned int i;
	u8 bytes[4] = { 0xff, 0xff, 0xff, 0xff };

	for (i = 0; i < dptx->link.lanes; i++) {
		u8 byte = 0;

		byte |= ((dptx->link.vswing_level[i] <<
			  DP_TRAIN_VOLTAGE_SWING_SHIFT) &
			 DP_TRAIN_VOLTAGE_SWING_MASK);

		if (dptx->link.vswing_level[i] == 3)
			byte |= DP_TRAIN_MAX_SWING_REACHED;

		byte |= ((dptx->link.preemp_level[i] <<
			  DP_TRAIN_PRE_EMPHASIS_SHIFT) &
			 DP_TRAIN_PRE_EMPHASIS_MASK);

		if (dptx->link.preemp_level[i] == 2)
			byte |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		bytes[i] = byte;
	}

	retval = dptx_write_bytes_to_dpcd(dptx, DP_TRAINING_LANE0_SET, bytes,
					  dptx->link.lanes);
	if (retval)
		return retval;

	return 0;
}

int dptx_link_adjust_drive_settings(struct dptx *dptx, int *out_changed)
{
	int retval;
	unsigned int lanes;
	unsigned int i;
	u8 byte;
	u8 adj[4] = { 0, };
	int changed = false;

	lanes = dptx->link.lanes;

	switch (lanes) {
	case 4:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE2_3, &byte);
		if (retval)
			return retval;

		adj[2] = byte & 0x0f;
		adj[3] = (byte & 0xf0) >> 4;
		fallthrough;
	case 2:
	case 1:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &byte);
		if (retval)
			return retval;

		adj[0] = byte & 0x0f;
		adj[1] = (byte & 0xf0) >> 4;
		break;
	default:
		WARN(1, "Invalid number of lanes %d\n", lanes);
		return -EINVAL;
	}

	/* Save the drive settings */
	for (i = 0; i < lanes; i++) {
		u8 vs = adj[i] & 0x3;
		u8 pe = (adj[i] & 0xc) >> 2;

		if (dptx->link.vswing_level[i] != vs)
			changed = true;

		dptx->link.vswing_level[i] = vs;
		dptx->link.preemp_level[i] = pe;
	}

	dptx_link_set_preemp_vswing(dptx);

	retval = dptx_link_training_lanes_set(dptx);
	if (retval)
		return retval;

	if (out_changed)
		*out_changed = changed;

	return 0;
}

int dptx_set_link_configs(struct dptx *dptx, u8 rate, u8 lanes)
{
	u8 sink_max_rate;
	u8 sink_max_lanes;

	if (WARN(rate > DPTX_PHYIF_CTRL_RATE_HBR3,
		 "Invalid rate %d\n", rate))
		rate = DPTX_PHYIF_CTRL_RATE_RBR;

	if (WARN(!lanes || lanes == 3 || lanes > 4,
		 "Invalid lanes %d\n", lanes))
		lanes = 1;

	memset(dptx->link.preemp_level, 0, sizeof(u8) * 4);
	memset(dptx->link.vswing_level, 0, sizeof(u8) * 4);
	memset(dptx->link.status, 0, DP_LINK_STATUS_SIZE);

	sink_max_lanes = drm_dp_max_lane_count(dptx->rx_caps);
	if (lanes > sink_max_lanes)
		lanes = sink_max_lanes;

	sink_max_rate = dptx->rx_caps[DP_MAX_LINK_RATE];
	sink_max_rate = dptx_bw_to_phy_rate(sink_max_rate);

	if (rate > sink_max_rate)
		rate = sink_max_rate;

	dptx->link.lanes = lanes;
	dptx->link.rate = rate;
	dptx->link.trained = false;
	dev_info(dptx->dev, "%s:%d lanes:%d rate:%d\n", __func__, __LINE__, lanes, rate);

	return 0;
}

int dptx_xmit_enable(struct dptx *dptx, bool enable)
{
	uint32_t lanes = dptx->link.lanes, xmit_enable = 0;

	if (enable) {
		switch (lanes) {
		case 1:  xmit_enable = 0x1; break;
		case 2:  xmit_enable = 0x3; break;
		case 4:  xmit_enable = 0xf; break;
		default:
			dptx_err(dptx, "not support lane num=%d\n", lanes);
			return -1;
		}
	}
	dptx_write_regfield(dptx, dptx->field_xmit_enable, xmit_enable);

	return 0;
}

int dptx_link_training_pattern_set(struct dptx *dptx, u8 pattern)
{
	int retval;

	retval = dptx_write_dpcd(dptx, DP_TRAINING_PATTERN_SET, pattern);
	if (retval)
		return retval;

	return 0;
}

static int dptx_link_training_start(struct dptx *dptx)
{
	int retval;
	u8 byte;
	u8 training_set_bytes[5] = { 0x21, 0x00, 0x00, 0x00, 0x00 };

	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 3);
	retval = dptx_phy_wait_busy(dptx, dptx->max_lanes);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	dptx_phy_set_lanes(dptx, dptx->link.lanes);
	dptx_phy_set_rate(dptx, dptx->link.rate);
	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 0);
	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);

	dptx_link_set_preemp_vswing(dptx);

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_1);

	dptx_phy_enable_xmit(dptx, dptx->link.lanes, true);

	dptx_write_regfield(dptx, dptx->field_enable_mst_mode, dptx->mst);

	retval = dptx_phy_rate_to_bw(dptx->link.rate);
	if (retval < 0)
		return retval;

	byte = retval;
	retval = dptx_write_dpcd(dptx, DP_LINK_BW_SET, byte);
	if (retval)
		return retval;

	retval = dptx_lanes_to_dpcd_lanes(dptx->link.lanes,
						dptx->enhance_frame_en);
	if (retval == 0)
		return -1;

	byte = retval;
	retval = dptx_write_dpcd(dptx, DP_LANE_COUNT_SET, byte);
	if (retval)
		return retval;

	if (dptx->ssc_en && dptx_sink_enabled_ssc(dptx))
		byte = DP_SPREAD_AMP_0_5;
	else
		byte = 0;

	retval = dptx_write_dpcd(dptx, DP_DOWNSPREAD_CTRL, byte);
	if (retval)
		return retval;

	byte = 1;
	retval = dptx_write_dpcd(dptx, DP_MAIN_LINK_CHANNEL_CODING_SET, byte);
	if (retval)
		return retval;

	retval = dptx_write_bytes_to_dpcd(dptx, DP_TRAINING_PATTERN_SET,
					  training_set_bytes, 5);

	return 0;
}

int dptx_link_wait_cr_and_adjust(struct dptx *dptx, bool ch_eq)
{
	int i;
	int retval;
	int changed = 0;
	bool done = false;

	retval = dptx_link_check_cr_done(dptx, &done);
	if (retval)
		return retval;

	if (done)
		return 0;

	for (i = 0; i < 5; i++) {
		retval = dptx_link_adjust_drive_settings(dptx, &changed);
		if (retval)
			return retval;

		if (changed)
			i = 0;

		retval = dptx_link_check_cr_done(dptx, &done);
		if (retval)
			return retval;

		if (done)
			return 0;

		if (dptx->link.vswing_level[0] == 3)
			return -EPROTO;
	}

	return -EPROTO;
}

int dptx_link_cr(struct dptx *dptx)
{
	return dptx_link_wait_cr_and_adjust(dptx, false);
}

int dptx_link_ch_eq(struct dptx *dptx)
{
	int retval;
	bool cr_done;
	bool ch_eq_done;
	unsigned int pattern;
	unsigned int i;
	u8 dp_pattern;

	switch (dptx->max_rate) {
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		if (drm_dp_tps4_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_4;
			dp_pattern = DP_TRAINING_PATTERN_4;
			break;
		}
		fallthrough;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		if (drm_dp_tps3_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_3;
			dp_pattern = DP_TRAINING_PATTERN_3;
			break;
		}
		fallthrough;
	case DPTX_PHYIF_CTRL_RATE_RBR:
	case DPTX_PHYIF_CTRL_RATE_HBR:
		pattern = DPTX_PHYIF_CTRL_TPS_2;
		dp_pattern = DP_TRAINING_PATTERN_2;
		break;
	default:
		WARN(1, "Invalid rate %d\n", dptx->link.rate);
		return -EINVAL;
	}

	dptx_phy_set_pattern(dptx, pattern);
	if (dp_pattern != DP_TRAINING_PATTERN_4) {
		retval = dptx_link_training_pattern_set(dptx, dp_pattern | 0x20);
	} else {
		retval = dptx_link_training_pattern_set(dptx, dp_pattern);

		dptx_dbg(dptx, "%s:  Enabling scrambling for TPS4\n",
		 __func__);
		dptx_write_regfield(dptx, dptx->field_scramble_dis, 0);
	}

	if (retval)
		return retval;

	for (i = 0; i < 5; i++) {
		retval = dptx_link_check_ch_eq_done(dptx, &cr_done, &ch_eq_done);

		if (retval)
			return retval;

		dptx->cr_fail = false;

		if (!cr_done) {
			dptx->cr_fail = true;
			return -EPROTO;
		}

		if (ch_eq_done)
			return 0;

		retval = dptx_link_adjust_drive_settings(dptx, NULL);
		if (retval)
			return retval;
	}

	return -EPROTO;
}

int dptx_link_reduce_rate(struct dptx *dptx)
{
	unsigned int rate = dptx->link.rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		return -EPROTO;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		rate = DPTX_PHYIF_CTRL_RATE_RBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		rate = DPTX_PHYIF_CTRL_RATE_HBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		rate = DPTX_PHYIF_CTRL_RATE_HBR2;
		break;
	}

	dptx_dbg(dptx, "%s: Reducing rate from %d to %d\n",
		__func__, dptx->link.rate, rate);
	dptx->link.rate = rate;
	return 0;
}

int dptx_link_reduce_lanes(struct dptx *dptx)
{
	unsigned int lanes;

	switch (dptx->link.lanes) {
	case 4:
		lanes = 2;
		break;
	case 2:
		lanes = 1;
		break;
	case 1:
	default:
		return -EPROTO;
	}

	dptx_dbg(dptx, "%s: Reducing lanes from %d to %d\n",
		 __func__, dptx->link.lanes, lanes);
	dptx->link.lanes = lanes;
	dptx->link.rate  = dptx->max_rate;
	return 0;
}

int dptx_link_training(struct dptx *dptx)
{
	int retval, retval1;
	u8 byte;
	u32 hpd_sts;

again:
	dptx_info(dptx, "%s: >>>>>>>>>>>>>>>>> Starting link training\n", __func__);
	retval = dptx_link_training_start(dptx);
	if (retval)
		goto fail;

	retval = dptx_link_cr(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (dptx_link_reduce_rate(dptx)) {
				if (dptx_link_reduce_lanes(dptx)) {
					retval = -EPROTO;
					goto fail;
				} else {
					if (!(dptx->link.status[0] & 1))
						goto fail;
				}
			}

			dptx_set_link_configs(dptx,
						dptx->link.rate,
						dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}

	}

	dptx_info(dptx, "%s: link training CR done\n", __func__);
	retval = dptx_link_ch_eq(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (!dptx->cr_fail) {
				dptx_err(dptx, "Link training failure %0x, %0x", retval, -EPROTO);
				if (dptx->link.lanes == 1) {
					if (dptx_link_reduce_rate(dptx))
						goto fail;
					dptx->link.lanes = dptx->max_lanes;
				} else {
					dptx_link_reduce_lanes(dptx);
				}
			} else {
				if (dptx_link_reduce_rate(dptx)) {
					if (dptx_link_reduce_lanes(dptx)) {
						retval = -EPROTO;
						goto fail;
					}
				}
			}

			dptx_set_link_configs(dptx, dptx->link.rate, dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}
	}
	dptx_info(dptx, "%s: link training CH_EQ done\n", __func__);

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);

	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_DISABLE);
	if (retval)
		goto fail;

	dptx_info(dptx, "%s: Starting video stream\n", __func__);
	dptx_enable_default_video_stream(dptx, 0);
	dptx_phy_enable_xmit(dptx, dptx->link.lanes, true);
	dptx->link.trained = true;

	retval = dptx_read_dpcd(dptx, DP_SINK_COUNT, &byte);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, 0x2002, &byte);
	if (retval)
		return retval;

	dptx_video_ts_change(dptx, 0);
	dptx_info(dptx, "<<<<<<<<<<<<<<< Link training succeeded rate=%d lanes=%d\n",
		 dptx->link.rate, dptx->link.lanes);

	return 0;

fail:
	hpd_sts = dptx_read_regfield(dptx, dptx->field_hpd_status);
	if (hpd_sts) {
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);
		retval1 = dptx_link_training_pattern_set(dptx, DP_TRAINING_PATTERN_DISABLE);
		if (retval1)
			return retval1;

		dptx_err(dptx, "Link training failed %d\n", retval);

	} else {
		dptx_err(dptx, "Link training failed  as sink is disconnected %d\n", retval);
	}

	return retval;
}

int dptx_fast_link_training(struct dptx *dptx)
{
	int nr_lanes;
	int link_rate;
	int count;

	nr_lanes = dptx->max_lanes;
	link_rate = dptx->max_rate;
	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 0);
	dptx_write_regfield(dptx, dptx->field_phyrate, link_rate);

	switch (nr_lanes) {
	case (1):
		dptx_write_regfield(dptx, dptx->field_phy_lanes, 0);
		break;
	case (2):
		dptx_write_regfield(dptx, dptx->field_phy_lanes, 2);
		break;
	case (4):
		dptx_write_regfield(dptx, dptx->field_phy_lanes, 4);
		break;
	default:
		dptx_write_regfield(dptx, dptx->field_phy_lanes, 0);
	}

	count = 0;
	while (dptx_read_regfield(dptx, dptx->field_phy_busy)) {
		count++;
		if (count > 1000)
			return -EBUSY;
		msleep(20);
	}

	dptx_link_set_preemp_vswing(dptx);
	dptx_phy_set_pattern(dptx, 1);
	dptx_phy_enable_xmit(dptx, nr_lanes, true);

	usleep_range(500, 510);

	switch (link_rate) {
	case (DPTX_PHYIF_CTRL_RATE_HBR):
		dptx_phy_set_pattern(dptx, 2);
		break;
	case (DPTX_PHYIF_CTRL_RATE_HBR2):
		dptx_phy_set_pattern(dptx, 3);
		break;
	case (DPTX_PHYIF_CTRL_RATE_HBR3):
		dptx_phy_set_pattern(dptx, 4);
		break;
	default:
		dptx_phy_set_pattern(dptx, 2);
		break;
	}

	usleep_range(500, 510);

	dptx_phy_set_pattern(dptx, 0);

	return 0;
}

int dptx_link_check_status(struct dptx *dptx)
{
	int retval;
	u8 byte;
	u8 bytes[2];

	retval = dptx_read_bytes_from_dpcd(dptx, DP_SINK_COUNT, bytes, 2);
	if (retval)
		return retval;

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	byte = dptx->link.status[DP_LANE_ALIGN_STATUS_UPDATED -
				 DP_LANE0_1_STATUS];

	if (!(byte & DP_LINK_STATUS_UPDATED))
		return 0;

	if (dptx->link.trained &&
	    (!drm_dp_channel_eq_ok(dptx->link.status, dptx->link.lanes) ||
	     !drm_dp_clock_recovery_ok(dptx->link.status, dptx->link.lanes))) {
		dptx_dbg(dptx, "%s: Retraining link\n", __func__);
		dptx_set_link_configs(dptx, DPTX_MAX_LINK_RATE, DPTX_MAX_LINK_LANES);

		return dptx_link_training(dptx);
	}

	return 0;
}

int dptx_disconnect_link(struct dptx *dptx)
{
	dev_dbg(dptx->dev, ">>>>>>>>>>> Disabling Forward Error Correction\n");
	dptx_write_regfield(dptx, dptx->field_xmit_enable, 0);
	dev_dbg(dptx->dev, "<<<<<<<<<<<<<<<<<<<<<<<<<\n");
	return 0;
}
