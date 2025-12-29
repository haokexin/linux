// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "api/api.h"
#include "dptx_csr.h"
#include "dptx_utils.h"

static int handle_test_link_training(struct dptx *dptx)
{
	int retval;
	u8 lanes;
	u8 rate;
	struct video_params *vparams;
	struct dtd *mdtd;

	dptx_enable_ssc(dptx);
	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 0);

	retval = dptx_read_dpcd(dptx, DP_TEST_LINK_RATE, &rate);
	if (retval)
		return retval;

	retval = dptx_bw_to_phy_rate(rate);
	if (retval < 0)
		return retval;

	rate = retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_LANE_COUNT, &lanes);
	if (retval)
		return retval;

	dptx_dbg(dptx, "%s: Strating link training rate=%d, lanes=%d\n",
		 __func__, rate, lanes);

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;

	retval = dptx_video_ts_calculate(dptx, lanes, rate, vparams->bpc,
					 vparams->pix_enc, mdtd->pixel_clock);
	if (retval)
		return retval;

	retval = dptx_set_link_configs(dptx, rate, lanes);
	retval = dptx_link_training(dptx);
	if (retval)
		dptx_err(dptx, "Link training failed %d\n", retval);
	else
		dptx_info(dptx, "Link training succeeded\n");

	return retval;
}

static __maybe_unused int handle_test_link_video_timming(struct dptx *dptx,
							 int stream)
{
	int retval, i;
	u8 test_h_total_lsb, test_h_total_msb, test_v_total_lsb,
		test_v_total_msb, test_h_start_lsb, test_h_start_msb,
		test_v_start_lsb, test_v_start_msb, test_hsync_width_lsb,
		test_hsync_width_msb, test_vsync_width_lsb,
		test_vsync_width_msb, test_h_width_lsb, test_h_width_msb,
		test_v_width_lsb, test_v_width_msb;
	u32 h_total, v_total, h_start, v_start, h_width, v_width, hsync_width,
		vsync_width, h_sync_pol, v_sync_pol, refresh_rate;
	enum video_format_type video_format;
	u8 vmode;
	u8 test_refresh_rate;
	struct video_params *vparams;
	struct dtd mdtd;

	vparams = &dptx->vparams;
	retval = 0;
	h_total = 0;
	v_total = 0;
	h_start = 0;
	v_start = 0;
	v_width = 0;
	h_width = 0;
	hsync_width = 0;
	vsync_width = 0;
	h_sync_pol = 0;
	v_sync_pol = 0;
	test_refresh_rate = 0;
	i = 0;

	retval = dptx_read_dpcd(dptx, DP_TEST_H_TOTAL_LSB, &test_h_total_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_H_TOTAL_MSB, &test_h_total_msb);
	if (retval)
		return retval;
	h_total |= test_h_total_lsb;
	h_total |= test_h_total_msb << 8;
	dptx_dbg(dptx, "h_total = %d\n", h_total);

	retval = dptx_read_dpcd(dptx, DP_TEST_V_TOTAL_LSB, &test_v_total_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_V_TOTAL_MSB, &test_v_total_msb);
	if (retval)
		return retval;
	v_total |= test_v_total_lsb;
	v_total |= test_v_total_msb << 8;
	dptx_dbg(dptx, "v_total = %d\n", v_total);

	retval = dptx_read_dpcd(dptx, DP_TEST_H_START_LSB, &test_h_start_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_H_START_MSB, &test_h_start_msb);
	if (retval)
		return retval;
	h_start |= test_h_start_lsb;
	h_start |= test_h_start_msb << 8;
	dptx_dbg(dptx, "h_start = %d\n", h_start);

	retval = dptx_read_dpcd(dptx, DP_TEST_V_START_LSB, &test_v_start_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_V_START_MSB, &test_v_start_msb);
	if (retval)
		return retval;
	v_start |= test_v_start_lsb;
	v_start |= test_v_start_msb << 8;
	dptx_dbg(dptx, "v_start = %d\n", v_start);

	retval = dptx_read_dpcd(dptx, DP_TEST_H_SYNC_WIDTH_LSB,
				&test_hsync_width_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_H_SYNC_WIDTH_MSB,
				&test_hsync_width_msb);
	if (retval)
		return retval;
	hsync_width |= test_hsync_width_lsb;
	hsync_width |= (test_hsync_width_msb & (~(1 << 7))) << 8;
	h_sync_pol = (test_hsync_width_msb & (1 << 7)) >> 7;
	dptx_dbg(dptx, "hsync_width = %d\n", hsync_width);
	dptx_dbg(dptx, "h_sync_pol = %d\n", h_sync_pol);

	retval = dptx_read_dpcd(dptx, DP_TEST_V_SYNC_WIDTH_LSB,
				&test_vsync_width_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_V_SYNC_WIDTH_MSB,
				&test_vsync_width_msb);
	if (retval)
		return retval;
	vsync_width |= test_vsync_width_lsb;
	vsync_width |= (test_vsync_width_msb & (~(1 << 7))) << 8;
	v_sync_pol = (test_vsync_width_msb & (1 << 7)) >> 7;
	dptx_dbg(dptx, "vsync_width = %d\n", vsync_width);
	dptx_dbg(dptx, "v_sync_pol = %d\n", v_sync_pol);

	retval = dptx_read_dpcd(dptx, DP_TEST_H_WIDTH_LSB, &test_h_width_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_H_WIDTH_MSB, &test_h_width_msb);
	if (retval)
		return retval;
	h_width |= test_h_width_lsb;
	h_width |= test_h_width_msb << 8;
	dptx_dbg(dptx, "h_width = %d\n", h_width);

	retval = dptx_read_dpcd(dptx, DP_TEST_V_WIDTH_LSB, &test_v_width_lsb);
	if (retval)
		return retval;
	retval = dptx_read_dpcd(dptx, DP_TEST_V_WIDTH_MSB, &test_v_width_msb);
	if (retval)
		return retval;
	v_width |= test_v_width_lsb;
	v_width |= test_v_width_msb << 8;
	dptx_dbg(dptx, "v_width = %d\n", v_width);

	retval = dptx_read_dpcd(dptx, 0x234, &test_refresh_rate);
	if (retval)
		return retval;
	dptx_dbg(dptx, "test_refresh_rate = %d\n", test_refresh_rate);

	video_format = DMT;
	refresh_rate = test_refresh_rate * 1000;

	if (h_total == 1056 && v_total == 628 && h_start == 216 &&
	    v_start == 27 && hsync_width == 128 && vsync_width == 4 &&
	    h_width == 800 && v_width == 600) {
		vmode = 9;
	} else if (h_total == 1088 && v_total == 517 && h_start == 224 &&
		   v_start == 31 && hsync_width == 112 && vsync_width == 8 &&
		   h_width == 848 && v_width == 480) {
		vmode = 14;
	} else if (h_total == 1344 && v_total == 806 && h_start == 296 &&
		   v_start == 35 && hsync_width == 136 && vsync_width == 6 &&
		   h_width == 1024 && v_width == 768) {
		vmode = 16;
	} else if (h_total == 1440 && v_total == 790 && h_start == 112 &&
		   v_start == 19 && hsync_width == 32 && vsync_width == 7 &&
		   h_width == 1280 && v_width == 768) {
		vmode = 22;
	} else if (h_total == 1664 && v_total == 798 && h_start == 320 &&
		   v_start == 27 && hsync_width == 128 && vsync_width == 7 &&
		   h_width == 1280 && v_width == 768) {
		vmode = 23;
	} else if (h_total == 1440 && v_total == 823 && h_start == 112 &&
		   v_start == 20 && hsync_width == 32 && vsync_width == 6 &&
		   h_width == 1280 && v_width == 800) {
		vmode = 27;
	} else if (h_total == 1800 && v_total == 1000 && h_start == 424 &&
		   v_start == 39 && hsync_width == 112 && vsync_width == 3 &&
		   h_width == 1280 && v_width == 960) {
		vmode = 32;
	} else if (h_total == 1688 && v_total == 1066 && h_start == 360 &&
		   v_start == 41 && hsync_width == 112 && vsync_width == 3 &&
		   h_width == 1280 && v_width == 1024) {
		vmode = 35;
	} else if (h_total == 1792 && v_total == 795 && h_start == 368 &&
		   v_start == 24 && hsync_width == 112 && vsync_width == 6 &&
		   h_width == 1360 && v_width == 768) {
		vmode = 39;
	} else if (h_total == 1560 && v_total == 1080 && h_start == 112 &&
		   v_start == 27 && hsync_width == 32 && vsync_width == 4 &&
		   h_width == 1400 && v_width == 1050) {
		vmode = 41;
	} else if (h_total == 2160 && v_total == 1250 && h_start == 496 &&
		   v_start == 49 && hsync_width == 192 && vsync_width == 3 &&
		   h_width == 1600 && v_width == 1200) {
		vmode = 51;
	} else if (h_total == 2448 && v_total == 1394 && h_start == 528 &&
		   v_start == 49 && hsync_width == 200 && vsync_width == 3 &&
		   h_width == 1792 && v_width == 1344) {
		vmode = 62;
	} else if (h_total == 2600 && v_total == 1500 && h_start == 552 &&
		   v_start == 59 && hsync_width == 208 && vsync_width == 3 &&
		   h_width == 1920 && v_width == 1440) {
		vmode = 73;
	} else if (h_total == 2200 && v_total == 1125 && h_start == 192 &&
		   v_start == 41 && hsync_width == 44 && vsync_width == 5 &&
		   h_width == 1920 && v_width == 1080) {
		if (refresh_rate == 120000) {
			vmode = 63;
			video_format = VCEA;
		} else {
			vmode = 82;
		}
	} else if (h_total == 800 && v_total == 525 && h_start == 144 &&
		   v_start == 35 && hsync_width == 96 && vsync_width == 2 &&
		   h_width == 640 && v_width == 480) {
		vmode = 1;
		video_format = VCEA;
	} else if (h_total == 1650 && v_total == 750 && h_start == 260 &&
		   v_start == 25 && hsync_width == 40 && vsync_width == 5 &&
		   h_width == 1280 && v_width == 720) {
		vmode = 4;
		video_format = VCEA;
	} else if (h_total == 1680 && v_total == 831 && h_start == 328 &&
		   v_start == 28 && hsync_width == 128 && vsync_width == 6 &&
		   h_width == 1280 && v_width == 800) {
		vmode = 28;
		video_format = CVT;
	} else if (h_total == 1760 && v_total == 1235 && h_start == 112 &&
		   v_start == 32 && hsync_width == 32 && vsync_width == 4 &&
		   h_width == 1600 && v_width == 1200) {
		vmode = 40;
		video_format = CVT;
	} else if (h_total == 2208 && v_total == 1580 && h_start == 112 &&
		   v_start == 41 && hsync_width == 32 && vsync_width == 4 &&
		   h_width == 2048 && v_width == 1536) {
		vmode = 41;
		video_format = CVT;
	} else {
		dptx_dbg(dptx, "Unknown video mode\n");
		return -EINVAL;
	}

	if (!dptx_dtd_fill(&mdtd, vmode, refresh_rate, video_format)) {
		dptx_dbg(dptx, "%s: Invalid video mode value %d\n", __func__,
			 vmode);
		retval = -EINVAL;
		goto fail;
	}
	vparams->mdtd = mdtd;
	vparams->refresh_rate = refresh_rate;
	retval = dptx_video_ts_calculate(dptx, dptx->link.lanes,
					 dptx->link.rate, vparams->bpc,
					 vparams->pix_enc, mdtd.pixel_clock);
	if (retval)
		return retval;

	dptx_video_reset(dptx, 0, stream);
	vparams->mode = vmode;
	vparams->video_format = video_format;
	dptx_video_timing_change(dptx, stream);
fail:
	return retval;
}

static int handle_test_link_audio_pattern(struct dptx *dptx)
{
	int retval;
	u8 test_audio_mode, test_audio_smaple_range, test_audio_ch_count,
		audio_ch_count, orig_sample_freq, sample_freq;
	u32 audio_clock_freq;
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	retval = dptx_read_dpcd(dptx, DP_TEST_AUDIO_MODE, &test_audio_mode);
	if (retval)
		return retval;

	dptx_dbg(dptx, "test_audio_mode = %d\n", test_audio_mode);

	test_audio_smaple_range = test_audio_mode &
				  DP_TEST_AUDIO_SAMPLING_RATE_MASK;
	test_audio_ch_count = (test_audio_mode & DP_TEST_AUDIO_CH_COUNT_MASK) >>
			      DP_TEST_AUDIO_CH_COUNT_SHIFT;

	switch (test_audio_ch_count) {
	case DP_TEST_AUDIO_CHANNEL1:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL1\n");
		audio_ch_count = 1;
		break;
	case DP_TEST_AUDIO_CHANNEL2:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL2\n");
		audio_ch_count = 2;
		break;
	case DP_TEST_AUDIO_CHANNEL3:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL3\n");
		audio_ch_count = 3;
		break;
	case DP_TEST_AUDIO_CHANNEL4:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL4\n");
		audio_ch_count = 4;
		break;
	case DP_TEST_AUDIO_CHANNEL5:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL5\n");
		audio_ch_count = 5;
		break;
	case DP_TEST_AUDIO_CHANNEL6:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL6\n");
		audio_ch_count = 6;
		break;
	case DP_TEST_AUDIO_CHANNEL7:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL7\n");
		audio_ch_count = 7;
		break;
	case DP_TEST_AUDIO_CHANNEL8:
		dptx_dbg(dptx, "DP_TEST_AUDIO_CHANNEL8\n");
		audio_ch_count = 8;
		break;
	default:
		dptx_dbg(dptx, "Invalid TEST_AUDIO_CHANNEL_COUNT\n");
		return -EINVAL;
	}
	dptx_dbg(dptx, "test_audio_ch_count = %d\n", audio_ch_count);
	aparams->num_channels = audio_ch_count;

	switch (test_audio_smaple_range) {
	case DP_TEST_AUDIO_SAMPLING_RATE_32:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_32\n");
		orig_sample_freq = 12;
		sample_freq = 3;
		audio_clock_freq = 320;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_44_1:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_44_1\n");
		orig_sample_freq = 15;
		sample_freq = 0;
		audio_clock_freq = 441;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_48:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_48\n");
		orig_sample_freq = 13;
		sample_freq = 2;
		audio_clock_freq = 480;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_88_2:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_88_2\n");
		orig_sample_freq = 7;
		sample_freq = 8;
		audio_clock_freq = 882;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_96:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_96\n");
		orig_sample_freq = 5;
		sample_freq = 10;
		audio_clock_freq = 960;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_176_4:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_176_4\n");
		orig_sample_freq = 3;
		sample_freq = 12;
		audio_clock_freq = 1764;
		break;
	case DP_TEST_AUDIO_SAMPLING_RATE_192:
		dptx_dbg(dptx, "DP_TEST_AUDIO_SAMPLING_RATE_192\n");
		orig_sample_freq = 1;
		sample_freq = 14;
		audio_clock_freq = 1920;
		break;
	default:
		dptx_dbg(dptx, "Invalid TEST_AUDIO_SAMPLING_RATE\n");
		return -EINVAL;
	}
	dptx_dbg(dptx, "sample_freq = %d\n", sample_freq);
	dptx_dbg(dptx, "orig_sample_freq = %d\n", orig_sample_freq);

	aparams->iec_samp_freq = sample_freq;
	aparams->iec_orig_samp_freq = orig_sample_freq;

	dptx_audio_num_ch_change(dptx);
	dptx_audio_samp_freq_config(dptx);
	dptx_audio_infoframe_sdp_send(dptx);

	return retval;
}

static int dptx_set_custom_pattern(struct dptx *dptx)
{
	int retval;
	u8 pattern0, pattern1, pattern2, pattern3, pattern4, pattern5, pattern6,
		pattern7, pattern8, pattern9;

	u32 custompat0;
	u32 custompat1;
	u32 custompat2;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_0, &pattern0);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_1, &pattern1);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_2, &pattern2);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_3, &pattern3);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_4, &pattern4);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_5, &pattern5);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_6, &pattern6);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_7, &pattern7);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_8, &pattern8);
	if (retval)
		return retval;

	retval =
		dptx_read_dpcd(dptx, DP_TEST_80BIT_CUSTOM_PATTERN_9, &pattern9);
	if (retval)
		return retval;

	custompat0 = ((((((pattern3 & (0xff >> 2)) << 8) | pattern2) << 8) |
		       pattern1)
		      << 8) |
		     pattern0;
	custompat1 =
		((((((((pattern7 & (0xf)) << 8) | pattern6) << 8) | pattern5)
		   << 8) |
		  pattern4)
		 << 2) |
		((pattern3 >> 6) & 0x3);
	custompat2 = (((pattern9 << 8) | pattern8) << 4) |
		     ((pattern7 >> 4) & 0xf);

	dptx_write_reg(dptx, dptx->regs[DPTX], CUSTOMPAT0, custompat0);
	dptx_write_reg(dptx, dptx->regs[DPTX], CUSTOMPAT1, custompat1);
	dptx_write_reg(dptx, dptx->regs[DPTX], CUSTOMPAT2, custompat2);

	return 0;
}

static int adjust_vswing_and_preemphasis(struct dptx *dptx)
{
	int retval;
	int i;
	u8 lane_01;
	u8 lane_23;

	retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &lane_01);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE2_3, &lane_23);
	if (retval)
		return retval;

	for (i = 0; i < dptx->link.lanes; i++) {
		u8 pe;
		u8 vs;

		switch (i) {
		case 0:
			pe = (lane_01 & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK) >>
			     DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
			vs = (lane_01 & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK) >>
			     DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
			break;
		case 1:
			pe = (lane_01 & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK) >>
			     DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
			vs = (lane_01 & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK) >>
			     DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
			break;
		case 2:
			pe = (lane_23 & DP_ADJUST_PRE_EMPHASIS_LANE0_MASK) >>
			     DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT;
			vs = (lane_23 & DP_ADJUST_VOLTAGE_SWING_LANE0_MASK) >>
			     DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT;
			break;
		case 3:
			pe = (lane_23 & DP_ADJUST_PRE_EMPHASIS_LANE1_MASK) >>
			     DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT;
			vs = (lane_23 & DP_ADJUST_VOLTAGE_SWING_LANE1_MASK) >>
			     DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT;
			break;
		default:
			break;
		}

		dptx_phy_set_pre_emphasis(dptx, i, pe);
		dptx_phy_set_vswing(dptx, i, vs);
	}

	return 0;
}

static int handle_test_phy_pattern(struct dptx *dptx)
{
	u8 pattern;
	int retval;

	retval = dptx_read_dpcd(dptx, DP_TEST_PHY_PATTERN, &pattern);
	if (retval)
		return retval;

	pattern &= DP_TEST_PHY_PATTERN_SEL_MASK;

	switch (pattern) {
	case DP_TEST_PHY_PATTERN_NONE:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "No test pattern selected\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);
		break;
	case DP_TEST_PHY_PATTERN_D10:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "D10.2 without scrambling test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_1);
		break;
	case DP_TEST_PHY_PATTERN_SEMC:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx,
			 "Symbol error measurement count test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_SYM_ERM);
		break;
	case DP_TEST_PHY_PATTERN_PRBS7:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "PRBS7 test phy pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_PRBS7);
		break;
	case DP_TEST_PHY_PATTERN_CUSTOM:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(
			dptx,
			"80-bit custom pattern transmitted test phy pattern\n");

		retval = dptx_set_custom_pattern(dptx);
		if (retval)
			return retval;
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CUSTOM80);
		break;
	case DP_TEST_PHY_PATTERN_CP2520_1:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "CP2520_1 - HBR2 Compliance EYE pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CP2520_1);
		break;
	case DP_TEST_PHY_PATTERN_CP2520_2:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "CP2520_2 - pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_CP2520_2);
		break;
	case DP_TEST_PHY_PATTERN_CP2520_3_TPS4:
		retval = adjust_vswing_and_preemphasis(dptx);
		if (retval)
			return retval;
		dptx_dbg(dptx, "DP_TEST_PHY_PATTERN_CP2520_3_TPS4 - pattern\n");
		dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_4);
		break;
	default:
		dptx_dbg(dptx, "Invalid TEST_PHY_PATTERN\n");
		return -EINVAL;
	}
	return retval;
}

static int handle_automated_test_request(struct dptx *dptx)
{
	int retval;
	u8 test;

	retval = dptx_read_dpcd(dptx, DP_TEST_REQUEST, &test);
	if (retval)
		return retval;

	if (test & DP_TEST_LINK_TRAINING) {
		dptx_dbg(dptx, "%s: DP_TEST_LINK_TRAINING\n", __func__);

		retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE, DP_TEST_ACK);
		if (retval)
			return retval;

		retval = handle_test_link_training(dptx);
		if (retval)
			return retval;
	}

	if (test & DP_TEST_LINK_AUDIO_PATTERN) {
		dptx_dbg(dptx, "%s:DP_TEST_LINK_AUDIO_PATTERN\n", __func__);

		retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE, DP_TEST_ACK);
		if (retval)
			return retval;

		retval = handle_test_link_audio_pattern(dptx);
		if (retval)
			return retval;
	}

	if (test & DP_TEST_LINK_EDID_READ) {
		/* Invalid, this should happen on HOTPLUG */
		dptx_dbg(dptx, "%s:DP_TEST_LINK_EDID_READ\n", __func__);
		return -ENOTSUPP;
	}
	if (test & DP_TEST_LINK_PHY_TEST_PATTERN) {
		dptx_dbg(dptx, "%s:DP_TEST_LINK_PHY_TEST_PATTERN\n", __func__);
		retval = handle_test_phy_pattern(dptx);
		if (retval)
			return retval;
	}
	return 0;
}

static int handle_sink_request(struct dptx *dptx)
{
	int retval;
	u8 vector;
	u32 reg;

	retval = dptx_link_check_status(dptx);
	if (retval)
		return retval;

	retval = dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);
	if (retval)
		return retval;

	dptx_dbg(dptx, "%s: IRQ_VECTOR: 0x%02x\n", __func__, vector);

	if (!vector)
		return 0;

	if (vector & DP_REMOTE_CONTROL_COMMAND_PENDING) {
		dptx_warn(
			dptx,
			"%s: DP_REMOTE_CONTROL_COMMAND_PENDING: Not yet implemented",
			__func__);
	}

	if (vector & DP_AUTOMATED_TEST_REQUEST) {
		dptx_dbg(dptx, "%s: DP_AUTOMATED_TEST_REQUEST", __func__);
		retval = handle_automated_test_request(dptx);
		if (retval) {
			dptx_err(dptx, "Automated test request failed\n");
			if (retval == -ENOTSUPP) {
				retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE,
							 DP_TEST_NAK);
				if (retval)
					return retval;
			}
		}
	}

	if (vector & DP_CP_IRQ) {
		dptx_warn(dptx, "%s: DP_CP_IRQ", __func__);
		retval = dptx_write_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR,
					 DP_CP_IRQ);

		dptx_write_regfield(dptx, dptx->field_cp_irq, 1);
		reg = dptx_read_reg(dptx, dptx->regs[DPTX], HDCPCFG);
		dptx_warn(dptx, "%s: DP_CP_IRQ1--- 0x%x", __func__, reg);

		dptx_write_regfield(dptx, dptx->field_cp_irq, 1);
		if (retval)
			return retval;
	}
	if (vector & DP_MCCS_IRQ) {
		dptx_warn(dptx, "%s: DP_MCCS_IRQ: Not yet implemented",
			  __func__);
		retval = -ENOTSUPP;
	}

	if (vector & DP_DOWN_REP_MSG_RDY) {
		dptx_warn(dptx, "%s: DP_DOWN_REP_MSG_RDY: Not yet implemented",
			  __func__);
		retval = -ENOTSUPP;
	}

	if (vector & DP_UP_REQ_MSG_RDY) {
		dptx_warn(dptx, "%s: DP_UP_REQ_MSG_RDY: Not yet implemented",
			  __func__);
		retval = -ENOTSUPP;
	}

	if (vector & DP_SINK_SPECIFIC_IRQ) {
		dptx_warn(dptx, "%s: DP_SINK_SPECIFIC_IRQ: Not yet implemented",
			  __func__);
		retval = -ENOTSUPP;
	}

	return retval;
}

static int handle_hotunplug(struct dptx *dptx)
{
	u8 retval;

	dptx->dummy_dtds_present = false;
	dev_dbg(dptx->dev, "Disabling Forward Error Correction\n");
	dptx_write_regfield(dptx, dptx->field_enable_fec, 0);
	msleep(100);
	dptx_write_regfield(dptx, dptx->field_xmit_enable, 0);
	dptx_write_regfield(dptx, dptx->field_phy_powerdown, 3);

	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		dptx_err(dptx, "Timed out waiting for PHY BUSY\n");
		return retval;
	}

	if (!dptx->link.bypass_training) {
		atomic_set(&dptx->sink_request, 0);
		dptx->link.trained = false;
	}
	return 0;
}

static int dptx_read_edid_block(struct dptx *dptx, unsigned int block)
{
	int retval;
	int retry = 0;
	int i;

	u8 offset = block * 128;
	u8 segment = block >> 1;

	dptx_dbg(dptx, "%s: block=%d\n", __func__, block);

again:
	retval = dptx_write_bytes_to_i2c(dptx, 0x30, &segment, 1);
	retval = dptx_write_bytes_to_i2c(dptx, 0x50, &offset, 1);

	retval = dptx_read_bytes_from_i2c(dptx, 0x50, &dptx->edid[block * 128],
					  128);
	if ((retval == -EINVAL) && !retry) { // retry if edid read failed
		retry = 1;
		goto again;
	}

	dptx_i2c_address_only(dptx, 0x50);

	if (retval == -EINVAL)
		for (i = 0; i < 128; i++)
			dptx->edid_second[i] = 0x00;
	else
		for (i = 0; i < 128; i++)
			dptx->edid_second[i] = dptx->edid[128 + i];

	print_buf(&dptx->edid[block * 128], 128);

	return 0;
}

int dptx_read_edid(struct dptx *dptx)
{
	int i;
	int retval = 0;
	unsigned int ext_blocks = 0;
	u8 *first_edid_block = NULL;

	memset(dptx->edid, 0, DPTX_DEFAULT_EDID_BUFLEN);
	retval = dptx_read_edid_block(dptx, 0);
	if (retval)
		goto fail;

	if (dptx->edid[0x7e] > 10) {
		ext_blocks = 2;
		dptx_dbg(dptx, "%s: harutk num_ext_blocks=%d\n", __func__,
			  dptx->edid[0x7e]);
	} else {
		ext_blocks = dptx->edid[0x7e];
		dptx_dbg(dptx, "%s: harutk num_ext_blocks=%d\n", __func__,
			  dptx->edid[0x7e]);
	}

	first_edid_block = kmalloc(128, GFP_KERNEL);
	memcpy(first_edid_block, dptx->edid, 128);
	kfree(dptx->edid);
	dptx->edid = kzalloc(128 * ext_blocks + 128, GFP_KERNEL);
	memcpy(dptx->edid, first_edid_block, 128);

	for (i = 1; i <= ext_blocks; i++) {
		retval = dptx_read_edid_block(dptx, i);
		if (retval)
			goto fail;
	}
fail:
	kfree(first_edid_block);
	return retval;
}

int dptx_check_edid(struct dptx *dptx)
{
	int i;
	u32 edid_sum = 0;

	for (i = 0; i < 128; i++)
		edid_sum += dptx->edid[i];
	if (edid_sum & 0xFF) {
		dptx_err(dptx, "Invalid EDID checksum\n");
		return -EINVAL;
	}
	return 0;
}

static u8 drm_dp_msg_header_crc4(const u8 *data, size_t num_nibbles)
{
	u8 bitmask = 0x80;
	u8 bitshift = 7;
	u8 array_index = 0;
	int number_of_bits = num_nibbles * 4;
	u8 remainder = 0;

	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		remainder |= (data[array_index] & bitmask) >> bitshift;
		bitmask >>= 1;
		bitshift--;
		if (bitmask == 0) {
			bitmask = 0x80;
			bitshift = 7;
			array_index++;
		}
		if ((remainder & 0x10) == 0x10)
			remainder ^= 0x13;
	}

	number_of_bits = 4;
	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		if ((remainder & 0x10) != 0)
			remainder ^= 0x13;
	}

	return remainder;
}

static u8 drm_dp_msg_data_crc4(const u8 *data, u8 number_of_bytes)
{
	u8 bitmask = 0x80;
	u8 bitshift = 7;
	u8 array_index = 0;
	int number_of_bits = number_of_bytes * 8;
	u16 remainder = 0;

	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		remainder |= (data[array_index] & bitmask) >> bitshift;
		bitmask >>= 1;
		bitshift--;
		if (bitmask == 0) {
			bitmask = 0x80;
			bitshift = 7;
			array_index++;
		}
		if ((remainder & 0x100) == 0x100)
			remainder ^= 0xd5;
	}

	number_of_bits = 8;
	while (number_of_bits != 0) {
		number_of_bits--;
		remainder <<= 1;
		if ((remainder & 0x100) != 0)
			remainder ^= 0xd5;
	}

	return remainder & 0xff;
}

static void drm_dp_encode_sideband_msg_hdr(struct drm_dp_sideband_msg_hdr *hdr,
					   u8 *buf, int *len)
{
	int idx = 0;
	int i;
	u8 crc4;

	buf[idx++] = ((hdr->lct & 0xf) << 4) | (hdr->lcr & 0xf);
	for (i = 0; i < (hdr->lct / 2); i++) {
		buf[idx++] = (hdr->rad[i]) << 4;
		pr_err("sahakyan: i = %d, idx = %d,  buf = %x, rad = %x\n", i,
		       idx, buf[idx], hdr->rad[i]);
	}
	buf[idx++] = (hdr->broadcast << 7) | (hdr->path_msg << 6) |
		     (hdr->msg_len & 0x3f);
	buf[idx++] = (hdr->somt << 7) | (hdr->eomt << 6) | (hdr->seqno << 4);

	crc4 = drm_dp_msg_header_crc4(buf, (idx * 2) - 1);
	buf[idx - 1] |= (crc4 & 0xf);
	*len = idx;
}

static void drm_dp_crc_sideband_chunk_req(u8 *msg, u8 len)
{
	u8 crc4;

	crc4 = drm_dp_msg_data_crc4(msg, len);
	msg[len] = crc4;
}

static bool drm_dp_decode_sideband_msg_hdr(struct drm_dp_sideband_msg_hdr *hdr,
					   u8 *buf, int buflen, u8 *hdrlen)
{
	u8 crc4;
	u8 len;
	int i;
	u8 idx;

	if (buf[0] == 0)
		return false;
	len = 3;
	len += ((buf[0] & 0xf0) >> 4) / 2;
	if (len > buflen)
		return false;
	crc4 = drm_dp_msg_header_crc4(buf, (len * 2) - 1);

	if ((crc4 & 0xf) != (buf[len - 1] & 0xf)) {
		return false;
	}

	hdr->lct = (buf[0] & 0xf0) >> 4;
	hdr->lcr = (buf[0] & 0xf);
	idx = 1;
	for (i = 0; i < (hdr->lct / 2); i++)
		hdr->rad[i] = buf[idx++];
	hdr->broadcast = (buf[idx] >> 7) & 0x1;
	hdr->path_msg = (buf[idx] >> 6) & 0x1;
	hdr->msg_len = buf[idx] & 0x3f;
	idx++;
	hdr->somt = (buf[idx] >> 7) & 0x1;
	hdr->eomt = (buf[idx] >> 6) & 0x1;
	hdr->seqno = (buf[idx] >> 4) & 0x1;
	idx++;
	*hdrlen = idx;
	return true;
}

static const char *
dptx_sideband_header_rad_string(struct drm_dp_sideband_msg_hdr *header)
{
	if (header->lct > 1)
		return "TODO";

	return "none";
}

static void dptx_print_sideband_header(struct dptx *dptx,
				       struct drm_dp_sideband_msg_hdr *header)
{
	dptx_dbg(dptx,
		 "SIDEBAND_MSG_HEADER: "
		 "lct=%d, lcr=%d, rad=%s, bcast=%d, "
		 "path=%d, msglen=%d, somt=%d, eomt=%d, seqno=%d\n",
		 header->lct, header->lcr,
		 dptx_sideband_header_rad_string(header), header->broadcast,
		 header->path_msg, header->msg_len, header->somt, header->eomt,
		 header->seqno);
}

static int dptx_wait_down_rep(struct dptx *dptx)
{
	int count = 0;
	u8 vector;

	while (1) {
		dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);

		if (vector & DP_DOWN_REP_MSG_RDY) {
			dptx_dbg(dptx, "%s: vector set\n", __func__);
			break;
		}

		count++;
		if (count > 2000) {
			dptx_dbg(dptx, "%s: Timed out\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(950, 1000);
	}

	return 0;
}

static int dptx_clear_down_rep(struct dptx *dptx)
{
	int count = 0;
	u8 vector;

	while (1) {
		dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);

		if (!(vector & DP_DOWN_REP_MSG_RDY)) {
			dptx_dbg(dptx, "%s: vector clear\n", __func__);
			break;
		}

		dptx_write_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR,
				DP_DOWN_REP_MSG_RDY);

		count++;
		if (count > 2000) {
			dptx_dbg(dptx, "%s: Timed out\n", __func__);
			return -ETIMEDOUT;
		}

		usleep_range(950, 1000);
	}

	return 0;
}

static int dptx_sideband_get_down_rep(struct dptx *dptx, u8 request_id,
				      u8 *msg_out)
{
	struct drm_dp_sideband_msg_hdr header;
	u8 buf[256];
	u8 header_len;
	int retval;
	int first = 1;
	u8 msg[1024];
	u8 msg_len;
	int retries = 0;

again:
	memset(msg, 0, 1024);
	msg_len = 0;

	while (1) {
		retval = dptx_wait_down_rep(dptx);
		if (retval) {
			dptx_err(dptx, "%s: Error waiting down rep (%d)\n",
				 __func__, retval);
			return retval;
		}

		retval = dptx_read_bytes_from_dpcd(
			dptx, DP_SIDEBAND_MSG_DOWN_REP_BASE, buf, 256);
		if (retval) {
			dptx_err(dptx, "%s: Error reading down rep (%d)\n",
				 __func__, retval);
			return retval;
		}
		if (!drm_dp_decode_sideband_msg_hdr(&header, buf, 256,
						    &header_len)) {
			dptx_err(dptx,
				 "%s: Error decoding sideband header (%d)\n",
				 __func__, retval);
			return -EINVAL;
		}

		dptx_print_sideband_header(dptx, &header);

		header.msg_len -= 1;
		memcpy(&msg[msg_len], &buf[header_len], header.msg_len);
		msg_len += header.msg_len;

		if (first && !header.somt) {
			dptx_err(dptx, "%s: SOMT not set\n", __func__);
			return -EINVAL;
		}
		first = 0;

		dptx_write_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR,
				DP_DOWN_REP_MSG_RDY);

		if (header.eomt)
			break;
	}

	print_buf(msg, msg_len);
	if ((msg[0] & 0x7f) != request_id) {
		if (retries < 3) {
			dptx_err(
				dptx,
				"%s: request_id %d does not match expected %d, retrying\n",
				__func__, msg[0] & 0x7f, request_id);
			retries++;
			goto again;
		} else {
			dptx_err(
				dptx,
				"%s: request_id %d does not match expected %d, giving up\n",
				__func__, msg[0] & 0x7f, request_id);
			return -EINVAL;
		}
	}

	retval = dptx_clear_down_rep(dptx);
	if (retval) {
		dptx_err(dptx, "%s: Error waiting down rep clear (%d)\n",
			 __func__, retval);
		return retval;
	}

	if (msg_out)
		memcpy(msg_out, msg, msg_len);

	return msg_len;
}

static __maybe_unused int dptx_aux_msg_clear_payload_id_table(struct dptx *dptx)
{
	struct drm_dp_sideband_msg_hdr header = {
		.lct = 1,
		.lcr = 6,
		.rad = { 0, },
		.broadcast = true,
		.path_msg = 1,
		.msg_len = 2,
		.somt = 1,
		.eomt = 1,
		.seqno = 0,
	};

	u8 buf[256];
	int len = 256;
	u8 *msg;

	drm_dp_encode_sideband_msg_hdr(&header, buf, &len);

	msg = &buf[len];
	msg[0] = DP_CLEAR_PAYLOAD_ID_TABLE;
	drm_dp_crc_sideband_chunk_req(msg, 1);

	len += 2;

	dptx_dbg(dptx, "%s: Sending DOWN_REQ\n", __func__);
	dptx_write_bytes_to_dpcd(dptx, DP_SIDEBAND_MSG_DOWN_REQ_BASE, buf, len);

	dptx_sideband_get_down_rep(dptx, DP_CLEAR_PAYLOAD_ID_TABLE, NULL);

	return 0;
}

static bool drm_dp_sideband_parse_link_address(
	struct drm_dp_sideband_msg_rx *raw,
	struct drm_dp_sideband_msg_reply_body *repmsg)
{
	int idx = 1;
	int i;

	memcpy(repmsg->u.link_addr.guid, &raw->msg[idx], 16);
	idx += 16;
	repmsg->u.link_addr.nports = raw->msg[idx] & 0xf;
	idx++;
	if (idx > raw->curlen)
		goto fail_len;
	for (i = 0; i < repmsg->u.link_addr.nports; i++) {
		if (raw->msg[idx] & 0x80)
			repmsg->u.link_addr.ports[i].input_port = 1;

		repmsg->u.link_addr.ports[i].peer_device_type =
			(raw->msg[idx] >> 4) & 0x7;
		repmsg->u.link_addr.ports[i].port_number =
			(raw->msg[idx] & 0xf);

		idx++;
		if (idx > raw->curlen)
			goto fail_len;
		repmsg->u.link_addr.ports[i].mcs = (raw->msg[idx] >> 7) & 0x1;
		repmsg->u.link_addr.ports[i].ddps = (raw->msg[idx] >> 6) & 0x1;
		if (repmsg->u.link_addr.ports[i].input_port == 0)
			repmsg->u.link_addr.ports[i].legacy_device_plug_status =
				(raw->msg[idx] >> 5) & 0x1;
		idx++;
		if (idx > raw->curlen)
			goto fail_len;
		if (repmsg->u.link_addr.ports[i].input_port == 0) {
			repmsg->u.link_addr.ports[i].dpcd_revision =
				(raw->msg[idx]);
			idx++;
			if (idx > raw->curlen)
				goto fail_len;
			memcpy(repmsg->u.link_addr.ports[i].peer_guid,
			       &raw->msg[idx], 16);
			idx += 16;
			if (idx > raw->curlen)
				goto fail_len;
			repmsg->u.link_addr.ports[i].num_sdp_streams =
				(raw->msg[idx] >> 4) & 0xf;
			repmsg->u.link_addr.ports[i].num_sdp_stream_sinks =
				(raw->msg[idx] & 0xf);
			idx++;
		}
		if (idx > raw->curlen)
			goto fail_len;
	}

	return true;
fail_len:
	pr_info("Link address reply parse length fail %d %d\n", idx,
		raw->curlen);
	return false;
}

static __maybe_unused int
dptx_aux_msg_link_address(struct dptx *dptx, struct drm_dp_sideband_msg_rx *raw,
			  struct drm_dp_sideband_msg_reply_body *rep, int port1)
{
	struct drm_dp_sideband_msg_hdr header;
	u8 buf[256];
	int len = 256;

	u8 *msg;

	memset(&header, 0, sizeof(struct drm_dp_sideband_msg_hdr));

	header.lct = 1;
	header.lcr = 0;
	header.rad[0] = 0;
	header.broadcast = false;
	header.path_msg = 0;
	header.msg_len = 2;
	header.somt = 1;
	header.eomt = 1;
	header.seqno = 0;

	if (port1 >= 0) {
		header.lct = 2;
		header.lcr = 1;
		header.rad[0] = port1;
		pr_err("sahakyan: link port1=%d", port1);
	}

	drm_dp_encode_sideband_msg_hdr(&header, buf, &len);

	msg = &buf[len];
	msg[0] = DP_LINK_ADDRESS;

	drm_dp_crc_sideband_chunk_req(msg, 1);

	len += 2;
	print_buf(buf, len);
	dptx_dbg(dptx, "%s: Sending DOWN_REQ\n", __func__);
	dptx_write_bytes_to_dpcd(dptx, DP_SIDEBAND_MSG_DOWN_REQ_BASE, buf, len);

	len = dptx_sideband_get_down_rep(dptx, DP_LINK_ADDRESS, raw->msg);
	raw->curlen = len;
	dptx_dbg(dptx, "%s: rawlen = %d\n", __func__, len);

	print_buf(raw->msg, len);
	drm_dp_sideband_parse_link_address(raw, rep);

	return 0;
}

static __maybe_unused int dptx_aux_msg_allocate_payload(struct dptx *dptx,
							u8 port, u8 vcpid,
							u16 pbn, int port1,
							u8 pdt)
{
	struct drm_dp_sideband_msg_hdr header;
	u8 buf[256];
	int len = 256;
	u8 *msg;
	int i;

	memset(&header, 0, sizeof(struct drm_dp_sideband_msg_hdr));

	header.lct = 1;
	header.lcr = 0;
	header.rad[0] = 0;
	header.broadcast = false;
	header.path_msg = 1;
	header.msg_len = 6;
	header.somt = 1;
	header.eomt = 1;
	header.seqno = 0;

	dptx_dbg(dptx, "%s: PBN=%d\n", __func__, pbn);
	if (port1 >= 0) {
		header.lct = 2;
		header.lcr = 1;
		header.rad[0] = port1;
		for (i = 0; i < 8; i++)
			pr_err("sahakyan: --------------------- header[%d].rad = %d",
			       i, header.rad[i]);
	}
	drm_dp_encode_sideband_msg_hdr(&header, buf, &len);

	msg = &buf[len];
	msg[0] = DP_ALLOCATE_PAYLOAD;
	msg[1] = ((port & 0xf) << 4);
	msg[2] = vcpid & 0x7f;
	msg[3] = pbn >> 8;
	msg[4] = pbn & 0xff;
	drm_dp_crc_sideband_chunk_req(msg, 5);

	len += 6;

	dptx_dbg(dptx, "%s: Sending DOWN_REQ\n", __func__);
	dptx_write_bytes_to_dpcd(dptx, DP_SIDEBAND_MSG_DOWN_REQ_BASE, buf, len);

	dptx_sideband_get_down_rep(dptx, DP_ALLOCATE_PAYLOAD, NULL);
	return 0;
}

static u32 __maybe_unused dptx_calc_num_slots(struct dptx *dptx, int pbn)
{
	int div;
	int dp_link_bw = dptx_phy_rate_to_bw(dptx->link.rate);
	int dp_link_count = dptx->link.lanes;

	switch (dp_link_bw) {
	case DP_LINK_BW_1_62:
		div = 3 * dp_link_count;
		break;
	case DP_LINK_BW_2_7:
		div = 5 * dp_link_count;
		break;
	case DP_LINK_BW_5_4:
		div = 10 * dp_link_count;
		break;
	case DP_LINK_BW_8_1:
		div = 15 * dp_link_count;
		break;
	default:
		return 0;
	}

	return DIV_ROUND_UP(pbn, div);
}

static void dptx_audio_sfreq_based_on_edid(struct dptx *dptx, int edid_index)
{
	u8 sample_freq;
	struct audio_short_desc *audio_desc;

	audio_desc = &dptx->audio_desc;

	sample_freq = dptx->edid_second[edid_index + 2] & GENMASK(6, 0);

	if (sample_freq & BIT(0)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 32khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_32;
	}

	if (sample_freq & BIT(1)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 44.1khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_44_1;
	}

	if (sample_freq & BIT(2)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 48khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_48;
	}

	if (sample_freq & BIT(3)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 88.2khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_88_2;
	}

	if (sample_freq & BIT(4)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 96khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_96;
	}

	if (sample_freq & BIT(5)) {
		dev_dbg(dptx->dev,
			"AUDIO EDID: Sink supports 176.4khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_176_4;
	}
	if (sample_freq & BIT(6)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 192khz audio\n");
		audio_desc->max_sampling_freq = SAMPLE_FREQ_192;
	}
}

static void dptx_audio_bps_based_on_edid(struct dptx *dptx, int edid_index)
{
	u8 bpsample;
	struct audio_short_desc *audio_desc;

	audio_desc = &dptx->audio_desc;
	bpsample = dptx->edid_second[edid_index + 3] & GENMASK(2, 0);

	if (bpsample & BIT(0)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 16 bit audio\n");
		audio_desc->max_bit_per_sample = 16;
	}

	if (bpsample & BIT(1)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 20 bit audio\n");
		audio_desc->max_bit_per_sample = 20;
	}

	if (bpsample & BIT(2)) {
		dev_dbg(dptx->dev, "AUDIO EDID: Sink supports 24 bit audio\n");
		audio_desc->max_bit_per_sample = 24;
	}
}

static void dptx_fill_audio_short_desc(struct dptx *dptx, int edid_index)
{
	struct audio_short_desc *audio_desc;
	u8 audio_data_size;

	audio_desc = &dptx->audio_desc;
	audio_data_size = (dptx->edid_second[edid_index] & EDID_SIZE_MASK) >>
			  EDID_SIZE_SHIFT;

	audio_desc->max_num_of_channels =
		(dptx->edid_second[edid_index + 1] & GENMASK(2, 0)) + 1;
	dev_dbg(dptx->dev, "AUDIO EDID: Sink supports up to %d channels\n",
		audio_desc->max_num_of_channels);

	dptx_audio_sfreq_based_on_edid(dptx, edid_index);
	dptx_audio_bps_based_on_edid(dptx, edid_index);
}

static void dptx_parse_established_timing(struct dptx *dptx)
{
	u8 byte1, byte2, byte3;

	byte1 = dptx->edid[35];
	byte2 = dptx->edid[36];
	byte3 = dptx->edid[37];

#ifndef PARSE_EST_TIMINGS_FROM_BYTE3

	if (byte1 & ET1_800x600_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET1_800x600_60hz\n");
		dptx->selected_est_timing = DMT_800x600_60hz;
		return;
	}

	if (byte1 & ET1_800x600_56hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_800x600_56hz, but we dont\n");

	if (byte1 & ET1_640x480_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_75hz, but we dont\n");

	if (byte1 & ET1_640x480_72hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_72hz, but we dont\n");

	if (byte1 & ET1_640x480_67hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_67hz, but we dont\n");

	if (byte1 & ET1_640x480_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET1_640x480_60hz\n");
		dptx->selected_est_timing = DMT_640x480_60hz;
		return;
	}

	if (byte1 & ET1_720x400_88hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_720x400_88hz, but we dont\n");

	if (byte1 & ET1_720x400_70hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_720x400_70hz, but we dont\n");

	if (byte2 & ET2_1280x1024_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1280x1024_75hz, but we dont\n");

	if (byte2 & ET2_1024x768_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_75hz, but we dont\n");

	if (byte2 & ET2_1024x768_70hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_70hz, but we dont\n");

	if (byte2 & ET2_1024x768_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET2_1024x768_60hz\n");
		dptx->selected_est_timing = DMT_1024x768_60hz;
		return;
	}

	if (byte2 & ET2_1024x768_87hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_87hz, but we dont\n");

	if (byte2 & ET2_832x624_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_832x624_75hz, but we dont\n");

	if (byte2 & ET2_800x600_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_800x600_75hz, but we dont\n");

	if (byte2 & ET2_800x600_72hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_800x600_72hz, but we dont\n");

	if (byte3 & ET3_1152x870_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET3_1152x870_75hz, but we dont\n");

#else

	if (byte3 & ET3_1152x870_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET3_1152x870_75hz, but we dont\n");

	if (byte2 & ET2_800x600_72hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_800x600_72hz, but we dont\n");

	if (byte2 & ET2_800x600_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_800x600_75hz, but we dont\n");

	if (byte2 & ET2_832x624_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_832x624_75hz, but we dont\n");

	if (byte2 & ET2_1024x768_87hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_87hz, but we dont\n");

	if (byte2 & ET2_1024x768_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET2_1024x768_60hz\n");
		dptx->selected_est_timing = DMT_1024x768_60hz;
		return;
	}

	if (byte2 & ET2_1024x768_70hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_70hz, but we dont\n");

	if (byte2 & ET2_1024x768_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1024x768_75hz, but we dont\n");

	if (byte2 & ET2_1280x1024_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET2_1280x1024_75hz, but we dont\n");

	if (byte1 & ET1_720x400_70hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_720x400_70hz, but we dont\n");

	if (byte1 & ET1_720x400_88hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_720x400_88hz, but we dont\n");

	if (byte1 & ET1_640x480_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET1_640x480_60hz\n");
		dptx->selected_est_timing = DMT_640x480_60hz;
		return;
	}

	if (byte1 & ET1_640x480_67hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_67hz, but we dont\n");

	if (byte1 & ET1_640x480_72hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_72hz, but we dont\n");

	if (byte1 & ET1_640x480_75hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_640x480_75hz, but we dont\n");

	if (byte1 & ET1_800x600_56hz)
		dev_dbg(dptx->dev,
			"Sink supports ET1_800x600_56hz, but we dont\n");

	if (byte1 & ET1_800x600_60hz) {
		dev_dbg(dptx->dev, "Sink supports ET1_800x600_60hz\n");
		dptx->selected_est_timing = DMT_800x600_60hz;
		return;
	}
#endif
}

static __maybe_unused void
dptx_check_detailed_timing_descriptors(struct dptx *dptx)
{
	dev_dbg(dptx->dev, "dptx->edid[54] = %d, dptx->edid[55] = %d,\n",
		dptx->edid[54], dptx->edid[55]);
	dev_dbg(dptx->dev, "dptx->edid[72] = %d, dptx->edid[73] = %d,\n",
		dptx->edid[72], dptx->edid[73]);
	if ((dptx->edid[54] == 0 && dptx->edid[55] == 0) &&
	    (dptx->edid[72] == 0 && dptx->edid[73] == 0)) {
		dev_err(dptx->dev, "%s FOUND EDID DUMMY BLOCKS\n", __func__);
		dev_err(dptx->dev, "%s: Going to parse established timings\n",
			__func__);
		dptx->dummy_dtds_present = true;
		dptx_parse_established_timing(dptx);
	} else {
		dev_err(dptx->dev,
			"%s: EDID Dummy blocks not found, continuing with usual way\n",
			__func__);
	}
}

static __maybe_unused void dptx_parse_edid_audio_data_block(struct dptx *dptx)
{
	u8 byte;
	u8 tag, size;
	u8 edid_block1[128];
	int i, index;

	for (i = 0; i < 128; i++)
		edid_block1[i] = dptx->edid_second[i];

	byte = edid_block1[4];
	index = 4;
	tag = (byte & EDID_TAG_MASK) >> EDID_TAG_SHIFT;
	size = (byte & EDID_SIZE_MASK) >> EDID_SIZE_SHIFT;

	/* find the audio tag  containing byte */
	while (tag != AUDIO_TAG) {
		size = (byte & EDID_SIZE_MASK) >> EDID_SIZE_SHIFT;
		index = index + size + 1;
		byte = dptx->edid_second[index];
		tag = (byte & EDID_TAG_MASK) >> EDID_TAG_SHIFT;
	}

	dptx_fill_audio_short_desc(dptx, index);
}

static __maybe_unused int dptx_config_audio_based_on_edid(struct dptx *dptx)
{
	int retval = 0;
	u8 audio_ch_count, orig_sample_freq, sample_freq, desc_audio_ch_count;
	u32 audio_clock_freq;
	struct audio_short_desc *audio_desc;
	enum audio_sample_freq audio_smaple_range;
	struct audio_params *aparams;

	audio_desc = &dptx->audio_desc;
	aparams = &dptx->aparams;

	audio_smaple_range = audio_desc->max_sampling_freq;
	desc_audio_ch_count = audio_desc->max_num_of_channels;

	switch (desc_audio_ch_count) {
	case 1:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL1\n");
		audio_ch_count = 1;
		break;
	case 2:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL2\n");
		audio_ch_count = 2;
		break;
	case 3:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL3\n");
		audio_ch_count = 3;
		break;
	case 4:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL4\n");
		audio_ch_count = 4;
		break;
	case 5:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL5\n");
		audio_ch_count = 5;
		break;
	case 6:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL6\n");
		audio_ch_count = 6;
		break;
	case 7:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL7\n");
		audio_ch_count = 7;
		break;
	case 8:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_CHANNEL8\n");
		audio_ch_count = 8;
		break;
	default:
		dptx_dbg(dptx,
			 "Invalid SHORT_AUDIO_DESC AUDIO_CHANNEL_COUNT\n");
		return -EINVAL;
	}
	dptx_dbg(dptx, "audio_ch_count = %d\n", audio_ch_count);
	aparams->num_channels = audio_ch_count;

	switch (audio_smaple_range) {
	case SAMPLE_FREQ_32:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_32\n");
		orig_sample_freq = 12;
		sample_freq = 3;
		audio_clock_freq = 320;
		break;
	case SAMPLE_FREQ_44_1:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_44_1\n");
		orig_sample_freq = 15;
		sample_freq = 0;
		audio_clock_freq = 441;
		break;
	case SAMPLE_FREQ_48:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_48\n");
		orig_sample_freq = 13;
		sample_freq = 2;
		audio_clock_freq = 480;
		break;
	case SAMPLE_FREQ_88_2:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_88_2\n");
		orig_sample_freq = 7;
		sample_freq = 8;
		audio_clock_freq = 882;
		break;
	case SAMPLE_FREQ_96:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_96\n");
		orig_sample_freq = 5;
		sample_freq = 10;
		audio_clock_freq = 960;
		break;
	case SAMPLE_FREQ_176_4:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_176_4\n");
		orig_sample_freq = 3;
		sample_freq = 12;
		audio_clock_freq = 1764;
		break;
	case SAMPLE_FREQ_192:
		dptx_dbg(dptx, "SHORT AUDIO DESC AUDIO_SAMPLING_RATE_192\n");
		orig_sample_freq = 1;
		sample_freq = 14;
		audio_clock_freq = 1920;
		break;
	default:
		dptx_dbg(dptx,
			 "Invalid SHORT AUDIO DESC AUDIO_SAMPLING_RATE\n");
		return -EINVAL;
	}
	dptx_dbg(dptx, "sample_freq = %d\n", sample_freq);
	dptx_dbg(dptx, "orig_sample_freq = %d\n", orig_sample_freq);

	aparams->data_width = audio_desc->max_bit_per_sample;
	dptx_dbg(dptx, "SHORT AUDIO DATA WIDTH = %d\n", aparams->data_width);
	dptx_audio_data_width_change(dptx);

	aparams->iec_samp_freq = sample_freq;
	aparams->iec_orig_samp_freq = orig_sample_freq;

	dptx_audio_num_ch_change(dptx);
	dptx_audio_samp_freq_config(dptx);
	dptx_audio_infoframe_sdp_send(dptx);

	return retval;
}

static int __maybe_unused dptx_dtd_fill_based_on_est_timings(struct dptx *dptx,
					      struct dtd *mdtd)
{
	struct video_params *vparams = &dptx->vparams;

	switch (dptx->selected_est_timing) {
	case DMT_640x480_60hz:
		dev_err(dptx->dev, "Set Video mode to DMT 640x480\n");
		vparams->video_format = DMT;
		dptx_dtd_fill(mdtd, 4, vparams->refresh_rate,
			      vparams->video_format);
		return 0;
	case DMT_800x600_60hz:
		dev_err(dptx->dev, "Set Video mode to DMT 800x600\n");
		vparams->video_format = DMT;
		dptx_dtd_fill(mdtd, 9, vparams->refresh_rate,
			      vparams->video_format);
		return 0;
	case DMT_1024x768_60hz:
		dev_err(dptx->dev, "Set Video mode to DMT 1024x768\n");
		vparams->video_format = DMT;
		dptx_dtd_fill(mdtd, 16, vparams->refresh_rate,
			      vparams->video_format);
		return 0;
	case NONE:
	default:
		dev_err(dptx->dev,
			"%s: Not Found selected timing in Established timings\n",
			__func__);
		return -EINVAL;
	}
}

static int handle_hotplug(struct dptx *dptx)
{
	u8 rev;
	int retval;
	u8 preferred_vic[18];
	struct video_params *vparams;
	struct hdcp_params *hparams;
	struct dtd mdtd;
	int i;
	u8 result;
	struct drm_dp_sideband_msg_rx raw;
	struct drm_dp_sideband_msg_reply_body rep;

	memset(&raw, 0, sizeof(raw));
	memset(&rep, 0, sizeof(rep));

	vparams = &dptx->vparams;
	hparams = &dptx->hparams;

	dptx_enable_ssc(dptx);

	dptx_soft_reset(dptx, DPTX_SRST_CTRL_AUX);

	dptx_core_init_phy(dptx);

	dptx_write_regfield(dptx, dptx->field_hdcp_module_reset, 0x1);
	usleep_range(10, 20);
	dptx_write_regfield(dptx, dptx->field_hdcp_module_reset, 0x0);

	dptx_write_regfield(dptx, dptx->field_en_audio_stream_sdp_vertical_ctrl,
			    0x0);
	dptx_write_regfield(
		dptx, dptx->field_en_audio_stream_sdp_horizontal_ctrl, 0x0);
	dptx_write_regfield(
		dptx, dptx->field_en_audio_timestamp_sdp_vertical_ctrl, 0x0);
	dptx_write_regfield(
		dptx, dptx->field_en_audio_timestamp_sdp_horizontal_ctrl, 0x0);

	dptx_write_regfield(dptx, dptx->field_audio_sampler_reset, 0x1);
	if (dptx->mst) {
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream1, 0x1);
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream2, 0x1);
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream3, 0x1);
	}
	usleep_range(10, 20);
	dptx_write_regfield(dptx, dptx->field_audio_sampler_reset, 0x0);
	if (dptx->mst) {
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream1, 0x0);
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream2, 0x0);
		dptx_write_regfield(
			dptx, dptx->field_audio_sampler_reset_stream3, 0x0);
	}

	retval = dptx_read_edid(dptx);
	if (retval)
		return retval;
	dptx_check_detailed_timing_descriptors(dptx);
	dptx_parse_edid_audio_data_block(dptx);

	retval = dptx_read_dpcd(dptx, DP_DPCD_REV, &rev);
	if (retval)
		return retval;
	dptx_info(dptx, "DP Revision %x.%x\n", (rev & 0xf0) >> 4, rev & 0xf);

	memset(dptx->rx_caps, 0, DPTX_RECEIVER_CAP_SIZE);
	retval = dptx_read_bytes_from_dpcd(dptx, DP_DPCD_REV, dptx->rx_caps,
					   DPTX_RECEIVER_CAP_SIZE);
	if (retval)
		return retval;

	if (dptx->rx_caps[DP_TRAINING_AUX_RD_INTERVAL] &
	    DP_EXTENDED_RECEIVER_CAPABILITY_FIELD_PRESENT) {
		retval = dptx_read_bytes_from_dpcd(dptx, 0x2200, dptx->rx_caps,
						   DPTX_RECEIVER_CAP_SIZE);
		if (retval)
			return retval;
	}

	retval = dptx_set_link_configs(dptx, dptx->max_rate, dptx->max_lanes);
	if (retval)
		return retval;

	if (dptx->fec) {
		dptx_write_regfield(dptx,
				    dptx->field_enhance_framing_with_fec_en, 1);
		retval = dptx_write_dpcd(dptx, DP_FEC_CONFIGURATION,
					 DP_FEC_READY);
		if (retval)
			return retval;
	}

	dptx_write_regfield(dptx, dptx->field_default_fast_link_train_en, 0);

	if (dptx->rx_caps[MAX_DOWNSPREAD] & NO_AUX_TRANSACTION_LINK_TRAINING) {
		dptx_fast_link_training(dptx);
	} else {
		retval = dptx_link_training(dptx);
		if (retval)
			return retval;
	}

	msleep(20);

	if (dptx->fec) {
		dptx_write_regfield(dptx, dptx->field_enable_fec, 1);
		dev_dbg(dptx->dev, "%s: Enabling Forward Error Correction\n",
			__func__);

		retval = dptx_read_dpcd(dptx, DP_FEC_STATUS, &result);
		if (retval)
			dev_dbg(dptx->dev, "DPCD read failed\n");
		dev_dbg(dptx->dev, "FEC Status = %x\n", result);

		retval = dptx_read_dpcd(dptx, DP_FEC_ERROR_COUNT_LSB, &result);
		if (retval)
			dev_dbg(dptx->dev, "DPCD read failed\n");

		dev_dbg(dptx->dev, "FEC Error Count %x\n", result);
	}

	dptx_dbg(dptx, "Configure Controller for Video Mode");

	retval = dptx_check_edid(dptx);
	if (retval) {
		vparams->video_format = VCEA;
		dptx_dtd_fill(&mdtd, dptx->vparams.mode, vparams->refresh_rate,
					vparams->video_format);
	} else {
		memcpy(preferred_vic, dptx->edid + 0x36, 0x12);
		retval = dptx_dtd_parse(dptx, &mdtd,
					preferred_vic);
		if (retval) {
			vparams->video_format = VCEA;
			dptx_dtd_fill(&mdtd, dptx->vparams.mode,
						vparams->refresh_rate,
						vparams->video_format);
		}
	}
	memcpy(&vparams->mdtd, &mdtd, sizeof(mdtd));

	dev_dbg(dptx->dev, "mdtd pixel_clock=%d\n", vparams->mdtd.pixel_clock);
	dev_dbg(dptx->dev, "pixel_repetition_input = %d\n",vparams->mdtd.pixel_repetition_input);
	dev_dbg(dptx->dev, "pixel_clock = %d\n", vparams->mdtd.pixel_clock);
	dev_dbg(dptx->dev, "h_active = %d\n",vparams->mdtd.h_active);
	dev_dbg(dptx->dev, "h_blanking = %d\n",vparams->mdtd.h_blanking);
	dev_dbg(dptx->dev, "h_sync_offset = %d\n",vparams->mdtd.h_sync_offset);
	dev_dbg(dptx->dev, "h_sync_pulse_width = %d\n",vparams->mdtd.h_sync_pulse_width);
	dev_dbg(dptx->dev, "h_image_size = %d\n",vparams->mdtd.h_image_size);
	dev_dbg(dptx->dev, "v_active = %d\n",vparams->mdtd.v_active);
	dev_dbg(dptx->dev, "v_blanking = %d\n",vparams->mdtd.v_blanking);
	dev_dbg(dptx->dev, "v_sync_offset = %d\n",vparams->mdtd.v_sync_offset);
	dev_dbg(dptx->dev, "v_sync_pulse_width = %d\n",vparams->mdtd.v_sync_pulse_width);
	dev_dbg(dptx->dev, "v_image_size = %d\n",vparams->mdtd.v_image_size);
	dev_dbg(dptx->dev, "interlaced = %d\n",vparams->mdtd.interlaced);
	dev_dbg(dptx->dev, "v_sync_polarity = %d\n",vparams->mdtd.v_sync_polarity);
	dev_dbg(dptx->dev, "h_sync_polarity = %d\n",vparams->mdtd.h_sync_polarity);

	dptx->active_mst_vc_payload = 0;

	dptx->streams = dptx_read_regfield(dptx, dptx->field_num_streams);
	dev_dbg(dptx->dev, "%s: NR STREAMS: %d\n", __func__, dptx->streams);

	for (i = 0; i < dptx->streams; i++) {
		dptx_video_set_core_bpc(dptx, i);
		dptx_video_set_timing_info(dptx, i);
		dptx_video_set_MSA(dptx, i);
	}
	dptx_dbg(dptx, "Configure Controller for Audio Mode");
	dptx_audio_core_config(dptx);
	dptx_audio_sdp_en(dptx);
	dptx_audio_timestamp_sdp_en(dptx);
	if (vparams->pix_enc == YCBCR420)
		dptx_vsd_ycbcr420_send(dptx, 1);

	dptx_mux_enable(dptx, false);
	retval = dptx_read_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT);
	dptx_info(dptx, "Before clear: [%s | %s]\n", retval & BIT(8) ? "UNDERFLOW" : " ", retval & BIT(6) ? "OVERFLOW" : " ");
	if (retval & (BIT(6) | BIT(8))) {
		dptx_write_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT, retval & (BIT(6) | BIT(8)));
		retval = dptx_read_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT);
		dptx_info(dptx, "After clear: [%s | %s]\n", retval & BIT(8) ? "UNDERFLOW" : " ", retval & BIT(6) ? "OVERFLOW" : " ");
	}
	dptx_mux_enable(dptx, true);

	return 0;
}

static void handle_hdcp_intr(struct dptx *dptx)
{
	u32 hdcpintsts;
	u32 hdcpgpiostchg;
	struct hdcp_params *hparams;

	hparams = &dptx->hparams;
	hdcpintsts = dptx_read_reg(dptx, dptx->regs[DPTX], HDCPAPIINTSTAT);
	dptx_dbg_irq(dptx, "%s: >>>> HDCP_INT_STS=0x%08x\n", __func__,
		     hdcpintsts);

	if (dptx_read_regfield(dptx, dptx->field_ksvaccessint_stat))
		dptx_dbg(
			dptx,
			"%s: KSV memory access guaranteed for read, write access\n",
			__func__);

	if (dptx_read_regfield(dptx, dptx->field_ksvsha1calcdoneint_stat))
		dptx_dbg(dptx, "%s: SHA1 verification has been done\n",
			 __func__);

	if (dptx_read_regfield(dptx, dptx->field_auxresptimeout_stat)) {
		dptx_dbg(dptx, "%s: AUXRESPTIMEOUT\n", __func__);
	}

	if (dptx_read_regfield(dptx, dptx->field_hdcp_failed_stat)) {
		hparams->auth_fail_count++;
		if (hparams->auth_fail_count > DPTX_HDCP_MAX_AUTH_RETRY) {
			dptx_dbg(dptx,
				 "%s: Reach max allowed retries count %d\n",
				 __func__, hparams->auth_fail_count);
		}
		dptx_dbg(dptx, "%s: HDCP authentication process was failed\n",
			 __func__);
	}

	if (dptx_read_regfield(dptx, dptx->field_hdcp_engaged_stat)) {
		if (hparams->hdcp13_is_en)
			hparams->auth_fail_count = 0;
		dptx_dbg(dptx,
			 "%s: HDCP authentication process was successful\n",
			 __func__);
	}

	if (dptx_read_regfield(dptx, dptx->field_hdcp2_gpioint_stat)) {
		dptx_dbg(dptx, "%s: HDCP22_GPIOINT\n", __func__);
		hdcpgpiostchg = dptx_read_reg(dptx, dptx->regs[DPTX],
					      DPTX_HDCP22GPIOOUTCHNGSTS);
		dptx_dbg(dptx, "%s: HDCP2.2 GPIO status changed %0x ", __func__,
			 hdcpgpiostchg);
		dptx_write_reg(dptx, dptx->regs[DPTX],
			       DPTX_HDCP22GPIOOUTCHNGSTS, hdcpgpiostchg);

		if ((hdcpgpiostchg & 56))
			dptx->bstatus = 1;
		else
			dptx->bstatus = 0;
	}
}

irqreturn_t dptx_threaded_irq(int irq, void *dev)
{
	int retval;
	struct dptx *dptx = dev;
	u32 hpdsts;

	dptx_dbg(dptx, "\n\n\n%s: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n",
		 __func__);
	dptx_dbg(dptx, "%s:\n", __func__);

	mutex_lock(&dptx->mutex);

	hpdsts = dptx_read_reg(dptx, dptx->regs[DPTX], HPD_STATUS);
	dptx_dbg(dptx, "%s: HPDSTS = 0x%08x\n", __func__, hpdsts);

	atomic_set(&dptx->aux.abort, 0);
	if (atomic_read(&dptx->c_connect)) {
		if (dptx_read_regfield(dptx, dptx->field_hpd_status))
			handle_hotplug(dptx);
		else {
			handle_hotunplug(dptx);
			atomic_set(&dptx->c_connect, 0);
		}
	}

	if (atomic_read(&dptx->sink_request)) {
		atomic_set(&dptx->sink_request, 0);
		retval = handle_sink_request(dptx);
		if (retval)
			dptx_err(dptx, "Unable to handle sink request %d\n",
				 retval);
	}

	dptx_dbg(dptx, "%s: DONE\n", __func__);
	dptx_dbg(dptx, "%s: =======================================\n\n",
		 __func__);

	mutex_unlock(&dptx->mutex);

	return IRQ_HANDLED;
}

static void handle_hpd_irq(struct dptx *dptx)
{
	dptx_dbg(dptx, "%s: HPD_IRQ\n", __func__);
	atomic_set(&dptx->sink_request, 1);
	dptx_notify(dptx);
}

irqreturn_t dptx_irq(int irq, void *dev)
{
	irqreturn_t retval = IRQ_HANDLED;
	struct dptx *dptx = dev;
	u32 ists;

	ists = dptx_read_reg(dptx, dptx->regs[DPTX], GENERAL_INTERRUPT);
	dptx_dbg_irq(dptx, "%s: >>>> ISTS=0x%08x\n", __func__, ists);

	if (!(ists & DPTX_ISTS_ALL_INTR)) {
		retval = IRQ_NONE;
		dptx_dbg(dptx, "%s: IRQ_NONE\n", __func__);
		goto done;
	}

	if (dptx_read_regfield(dptx, dptx->field_hdcp_event)) {
		dptx_dbg(dptx, "%s: DPTX_ISTS_HDCP\n", __func__);
		handle_hdcp_intr(dptx);
	}

	if (dptx_read_regfield(dptx, dptx->field_sdp_event_stream0)) {
		dptx_dbg(dptx, "%s: DPTX_ISTS_SDP\n", __func__);
	}

	if (dptx_read_regfield(dptx, dptx->field_audio_fifo_overflow_stream0)) {
		if (dptx_read_regfield(
			    dptx, dptx->field_audio_fifo_overflow_en_stream0)) {
			dptx_dbg(dptx, "%s: DPTX_ISTS_AUDIO_FIFO_OVERFLOW\n",
				 __func__);
			dptx_write_regfield(
				dptx, dptx->field_audio_fifo_overflow_stream0,
				1);
		}
	}

	if (dptx_read_regfield(dptx, dptx->field_video_fifo_overflow_stream0)) {
		if (dptx_read_regfield(
			    dptx, dptx->field_video_fifo_overflow_en_stream0)) {
			dptx_dbg(dptx, "%s: DPTX_ISTS_VIDEO_FIFO_OVERFLOW\n",
				 __func__);
			dptx_write_regfield(
				dptx, dptx->field_video_fifo_overflow_stream0,
				1);
		}
	}

	if (dptx_read_regfield(dptx, dptx->field_hpd_event)) {
		u32 hpdsts;

		dptx_dbg(dptx, "%s: HPD_EVENT\n", __func__);
		hpdsts = dptx_read_reg(dptx, dptx->regs[DPTX], HPD_STATUS);

		dptx_dbg(dptx, "%s: HPDSTS = 0x%08x\n", __func__, hpdsts);

		if (dptx_read_regfield(dptx, dptx->field_hpd_irq)) {
			dptx_dbg(dptx, "%s: DPTX_HPDSTS_IRQ\n", __func__);
			dptx_write_regfield(dptx, dptx->field_hpd_irq,
					    1);
			handle_hpd_irq(dptx);
			retval = IRQ_WAKE_THREAD;
		}

		if (dptx_read_regfield(dptx, dptx->field_hpd_hot_plug)) {
			dptx_info(dptx, "%s: HPD_STATUS - Hot Plug Detected\n",
				  __func__);

			dptx_write_regfield(dptx, dptx->field_phy_powerdown, 0);
			dptx_write_regfield(dptx, dptx->field_hpd_hot_plug, 1);

			atomic_set(&dptx->aux.abort, 1);
			atomic_set(&dptx->c_connect, 1);
			dptx_notify(dptx);
			retval = IRQ_WAKE_THREAD;
		}

		if (dptx_read_regfield(dptx, dptx->field_hpd_hot_unplug)) {
			dptx_info(dptx, "%s: DPTX_HPDSTS_HOT_UNPLUG\n",
				  __func__);
			dptx_write_regfield(dptx, dptx->field_hpd_hot_unplug,
					    1);
			atomic_set(&dptx->aux.abort, 1);
			atomic_set(&dptx->c_connect, 1);
			dptx_notify(dptx);
			retval = IRQ_WAKE_THREAD;
		}
	}

done:
	dptx_dbg_irq(dptx, "%s: <<<<\n", __func__);
	return retval;
}
