// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "dptx_drv.h"
#include "dptx_dbg.h"

u8 dptx_bit_field(const u16 data, u8 shift, u8 width)
{
	return ((data >> shift) & ((((u16)1) << width) - 1));
}

u16 dptx_concat_bits(u8 bhi, u8 ohi, u8 nhi, u8 blo, u8 olo, u8 nlo)
{
	return (dptx_bit_field(bhi, ohi, nhi) << nlo) |
	       dptx_bit_field(blo, olo, nlo);
}

u16 dptx_byte_to_word(const u8 hi, const u8 lo)
{
	return dptx_concat_bits(hi, 0, 8, lo, 0, 8);
}

u32 dptx_byte_to_dword(u8 b3, u8 b2, u8 b1, u8 b0)
{
	u32 retval = 0;

	retval |= b0 << (0 * 8);
	retval |= b1 << (1 * 8);
	retval |= b2 << (2 * 8);
	retval |= b3 << (3 * 8);
	return retval;
}

int dptx_dtd_parse(struct dptx *dptx, struct dtd *mdtd, u8 data[18])
{
	mdtd->pixel_repetition_input = 0;

	mdtd->pixel_clock = dptx_byte_to_word(data[1], data[0]);
	if (mdtd->pixel_clock < 0x01)
		return -EINVAL;

	mdtd->h_active = dptx_concat_bits(data[4], 4, 4, data[2], 0, 8);
	mdtd->h_blanking = dptx_concat_bits(data[4], 0, 4, data[3], 0, 8);
	mdtd->h_sync_offset = dptx_concat_bits(data[11], 6, 2, data[8], 0, 8);
	mdtd->h_sync_pulse_width =
		dptx_concat_bits(data[11], 4, 2, data[9], 0, 8);
	mdtd->h_image_size = dptx_concat_bits(data[14], 4, 4, data[12], 0, 8);
	mdtd->v_active = dptx_concat_bits(data[7], 4, 4, data[5], 0, 8);
	mdtd->v_blanking = dptx_concat_bits(data[7], 0, 4, data[6], 0, 8);
	mdtd->v_sync_offset = dptx_concat_bits(data[11], 2, 2, data[10], 4, 4);
	mdtd->v_sync_pulse_width =
		dptx_concat_bits(data[11], 0, 2, data[10], 0, 4);
	mdtd->v_image_size = dptx_concat_bits(data[14], 0, 4, data[13], 0, 8);
	if (dptx_bit_field(data[17], 4, 1) != 1)
		return -EINVAL;
	if (dptx_bit_field(data[17], 3, 1) != 1)
		return -EINVAL;

	mdtd->interlaced = dptx_bit_field(data[17], 7, 1) == 1;
	mdtd->v_sync_polarity = dptx_bit_field(data[17], 2, 1);
	mdtd->h_sync_polarity = dptx_bit_field(data[17], 1, 1);
	if (mdtd->interlaced == 1)
		mdtd->v_active /= 2;
	mdtd->pixel_clock *= 10;
	dptx_dbg(dptx, "DTD pixel_clock: %d interlaced: %d\n",
		 mdtd->pixel_clock, mdtd->interlaced);
	dptx_dbg(dptx, "h_active: %d h_blanking: %d h_sync_offset: %d\n",
		 mdtd->h_active, mdtd->h_blanking, mdtd->h_sync_offset);
	dptx_dbg(
		dptx,
		"h_sync_pulse_width: %d h_image_size: %d h_sync_polarity: %d\n",
		mdtd->h_sync_pulse_width, mdtd->h_image_size,
		mdtd->h_sync_polarity);
	dptx_dbg(dptx, "v_active: %d v_blanking: %d v_sync_offset: %d\n",
		 mdtd->v_active, mdtd->v_blanking, mdtd->v_sync_offset);
	dptx_dbg(
		dptx,
		"v_sync_pulse_width: %d v_image_size: %d v_sync_polarity: %d\n",
		mdtd->v_sync_pulse_width, mdtd->v_image_size,
		mdtd->v_sync_polarity);

	return 0;
}

void dptx_audio_sdp_en(struct dptx *dptx)
{
	dptx_write_regfield(dptx, dptx->field_en_audio_stream_sdp_vertical_ctrl,
			    1);
	dptx_write_regfield(dptx,
			    dptx->field_en_audio_stream_sdp_horizontal_ctrl, 1);
}

void dptx_audio_timestamp_sdp_en(struct dptx *dptx)
{
	dptx_write_regfield(
		dptx, dptx->field_en_audio_timestamp_sdp_vertical_ctrl, 1);
	dptx_write_regfield(
		dptx, dptx->field_en_audio_timestamp_sdp_horizontal_ctrl, 1);
}

void dptx_audio_infoframe_sdp_send(struct dptx *dptx)
{
	u32 audio_infoframe_header = AUDIO_INFOFREAME_HEADER;
	u32 audio_infoframe_data[3] = { 0x00000710, 0x0, 0x0 };
	u8 orig_sample_freq = 0;
	u8 sample_freq = 0;
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	sample_freq = aparams->iec_samp_freq;
	orig_sample_freq = aparams->iec_orig_samp_freq;

	if (orig_sample_freq == 12 && sample_freq == 3)
		audio_infoframe_data[0] = 0x00000710;
	else if (orig_sample_freq == 15 && sample_freq == 0)
		audio_infoframe_data[0] = 0x00000B10;
	else if (orig_sample_freq == 13 && sample_freq == 2)
		audio_infoframe_data[0] = 0x00000F10;
	else if (orig_sample_freq == 7 && sample_freq == 8)
		audio_infoframe_data[0] = 0x00001310;
	else if (orig_sample_freq == 5 && sample_freq == 10)
		audio_infoframe_data[0] = 0x00001710;
	else if (orig_sample_freq == 3 && sample_freq == 12)
		audio_infoframe_data[0] = 0x00001B10;
	else
		audio_infoframe_data[0] = 0x00001F10;

	audio_infoframe_data[0] |= (aparams->num_channels - 1);
	if (aparams->num_channels == 3)
		audio_infoframe_data[0] |= 0x02000000;
	else if (aparams->num_channels == 4)
		audio_infoframe_data[0] |= 0x03000000;
	else if (aparams->num_channels == 5)
		audio_infoframe_data[0] |= 0x07000000;
	else if (aparams->num_channels == 6)
		audio_infoframe_data[0] |= 0x0b000000;
	else if (aparams->num_channels == 7)
		audio_infoframe_data[0] |= 0x0f000000;
	else if (aparams->num_channels == 8)
		audio_infoframe_data[0] |= 0x13000000;

	//dev_err(dptx->dev, "audio_infoframe_data[0] before = %x\n", audio_infoframe_data[0]);
	switch (aparams->data_width) {
	case 16:
		//dev_dbg(dptx->dev, "%s: data_width = 16\n", __func__);
		audio_infoframe_data[0] &= ~GENMASK(9, 8);
		audio_infoframe_data[0] |= 1 << 8;
		break;
	case 20:
		//dev_dbg(dptx->dev, "%s: data_width = 20\n", __func__);
		audio_infoframe_data[0] &= ~GENMASK(9, 8);
		audio_infoframe_data[0] |= 2 << 8;
		break;
	case 24:
		//dev_dbg(dptx->dev, "%s: data_width = 24\n", __func__);
		audio_infoframe_data[0] &= ~GENMASK(9, 8);
		audio_infoframe_data[0] |= 3 << 8;
		break;
	default:
		dev_dbg(dptx->dev, "%s: data_width not found\n", __func__);
		break;
	}

	//dev_err(dptx->dev, "audio_infoframe_data[0] after = %x\n", audio_infoframe_data[0]);

	dptx->sdp_list[0].payload[0] = audio_infoframe_header;
	dptx_write_reg(dptx, dptx->regs[DPTX], SDP_REGISTER_BANK_0,
		       audio_infoframe_header);
	dptx_write_reg(dptx, dptx->regs[DPTX], SDP_REGISTER_BANK_1,
		       audio_infoframe_data[0]);
	dptx_write_reg(dptx, dptx->regs[DPTX], SDP_REGISTER_BANK_2,
		       audio_infoframe_data[1]);
	dptx_write_reg(dptx, dptx->regs[DPTX], SDP_REGISTER_BANK_3,
		       audio_infoframe_data[2]);

	dptx_write_regfield(dptx, dptx->field_en_vertical_sdp_n, 1);
}

void dptx_disable_sdp(struct dptx *dptx, u32 *payload)
{
	int i;

	for (i = 0; i < DPTX_SDP_NUM; i++)
		if (!memcmp(dptx->sdp_list[i].payload, payload, 9))
			memset(dptx->sdp_list[i].payload, 0,
			       sizeof(*payload) * 9);
}

void dptx_enable_sdp(struct dptx *dptx, struct sdp_full_data *data)
{
	int i;
	u32 reg;
	int reg_num;
	u32 header;
	int sdp_offset;

	reg_num = 0;
	header = cpu_to_be32(data->payload[0]);
	for (i = 0; i < DPTX_SDP_NUM; i++)
		if (dptx->sdp_list[i].payload[0] == 0) {
			dptx->sdp_list[i].payload[0] = header;
			sdp_offset = i * DPTX_SDP_SIZE;
			reg_num = 0;
			while (reg_num < DPTX_SDP_LEN) {
				dptx_write_reg(
					dptx, dptx->regs[DPTX],
					SDP_REGISTER_BANK_0 + sdp_offset +
						reg_num * 4,
					cpu_to_be32(data->payload[reg_num]));
				reg_num++;
			}
			switch (data->blanking) {
			case 0:
				reg = dptx_read_reg(dptx, dptx->regs[DPTX],
						    SDP_VERTICAL_CTRL);
				reg |= (1 << (2 + i));
				dptx_write_reg(dptx, dptx->regs[DPTX],
					       SDP_VERTICAL_CTRL, reg);
				break;
			case 1:
				reg = dptx_read_reg(dptx, dptx->regs[DPTX],
						    SDP_HORIZONTAL_CTRL);
				reg |= (1 << (2 + i));
				dptx_write_reg(dptx, dptx->regs[DPTX],
					       SDP_HORIZONTAL_CTRL, reg);
				break;
			case 2:
				reg = dptx_read_reg(dptx, dptx->regs[DPTX],
						    SDP_VERTICAL_CTRL);
				reg |= (1 << (2 + i));
				dptx_write_reg(dptx, dptx->regs[DPTX],
					       SDP_VERTICAL_CTRL, reg);
				reg = dptx_read_reg(dptx, dptx->regs[DPTX],
						    SDP_HORIZONTAL_CTRL);
				reg |= (1 << (2 + i));
				dptx_write_reg(dptx, dptx->regs[DPTX],
					       SDP_HORIZONTAL_CTRL, reg);
				break;
			}
			break;
		}
}

void dptx_fill_sdp(struct dptx *dptx, struct sdp_full_data *data)
{
	if (data->en == 1)
		dptx_enable_sdp(dptx, data);
	else
		dptx_disable_sdp(dptx, data->payload);
}

void dptx_vsd_ycbcr420_send(struct dptx *dptx, u8 enable)
{
	struct sdp_full_data vsc_data;
	int i;

	struct video_params *vparams;

	vparams = &dptx->vparams;

	vsc_data.en = enable;
	for (i = 0; i < 9; i++) {
		if (i == 0)
			vsc_data.payload[i] = 0x00070513;
		else if (i == 5)
			switch (vparams->bpc) {
			case COLOR_DEPTH_8:
				vsc_data.payload[i] = 0x30010000;
				break;
			case COLOR_DEPTH_10:
				vsc_data.payload[i] = 0x30020000;
				break;
			case COLOR_DEPTH_12:
				vsc_data.payload[i] = 0x30030000;
				break;
			case COLOR_DEPTH_16:
				vsc_data.payload[i] = 0x30040000;
				break;
			}
		else
			vsc_data.payload[i] = 0x0;
	}
	vsc_data.blanking = 0;
	vsc_data.cont = 1;

	dptx_fill_sdp(dptx, &vsc_data);
}

void dptx_en_audio_channel(struct dptx *dptx, int ch_num, int enable)
{
	u32 reg = 0;
	u32 data_en = 0;

	reg = dptx_read_reg(dptx, dptx->regs[DPTX], AUD_CONFIG1);
	reg &= ~DPTX_AUD_CONFIG1_DATA_EN_IN_MASK;

	if (enable) {
		switch (ch_num) {
		case 1:
			data_en = DPTX_EN_AUDIO_CH_1;
			break;
		case 2:
			data_en = DPTX_EN_AUDIO_CH_2;
			break;
		case 3:
			data_en = DPTX_EN_AUDIO_CH_3;
			break;
		case 4:
			data_en = DPTX_EN_AUDIO_CH_4;
			break;
		case 5:
			data_en = DPTX_EN_AUDIO_CH_5;
			break;
		case 6:
			data_en = DPTX_EN_AUDIO_CH_6;
			break;
		case 7:
			data_en = DPTX_EN_AUDIO_CH_7;
			break;
		case 8:
			data_en = DPTX_EN_AUDIO_CH_8;
			break;
		}
		reg |= data_en << DPTX_AUD_CONFIG1_DATA_EN_IN_SHIFT;
	} else {
		switch (ch_num) {
		case 1:
			data_en = ~DPTX_EN_AUDIO_CH_1;
			break;
		case 2:
			data_en = ~DPTX_EN_AUDIO_CH_2;
			break;
		case 3:
			data_en = ~DPTX_EN_AUDIO_CH_3;
			break;
		case 4:
			data_en = ~DPTX_EN_AUDIO_CH_4;
			break;
		case 5:
			data_en = ~DPTX_EN_AUDIO_CH_5;
			break;
		case 6:
			data_en = ~DPTX_EN_AUDIO_CH_6;
			break;
		case 7:
			data_en = ~DPTX_EN_AUDIO_CH_7;
			break;
		case 8:
			data_en = ~DPTX_EN_AUDIO_CH_8;
			break;
		}
		reg &= data_en << DPTX_AUD_CONFIG1_DATA_EN_IN_SHIFT;
	}
	dptx_write_reg(dptx, dptx->regs[DPTX], AUD_CONFIG1, reg);
}

void dptx_video_reset(struct dptx *dptx, int enable, int stream)
{
	u32 reg;

	reg = dptx_read_regfield(dptx, dptx->field_video_reset);

	if (enable)
		reg |= BIT(stream);
	else
		reg &= ~BIT(stream);

	dptx_write_regfield(dptx, dptx->field_video_reset, reg);
}

void dptx_audio_mute(struct dptx *dptx)
{
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	if (aparams->mute == 1)
		dptx_write_regfield(dptx, dptx->field_audio_mute, 1);
	else
		dptx_write_regfield(dptx, dptx->field_audio_mute, 0);
}

void dptx_audio_config(struct dptx *dptx)
{
	dptx_audio_core_config(dptx);
	dptx_audio_sdp_en(dptx);
	dptx_audio_timestamp_sdp_en(dptx);
	dptx_audio_infoframe_sdp_send(dptx);
}

void dptx_audio_core_config(struct dptx *dptx)
{
	struct audio_params *aparams;

	aparams = &dptx->aparams;

	dptx_audio_inf_type_change(dptx);
	dptx_audio_num_ch_change(dptx);
	dptx_audio_data_width_change(dptx);
	dptx_write_regfield(dptx, dptx->field_audio_timestamp_version_num,
			    aparams->ats_ver);
	dptx_en_audio_channel(dptx, aparams->num_channels, 1);
}

void dptx_audio_inf_type_change(struct dptx *dptx)
{
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	dptx_write_regfield(dptx, dptx->field_audio_inf_select,
			    aparams->inf_type);
}

void dptx_audio_num_ch_change(struct dptx *dptx)
{
	u32 num_ch_map;
	struct audio_params *aparams;

	aparams = &dptx->aparams;

	if (aparams->num_channels == 1)
		num_ch_map = 0;
	else if (aparams->num_channels == 2)
		num_ch_map = 1;
	else
		num_ch_map = aparams->num_channels - 1;

	dptx_write_regfield(dptx, dptx->field_num_channels, num_ch_map);
}

void dptx_audio_data_width_change(struct dptx *dptx)
{
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	dptx_write_regfield(dptx, dptx->field_audio_data_width,
			    aparams->data_width);
}

void dptx_audio_samp_freq_config(struct dptx *dptx)
{
	struct audio_params *aparams;

	aparams = &dptx->aparams;
	dptx_write_regfield(dptx, dptx->field_iec_samp_freq,
			    aparams->iec_samp_freq);
	dptx_write_regfield(dptx, dptx->field_iec_origsampfreq,
			    aparams->iec_orig_samp_freq);
}

/*
 * Video Generation
 */

void dptx_video_timing_change(struct dptx *dptx, int stream)
{
	dptx_disable_default_video_stream(dptx, stream);
	dptx_video_core_config(dptx, stream);
	dptx_video_ts_change(dptx, stream);
	dptx_enable_default_video_stream(dptx, stream);
}

int dptx_video_mode_change(struct dptx *dptx, u8 vmode, int stream)
{
	int retval;
	struct video_params *vparams;
	struct dtd mdtd;

	vparams = &dptx->vparams;
	if (!dptx_dtd_fill(&mdtd, vmode, vparams->refresh_rate,
			   vparams->video_format)) {
		dptx_dbg(dptx, "%s: Invalid video mode value %d\n", __func__,
			 vmode);
		return -EINVAL;
	}

	retval = dptx_video_ts_calculate(dptx, dptx->link.lanes,
					 dptx->link.rate, vparams->bpc,
					 vparams->pix_enc, mdtd.pixel_clock);
	if (retval)
		return retval;
	vparams->mdtd = mdtd;
	vparams->mode = vmode;

	dptx_video_timing_change(dptx, stream);
	dptx_dbg(dptx, "%s: Change video mode to %d\n", __func__, vmode);

	return retval;
}

int dptx_video_config(struct dptx *dptx, int stream)
{
	struct video_params *vparams;
	struct dtd *mdtd;

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;
	dptx_info(dptx, "dtd: mode:%d, refresh_rate:%d, video_format:%d",
		  vparams->mode, vparams->refresh_rate, vparams->video_format);
	if (!dptx_dtd_fill(mdtd, vparams->mode, vparams->refresh_rate,
			   vparams->video_format))
		return -EINVAL;

	dptx_video_core_config(dptx, stream);

	return 0;
}

int dptx_calculate_hblank_interval(struct dptx *dptx)
{
	struct video_params *vparams;
	int pixel_clk;
	u16 h_blank;
	u32 link_clk;
	u8 rate;
	int hblank_interval;

	vparams = &dptx->vparams;
	pixel_clk = vparams->mdtd.pixel_clock;
	h_blank = vparams->mdtd.h_blanking;
	rate = dptx->link.rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_clk = 202500;
		break;
	default:
		WARN(1, "Invalid rate 0x%x\n", rate);
		return -EINVAL;
	}

	hblank_interval = h_blank * link_clk / pixel_clk;

	return hblank_interval;
}

void dptx_video_core_config(struct dptx *dptx, int stream)
{
	u32 reg = 0;
	u8 vmode;

	struct video_params *vparams;
	struct dtd *mdtd;

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;
	vmode = vparams->mode;

	dptx_video_set_core_bpc(dptx, stream);
	reg = dptx_read_reg(dptx, dptx->regs[DPTX],
			    DPTX_VSAMPLE_CTRL_N(stream));
	reg &= ~DPTX_VSAMPLE_CTRL_MULTI_PIXEL_MASK;
	reg |= dptx->multipixel << DPTX_VSAMPLE_CTRL_MULTI_PIXEL_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VSAMPLE_CTRL_N(stream),
		       reg);

	reg = 0;
	if (mdtd->h_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_H_SYNC_POL_EN;
	if (mdtd->v_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_V_SYNC_POL_EN;

	dptx_write_reg(dptx, dptx->regs[DPTX],
		       DPTX_VSAMPLE_POLARITY_CTRL_N(stream), reg);

	reg = 0;
	if (vparams->video_format == VCEA) {
		if (vmode == 5 || vmode == 6 || vmode == 7 || vmode == 10 ||
		    vmode == 11 || vmode == 20 || vmode == 21 || vmode == 22 ||
		    vmode == 39 || vmode == 25 || vmode == 26 || vmode == 40 ||
		    vmode == 44 || vmode == 45 || vmode == 46 || vmode == 50 ||
		    vmode == 51 || vmode == 54 || vmode == 55 || vmode == 58 ||
		    vmode == 59)
			reg |= DPTX_VIDEO_CONFIG1_IN_OSC_EN;
	}

	if (mdtd->interlaced == 1)
		reg |= DPTX_VIDEO_CONFIG1_O_IP_EN;

	reg |= mdtd->h_active << DPTX_VIDEO_H_ACTIVE_SHIFT;
	reg |= mdtd->h_blanking << DPTX_VIDEO_H_BLANK_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG1_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->v_active << DPTX_VIDEO_V_ACTIVE_SHIFT;
	reg |= mdtd->v_blanking << DPTX_VIDEO_V_BLANK_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG2_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->h_sync_offset << DPTX_VIDEO_H_FRONT_PORCH;
	reg |= mdtd->h_sync_pulse_width << DPTX_VIDEO_H_SYNC_WIDTH;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG3_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->v_sync_offset << DPTX_VIDEO_V_FRONT_PORCH;
	reg |= mdtd->v_sync_pulse_width << DPTX_VIDEO_V_SYNC_WIDTH;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG4_N(stream),
		       reg);

	dptx_video_ts_change(dptx, stream);

	reg = 0;
	reg |= (mdtd->h_blanking - mdtd->h_sync_offset)
	       << DPTX_VIDEO_MSA1_H_START_SHIFT;
	reg |= (mdtd->v_blanking - mdtd->v_sync_offset)
	       << DPTX_VIDEO_MSA1_V_START_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA1_N(stream), reg);

	dptx_video_set_sink_bpc(dptx, stream);

	reg = dptx_calculate_hblank_interval(dptx);
	dptx_write_regfield(dptx, dptx->field_hblank_interval, reg);
}

int dptx_get_vc_payload_size(struct dptx *dptx)
{
	struct video_params *vparams;
	int vc_payload_size, ts_int, ts_frac;
	int peak_stream_bandwidth;
	int link_bandwidth;
	int link_rate, bpp;

	vparams = &dptx->vparams;

	switch (dptx->link.rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_rate = 162;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_rate = 270;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_rate = 540;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_rate = 810;
		break;
	default:
		link_rate = 162;
	}

	switch (vparams->bpc) {
	case COLOR_DEPTH_6:
		bpp = 18;
		break;
	case COLOR_DEPTH_8:
		if (vparams->pix_enc == YCBCR420)
			bpp = 12;
		else if (vparams->pix_enc == YCBCR422)
			bpp = 16;
		else if (vparams->pix_enc == YONLY)
			bpp = 8;
		else
			bpp = 24;
		break;
	case COLOR_DEPTH_10:
		if (vparams->pix_enc == YCBCR420)
			bpp = 15;
		else if (vparams->pix_enc == YCBCR422)
			bpp = 20;
		else if (vparams->pix_enc == YONLY)
			bpp = 10;
		else
			bpp = 30;
		break;

	case COLOR_DEPTH_12:
		if (vparams->pix_enc == YCBCR420)
			bpp = 18;
		else if (vparams->pix_enc == YCBCR422)
			bpp = 24;
		else if (vparams->pix_enc == YONLY)
			bpp = 12;
		else
			bpp = 36;
		break;

	case COLOR_DEPTH_16:
		if (vparams->pix_enc == YCBCR420)
			bpp = 24;
		else if (vparams->pix_enc == YCBCR422)
			bpp = 32;
		else if (vparams->pix_enc == YONLY)
			bpp = 16;
		else
			bpp = 48;
		break;
	default:
		bpp = 18;
		break;
	}

	peak_stream_bandwidth = (vparams->mdtd.pixel_clock * bpp) / (8 * 1000);
	link_bandwidth = link_rate * dptx->link.lanes;
	vc_payload_size =
		DIV_ROUND_UP_ULL(64 * peak_stream_bandwidth, link_bandwidth);
	ts_int = DIV_ROUND_DOWN_ULL(64 * peak_stream_bandwidth, link_bandwidth);
	ts_frac = ((64 * 100 * peak_stream_bandwidth / link_bandwidth) -
		   ts_int * 100);
	vparams->aver_bytes_per_tu = ts_int;
	vparams->aver_bytes_per_tu_frac = ts_frac;

	return vc_payload_size;
}

int dptx_video_ts_calculate(struct dptx *dptx, int lane_num, int rate, int bpc,
			    int encoding, int pixel_clock)
{
	struct video_params *vparams;
	struct dtd *mdtd;
	int link_rate;
	int link_clk;
	int retval = 0;
	int ts;
	int T1;
	int T2;
	int tu;
	int tu_frac;
	int color_dep;

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;

	if (!pixel_clock) {
		dev_err(dptx->dev, "Error, pixel clock value illegal!\n");
		return -1;
	}
	dev_dbg(dptx->dev, "vparams h_active = %d\n", mdtd->h_active);
	dev_dbg(dptx->dev, "vparams pixel_clock = %d\n", mdtd->pixel_clock);
	dev_dbg(dptx->dev, "vparams h_sync_polarity = %d\n", mdtd->h_sync_polarity);
	dev_dbg(dptx->dev, "vparams h_blanking = %d\n", mdtd->h_blanking);
	dev_dbg(dptx->dev, "vparams h_sync_offset = %d\n", mdtd->h_sync_offset);
	dev_dbg(dptx->dev, "vparams h_sync_pulse_width = %d\n", mdtd->h_sync_pulse_width);
	dev_dbg(dptx->dev, "vparams v_active = %d\n", mdtd->v_active);
	dev_dbg(dptx->dev, "vparams v_blanking = %d\n", mdtd->v_blanking);
	dev_dbg(dptx->dev, "vparams v_sync_offset = %d\n", mdtd->v_sync_offset);
	dev_dbg(dptx->dev, "vparams v_sync_pulse_width = %d\n", mdtd->v_sync_pulse_width);
	dev_dbg(dptx->dev, "vparams v_sync_polarity = %d\n", mdtd->v_sync_polarity);

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		link_rate = 162;
		link_clk = 40500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		link_rate = 270;
		link_clk = 67500;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		link_rate = 540;
		link_clk = 135000;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		link_rate = 810;
		link_clk = 202500;
		break;
	default: //sahakyan
		link_rate = 162;
		link_clk = 40500;
	}

	switch (bpc) {
	case COLOR_DEPTH_6:
		color_dep = 18;
		break;
	case COLOR_DEPTH_8:
		if (encoding == YCBCR420)
			color_dep = 12;
		else if (encoding == YCBCR422)
			color_dep = 16;
		else if (encoding == YONLY)
			color_dep = 8;
		else
			color_dep = 24;
		break;
	case COLOR_DEPTH_10:
		if (encoding == YCBCR420)
			color_dep = 15;
		else if (encoding == YCBCR422)
			color_dep = 20;
		else if (encoding == YONLY)
			color_dep = 10;
		else
			color_dep = 30;
		break;

	case COLOR_DEPTH_12:
		if (encoding == YCBCR420)
			color_dep = 18;
		else if (encoding == YCBCR422)
			color_dep = 24;
		else if (encoding == YONLY)
			color_dep = 12;
		else
			color_dep = 36;
		break;

	case COLOR_DEPTH_16:
		if (encoding == YCBCR420)
			color_dep = 24;
		else if (encoding == YCBCR422)
			color_dep = 32;
		else if (encoding == YONLY)
			color_dep = 16;
		else
			color_dep = 48;
		break;
	default:
		color_dep = 18;
		break;
	}

	ts = (8 * color_dep * pixel_clock) / (lane_num * link_rate);

	tu = ts / 1000;
	if (tu >= 65) {
		dptx_dbg(dptx, "%s: tu(%d) > 65", __func__, tu);
		return -EINVAL;
	}

	tu_frac = ts / 100 - tu * 10;


	T1 = 0;
	T2 = 0;
	if (dptx->multipixel == DPTX_MP_SINGLE_PIXEL) {
		if (tu < 16)
			vparams->init_threshold = 32;
		else if (mdtd->h_blanking <= 40 && encoding >= YCBCR420 &&
			 encoding <= YCBCR444)
			vparams->init_threshold = 3;
		else if (mdtd->h_blanking <= 80 && encoding >= YCBCR420 &&
			 encoding <= YCBCR444)
			vparams->init_threshold = 12;
		else
			vparams->init_threshold = 16;
	} else {
		switch (bpc) {
		case COLOR_DEPTH_6:
			T1 = (4 * 1000 / 9) * lane_num;
			break;
		case COLOR_DEPTH_8:
			if (encoding == YCBCR422)
				T1 = (1000 / 2) * lane_num;
			else if (encoding == YONLY)
				T1 = lane_num * 1000;
			else if (dptx->multipixel == DPTX_MP_DUAL_PIXEL)
				T1 = (1000 / 3) * lane_num;
			else
				T1 = (3000 / 16) * lane_num;
			break;
		case COLOR_DEPTH_10:
			if (encoding == YCBCR422)
				T1 = (2000 / 5) * lane_num;
			else if (encoding == YONLY)
				T1 = (4000 / 5) * lane_num;
			else
				T1 = (4000 / 15) * lane_num;
			break;
		case COLOR_DEPTH_12:
			if (encoding == YCBCR422)
				if (dptx->multipixel == DPTX_MP_DUAL_PIXEL)
					T1 = (1000 / 6) * lane_num;
				else
					T1 = (1000 / 3) * lane_num;
			else if (encoding == YONLY)
				T1 = (2000 / 3) * lane_num;
			else
				T1 = (2000 / 9) * lane_num;
			break;
		case COLOR_DEPTH_16:
			if (encoding == YONLY)
				T1 = (1000 / 2) * lane_num;
			else if ((encoding != YONLY) && (encoding != YCBCR422) &&
			    (dptx->multipixel == DPTX_MP_DUAL_PIXEL))
				T1 = (1000 / 6) * lane_num;
			else
				T1 = (1000 / 4) * lane_num;
			break;
		default:
			dptx_dbg(dptx, "Invalid param BPC = %d\n", bpc);
			return -EINVAL;
		}

		if (encoding == YCBCR420)
			pixel_clock = pixel_clock / 2;

		T2 = (link_clk * 1000 / pixel_clock);

		vparams->init_threshold = T1 * T2 * tu / (1000 * 1000);
	}

	dptx_info(dptx, "T1 = %d, T2 =%d, vparams->init_threshold = %d,", T1,
		  T2, vparams->init_threshold);
	dptx_info(dptx, "tu = %d, tu_frac = %d\n", tu, tu_frac);
	vparams->aver_bytes_per_tu = tu;
	vparams->aver_bytes_per_tu_frac = tu_frac;

	if (dptx->mst) {
		u32 tu_mst;
		u32 tu_frac_mst;

		int numerator;
		int denominator;
		s64 fixp;

		dptx_dbg(dptx, "MST: pixel_clock=%d\n", mdtd->pixel_clock);
		numerator = 25175 * 3 * 10;
		dptx_dbg(dptx, "MST: numerator=%d\n", numerator);
		denominator = (link_rate)*lane_num * 100 * 1000 / 10;
		dptx_dbg(dptx, "MST: denominator=%d\n", denominator);
		fixp = drm_fixp_from_fraction(numerator * 64, denominator);
		tu_mst = drm_fixp2int(fixp);

		fixp &= DRM_FIXED_DECIMAL_MASK;
		fixp *= 64;
		tu_frac_mst = drm_fixp2int(fixp);

		dptx_dbg(dptx, "MST: tu = %d, tu_frac = %d\n", tu_mst,
			 tu_frac_mst);
		vparams->aver_bytes_per_tu = tu_mst;
		vparams->aver_bytes_per_tu_frac = tu_frac_mst;

		/* TODO this is a duplicate calculation from above */
		if (tu_mst < 6) {
			vparams->init_threshold = 32;
		} else if ((encoding == RGB || encoding == YCBCR444) &&
			   mdtd->h_blanking <= 80) {
			if (dptx->multipixel == DPTX_MP_QUAD_PIXEL)
				vparams->init_threshold = 4;
			else
				vparams->init_threshold = 12;
		} else {
			vparams->init_threshold = 15;
		}
	}

	return retval;
}

void dptx_video_ts_change(struct dptx *dptx, int stream)
{
	u32 reg;
	struct video_params *vparams;

	vparams = &dptx->vparams;

	dptx_dbg(dptx, "%s: ts calculate - tu:%d frac:%d init_threshold:%d trained:%d pix_clk:%d\n", __func__,
	       dptx->vparams.aver_bytes_per_tu,
	       dptx->vparams.aver_bytes_per_tu_frac,
	       dptx->vparams.init_threshold, dptx->link.trained, dptx->vparams.mdtd.pixel_clock);

	reg = dptx_read_reg(dptx, dptx->regs[DPTX],
			    DPTX_VIDEO_CONFIG5_N(stream));
	reg = reg & (~DPTX_VIDEO_CONFIG5_TU_MASK);
	reg = reg | (vparams->aver_bytes_per_tu << DPTX_VIDEO_CONFIG5_TU_SHIFT);
	if (dptx->mst) {
		reg = reg & (~DPTX_VIDEO_CONFIG5_TU_FRAC_MASK_MST);
		reg = reg | (vparams->aver_bytes_per_tu_frac
			     << DPTX_VIDEO_CONFIG5_TU_FRAC_SHIFT_MST);
	} else {
		reg = reg & (~DPTX_VIDEO_CONFIG5_TU_FRAC_MASK_SST);
		reg = reg | (vparams->aver_bytes_per_tu_frac
			     << DPTX_VIDEO_CONFIG5_TU_FRAC_SHIFT_SST);
	}
	reg = reg & (~DPTX_VIDEO_CONFIG5_INIT_THRESHOLD_MASK);
	reg = reg | (vparams->init_threshold
		     << DPTX_VIDEO_CONFIG5_INIT_THRESHOLD_SHIFT);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG5_N(stream),
		       reg);
}

void dptx_video_bpc_change(struct dptx *dptx, int stream)
{
	dptx_video_set_core_bpc(dptx, stream);
	dptx_video_set_sink_bpc(dptx, stream);
}

void dptx_video_set_core_bpc(struct dptx *dptx, int stream)
{
	u32 reg;
	u8 bpc_mapping = 0, bpc = 0;
	enum pixel_enc_type pix_enc;
	struct video_params *vparams;

	vparams = &dptx->vparams;
	bpc = vparams->bpc;
	pix_enc = vparams->pix_enc;

	reg = dptx_read_reg(dptx, dptx->regs[DPTX],
			    DPTX_VSAMPLE_CTRL_N(stream));
	reg &= ~DPTX_VSAMPLE_CTRL_VMAP_BPC_MASK;

	switch (pix_enc) {
	case RGB:
		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 0;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR444:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 5;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 6;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 7;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 8;
		break;
	case YCBCR422:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 9;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 10;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 11;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 12;
		break;
	case YCBCR420:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 13;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 14;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 15;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 16;
		break;
	case YONLY:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 17;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 18;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 19;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 20;
		break;
	case RAW:
		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 23;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 24;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 25;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 27;
		break;
	}

	reg |= (bpc_mapping << DPTX_VSAMPLE_CTRL_VMAP_BPC_SHIFT);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VSAMPLE_CTRL_N(stream),
		       reg);
}

void dptx_video_set_sink_col(struct dptx *dptx, int stream)
{
	u32 reg_msa2;
	u8 col_mapping;
	u8 colorimetry;
	u8 dynamic_range;
	struct video_params *vparams;
	enum pixel_enc_type pix_enc;

	vparams = &dptx->vparams;
	pix_enc = vparams->pix_enc;
	colorimetry = vparams->colorimetry;
	dynamic_range = vparams->dynamic_range;

	reg_msa2 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA2_N(stream));
	reg_msa2 &= ~DPTX_VIDEO_VMSA2_COL_MASK;

	col_mapping = 0;

	/* According to Table 2-94 of DisplayPort spec 1.3 */
	switch (pix_enc) {
	case RGB:
		if (dynamic_range == CEA)
			col_mapping = 4;
		else if (dynamic_range == VESA)
			col_mapping = 0;
		break;
	case YCBCR422:
		if (colorimetry == ITU601)
			col_mapping = 5;
		else if (colorimetry == ITU709)
			col_mapping = 13;
		break;
	case YCBCR444:
		if (colorimetry == ITU601)
			col_mapping = 6;
		else if (colorimetry == ITU709)
			col_mapping = 14;
		break;
	case RAW:
		col_mapping = 1;
		break;
	case YCBCR420:
	case YONLY:
		break;
	}

	reg_msa2 |= (col_mapping << DPTX_VIDEO_VMSA2_COL_SHIFT);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA2_N(stream),
		       reg_msa2);
}

void dptx_video_set_sink_bpc(struct dptx *dptx, int stream)
{
	u32 reg_msa2, reg_msa3;
	u8 bpc_mapping = 0, bpc = 0;
	struct video_params *vparams;
	enum pixel_enc_type pix_enc;

	vparams = &dptx->vparams;
	pix_enc = vparams->pix_enc;
	bpc = vparams->bpc;

	reg_msa2 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA2_N(stream));
	reg_msa3 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA3_N(stream));

	reg_msa2 &= ~DPTX_VIDEO_VMSA2_BPC_MASK;
	reg_msa3 &= ~DPTX_VIDEO_VMSA3_PIX_ENC_MASK;

	switch (pix_enc) {
	case RGB:

		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 0;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR444:

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR422:

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR420:
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_YCBCR420_SHIFT;
		break;
	case YONLY:
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT;

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case RAW:
		reg_msa3 |= (1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT);

		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 3;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 4;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 5;
		else if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 7;
		break;
	}

	reg_msa2 |= (bpc_mapping << DPTX_VIDEO_VMSA2_BPC_SHIFT);

	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA2_N(stream),
		       reg_msa2);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA3_N(stream),
		       reg_msa3);

	dptx_video_set_sink_col(dptx, stream);
}

void dptx_video_set_timing_info(struct dptx *dptx, int stream)
{
	u32 reg = 0;
	u8 vmode;

	struct video_params *vparams;
	struct dtd *mdtd;

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;
	vmode = vparams->mode;

	dptx_video_set_core_bpc(dptx, stream);
	reg = dptx_read_reg(dptx, dptx->regs[DPTX],
			    DPTX_VSAMPLE_CTRL_N(stream));
	reg &= ~DPTX_VSAMPLE_CTRL_MULTI_PIXEL_MASK;
	reg |= dptx->multipixel << DPTX_VSAMPLE_CTRL_MULTI_PIXEL_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VSAMPLE_CTRL_N(stream),
		       reg);

	reg = 0;
	if (mdtd->h_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_H_SYNC_POL_EN;
	if (mdtd->v_sync_polarity == 1)
		reg |= DPTX_POL_CTRL_V_SYNC_POL_EN;

	dptx_write_reg(dptx, dptx->regs[DPTX],
		       DPTX_VSAMPLE_POLARITY_CTRL_N(stream), reg);

	reg = 0;
	if (vparams->video_format == VCEA) {
		if (vmode == 5 || vmode == 6 || vmode == 7 || vmode == 10 ||
		    vmode == 11 || vmode == 20 || vmode == 21 || vmode == 22 ||
		    vmode == 39 || vmode == 25 || vmode == 26 || vmode == 40 ||
		    vmode == 44 || vmode == 45 || vmode == 46 || vmode == 50 ||
		    vmode == 51 || vmode == 54 || vmode == 55 || vmode == 58 ||
		    vmode == 59)
			reg |= DPTX_VIDEO_CONFIG1_IN_OSC_EN;
	}

	if (mdtd->interlaced == 1)
		reg |= DPTX_VIDEO_CONFIG1_O_IP_EN;

	reg |= mdtd->h_active << DPTX_VIDEO_H_ACTIVE_SHIFT;
	reg |= mdtd->h_blanking << DPTX_VIDEO_H_BLANK_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG1_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->v_active << DPTX_VIDEO_V_ACTIVE_SHIFT;
	reg |= mdtd->v_blanking << DPTX_VIDEO_V_BLANK_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG2_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->h_sync_offset << DPTX_VIDEO_H_FRONT_PORCH;
	reg |= mdtd->h_sync_pulse_width << DPTX_VIDEO_H_SYNC_WIDTH;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG3_N(stream),
		       reg);

	reg = 0;
	reg |= mdtd->v_sync_offset << DPTX_VIDEO_V_FRONT_PORCH;
	reg |= mdtd->v_sync_pulse_width << DPTX_VIDEO_V_SYNC_WIDTH;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_CONFIG4_N(stream),
		       reg);

	dptx_video_ts_change(dptx, stream);
}

void dptx_video_set_MSA(struct dptx *dptx, int stream)
{
	u32 reg;
	u32 reg_msa2, reg_msa3;
	u8 bpc_mapping = 0, bpc = 0;
	u8 col_mapping;
	u8 colorimetry;
	u8 dynamic_range;
	struct video_params *vparams;
	struct dtd *mdtd;
	enum pixel_enc_type pix_enc;

	vparams = &dptx->vparams;
	mdtd = &vparams->mdtd;
	vparams = &dptx->vparams;
	pix_enc = vparams->pix_enc;
	bpc = vparams->bpc;
	colorimetry = vparams->colorimetry;
	dynamic_range = vparams->dynamic_range;

	reg = 0;
	reg |= (mdtd->h_blanking - mdtd->h_sync_offset)
	       << DPTX_VIDEO_MSA1_H_START_SHIFT;
	reg |= (mdtd->v_blanking - mdtd->v_sync_offset)
	       << DPTX_VIDEO_MSA1_V_START_SHIFT;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA1_N(stream), reg);

	reg_msa2 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA2_N(stream));
	reg_msa3 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA3_N(stream));

	reg_msa2 &= ~DPTX_VIDEO_VMSA2_BPC_MASK;
	reg_msa3 &= ~DPTX_VIDEO_VMSA3_PIX_ENC_MASK;

	switch (pix_enc) {
	case RGB:

		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 0;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR444:

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR422:

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case YCBCR420:
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_YCBCR420_SHIFT;
		break;
	case YONLY:
		reg_msa3 |= 1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT;

		if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 2;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 3;
		if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 4;
		break;
	case RAW:
		reg_msa3 |= (1 << DPTX_VIDEO_VMSA3_PIX_ENC_SHIFT);

		if (bpc == COLOR_DEPTH_6)
			bpc_mapping = 1;
		else if (bpc == COLOR_DEPTH_8)
			bpc_mapping = 3;
		else if (bpc == COLOR_DEPTH_10)
			bpc_mapping = 4;
		else if (bpc == COLOR_DEPTH_12)
			bpc_mapping = 5;
		else if (bpc == COLOR_DEPTH_16)
			bpc_mapping = 7;
		break;
	}

	reg_msa2 |= (bpc_mapping << DPTX_VIDEO_VMSA2_BPC_SHIFT);

	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA2_N(stream),
		       reg_msa2);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA3_N(stream),
		       reg_msa3);

	reg_msa2 = dptx_read_reg(dptx, dptx->regs[DPTX],
				 DPTX_VIDEO_MSA2_N(stream));
	reg_msa2 &= ~DPTX_VIDEO_VMSA2_COL_MASK;

	col_mapping = 0;
	switch (pix_enc) {
	case RGB:
		if (dynamic_range == CEA)
			col_mapping = 4;
		else if (dynamic_range == VESA)
			col_mapping = 0;
		break;
	case YCBCR422:
		if (colorimetry == ITU601)
			col_mapping = 5;
		else if (colorimetry == ITU709)
			col_mapping = 13;
		break;
	case YCBCR444:
		if (colorimetry == ITU601)
			col_mapping = 6;
		else if (colorimetry == ITU709)
			col_mapping = 14;
		break;
	case RAW:
		col_mapping = 1;
		break;
	case YCBCR420:
	case YONLY:
		break;
	}

	reg_msa2 |= (col_mapping << DPTX_VIDEO_VMSA2_COL_SHIFT);
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VIDEO_MSA2_N(stream),
		       reg_msa2);
}

void dptx_disable_default_video_stream(struct dptx *dptx, int stream)
{
	u32 vsamplectrl;

	vsamplectrl = dptx_read_reg(dptx, dptx->regs[DPTX],
				    DPTX_VSAMPLE_CTRL_N(stream));
	vsamplectrl &= ~DPTX_VSAMPLE_CTRL_STREAM_EN;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VSAMPLE_CTRL_N(stream),
		       vsamplectrl);
}

void dptx_enable_default_video_stream(struct dptx *dptx, int stream)
{
	u32 vsamplectrl;

	vsamplectrl = dptx_read_reg(dptx, dptx->regs[DPTX],
				    DPTX_VSAMPLE_CTRL_N(stream));
	vsamplectrl |= DPTX_VSAMPLE_CTRL_STREAM_EN;
	dptx_write_reg(dptx, dptx->regs[DPTX], DPTX_VSAMPLE_CTRL_N(stream),
		       vsamplectrl);
}

void dptx_audio_params_reset(struct audio_params *params)
{
	params->iec_channel_numcl0 = 8;
	params->iec_channel_numcr0 = 4;
	params->use_lut = 1;
	params->iec_samp_freq = 3;
	params->iec_word_length = 11;
	params->iec_orig_samp_freq = 12;
	params->data_width = 24;
	params->num_channels = 2;
	params->inf_type = 0;
	params->ats_ver = 18;
	params->mute = 0;
}

void dptx_video_params_reset(struct dptx *dptx)
{
	struct video_params *params = &dptx->vparams;

	params->bpc = COLOR_DEPTH_8;
	params->pix_enc = RGB;
	params->mode = 1;
	params->colorimetry = ITU601;
	params->dynamic_range = CEA;
	params->video_format = VCEA;
	params->aver_bytes_per_tu = 29;
	params->aver_bytes_per_tu_frac = 7;
	params->init_threshold = 16;
	params->refresh_rate = 60000;
}

void bst_dptx_dtd_reset(struct dtd *mdtd)
{
	mdtd->pixel_repetition_input = 0;
	mdtd->pixel_clock = 0;
	mdtd->h_active = 0;
	mdtd->h_blanking = 0;
	mdtd->h_sync_offset = 0;
	mdtd->h_sync_pulse_width = 0;
	mdtd->h_image_size = 0;
	mdtd->v_active = 0;
	mdtd->v_blanking = 0;
	mdtd->v_sync_offset = 0;
	mdtd->v_sync_pulse_width = 0;
	mdtd->v_image_size = 0;
	mdtd->interlaced = 0;
	mdtd->v_sync_polarity = 0;
	mdtd->h_sync_polarity = 0;
}

int dptx_dtd_fill(struct dtd *mdtd, u8 code, u32 refresh_rate, u8 video_format)
{
	bst_dptx_dtd_reset(mdtd);

	mdtd->h_image_size = 16;
	mdtd->v_image_size = 9;

	if (video_format == VCEA) {
		switch (code) {
		case 1: /* 640x480p @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 640;
			mdtd->v_active = 480;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 10;
			mdtd->h_sync_pulse_width = 96;
			mdtd->v_sync_pulse_width = 2;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 25175;
			break;
		case 2: /* 720x480p @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 3: /* 720x480p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 480;
			mdtd->h_blanking = 138;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 9;
			mdtd->h_sync_pulse_width = 62;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 27000;
			break;
		case 69:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 4: /* 1280x720p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 370;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 110;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 5: /* 1920x1080i @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 540;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 74250;
			break;
		case 6: /* 720(1440)x480i @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 7: /* 720(1440)x480i @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 240;
			mdtd->h_blanking = 276;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 38;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 124;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 27000;
			break;
		case 8: /* 720(1440)x240p @ 59.826/60.054/59.886/60.115Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 9: /* 720(1440)x240p @59.826/60.054/59.886/60.115Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 240;
			mdtd->h_blanking = 276;
			mdtd->v_blanking = (refresh_rate == 59940) ? 22 : 23;
			mdtd->h_sync_offset = 38;
			mdtd->v_sync_offset = (refresh_rate == 59940) ? 4 : 5;
			mdtd->h_sync_pulse_width = 124;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 27000;
			break;
		case 10: /* 2880x480i @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 11: /* 2880x480i @ 59.94/60Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 240;
			mdtd->h_blanking = 552;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 76;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 248;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 54000;
			break;
		case 12: /* 2880x240p @ 59.826/60.054/59.886/60.115Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 13: /* 2880x240p @ 59.826/60.054/59.886/60.115Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 240;
			mdtd->h_blanking = 552;
			mdtd->v_blanking = (refresh_rate == 60054) ? 22 : 23;
			mdtd->h_sync_offset = 76;
			mdtd->v_sync_offset = (refresh_rate == 60054) ? 4 : 5;
			mdtd->h_sync_pulse_width = 248;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 14: /* 1440x480p @ 59.94/60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 15: /* 1440x480p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 480;
			mdtd->h_blanking = 276;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 32;
			mdtd->v_sync_offset = 9;
			mdtd->h_sync_pulse_width = 124;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 76:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 16: /* 1920x1080p @ 59.94/60Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 148500;
			break;
		case 17: /* 720x576p @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 18: /* 720x576p @ 50Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 576;
			mdtd->h_blanking = 144;
			mdtd->v_blanking = 49;
			mdtd->h_sync_offset = 12;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 64;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 27000;
			break;
		case 68:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 19: /* 1280x720p @ 50Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 700;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 440;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 20: /* 1920x1080i @ 50Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 540;
			mdtd->h_blanking = 720;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 528;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 74250;
			break;
		case 21: /* 720(1440)x576i @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 22: /* 720(1440)x576i @ 50Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 288;
			mdtd->h_blanking = 288;
			mdtd->v_blanking = 24;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 126;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 27000;
			break;
		case 23: /* 720(1440)x288p @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 24: /* 720(1440)x288p @ 50Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 288;
			mdtd->h_blanking = 288;
			mdtd->v_blanking =
				(refresh_rate == 50080) ?
					24 :
					((refresh_rate == 49920) ? 25 : 26);
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset =
				(refresh_rate == 50080) ?
					2 :
					((refresh_rate == 49920) ? 3 : 4);
			mdtd->h_sync_pulse_width = 126;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 27000;
			break;
		case 25: /* 2880x576i @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 26: /* 2880x576i @ 50Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 288;
			mdtd->h_blanking = 576;
			mdtd->v_blanking = 24;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 252;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 54000;
			break;
		case 27: /* 2880x288p @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 28: /* 2880x288p @ 50Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 288;
			mdtd->h_blanking = 576;
			mdtd->v_blanking =
				(refresh_rate == 50080) ?
					24 :
					((refresh_rate == 49920) ? 25 : 26);
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset =
				(refresh_rate == 50080) ?
					2 :
					((refresh_rate == 49920) ? 3 : 4);
			mdtd->h_sync_pulse_width = 252;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 29: /* 1440x576p @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 30: /* 1440x576p @ 50Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 576;
			mdtd->h_blanking = 288;
			mdtd->v_blanking = 49;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 75:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 31: /* 1920x1080p @ 50Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 720;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 528;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 148500;
			break;
		case 72:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 32: /* 1920x1080p @ 23.976/24Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 830;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 638;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 73:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 33: /* 1920x1080p @ 25Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 720;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 528;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 74:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 34: /* 1920x1080p @ 29.97/30Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 35: /* 2880x480p @ 60Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 36: /* 2880x480p @ 60Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 480;
			mdtd->h_blanking = 552;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 9;
			mdtd->h_sync_pulse_width = 248;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 108000;
			break;
		case 37: /* 2880x576p @ 50Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 38: /* 2880x576p @ 50Hz 16:9 */
			mdtd->h_active = 2880;
			mdtd->v_active = 576;
			mdtd->h_blanking = 576;
			mdtd->v_blanking = 49;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 256;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 108000;
			break;
		case 39: /* 1920x1080i (1250 total) @ 50Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 540;
			mdtd->h_blanking = 384;
			mdtd->v_blanking = 85;
			mdtd->h_sync_offset = 32;
			mdtd->v_sync_offset = 23;
			mdtd->h_sync_pulse_width = 168;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 72000;
			break;
		case 40: /* 1920x1080i @ 100Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 540;
			mdtd->h_blanking = 720;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 528;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 148500;
			break;
		case 70:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 41: /* 1280x720p @ 100Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 700;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 440;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 148500;
			break;
		case 42: /* 720x576p @ 100Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 43: /* 720x576p @ 100Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 576;
			mdtd->h_blanking = 144;
			mdtd->v_blanking = 49;
			mdtd->h_sync_offset = 12;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 64;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 44: /* 720(1440)x576i @ 100Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 45: /* 720(1440)x576i @ 100Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 288;
			mdtd->h_blanking = 288;
			mdtd->v_blanking = 24;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 126;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 54000;
			break;
		case 46: /* 1920x1080i @ 119.88/120Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 540;
			mdtd->h_blanking = 288;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 148500;
			break;
		case 71:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 47: /* 1280x720p @ 119.88/120Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 370;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 110;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 148500;
			break;
		case 48: /* 720x480p @ 119.88/120Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 49: /* 720x480p @ 119.88/120Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 480;
			mdtd->h_blanking = 138;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 9;
			mdtd->h_sync_pulse_width = 62;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 54000;
			break;
		case 50: /* 720(1440)x480i @ 119.88/120Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 51: /* 720(1440)x480i @ 119.88/120Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 240;
			mdtd->h_blanking = 276;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 38;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 124;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 54000;
			break;
		case 52: /* 720X576p @ 200Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 53: /* 720X576p @ 200Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 576;
			mdtd->h_blanking = 144;
			mdtd->v_blanking = 49;
			mdtd->h_sync_offset = 12;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 64;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 108000;
			break;
		case 54: /* 720(1440)x576i @ 200Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 55: /* 720(1440)x576i @ 200Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 288;
			mdtd->h_blanking = 288;
			mdtd->v_blanking = 24;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 126;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 108000;
			break;
		case 56: /* 720x480p @ 239.76/240Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 57: /* 720x480p @ 239.76/240Hz 16:9 */
			mdtd->h_active = 720;
			mdtd->v_active = 480;
			mdtd->h_blanking = 138;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 9;
			mdtd->h_sync_pulse_width = 62;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 108000;
			break;
		case 58: /* 720(1440)x480i @ 239.76/240Hz 4:3 */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 59: /* 720(1440)x480i @ 239.76/240Hz 16:9 */
			mdtd->h_active = 1440;
			mdtd->v_active = 240;
			mdtd->h_blanking = 276;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 38;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 124;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 1;
			mdtd->pixel_clock = 108000;
			break;
		case 65:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 60: /* 1280x720p @ 23.97/24Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 2020;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 1760;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 66:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 61: /* 1280x720p @ 25Hz 16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 2680;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 2420;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 67:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 62: /* 1280x720p @ 29.97/30Hz  16:9 */
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 2020;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 1760;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 74250;
			break;
		case 78:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 63: /* 1920x1080p @ 119.88/120Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 77:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 64: /* 1920x1080p @ 100Hz 16:9 */
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 720;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 528;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 79:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 1620;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 1360;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 80:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 1488;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 1228;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 81:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 960;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 700;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 82:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 520;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 260;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 82500;
			break;
		case 83:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 520;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 260;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 99000;
			break;
		case 84:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 320;
			mdtd->v_blanking = 105;
			mdtd->h_sync_offset = 60;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 165000;
			break;
		case 85:
			mdtd->h_active = 1680;
			mdtd->v_active = 720;
			mdtd->h_blanking = 320;
			mdtd->v_blanking = 105;
			mdtd->h_sync_offset = 60;
			mdtd->v_sync_offset = 5;
			mdtd->h_sync_pulse_width = 40;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 198000;
			break;
		case 86:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 1190;
			mdtd->v_blanking = 20;
			mdtd->h_sync_offset = 998;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 99000;
			break;
		case 87:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 640;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 448;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 90000;
			break;
		case 88:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 960;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 768;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 118800;
			break;
		case 89:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 740;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 548;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 185625;
			break;
		case 90:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 440;
			mdtd->v_blanking = 20;
			mdtd->h_sync_offset = 248;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 198000;
			break;
		case 91:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 410;
			mdtd->v_blanking = 170;
			mdtd->h_sync_offset = 218;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 371250;
			break;
		case 92:
			mdtd->h_active = 2560;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 740;
			mdtd->v_blanking = 170;
			mdtd->h_sync_offset = 548;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 495000;
			break;
		case 101:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1184;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 968;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 59400;
			break;
		case 100:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 304;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 99:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1184;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 968;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 102:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 304;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 103:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 93: /* 4k x 2k, 30Hz */
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1660;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 1276;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 104:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 94:
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1440;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 1056;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 105:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 95:
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 560;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 176;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		case 106:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 96:
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1440;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 1056;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 107:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			fallthrough;
		case 97:
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 560;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 176;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 594000;
			break;
		case 98:
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 1404;
			mdtd->v_blanking = 90;
			mdtd->h_sync_offset = 1020;
			mdtd->v_sync_offset = 8;
			mdtd->h_sync_pulse_width = 88;
			mdtd->v_sync_pulse_width = 10;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0;
			mdtd->pixel_clock = 297000;
			break;
		default:
			return false;
		}
	} else if (video_format == CVT) {
		switch (code) {
		case 1:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 640;
			mdtd->v_active = 480;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 20;
			mdtd->h_sync_offset = 8;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 8;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 23750;
			break;
		case 2:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 800;
			mdtd->v_active = 600;
			mdtd->h_blanking = 224;
			mdtd->v_blanking = 24;
			mdtd->h_sync_offset = 31;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 81;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 38250;
			break;
		case 3:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1024;
			mdtd->v_active = 768;
			mdtd->h_blanking = 304;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 104;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 63500;
			break;
		case 4:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 960;
			mdtd->h_blanking = 416;
			mdtd->v_blanking = 36;
			mdtd->h_sync_offset = 80;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 101250;
			break;
		case 5:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1400;
			mdtd->v_active = 1050;
			mdtd->h_blanking = 464;
			mdtd->v_blanking = 39;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 144;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 121750;
			break;
		case 6:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1600;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 560;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 112;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 68;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 161000;
			break;
		case 12:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 1024;
			mdtd->h_blanking = 432;
			mdtd->v_blanking = 39;
			mdtd->h_sync_offset = 80;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 136;
			mdtd->v_sync_pulse_width = 7;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 109000;
			break;
		case 13:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 768;
			mdtd->h_blanking = 384;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 7;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 79500;
			break;
		case 16:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 720;
			mdtd->h_blanking = 384;
			mdtd->v_blanking = 28;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 74500;
			break;
		case 17:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1360;
			mdtd->v_active = 768;
			mdtd->h_blanking = 416;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 72;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 136;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 84750;
			break;
		case 20:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 656;
			mdtd->v_blanking = 40;
			mdtd->h_sync_offset = 128;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 200;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 173000;
			break;
		case 22:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 2560;
			mdtd->v_active = 1440;
			mdtd->h_blanking = 928;
			mdtd->v_blanking = 53;
			mdtd->h_sync_offset = 192;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 272;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 312250;
			break;
		case 28:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 800;
			mdtd->h_blanking = 400;
			mdtd->v_blanking = 31;
			mdtd->h_sync_offset = 72;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 83500;
			break;
		case 34:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 672;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 136;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 200;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 193250;
			break;
		case 38:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 3840;
			mdtd->v_active = 2400;
			mdtd->h_blanking = 80;
			mdtd->v_blanking = 69;
			mdtd->h_sync_offset = 320;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 424;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 580128;
			break;
		case 40:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1600;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 35;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 124076;
			break;
		case 41:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 2048;
			mdtd->v_active = 1536;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 44;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 208000;
			break;
		default:
			return false;
		}
	} else if (video_format == DMT) {
		switch (code) {
		case 1: // HISilicon timing
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 3600;
			mdtd->v_active = 1800;
			mdtd->h_blanking = 120;
			mdtd->v_blanking = 128;
			mdtd->h_sync_offset = 20;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 20;
			mdtd->v_sync_pulse_width = 2;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 645500;
			break;
		case 2:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 3840;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 62;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 533000;
			break;
		case 4:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 640;
			mdtd->v_active = 480;
			mdtd->h_blanking = 144;
			mdtd->v_blanking = 29;
			mdtd->h_sync_offset = 8;
			mdtd->v_sync_offset = 2;
			mdtd->h_sync_pulse_width = 96;
			mdtd->v_sync_pulse_width = 2;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 25175;
			break;
		case 13:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 800;
			mdtd->v_active = 600;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 36;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 73250;
			break;
		case 14: /* 848x480p@60Hz */
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 848;
			mdtd->v_active = 480;
			mdtd->h_blanking = 240;
			mdtd->v_blanking = 37;
			mdtd->h_sync_offset = 16;
			mdtd->v_sync_offset = 6;
			mdtd->h_sync_pulse_width = 112;
			mdtd->v_sync_pulse_width = 8;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI)  */
			;
			mdtd->pixel_clock = 33750;
			break;
		case 22:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 768;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 22;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 7;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 68250;
			break;
		case 35:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 1024;
			mdtd->h_blanking = 408;
			mdtd->v_blanking = 42;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 112;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 108000;
			break;
		case 39:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1360;
			mdtd->v_active = 768;
			mdtd->h_blanking = 432;
			mdtd->v_blanking = 27;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 112;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 85500;
			break;
		case 40:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1360;
			mdtd->v_active = 768;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 148250;
			break;
		case 81:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1366;
			mdtd->v_active = 768;
			mdtd->h_blanking = 426;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 70;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 142;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 85500;
			break;
		case 86:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1366;
			mdtd->v_active = 768;
			mdtd->h_blanking = 134;
			mdtd->v_blanking = 32;
			mdtd->h_sync_offset = 14;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 56;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 72000;
			break;
		case 87:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 80;
			mdtd->v_blanking = 62;
			mdtd->h_sync_offset = 8;
			mdtd->v_sync_offset = 48;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 8;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 556744;
			break;
		case 88:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 4096;
			mdtd->v_active = 2160;
			mdtd->h_blanking = 80;
			mdtd->v_blanking = 62;
			mdtd->h_sync_offset = 8;
			mdtd->v_sync_offset = 48;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 8;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 556188;
			break;
		case 41:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1400;
			mdtd->v_active = 1050;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 101000;
			break;
		case 42:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1400;
			mdtd->v_active = 1050;
			mdtd->h_blanking = 464;
			mdtd->v_blanking = 39;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 144;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 121750;
			break;
		case 46:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1440;
			mdtd->v_active = 900;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 26;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 88750;
			break;
		case 47:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1440;
			mdtd->v_active = 900;
			mdtd->h_blanking = 464;
			mdtd->v_blanking = 34;
			mdtd->h_sync_offset = 80;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 152;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 106500;
			break;
		case 51:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1600;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 560;
			mdtd->v_blanking = 50;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 192;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 162000;
			break;
		case 57:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1680;
			mdtd->v_active = 1050;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 119000;
			break;
		case 58:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1680;
			mdtd->v_active = 1050;
			mdtd->h_blanking = 560;
			mdtd->v_blanking = 39;
			mdtd->h_sync_offset = 104;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 176;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 146250;
			break;
		case 68:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 35;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 154000;
			break;
		case 69:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1200;
			mdtd->h_blanking = 672;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 136;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 200;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 193250;
			break;
		case 82:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1080;
			mdtd->h_blanking = 280;
			mdtd->v_blanking = 45;
			mdtd->h_sync_offset = 88;
			mdtd->v_sync_offset = 4;
			mdtd->h_sync_pulse_width = 44;
			mdtd->v_sync_pulse_width = 5;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 148500;
			break;
		case 83:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1600;
			mdtd->v_active = 900;
			mdtd->h_blanking = 200;
			mdtd->v_blanking = 100;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 80;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 108000;
			break;
		case 9:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 800;
			mdtd->v_active = 600;
			mdtd->h_blanking = 256;
			mdtd->v_blanking = 28;
			mdtd->h_sync_offset = 40;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 4;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 40000;
			break;
		case 16:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1024;
			mdtd->v_active = 768;
			mdtd->h_blanking = 320;
			mdtd->v_blanking = 38;
			mdtd->h_sync_offset = 24;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 136;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 65000;
			break;
		case 23:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 768;
			mdtd->h_blanking = 384;
			mdtd->v_blanking = 30;
			mdtd->h_sync_offset = 64;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 128;
			mdtd->v_sync_pulse_width = 7;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 79500;
			break;
		case 62:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1792;
			mdtd->v_active = 1344;
			mdtd->h_blanking = 656;
			mdtd->v_blanking = 50;
			mdtd->h_sync_offset = 128;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 200;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 204750;
			break;
		case 32:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 960;
			mdtd->h_blanking = 520;
			mdtd->v_blanking = 40;
			mdtd->h_sync_offset = 96;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 112;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 108000;
			break;
		case 73:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1920;
			mdtd->v_active = 1440;
			mdtd->h_blanking = 680;
			mdtd->v_blanking = 60;
			mdtd->h_sync_offset = 128;
			mdtd->v_sync_offset = 1;
			mdtd->h_sync_pulse_width = 208;
			mdtd->v_sync_pulse_width = 3;
			mdtd->h_sync_polarity = 0;
			mdtd->v_sync_polarity = 1;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 234000;
			break;
		case 27:
			mdtd->h_image_size = 4;
			mdtd->v_image_size = 3;
			mdtd->h_active = 1280;
			mdtd->v_active = 800;
			mdtd->h_blanking = 160;
			mdtd->v_blanking = 23;
			mdtd->h_sync_offset = 48;
			mdtd->v_sync_offset = 3;
			mdtd->h_sync_pulse_width = 32;
			mdtd->v_sync_pulse_width = 6;
			mdtd->h_sync_polarity = 1;
			mdtd->v_sync_polarity = 0;
			mdtd->interlaced = 0; /* (progressive_nI) */
			mdtd->pixel_clock = 71000;
			break;
		default:
			return false;
		}
	}

	return true;
}
