// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_print.h>
#include "dptx_drv.h"
#include "dptx_csr.h"
#include "api/api.h"
#include "bst_disp_conn.h"

struct dptx_dt_info {
	u8 pixel_encode;
	u8 video_bpc;
	u8 colorimetry;
	u8 dynamic_range;
	u8 lane_num;
	u32 lane_speed;
};

struct drm_dptx {
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_display_mode mode;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct videomode *vm;
	struct edid *drm_edid;
	struct dptx_dt_info dt_info;
	bool sink_has_audio;
	struct dptx *dptx;
	struct mutex drm_lock;
};


static struct drm_dptx *__dptx_drm_handle;

#define DEFAULT_STREAM 0
#define DPTX_PORT_IN   0
#define DPRX_PORT_OUT  1
#define EDID_DETAILED_TIMINGS 4

static int video_param_update_from_videomode(struct drm_dptx *drm_handle,
					     struct videomode *vm)
{
	struct dptx *dptx = drm_handle->dptx;
	struct video_params *params = &dptx->vparams;
	struct dtd *mdtd = &params->mdtd;

	mdtd->pixel_clock = vm->pixelclock / 1000; // to KHz
	mdtd->h_active = vm->hactive;
	mdtd->h_blanking = vm->hfront_porch + vm->hsync_len + vm->hback_porch;
	mdtd->h_sync_offset = vm->hfront_porch;
	mdtd->h_sync_pulse_width = vm->hsync_len;

	mdtd->v_active = vm->vactive;
	mdtd->v_blanking = vm->vfront_porch + vm->vsync_len + vm->vback_porch;
	mdtd->v_sync_offset = vm->vfront_porch;
	mdtd->v_sync_pulse_width = vm->vsync_len;

	if (vm->flags & DISPLAY_FLAGS_HSYNC_HIGH) {
		mdtd->h_sync_polarity = 1;
	} else
		mdtd->h_sync_polarity = 0;

	if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH) {
		mdtd->v_sync_polarity = 1;
	} else
		mdtd->v_sync_polarity = 0;


	mdtd->pixel_repetition_input = 0;
	mdtd->interlaced = 0;

	dev_dbg(dptx->dev, "mdtd pixel_clock=%d,hpol=%d,vpol=%d\n",
		 mdtd->pixel_clock, mdtd->h_sync_polarity,
		 mdtd->v_sync_polarity);
	dev_dbg(dptx->dev, "mdtd timing H[%d,%d,%d,%d,%d,%d]\n",
		 mdtd->h_active, mdtd->h_blanking, mdtd->h_sync_offset,
		 mdtd->h_sync_pulse_width, mdtd->h_sync_polarity,
		 mdtd->h_image_size);
	dev_dbg(dptx->dev, "mdtd timing V[%d,%d,%d,%d,%d,%d]\n",
		 mdtd->v_active, mdtd->v_blanking, mdtd->v_sync_offset,
		 mdtd->v_sync_pulse_width, mdtd->v_sync_polarity,
		 mdtd->v_image_size);

	return 0;
}

static int dptx_param_update_from_dt_info(struct drm_dptx *drm_handle)
{
	struct dptx *dptx = drm_handle->dptx;
	struct video_params *params = &dptx->vparams;
	struct dptx_link *link = &dptx->link;
	struct dptx_dt_info *dt_info = &drm_handle->dt_info;

	link->lanes = dt_info->lane_num;
	switch (dt_info->lane_speed) {
	case 1620:
		link->rate = DPTX_PHYIF_CTRL_RATE_RBR;
		break;
	case 2700:
		link->rate = DPTX_PHYIF_CTRL_RATE_HBR;
		break;
	case 5400:
		link->rate = DPTX_PHYIF_CTRL_RATE_HBR2;
		break;
	case 8100:
		link->rate = DPTX_PHYIF_CTRL_RATE_HBR3;
		break;
	default:
		dev_err(dptx->dev, "not support link rate=%d\n",
			dt_info->lane_speed);
		return -1;
	}
	dptx->max_lanes = link->lanes;
	dptx->max_rate = link->rate;
	if (dt_info->video_bpc) {
		params->bpc = dt_info->video_bpc;
		params->pix_enc = dt_info->pixel_encode;
	}
	params->colorimetry = dt_info->colorimetry;
	params->dynamic_range = dt_info->dynamic_range;

	dev_dbg(dptx->dev, "video dts info[%d,%d,%d,%d,%d,%d]\n",
		 dt_info->lane_speed, dt_info->lane_num, dt_info->video_bpc,
		 dt_info->pixel_encode, dt_info->colorimetry,
		 dt_info->dynamic_range);

	return 0;
}

static int dptx_parse_dt_params(struct drm_dptx *drm_handle)
{
	struct device *dev = drm_handle->dptx->dev;
	struct dptx *dptx = drm_handle->dptx;
	struct device_node *np = dev->of_node;
	struct dptx_dt_info *dt_info = &drm_handle->dt_info;
	u32 pixel_encode;
	u32 video_bpc;
	u32 colorimetry;
	u32 dynamic_range;
	u32 is_edp;
	u32 lane_num;
	u32 lane_speed;
	u32 bypass_train;
	u32 hpd;
	u32 ssc_en;
	int ret = 0;

	ret = of_property_read_u32(np, "lane-num", &lane_num);
	ret |= of_property_read_u32(np, "lane-speed", &lane_speed);
	ret |= of_property_read_u32(np, "colorimetry", &colorimetry);
	ret |= of_property_read_u32(np, "dynamic-range", &dynamic_range);
	if (!ret) {
		dt_info->lane_num = lane_num;
		dt_info->lane_speed = lane_speed;
		dt_info->colorimetry = colorimetry;
		dt_info->dynamic_range = dynamic_range;
	} else {
		dev_err(dev, "parse device tree of dptx error, ret=%d", ret);
		return -1;
	}

	ret = of_property_read_u32(np, "video-bpc", &video_bpc);
	ret |= of_property_read_u32(np, "pixel-encode", &pixel_encode);
	if (!ret) {
		dt_info->video_bpc = video_bpc;
		dt_info->pixel_encode = pixel_encode;
	} else {
		dt_info->video_bpc = 0;
		dt_info->pixel_encode = 0;
		dev_info(dev, "dts no bpc/pixel_enc, load from panel later");
		return 0;
	}

	ret = of_property_read_u32(np, "dp-interface", &is_edp);
	if (!ret)
		dptx->edp = is_edp;

	ret = of_property_read_u32(np, "bypass-training", &bypass_train);
	if (!ret) {
		dptx->link.trained = bypass_train;
		dptx->link.bypass_training = bypass_train;
	}
	ret = of_property_read_u32(np, "force-hpd", &hpd);
	if (!ret)
		dptx->force_hpd = hpd;

	ret = of_property_read_u32(np, "ssc-enable", &ssc_en);
	if (!ret)
		dptx->ssc_en = ssc_en ? true : false;

	return 0;
}

struct dptx *dptx_get_handle(void)
{
	return __dptx_drm_handle->dptx;
}

void dptx_notify(struct dptx *dptx)
{
	wake_up_interruptible(&dptx->waitq);
}

void dptx_notify_shutdown(struct dptx *dptx)
{
	atomic_set(&dptx->shutdown, 1);
	dptx_notify(dptx);
}

static inline struct drm_dptx *connector_to_drm_dptx(struct drm_connector *c)
{
	return container_of(c, struct drm_dptx, connector);
}

static inline struct drm_dptx *encoder_to_drm_dptx(struct drm_encoder *e)
{
	return container_of(e, struct drm_dptx, encoder);
}

static inline struct drm_dptx *mode_to_drm_dptx(struct drm_display_mode *m)
{
	return container_of(m, struct drm_dptx, mode);
}

static enum drm_connector_status
dptx_connector_detect(struct drm_connector *connector, bool force)
{
	struct drm_dptx *drm_handle = connector_to_drm_dptx(connector);
	enum drm_connector_status status = connector_status_disconnected;
	struct dptx *dptx = drm_handle->dptx;

	mutex_lock(&drm_handle->drm_lock);
	if (atomic_read(&dptx->c_connect)) {
		status = connector_status_connected;
		dptx_dbg(dptx, "connector detect status connected\n");
	} else {
		status = connector_status_disconnected;
		dptx_dbg(dptx, "connector detect status disconnected\n");
	}

	mutex_unlock(&drm_handle->drm_lock);

	return status;
}

static void dptx_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs dptx_connector_funcs = {
	.detect = dptx_connector_detect,
	.destroy = dptx_connector_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int dptx_get_edid_block(void *data, u8 *edid, unsigned int block,
			       size_t length)
{
	struct drm_dptx *drm_handle = data;
	struct dptx *dptx = drm_handle->dptx;
	int ret;

	ret = dptx_read_edid(dptx);
	if (ret)
		DRM_DEV_ERROR(dptx->dev, "get block[%d] edid failed: %d\n",
			      block, ret);

	ret = dptx_check_edid(dptx);

	if (length > EDID_LENGTH)
		return -EINVAL;

	if (block > 2) {
		DRM_ERROR("Error, virt dp edid block num = %d", block);
		return -EINVAL;
	}

	if (!ret)
		memcpy(edid, &dptx->edid[EDID_LENGTH * block], length);

	return ret;
}

static int dptx_get_sink_capability(struct drm_dptx *drm_handle)
{
	struct dptx *dptx = drm_handle->dptx;
	int ret;
	u8 revision;

	if (!atomic_read(&dptx->c_connect))
		return -ENODEV;

	ret = dptx_read_dpcd(dptx, DP_DPCD_REV, &revision);
	if (ret) {
		DRM_DEV_ERROR(dptx->dev, "Failed to get caps %d\n", ret);
		return ret;
	}
	dptx_info(dptx, "DP Revision %x.%x\n", (revision & 0xf0) >> 4,
		  revision & 0xf);

	if (drm_handle->drm_edid)
		kfree(drm_handle->drm_edid);

	drm_handle->drm_edid = drm_do_get_edid(&drm_handle->connector,
					       dptx_get_edid_block, drm_handle);
	return 0;
}

static bool edid_preferred_mode_check(struct edid *edid, struct detailed_timing *timing, struct drm_display_mode *mode) {
	struct detailed_pixel_timing *pt = &timing->data.pixel_data;
	unsigned hactive = (pt->hactive_hblank_hi & 0xf0) << 4 | pt->hactive_lo;
	unsigned vactive = (pt->vactive_vblank_hi & 0xf0) << 4 | pt->vactive_lo;
	unsigned hblank = (pt->hactive_hblank_hi & 0xf) << 8 | pt->hblank_lo;
	unsigned vblank = (pt->vactive_vblank_hi & 0xf) << 8 | pt->vblank_lo;
	unsigned hsync_offset = (pt->hsync_vsync_offset_pulse_width_hi & 0xc0) << 2 | pt->hsync_offset_lo;
	unsigned hsync_pulse_width = (pt->hsync_vsync_offset_pulse_width_hi & 0x30) << 4 | pt->hsync_pulse_width_lo;
	unsigned vsync_offset = (pt->hsync_vsync_offset_pulse_width_hi & 0xc) << 2 | pt->vsync_offset_pulse_width_lo >> 4;
	unsigned vsync_pulse_width = (pt->hsync_vsync_offset_pulse_width_hi & 0x3) << 4 | (pt->vsync_offset_pulse_width_lo & 0xf);

	/* ignore tiny modes */
	if (hactive < 64 || vactive < 64)
		return false;

	if (pt->misc & DRM_EDID_PT_STEREO) {
		DRM_DEBUG_KMS("stereo mode not supported\n");
		return false;
	}
	if (!(pt->misc & DRM_EDID_PT_SEPARATE_SYNC)) {
		DRM_DEBUG_KMS("composite sync not supported\n");
	}

	/* it is incorrect if hsync/vsync width is zero */
	if (!hsync_pulse_width || !vsync_pulse_width) {
		DRM_DEBUG_KMS("Incorrect Detailed timing. "
				"Wrong Hsync/Vsync pulse width\n");
		return false;
	}
	mode->clock = le16_to_cpu(timing->pixel_clock) * 10;

	mode->hdisplay = hactive;
	mode->hsync_start = mode->hdisplay + hsync_offset;
	mode->hsync_end = mode->hsync_start + hsync_pulse_width;
	mode->htotal = mode->hdisplay + hblank;

	mode->vdisplay = vactive;
	mode->vsync_start = mode->vdisplay + vsync_offset;
	mode->vsync_end = mode->vsync_start + vsync_pulse_width;
	mode->vtotal = mode->vdisplay + vblank;

	return true;
}

static bool edid_preferred_timing_fixup(u8 *raw_edid, struct drm_connector *connector)
{
	struct drm_display_mode mode = {0};
	uint8_t *preferred_vic = &raw_edid[0x36];
	struct edid *edid = (struct edid *)raw_edid;
	bool illegal = false;
	u8 *d;
	int i = 0;

	for (i = 0; i < EDID_DETAILED_TIMINGS; i++) {
		d = (u8 *)&(edid->detailed_timings[i]);
		if (!(d[0] != 0x00 || d[1] != 0x00))
			continue;
		if (edid_preferred_mode_check(edid,
					      &(edid->detailed_timings[i]), &mode))
			break;
	}

	/* Some EDIDs have bogus h/vtotal values */
	if (mode.hsync_end > mode.htotal)
		illegal = true;
	if (mode.vsync_end > mode.vtotal)
		illegal = true;
	if (mode.hsync_start > mode.hsync_end)
		illegal = true;
	if (mode.vsync_start > mode.vsync_end)
		illegal = true;

	if (illegal && mode.hdisplay == 1920 && mode.vdisplay == 720 && mode.htotal == 2080 && mode.hsync_start == 1990
			&& mode.hsync_end == 2133 && mode.vtotal == 750
			&& mode.vsync_end == 726 && mode.vsync_start == 723) {
		mode.htotal = 2080;
		mode.hsync_start = 1990;
		mode.hsync_end = 2033;
		bst_conn_edid_byte_gen(&preferred_vic[4], 4, 4, &preferred_vic[2], 0, 8,
					mode.hdisplay);
		bst_conn_edid_byte_gen(&preferred_vic[4], 0, 4, &preferred_vic[3], 0, 8,
					mode.htotal - mode.hdisplay);
		bst_conn_edid_byte_gen(&preferred_vic[11], 6, 2, &preferred_vic[8], 0,
					8, mode.hsync_start - mode.hdisplay);
		bst_conn_edid_byte_gen(&preferred_vic[11], 4, 2, &preferred_vic[9], 0,
					8, mode.hsync_end - mode.hsync_start);
		raw_edid[127] = bst_edid_block_checksum(raw_edid);
		illegal = false;
	}

	return illegal;
}

static int dptx_connector_get_modes(struct drm_connector *connector)
{
	struct drm_dptx *drm_handle = connector_to_drm_dptx(connector);
	struct dptx *dptx = drm_handle->dptx;
	struct drm_display_info *disp_info = &connector->display_info;
	struct dptx_dt_info *dt_info = &drm_handle->dt_info;
	struct drm_panel *panel = drm_handle->panel;
	struct video_params *params = &dptx->vparams;
	struct edid *drm_edid;
	int mode_num = 0;

	mutex_lock(&drm_handle->drm_lock);
	if (atomic_read(&dptx->c_connect)) {
		dptx_get_sink_capability(drm_handle);
	}

	drm_edid = drm_handle->drm_edid;
	if (drm_edid) {
		drm_handle->sink_has_audio = drm_detect_monitor_audio(drm_edid);
		drm_connector_update_edid_property(connector, drm_edid);
		dptx_dbg(dptx, "got edid: width[%d] x height[%d]\n",
			  drm_edid->width_cm, drm_edid->height_cm);
	}

	if (panel) {
		mode_num += drm_panel_get_modes(panel, connector);
		params->bpc = disp_info->bpc;
		switch (disp_info->color_formats) {
		case DRM_COLOR_FORMAT_RGB444:
			params->pix_enc = RGB;
			break;
		case DRM_COLOR_FORMAT_YCBCR420:
			params->pix_enc = YCBCR420;
			break;
		case DRM_COLOR_FORMAT_YCBCR422:
			params->pix_enc = YCBCR422;
			break;
		case DRM_COLOR_FORMAT_YCBCR444:
			params->pix_enc = YCBCR444;
			break;
		default:
			dptx_info(dptx, "invalid pixel_encode value=%d\n",
				  disp_info->color_formats);
		}
		params->mdtd.h_image_size = disp_info->width_mm;
		params->mdtd.v_image_size = disp_info->height_mm;
		dptx_info(dptx,
			  "DPTX set color format=%d,bpc=%d,size=%dx%d(mm)\n",
			  disp_info->color_formats, disp_info->bpc,
			  disp_info->width_mm, disp_info->height_mm);
		if (mode_num > 0)
			goto out;
	}

	if (drm_edid) {
		if (edid_preferred_timing_fixup((u8 *)drm_edid, connector)) {
			DRM_WARN_ONCE("Warning! EDID preferred timing is illegal, please configure DTS panel-timing!\n");
		} else {
			mode_num += drm_add_edid_modes(connector, drm_edid);
		}
	} else {
		DRM_WARN_ONCE("Warning! EDID is NULL, please configure DTS panel-timing!\n");
	}

	if (!panel && dt_info->video_bpc) {
		disp_info->bpc = dt_info->video_bpc;
		switch (dt_info->pixel_encode) {
		case RGB:
			disp_info->color_formats = DRM_COLOR_FORMAT_RGB444;
			break;
		case YCBCR420:
			disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR420;
			break;
		case YCBCR422:
			disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR422;
			break;
		case YCBCR444:
			disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR444;
			break;
		case YONLY:
		case RAW:
			dptx_info(dptx, "not support YONLY/RAW yet\n");
			break;
		default:
			dptx_info(dptx, "invalid pixel_encode value\n");
		}
	} else {
		DRM_DEV_ERROR(dptx->dev,
			      "no bpc and pix_enc found force to RGB24\n");
		disp_info->color_formats = DRM_COLOR_FORMAT_RGB444;
		disp_info->bpc = 8;
	}
out:
	mutex_unlock(&drm_handle->drm_lock);

	return mode_num;
}

static int dptx_connector_mode_valid(struct drm_connector *connector,
				     struct drm_display_mode *mode)
{
	struct drm_dptx *drm_handle = connector_to_drm_dptx(connector);
	struct dptx *dptx = drm_handle->dptx;
	struct drm_display_info *disp_info = &connector->display_info;
	u32 requested, actual, rate = DP_LINK_BW_1_62;
	u8 lanes;

	if (!atomic_read(&dptx->c_connect))
		return MODE_BAD;

	lanes = dptx->link.lanes;
	switch (dptx->link.rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		rate = DP_LINK_BW_1_62;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		rate = DP_LINK_BW_2_7;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		rate = DP_LINK_BW_5_4;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		rate = DP_LINK_BW_8_1;
		break;
	}

	requested = mode->clock * disp_info->bpc * 3 / 1000;
	rate = drm_dp_bw_code_to_link_rate(rate);
	actual = rate * lanes / 100;
	actual = actual * 8 / 10;

	if (requested > actual) {
		DRM_DEV_ERROR(
			dptx->dev,
			"dptx bandwidth: requested=%d,actual=%d,clock=%d\n",
			requested, actual, mode->clock);
		return MODE_CLOCK_HIGH;
	}

	return MODE_OK;
}

static struct drm_connector_helper_funcs dptx_connector_helper_funcs = {
	.get_modes = dptx_connector_get_modes,
	.mode_valid = dptx_connector_mode_valid,
};

static void dptx_encoder_mode_set(struct drm_encoder *encoder,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted)
{
	struct drm_dptx *drm_handle = encoder_to_drm_dptx(encoder);
	struct dptx *dptx = drm_handle->dptx;
	struct video_params *params = &dptx->vparams;
	struct dptx_link *link = &dptx->link;
	struct videomode vm = { 0 };
	int ret = 0;

	mutex_lock(&drm_handle->drm_lock);

	drm_display_mode_to_videomode(adjusted, &vm);

	video_param_update_from_videomode(drm_handle, &vm);

	ret = dptx_video_ts_calculate(dptx, link->lanes, link->rate, params->bpc,
				params->pix_enc, vm.pixelclock / 1000);
	if (ret < 0)
		dptx_err(dptx, "TU size is wrong!\n");

	memcpy(&drm_handle->mode, adjusted, sizeof(*adjusted));

	mutex_unlock(&drm_handle->drm_lock);
}

static void dptx_encoder_enable(struct drm_encoder *encoder)
{
	struct drm_dptx *drm_handle = encoder_to_drm_dptx(encoder);
	struct dptx *dptx = drm_handle->dptx;
	struct bst_dev *mdev = encoder->dev->dev_private;
	int ret;

	if(mdev->resume) {
		dptx_csr_reset(dptx);
		if (!dptx_check_dptx_id(dptx)) {
			DRM_DEV_ERROR(dptx->dev, "DPTX_ID not match!!\n");
			return;
		}

		dptx_global_intr_dis(dptx);
		dptx_csr_func_irq_dis_all(dptx);

		dptx_csr_func_irq_en_dptx(dptx);
		dptx_init_remote_source(dptx);
		dptx_init_hwparams(dptx);
	}

	mutex_lock(&drm_handle->drm_lock);
	if (drm_handle->panel) {
		drm_panel_prepare(drm_handle->panel);
		drm_panel_enable(drm_handle->panel);
	}
	if (!dptx->link.trained && !dptx->link.bypass_training) {
		ret = dptx_link_retrain(dptx, dptx->link.rate,
					dptx->link.lanes);
		if (ret) {
			DRM_DEV_ERROR(dptx->dev, "Failed link train %d\n", ret);
			goto out;
		}
	}

	ret = dptx_xmit_enable(dptx, false);
	if (ret)
		goto out;

	ret = dptx_csr_set_video_ctrl(dptx);
	if (ret)
		goto out;

	ret = dptx_xmit_enable(dptx, true);
	if (ret)
		goto out;

	dptx_disable_default_video_stream(dptx, DEFAULT_STREAM);

	dptx_video_set_MSA(dptx, DEFAULT_STREAM);

	dptx_video_core_config(dptx, DEFAULT_STREAM);

	if (dptx->vparams.pix_enc == YCBCR420)
		dptx_vsd_ycbcr420_send(dptx, 1);

	dptx_enable_default_video_stream(dptx, DEFAULT_STREAM);

	dev_dbg(dptx->dev, "CCTL=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], CCTL));
	dev_dbg(dptx->dev, "PHYIF_CTRL=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], PHYIF_CTRL));
	dev_dbg(dptx->dev, "VIDEO_MSA1=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_MSA1_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_MSA2=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_MSA2_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_MSA3=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_MSA3_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VINPUT_POLARITY=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VSAMPLE_POLARITY_CTRL_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "HBLANK_INTERVAL=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], VIDEO_HBLANK_INTERVAL));
	dev_dbg(dptx->dev, "VIDEO_CONFIG1=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_CONFIG1_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_CONFIG2=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_CONFIG2_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_CONFIG3=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_CONFIG3_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_CONFIG4=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_CONFIG4_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "VIDEO_CONFIG5=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX],
			       DPTX_VIDEO_CONFIG5_N(DEFAULT_STREAM)));
	dev_dbg(dptx->dev, "SDP_HORIZONTAL_CTRL=0x%08x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], SDP_HORIZONTAL_CTRL));
	dev_dbg(dptx->dev, "SDP_VERTICAL_CTRL=0x%08x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], SDP_VERTICAL_CTRL));
	dev_dbg(dptx->dev, "VSAMPLE_CTRL=0x%x\n",
		 dptx_read_reg(dptx, dptx->regs[DPTX], VSAMPLE_CTRL));

out:
	mutex_unlock(&drm_handle->drm_lock);
}

static void dptx_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_dptx *drm_handle = encoder_to_drm_dptx(encoder);
	struct dptx *dptx = drm_handle->dptx;

	mutex_lock(&drm_handle->drm_lock);
	dptx_disable_default_video_stream(dptx, DEFAULT_STREAM);
	if (dptx->link.trained) {
		dptx_disconnect_link(dptx);
	}
	if (drm_handle->panel) {
		drm_panel_disable(drm_handle->panel);
		drm_panel_unprepare(drm_handle->panel);
	}
	mutex_unlock(&drm_handle->drm_lock);
}

static int dptx_encoder_atomic_check(struct drm_encoder *encoder,
				     struct drm_crtc_state *crtc_state,
				     struct drm_connector_state *conn_state)
{
	/* do nothing now*/
	return 0;
}

static const struct drm_encoder_helper_funcs dptx_encoder_helper_funcs = {
	.mode_set = dptx_encoder_mode_set,
	.enable = dptx_encoder_enable,
	.disable = dptx_encoder_disable,
	.atomic_check = dptx_encoder_atomic_check,
};

static int dptx_drm_register(struct device *dev, struct drm_device *drm,
			     struct drm_dptx *drm_dptx)
{
	struct drm_encoder *encoder = &drm_dptx->encoder;
	struct drm_connector *connector = &drm_dptx->connector;
	int ret;

	ret = drm_of_find_panel_or_bridge(dev->of_node, DPRX_PORT_OUT, 0,
					  &drm_dptx->panel, NULL);
	if (ret) {
		dev_info(drm->dev, "not a valid panel or bridge found!!");
	}

	if (drm_dptx->panel) {
		dev_info(drm->dev, "a valid panel found!!");
	}

	encoder->possible_crtcs =
		drm_of_find_possible_crtcs(drm, drm_dptx->dptx->dev->of_node);
	/* FIXME: cloning support not clear, disable it all for now */
	encoder->possible_clones = 0;

	ret = drm_connector_init(drm, connector, &dptx_connector_funcs,
				 drm_dptx->dptx->edp ? DRM_MODE_CONNECTOR_eDP : DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		DRM_ERROR("failed to init DPTX connector\n");
		goto err_conn_init;
	}
	drm_connector_helper_add(connector, &dptx_connector_helper_funcs);

	ret = drm_connector_register(connector);
	if (ret < 0) {
		DRM_ERROR("failed to register DPTX connector\n");
		goto err_conn_register;
	}
	/* set the connector's dpms to OFF so that
	 * drm_helper_connector_dpms() won't return
	 * immediately since the current state is ON
	 * at this point.
	 */
	drm_dptx->connector.dpms = DRM_MODE_DPMS_OFF;
	drm_dptx->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_TMDS);
	if (ret < 0) {
		DRM_ERROR("failed to init DPTX encoder\n");
		goto err_encoder_init;
	}
	drm_encoder_helper_add(encoder, &dptx_encoder_helper_funcs);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret < 0) {
		DRM_ERROR("failed to init DPTX encoder\n");
		goto err_attach;
	}
	drm_mode_config_reset(drm);

	return 0;

err_attach:
err_encoder_init:
	drm_encoder_cleanup(encoder);
err_conn_init:
err_conn_register:
	drm_connector_cleanup(connector);

	return ret;
}

static int bst_dptx_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct drm_dptx *drm_handle = dev_get_drvdata(dev);
	struct dptx *dptx;
	int ret;

	if (drm_handle && drm && drm_handle->dptx) {
		dptx = drm_handle->dptx;
		if (dptx_regmap_fields_init(dptx)) {
			dev_err(dev, "Failed to init register layout map\n");
			return -1;
		}

		dptx_csr_reset(dptx);
		if (!dptx_check_dptx_id(dptx)) {
			dev_err(dev, "DPTX_ID not match!!\n");
			return -1;
		}

		dptx_global_intr_dis(dptx);
		dptx_csr_func_irq_dis_all(dptx);
		ret = devm_request_threaded_irq(dptx->dev,
						dptx->irq[MAIN_FUNC_IRQ],
						dptx_irq, dptx_threaded_irq,
						IRQF_SHARED | IRQ_LEVEL,
						"bst_dptx_main_handler", dptx);
		if (ret) {
			dev_err(dev, "Request for irq %d failed\n",
				dptx->irq[MAIN_FUNC_IRQ]);
			return ret;
		}

		dptx_csr_func_irq_en_dptx(dptx);
		dptx_init_remote_source(dptx);
		dptx_init_hwparams(dptx);
		ret = dptx_core_init(dptx);
		if (ret)
			return ret;

		ret = dptx_param_update_from_dt_info(drm_handle);
		if (ret)
			return ret;

		dptx_drm_register(dev, drm, drm_handle);
	}

	dev_info(dev, "bind dptx driver to master okay!!\n");

	return 0;
}

static void bst_dptx_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct drm_dptx *drm_handle = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &drm_handle->encoder;
	struct drm_connector *connector = &drm_handle->connector;

	dptx_encoder_disable(encoder);
	encoder->funcs->destroy(encoder);
	connector->funcs->destroy(connector);
	if (drm_handle->dptx && drm_handle->dptx->host_dpu)
		put_device(drm_handle->dptx->host_dpu);
}

static const struct component_ops dptx_component_ops = {
	.bind = bst_dptx_bind,
	.unbind = bst_dptx_unbind,
};

static int dptx_resource_get(struct dptx *dptx, struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;

	/*  Get MEM resources of DPTX IP regs and
	 *  Map regfields to memory resource
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, DPTX);
	if (!res) {
		dev_err(dev, "Failed to get memory resource %d\n", DPTX);
		return -ENODEV;
	}
	dptx->base[DPTX] = devm_ioremap_resource(dev, res);
	if (IS_ERR(dptx->base[DPTX])) {
		dev_err(dev, "Failed to map memory resource\n");
		return PTR_ERR(dptx->base[DPTX]);
	}
	dptx->regs[DPTX] = devm_regmap_init_mmio(dev, dptx->base[DPTX],
						 &bst_dptx_regmap_cfg);
	if (IS_ERR(dptx->regs)) {
		dev_err(dev, "Failed to create BST_DPTX regmap\n");
		return PTR_ERR(dptx->regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, DPTX_CSR);
	if (!res) {
		dev_err(dev, "Failed to get memory resource %d\n", DPTX_CSR);
		return -ENODEV;
	}
	dptx->base[DPTX_CSR] = devm_ioremap_resource(dev, res);
	if (IS_ERR(dptx->base[DPTX_CSR])) {
		dev_err(dev, "Failed to map memory resource\n");
		return PTR_ERR(dptx->base[DPTX_CSR]);
	}
	dptx->regs[DPTX_CSR] = devm_regmap_init_mmio(dev, dptx->base[DPTX_CSR],
						     &bst_dptx_regmap_cfg);
	if (IS_ERR(dptx->regs)) {
		dev_err(dev, "Failed to create DPTX_CSR regmap\n");
		return PTR_ERR(dptx->regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, DPTX_APG);
	if (!res) {
		dev_err(dev, "Failed to get memory resource %d\n", DPTX_APG);
		return -ENODEV;
	}
	dptx->base[DPTX_APG] = devm_ioremap_resource(dev, res);
	if (IS_ERR(dptx->base[DPTX_APG])) {
		dev_err(dev, "Failed to map memory resource\n");
		return PTR_ERR(dptx->base[DPTX_APG]);
	}
	dptx->regs[DPTX_APG] = devm_regmap_init_mmio(dev, dptx->base[DPTX_APG],
						     &bst_dptx_regmap_cfg);
	if (IS_ERR(dptx->regs)) {
		dev_err(dev, "Failed to create DPTX_APG regmap\n");
		return PTR_ERR(dptx->regs);
	}

	dptx->irq[MAIN_FUNC_IRQ] = platform_get_irq(pdev, MAIN_FUNC_IRQ);
	if (dptx->irq[MAIN_FUNC_IRQ] < 0) {
		return -ENODEV;
	}

	return 0;
}

static int dptx_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct dptx *dptx;

	dev = &pdev->dev;

	dptx = devm_kzalloc(dev, sizeof(*dptx), GFP_KERNEL);
	if (!dptx)
		return -ENOMEM;

	memset(dptx, 0, sizeof(struct dptx));

	dptx->dev = dev;

	dev_info(dev, "****************************************\n");
	dev_info(dev, "Installing BST DPTX module\n");
	dev_info(dev, "****************************************\n");
	dev_info(dev, "Driver's name '%s'\n", "BST_DPTX");


	if (dptx_resource_get(dptx, pdev)) {
		dev_err(dev, "Driver get device resource failed\n");
		return -ENODEV;
	}

	dptx->cr_fail = false;
	dptx->mst = false;
	dptx->ssc_en = false;
	dptx->fec = false;
	dptx->dsc = false;
	dptx->edp = false;
	dptx->streams = 1;
	dptx->enhance_frame_en = true;
	dptx->multipixel = DPTX_MP_SINGLE_PIXEL;
	dptx->dummy_dtds_present = false;
	dptx->selected_est_timing = NONE;
	mutex_init(&dptx->mutex);
	init_waitqueue_head(&dptx->waitq);
	atomic_set(&dptx->sink_request, 0);
	atomic_set(&dptx->shutdown, 0);
	atomic_set(&dptx->c_connect, 0);
	dptx->max_rate = DPTX_DEFAULT_LINK_RATE;
	dptx->max_lanes = DPTX_DEFAULT_LINK_LANES;
	dptx->edid = kzalloc(DPTX_DEFAULT_EDID_BUFLEN, GFP_KERNEL);
	dptx->edid_second = kzalloc(DPTX_DEFAULT_EDID_BUFLEN, GFP_KERNEL);
	dptx->bstatus = 0;

#ifdef CONFIG_DEBUG_FS
	dptx_debugfs_init(dptx);
#endif

	__dptx_drm_handle = kzalloc(sizeof(struct drm_dptx), GFP_KERNEL);
	if (!__dptx_drm_handle)
		return -ENOMEM;

	__dptx_drm_handle->dptx = dptx;

	if (dptx_parse_dt_params(__dptx_drm_handle)) {
		dev_info(dev, "Driver get video param from DTS failed\n");
	}

	mutex_init(&__dptx_drm_handle->drm_lock);

	platform_set_drvdata(pdev, __dptx_drm_handle);

	return component_add(&pdev->dev, &dptx_component_ops);
}

static int dptx_remove(struct platform_device *plat)
{
	struct drm_dptx *drm_handle = platform_get_drvdata(plat);
	struct dptx *dptx = drm_handle->dptx;

	dptx_notify_shutdown(dptx);
	msleep(20);
	kfree(dptx->edid);
	if (dptx->edid_second)
		kfree(dptx->edid_second);
	dptx_core_deinit(dptx);
	dptx_debugfs_exit(dptx);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dptx_suspend(struct device *dev)
{
	return 0;
}

static int dptx_resume(struct device *dev)
{
	struct drm_dptx *drm_handle = dev_get_drvdata(dev);

	dptx_core_init(drm_handle->dptx);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int dptx_runtime_suspend(struct device *dev)
{
	return 0;
}

static int dptx_runtime_resume(struct device *dev)
{
	return 0;
}

static int dptx_runtime_idle(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops dptx_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dptx_suspend, dptx_resume) SET_RUNTIME_PM_OPS(
		dptx_runtime_suspend, dptx_runtime_resume, dptx_runtime_idle)
};

static const struct of_device_id bst_dw_dptx_dt_ids[] = {
	{
		.compatible = "bst,bst-dptx",
	},
	{ /* sentinel dsad*/ }
};
MODULE_DEVICE_TABLE(of, bst_dw_dptx_dt_ids);

static void dptx_shutdown(struct platform_device *pdev)
{
	struct drm_dptx *drm_handle = dev_get_drvdata(&pdev->dev);

	if (!pm_runtime_status_suspended(&pdev->dev))
		dptx_core_deinit(drm_handle->dptx);

	return;
}

static struct platform_driver dptx_driver = {
	.probe		= dptx_probe,
	.remove		= dptx_remove,
	.shutdown   = dptx_shutdown,
	.driver		= {
		.name	= "bst-dptx",
		.pm = &dptx_dev_pm_ops,
		.of_match_table = bst_dw_dptx_dt_ids,
	},
};

module_platform_driver(dptx_driver);

MODULE_AUTHOR("BST Ltd.");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Black Sesame Technologies DisplayPort TX Driver");
