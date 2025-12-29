/*
 * Copyright (c) 2006 Luc Verhaegen (quirks list)
 * Copyright (c) 2007-2008 Intel Corporation
 *   Jesse Barnes <jesse.barnes@intel.com>
 * Copyright 2010 Red Hat, Inc.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * DDC probing routines (drm_ddc_read & drm_do_probe_ddc_edid) originally from
 * FB layer.
 *   Copyright (C) 2006 Dennis Munsie <dmunsie@cecropia.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define BST_VIRT_TAG "virt-dp-comp"

#include <video/videomode.h>
#include <drm/drm_panel.h>
#include <drm/drm_edid.h>
#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst_display_conn_cmdset.h"
#include "bst_virt_pipeline.h"
#include "virt_dp_dev.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_drm_connector.h"

#define EDID_DETAILED_TIMINGS 4

static bool dp_wait_link_training_done(struct bst_virt_component *c)
{
	struct bst_display_submodule_info hw_dp_info = { 0 };
	struct bst_display_submodule_req hw_dp_req = { 
		.submodule_id = SUBMODULE_ID_DP_VIDEO
	};
	int ret;
	ktime_t start_time, end_time;
	#define WAIT_DP_LINK_TRAINING_RETRY_COUNT 300
	u32 retry = WAIT_DP_LINK_TRAINING_RETRY_COUNT;
	start_time = ktime_get();
	/* Wait server DP training done. */
	do {
		ret = bst_display_conn_cmd_get_submodule_info(c->subdev_session, &hw_dp_req, &hw_dp_info);
		if (!ret) {
			if (hw_dp_info.info.video_info.trained)
				break;
		}
		msleep(10);
	} while (retry--);

	if (retry != WAIT_DP_LINK_TRAINING_RETRY_COUNT) {
		end_time = ktime_get();
		DRM_WARN("Wait DP trained retry %u times < %d times, total %lld ms\n",
			 WAIT_DP_LINK_TRAINING_RETRY_COUNT - retry,
			 WAIT_DP_LINK_TRAINING_RETRY_COUNT,
			 ktime_to_ns(ktime_sub(end_time, start_time)) / 1000000);
	}
	return hw_dp_info.info.video_info.trained;
}

static void dp_enable(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_virt_device *virt_dev = c->pipe->subdevs[BST_VIRT_CONN_IDX];
	struct virt_dp_dev *dp_dev =
		(struct virt_dp_dev *)virt_dev->virt_dev_data;

	struct bst_display_training_req hw_dp_training = {
		.lanes = v_conn->lanes,
		.rate = firmware_dp_rate_from_drm(v_conn->rate),
		.bpc = v_conn->bpc,
		.video_format = v_conn->video_format,
		.dynamic_range = dp_dev->dynamic_range,
		.colorimetry = dp_dev->colorimetry,
		.train_type = DP_LINK_TRAINING
	};
	struct bst_display_training_status training_status = {};
	struct bst_display_set_video_stream_req video_req = {
		.enable = 1,
		.ext_info.edp_param = {
			.lanes = v_conn->lanes,
			.bpc = v_conn->bpc,
			.video_format = v_conn->video_format,
			.dynamic_range = dp_dev->dynamic_range,
			.colorimetry = dp_dev->colorimetry,
		},
	};
	struct bst_display_comm_reply video_status = {};
	int ret;

	memcpy(&hw_dp_training.timing, &v_conn->cur_timing,
	       sizeof(struct video_timing));
	memcpy(&video_req.timing, &v_conn->cur_timing,
	       sizeof(struct video_timing));

	dp_dev->trained = dp_wait_link_training_done(c);

	if (!dp_dev->trained) {
		WARNING("Warning, DP need retraining!\n");
		ret = bst_display_conn_cmd_link_training(
			c->subdev_session, &hw_dp_training, &training_status);
		if (ret) {
			DRM_ERROR("Error, fw do training failed(%d)!", ret);
			return;
		}
		v_conn->rate = drm_dp_rate_from_firmware(training_status.rate);
		v_conn->lanes = training_status.lanes;
		dp_dev->trained = training_status.trained;
		atomic_set(&v_conn->connected, training_status.trained ? 1 : 0);
	}
	if (dp_dev->trained) {
		ret = bst_display_conn_cmd_set_video_stream(c->subdev_session, &video_req, &video_status);
		DRM_INFO("virt dp enable Video %s", (video_status.base.status == DISP_COMM_REPLAY_OK) ? "Success!" : "Failed!");
	}
	if (v_conn->bconn->bd)
		backlight_enable(v_conn->bconn->bd);
}

static void dp_disable(struct bst_virt_component *c)
{
	// struct bst_virt_connector *v_conn =
	// 	container_of(c, struct bst_virt_connector, base);
	struct bst_display_submodule_req submodule_req = { 0 };
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	// if (v_conn->bconn->bd)
	// 	backlight_device_set_brightness(v_conn->bconn->bd, 0);

	submodule_req.submodule_id = SUBMODULE_ID_DP_VIDEO;
	ret = bst_display_conn_cmd_disable_submodule(subdev_session, &submodule_req, &reply);
	if (!ret && reply.base.status == DISP_COMM_REPLAY_OK)
		DRM_DEBUG_ATOMIC("lvds submodule_id:%d disable ok!!\n", submodule_req.submodule_id);
	else
		DRM_ERROR("lvds submodule_id:%d disable falied!!\n", submodule_req.submodule_id);
}

static void dp_update(struct bst_virt_component *c,
		      struct bst_virt_component_state *state)
{
	DRM_DEBUG("[%s:%d]", __FUNCTION__, __LINE__);
}

static void drm_videomode_frome_fw(struct videomode *vm,
				   struct video_timing *timing)
{
	vm->pixelclock = timing->video_info.pixel_clock * 1000;
	vm->hactive = timing->video_info.h_active;
	vm->hfront_porch = timing->video_info.h_sync_offset;
	vm->hback_porch = timing->video_info.h_blanking -
			  timing->video_info.h_sync_offset -
			  timing->video_info.h_sync_pulse_width;
	vm->hsync_len = timing->video_info.h_sync_pulse_width;
	vm->vactive = timing->video_info.v_active;
	vm->vfront_porch = timing->video_info.v_sync_offset;
	vm->vback_porch = timing->video_info.v_blanking -
			  timing->video_info.v_sync_offset -
			  timing->video_info.v_sync_pulse_width;
	vm->vsync_len = timing->video_info.v_sync_pulse_width;
	vm->flags = timing->video_info.v_sync_polarity ?
				  DISPLAY_FLAGS_VSYNC_HIGH :
				  DISPLAY_FLAGS_VSYNC_LOW;
	vm->flags |= timing->video_info.h_sync_polarity ?
				   DISPLAY_FLAGS_HSYNC_HIGH :
				   DISPLAY_FLAGS_HSYNC_LOW;
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

	if (hactive < 64 || vactive < 64)
		return false;

	if (pt->misc & DRM_EDID_PT_STEREO) {
		DRM_DEBUG_KMS("stereo mode not supported\n");
		return false;
	}
	if (!(pt->misc & DRM_EDID_PT_SEPARATE_SYNC)) {
		DRM_DEBUG_KMS("composite sync not supported\n");
	}

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

static int dp_get_modes(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_connector *bconn = v_conn->bconn;
	struct drm_connector *connector = &bconn->base;
	struct bst_virt_pipe *pipe = c->pipe;
	uint32_t subdev_session = c->subdev_session;
	struct bst_virt_device *virt_dev = pipe->subdevs[BST_VIRT_CONN_IDX];
	struct virt_dp_dev *dp_dev =
		(struct virt_dp_dev *)virt_dev->virt_dev_data;
	struct bst_display_dp_probed_info *probed_info =
		(struct bst_display_dp_probed_info *)virt_dev->dev_info.private;
	struct bst_display_vm_setting vm_info = { 0 };
	struct bst_display_vm_req vm_req ;
	struct drm_display_info *disp_info = &connector->display_info;
	struct drm_display_mode *mode;
	struct edid *drm_edid = (struct edid*)dp_dev->edid;
	struct videomode vm;
	int ret, mode_num = 0;

	if (drm_edid) {
		dp_dev->audio_support = drm_detect_monitor_audio(drm_edid);
	}

	if (probed_info->video_timing_nums > 0) {
		ret = bst_display_conn_cmd_get_cur_video_mode(subdev_session, &vm_req, &vm_info);
		if (ret) {
			DRM_ERROR("Failed to get DP video info from FW\n");
			goto fail;
		}
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			DRM_ERROR("failed to create a new display mode\n");
			goto fail;
		}
		drm_videomode_frome_fw(&vm, &vm_info.timing);
		drm_display_mode_from_videomode(&vm, mode);
		mode->type = DRM_MODE_TYPE_DRIVER;
		mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
		mode_num++;
		if (mode_num > 0)
			goto out;
	}
	if (drm_edid) {
		if (edid_preferred_timing_fixup((u8 *)drm_edid, connector)) {
			DRM_WARN_ONCE("Warning! EDID preferred timing is illegal, please configure firmware panel-timing!\n");
		} else {
			mode_num += drm_add_edid_modes(connector, drm_edid);
		}
	} else {
		DRM_WARN_ONCE("Warning! EDID is NULL, please configure firmware panel-timing!\n");
		goto fail;
	}
out:
	drm_display_info_form_fw(disp_info,
			&probed_info->preferred_screen, v_conn);
fail:
	return mode_num;
}

static int dp_detect(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_virt_pipe *pipe = c->pipe;
	struct bst_virt_device *virt_dev = pipe->subdevs[BST_VIRT_CONN_IDX];
	struct virt_dp_dev *dp_dev =
		(struct virt_dp_dev *)virt_dev->virt_dev_data;
	
	if (atomic_read(&v_conn->connected) && !dp_dev->trained)
		dp_dev->trained = dp_wait_link_training_done(c);

	return (atomic_read(&v_conn->connected) && dp_dev->trained) ? connector_status_connected :
						 connector_status_disconnected;
}

static void dp_dump(struct bst_virt_component *c, struct seq_file *seq)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_connector *bconn = v_conn->bconn;
	struct bst_virt_pipe *pipe = c->pipe;
	uint32_t subdev_session = c->subdev_session;
	struct bst_virt_device *virt_dev = pipe->subdevs[BST_VIRT_CONN_IDX];
	struct bst_display_dp_probed_info *probed_info =
		(struct bst_display_dp_probed_info *)virt_dev->dev_info.private;
	struct virt_dp_dev *dp_dev =
		(struct virt_dp_dev *)virt_dev->virt_dev_data;
	seq_printf(seq, "       session:%#x\n", subdev_session);
	seq_printf(seq, "   colorimetry:%d\n", probed_info->colorimetry);
	seq_printf(seq, " dynamic_range:%d\n", probed_info->dynamic_range);
	seq_printf(seq, "     connected:%d\n", atomic_read(&bconn->virt_conn->connected));
	seq_printf(seq, "       trained:%d\n", dp_dev->trained);
	seq_printf(seq, "         lanes:%d\n", bconn->virt_conn->lanes);
	seq_printf(seq, "          rate:%d\n", bconn->virt_conn->rate);
	seq_printf(seq, "           bpc:%d\n", bconn->virt_conn->bpc);
}

const static struct bst_virt_component_funcs dp_funcs = {
	.enable = dp_enable,
	.disable = dp_disable,
	.update = dp_update,
	.get_modes = dp_get_modes,
	.detect = dp_detect,
	.dump_log = dp_dump,
};

int virt_dp_init_submodule(struct virt_dp_dev *dp,
		       struct bst_display_submodule_header *submodule)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_connector *v_conn;
	uint32_t conn_fw_id = SUBMODULE_INFO_SUBMODULE_ID(submodule->submodule_info);
	uint32_t subdev_session = dp->base_dev->dev_info.subdev_session;

	struct bst_display_dp_probed_info *probed_info =
		(struct bst_display_dp_probed_info *)dp->base_dev->dev_info.private;
	struct bst_display_submodule_info hw_dp_info = { 0 };
	struct bst_display_submodule_req hw_dp_req = { 0 };
	int ret;

	comp = bst_virt_component_add(dp->base_dev->this_pipe, dp->base_dev,
				      sizeof(*v_conn),
				      BST_VIRT_COMPONENT_CONN_eDP_VIDEO, conn_fw_id,
				      &dp_funcs, 0, 1, 1, "VIRT_eDP-%d",
				      0);
	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add connector component\n");
		return PTR_ERR(comp);
	}

	v_conn = to_virt_connector(comp);
	hw_dp_req.submodule_id = SUBMODULE_ID_DP_VIDEO;
	ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_dp_req, &hw_dp_info);
	if (ret) {
		DRM_ERROR("Failed to get dp info from FW, ret(%d)\n", ret);
		return -1;
	}
	atomic_set(&v_conn->connected, hw_dp_info.info.video_info.connected ? 1 : 0);
	v_conn->lanes = probed_info->lanes;
	v_conn->rate = drm_dp_rate_from_firmware(probed_info->rate);
	v_conn->bpc = probed_info->bpc;
	v_conn->video_format = probed_info->video_format;
	v_conn->supported_color_formats = hw_dp_info.info.video_info.supported_color_formats;
	v_conn->supported_color_depths = hw_dp_info.info.video_info.supported_color_depths;
	dp->colorimetry = probed_info->colorimetry;
	dp->dynamic_range = probed_info->dynamic_range;
	dp->trained = hw_dp_info.info.video_info.trained;

	ret = bst_virt_drm_connector_get_edid(v_conn);
	if (!ret) {
		dp->edid = v_conn->edid;
	}

	return ret;
}
