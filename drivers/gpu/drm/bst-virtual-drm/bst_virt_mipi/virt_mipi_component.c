// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#define VIRT_TAG "bst-virt-mipi"

#include <video/videomode.h>
#include <drm/drm_panel.h>
#include "bst_display_platform.h"
#include "bst_virt_pipeline.h"
#include "bst_virt_mipi/virt_mipi_dev.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_drm_connector.h"

static void mipi_enable(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_display_comm_reply reply = { 0 };
	struct bst_display_set_video_stream_req video_req = {
		.enable = 1,
	};
	int ret = 0;
	DRM_INFO("[%s:%d]", __FUNCTION__, __LINE__);
	memcpy(&video_req.timing, &v_conn->cur_timing, sizeof(struct video_timing));
	ret = bst_display_conn_cmd_set_video_stream(c->subdev_session, &video_req, &reply);
	if (ret) {
		DRM_ERROR("Failed, mipi enable!\n");
	}
	if (v_conn->bconn->bd)
		backlight_enable(v_conn->bconn->bd);
}
static void mipi_disable(struct bst_virt_component *c)
{
	// struct bst_virt_connector *v_conn =
	// 	container_of(c, struct bst_virt_connector, base);
	struct bst_display_submodule_req submodule_req = { 0 };
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	// if (v_conn->bconn->bd)
	// 	backlight_device_set_brightness(v_conn->bconn->bd, 0);

	submodule_req.submodule_id = SUBMODULE_ID_MIPI_VIDEO;
	ret = bst_display_conn_cmd_disable_submodule(subdev_session, &submodule_req, &reply);
	if (!ret && reply.base.status == DISP_COMM_REPLAY_OK)
		DRM_DEBUG_ATOMIC("mipi submodule_id:%d disable ok!!\n", submodule_req.submodule_id);
	else
		DRM_ERROR("mipi submodule_id:%d disable falied!!\n", submodule_req.submodule_id);
}
static void mipi_update(struct bst_virt_component *c,
		      struct bst_virt_component_state *state)
{
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

static int mipi_get_modes(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn = container_of(c, struct bst_virt_connector, base);
	struct bst_connector *bconn = v_conn->bconn;
	struct drm_connector *connector = &bconn->base;
	struct bst_virt_pipe *pipe = c->pipe;
	uint32_t subdev_session = c->subdev_session;
	struct bst_virt_device *virt_dev = pipe->subdevs[BST_VIRT_CONN_IDX];
	struct bst_display_mipi_probed_info *probed_info =
		(struct bst_display_mipi_probed_info *)virt_dev->dev_info.private;
	struct bst_display_vm_setting vm_info = { 0 };
	struct bst_display_vm_req hw_mipi_req;
	struct drm_display_info *disp_info = &connector->display_info;
	struct drm_display_mode *mode;
	struct videomode vm;

	int ret, mode_num = 0;
	
	ret = bst_display_conn_cmd_get_cur_video_mode(subdev_session, &hw_mipi_req, &vm_info);
	if (ret) {
		DRM_ERROR("Failed to get dev(%d) video info from FW!\n", virt_dev->device_type);
		goto out;
	}
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		goto out;
	}
	drm_videomode_frome_fw(&vm, &vm_info.timing);
	drm_display_mode_from_videomode(&vm, mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	mode_num++;
	drm_display_info_form_fw(disp_info,	&probed_info->preferred_screen, v_conn);

out:
	return mode_num;
}

static int mipi_detect(struct bst_virt_component *c)
{
	return connector_status_connected;
}

const static struct bst_virt_component_funcs mipi_funcs = {
	.enable = mipi_enable,
	.disable = mipi_disable,
	.update = mipi_update,
	.get_modes = mipi_get_modes,
	.detect = mipi_detect,
};

int virt_mipi_init_submodule(struct virt_mipi_dev *mipi,
		       struct bst_display_submodule_header *submodule)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_connector *v_conn;
	uint32_t conn_fw_id = SUBMODULE_INFO_SUBMODULE_ID(submodule->submodule_info);
	uint32_t subdev_session = mipi->base_dev->dev_info.subdev_session;
	struct bst_display_submodule_info hw_mipi_info = { 0 };
	struct bst_display_submodule_req hw_mipi_req = { 0 };
	struct bst_display_mipi_probed_info *probed_info = 
		(struct bst_display_mipi_probed_info *)mipi->base_dev->dev_info.private;
	int ret, mipi_id = mipi->base_dev->device_type == DEVICE_TYPE_VIRT_DSI0 ? 0 : 1;

	comp = bst_virt_component_add(mipi->base_dev->this_pipe, mipi->base_dev,
				      sizeof(*v_conn),
				      BST_VIRT_COMPONENT_CONN_DSI_VIDEO, conn_fw_id,
				      &mipi_funcs, 0, 1, 1, "VIRT_MIPI-%d", mipi_id);
	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add connector component\n");
		return PTR_ERR(comp);
	}

	v_conn = to_virt_mipi_connector(comp);
	hw_mipi_req.submodule_id = SUBMODULE_ID_MIPI_VIDEO;
	ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_mipi_req, &hw_mipi_info);
	if (ret) {
		DRM_ERROR("Failed to get mipi info from FW\n");
		return -1;
	}
	mipi->lanes = probed_info->lanes;
	v_conn->bpc = probed_info->bpc;
	v_conn->video_format = probed_info->format;
	v_conn->supported_color_depths = hw_mipi_info.info.video_info.supported_color_depths;
	v_conn->supported_color_formats = hw_mipi_info.info.video_info.supported_color_formats;
	DRM_DEBUG("mipi_cmd_get_info_handler supported_color_formats:0x%x supported_color_depths:0x%x\n",
		v_conn->supported_color_formats, v_conn->supported_color_depths);

	ret = bst_virt_drm_connector_get_edid(v_conn);

	return ret;
}
