// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <video/videomode.h>
#include "bst_display_global_api.h"
#include "bst_virt_drm_device.h"
#include "bst_virt_pipeline.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_drm_connector.h"

#if 0
struct virt_shared_conn_dev {
	struct bst_virt_device *base_dev;
};

static void virt_shared_conn_cleanup(struct bst_virt_device *vdev)
{
	struct virt_shared_conn_dev *shared_conn = vdev->virt_dev_data;

	if (!shared_conn)
		return;

	vdev->virt_dev_data = NULL;
}

static int virt_shared_conn_update(struct bst_virt_device *virt_dev, void *properties)
{
	return 0;
}

static void virt_shared_conn_flush(struct bst_virt_device *virt_dev)
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

static int shared_conn_get_modes(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_connector *bconn = v_conn->bconn;
	struct drm_connector *connector = &bconn->base;
	struct bst_virt_device *virt_dev = c->pipe->subdevs[BST_VIRT_CONN_IDX];
	struct bst_display_vm_req req;
	struct bst_display_vm_setting vm_info = { 0 };
	struct drm_display_mode *mode;
	struct screen_state *preferred_screen = NULL;
	struct videomode vm;
	int ret;
	struct drm_display_info *disp_info = &connector->display_info;
	struct bst_display_dp_probed_info *dp_probed_info;
	struct bst_display_mipi_probed_info *mipi_probed_info;
	struct bst_display_lvds_probed_info *lvds_probed_info;

	ret = bst_display_conn_cmd_get_cur_video_mode(c->subdev_session, &req, &vm_info);
	if (ret) {
		DRM_ERROR("Failed to get shared video mode from FW\n");
		DRM_ERROR("It maybe owner client not boot done, retry later\n");
		return 0;
	}
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	drm_videomode_frome_fw(&vm, &vm_info.timing);
	drm_display_mode_from_videomode(&vm, mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_probed_add(connector, mode);

	switch (virt_dev->device_type) {
		case DEVICE_TYPE_VIRT_DP:
			dp_probed_info = (struct bst_display_dp_probed_info *)virt_dev->dev_info.private;
			preferred_screen = &dp_probed_info->preferred_screen;
			break;
		case DEVICE_TYPE_VIRT_DSI0:
		case DEVICE_TYPE_VIRT_DSI1:
			mipi_probed_info = (struct bst_display_mipi_probed_info *)virt_dev->dev_info.private;
			preferred_screen = &mipi_probed_info->preferred_screen;
			break;
		case DEVICE_TYPE_VIRT_LVDS0:
		case DEVICE_TYPE_VIRT_LVDS1:
			lvds_probed_info = (struct bst_display_lvds_probed_info *)virt_dev->dev_info.private;
			preferred_screen = &lvds_probed_info->preferred_screen;
			break;
		default:
			DRM_ERROR("device type(%d) not found!\n", virt_dev->device_type);
			return 0;
	}
	drm_display_info_form_fw(disp_info,	preferred_screen, v_conn);

	return 1;
}

static int shared_conn_detect(struct bst_virt_component *c)
{
	struct bst_virt_connector *v_conn =
		container_of(c, struct bst_virt_connector, base);
	struct bst_display_vm_req req;
	struct bst_display_vm_setting vm_info = { 0 };
	int ret;

	ret = bst_display_conn_cmd_get_cur_video_mode(c->subdev_session, &req, &vm_info);
	if (ret) {
		DRM_WARN("Failed to get shared video mode from FW\n");
		DRM_WARN("It maybe owner client not boot done, retry later\n");
		atomic_set(&v_conn->connected, 0);
		return connector_status_disconnected;
	}
	atomic_set(&v_conn->connected, 1);

	return connector_status_connected;
}

const static struct bst_virt_component_funcs shared_conn_funcs = {
	.get_modes = shared_conn_get_modes,
	.detect = shared_conn_detect,
};

static int virt_shared_conn_init_submodule(struct bst_virt_device *base_dev)
{
	struct bst_virt_component *comp = NULL;
	struct bst_virt_connector *v_conn;
	uint32_t dev_type = base_dev->device_type;
	uint32_t subdev_session = base_dev->dev_info.subdev_session;
	struct bst_display_dp_probed_info *dp_probed_info =
		(struct bst_display_dp_probed_info *)base_dev->dev_info.private;
	struct bst_display_mipi_probed_info *probed_mipi_info =
		(struct bst_display_mipi_probed_info *)base_dev->dev_info.private;
	struct bst_display_lvds_probed_info *probed_lvds_info =
		(struct bst_display_lvds_probed_info *)base_dev->dev_info.private;

	struct bst_display_submodule_req hw_get_subm_info_req = { 0 };
	struct bst_display_submodule_info hw_submodule_info = { 0 };
	int ret;

	switch (dev_type) {
	case DEVICE_TYPE_VIRT_DP:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_eDP_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_DP(shared)-%d", 0);
		if (IS_ERR(comp)) {
			DRM_ERROR("Failed to add connector component\n");
			return PTR_ERR(comp);
		}
		v_conn = container_of(comp, struct bst_virt_connector, base);
		hw_get_subm_info_req.submodule_id = SUBMODULE_ID_DP_VIDEO;
		ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_get_subm_info_req, &hw_submodule_info);
		if (ret) {
			DRM_ERROR("Failed to get dp info from FW, ret(%d)\n", ret);
			return -1;
		}
		atomic_set(&v_conn->connected, hw_submodule_info.info.video_info.connected ? 1 : 0);
		v_conn->lanes = dp_probed_info->lanes;
		v_conn->rate = drm_dp_rate_from_firmware(dp_probed_info->rate);
		v_conn->bpc = dp_probed_info->bpc;
		v_conn->video_format = dp_probed_info->video_format;
		v_conn->supported_color_formats = hw_submodule_info.info.video_info.supported_color_formats;
		v_conn->supported_color_depths = hw_submodule_info.info.video_info.supported_color_depths;

		DRM_INFO("dptx(shared) lane:%d, bpc:%d, rate:%d, video_format:%d, supported_color_formats:%d, supported_color_depths:%d\n",
			v_conn->lanes,
			v_conn->rate,
			v_conn->bpc,
			v_conn->video_format,
			v_conn->supported_color_formats,
			v_conn->supported_color_depths);

		break;
	case DEVICE_TYPE_VIRT_DSI0:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_DSI_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_DSI(shared)-%d", 0);

		v_conn = container_of(comp, struct bst_virt_connector, base);
		hw_get_subm_info_req.submodule_id = SUBMODULE_ID_MIPI_VIDEO;
		ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_get_subm_info_req, &hw_submodule_info);
		if (ret) {
			DRM_ERROR("Failed to get mipi dsi0 info from FW, ret(%d)\n", ret);
			return -1;
		}
		atomic_set(&v_conn->connected, 1);
		v_conn->lanes = probed_mipi_info->lanes;
		v_conn->bpc = probed_mipi_info->bpc;
		v_conn->video_format = probed_mipi_info->format;
		v_conn->supported_color_depths = hw_submodule_info.info.video_info.supported_color_depths;
		v_conn->supported_color_formats = hw_submodule_info.info.video_info.supported_color_formats;
		DRM_INFO("mipi_dsi0(shared) lane:%d, bpc:%d, video_format:%d, supported_color_formats:%d, supported_color_depths:%d\n",
			v_conn->lanes,
			v_conn->bpc,
			v_conn->video_format,
			v_conn->supported_color_formats,
			v_conn->supported_color_depths);
		break;
	case DEVICE_TYPE_VIRT_DSI1:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_DSI_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_DSI(shared)-%d", 1);
		v_conn = container_of(comp, struct bst_virt_connector, base);
		hw_get_subm_info_req.submodule_id = SUBMODULE_ID_MIPI_VIDEO;
		ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_get_subm_info_req, &hw_submodule_info);
		if (ret) {
			DRM_ERROR("Failed to get mipi dsi1 info from FW, ret(%d)\n", ret);
			return -1;
		}
		atomic_set(&v_conn->connected, 1);
		v_conn->lanes = probed_mipi_info->lanes;
		v_conn->bpc = probed_mipi_info->bpc;
		v_conn->video_format = probed_mipi_info->format;
		v_conn->supported_color_depths = hw_submodule_info.info.video_info.supported_color_depths;
		v_conn->supported_color_formats = hw_submodule_info.info.video_info.supported_color_formats;
		DRM_INFO("mipi_dsi1(shared) lane:%d, bpc:%d, video_format:%d, supported_color_formats:%d, supported_color_depths:%d\n",
			v_conn->lanes,
			v_conn->bpc,
			v_conn->video_format,
			v_conn->supported_color_formats,
			v_conn->supported_color_depths);
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_LVDS_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_LVDS(shared)-%d", 0);
		v_conn = container_of(comp, struct bst_virt_connector, base);
		hw_get_subm_info_req.submodule_id = SUBMODULE_ID_LVDS_VIDEO;
		ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_get_subm_info_req, &hw_submodule_info);
		if (ret) {
			DRM_ERROR("Failed to get lvds0 info from FW, ret(%d)\n", ret);
			return -1;
		}
		atomic_set(&v_conn->connected, 1);
		v_conn->lanes = 4;
		v_conn->bpc = probed_lvds_info->bpc;
		v_conn->video_format = probed_lvds_info->video_format;
		v_conn->supported_color_depths = hw_submodule_info.info.video_info.supported_color_depths;
		v_conn->supported_color_formats = hw_submodule_info.info.video_info.supported_color_formats;
		DRM_INFO("lvds0(shared) lane:%d, bpc:%d, video_format:%d, supported_color_formats:%d, supported_color_depths:%d\n",
			v_conn->lanes,
			v_conn->bpc,
			v_conn->video_format,
			v_conn->supported_color_formats,
			v_conn->supported_color_depths);
		break;
	case DEVICE_TYPE_VIRT_LVDS1:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_LVDS_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_LVDS(shared)-%d", 1);
		v_conn = container_of(comp, struct bst_virt_connector, base);
		hw_get_subm_info_req.submodule_id = SUBMODULE_ID_LVDS_VIDEO;
		ret = bst_display_conn_cmd_get_submodule_info(subdev_session, &hw_get_subm_info_req, &hw_submodule_info);
		if (ret) {
			DRM_ERROR("Failed to get lvds1 info from FW, ret(%d)\n", ret);
			return -1;
		}
		atomic_set(&v_conn->connected, 1);
		v_conn->lanes = 4;
		v_conn->bpc = probed_lvds_info->bpc;
		v_conn->video_format = probed_lvds_info->video_format;
		v_conn->supported_color_depths = hw_submodule_info.info.video_info.supported_color_depths;
		v_conn->supported_color_formats = hw_submodule_info.info.video_info.supported_color_formats;
		DRM_INFO("lvds1(shared) lane:%d, bpc:%d, video_format:%d, supported_color_formats:%d, supported_color_depths:%d\n",
			v_conn->lanes,
			v_conn->bpc,
			v_conn->video_format,
			v_conn->supported_color_formats,
			v_conn->supported_color_depths);
		break;
	case DEVICE_TYPE_VIRT_DUAL_LVDS:
		comp = bst_virt_component_add(base_dev->this_pipe, base_dev,
					sizeof(*v_conn),
					BST_VIRT_COMPONENT_CONN_LVDS_VIDEO, 0,
					&shared_conn_funcs, 0, 1, 1, "VIRT_DUAL_LVDS(shared)-%d", 0);
		v_conn = container_of(comp, struct bst_virt_connector, base);
		break;
	default:
		DRM_ERROR("invalid dev_type\n");
		return -1;
	}
	if (IS_ERR(comp)) {
		DRM_ERROR("Failed to add connector component\n");
		return PTR_ERR(comp);
	}

	ret = bst_virt_drm_connector_get_edid(v_conn);
	if (ret)
		DRM_ERROR("device type(%d) get edid FAILED!\n", dev_type);
	return 0;
}

static int virt_shared_conn_probe(struct bst_virt_device *vdev)
{
	struct device *dev = vdev->dev;
	struct virt_shared_conn_dev *shared_conn_dev;
	shared_conn_dev = devm_kzalloc(dev, sizeof(*shared_conn_dev), GFP_KERNEL);
	if (!shared_conn_dev)
		return -ENOMEM;
	shared_conn_dev->base_dev = vdev;
	vdev->virt_dev_data = shared_conn_dev;

	return virt_shared_conn_init_submodule(vdev);
}

const struct bst_virt_device_funcs virt_shared_conn_dev_funcs = {
	.probe = virt_shared_conn_probe,
	.cleanup = virt_shared_conn_cleanup,
	.update = virt_shared_conn_update,
	.flush = virt_shared_conn_flush,
};
#endif