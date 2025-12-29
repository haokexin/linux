// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "linux/types.h"
#define BST_VIRT_TAG "virt-drm-conn"

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>
#include <linux/platform_device.h>
#include <video/videomode.h>
#include <linux/of_graph.h>
#include <linux/component.h>
#include <drm/drm_probe_helper.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include "bst_virt_drm_device.h"
#include "bst_virt_dp/virt_dp_dev.h"
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include "bst_virt_pipeline.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_drm_connector.h"
#include "bst_virt_drm_debugfs.h"

int bst_virt_drm_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len)
{
	struct bst_virt_connector *vconn = (struct bst_virt_connector *)data;
	struct drm_connector *conn = &vconn->bconn->base;
	u8 *edid = (u8 *)vconn->edid;

	if (len > EDID_LENGTH) return -EINVAL;
	if (block >= 2) {
		DRM_ERROR("Error, virt conn(%s) edid block num = %d", conn->name, block);
		return -EINVAL;
	}

	memcpy(buf, &edid[EDID_LENGTH * block], len);

	return 0;
}


int bst_virt_drm_connector_get_edid(struct bst_virt_connector *vconn)
{
	struct bst_virt_component *c = &vconn->base;
	struct drm_connector *conn = &vconn->bconn->base;
	uint32_t subdev_session = c->subdev_session;
	struct bst_display_edid_req edid;
	struct bst_display_edid_info edid_info = {};
	uint8_t edid_data[EDID_LENGTH];
	uint8_t *v_edid = vconn->edid;
	uint32_t ext_blocks = 0;
	uint32_t blocks = 1;
	int ret, count, pos;

	edid.type = EDID_BLOCK_TOP;
	ret = bst_display_conn_cmd_get_edid(subdev_session, &edid, &edid_info);
	if (ret) {
		DRM_ERROR("Failed to get %s edid top info from FW, ret(%d)\n", conn->name, ret);
		return -1;
	}
	memcpy(&edid_data[0], &edid_info.edid[0], sizeof(edid_info.edid));
	edid.type = EDID_BLOCK_BOTTOM;
	ret = bst_display_conn_cmd_get_edid(subdev_session, &edid, &edid_info);
	if (ret) {
		DRM_ERROR("Failed to get %s edid bottom info from FW, ret(%d)\n", conn->name, ret);
		return -1;
	}
	memcpy(&edid_data[64], &edid_info.edid[0], sizeof(edid_info.edid));
	if (edid_data[0x7e] > 10) {
		ext_blocks = 2;
	} else {
		ext_blocks = edid_data[0x7e];
	}
	if (((ext_blocks + 1) * 2) > EDID_MAX_BLOCK_NUM) {
		DRM_WARN("conn[%s]: cann't support ext blocks num [%d].\n",
			conn->name, ext_blocks);
		ext_blocks = (EDID_MAX_BLOCK_NUM / 2) - 1;
	}

	memcpy(v_edid, edid_data, sizeof(edid_data));
	memset(edid_data, 0, EDID_LENGTH);
	for (count = 1; count <= ext_blocks * 2; count++) {
		edid.type = count + 1;
		pos = edid.type % 2 ? 64 : 0;
		ret = bst_display_conn_cmd_get_edid(subdev_session, &edid, &edid_info);
		if (ret) {
			DRM_ERROR("Failed to get %s edid bottom info from FW, ret(%d)\n", conn->name, ret);
			goto out;
		}
		memcpy(&edid_data[pos], &edid_info.edid[0], sizeof(edid_info.edid));
		if (pos == 64) {
			memcpy(&v_edid[EDID_LENGTH * blocks], edid_data, sizeof(edid_data));
			memset(edid_data, 0, EDID_LENGTH);
			blocks++;
		}
	}

out:
	return ret;
}

static enum drm_mode_status
bst_connector_mode_valid(struct drm_connector *connector,
			 struct drm_display_mode *mode)
{
	struct bst_connector *bconn =
		container_of(connector, struct bst_connector, base);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct bst_virt_component *c = &vconn->base;
	struct bst_virt_device *vdev = c->pipe->subdevs[BST_VIRT_CONN_IDX];
	struct drm_display_info *disp_info = &connector->display_info;
	struct virt_dp_dev *dp_dev;
	enum drm_mode_status status = MODE_ERROR;
	u8 lanes;
	u32 requested, actual;

	switch (vdev->device_type) {
	case DEVICE_TYPE_VIRT_DP:
		dp_dev = (struct virt_dp_dev *)vdev->virt_dev_data;
		if (!atomic_read(&vconn->connected)) {
			DRM_WARN("dptx not connected\n");
			return MODE_BAD;
		}
		lanes = vconn->lanes;
		requested = mode->clock * disp_info->bpc * 3 / 1000;
		actual = vconn->rate * lanes / 100;
		/* efficiency is about 0.8 */
		actual = actual * 8 / 10;
		if (requested > actual) {
			DRM_WARN(
				"dptx bandwidth: requested=%d > actual=%d,clock=%d, rate=%d, lanes:%d\n",
				requested, actual, mode->clock, vconn->rate,
				lanes);
			return MODE_CLOCK_HIGH;
		}
		status = MODE_OK;
		break;
	case DEVICE_TYPE_VIRT_DSI0:
		status = MODE_OK;
		break;
	case DEVICE_TYPE_VIRT_DSI1:
		status = MODE_OK;
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
		status = MODE_OK;
		break;
	case DEVICE_TYPE_VIRT_LVDS1:
		status = MODE_OK;
		break;
	default:
		break;
	}
	return status;
}
static uint32_t bst_color_formats_to_drm_formats(uint32_t bst_formats)
{
	uint32_t dst_formats = 0;

	if (bst_formats & DRM_COLOR_FORMAT_RGB444)
		dst_formats |= DRM_COLOR_FORMAT_RGB444;
	if (bst_formats & DRM_COLOR_FORMAT_YCBCR420)
		dst_formats |= DRM_COLOR_FORMAT_YCBCR420;
	if (bst_formats & DRM_COLOR_FORMAT_YCBCR422)
		dst_formats |= DRM_COLOR_FORMAT_YCBCR422;
	if (bst_formats & DRM_COLOR_FORMAT_YCBCR444)
		dst_formats |= DRM_COLOR_FORMAT_YCBCR444;
	return dst_formats;
}
static int bst_connector_get_modes(struct drm_connector *connector)
{
	struct bst_connector *bconn =
		container_of(connector, struct bst_connector, base);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct bst_virt_component *c = &vconn->base;
	const struct bst_virt_component_funcs *funcs =
		bconn->virt_conn->base.funcs;
	struct edid *drm_edid;
	int mode_num = 0;
	uint32_t support_formats;
	struct drm_display_info *disp_info = &connector->display_info;

	drm_edid = drm_do_get_edid(connector, bst_virt_drm_get_edid_block,
					   vconn);
	if (drm_edid) {
		drm_connector_update_edid_property(connector, drm_edid);
	}
	if (funcs->get_modes) {
		mode_num += funcs->get_modes(c);
		if (!mode_num)
			DRM_ERROR("Please check %s video timing!\n",
				  connector->name);
	}

	switch (disp_info->bpc) {
	case COLOR_DEPTH_8:
	case COLOR_DEPTH_10:
	case COLOR_DEPTH_12:
	case COLOR_DEPTH_16:
		if (BIT(disp_info->bpc) & vconn->supported_color_depths) {
			vconn->bpc = disp_info->bpc;
			break;
		}
	fallthrough;
	default:
	DRM_WARN("Color bpc(%d) ERROR: Using default value(%d)!\n",
		  disp_info->bpc, vconn->bpc);
	disp_info->bpc = vconn->bpc;
	break;
	}
	support_formats = bst_color_formats_to_drm_formats(
		vconn->supported_color_formats);
	if (!(support_formats & disp_info->color_formats)) {
		DRM_WARN("Drm support_formats(%#x) ERROR: Using default value(%d)!\n",
		  support_formats, vconn->video_format);
		disp_info->color_formats = vconn->video_format;
	}
	if (drm_edid)
		kfree(drm_edid);
	return mode_num;
}

static enum drm_connector_status
bst_connector_detect(struct drm_connector *connector, bool force)
{
	struct bst_connector *bconn =
		container_of(connector, struct bst_connector, base);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct bst_virt_component *c = &vconn->base;
	const struct bst_virt_component_funcs *funcs =
		bconn->virt_conn->base.funcs;
	enum drm_connector_status status = connector_status_unknown;

	if (funcs->detect) {
		status = funcs->detect(c);
	}
	return status;
}

static void bst_connector_destroy(struct drm_connector *connector)
{
	DBG("%s", connector->name);
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static uint16_t aspect_gcd(int h, int v)
{
    int temp;
    while (v != 0)
    {
        temp = v;
        v = h % v;
        h = temp;
    }
    return h;
}

static void bst_timing_from_videomode(struct bst_virt_connector *vconn,
				      struct videomode *vm)
{
	uint8_t ratio = aspect_gcd(vm->hactive, vm->vactive);
	vconn->cur_timing.video_info.h_image_size = vm->hactive / ratio;
	vconn->cur_timing.video_info.v_image_size = vm->vactive / ratio;
	vconn->cur_timing.display_protocol = INVALID;
	vconn->cur_timing.video_timing_id = INVALID;
	vconn->cur_timing.video_info.h_active = vm->hactive;
	vconn->cur_timing.video_info.pixel_clock = vm->pixelclock / 1000; // to KHz
	vconn->cur_timing.video_info.h_blanking =
		vm->hfront_porch + vm->hsync_len + vm->hback_porch;
	vconn->cur_timing.video_info.h_sync_offset = vm->hfront_porch;
	vconn->cur_timing.video_info.h_sync_pulse_width = vm->hsync_len;
	vconn->cur_timing.video_info.v_active = vm->vactive;
	vconn->cur_timing.video_info.v_blanking =
		vm->vfront_porch + vm->vsync_len + vm->vback_porch;
	vconn->cur_timing.video_info.v_sync_offset = vm->vfront_porch;
	vconn->cur_timing.video_info.v_sync_pulse_width = vm->vsync_len;
	vconn->cur_timing.video_info.pixel_repetition_input = 0;
	vconn->cur_timing.video_info.interlaced = 0;


	if (vm->flags & DISPLAY_FLAGS_HSYNC_HIGH) {
 		vconn->cur_timing.video_info.h_sync_polarity = 1;
	} else
 		vconn->cur_timing.video_info.h_sync_polarity = 0;

	if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH) {
 		vconn->cur_timing.video_info.v_sync_polarity = 1;
	} else
		vconn->cur_timing.video_info.v_sync_polarity = 0;

}

static void bst_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted)
{
	struct bst_connector *bconn =
		container_of(encoder, struct bst_connector, encoder);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct videomode vm = {};

	drm_display_mode_to_videomode(adjusted, &vm);
	bst_timing_from_videomode(vconn, &vm);
}

static void bst_encoder_enable(struct drm_encoder *encoder)
{
	struct bst_connector *bconn =
		container_of(encoder, struct bst_connector, encoder);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct bst_virt_component *c = &vconn->base;
	const struct bst_virt_component_funcs *funcs =
		bconn->virt_conn->base.funcs;

	if (funcs->enable) {
		funcs->enable(c);
	}
}

static void bst_encoder_disable(struct drm_encoder *encoder)
{
	struct bst_connector *bconn =
		container_of(encoder, struct bst_connector, encoder);
	struct bst_virt_connector *vconn = bconn->virt_conn;
	struct bst_virt_component *c = &vconn->base;
	const struct bst_virt_component_funcs *funcs =
		bconn->virt_conn->base.funcs;

	if (funcs->disable) {
		funcs->disable(c);
	}
}

static int bst_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	return 0;
}

static const struct drm_connector_funcs bst_connector_funcs = {
	.detect = bst_connector_detect,
	.destroy = bst_connector_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs bst_connector_helper_funcs = {
	.get_modes = bst_connector_get_modes,
	.mode_valid = bst_connector_mode_valid,
};

static const struct drm_encoder_helper_funcs bst_encoder_helper_funcs = {
	.mode_set = bst_encoder_mode_set,
	.enable = bst_encoder_enable,
	.disable = bst_encoder_disable,
	.atomic_check = bst_encoder_atomic_check,
};

static int bst_virt_drm_connector_init(struct drm_device *dev,
				       struct bst_connector *bst_conn,
				       struct bst_crtc *bcrtc)
{
	struct drm_connector *connector = NULL;
	struct bst_virt_device *subdev = bcrtc->master->subdevs[BST_VIRT_CONN_IDX];
	int conn_type = DRM_MODE_CONNECTOR_Unknown, err = 0;
	int encoder_type = DRM_MODE_ENCODER_NONE;

	connector = &bst_conn->base;

	switch (subdev->device_type) {
	case DEVICE_TYPE_VIRT_DP:
		connector->interlace_allowed = 1;
		connector->dpms = DRM_MODE_DPMS_OFF;
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		conn_type = DRM_MODE_CONNECTOR_DisplayPort;
		encoder_type = DRM_MODE_ENCODER_TMDS;
		bst_conn->encoder.possible_crtcs = BIT(drm_crtc_index(&bcrtc->base));
		break;
	case DEVICE_TYPE_VIRT_DSI0:
	case DEVICE_TYPE_VIRT_DSI1:
		connector->interlace_allowed = 0;
		connector->dpms = DRM_MODE_DPMS_OFF;
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		conn_type = DRM_MODE_CONNECTOR_DSI;
		encoder_type = DRM_MODE_ENCODER_DSI;
		bst_conn->encoder.possible_crtcs = BIT(drm_crtc_index(&bcrtc->base));
		break;
	case DEVICE_TYPE_VIRT_LVDS0:
	case DEVICE_TYPE_VIRT_LVDS1:
		connector->interlace_allowed = 0;
		connector->dpms = DRM_MODE_DPMS_OFF;
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		conn_type = DRM_MODE_CONNECTOR_LVDS;
		encoder_type = DRM_MODE_ENCODER_LVDS;
		bst_conn->encoder.possible_crtcs = BIT(drm_crtc_index(&bcrtc->base));
		break;
	default:
		break;
	}
	drm_connector_init(dev, connector, &bst_connector_funcs, conn_type);
	drm_connector_helper_add(connector, &bst_connector_helper_funcs);
	err = drm_connector_register(connector);
	if (err < 0) {
		DRM_ERROR("failed to register bst-virt connector!\n");
		goto fail;
	}

	err = drm_simple_encoder_init(dev, &bst_conn->encoder, encoder_type);

	if (err < 0) {
		DRM_ERROR("failed to init bst-virt encoder!\n");
		goto err_encoder;
	}
	drm_encoder_helper_add(&bst_conn->encoder, &bst_encoder_helper_funcs);

	err = drm_connector_attach_encoder(connector,
						&bst_conn->encoder);
	if (err < 0) {
		DRM_ERROR("failed to init bst-virt encoder\n");
		goto err_attach;
	}

	drm_mode_config_reset(dev);

	bst_conn->virt_conn = bcrtc->master->master_conn;
	bst_conn->virt_conn->bconn = bst_conn;
	bcrtc->master_conn = bst_conn;
	connector->display_info.bpc = bst_conn->virt_conn->bpc;
#ifdef CONFIG_DEBUG_FS
	bst_virt_drm_debugfs_init(subdev);
#endif
	return 0;

err_attach:
 	drm_encoder_cleanup(&bst_conn->encoder);
err_encoder:
	drm_connector_cleanup(connector);
fail:
	drm_crtc_cleanup(&bcrtc->base);

	return err;
}

static int bst_virt_conn_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct device_node *port, *ep, *remote;
	struct bst_kms_dev *kms = data;
	struct bst_super_device *super_dev = dev_to_super_dev(master);
	struct bst_connector *bst_conn = dev_get_drvdata(dev);
	struct device_node *node = dev->of_node;
	struct bst_virt_device *subdev;
	struct bst_virt_platform_info plat_info = {
		.platform_id = super_dev->super_info.platform_id,
	};
	u32 pipe_idx = 0;
	int ret = 0;

	port = of_graph_get_port_by_id(node, 0);
	if (!port)
		return -EINVAL;

	ep = of_get_next_available_child(port, NULL);
	if (!ep) {
		ret = -EINVAL;
		goto port_of_put;
	}

	remote = of_graph_get_remote_port_parent(ep);
	if (!remote) {
		ret = -EINVAL;
		goto remote_of_put;
	}

	of_property_read_u32(remote, "reg", &pipe_idx);
	if (pipe_idx > kms->n_crtcs - 1) {
		DRM_ERROR("Request CRTCS num is error!");
		ret = -EINVAL;
		goto bind_fail;
	}

	plat_info.device_type = super_dev->super_info.device_map[pipe_idx][BST_VIRT_CONN_IDX];
	subdev = bst_virt_create_subdevice(dev, &plat_info,
					   super_dev->pipelines[pipe_idx]);
	if (IS_ERR_OR_NULL(subdev)) {
		DRM_ERROR("Subdev[%d] create FAILED!, device_type mismatch", plat_info.device_type);
		ret = -EINVAL;
		goto bind_fail;
	}

	if (IS_ERR_OR_NULL(super_dev) || IS_ERR_OR_NULL(super_dev->pipelines[pipe_idx])) {
		ret = -EINVAL;
		goto bind_fail;
	}

	super_dev->subdevs[pipe_idx][BST_VIRT_CONN_IDX] = subdev;
	super_dev->pipelines[pipe_idx]->subdevs[BST_VIRT_CONN_IDX] = subdev;
	subdev->fmt_tbl = NULL;

	if (subdev->this_pipe->pipe_id != pipe_idx) {
		DRM_ERROR("Subdev[%d] create FAILED!, pipe_id mismatch", plat_info.device_type);
		goto bind_fail;
	}
	if (!is_valid_connector(plat_info.device_type)) {
		ret = -EINVAL;
		DRM_ERROR("crtcs and connector is not match!");
		goto bind_fail;
	}

	ret = bst_virt_drm_connector_init(&kms->base, bst_conn,
					  &kms->crtcs[pipe_idx]);

bind_fail:
remote_of_put:
	of_node_put(ep);
port_of_put:
	of_node_put(port);

	return ret;
}

static void bst_virt_conn_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct bst_kms_dev *kms = data;
	struct drm_device *drm = &kms->base;
	DRM_INFO("bst virt conn(%s) component unbind\n", dev_name(dev));
	component_unbind_all(drm->dev, drm);
}

static const struct component_ops bst_virt_conn_ops = {
	.bind = bst_virt_conn_bind,
	.unbind = bst_virt_conn_unbind,
};

static int bst_virt_connectors_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_connector *bst_conn = NULL;
	struct backlight_device *bd = NULL;
	struct device_node *backlight_node = NULL;

	backlight_node = of_parse_phandle(dev->of_node, "backlight", 0);
	if (!backlight_node) {
		dev_info(dev, "No backlight node specified in connector DTS.\n");
	} else {
		bd = of_find_backlight_by_node(backlight_node);
		of_node_put(backlight_node);
		backlight_node = NULL;
		if (!bd) {
			dev_info(dev, "backlight device not available, deferring probe\n");
			return -EPROBE_DEFER;
		}
	}

	bst_conn = devm_kzalloc(dev, sizeof(*bst_conn), GFP_KERNEL);
	if (!bst_conn)
		return	-ENOMEM;

	bst_conn->bd = bd;

	platform_set_drvdata(pdev, bst_conn);

	return component_add(&pdev->dev, &bst_virt_conn_ops);
}

static int bst_virt_connectors_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &bst_virt_conn_ops);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bst_conn_pm_suspend(struct device *dev)
{
	// struct bst_connector *bconn = dev_get_drvdata(dev);
	// struct bst_virt_connector *vconn = bconn->virt_conn;
	// struct bst_virt_component *c = &vconn->base;
	// const struct bst_virt_component_funcs *funcs =
	// 	bconn->virt_conn->base.funcs;
	return 0;
}

static int bst_conn_pm_resume(struct device *dev)
{
	// struct bst_connector *bconn = dev_get_drvdata(dev);
	// struct bst_virt_connector *vconn = bconn->virt_conn;
	// struct bst_virt_component *c = &vconn->base;
	// const struct bst_virt_component_funcs *funcs =
	// 	bconn->virt_conn->base.funcs;
	return 0;
}
#endif
static const struct dev_pm_ops bst_connector_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bst_conn_pm_suspend, bst_conn_pm_resume)
};


static const struct of_device_id bst_virt_connectors_dt_match[] = {
	{ .compatible = "bst,virt-dp" },
	{ .compatible = "bst,virt-lvds0" },
	{ .compatible = "bst,virt-lvds1" },
	{ .compatible = "bst,virt-dual-lvds" },
	{ .compatible = "bst,virt-dsi0" },
	{ .compatible = "bst,virt-dsi1" },
	{}
};
struct platform_driver bst_virt_connectors_driver = {
	.probe = bst_virt_connectors_probe,
	.remove = bst_virt_connectors_remove,
	.driver = {
		.name = "bst-virt-conn",
		.of_match_table = bst_virt_connectors_dt_match,
		.pm = &bst_connector_pm_ops,
	},
};