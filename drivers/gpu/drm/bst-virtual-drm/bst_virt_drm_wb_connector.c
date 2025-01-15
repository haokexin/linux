// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <drm/drm_framebuffer.h>
#include "bst_virt_drm_device.h"
#include "bst_virt_drm_kms.h"
#include <drm/drm_probe_helper.h>
#include <video/videomode.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

static int
bst_wb_init_data_flow(struct bst_virt_layer *wb_layer,
			 struct drm_connector_state *conn_st,
			 struct bst_crtc_state *bcrtc_st,
			 struct bst_data_flow_cfg *dflow)
{
	struct drm_framebuffer *fb = conn_st->writeback_job->fb;

	memset(dflow, 0, sizeof(*dflow));

	dflow->out_w = fb->width;
	dflow->out_h = fb->height;

	bst_virt_pipe_dc_crtc_size(bcrtc_st, &dflow->in_w, &dflow->in_h);
	dflow->pixel_blend_mode = DRM_MODE_BLEND_PIXEL_NONE;
	dflow->rot = DRM_MODE_ROTATE_0;

	bst_complete_data_flow_cfg(wb_layer, dflow, fb);

	return 0;
}

static int
bst_wb_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_st,
			       struct drm_connector_state *conn_st)
{
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(crtc_st);
	struct drm_writeback_job *writeback_job = conn_st->writeback_job;
	struct bst_virt_layer *wb_layer;
	struct bst_data_flow_cfg dflow;
	int err;

	if (!writeback_job)
		return 0;

	if (!crtc_st->active) {
		DRM_DEBUG_ATOMIC("Cannot write the composition result out on a inactive CRTC.\n");
		return -EINVAL;
	}

	wb_layer = to_bwconn(to_wb_conn(conn_st->connector))->wb_layer;

	if (crtc_st->connectors_changed &&
	    is_only_changed_connector(crtc_st, conn_st->connector))
		crtc_st->connectors_changed = false;

	err = bst_wb_init_data_flow(wb_layer, conn_st, bcrtc_st, &dflow);
	if (err)
		return err;

	err = bst_build_wb_data_flow(wb_layer,
			conn_st, bcrtc_st, &dflow);

	return err;
}

static const struct drm_encoder_helper_funcs bst_wb_encoder_helper_funcs = {
	.atomic_check = bst_wb_encoder_atomic_check,
};

static int
bst_wb_connector_get_modes(struct drm_connector *connector)
{
	int ret;
	ret = drm_add_modes_noedid(connector, 640, 480);
	drm_set_preferred_mode(connector, 640, 480);

    return 1;
}

static enum drm_mode_status
bst_wb_connector_mode_valid(struct drm_connector *connector,
			       struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	int w = mode->hdisplay, h = mode->vdisplay;

	if ((w < mode_config->min_width) || (w > mode_config->max_width))
		return MODE_BAD_HVALUE;

	if ((h < mode_config->min_height) || (h > mode_config->max_height))
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs bst_wb_conn_helper_funcs = {
	.get_modes	= bst_wb_connector_get_modes,
	.mode_valid	= bst_wb_connector_mode_valid,
};

static enum drm_connector_status
bst_wb_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void bst_wb_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
	kfree(to_bwconn(to_wb_conn(connector)));
}

static const struct drm_connector_funcs bst_wb_connector_funcs = {
	.reset			= drm_atomic_helper_connector_reset,
	.detect			= bst_wb_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= bst_wb_connector_destroy,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static int bst_wb_connector_add(struct bst_kms_dev *kms,
				   struct bst_crtc *bcrtc)
{
	struct bst_super_device *sdev = kms->base.dev_private;
	struct bst_wb_connector *bwb_conn;
	struct drm_writeback_connector *wb_conn;
	struct drm_display_info *info;
	u32 *formats, n_formats = 0;
	int err;

	if (!bcrtc->master->dc_wb_layer)
		return 0;

	bwb_conn = kzalloc(sizeof(*bwb_conn), GFP_KERNEL);
	if (!bwb_conn)
		return -ENOMEM;

	bwb_conn->wb_layer = bcrtc->master->dc_wb_layer;

	wb_conn = &bwb_conn->base;

	formats = bst_get_layer_fourcc_list(&sdev->fmt_tbl,
					       bwb_conn->wb_layer->layer_type,
					       &n_formats);
	err = drm_writeback_connector_init(&kms->base, wb_conn,
					   &bst_wb_connector_funcs,
					   &bst_wb_encoder_helper_funcs,
					   formats, n_formats,
					   BIT(drm_crtc_index(&bcrtc->base)));
	bst_put_fourcc_list(formats);
	if (err) {
		kfree(bwb_conn);
		return err;
	}
	drm_connector_helper_add(&wb_conn->base, &bst_wb_conn_helper_funcs);

	info = &bwb_conn->base.base.display_info;
	info->bpc = __fls(bcrtc->master->dc_crtc->supported_color_depths);
	info->color_formats = bcrtc->master->dc_crtc->supported_color_formats;

	bcrtc->wb_conn = bwb_conn;

	return 0;
}

int bst_kms_add_wb_connectors(struct bst_kms_dev *kms)
{
	int i, err;

	for (i = 0; i < kms->n_crtcs; i++) {
		err = bst_wb_connector_add(kms, &kms->crtcs[i]);
		if (err)
			return err;
	}

	return 0;
}
