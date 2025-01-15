// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <drm/drm_framebuffer.h>
#include "bst_drm_dev.h"
#include "bst_drm_kms.h"
#include <drm/drm_probe_helper.h>
#include <video/videomode.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <video/videomode.h>

static int
bst_wb_init_data_flow(struct bst_layer *wb_layer,
			 struct drm_connector_state *conn_st,
			 struct bst_crtc_state *bcrtc_st,
			 struct bst_data_flow_cfg *dflow)
{
	struct drm_framebuffer *fb = conn_st->writeback_job->fb;

	memset(dflow, 0, sizeof(*dflow));

	dflow->out_w = fb->width;
	dflow->out_h = fb->height;

	pipeline_composition_size(bcrtc_st, &dflow->in_w, &dflow->in_h);
	dflow->input.component = &wb_layer->base.pipeline->compiz->base;
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
	struct bst_layer *wb_layer;
	struct bst_data_flow_cfg dflow;
	int err;

	if (!writeback_job)
		return 0;

	if (!crtc_st->active) {
		DRM_DEBUG_ATOMIC("Cannot write the composition result out on a inactive CRTC.\n");
		return -EINVAL;
	}

	wb_layer = to_bconn(to_wb_conn(conn_st->connector))->wb_layer;

	if (crtc_st->connectors_changed &&
	    is_only_changed_connector(crtc_st, conn_st->connector))
		crtc_st->connectors_changed = false;

	err = bst_wb_init_data_flow(wb_layer, conn_st, bcrtc_st, &dflow);
	if (err)
		return err;

	if (dflow.en_split)
		err = bst_build_wb_split_data_flow(wb_layer,
				conn_st, bcrtc_st, &dflow);
	else
		err = bst_build_wb_data_flow(wb_layer,
				conn_st, bcrtc_st, &dflow);

	return err;
}

static const struct drm_encoder_helper_funcs bst_wb_encoder_helper_funcs = {
	.atomic_check = bst_wb_encoder_atomic_check,
};

static void bst_writeback_preferred_videomode(struct videomode *vm)
{
	vm->pixelclock = 594000 * 1000;
	vm->hactive = 3840;
	vm->hfront_porch = 176;
	vm->hback_porch = 296;
	vm->hsync_len = 88;
	vm->vactive = 2160;
	vm->vfront_porch = 8;
	vm->vback_porch = 72;
	vm->vsync_len = 10;
	vm->flags = DISPLAY_FLAGS_VSYNC_HIGH | DISPLAY_FLAGS_HSYNC_HIGH;
}


static int
bst_wb_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	struct videomode vm;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new writeback display mode\n");
		return 0;
	}
	bst_writeback_preferred_videomode(&vm);
	drm_display_mode_from_videomode(&vm, mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

    return 1 + drm_add_modes_noedid(connector,
					dev->mode_config.max_width,
				    dev->mode_config.max_height);
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
	kfree(to_bconn(to_wb_conn(connector)));
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
	struct bst_dev *mdev = kms->base.dev_private;
	struct bst_wb_connector *kwb_conn;
	struct drm_writeback_connector *wb_conn;
	struct drm_display_info *info;
	u32 *formats, n_formats = 0;
	int err;

	if (!bcrtc->master->wb_layer)
		return 0;

	kwb_conn = kzalloc(sizeof(*kwb_conn), GFP_KERNEL);
	if (!kwb_conn)
		return -ENOMEM;

	kwb_conn->wb_layer = bcrtc->master->wb_layer;

	wb_conn = &kwb_conn->base;

	formats = bst_get_layer_fourcc_list(&mdev->fmt_tbl,
					       kwb_conn->wb_layer->layer_type,
					       &n_formats);
	err = drm_writeback_connector_init(&kms->base, wb_conn,
					   &bst_wb_connector_funcs,
					   &bst_wb_encoder_helper_funcs,
					   formats, n_formats,
					   BIT(drm_crtc_index(&bcrtc->base)));
	bst_put_fourcc_list(formats);
	if (err) {
		kfree(kwb_conn);
		return err;
	}
	drm_connector_helper_add(&wb_conn->base, &bst_wb_conn_helper_funcs);

	info = &kwb_conn->base.base.display_info;
	info->bpc = __fls(bcrtc->master->improc->supported_color_depths);
	info->color_formats = bcrtc->master->improc->supported_color_formats;

	bcrtc->wb_conn = kwb_conn;

	DRM_INFO("bst drm wirteback :%s init done\n", wb_conn->base.name);

	return 0;
}

int bst_kms_add_wb_connectors(struct bst_kms_dev *kms,
				 struct bst_dev *mdev)
{
	int i, err;

	for (i = 0; i < kms->n_crtcs; i++) {
		err = bst_wb_connector_add(kms, &kms->crtcs[i]);
		if (err)
			return err;
	}

	return 0;
}
