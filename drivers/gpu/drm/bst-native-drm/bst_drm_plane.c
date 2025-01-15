// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>
#include "bst_drm_dev.h"
#include "bst_drm_kms.h"
#include "bst_drm_framebuffer.h"

static int
bst_plane_init_data_flow(struct drm_plane_state *st,
			    struct bst_crtc_state *bcrtc_st,
			    struct bst_data_flow_cfg *dflow)
{
	struct bst_plane *bplane = to_bplane(st->plane);
	struct drm_framebuffer *fb = st->fb;
	const struct bst_format_caps *caps = to_kfb(fb)->format_caps;
	struct bst_pipeline *pipe = bplane->layer->base.pipeline;

	memset(dflow, 0, sizeof(*dflow));

	dflow->blending_zorder = st->normalized_zpos;
	if (pipe == to_bcrtc(st->crtc)->master)
		dflow->blending_zorder -= bcrtc_st->max_slave_zorder;
	if (dflow->blending_zorder < 0) {
		DRM_DEBUG_ATOMIC("%s zorder:%d < max_slave_zorder: %d.\n",
				 st->plane->name, st->normalized_zpos,
				 bcrtc_st->max_slave_zorder);
		return -EINVAL;
	}

	dflow->pixel_blend_mode = st->pixel_blend_mode;
	dflow->layer_alpha = st->alpha >> 8;

	dflow->out_x = st->crtc_x;
	dflow->out_y = st->crtc_y;
	dflow->out_w = st->crtc_w;
	dflow->out_h = st->crtc_h;

	dflow->in_x = st->src_x >> 16;
	dflow->in_y = st->src_y >> 16;
	dflow->in_w = st->src_w >> 16;
	dflow->in_h = st->src_h >> 16;

	dflow->rot = drm_rotation_simplify(st->rotation, caps->supported_rots);
	if (!has_bits(dflow->rot, caps->supported_rots)) {
		DRM_DEBUG_ATOMIC("rotation(0x%x) isn't supported by %p4cc with modifier: 0x%llx.\n",
				 dflow->rot, &caps->fourcc, fb->modifier);
		return -EINVAL;
	}

	bst_complete_data_flow_cfg(bplane->layer, dflow, fb);

	return 0;
}

static int
bst_plane_atomic_check(struct drm_plane *plane,
			  struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
										 plane);
	struct bst_plane *bplane = to_bplane(plane);
	struct bst_plane_state *bplane_st = to_bplane_st(new_plane_state);
	struct bst_layer *layer = bplane->layer;
	struct drm_crtc_state *crtc_st;
	struct bst_crtc_state *bcrtc_st;
	struct bst_data_flow_cfg dflow;
	int err;

	if (!new_plane_state->crtc || !new_plane_state->fb)
		return 0;

	crtc_st = drm_atomic_get_crtc_state(state,
					    new_plane_state->crtc);
	if (IS_ERR(crtc_st) || !crtc_st->enable) {
		DRM_DEBUG_ATOMIC("Cannot update plane on a disabled CRTC.\n");
		return -EINVAL;
	}

	if (!crtc_st->active)
		return 0;

	bcrtc_st = to_bcrtc_st(crtc_st);

	err = bst_plane_init_data_flow(new_plane_state, bcrtc_st, &dflow);
	if (err)
		return err;

	if (dflow.en_split)
		err = bst_build_layer_split_data_flow(layer,
				bplane_st, bcrtc_st, &dflow);
	else
		err = bst_build_layer_data_flow(layer,
				bplane_st, bcrtc_st, &dflow);

	return err;
}

static void
bst_plane_atomic_update(struct drm_plane *plane,
			   struct drm_atomic_state *old_state)
{
}

static const struct drm_plane_helper_funcs bst_plane_helper_funcs = {
	.atomic_check	= bst_plane_atomic_check,
	.atomic_update	= bst_plane_atomic_update,
};

static void bst_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);

	kfree(to_bplane(plane));
}

static void bst_plane_reset(struct drm_plane *plane)
{
	struct bst_plane_state *state;
	struct bst_plane *bplane = to_bplane(plane);

	if (plane->state)
		__drm_atomic_helper_plane_destroy_state(plane->state);

	kfree(plane->state);
	plane->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state) {
		state->base.rotation = DRM_MODE_ROTATE_0;
		state->base.pixel_blend_mode = DRM_MODE_BLEND_PREMULTI;
		state->base.alpha = DRM_BLEND_ALPHA_OPAQUE;
		state->base.zpos = bplane->layer->base.id;
		state->base.color_encoding = DRM_COLOR_YCBCR_BT601;
		state->base.color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;
		plane->state = &state->base;
		plane->state->plane = plane;
	}
}

static struct drm_plane_state *
bst_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct bst_plane_state *new;

	if (WARN_ON(!plane->state))
		return NULL;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &new->base);

	return &new->base;
}

static void
bst_plane_atomic_destroy_state(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_bplane_st(state));
}

static bool
bst_plane_format_mod_supported(struct drm_plane *plane,
				  u32 format, u64 modifier)
{
	struct bst_dev *mdev = plane->dev->dev_private;
	struct bst_plane *bplane = to_bplane(plane);
	u32 layer_type = bplane->layer->layer_type;

	return bst_format_mod_supported(&mdev->fmt_tbl, layer_type,
					   format, modifier, 0);
}

static const struct drm_plane_funcs bst_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= bst_plane_destroy,
	.reset			= bst_plane_reset,
	.atomic_duplicate_state	= bst_plane_atomic_duplicate_state,
	.atomic_destroy_state	= bst_plane_atomic_destroy_state,
	.format_mod_supported	= bst_plane_format_mod_supported,
};

static u32 get_possible_crtcs(struct bst_kms_dev *kms,
			      struct bst_pipeline *pipe)
{
	struct bst_crtc *crtc;
	u32 possible_crtcs = 0;
	int i;

	for (i = 0; i < kms->n_crtcs; i++) {
		crtc = &kms->crtcs[i];

		if ((pipe == crtc->master))
			possible_crtcs |= BIT(i);
	}

	return possible_crtcs;
}

static void
bst_set_crtc_plane_mask(struct bst_kms_dev *kms,
			   struct bst_pipeline *pipe,
			   struct drm_plane *plane)
{
	struct bst_crtc *bcrtc;
	int i;

	for (i = 0; i < kms->n_crtcs; i++) {
		bcrtc = &kms->crtcs[i];

		if (pipe == bcrtc->slave)
			bcrtc->slave_planes |= BIT(drm_plane_index(plane));
	}
}

static u32 get_plane_type(struct bst_kms_dev *kms,
			  struct bst_component *c)
{
	bool is_primary = (c->id == BST_DRM_COMPONENT_LAYER0);

	return is_primary ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
}

static int bst_plane_add(struct bst_kms_dev *kms,
			    struct bst_layer *layer)
{
	struct bst_dev *mdev = kms->base.dev_private;
	struct bst_component *c = &layer->base;
	struct bst_plane *bplane;
	struct drm_plane *plane;
	u32 *formats, n_formats = 0;
	int err;

	bplane = kzalloc(sizeof(*bplane), GFP_KERNEL);
	if (!bplane)
		return -ENOMEM;

	plane = &bplane->base;
	bplane->layer = layer;

	formats = bst_get_layer_fourcc_list(&mdev->fmt_tbl,
					       layer->layer_type, &n_formats);
	if (!formats) {
		kfree(bplane);
		return -ENOMEM;
	}

	err = drm_universal_plane_init(&kms->base, plane,
			get_possible_crtcs(kms, c->pipeline),
			&bst_plane_funcs,
			formats, n_formats, bst_supported_modifiers,
			get_plane_type(kms, c),
			"%s", c->name);

	bst_put_fourcc_list(formats);

	if (err) {
		kfree(bplane);
		return err;
	}

	drm_plane_helper_add(plane, &bst_plane_helper_funcs);

	err = drm_plane_create_rotation_property(plane, DRM_MODE_ROTATE_0,
						 layer->supported_rots);
	if (err)
		goto cleanup;

	err = drm_plane_create_alpha_property(plane);
	if (err)
		goto cleanup;

	err = drm_plane_create_blend_mode_property(plane,
			BIT(DRM_MODE_BLEND_PIXEL_NONE) |
			BIT(DRM_MODE_BLEND_PREMULTI)   |
			BIT(DRM_MODE_BLEND_COVERAGE));
	if (err)
		goto cleanup;

	err = drm_plane_create_color_properties(plane,
			BIT(DRM_COLOR_YCBCR_BT601) |
			BIT(DRM_COLOR_YCBCR_BT709) |
			BIT(DRM_COLOR_YCBCR_BT2020),
			BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
			BIT(DRM_COLOR_YCBCR_FULL_RANGE),
			DRM_COLOR_YCBCR_BT601,
			DRM_COLOR_YCBCR_LIMITED_RANGE);
	if (err)
		goto cleanup;

	err = drm_plane_create_zpos_property(plane, layer->base.id, 0, 8);
	if (err)
		goto cleanup;

	bst_set_crtc_plane_mask(kms, c->pipeline, plane);

	return 0;
cleanup:
	bst_plane_destroy(plane);
	return err;
}

int bst_kms_add_planes(struct bst_kms_dev *kms, struct bst_dev *mdev)
{
	struct bst_pipeline *pipe;
	int i, j, err;

	for (i = 0; i < mdev->n_pipelines; i++) {
		pipe = mdev->pipelines[i];

		for (j = 0; j < pipe->n_layers; j++) {
			if (BST_LIMIT_USR_PLANES == 1) {
				if (j==0) {
					err = bst_plane_add(kms, pipe->layers[j]);
					if (err)
						return err;
				}
			} else if (BST_LIMIT_USR_PLANES == 2) {
				if (j == 0 || j == 2) {
					err = bst_plane_add(kms, pipe->layers[j]);
					if (err)
						return err;
				}
			} else if (BST_LIMIT_USR_PLANES == 3) {
				if (j == 0 || j == 1 || j == 2) {
					err = bst_plane_add(kms, pipe->layers[j]);
					if (err)
						return err;
				}
			} else {
				err = bst_plane_add(kms, pipe->layers[j]);
				if (err)
					return err;
			}
		}
	}

	return 0;
}
