// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <drm/drm_print.h>
#include <linux/clk.h>
#include "bst_drm_dev.h"
#include "bst_drm_kms.h"
#include "bst_pipeline.h"
#include "bst_drm_framebuffer.h"

static inline bool is_switching_user(void *old, void *new)
{
	if (!old || !new)
		return false;

	return old != new;
}

static struct bst_pipeline_state *
bst_pipeline_get_state(struct bst_pipeline *pipe,
			  struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_private_obj_state(state, &pipe->obj);
	if (IS_ERR(priv_st))
		return ERR_CAST(priv_st);

	return priv_to_pipe_st(priv_st);
}

struct bst_pipeline_state *
bst_pipeline_get_old_state(struct bst_pipeline *pipe,
			      struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_old_private_obj_state(state, &pipe->obj);
	if (priv_st)
		return priv_to_pipe_st(priv_st);
	return NULL;
}

static struct bst_pipeline_state *
bst_pipeline_get_new_state(struct bst_pipeline *pipe,
			      struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_new_private_obj_state(state, &pipe->obj);
	if (priv_st)
		return priv_to_pipe_st(priv_st);
	return NULL;
}

static struct bst_pipeline_state *
bst_pipeline_get_state_and_set_crtc(struct bst_pipeline *pipe,
				       struct drm_atomic_state *state,
				       struct drm_crtc *crtc)
{
	struct bst_pipeline_state *st;

	st = bst_pipeline_get_state(pipe, state);
	if (IS_ERR(st))
		return st;

	if (is_switching_user(crtc, st->crtc)) {
		DRM_DEBUG_ATOMIC("CRTC%d required pipeline%d is busy.\n",
				 drm_crtc_index(crtc), pipe->id);
		return ERR_PTR(-EBUSY);
	}

	if (!crtc && st->active_comps) {
		DRM_DEBUG_ATOMIC("Disabling a busy pipeline:%d.\n", pipe->id);
		return ERR_PTR(-EBUSY);
	}

	st->crtc = crtc;

	if (crtc) {
		struct bst_crtc_state *bcrtc_st;

		bcrtc_st = to_bcrtc_st(drm_atomic_get_new_crtc_state(state,
								     crtc));

		bcrtc_st->active_pipes |= BIT(pipe->id);
		bcrtc_st->affected_pipes |= BIT(pipe->id);
	}
	return st;
}

static struct bst_component_state *
bst_component_get_state(struct bst_component *c,
			   struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	WARN_ON(!drm_modeset_is_locked(&c->pipeline->obj.lock));

	priv_st = drm_atomic_get_private_obj_state(state, &c->obj);
	if (IS_ERR(priv_st))
		return ERR_CAST(priv_st);

	return priv_to_comp_st(priv_st);
}

static struct bst_component_state *
bst_component_get_old_state(struct bst_component *c,
			       struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_old_private_obj_state(state, &c->obj);
	if (priv_st)
		return priv_to_comp_st(priv_st);
	return NULL;
}

static struct bst_component_state *
bst_component_get_state_and_set_user(struct bst_component *c,
					struct drm_atomic_state *state,
					void *user,
					struct drm_crtc *crtc)
{
	struct bst_pipeline_state *pipe_st;
	struct bst_component_state *st;

	pipe_st = bst_pipeline_get_state_and_set_crtc(c->pipeline,
							 state, crtc);
	if (IS_ERR(pipe_st))
		return ERR_CAST(pipe_st);

	st = bst_component_get_state(c, state);
	if (IS_ERR(st))
		return st;

	if (is_switching_user(user, st->binding_user)) {
		DRM_DEBUG_ATOMIC("required %s is busy.\n", c->name);
		return ERR_PTR(-EBUSY);
	}

	st->binding_user = user;
	if (st->binding_user)
		pipe_st->active_comps |= BIT(c->id);

	return st;
}

static void
bst_component_add_input(struct bst_component_state *state,
			   struct bst_component_output *input,
			   int idx)
{
	struct bst_component *c = state->component;

	WARN_ON((idx < 0 || idx >= c->max_active_inputs));
	if (!has_bit(idx, state->affected_inputs) ||
	    memcmp(&state->inputs[idx], input, sizeof(*input))) {
		memcpy(&state->inputs[idx], input, sizeof(*input));
		state->changed_active_inputs |= BIT(idx);
	}
	state->active_inputs |= BIT(idx);
	state->affected_inputs |= BIT(idx);
}

static int
bst_component_check_input(struct bst_component_state *state,
			     struct bst_component_output *input,
			     int idx)
{
	struct bst_component *c = state->component;

	if ((idx < 0) || (idx >= c->max_active_inputs)) {
		DRM_DEBUG_ATOMIC("%s required an invalid %s-input[%d].\n",
				 input->component->name, c->name, idx);
		return -EINVAL;
	}

	if (has_bit(idx, state->active_inputs)) {
		DRM_DEBUG_ATOMIC("%s required %s-input[%d] has been occupied already.\n",
				 input->component->name, c->name, idx);
		return -EINVAL;
	}

	return 0;
}

static void
bst_component_set_output(struct bst_component_output *output,
			    struct bst_component *comp,
			    u8 output_port)
{
	output->component = comp;
	output->output_port = output_port;
}

static int
bst_component_validate_private(struct bst_component *c,
				  struct bst_component_state *st)
{
	int err;

	if (!c->funcs->validate)
		return 0;

	err = c->funcs->validate(c, st);
	if (err)
		DRM_DEBUG_ATOMIC("%s validate private failed.\n", c->name);

	return err;
}

static struct bst_scaler *
bst_component_get_avail_scaler(struct bst_component *c,
				  struct drm_atomic_state *state)
{
	struct bst_pipeline_state *pipe_st;
	u32 avail_scalers;

	pipe_st = bst_pipeline_get_state(c->pipeline, state);
	if (!pipe_st)
		return NULL;

	avail_scalers = (pipe_st->active_comps & BST_DRM_PIPELINE_SCALERS) ^
			BST_DRM_PIPELINE_SCALERS;

	c = bst_component_pickup_output(c, avail_scalers);

	return to_scaler(c);
}

static void
bst_rotate_data_flow(struct bst_data_flow_cfg *dflow, u32 rot)
{
	if (drm_rotation_90_or_270(rot)) {
		swap(dflow->in_h, dflow->in_w);
		swap(dflow->total_in_h, dflow->total_in_w);
	}
}

static int
bst_layer_check_cfg(struct bst_layer *layer,
		       struct bst_fb *kfb,
		       struct bst_data_flow_cfg *dflow)
{
	u32 src_x, src_y, src_w, src_h;
	u32 line_sz, max_line_sz;

	if (!bst_fb_is_layer_supported(kfb, layer->layer_type, dflow->rot))
		return -EINVAL;

	if (layer->base.id == BST_DRM_COMPONENT_WB_LAYER) {
		src_x = dflow->out_x;
		src_y = dflow->out_y;
		src_w = dflow->out_w;
		src_h = dflow->out_h;
	} else {
		src_x = dflow->in_x;
		src_y = dflow->in_y;
		src_w = dflow->in_w;
		src_h = dflow->in_h;
	}

	if (bst_fb_check_src_coords(kfb, src_x, src_y, src_w, src_h))
		return -EINVAL;

	if (!in_range(&layer->hsize_in, src_w)) {
		DRM_DEBUG_ATOMIC("invalidate src_w %d.\n", src_w);
		return -EINVAL;
	}

	if (!in_range(&layer->vsize_in, src_h)) {
		DRM_DEBUG_ATOMIC("invalidate src_h %d.\n", src_h);
		return -EINVAL;
	}

	if (drm_rotation_90_or_270(dflow->rot))
		line_sz = dflow->in_h;
	else
		line_sz = dflow->in_w;

	if (kfb->base.format->hsub > 1)
		max_line_sz = layer->yuv_line_sz;
	else
		max_line_sz = layer->line_sz;

	if (line_sz > max_line_sz) {
		DRM_DEBUG_ATOMIC("Required line_sz: %d exceeds the max size %d\n",
				 line_sz, max_line_sz);
		return -EINVAL;
	}

	return 0;
}

static int
bst_layer_validate(struct bst_layer *layer,
		      struct bst_plane_state *bplane_st,
		      struct bst_data_flow_cfg *dflow)
{
	struct drm_plane_state *plane_st = &bplane_st->base;
	struct drm_framebuffer *fb = plane_st->fb;
	struct bst_fb *kfb = to_kfb(fb);
	struct bst_component_state *c_st;
	struct bst_layer_state *st;
	int i, err;

	err = bst_layer_check_cfg(layer, kfb, dflow);
	if (err)
		return err;

	c_st = bst_component_get_state_and_set_user(&layer->base,
			plane_st->state, plane_st->plane, plane_st->crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_layer_st(c_st);

	st->rot = dflow->rot;

	if (fb->modifier) {
		st->hsize = kfb->aligned_w;
		st->vsize = kfb->aligned_h;
		st->afbc_crop_l = dflow->in_x;
		st->afbc_crop_r = kfb->aligned_w - dflow->in_x - dflow->in_w;
		st->afbc_crop_t = dflow->in_y;
		st->afbc_crop_b = kfb->aligned_h - dflow->in_y - dflow->in_h;
	} else {
		st->hsize = dflow->in_w;
		st->vsize = dflow->in_h;
		st->afbc_crop_l = 0;
		st->afbc_crop_r = 0;
		st->afbc_crop_t = 0;
		st->afbc_crop_b = 0;
	}

	for (i = 0; i < fb->format->num_planes; i++)
		st->addr[i] = bst_fb_get_pixel_addr(kfb, dflow->in_x,
						       dflow->in_y, i);

	err = bst_component_validate_private(&layer->base, c_st);
	if (err)
		return err;

	bst_component_set_output(&dflow->input, &layer->base, 0);
	bst_rotate_data_flow(dflow, st->rot);

	return 0;
}

static int
bst_wb_layer_validate(struct bst_layer *wb_layer,
			 struct drm_connector_state *conn_st,
			 struct bst_data_flow_cfg *dflow)
{
	struct bst_fb *kfb = to_kfb(conn_st->writeback_job->fb);
	struct bst_component_state *c_st;
	struct bst_layer_state *st;
	int i, err;

	err = bst_layer_check_cfg(wb_layer, kfb, dflow);
	if (err)
		return err;

	c_st = bst_component_get_state_and_set_user(&wb_layer->base,
			conn_st->state, conn_st->connector, conn_st->crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_layer_st(c_st);

	st->hsize = dflow->out_w;
	st->vsize = dflow->out_h;

	for (i = 0; i < kfb->base.format->num_planes; i++)
		st->addr[i] = bst_fb_get_pixel_addr(kfb, dflow->out_x,
						       dflow->out_y, i);

	bst_component_add_input(&st->base, &dflow->input, 0);
	bst_component_set_output(&dflow->input, &wb_layer->base, 0);

	return 0;
}

static bool scaling_ratio_valid(u32 size_in, u32 size_out,
				u32 max_upscaling, u32 max_downscaling)
{
	if (size_out > size_in * max_upscaling)
		return false;
	else if (size_in > size_out * max_downscaling)
		return false;
	return true;
}

static int
bst_scaler_check_cfg(struct bst_scaler *scaler,
			struct bst_crtc_state *bcrtc_st,
			struct bst_data_flow_cfg *dflow)
{
	u32 hsize_in, vsize_in, hsize_out, vsize_out;
	u32 max_upscaling;

	hsize_in = dflow->in_w;
	vsize_in = dflow->in_h;
	hsize_out = dflow->out_w;
	vsize_out = dflow->out_h;

	if (!in_range(&scaler->hsize, hsize_in) ||
	    !in_range(&scaler->hsize, hsize_out)) {
		DRM_DEBUG_ATOMIC("Invalid horizontal sizes");
		return -EINVAL;
	}

	if (!in_range(&scaler->vsize, vsize_in) ||
	    !in_range(&scaler->vsize, vsize_out)) {
		DRM_DEBUG_ATOMIC("Invalid vertical sizes");
		return -EINVAL;
	}

	if (has_bit(dflow->input.component->id, BST_DRM_PIPELINE_COMPIZS))
		max_upscaling = 1;
	else
		max_upscaling = scaler->max_upscaling;

	if (!scaling_ratio_valid(hsize_in, hsize_out, max_upscaling,
				 scaler->max_downscaling)) {
		DRM_DEBUG_ATOMIC("Invalid horizontal scaling ratio");
		return -EINVAL;
	}

	if (!scaling_ratio_valid(vsize_in, vsize_out, max_upscaling,
				 scaler->max_downscaling)) {
		DRM_DEBUG_ATOMIC("Invalid vertical scaling ratio");
		return -EINVAL;
	}

	if (hsize_in > hsize_out || vsize_in > vsize_out) {
		struct bst_pipeline *pipe = scaler->base.pipeline;
		int err;

		err = pipe->funcs->downscaling_clk_check(pipe,
					&bcrtc_st->base.adjusted_mode,
					bst_crtc_get_aclk(bcrtc_st), dflow);
		if (err) {
			DRM_DEBUG_ATOMIC("aclk can't satisfy the clock requirement of the downscaling\n");
			return err;
		}
	}

	return 0;
}

static int
bst_scaler_validate(void *user,
		       struct bst_crtc_state *bcrtc_st,
		       struct bst_data_flow_cfg *dflow)
{
	struct drm_atomic_state *drm_st = bcrtc_st->base.state;
	struct bst_component_state *c_st;
	struct bst_scaler_state *st;
	struct bst_scaler *scaler;
	int err = 0;

	if (!(dflow->en_scaling || dflow->en_img_enhancement))
		return 0;

	scaler = bst_component_get_avail_scaler(dflow->input.component,
						   drm_st);
	if (!scaler) {
		DRM_DEBUG_ATOMIC("No scaler available");
		return -EINVAL;
	}

	err = bst_scaler_check_cfg(scaler, bcrtc_st, dflow);
	if (err)
		return err;

	c_st = bst_component_get_state_and_set_user(&scaler->base,
			drm_st, user, bcrtc_st->base.crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_scaler_st(c_st);

	st->hsize_in = dflow->in_w;
	st->vsize_in = dflow->in_h;
	st->hsize_out = dflow->out_w;
	st->vsize_out = dflow->out_h;
	st->right_crop = dflow->right_crop;
	st->left_crop = dflow->left_crop;
	st->total_vsize_in = dflow->total_in_h;
	st->total_hsize_in = dflow->total_in_w;
	st->total_hsize_out = dflow->total_out_w;
	st->en_alpha = dflow->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE;
	st->en_scaling = dflow->en_scaling;
	st->en_img_enhancement = dflow->en_img_enhancement;
	st->en_split = dflow->en_split;
	st->right_part = dflow->right_part;

	bst_component_add_input(&st->base, &dflow->input, 0);
	bst_component_set_output(&dflow->input, &scaler->base, 0);
	return err;
}

static void bst_split_data_flow(struct bst_scaler *scaler,
				   struct bst_data_flow_cfg *dflow,
				   struct bst_data_flow_cfg *l_dflow,
				   struct bst_data_flow_cfg *r_dflow);

static int
bst_splitter_validate(struct bst_splitter *splitter,
			 struct drm_connector_state *conn_st,
			 struct bst_data_flow_cfg *dflow,
			 struct bst_data_flow_cfg *l_output,
			 struct bst_data_flow_cfg *r_output)
{
	struct bst_component_state *c_st;
	struct bst_splitter_state *st;

	if (!splitter) {
		DRM_DEBUG_ATOMIC("Current HW doesn't support splitter.\n");
		return -EINVAL;
	}

	if (!in_range(&splitter->hsize, dflow->in_w)) {
		DRM_DEBUG_ATOMIC("split in_w:%d is out of the acceptable range.\n",
				 dflow->in_w);
		return -EINVAL;
	}

	if (!in_range(&splitter->vsize, dflow->in_h)) {
		DRM_DEBUG_ATOMIC("split in_h: %d exceeds the acceptable range.\n",
				 dflow->in_h);
		return -EINVAL;
	}

	c_st = bst_component_get_state_and_set_user(&splitter->base,
			conn_st->state, conn_st->connector, conn_st->crtc);

	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	bst_split_data_flow(splitter->base.pipeline->scalers[0],
			       dflow, l_output, r_output);

	st = to_splitter_st(c_st);
	st->hsize = dflow->in_w;
	st->vsize = dflow->in_h;
	st->overlap = dflow->overlap;

	bst_component_add_input(&st->base, &dflow->input, 0);
	bst_component_set_output(&l_output->input, &splitter->base, 0);
	bst_component_set_output(&r_output->input, &splitter->base, 1);

	return 0;
}

static int
bst_merger_validate(struct bst_merger *merger,
		       void *user,
		       struct bst_crtc_state *bcrtc_st,
		       struct bst_data_flow_cfg *left_input,
		       struct bst_data_flow_cfg *right_input,
		       struct bst_data_flow_cfg *output)
{
	struct bst_component_state *c_st;
	struct bst_merger_state *st;
	int err = 0;

	if (!merger) {
		DRM_DEBUG_ATOMIC("No merger is available");
		return -EINVAL;
	}

	if (!in_range(&merger->hsize_merged, output->out_w)) {
		DRM_DEBUG_ATOMIC("merged_w: %d is out of the accepted range.\n",
				 output->out_w);
		return -EINVAL;
	}

	if (!in_range(&merger->vsize_merged, output->out_h)) {
		DRM_DEBUG_ATOMIC("merged_h: %d is out of the accepted range.\n",
				 output->out_h);
		return -EINVAL;
	}

	c_st = bst_component_get_state_and_set_user(&merger->base,
			bcrtc_st->base.state, bcrtc_st->base.crtc, bcrtc_st->base.crtc);

	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_merger_st(c_st);
	st->hsize_merged = output->out_w;
	st->vsize_merged = output->out_h;

	bst_component_add_input(c_st, &left_input->input, 0);
	bst_component_add_input(c_st, &right_input->input, 1);
	bst_component_set_output(&output->input, &merger->base, 0);

	return err;
}

void pipeline_composition_size(struct bst_crtc_state *bcrtc_st,
			       u16 *hsize, u16 *vsize)
{
	struct drm_display_mode *m = &bcrtc_st->base.adjusted_mode;

	if (hsize)
		*hsize = m->hdisplay;
	if (vsize)
		*vsize = m->vdisplay;
}

static int
bst_compiz_set_input(struct bst_compiz *compiz,
			struct bst_crtc_state *bcrtc_st,
			struct bst_data_flow_cfg *dflow)
{
	struct drm_atomic_state *drm_st = bcrtc_st->base.state;
	struct bst_component_state *c_st, *old_st;
	struct bst_compiz_input_cfg *cin;
	u16 compiz_w, compiz_h;
	int idx = dflow->blending_zorder;

	pipeline_composition_size(bcrtc_st, &compiz_w, &compiz_h);
	if ((dflow->out_x + dflow->out_w > compiz_w) ||
	    (dflow->out_y + dflow->out_h > compiz_h) ||
	     dflow->out_w == 0 || dflow->out_h == 0) {
		DRM_DEBUG_ATOMIC("invalid disp rect [x=%d, y=%d, w=%d, h=%d]\n",
				 dflow->out_x, dflow->out_y,
				 dflow->out_w, dflow->out_h);
		return -EINVAL;
	}

	c_st = bst_component_get_state_and_set_user(&compiz->base, drm_st,
			bcrtc_st->base.crtc, bcrtc_st->base.crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	if (bst_component_check_input(c_st, &dflow->input, idx))
		return -EINVAL;

	cin = &(to_compiz_st(c_st)->cins[idx]);

	cin->hsize   = dflow->out_w;
	cin->vsize   = dflow->out_h;
	cin->hoffset = dflow->out_x;
	cin->voffset = dflow->out_y;
	cin->pixel_blend_mode = dflow->pixel_blend_mode;
	cin->layer_alpha = dflow->layer_alpha;

	old_st = bst_component_get_old_state(&compiz->base, drm_st);
	WARN_ON(!old_st);
	if (memcmp(&(to_compiz_st(old_st)->cins[idx]), cin, sizeof(*cin)))
		c_st->changed_active_inputs |= BIT(idx);

	bst_component_add_input(c_st, &dflow->input, idx);
	bst_component_set_output(&dflow->input, &compiz->base, 0);

	return 0;
}

static int
bst_compiz_validate(struct bst_compiz *compiz,
		       struct bst_crtc_state *state,
		       struct bst_data_flow_cfg *dflow)
{
	struct bst_component_state *c_st;
	struct bst_compiz_state *st;

	c_st = bst_component_get_state_and_set_user(&compiz->base,
			state->base.state, state->base.crtc, state->base.crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_compiz_st(c_st);

	pipeline_composition_size(state, &st->hsize, &st->vsize);

	if (dflow) {
		bst_component_set_output(&dflow->input, &compiz->base, 0);
		dflow->in_w = st->hsize;
		dflow->in_h = st->vsize;
		dflow->out_w = dflow->in_w;
		dflow->out_h = dflow->in_h;
		dflow->pixel_blend_mode = DRM_MODE_BLEND_PIXEL_NONE;
		dflow->layer_alpha = 0xFF;
		dflow->blending_zorder = 0;
	}

	return 0;
}

static int
bst_improc_validate(struct bst_improc *improc,
		       struct bst_crtc_state *bcrtc_st,
		       struct bst_data_flow_cfg *dflow)
{
	struct drm_crtc *crtc = bcrtc_st->base.crtc;
	struct drm_crtc_state *crtc_st = &bcrtc_st->base;
	struct bst_component_state *c_st;
	struct bst_improc_state *st;

	c_st = bst_component_get_state_and_set_user(&improc->base,
			bcrtc_st->base.state, crtc, crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_improc_st(c_st);

	st->hsize = dflow->in_w;
	st->vsize = dflow->in_h;

	if (drm_atomic_crtc_needs_modeset(crtc_st)) {
		u32 output_depths, output_formats;
		u32 avail_depths, avail_formats;

		bst_crtc_get_color_config(crtc_st, &output_depths,
					     &output_formats);

		avail_depths = output_depths & improc->supported_color_depths;
		if (avail_depths == 0) {
			DRM_DEBUG_ATOMIC("No available color depths, conn depths: 0x%x & display: 0x%x\n",
					 output_depths,
					 improc->supported_color_depths);
			return -EINVAL;
		}

		avail_formats = output_formats &
				improc->supported_color_formats;
		if (!avail_formats) {
			DRM_DEBUG_ATOMIC("No available color_formats, conn formats 0x%x & display: 0x%x\n",
					 output_formats,
					 improc->supported_color_formats);
			return -EINVAL;
		}

		st->color_depth = __fls(avail_depths);
		st->color_format = BIT(__ffs(avail_formats));
	}

	if (bcrtc_st->base.color_mgmt_changed) {
		drm_lut_to_fgamma_coeffs(bcrtc_st->base.gamma_lut,
					 st->fgamma_coeffs);
		drm_ctm_to_coeffs(bcrtc_st->base.ctm, st->ctm_coeffs);
	}

	bst_component_add_input(&st->base, &dflow->input, 0);
	bst_component_set_output(&dflow->input, &improc->base, 0);

	return 0;
}

static int
bst_timing_ctrlr_validate(struct bst_timing_ctrlr *ctrlr,
			     struct bst_crtc_state *bcrtc_st,
			     struct bst_data_flow_cfg *dflow)
{
	struct drm_crtc *crtc = bcrtc_st->base.crtc;
	struct bst_timing_ctrlr_state *st;
	struct bst_component_state *c_st;

	c_st = bst_component_get_state_and_set_user(&ctrlr->base,
			bcrtc_st->base.state, crtc, crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_ctrlr_st(c_st);

	bst_component_add_input(&st->base, &dflow->input, 0);
	bst_component_set_output(&dflow->input, &ctrlr->base, 0);

	return 0;
}

void bst_complete_data_flow_cfg(struct bst_layer *layer,
				   struct bst_data_flow_cfg *dflow,
				   struct drm_framebuffer *fb)
{
	struct bst_scaler *scaler = layer->base.pipeline->scalers[0];
	u32 w = dflow->in_w;
	u32 h = dflow->in_h;

	dflow->total_in_w = dflow->in_w;
	dflow->total_in_h = dflow->in_h;
	dflow->total_out_w = dflow->out_w;
	if (!fb->format->has_alpha)
		dflow->pixel_blend_mode = DRM_MODE_BLEND_PIXEL_NONE;

	if (drm_rotation_90_or_270(dflow->rot))
		swap(w, h);

	if (w < 2048)
		dflow->en_scaling = (w != dflow->out_w) || (h != dflow->out_h);

	dflow->is_yuv = fb->format->is_yuv;

	dflow->en_img_enhancement = dflow->out_w >= 2 * w ||
				    dflow->out_h >= 2 * h;

	if (dflow->en_scaling && scaler)
		dflow->en_split = !in_range(&scaler->hsize, dflow->in_w) ||
				  !in_range(&scaler->hsize, dflow->out_w);
}

static bool merger_is_available(struct bst_pipeline *pipe,
				struct bst_data_flow_cfg *dflow)
{
	u32 avail_inputs = pipe->merger ?
			   pipe->merger->base.supported_inputs : 0;

	return has_bit(dflow->input.component->id, avail_inputs);
}

int bst_build_layer_data_flow(struct bst_layer *layer,
				 struct bst_plane_state *bplane_st,
				 struct bst_crtc_state *bcrtc_st,
				 struct bst_data_flow_cfg *dflow)
{
	struct drm_plane *plane = bplane_st->base.plane;
	struct bst_pipeline *pipe = layer->base.pipeline;
	int err;

	DRM_DEBUG_ATOMIC("%s handling [PLANE:%d:%s]: src[x/y:%d/%d, w/h:%d/%d] disp[x/y:%d/%d, w/h:%d/%d]",
			 layer->base.name, plane->base.id, plane->name,
			 dflow->in_x, dflow->in_y, dflow->in_w, dflow->in_h,
			 dflow->out_x, dflow->out_y, dflow->out_w, dflow->out_h);

	err = bst_layer_validate(layer, bplane_st, dflow);
	if (err)
		return err;

	err = bst_scaler_validate(plane, bcrtc_st, dflow);
	if (err)
		return err;

	/* if split, check if can put the data flow into merger */
	if (dflow->en_split && merger_is_available(pipe, dflow))
		return 0;

	err = bst_compiz_set_input(pipe->compiz, bcrtc_st, dflow);

	return err;
}

static void bst_split_data_flow(struct bst_scaler *scaler,
				   struct bst_data_flow_cfg *dflow,
				   struct bst_data_flow_cfg *l_dflow,
				   struct bst_data_flow_cfg *r_dflow)
{
	bool r90 = drm_rotation_90_or_270(dflow->rot);
	bool flip_h = has_flip_h(dflow->rot);
	u32 l_out, r_out, overlap;

	memcpy(l_dflow, dflow, sizeof(*dflow));
	memcpy(r_dflow, dflow, sizeof(*dflow));

	l_dflow->right_part = false;
	r_dflow->right_part = true;
	r_dflow->blending_zorder = dflow->blending_zorder + 1;

	overlap = 0;
	if (dflow->en_scaling && scaler)
		overlap += scaler->scaling_split_overlap;

	dflow->overlap = overlap;

	if (dflow->en_img_enhancement && scaler)
		overlap += scaler->enh_split_overlap;

	l_dflow->overlap = overlap;
	r_dflow->overlap = overlap;

	if (r90) {
		if (dflow->en_scaling) {
			l_dflow->in_h = ALIGN(dflow->in_h, 2) / 2 + l_dflow->overlap;
			r_dflow->in_h = l_dflow->in_h;
		} else if (dflow->en_img_enhancement) {
			l_dflow->in_h = ALIGN(dflow->in_h, 2) / 2 + l_dflow->overlap;
			r_dflow->in_h = dflow->in_h / 2 + r_dflow->overlap;
		} else {
			l_dflow->in_h = ALIGN(((dflow->in_h + 1) >> 1), 2);
			r_dflow->in_h = dflow->in_h - l_dflow->in_h;
		}

		if ((overlap != 0) && dflow->is_yuv) {
			l_dflow->in_h = ALIGN(l_dflow->in_h, 2);
			r_dflow->in_h = ALIGN(r_dflow->in_h, 2);
		}

		if (flip_h)
			l_dflow->in_y = dflow->in_y + dflow->in_h - l_dflow->in_h;
		else
			r_dflow->in_y = dflow->in_y + dflow->in_h - r_dflow->in_h;
	} else {
		if (dflow->en_scaling) {
			l_dflow->in_w = ALIGN(dflow->in_w, 2) / 2 + l_dflow->overlap;
			r_dflow->in_w = l_dflow->in_w;
		} else if (dflow->en_img_enhancement) {
			l_dflow->in_w = ALIGN(dflow->in_w, 2) / 2 + l_dflow->overlap;
			r_dflow->in_w = dflow->in_w / 2 + r_dflow->overlap;
		} else {
			l_dflow->in_w = ALIGN(((dflow->in_w + 1) >> 1), 2);
			r_dflow->in_w = dflow->in_w - l_dflow->in_w;
		}

		if ((overlap != 0) && dflow->is_yuv) {
			l_dflow->in_w = ALIGN(l_dflow->in_w, 2);
			r_dflow->in_w = ALIGN(r_dflow->in_w, 2);
		}

		if (flip_h)
			l_dflow->in_x = dflow->in_w + dflow->in_x - l_dflow->in_w;
		else
			r_dflow->in_x = dflow->in_w + dflow->in_x - r_dflow->in_w;
	}

	if (dflow->en_scaling || dflow->en_img_enhancement)
		l_dflow->out_w = ((dflow->out_w + 1) >> 1);
	else
		l_dflow->out_w = ALIGN(((dflow->out_w + 1) >> 1), 2);

	r_dflow->out_w = dflow->out_w - l_dflow->out_w;

	l_dflow->out_x = dflow->out_x;
	r_dflow->out_x = l_dflow->out_w + l_dflow->out_x;

	if (r90) {
		l_out = (dflow->out_w * l_dflow->in_h) / dflow->in_h;
		r_out = (dflow->out_w * r_dflow->in_h) / dflow->in_h;
	} else {
		l_out = (dflow->out_w * l_dflow->in_w) / dflow->in_w;
		r_out = (dflow->out_w * r_dflow->in_w) / dflow->in_w;
	}

	l_dflow->left_crop  = 0;
	l_dflow->right_crop = l_out - l_dflow->out_w;
	r_dflow->left_crop  = r_out - r_dflow->out_w;
	r_dflow->right_crop = 0;

	l_dflow->out_w += l_dflow->right_crop + l_dflow->left_crop;
	r_dflow->out_w += r_dflow->right_crop + r_dflow->left_crop;
}

int bst_build_layer_split_data_flow(struct bst_layer *left,
				       struct bst_plane_state *bplane_st,
				       struct bst_crtc_state *bcrtc_st,
				       struct bst_data_flow_cfg *dflow)
{
	struct drm_plane *plane = bplane_st->base.plane;
	struct bst_pipeline *pipe = left->base.pipeline;
	struct bst_layer *right = left->right;
	struct bst_data_flow_cfg l_dflow, r_dflow;
	int err;

	bst_split_data_flow(pipe->scalers[0], dflow, &l_dflow, &r_dflow);

	DRM_DEBUG_ATOMIC("Assign %s + %s to [PLANE:%d:%s]: "
			 "src[x/y:%d/%d, w/h:%d/%d] disp[x/y:%d/%d, w/h:%d/%d]",
			 left->base.name, right->base.name,
			 plane->base.id, plane->name,
			 dflow->in_x, dflow->in_y, dflow->in_w, dflow->in_h,
			 dflow->out_x, dflow->out_y, dflow->out_w, dflow->out_h);

	err = bst_build_layer_data_flow(left, bplane_st, bcrtc_st, &l_dflow);
	if (err)
		return err;

	err = bst_build_layer_data_flow(right, bplane_st, bcrtc_st, &r_dflow);
	if (err)
		return err;

	bst_rotate_data_flow(dflow, dflow->rot);
	if (r_dflow.input.component == l_dflow.input.component)
		return 0;

	err = bst_merger_validate(pipe->merger, plane, bcrtc_st,
				     &l_dflow, &r_dflow, dflow);
	if (err)
		return err;

	err = bst_compiz_set_input(pipe->compiz, bcrtc_st, dflow);

	return err;
}

int bst_build_wb_data_flow(struct bst_layer *wb_layer,
			      struct drm_connector_state *conn_st,
			      struct bst_crtc_state *bcrtc_st,
			      struct bst_data_flow_cfg *dflow)
{
	struct drm_connector *conn = conn_st->connector;
	int err;

	err = bst_scaler_validate(conn, bcrtc_st, dflow);
	if (err)
		return err;

	return bst_wb_layer_validate(wb_layer, conn_st, dflow);
}

int bst_build_wb_split_data_flow(struct bst_layer *wb_layer,
				    struct drm_connector_state *conn_st,
				    struct bst_crtc_state *bcrtc_st,
				    struct bst_data_flow_cfg *dflow)
{
	struct bst_pipeline *pipe = wb_layer->base.pipeline;
	struct drm_connector *conn = conn_st->connector;
	struct bst_data_flow_cfg l_dflow, r_dflow;
	int err;

	err = bst_splitter_validate(pipe->splitter, conn_st,
				       dflow, &l_dflow, &r_dflow);
	if (err)
		return err;
	err = bst_scaler_validate(conn, bcrtc_st, &l_dflow);
	if (err)
		return err;

	err = bst_scaler_validate(conn, bcrtc_st, &r_dflow);
	if (err)
		return err;

	err = bst_merger_validate(pipe->merger, conn_st, bcrtc_st,
				     &l_dflow, &r_dflow, dflow);
	if (err)
		return err;

	return bst_wb_layer_validate(wb_layer, conn_st, dflow);
}

int bst_build_display_data_flow(struct bst_crtc *bcrtc,
				   struct bst_crtc_state *bcrtc_st)
{
	struct bst_pipeline *master = bcrtc->master;
	struct bst_pipeline *slave  = bcrtc->slave;
	struct bst_data_flow_cfg m_dflow;
	struct bst_data_flow_cfg s_dflow;
	int err;

	memset(&m_dflow, 0, sizeof(m_dflow));
	memset(&s_dflow, 0, sizeof(s_dflow));

	if (slave && has_bit(slave->id, bcrtc_st->active_pipes)) {
		err = bst_compiz_validate(slave->compiz, bcrtc_st, &s_dflow);
		if (err)
			return err;

		err = bst_compiz_set_input(master->compiz, bcrtc_st,
					      &s_dflow);
		if (err)
			return err;
	}

	err = bst_compiz_validate(master->compiz, bcrtc_st, &m_dflow);
	if (err)
		return err;

	err = bst_improc_validate(master->improc, bcrtc_st, &m_dflow);
	if (err)
		return err;

	err = bst_timing_ctrlr_validate(master->ctrlr, bcrtc_st, &m_dflow);
	if (err)
		return err;

	return 0;
}

static int
bst_pipeline_unbound_components(struct bst_pipeline *pipe,
				   struct bst_pipeline_state *new)
{
	struct drm_atomic_state *drm_st = new->obj.state;
	struct bst_pipeline_state *old = priv_to_pipe_st(pipe->obj.state);
	struct bst_component_state *c_st;
	struct bst_component *c;
	u32 id;
	unsigned long disabling_comps;

	WARN_ON(!old);

	disabling_comps = (~new->active_comps) & old->active_comps;

	for_each_set_bit(id, &disabling_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);
		c_st = bst_component_get_state_and_set_user(c,
				drm_st, NULL, new->crtc);
		if (PTR_ERR(c_st) == -EDEADLK)
			return -EDEADLK;
		WARN_ON(IS_ERR(c_st));
	}

	return 0;
}

int bst_release_unclaimed_resources(struct bst_pipeline *pipe,
				       struct bst_crtc_state *bcrtc_st)
{
	struct drm_atomic_state *drm_st = bcrtc_st->base.state;
	struct bst_pipeline_state *st;

	if (!pipe || !has_bit(pipe->id, bcrtc_st->affected_pipes))
		return 0;

	if (has_bit(pipe->id, bcrtc_st->active_pipes))
		st = bst_pipeline_get_new_state(pipe, drm_st);
	else
		st = bst_pipeline_get_state_and_set_crtc(pipe, drm_st, NULL);

	if (WARN_ON(IS_ERR_OR_NULL(st)))
		return -EINVAL;

	return bst_pipeline_unbound_components(pipe, st);
}

bool bst_pipeline_disable(struct bst_pipeline *pipe,
			     struct drm_atomic_state *old_state)
{
	struct bst_pipeline_state *old;
	struct bst_component *c;
	struct bst_component_state *c_st;
	u32 id;
	unsigned long disabling_comps;

	old = bst_pipeline_get_old_state(pipe, old_state);
	if (!old) {
		DRM_ERROR("not older state.");
		return false;
	}
	disabling_comps = old->active_comps &
			  (~pipe->standalone_disabled_comps);
	if (!disabling_comps)
		disabling_comps = old->active_comps &
				  pipe->standalone_disabled_comps;

	DRM_INFO("PIPE%d: active_comps: 0x%x, disabling_comps: 0x%lx.\n",
			 pipe->id, old->active_comps, disabling_comps);

	for_each_set_bit(id, &disabling_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);
		c_st = priv_to_comp_st(c->obj.state);

		c_st->changed_active_inputs |= c_st->active_inputs;

		c->funcs->disable(c);
	}

	old->active_comps &= ~disabling_comps;

	return old->active_comps ? true : false;
}

void bst_pipeline_update(struct bst_pipeline *pipe,
			    struct drm_atomic_state *old_state)
{
	struct bst_pipeline_state *new = priv_to_pipe_st(pipe->obj.state);
	struct bst_pipeline_state *old;
	struct bst_component *c;
	u32 id;
	unsigned long changed_comps;

	old = bst_pipeline_get_old_state(pipe, old_state);

	changed_comps = new->active_comps | old->active_comps;

	DRM_DEBUG_ATOMIC("PIPE%d: active_comps: 0x%x, changed: 0x%lx.\n",
			 pipe->id, new->active_comps, changed_comps);

	for_each_set_bit(id, &changed_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);

		if (new->active_comps & BIT(c->id))
			c->funcs->update(c, priv_to_comp_st(c->obj.state));
		else
			c->funcs->disable(c);
	}
}
