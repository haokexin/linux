// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <drm/drm_print.h>
#include <linux/clk.h>
#include "bst_virt_dc/virt_dc_dev.h"
#include "bst_virt_drm_device.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_pipeline.h"
#include "bst_virt_drm_framebuffer.h"

static inline bool is_switching_user(void *old, void *new)
{
	if (!old || !new)
		return false;

	return old != new;
}

static struct bst_virt_pipe_state *
bst_virt_pipe_get_state(struct bst_virt_pipe *pipe,
			struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_private_obj_state(state, &pipe->obj);
	if (IS_ERR(priv_st))
		return ERR_CAST(priv_st);

	return priv_to_pipe_st(priv_st);
}

static struct bst_virt_pipe_state *
bst_virt_pipe_get_old_state(struct bst_virt_pipe *pipe,
			    struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_old_private_obj_state(state, &pipe->obj);
	if (priv_st)
		return priv_to_pipe_st(priv_st);
	return NULL;
}

static struct bst_virt_pipe_state *
bst_virt_pipe_get_new_state(struct bst_virt_pipe *pipe,
			    struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_new_private_obj_state(state, &pipe->obj);
	if (priv_st)
		return priv_to_pipe_st(priv_st);
	return NULL;
}

static struct bst_virt_pipe_state *
bst_virt_pipe_get_state_and_set_crtc(struct bst_virt_pipe *pipe,
				     struct drm_atomic_state *state,
				     struct drm_crtc *crtc)
{
	struct bst_virt_pipe_state *st;

	st = bst_virt_pipe_get_state(pipe, state);
	if (IS_ERR(st))
		return st;

	if (is_switching_user(crtc, st->crtc)) {
		DRM_DEBUG_ATOMIC("CRTC%d required pipeline%d is busy.\n",
				 drm_crtc_index(crtc), pipe->pipe_id);
		return ERR_PTR(-EBUSY);
	}

	if (!crtc && st->active_comps) {
		DRM_DEBUG_ATOMIC("Disabling a busy pipeline:%d.\n",
				 pipe->pipe_id);
		return ERR_PTR(-EBUSY);
	}

	st->crtc = crtc;

	if (crtc) {
		struct bst_crtc_state *bcrtc_st;

		bcrtc_st =
			to_bcrtc_st(drm_atomic_get_new_crtc_state(state, crtc));

		bcrtc_st->active_pipes |= BIT(pipe->pipe_id);
		bcrtc_st->affected_pipes |= BIT(pipe->pipe_id);
	}
	return st;
}

static struct bst_virt_component_state *
bst_virt_component_get_state(struct bst_virt_component *c,
			     struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	WARN_ON(!drm_modeset_is_locked(&c->pipe->obj.lock));

	priv_st = drm_atomic_get_private_obj_state(state, &c->obj);
	if (IS_ERR(priv_st))
		return ERR_CAST(priv_st);

	return priv_to_comp_st(priv_st);
}

#if 0
static struct bst_virt_component_state *
bst_virt_component_get_old_state(struct bst_virt_component *c,
				 struct drm_atomic_state *state)
{
	struct drm_private_state *priv_st;

	priv_st = drm_atomic_get_old_private_obj_state(state, &c->obj);
	if (priv_st)
		return priv_to_comp_st(priv_st);
	return NULL;
}
#endif

static struct bst_virt_component_state *
bst_virt_component_get_state_and_set_user(struct bst_virt_component *c,
					  struct drm_atomic_state *state,
					  void *user, struct drm_crtc *crtc)
{
	struct bst_virt_pipe_state *pipe_st;
	struct bst_virt_component_state *st;

	pipe_st = bst_virt_pipe_get_state_and_set_crtc(c->pipe, state, crtc);
	if (IS_ERR(pipe_st))
		return ERR_CAST(pipe_st);

	st = bst_virt_component_get_state(c, state);
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
bst_virt_component_add_input(struct bst_virt_component_state *state,
			     struct bst_virt_component_output *input, int idx)
{
	struct bst_virt_component *c = state->component;

	WARN_ON((idx < 0 || idx >= c->max_active_inputs));

	DRM_DEBUG_ATOMIC("name:%s input name:%s idx:%d\n",input->component->name,state->inputs[idx].component->name,idx);
	if (!has_bit(idx, state->affected_inputs) ||
	    memcmp(&state->inputs[idx], input, sizeof(*input))) {
		memcpy(&state->inputs[idx], input, sizeof(*input));
		state->changed_active_inputs |= BIT(idx);
	}
	state->active_inputs |= BIT(idx);
	state->affected_inputs |= BIT(idx);
}

static int
bst_virt_component_check_input(struct bst_virt_component_state *state,
			       struct bst_virt_component_output *input, int idx)
{
	struct bst_virt_component *c = state->component;

	if ((idx < 0) || (idx >= c->max_active_inputs)) {
		DRM_ERROR("%s required an invalid %s-input[%d].\n",
				 input->component->name, c->name, idx);
		return -EINVAL;
	}

	if (has_bit(idx, state->active_inputs)) {
		DRM_ERROR(
			"%s required %s-input[%d] has been occupied already.\n",
			input->component->name, c->name, idx);
		return -EINVAL;
	}

	return 0;
}

static void
bst_virt_component_set_output(struct bst_virt_component_output *output,
			      struct bst_virt_component *comp, u8 output_port)
{
	output->component = comp;
	output->output_port = output_port;
}

static int
bst_virt_component_validate_private(struct bst_virt_component *c,
				    struct bst_virt_component_state *st)
{
	int err = 0;

	if (!c->funcs->validate)
		return 0;

	err = c->funcs->validate(c, st);
	if (err)
		DRM_DEBUG_ATOMIC("%s validate private failed.\n", c->name);

	return err;
}

static void bst_rotate_data_flow(struct bst_data_flow_cfg *dflow, u32 rot)
{
	if (drm_rotation_90_or_270(rot)) {
		swap(dflow->in_h, dflow->in_w);
		swap(dflow->total_in_h, dflow->total_in_w);
	}
}

static int bst_virt_layer_check_cfg(struct bst_virt_layer *layer,
				    struct bst_fb *kfb,
				    struct bst_data_flow_cfg *dflow)
{
	u32 src_x, src_y, src_w, src_h;
	u32 line_sz, max_line_sz;

	if (!bst_fb_is_layer_supported(kfb, layer->layer_type, dflow->rot))
		return -EINVAL;

	if (layer->base.id == BST_VIRT_COMPONENT_DC_WB_LAYER) {
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
		DRM_DEBUG_ATOMIC(
			"Required line_sz: %d exceeds the max size %d\n",
			line_sz, max_line_sz);
		return -EINVAL;
	}

	return 0;
}

static int bst_virt_layer_validate(struct bst_virt_layer *layer,
				   struct bst_plane_state *bplane_st,
				   struct bst_data_flow_cfg *dflow)
{
	struct drm_plane_state *plane_st = &bplane_st->base;
	struct drm_framebuffer *fb = plane_st->fb;
	struct bst_fb *kfb = to_bfb(fb);
	struct bst_virt_component_state *c_st;
	struct bst_virt_layer_state *st;
	int i, err;

	err = bst_virt_layer_check_cfg(layer, kfb, dflow);
	if (err)
		return err;

	c_st = bst_virt_component_get_state_and_set_user(
		&layer->base, plane_st->state, plane_st->plane, plane_st->crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_layer_st(c_st);

	st->rot = dflow->rot;

	if (fb->modifier) {
		st->hsize = kfb->aligned_w;
		st->vsize = kfb->aligned_h;
		st->afbc_crop.afbc_crop_l = dflow->in_x;
		st->afbc_crop.afbc_crop_r = kfb->aligned_w - dflow->in_x - dflow->in_w;
		st->afbc_crop.afbc_crop_t = dflow->in_y;
		st->afbc_crop.afbc_crop_b = kfb->aligned_h - dflow->in_y - dflow->in_h;
		st->afbc_crop.crop_type = DC_LAYER_CROP_TYPE_AFBC;
	} else {
		st->hsize = dflow->in_w;
		st->vsize = dflow->in_h;
		st->afbc_crop.afbc_crop_l = 0;
		st->afbc_crop.afbc_crop_r = 0;
		st->afbc_crop.afbc_crop_t = 0;
		st->afbc_crop.afbc_crop_b = 0;
		st->afbc_crop.crop_type = DC_LAYER_CROP_TYPE_NORMAL;
	}

	st->cin.hsize = dflow->out_w;
	st->cin.vsize = dflow->out_h;
	st->cin.hoffset = dflow->out_x;
	st->cin.voffset = dflow->out_y;
	st->cin.pixel_blend_mode = dflow->pixel_blend_mode;
	st->cin.layer_alpha = dflow->layer_alpha;

	for (i = 0; i < fb->format->num_planes; i++)
		st->addr[i] =
			bst_fb_get_pixel_addr(kfb, dflow->in_x, dflow->in_y, i);

	err = bst_virt_component_validate_private(&layer->base, c_st);
	if (err) {
		DRM_ERROR("component validate private failed. err=%d\n", err);
		return err;
	}

	bst_virt_component_set_output(&dflow->input, &layer->base, 0);

	bst_rotate_data_flow(dflow, st->rot);

	return 0;
}

#ifdef DISPLAY_SUPPORT_SCALE
static int downscaling_clk_check( struct drm_display_mode *mode,
				     unsigned long aclk_rate,
				     struct bst_data_flow_cfg *dflow)
{
	u32 h_in = dflow->in_w;
	u32 v_in = dflow->in_h;
	u32 v_out = dflow->out_h;
	u64 fraction, denominator;

	if (v_in == v_out) {
		fraction = h_in;
		denominator = mode->hdisplay - 3;
	} else {
		fraction = (u64)(h_in) * v_in;
		denominator = (mode->htotal - 1) * (u64)(v_out) -  2 * v_in;
	}

	return aclk_rate * denominator >= mode->crtc_clock * 1000 * fraction ?
	       0 : -EINVAL;
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
bst_scaler_check_cfg(struct bst_virt_layer* layer,
			struct bst_crtc_state *bcrtc_st,
			struct bst_data_flow_cfg *dflow)
{
	u32 hsize_in, vsize_in, hsize_out, vsize_out;
	int err;

	hsize_in = dflow->in_w;
	vsize_in = dflow->in_h;
	hsize_out = dflow->out_w;
	vsize_out = dflow->out_h;

	if (!in_range(&layer->scaler_hsize, hsize_in) ||
	    !in_range(&layer->scaler_hsize, hsize_out)) {
		DRM_ERROR("Invalid horizontal sizes");
		return -EINVAL;
	}

	if (!in_range(&layer->scaler_vsize, vsize_in) ||
	    !in_range(&layer->scaler_vsize, vsize_out)) {
		DRM_ERROR("Invalid vertical sizes");
		return -EINVAL;
	}

	if (!scaling_ratio_valid(hsize_in, hsize_out,
				layer->max_upscaling,
				layer->max_downscaling)) {
		DRM_ERROR("Invalid horizontal scaling ratio");
		return -EINVAL;
	}

	if (!scaling_ratio_valid(vsize_in, vsize_out,
				layer->max_upscaling,
				layer->max_downscaling)) {
		DRM_ERROR("Invalid vertical scaling ratio");
		return -EINVAL;
	}

	if (hsize_in > hsize_out || vsize_in > vsize_out) {
		err = downscaling_clk_check(&bcrtc_st->base.adjusted_mode,
					bst_crtc_get_aclk(bcrtc_st), dflow);
		if (err) {
			DRM_ERROR("aclk can't satisfy the clock requirement of the downscaling\n");
			return err;
		}
	}

	return 0;
}

static int
bst_virt_scaler_data_build(struct bst_virt_layer *layer,
					struct bst_plane_state *bplane_st,
					struct bst_crtc_state *bcrtc_st,
					struct bst_data_flow_cfg *dflow)
{
	struct drm_plane_state *plane_st = &bplane_st->base;
	struct bst_virt_component_state *c_st;
	struct bst_virt_layer_state *layer_st;
	struct bst_scaler_cfg* scaler;
	int err;

	c_st = bst_virt_component_get_state_and_set_user(
		&layer->base, plane_st->state, plane_st->plane, plane_st->crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	layer_st = to_layer_st(c_st);
	scaler = &layer_st->scaler;

	memset(scaler, 0, sizeof(*scaler));
	if (!(dflow->en_scaling || dflow->en_img_enhancement))
		return 0;

	err = bst_scaler_check_cfg(layer, bcrtc_st, dflow);
	if (err) {
		DRM_ERROR("check scaler configure failed, err=%d\n", err);
		return err;
	}

	scaler->hsize_in = dflow->in_w;
	scaler->vsize_in = dflow->in_h;
	scaler->hsize_out = dflow->out_w;
	scaler->vsize_out = dflow->out_h;
	scaler->right_crop = dflow->right_crop;
	scaler->left_crop = dflow->left_crop;
	scaler->total_vsize_in = dflow->total_in_h;
	scaler->total_hsize_in = dflow->total_in_w;
	scaler->total_hsize_out = dflow->total_out_w;
	scaler->en_alpha = dflow->pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE;
	scaler->en_scaling = dflow->en_scaling;
	scaler->en_img_enhancement = dflow->en_img_enhancement;

	return err;
}
#endif

static int bst_wb_layer_validate(struct bst_virt_layer *wb_layer,
				 struct drm_connector_state *conn_st,
				 struct bst_data_flow_cfg *dflow)
{
	struct bst_fb *kfb = to_bfb(conn_st->writeback_job->fb);
	struct bst_virt_component_state *c_st;
	struct bst_virt_layer_state *st;
	int i, err;

	err = bst_virt_layer_check_cfg(wb_layer, kfb, dflow);
	if (err)
		return err;

	c_st = bst_virt_component_get_state_and_set_user(&wb_layer->base,
							 conn_st->state,
							 conn_st->connector,
							 conn_st->crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_layer_st(c_st);

	st->hsize = dflow->out_w;
	st->vsize = dflow->out_h;

	for (i = 0; i < kfb->base.format->num_planes; i++)
		st->addr[i] = bst_fb_get_pixel_addr(kfb, dflow->out_x,
						    dflow->out_y, i);

	bst_virt_component_add_input(&st->base, &dflow->input, 0);
	bst_virt_component_set_output(&dflow->input, &wb_layer->base, 0);

	return 0;
}

void bst_virt_pipe_dc_crtc_size(struct bst_crtc_state *bcrtc_st, u16 *hsize,
			       u16 *vsize)
{
	struct drm_display_mode *m = &bcrtc_st->base.adjusted_mode;

	if (hsize)
		*hsize = m->hdisplay;
	if (vsize)
		*vsize = m->vdisplay;
}

static int bst_virt_crtc_set_input(struct bst_virt_dc_crtc *crtc,
				     struct bst_crtc_state *bcrtc_st,
				     struct bst_data_flow_cfg *dflow)
{
	struct drm_atomic_state *drm_st = bcrtc_st->base.state;
	struct bst_virt_component_state *c_st;
	u16 crtc_w, crtc_h;
	int idx = dflow->blending_zorder;

	bst_virt_pipe_dc_crtc_size(bcrtc_st, &crtc_w, &crtc_h);
	if ((dflow->out_x + dflow->out_w > crtc_w) ||
	    (dflow->out_y + dflow->out_h > crtc_h) || dflow->out_w == 0 ||
	    dflow->out_h == 0) {
		DRM_ERROR("invalid disp rect [x=%d, y=%d, w=%d, h=%d],crtc [x=%d, y=%d]\n",
				 dflow->out_x, dflow->out_y, dflow->out_w,
				 dflow->out_h, crtc_w, crtc_h);
		return -EINVAL;
	}

	c_st = bst_virt_component_get_state_and_set_user(&crtc->base, drm_st,
							 bcrtc_st->base.crtc,
							 bcrtc_st->base.crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	if (bst_virt_component_check_input(c_st, &dflow->input, idx))
		return -EINVAL;

	/* Scaling changes require updating crtc */
	if (bcrtc_st->en_scaling != dflow->en_scaling) {
		bcrtc_st->en_scaling = dflow->en_scaling;
		c_st->changed_active_inputs |= BIT(idx);
	}

	bst_virt_component_add_input(c_st, &dflow->input, idx);
	bst_virt_component_set_output(&dflow->input, &crtc->base, 0);

	return 0;
}


static int bst_virt_dc_crtc_validate(struct bst_virt_dc_crtc *dc_crtc,
				    struct bst_crtc_state *bcrtc_st,
				    struct bst_data_flow_cfg *dflow)
{
	struct drm_crtc *crtc = bcrtc_st->base.crtc;
	struct drm_crtc_state *crtc_st = &bcrtc_st->base;
	struct bst_virt_dc_crtc_state *st;
	struct bst_virt_component_state *c_st;

	c_st = bst_virt_component_get_state_and_set_user(
		&dc_crtc->base, bcrtc_st->base.state, crtc, crtc);
	if (IS_ERR(c_st))
		return PTR_ERR(c_st);

	st = to_dc_crtc_st(c_st);

	st->hsize = dflow->in_w;
	st->vsize = dflow->in_h;

	if (drm_atomic_crtc_needs_modeset(crtc_st)) {
		u32 output_depths, output_formats;
		u32 avail_depths, avail_formats;

		bst_crtc_get_color_config(crtc_st, &output_depths,
					  &output_formats);

		avail_depths = output_depths & dc_crtc->supported_color_depths;
		if (avail_depths == 0) {
			DRM_ERROR(
				"No available color depths, conn depths: 0x%x & display: 0x%x\n",
				output_depths, dc_crtc->supported_color_depths);
			return -EINVAL;
		}

		avail_formats = output_formats &
				dc_crtc->supported_color_formats;
		if (!avail_formats) {
			DRM_ERROR(
				"No available color_formats, conn formats 0x%x & display: 0x%x\n",
				output_formats,
				dc_crtc->supported_color_formats);
			return -EINVAL;
		}

		st->color_depth = __fls(avail_depths);
		st->color_format = BIT(__ffs(avail_formats));
		DRM_INFO("[%s:%d] color_depth:%d color_format:%d.\n",__func__,__LINE__,st->color_depth,st->color_format);
	}

	if (bcrtc_st->base.color_mgmt_changed) {
		drm_lut_to_fgamma_coeffs(bcrtc_st->base.gamma_lut,
					 st->fgamma_coeffs);
		drm_ctm_to_coeffs(bcrtc_st->base.ctm, st->ctm_coeffs);
	}

	bst_virt_pipe_dc_crtc_size(bcrtc_st, &st->hsize, &st->vsize);

	dflow->in_w = st->hsize;
	dflow->in_h = st->vsize;
	dflow->out_w = dflow->in_w;
	dflow->out_h = dflow->in_h;
	dflow->pixel_blend_mode = DRM_MODE_BLEND_PIXEL_NONE;
	dflow->layer_alpha = 0xFF;
	dflow->blending_zorder = 0;

	return 0;
}

#if 0
//#ifdef DISPLAY_SUPPORT_SCALE
static void print_scaler_slot(struct bst_virt_layer *layer, bool is_add)
{
	uint8_t scaler_num = layer->base.pipe->max_scaler_num, i;
	struct bst_virt_layer *layer_scaler;


	for (i = 0; i < scaler_num; i++) {
		layer_scaler = layer->base.pipe->scaler_slot[i];
		if (layer_scaler)
			DRM_DEBUG("%s input-%s: scaler_ch:%d, slot[%d]-name:%s-valid_scaler_ch:%d",
			is_add ? "Add" : "Remove", layer->base.name, layer->valid_scaler_channel, i,
			layer_scaler->base.name, layer_scaler->valid_scaler_channel);
		else
			DRM_DEBUG("%s input-%s: scaler_ch:%d, slot[%d]-name:NULL-valid_scaler_ch:NULL",
			is_add ? "Add" : "Remove", layer->base.name, layer->valid_scaler_channel, i);
	}
}

void remove_scaler_from_slot(struct bst_virt_layer *layer)
{
	uint8_t ch = layer->valid_scaler_channel;
	struct bst_virt_layer *layer_scaler = layer->base.pipe->scaler_slot[ch];

	if (ch != BST_VIRT_DC_SCALER_NULL) {
		if (layer_scaler && layer_scaler == layer) {
			layer->base.pipe->scaler_slot[ch] = NULL;
			print_scaler_slot(layer, false);
		}
	}
}

static int add_scaler_to_slot(struct bst_virt_layer *layer)
{
	uint8_t ch = layer->valid_scaler_channel;
	struct bst_virt_layer *layer_scaler;

	if (ch != BST_VIRT_DC_SCALER_NULL) {
		layer_scaler = layer->base.pipe->scaler_slot[ch];
		if (layer_scaler && layer_scaler == layer) {
			DRM_DEBUG("scaler ch:%d for same layer(%s), no need add twice\n", ch, layer_scaler->base.name);
			print_scaler_slot(layer, true);
			return 0;
		} else if (layer_scaler && layer_scaler != layer) {
			DRM_DEBUG("%s want the scaler slot[%d] but current be used by %s",
				layer->base.name, ch , layer_scaler->base.name);
			print_scaler_slot(layer, true);
			return -1;
		}
		layer->base.pipe->scaler_slot[ch] = layer;
		print_scaler_slot(layer, true);
		return 0;
	}

	DRM_ERROR("not support scaler.\n");
	return -1;
}
#endif

int bst_complete_data_flow_cfg(struct bst_virt_layer *layer,
				struct bst_data_flow_cfg *dflow,
				struct drm_framebuffer *fb)
{
	u32 w = dflow->in_w;
	u32 h = dflow->in_h;
	int ret = 0;

	dflow->total_in_w = dflow->in_w;
	dflow->total_in_h = dflow->in_h;
	dflow->total_out_w = dflow->out_w;

	if (!fb->format->has_alpha)
		dflow->pixel_blend_mode = DRM_MODE_BLEND_PIXEL_NONE;

	if (drm_rotation_90_or_270(dflow->rot))
		swap(w, h);

#ifdef DISPLAY_SUPPORT_SCALE
	dflow->en_scaling = (w != dflow->out_w) || (h != dflow->out_h);
	//if (dflow->en_scaling) {
	//	ret = add_scaler_to_slot(layer);
	//	if (ret)
	//		return ret;
	//} else
	//	remove_scaler_from_slot(layer);
#endif

	dflow->is_yuv = fb->format->is_yuv;
	dflow->en_img_enhancement = dflow->out_w >= 2 * w ||
				    dflow->out_h >= 2 * h;

	return ret;
}

int bst_build_layer_data_flow(struct bst_virt_layer *layer,
			      struct bst_plane_state *bplane_st,
			      struct bst_crtc_state *bcrtc_st,
			      struct bst_data_flow_cfg *dflow)
{
	struct drm_plane *plane = bplane_st->base.plane;
	struct bst_virt_pipe *pipe = layer->base.pipe;
	int err;

	DRM_DEBUG_ATOMIC(
		"%s handling [PLANE:%d:%s]: src[x/y:%d/%d, w/h:%d/%d] disp[x/y:%d/%d, w/h:%d/%d, blend/alpha:%d/%d]",
		layer->base.name, plane->base.id, plane->name, dflow->in_x,
		dflow->in_y, dflow->in_w, dflow->in_h, dflow->out_x,
		dflow->out_y, dflow->out_w, dflow->out_h, dflow->pixel_blend_mode, dflow->layer_alpha);

	err = bst_virt_layer_validate(layer, bplane_st, dflow);
	if (err)
		return err;

#ifdef DISPLAY_SUPPORT_SCALE
	err = bst_virt_scaler_data_build(layer, bplane_st, bcrtc_st, dflow);
	if (err) {
		DRM_ERROR("bst_virt_scaler_data_build err:%d\n", err);
		return err;
	}
#endif

	err = bst_virt_crtc_set_input(pipe->dc_crtc, bcrtc_st, dflow);
	return err;
}

int bst_build_wb_data_flow(struct bst_virt_layer *wb_layer,
			   struct drm_connector_state *conn_st,
			   struct bst_crtc_state *bcrtc_st,
			   struct bst_data_flow_cfg *dflow)
{
	return bst_wb_layer_validate(wb_layer, conn_st, dflow);
}

int bst_build_display_data_flow(struct bst_crtc *bcrtc,
				struct bst_crtc_state *bcrtc_st)
{
	struct bst_virt_pipe *master = bcrtc->master;
	struct bst_data_flow_cfg m_dflow;
	int err;

	memset(&m_dflow, 0, sizeof(m_dflow));
	err = bst_virt_dc_crtc_validate(master->dc_crtc, bcrtc_st, &m_dflow);
	if (err)
		return err;
	return 0;
}

static void bst_virt_pipe_unbound_components(struct bst_virt_pipe *pipe,
					     struct bst_virt_pipe_state *new)
{
	struct drm_atomic_state *drm_st = new->obj.state;
	struct bst_virt_pipe_state *old = priv_to_pipe_st(pipe->obj.state);
	struct bst_virt_component_state *c_st;
	struct bst_virt_component *c;
	u32 id;
	unsigned long disabling_comps;

	WARN_ON(!old);

	disabling_comps = (~new->active_comps) & old->active_comps;

	for_each_set_bit(id, &disabling_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, id);
		c_st = bst_virt_component_get_state_and_set_user(
			c, drm_st, NULL, new->crtc);
		WARN_ON(IS_ERR(c_st));
	}
}

int bst_release_unclaimed_resources(struct bst_virt_pipe *pipe,
				    struct bst_crtc_state *bcrtc_st)
{
	struct drm_atomic_state *drm_st = bcrtc_st->base.state;
	struct bst_virt_pipe_state *st;

	if (!pipe || !has_bit(pipe->pipe_id, bcrtc_st->affected_pipes))
		return 0;

	if (has_bit(pipe->pipe_id, bcrtc_st->active_pipes))
		st = bst_virt_pipe_get_new_state(pipe, drm_st);
	else
		st = bst_virt_pipe_get_state_and_set_crtc(pipe, drm_st, NULL);

	if (WARN_ON(IS_ERR_OR_NULL(st)))
		return -EINVAL;

	bst_virt_pipe_unbound_components(pipe, st);

	return 0;
}

bool bst_virt_pipe_disable(struct bst_virt_pipe *pipe,
			   struct drm_atomic_state *old_state)
{
	struct bst_virt_pipe_state *old;
	struct bst_virt_component *c;
	struct bst_virt_component_state *c_st;
	u32 id;
	unsigned long disabling_comps;

	old = bst_virt_pipe_get_old_state(pipe, old_state);

	disabling_comps = old->active_comps &
			  (~pipe->standalone_disabled_comps);
	if (!disabling_comps)
		disabling_comps = old->active_comps &
				  pipe->standalone_disabled_comps;

	DRM_DEBUG_ATOMIC(
		"PIPE%d: active_comps: 0x%x, disabling_comps: 0x%lx.\n",
		pipe->pipe_id, old->active_comps, disabling_comps);

	for_each_set_bit(id, &disabling_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, id);
		c_st = priv_to_comp_st(c->obj.state);
		c_st->changed_active_inputs |= c_st->active_inputs;

		c->funcs->disable(c);
	}

	/* Update the pipeline state, if there are components that are still
	 * active, return true for calling the phase 2 disable.
	 */
	old->active_comps &= ~disabling_comps;

	return old->active_comps ? true : false;
}

void bst_virt_pipe_update(struct bst_virt_pipe *pipe,
			  struct drm_atomic_state *old_state)
{
	struct bst_virt_pipe_state *new = priv_to_pipe_st(pipe->obj.state);
	struct bst_virt_pipe_state *old;
	struct bst_virt_component *c;
	u32 id;
	unsigned long changed_comps;

	old = bst_virt_pipe_get_old_state(pipe, old_state);

	changed_comps = new->active_comps | old->active_comps;

	DRM_DEBUG_ATOMIC("PIPE%d: active_comps: 0x%x, changed: 0x%lx.\n",
			 pipe->pipe_id, new->active_comps, changed_comps);

	for_each_set_bit(id, &changed_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, id);
		if (new->active_comps & BIT(c->id)) {
			c->funcs->update(c, priv_to_comp_st(c->obj.state));
		} else
			c->funcs->disable(c);
	}
}
