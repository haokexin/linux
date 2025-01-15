// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "bst_virt_drm_device.h"
#include "bst_virt_drm_kms.h"

static void bst_component_state_reset(struct bst_virt_component_state *st)
{
	st->binding_user = NULL;
	st->affected_inputs = st->active_inputs;
	st->active_inputs = 0;
	st->changed_active_inputs = 0;
}

static struct drm_private_state *
bst_layer_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_virt_layer_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void bst_layer_atomic_destroy_state(struct drm_private_obj *obj,
					   struct drm_private_state *state)
{
	struct bst_virt_layer_state *st = to_layer_st(priv_to_comp_st(state));

	kfree(st);
}

static const struct drm_private_state_funcs bst_layer_obj_funcs = {
	.atomic_duplicate_state = bst_layer_atomic_duplicate_state,
	.atomic_destroy_state = bst_layer_atomic_destroy_state,
};

static int bst_layer_obj_add(struct bst_kms_dev *kms,
			     struct bst_virt_layer *layer)
{
	struct bst_virt_layer_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &layer->base;
	drm_atomic_private_obj_init(&kms->base, &layer->base.obj, &st->base.obj,
				    &bst_layer_obj_funcs);
	return 0;
}

static struct drm_private_state *
bst_virt_dc_crtc_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_virt_dc_crtc_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_virt_dc_crtc_atomic_destroy_state(struct drm_private_obj *obj,
				     struct drm_private_state *state)
{
	kfree(to_dc_crtc_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_virt_dc_crtc_obj_funcs = {
	.atomic_duplicate_state = bst_virt_dc_crtc_atomic_duplicate_state,
	.atomic_destroy_state = bst_virt_dc_crtc_atomic_destroy_state,
};

static int bst_virt_dc_crtc_obj_add(struct bst_kms_dev *kms,
				   struct bst_virt_dc_crtc *dc_crtc)
{
	struct bst_virt_dc_crtc_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &dc_crtc->base;
	drm_atomic_private_obj_init(&kms->base, &dc_crtc->base.obj,
				    &st->base.obj, &bst_virt_dc_crtc_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_virt_pipe_state_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_virt_pipe_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	st->active_comps = 0;

	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->obj);

	return &st->obj;
}

static void
bst_virt_pipe_state_atomic_destroy_state(struct drm_private_obj *obj,
					 struct drm_private_state *state)
{
	kfree(priv_to_pipe_st(state));
}

static const struct drm_private_state_funcs bst_virt_pipe_obj_funcs = {
	.atomic_duplicate_state = bst_virt_pipe_state_atomic_duplicate_state,
	.atomic_destroy_state = bst_virt_pipe_state_atomic_destroy_state,
};

static int bst_virt_pipe_obj_add(struct bst_kms_dev *kms,
				 struct bst_virt_pipe *pipe)
{
	struct bst_virt_pipe_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->pipe = pipe;
	drm_atomic_private_obj_init(&kms->base, &pipe->obj, &st->obj,
				    &bst_virt_pipe_obj_funcs);

	return 0;
}

int bst_kms_add_private_objs(struct bst_kms_dev *kms,
			     struct bst_super_device *super_dev)
{
	struct bst_virt_pipe *pipe;
	int i, j, err;

	for (i = 0; i < super_dev->n_pipelines; i++) {
		pipe = super_dev->pipelines[i];
		err = bst_virt_pipe_obj_add(kms, pipe);
		if (err)
			return err;
		for (j = 0; j < pipe->n_dc_layers; j++) {
			err = bst_layer_obj_add(kms, pipe->dc_layers[j]);
			if (err)
				return err;
		}

		if (pipe->dc_wb_layer) {
			err = bst_layer_obj_add(kms, pipe->dc_wb_layer);
			if (err)
				return err;
		}

		err = bst_virt_dc_crtc_obj_add(kms, pipe->dc_crtc);
		if (err)
			return err;
	}

	return 0;
}

void bst_kms_cleanup_private_objs(struct bst_kms_dev *kms)
{
	struct drm_mode_config *config = &kms->base.mode_config;
	struct drm_private_obj *obj, *next;

	list_for_each_entry_safe(obj, next, &config->privobj_list, head)
		drm_atomic_private_obj_fini(obj);
}
