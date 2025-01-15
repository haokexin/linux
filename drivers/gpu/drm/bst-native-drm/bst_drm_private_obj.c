// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#include "bst_drm_dev.h"
#include "bst_drm_kms.h"

static void
bst_component_state_reset(struct bst_component_state *st)
{
	st->binding_user = NULL;
	st->affected_inputs = st->active_inputs;
	st->active_inputs = 0;
	st->changed_active_inputs = 0;
}

static struct drm_private_state *
bst_layer_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_layer_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_layer_atomic_destroy_state(struct drm_private_obj *obj,
				  struct drm_private_state *state)
{
	struct bst_layer_state *st = to_layer_st(priv_to_comp_st(state));

	kfree(st);
}

static const struct drm_private_state_funcs bst_layer_obj_funcs = {
	.atomic_duplicate_state	= bst_layer_atomic_duplicate_state,
	.atomic_destroy_state	= bst_layer_atomic_destroy_state,
};

static int bst_layer_obj_add(struct bst_kms_dev *kms,
				struct bst_layer *layer)
{
	struct bst_layer_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &layer->base;
	drm_atomic_private_obj_init(&kms->base, &layer->base.obj, &st->base.obj,
				    &bst_layer_obj_funcs);
	return 0;
}

static struct drm_private_state *
bst_scaler_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_scaler_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_scaler_atomic_destroy_state(struct drm_private_obj *obj,
				   struct drm_private_state *state)
{
	kfree(to_scaler_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_scaler_obj_funcs = {
	.atomic_duplicate_state	= bst_scaler_atomic_duplicate_state,
	.atomic_destroy_state	= bst_scaler_atomic_destroy_state,
};

static int bst_scaler_obj_add(struct bst_kms_dev *kms,
				 struct bst_scaler *scaler)
{
	struct bst_scaler_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &scaler->base;
	drm_atomic_private_obj_init(&kms->base,
				    &scaler->base.obj, &st->base.obj,
				    &bst_scaler_obj_funcs);
	return 0;
}

static struct drm_private_state *
bst_compiz_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_compiz_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_compiz_atomic_destroy_state(struct drm_private_obj *obj,
				   struct drm_private_state *state)
{
	kfree(to_compiz_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_compiz_obj_funcs = {
	.atomic_duplicate_state	= bst_compiz_atomic_duplicate_state,
	.atomic_destroy_state	= bst_compiz_atomic_destroy_state,
};

static int bst_compiz_obj_add(struct bst_kms_dev *kms,
				 struct bst_compiz *compiz)
{
	struct bst_compiz_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &compiz->base;
	drm_atomic_private_obj_init(&kms->base, &compiz->base.obj, &st->base.obj,
				    &bst_compiz_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_splitter_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_splitter_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_splitter_atomic_destroy_state(struct drm_private_obj *obj,
				     struct drm_private_state *state)
{
	kfree(to_splitter_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_splitter_obj_funcs = {
	.atomic_duplicate_state	= bst_splitter_atomic_duplicate_state,
	.atomic_destroy_state	= bst_splitter_atomic_destroy_state,
};

static int bst_splitter_obj_add(struct bst_kms_dev *kms,
				   struct bst_splitter *splitter)
{
	struct bst_splitter_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &splitter->base;
	drm_atomic_private_obj_init(&kms->base,
				    &splitter->base.obj, &st->base.obj,
				    &bst_splitter_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_merger_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_merger_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void bst_merger_atomic_destroy_state(struct drm_private_obj *obj,
					       struct drm_private_state *state)
{
	kfree(to_merger_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_merger_obj_funcs = {
	.atomic_duplicate_state	= bst_merger_atomic_duplicate_state,
	.atomic_destroy_state	= bst_merger_atomic_destroy_state,
};

static int bst_merger_obj_add(struct bst_kms_dev *kms,
				 struct bst_merger *merger)
{
	struct bst_merger_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &merger->base;
	drm_atomic_private_obj_init(&kms->base,
				    &merger->base.obj, &st->base.obj,
				    &bst_merger_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_improc_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_improc_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_improc_atomic_destroy_state(struct drm_private_obj *obj,
				   struct drm_private_state *state)
{
	kfree(to_improc_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_improc_obj_funcs = {
	.atomic_duplicate_state	= bst_improc_atomic_duplicate_state,
	.atomic_destroy_state	= bst_improc_atomic_destroy_state,
};

static int bst_improc_obj_add(struct bst_kms_dev *kms,
				 struct bst_improc *improc)
{
	struct bst_improc_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &improc->base;
	drm_atomic_private_obj_init(&kms->base, &improc->base.obj, &st->base.obj,
				    &bst_improc_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_timing_ctrlr_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_timing_ctrlr_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	bst_component_state_reset(&st->base);
	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->base.obj);

	return &st->base.obj;
}

static void
bst_timing_ctrlr_atomic_destroy_state(struct drm_private_obj *obj,
					 struct drm_private_state *state)
{
	kfree(to_ctrlr_st(priv_to_comp_st(state)));
}

static const struct drm_private_state_funcs bst_timing_ctrlr_obj_funcs = {
	.atomic_duplicate_state	= bst_timing_ctrlr_atomic_duplicate_state,
	.atomic_destroy_state	= bst_timing_ctrlr_atomic_destroy_state,
};

static int bst_timing_ctrlr_obj_add(struct bst_kms_dev *kms,
				       struct bst_timing_ctrlr *ctrlr)
{
	struct bst_compiz_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->base.component = &ctrlr->base;
	drm_atomic_private_obj_init(&kms->base, &ctrlr->base.obj, &st->base.obj,
				    &bst_timing_ctrlr_obj_funcs);

	return 0;
}

static struct drm_private_state *
bst_pipeline_atomic_duplicate_state(struct drm_private_obj *obj)
{
	struct bst_pipeline_state *st;

	st = kmemdup(obj->state, sizeof(*st), GFP_KERNEL);
	if (!st)
		return NULL;

	st->active_comps = 0;

	__drm_atomic_helper_private_obj_duplicate_state(obj, &st->obj);

	return &st->obj;
}

static void
bst_pipeline_atomic_destroy_state(struct drm_private_obj *obj,
				     struct drm_private_state *state)
{
	kfree(priv_to_pipe_st(state));
}

static const struct drm_private_state_funcs bst_pipeline_obj_funcs = {
	.atomic_duplicate_state	= bst_pipeline_atomic_duplicate_state,
	.atomic_destroy_state	= bst_pipeline_atomic_destroy_state,
};

static int bst_pipeline_obj_add(struct bst_kms_dev *kms,
				   struct bst_pipeline *pipe)
{
	struct bst_pipeline_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->pipe = pipe;
	drm_atomic_private_obj_init(&kms->base, &pipe->obj, &st->obj,
				    &bst_pipeline_obj_funcs);

	return 0;
}

int bst_kms_add_private_objs(struct bst_kms_dev *kms,
				struct bst_dev *mdev)
{
	struct bst_pipeline *pipe;
	int i, j, err;

	for (i = 0; i < mdev->n_pipelines; i++) {
		pipe = mdev->pipelines[i];

		err = bst_pipeline_obj_add(kms, pipe);
		if (err)
			return err;

		for (j = 0; j < pipe->n_layers; j++) {
			err = bst_layer_obj_add(kms, pipe->layers[j]);
			if (err)
				return err;
		}

		if (pipe->wb_layer) {
			err = bst_layer_obj_add(kms, pipe->wb_layer);
			if (err)
				return err;
		}

		for (j = 0; j < pipe->n_scalers; j++) {
			err = bst_scaler_obj_add(kms, pipe->scalers[j]);
			if (err)
				return err;
		}

		err = bst_compiz_obj_add(kms, pipe->compiz);
		if (err)
			return err;

		if (pipe->splitter) {
			err = bst_splitter_obj_add(kms, pipe->splitter);
			if (err)
				return err;
		}

		if (pipe->merger) {
			err = bst_merger_obj_add(kms, pipe->merger);
			if (err)
				return err;
		}

		err = bst_improc_obj_add(kms, pipe->improc);
		if (err)
			return err;

		err = bst_timing_ctrlr_obj_add(kms, pipe->ctrlr);
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
