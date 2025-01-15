// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <drm/drm_print.h>
#include <linux/of.h>

#include "bst_drm_dev.h"
#include "bst_pipeline.h"

struct bst_pipeline *
bst_pipeline_add(struct bst_dev *mdev, size_t size,
		    const struct bst_pipeline_funcs *funcs)
{
	struct bst_pipeline *pipe;

	if (mdev->n_pipelines + 1 > BST_DRM_MAX_PIPELINES) {
		DRM_ERROR("Exceed max support %d pipelines.\n",
			  BST_DRM_MAX_PIPELINES);
		return ERR_PTR(-ENOSPC);
	}

	if (size < sizeof(*pipe)) {
		DRM_ERROR("Request pipeline size too small.\n");
		return ERR_PTR(-EINVAL);
	}

	pipe = devm_kzalloc(mdev->dev, size, GFP_KERNEL);
	if (!pipe)
		return ERR_PTR(-ENOMEM);

	pipe->mdev = mdev;
	pipe->id   = mdev->n_pipelines;
	pipe->funcs = funcs;

	mdev->pipelines[mdev->n_pipelines] = pipe;
	mdev->n_pipelines++;

	return pipe;
}
EXPORT_SYMBOL(bst_pipeline_add);
void bst_pipeline_destroy(struct bst_dev *mdev,
			     struct bst_pipeline *pipe)
{
	struct bst_component *c;
	int i;
	unsigned long avail_comps = pipe->avail_comps;

	for_each_set_bit(i, &avail_comps, 32) {
		c = bst_pipeline_get_component(pipe, i);
		bst_component_destroy(mdev, c);
	}

	clk_put(pipe->gate_pxlclk);

	of_node_put(pipe->of_output_links[0]);
	of_node_put(pipe->of_output_links[1]);
	of_node_put(pipe->of_output_port);
	of_node_put(pipe->of_node);

	devm_kfree(mdev->dev, pipe);
}

static struct bst_component **
bst_pipeline_get_component_pos(struct bst_pipeline *pipe, int id)
{
	struct bst_dev *mdev = pipe->mdev;
	struct bst_pipeline *temp = NULL;
	struct bst_component **pos = NULL;

	switch (id) {
	case BST_DRM_COMPONENT_LAYER0:
	case BST_DRM_COMPONENT_LAYER1:
	case BST_DRM_COMPONENT_LAYER2:
	case BST_DRM_COMPONENT_LAYER3:
		pos = to_cpos(pipe->layers[id - BST_DRM_COMPONENT_LAYER0]);
		break;
	case BST_DRM_COMPONENT_WB_LAYER:
		pos = to_cpos(pipe->wb_layer);
		break;
	case BST_DRM_COMPONENT_COMPIZ0:
	case BST_DRM_COMPONENT_COMPIZ1:
		temp = mdev->pipelines[id - BST_DRM_COMPONENT_COMPIZ0];
		if (!temp) {
			DRM_ERROR("compiz-%d doesn't exist.\n", id);
			return NULL;
		}
		pos = to_cpos(temp->compiz);
		break;
	case BST_DRM_COMPONENT_SCALER0:
	case BST_DRM_COMPONENT_SCALER1:
		pos = to_cpos(pipe->scalers[id - BST_DRM_COMPONENT_SCALER0]);
		break;
	case BST_DRM_COMPONENT_SPLITTER:
		pos = to_cpos(pipe->splitter);
		break;
	case BST_DRM_COMPONENT_MERGER:
		pos = to_cpos(pipe->merger);
		break;
	case BST_DRM_COMPONENT_IPS0:
	case BST_DRM_COMPONENT_IPS1:
		temp = mdev->pipelines[id - BST_DRM_COMPONENT_IPS0];
		if (!temp) {
			DRM_ERROR("ips-%d doesn't exist.\n", id);
			return NULL;
		}
		pos = to_cpos(temp->improc);
		break;
	case BST_DRM_COMPONENT_TIMING_CTRLR:
		pos = to_cpos(pipe->ctrlr);
		break;
	default:
		pos = NULL;
		DRM_ERROR("Unknown pipeline resource ID: %d.\n", id);
		break;
	}

	return pos;
}

struct bst_component *
bst_pipeline_get_component(struct bst_pipeline *pipe, int id)
{
	struct bst_component **pos = NULL;
	struct bst_component *c = NULL;

	pos = bst_pipeline_get_component_pos(pipe, id);
	if (pos)
		c = *pos;

	return c;
}

struct bst_component *
bst_pipeline_get_first_component(struct bst_pipeline *pipe,
				    u32 comp_mask)
{
	struct bst_component *c = NULL;
	unsigned long comp_mask_local = (unsigned long)comp_mask;
	int id;

	id = find_first_bit(&comp_mask_local, 32);
	if (id < 32)
		c = bst_pipeline_get_component(pipe, id);

	return c;
}

static struct bst_component *
bst_component_pickup_input(struct bst_component *c, u32 avail_comps)
{
	u32 avail_inputs = c->supported_inputs & (avail_comps);

	return bst_pipeline_get_first_component(c->pipeline, avail_inputs);
}

struct bst_component *
bst_component_add(struct bst_pipeline *pipe,
		     size_t comp_sz, u32 id, u32 hw_id,
		     const struct bst_component_funcs *funcs,
		     u8 max_active_inputs, u32 supported_inputs,
		     u8 max_active_outputs, u32 __iomem *reg,
		     const char *name_fmt, ...)
{
	struct bst_component **pos;
	struct bst_component *c;
	int idx, *num = NULL;

	if (max_active_inputs > BST_DRM_COMPONENT_N_INPUTS) {
		WARN(1, "please large BST_DRM_COMPONENT_N_INPUTS to %d.\n",
		     max_active_inputs);
		return ERR_PTR(-ENOSPC);
	}

	pos = bst_pipeline_get_component_pos(pipe, id);
	if (!pos || (*pos))
		return ERR_PTR(-EINVAL);

	if (has_bit(id, BST_DRM_PIPELINE_LAYERS)) {
		idx = id - BST_DRM_COMPONENT_LAYER0;
		num = &pipe->n_layers;
		if (idx != pipe->n_layers) {
			DRM_ERROR("please add Layer by id sequence.\n");
			return ERR_PTR(-EINVAL);
		}
	} else if (has_bit(id,  BST_DRM_PIPELINE_SCALERS)) {
		idx = id - BST_DRM_COMPONENT_SCALER0;
		num = &pipe->n_scalers;
		if (idx != pipe->n_scalers) {
			DRM_ERROR("please add Scaler by id sequence.\n");
			return ERR_PTR(-EINVAL);
		}
	}

	c = devm_kzalloc(pipe->mdev->dev, comp_sz, GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->id = id;
	c->hw_id = hw_id;
	c->reg = reg;
	c->pipeline = pipe;
	c->max_active_inputs = max_active_inputs;
	c->max_active_outputs = max_active_outputs;
	c->supported_inputs = supported_inputs;
	c->funcs = funcs;

	if (name_fmt) {
		va_list args;

		va_start(args, name_fmt);
		vsnprintf(c->name, sizeof(c->name), name_fmt, args);
		va_end(args);
	}

	if (num)
		*num = *num + 1;

	pipe->avail_comps |= BIT(c->id);
	*pos = c;

	return c;
}
EXPORT_SYMBOL(bst_component_add);
void bst_component_destroy(struct bst_dev *mdev,
			      struct bst_component *c)
{
	devm_kfree(mdev->dev, c);
}

static void bst_component_dump(struct bst_component *c)
{
	if (!c)
		return;

	DRM_DEBUG("	%s: ID %d-0x%08lx.\n",
		  c->name, c->id, BIT(c->id));
	DRM_DEBUG("		max_active_inputs:%d, supported_inputs: 0x%08x.\n",
		  c->max_active_inputs, c->supported_inputs);
	DRM_DEBUG("		max_active_outputs:%d, supported_outputs: 0x%08x.\n",
		  c->max_active_outputs, c->supported_outputs);
}

struct pixel_clk_map {
	int pix_clk_chan;
	char * pix_clk_chan_name;
};

static void bst_pipeline_dump(struct bst_pipeline *pipe)
{
	struct bst_component *c;
	int id;
	unsigned long avail_comps = pipe->avail_comps;

	DRM_INFO("Pipeline-%d: n_layers: %d, n_scalers: %d, output: %s.\n",
		 pipe->id, pipe->n_layers, pipe->n_scalers,
		 pipe->dual_link ? "dual-link" : "single-link");
	DRM_INFO("	output_link[0]: %s.\n",
		 pipe->of_output_links[0] ?
		 pipe->of_output_links[0]->full_name : "none");
	DRM_INFO("	output_link[1]: %s.\n",
		 pipe->of_output_links[1] ?
		 pipe->of_output_links[1]->full_name : "none");

	for_each_set_bit(id, &avail_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);

		bst_component_dump(c);
	}
}

static void bst_component_verify_inputs(struct bst_component *c)
{
	struct bst_pipeline *pipe = c->pipeline;
	struct bst_component *input;
	int id;
	unsigned long supported_inputs = c->supported_inputs;

	for_each_set_bit(id, &supported_inputs, 32) {
		input = bst_pipeline_get_component(pipe, id);
		if (!input) {
			c->supported_inputs &= ~(BIT(id));
			DRM_WARN("Can not find input(ID-%d) for component: %s.\n",
				 id, c->name);
			continue;
		}

		input->supported_outputs |= BIT(c->id);
	}
}

static struct bst_layer *
bst_get_layer_split_right_layer(struct bst_pipeline *pipe,
				   struct bst_layer *left)
{
	int index = left->base.id - BST_DRM_COMPONENT_LAYER0;
	int i;

	for (i = index + 1; i < pipe->n_layers; i++)
		if (left->layer_type == pipe->layers[i]->layer_type)
			return pipe->layers[i];
	return NULL;
}

static void bst_pipeline_assemble(struct bst_pipeline *pipe)
{
	struct bst_component *c;
	struct bst_layer *layer;
	int i, id;
	unsigned long avail_comps = pipe->avail_comps;

	for_each_set_bit(id, &avail_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);
		bst_component_verify_inputs(c);
	}

	for (i = 0; i < pipe->n_layers; i++) {
		layer = pipe->layers[i];

		layer->right = bst_get_layer_split_right_layer(pipe, layer);
	}

	if (pipe->dual_link && !pipe->ctrlr->dual_link_support) {
		pipe->dual_link = false;
		DRM_WARN("PIPE-%d doesn't support dual-link, ignore DT dual-link configuration.\n",
			 pipe->id);
	}
}

struct bst_pipeline *
bst_pipeline_get_slave(struct bst_pipeline *master)
{
	struct bst_component *slave;

	slave = bst_component_pickup_input(&master->compiz->base,
					      BST_DRM_PIPELINE_COMPIZS);

	return slave ? slave->pipeline : NULL;
}

int bst_assemble_pipelines(struct bst_dev *mdev)
{
	struct bst_pipeline *pipe;
	int i;

	for (i = 0; i < mdev->n_pipelines; i++) {
		pipe = mdev->pipelines[i];

		bst_pipeline_assemble(pipe);
		bst_pipeline_dump(pipe);
	}

	return 0;
}

void bst_pipeline_dump_register(struct bst_pipeline *pipe,
				   struct seq_file *sf)
{
	struct bst_component *c;
	u32 id;
	unsigned long avail_comps;

	seq_printf(sf, "\n======== Pipeline-%d ==========\n", pipe->id);

	if (pipe->funcs && pipe->funcs->dump_register)
		pipe->funcs->dump_register(pipe, sf);

	avail_comps = pipe->avail_comps;
	for_each_set_bit(id, &avail_comps, 32) {
		c = bst_pipeline_get_component(pipe, id);

		seq_printf(sf, "\n------%s------\n", c->name);
		if (c->funcs->dump_register)
			c->funcs->dump_register(c, sf);
	}
}
