// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <drm/drm_print.h>

#include "bst_display_global_api.h"
#include "bst_virt_drm_device.h"
#include "bst_virt_pipeline.h"

struct bst_virt_pipe *bst_virt_pipe_add(struct bst_super_device *super_dev,
					size_t size,
					const struct bst_virt_pipe_funcs *funcs)
{
	struct bst_virt_pipe *pipe;

	if (super_dev->n_pipelines + 1 > BST_VIRT_MAX_PIPELINES) {
		DRM_ERROR("Exceed max support %d pipelines.\n",
			  BST_VIRT_MAX_PIPELINES);
		return ERR_PTR(-ENOSPC);
	}

	if (size < sizeof(*pipe)) {
		DRM_ERROR("Request pipeline size too small.\n");
		return ERR_PTR(-EINVAL);
	}

	pipe = devm_kzalloc(super_dev->dev, size, GFP_KERNEL);
	if (!pipe)
		return ERR_PTR(-ENOMEM);

	pipe->pipe_id = super_dev->n_pipelines;
	pipe->funcs = funcs;
	super_dev->pipelines[pipe->pipe_id] = pipe;
	super_dev->n_pipelines++;
	return pipe;
}

void bst_virt_pipe_destroy(struct bst_super_device *super_dev,
			   struct bst_virt_pipe *pipe)
{
	struct bst_virt_component *c;
	int i;
	unsigned long avail_comps = pipe->avail_comps;

	for_each_set_bit(i, &avail_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, i);
		bst_virt_component_destroy(super_dev, c);
	}
}

static struct bst_virt_component **
bst_virt_pipe_get_component_pos(struct bst_virt_pipe *pipe, int comp_id)
{
	struct bst_virt_component **pos = NULL;
	switch (comp_id) {
	case BST_VIRT_COMPONENT_DC_LAYER0:
	case BST_VIRT_COMPONENT_DC_LAYER1:
	case BST_VIRT_COMPONENT_DC_LAYER2:
	case BST_VIRT_COMPONENT_DC_LAYER3:
		pos = to_cpos(
			pipe->dc_layers[comp_id - BST_VIRT_COMPONENT_DC_LAYER0]);
		break;
	case BST_VIRT_COMPONENT_DC_WB_LAYER:
		pos = to_cpos(pipe->dc_wb_layer);
		break;
	case BST_VIRT_COMPONENT_DC_CRTC:
		pos = to_cpos(pipe->dc_crtc);
		break;
	case BST_VIRT_COMPONENT_CONN_eDP_VIDEO:
		pos = to_cpos(pipe->master_conn);
		break;
	case BST_VIRT_COMPONENT_CONN_DSI_VIDEO:
		pos = to_cpos(pipe->master_conn);
		break;
	case BST_VIRT_COMPONENT_CONN_LVDS_VIDEO:
		pos = to_cpos(pipe->master_conn);
		break;
	default:
		pos = NULL;
		DRM_ERROR("Unknown pipeline resource ID: %d.\n", comp_id);
		break;
	}

	return pos;
}

struct bst_virt_component *
bst_virt_pipe_get_component(struct bst_virt_pipe *pipe, int comp_id)
{
	struct bst_virt_component **pos = NULL;
	struct bst_virt_component *c = NULL;

	pos = bst_virt_pipe_get_component_pos(pipe, comp_id);
	if (pos)
		c = *pos;

	return c;
}

struct bst_virt_component *
bst_virt_pipe_get_first_component(struct bst_virt_pipe *pipe, u32 comp_mask)
{
	struct bst_virt_component *c = NULL;
	unsigned long comp_mask_local = (unsigned long)comp_mask;
	int id;

	id = find_first_bit(&comp_mask_local, 32);
	if (id < 32)
		c = bst_virt_pipe_get_component(pipe, id);

	return c;
}

struct bst_virt_component *
bst_virt_component_add(struct bst_virt_pipe *pipe,
		       struct bst_virt_device *subdev, size_t comp_sz, u32 id,
		       u32 fw_id, const struct bst_virt_component_funcs *funcs,
		       u8 max_active_inputs, u32 supported_inputs,
		       u8 max_active_outputs, const char *name_fmt, ...)
{
	struct bst_virt_component **pos;
	struct bst_virt_component *c;
	int idx, *layer_num = NULL;
	if (max_active_inputs > BST_VIRT_COMPONENT_N_INPUTS) {
		DRM_WARN("please large BST_VIRT_COMPONENT_N_INPUTS to %d.\n",
		     max_active_inputs);
		return ERR_PTR(-ENOSPC);
	}
	if (!pipe) {
		DRM_ERROR("pipe is null,id:%d.\n", id);
		return ERR_PTR(-EINVAL);
	}
	pos = bst_virt_pipe_get_component_pos(pipe, id);
	if (!pos || (*pos)) {
		DRM_ERROR("wrong id(%d) to get component pos.\n", id);
		return ERR_PTR(-EINVAL);
	}

	if (has_bit(id, BST_VIRT_PIPE_LAYERS)) {
		idx = id - BST_VIRT_COMPONENT_DC_LAYER0;
		layer_num = &pipe->n_dc_layers;
		if (idx != pipe->n_dc_layers) {
			DRM_ERROR(
				"please add Layer by id idx(%d) != n_layers(%d)) sequence.\n",
				idx, pipe->n_dc_layers);
			return ERR_PTR(-EINVAL);
		}
	}

	c = devm_kzalloc(subdev->dev, comp_sz, GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->id = id;
	c->fw_id = fw_id;
	c->subdev_session = subdev->subdev_session;
	c->pipe = pipe;
	c->max_active_inputs = max_active_inputs;
	c->max_active_outputs = max_active_outputs;
	c->supported_inputs = supported_inputs;
	c->funcs = funcs;
	c->base_dev = subdev;

	if (name_fmt) {
		va_list args;

		va_start(args, name_fmt);
		vsnprintf(c->name, sizeof(c->name), name_fmt, args);
		va_end(args);
	}

	if (layer_num)
		*layer_num = *layer_num + 1;

	pipe->avail_comps |= BIT(c->id);

	*pos = c;

	return c;
}

void bst_virt_component_destroy(struct bst_super_device *super_dev,
				struct bst_virt_component *c)
{
}

static void bst_virt_component_dump(struct bst_virt_component *c)
{
	if (!c)
		return;

	DRM_DEBUG("	%s: ID %d-0x%08lx.\n", c->name, c->id, BIT(c->id));
	DRM_DEBUG(
		"		max_active_inputs:%d, supported_inputs: 0x%08x.\n",
		c->max_active_inputs, c->supported_inputs);
	DRM_DEBUG(
		"		max_active_outputs:%d, supported_outputs: 0x%08x.\n",
		c->max_active_outputs, c->supported_outputs);
}


static void bst_virt_pipe_dump(struct bst_virt_pipe *pipe)
{
	struct bst_virt_component *c;
	int id;
	unsigned long avail_comps = pipe->avail_comps;
	const char *link0_comp_name = NULL;
	const char *link1_comp_name = NULL;


	if (pipe->of_output_links[0])
		of_property_read_string_index(pipe->of_output_links[0],
								"compatible", 0, &link0_comp_name);
	if (pipe->of_output_links[1])
		of_property_read_string_index(pipe->of_output_links[1],
								"compatible", 0, &link1_comp_name);

	DRM_INFO("Pipeline-%d: n_dc_layers: %d, output: %s.\n", pipe->pipe_id,
		 pipe->n_dc_layers,
		 pipe->dual_link ? "dual-link" : "single-link");
	DRM_INFO("	output_link[0]: [%s]-[%s].\n",
		 pipe->of_output_links[0] ?
			 pipe->of_output_links[0]->full_name :
			 "none",
			 link0_comp_name ? link0_comp_name: "none");
	DRM_INFO("	output_link[1]: [%s]-[%s].\n",
		 pipe->of_output_links[1] ?
			 pipe->of_output_links[1]->full_name :
			 "none",
			 link1_comp_name ? link1_comp_name: "none");

	for_each_set_bit(id, &avail_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, id);

		bst_virt_component_dump(c);
	}
}

static void bst_virt_component_verify_inputs(struct bst_virt_component *c)
{
	struct bst_virt_pipe *pipe = c->pipe;
	struct bst_virt_component *input;
	int id;
	unsigned long supported_inputs = c->supported_inputs;

	for_each_set_bit(id, &supported_inputs, 32) {
		input = bst_virt_pipe_get_component(pipe, id);
		if (!input) {
			c->supported_inputs &= ~(BIT(id));
			DRM_WARN(
				"Can not find input(ID-%d) for component: %s.\n",
				id, c->name);
			continue;
		}

		input->supported_outputs |= BIT(c->id);
	}
}

static struct bst_virt_layer *
bst_get_layer_split_right_layer(struct bst_virt_pipe *pipe,
				struct bst_virt_layer *left)
{
	int index = left->base.id - BST_VIRT_COMPONENT_DC_LAYER0;
	int i;

	for (i = index + 1; i < pipe->n_dc_layers; i++)
		if (left->layer_type == pipe->dc_layers[i]->layer_type)
			return pipe->dc_layers[i];
	return NULL;
}

static void bst_virt_pipe_assemble(struct bst_virt_pipe *pipe)
{
	struct bst_virt_component *c;
	struct bst_virt_layer *layer;
	int i, id;
	unsigned long avail_comps = pipe->avail_comps;

	for_each_set_bit(id, &avail_comps, 32) {
		c = bst_virt_pipe_get_component(pipe, id);
		bst_virt_component_verify_inputs(c);
	}

	for (i = 0; i < pipe->n_dc_layers; i++) {
		layer = pipe->dc_layers[i];
		layer->right = bst_get_layer_split_right_layer(pipe, layer);
	}

	if (pipe->dual_link) {
		if (!pipe->dc_crtc->supports_dual_link) {
			pipe->dual_link = false;
			DRM_WARN(
				"PIPE-%d doesn't support dual-link, ignore DT dual-link configuration.\n",
				pipe->pipe_id);
		}
	}
}

int bst_virt_assemble_pipe(struct bst_super_device *super_dev)
{
	struct bst_virt_pipe *pipe;
	struct bst_virt_device *dc_dev;
	struct bst_virt_device *conn_dev;
	int i, ret = 0;
	struct bst_display_topology_info topo_info = {0};
	struct bst_display_topology_status topo_status = {0};
	if(!super_dev){
		DRM_ERROR("super_dev is null \n");
		return -1;
	}
	for (i = 0; i < super_dev->n_pipelines; i++) {
		dc_dev = super_dev->subdevs[i][BST_VIRT_DC_IDX];
		if (dc_dev) {
			conn_dev = super_dev->subdevs[i][BST_VIRT_CONN_IDX];
			topo_info.dc_subdev_session = dc_dev->dev_info.subdev_session;
			if(!conn_dev){
				DRM_ERROR("conn_dev is null \n");
				return -1;
			}
			topo_info.conn_subdev_session = conn_dev->dev_info.subdev_session;
			memset(&topo_status, 0, sizeof(topo_status));
			ret = bst_display_glb_cmd_is_valid_topology(&topo_info, &topo_status);
			if (!ret && topo_status.base.status == DISP_COMM_REPLAY_OK) {
				pipe = dc_dev->this_pipe;
				bst_virt_pipe_assemble(pipe);
				bst_virt_pipe_dump(pipe);
			} else {
				return -1;
			}
		}
	}

	return 0;
}
