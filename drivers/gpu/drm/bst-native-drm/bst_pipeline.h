/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef _BST_DRM_PIPELINE_H_
#define _BST_DRM_PIPELINE_H_

#include <linux/types.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include "bst_utils.h"
#include "bst_format_color.h"

#define BST_DRM_MAX_PIPELINES		2
#define BST_DRM_PIPELINE_MAX_LAYERS	4
#define BST_DRM_PIPELINE_MAX_SCALERS	2
#define BST_DRM_COMPONENT_N_INPUTS	5

enum {
	BST_DRM_COMPONENT_LAYER0		= 0,
	BST_DRM_COMPONENT_LAYER1		= 1,
	BST_DRM_COMPONENT_LAYER2		= 2,
	BST_DRM_COMPONENT_LAYER3		= 3,
	BST_DRM_COMPONENT_WB_LAYER	= 7, /* write back layer */
	BST_DRM_COMPONENT_SCALER0	= 8,
	BST_DRM_COMPONENT_SCALER1	= 9,
	BST_DRM_COMPONENT_SPLITTER	= 12,
	BST_DRM_COMPONENT_MERGER		= 14,
	BST_DRM_COMPONENT_COMPIZ0	= 16, /* compositor */
	BST_DRM_COMPONENT_COMPIZ1	= 17,
	BST_DRM_COMPONENT_IPS0		= 20, /* post image processor */
	BST_DRM_COMPONENT_IPS1		= 21,
	BST_DRM_COMPONENT_TIMING_CTRLR	= 22, /* timing controller */
};

#define BST_DRM_PIPELINE_LAYERS		(BIT(BST_DRM_COMPONENT_LAYER0) |\
					 BIT(BST_DRM_COMPONENT_LAYER1) |\
					 BIT(BST_DRM_COMPONENT_LAYER2) |\
					 BIT(BST_DRM_COMPONENT_LAYER3))

#define BST_DRM_PIPELINE_SCALERS		(BIT(BST_DRM_COMPONENT_SCALER0) |\
					 BIT(BST_DRM_COMPONENT_SCALER1))

#define BST_DRM_PIPELINE_COMPIZS		(BIT(BST_DRM_COMPONENT_COMPIZ0) |\
					 BIT(BST_DRM_COMPONENT_COMPIZ1))

#define BST_DRM_PIPELINE_IMPROCS		(BIT(BST_DRM_COMPONENT_IPS0) |\
					 BIT(BST_DRM_COMPONENT_IPS1))
struct bst_component;
struct bst_component_state;


struct bst_component_funcs {
	int (*validate)(struct bst_component *c,
			struct bst_component_state *state);
	void (*update)(struct bst_component *c,
		       struct bst_component_state *state);
	void (*disable)(struct bst_component *c);
	void (*dump_register)(struct bst_component *c, struct seq_file *seq);
};

struct bst_component {
	struct drm_private_obj obj;
	struct bst_pipeline *pipeline;
	char name[32];
	u32 __iomem *reg;
	u32 id;
	u32 hw_id;
	u8 max_active_inputs;
	u8 max_active_outputs;
	u32 supported_inputs;
	u32 supported_outputs;
	const struct bst_component_funcs *funcs;
};

struct bst_component_output {
	struct bst_component *component;
	u8 output_port;
};

struct bst_component_state {
	struct drm_private_state obj;
	struct bst_component *component;
	union {
		struct drm_crtc *crtc;
		struct drm_plane *plane;
		struct drm_connector *wb_conn;
		void *binding_user;
	};

	u16 active_inputs;
	u16 changed_active_inputs;
	u16 affected_inputs;
	struct bst_component_output inputs[BST_DRM_COMPONENT_N_INPUTS];
};

static inline u16 component_disabling_inputs(struct bst_component_state *st)
{
	return st->affected_inputs ^ st->active_inputs;
}

static inline u16 component_changed_inputs(struct bst_component_state *st)
{
	return component_disabling_inputs(st) | st->changed_active_inputs;
}

#define for_each_changed_input(st, i)	\
	for ((i) = 0; (i) < (st)->component->max_active_inputs; (i)++)	\
		if (has_bit((i), component_changed_inputs(st)))

#define to_comp(__c)	(((__c) == NULL) ? NULL : &((__c)->base))
#define to_cpos(__c)	((struct bst_component **)&(__c))

struct bst_layer {
	struct bst_component base;
	struct bstdc_range hsize_in, vsize_in;
	u32 layer_type;
	u32 line_sz;
	u32 yuv_line_sz;
	u32 supported_rots;
	struct bst_layer *right;
};

struct bst_layer_state {
	struct bst_component_state base;
	u16 hsize, vsize;
	u32 rot;
	u16 afbc_crop_l;
	u16 afbc_crop_r;
	u16 afbc_crop_t;
	u16 afbc_crop_b;
	dma_addr_t addr[3];
};

struct bst_scaler {
	struct bst_component base;
	struct bstdc_range hsize, vsize;
	u32 max_upscaling;
	u32 max_downscaling;
	u8 scaling_split_overlap;
	u8 enh_split_overlap;
};

struct bst_scaler_state {
	struct bst_component_state base;
	u16 hsize_in, vsize_in;
	u16 hsize_out, vsize_out;
	u16 total_hsize_in, total_vsize_in;
	u16 total_hsize_out;
	u16 left_crop, right_crop;
	u8 en_scaling : 1,
	   en_alpha : 1,
	   en_img_enhancement : 1,
	   en_split : 1,
	   right_part : 1;
};

struct bst_compiz {
	struct bst_component base;
	struct bstdc_range hsize, vsize;
};

struct bst_compiz_input_cfg {
	u16 hsize, vsize;
	u16 hoffset, voffset;
	u8 pixel_blend_mode, layer_alpha;
};

struct bst_compiz_state {
	struct bst_component_state base;
	u16 hsize, vsize;
	struct bst_compiz_input_cfg cins[BST_DRM_COMPONENT_N_INPUTS];
};

struct bst_merger {
	struct bst_component base;
	struct bstdc_range hsize_merged;
	struct bstdc_range vsize_merged;
};

struct bst_merger_state {
	struct bst_component_state base;
	u16 hsize_merged;
	u16 vsize_merged;
};

struct bst_splitter {
	struct bst_component base;
	struct bstdc_range hsize, vsize;
};

struct bst_splitter_state {
	struct bst_component_state base;
	u16 hsize, vsize;
	u16 overlap;
};

struct bst_improc {
	struct bst_component base;
	u32 supported_color_formats;
	u32 supported_color_depths;
	u8 supports_degamma : 1;
	u8 supports_csc : 1;
	u8 supports_gamma : 1;
};

struct bst_improc_state {
	struct bst_component_state base;
	u8 color_format, color_depth;
	u16 hsize, vsize;
	u32 fgamma_coeffs[BST_DRM_N_GAMMA_COEFFS];
	u32 ctm_coeffs[BST_DRM_N_CTM_COEFFS];
};

struct bst_timing_ctrlr {
	struct bst_component base;
	u8 dual_link_support : 1;
};

struct bst_timing_ctrlr_state {
	struct bst_component_state base;
};

struct bst_data_flow_cfg {
	struct bst_component_output input;
	u16 in_x, in_y, in_w, in_h;
	u32 out_x, out_y, out_w, out_h;
	u16 total_in_h, total_in_w;
	u16 total_out_w;
	u16 left_crop, right_crop, overlap;
	u32 rot;
	int blending_zorder;
	u8 pixel_blend_mode, layer_alpha;
	u8 en_scaling : 1,
	   en_img_enhancement : 1,
	   en_split : 1,
	   is_yuv : 1,
	   right_part : 1;
};

struct bst_pipeline_funcs {
	int (*downscaling_clk_check)(struct bst_pipeline *pipe,
				     struct drm_display_mode *mode,
				     unsigned long aclk_rate,
				     struct bst_data_flow_cfg *dflow);
	void (*dump_register)(struct bst_pipeline *pipe,
			      struct seq_file *sf);
};

struct bst_pipeline {
	struct drm_private_obj obj;
	struct bst_dev *mdev;
	struct clk *gate_pxlclk;
	struct clk *pll_clk;
	struct clk *div_clk;
	struct clk *gate_aclk;
	struct clk *gate_pclk;
	struct clk *mux_aclk;

	int pxlclk_chan;
	int id;
	u32 avail_comps;
	u32 standalone_disabled_comps;
	int n_layers;
	struct bst_layer *layers[BST_DRM_PIPELINE_MAX_LAYERS];
	int n_scalers;
	struct bst_scaler *scalers[BST_DRM_PIPELINE_MAX_SCALERS];
	struct bst_compiz *compiz;
	struct bst_splitter *splitter;
	struct bst_merger *merger;
	struct bst_layer  *wb_layer;
	struct bst_improc *improc;
	struct bst_timing_ctrlr *ctrlr;
	const struct bst_pipeline_funcs *funcs;
	struct device_node *of_node;
	struct device_node *of_output_port;
	struct device_node *of_output_links[2];
	bool dual_link;
};

struct bst_pipeline_state {
	struct drm_private_state obj;
	struct bst_pipeline *pipe;
	struct drm_crtc *crtc;
	u32 active_comps;
};

#define to_layer(c)	container_of(c, struct bst_layer, base)
#define to_compiz(c)	container_of(c, struct bst_compiz, base)
#define to_scaler(c)	container_of(c, struct bst_scaler, base)
#define to_splitter(c)	container_of(c, struct bst_splitter, base)
#define to_merger(c)	container_of(c, struct bst_merger, base)
#define to_improc(c)	container_of(c, struct bst_improc, base)
#define to_ctrlr(c)	container_of(c, struct bst_timing_ctrlr, base)

#define to_layer_st(c)	container_of(c, struct bst_layer_state, base)
#define to_compiz_st(c)	container_of(c, struct bst_compiz_state, base)
#define to_scaler_st(c)	container_of(c, struct bst_scaler_state, base)
#define to_splitter_st(c) container_of(c, struct bst_splitter_state, base)
#define to_merger_st(c)	container_of(c, struct bst_merger_state, base)
#define to_improc_st(c)	container_of(c, struct bst_improc_state, base)
#define to_ctrlr_st(c)	container_of(c, struct bst_timing_ctrlr_state, base)

#define priv_to_comp_st(o) container_of(o, struct bst_component_state, obj)
#define priv_to_pipe_st(o) container_of(o, struct bst_pipeline_state, obj)

struct bst_pipeline *
bst_pipeline_add(struct bst_dev *mdev, size_t size,
		    const struct bst_pipeline_funcs *funcs);
void bst_pipeline_destroy(struct bst_dev *mdev,
			     struct bst_pipeline *pipe);
struct bst_pipeline *
bst_pipeline_get_slave(struct bst_pipeline *master);
int bst_assemble_pipelines(struct bst_dev *mdev);
struct bst_component *
bst_pipeline_get_component(struct bst_pipeline *pipe, int id);
struct bst_component *
bst_pipeline_get_first_component(struct bst_pipeline *pipe,
				    u32 comp_mask);

void bst_pipeline_dump_register(struct bst_pipeline *pipe,
				   struct seq_file *sf);

extern __printf(10, 11)
struct bst_component *
bst_component_add(struct bst_pipeline *pipe,
		     size_t comp_sz, u32 id, u32 hw_id,
		     const struct bst_component_funcs *funcs,
		     u8 max_active_inputs, u32 supported_inputs,
		     u8 max_active_outputs, u32 __iomem *reg,
		     const char *name_fmt, ...);

void bst_component_destroy(struct bst_dev *mdev,
			      struct bst_component *c);

static inline struct bst_component *
bst_component_pickup_output(struct bst_component *c, u32 avail_comps)
{
	u32 avail_inputs = c->supported_outputs & (avail_comps);

	return bst_pipeline_get_first_component(c->pipeline, avail_inputs);
}

struct bst_plane_state;
struct bst_crtc_state;
struct bst_crtc;

void pipeline_composition_size(struct bst_crtc_state *bcrtc_st,
			       u16 *hsize, u16 *vsize);

int bst_build_layer_data_flow(struct bst_layer *layer,
				 struct bst_plane_state *bplane_st,
				 struct bst_crtc_state *bcrtc_st,
				 struct bst_data_flow_cfg *dflow);
int bst_build_wb_data_flow(struct bst_layer *wb_layer,
			      struct drm_connector_state *conn_st,
			      struct bst_crtc_state *bcrtc_st,
			      struct bst_data_flow_cfg *dflow);
int bst_build_display_data_flow(struct bst_crtc *bcrtc,
				   struct bst_crtc_state *bcrtc_st);

int bst_build_layer_split_data_flow(struct bst_layer *left,
				       struct bst_plane_state *bplane_st,
				       struct bst_crtc_state *bcrtc_st,
				       struct bst_data_flow_cfg *dflow);
int bst_build_wb_split_data_flow(struct bst_layer *wb_layer,
				    struct drm_connector_state *conn_st,
				    struct bst_crtc_state *bcrtc_st,
				    struct bst_data_flow_cfg *dflow);

int bst_release_unclaimed_resources(struct bst_pipeline *pipe,
				       struct bst_crtc_state *bcrtc_st);

struct bst_pipeline_state *
bst_pipeline_get_old_state(struct bst_pipeline *pipe,
			      struct drm_atomic_state *state);
bool bst_pipeline_disable(struct bst_pipeline *pipe,
			     struct drm_atomic_state *old_state);
void bst_pipeline_update(struct bst_pipeline *pipe,
			    struct drm_atomic_state *old_state);

void bst_complete_data_flow_cfg(struct bst_layer *layer,
				   struct bst_data_flow_cfg *dflow,
				   struct drm_framebuffer *fb);

#endif /* _BST_DRM_PIPELINE_H_*/
