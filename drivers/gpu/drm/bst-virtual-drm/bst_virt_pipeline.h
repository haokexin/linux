// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_VIRT_DRM_PIPELINE_H_
#define _BST_VIRT_DRM_PIPELINE_H_

#include <linux/types.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include "bst_virt_utils.h"
#include "bst_virt_format_color.h"
#include "bst_display_conn_cmdset.h"
#include "bst_display_global_api.h"

struct bst_virt_component;
struct bst_virt_component_state;
struct bst_super_device;

#define BST_VIRT_DC_IDX               0
#define BST_VIRT_CONN_IDX             1
#define BST_VIRT_MAX_SUBDEV_OF_1PIPE  2
#define BST_VIRT_MAX_PIPELINES        5
#define BST_VIRT_PIPELINE_MAX_LAYERS  4
#define BST_VIRT_PIPELINE_MAX_SCALERS 2
#define BST_VIRT_COMPONENT_N_INPUTS   5

enum {
	BST_VIRT_COMPONENT_DC_LAYER0 = 0, /* display controllor components */
	BST_VIRT_COMPONENT_DC_LAYER1,
	BST_VIRT_COMPONENT_DC_LAYER2,
	BST_VIRT_COMPONENT_DC_LAYER3,
	BST_VIRT_COMPONENT_DC_WB_LAYER,
	BST_VIRT_COMPONENT_DC_CRTC,
	BST_VIRT_COMPONENT_CONN_eDP_VIDEO, /* display connector components*/
	BST_VIRT_COMPONENT_CONN_eDP_AUDIO,
	BST_VIRT_COMPONENT_CONN_DSI_VIDEO,
	BST_VIRT_COMPONENT_CONN_LVDS_VIDEO,
};

#define BST_VIRT_PIPE_LAYERS                 \
	(BIT(BST_VIRT_COMPONENT_DC_LAYER0) | \
	 BIT(BST_VIRT_COMPONENT_DC_LAYER1) | \
	 BIT(BST_VIRT_COMPONENT_DC_LAYER2) | \
	 BIT(BST_VIRT_COMPONENT_DC_LAYER3))

#define BST_VIRT_DC_SCALER_CH0    (0)
#define BST_VIRT_DC_SCALER_CH1    (1)
#define BST_VIRT_DC_SCALER_NULL   (0xFF)

#define to_layer(c) container_of(c, struct bst_virt_layer, base)
#define to_dc_crtc(c) container_of(c, struct bst_virt_dc_crtc, base)

#define to_layer_st(c) container_of(c, struct bst_virt_layer_state, base)
#define to_dc_crtc_st(c) container_of(c, struct bst_virt_dc_crtc_state, base)

#define priv_to_comp_st(o) container_of(o, struct bst_virt_component_state, obj)
#define priv_to_pipe_st(o) container_of(o, struct bst_virt_pipe_state, obj)


struct bst_virt_component_funcs {
	int (*validate)(struct bst_virt_component *c,
			struct bst_virt_component_state *state);
	void (*update)(struct bst_virt_component *c,
		       struct bst_virt_component_state *state);
	void (*enable)(struct bst_virt_component *c);
	void (*disable)(struct bst_virt_component *c);
	int (*detect)(struct bst_virt_component *c);
	int (*get_modes)(struct bst_virt_component *c);
	void (*dump_log)(struct bst_virt_component *c, struct seq_file *seq);
};

struct bst_virt_component {
	struct drm_private_obj obj;
	struct bst_virt_pipe *pipe;
	char name[32];
	u32 subdev_session;
	u32 id;
	u32 fw_id;
	u8 max_active_inputs;
	u8 max_active_outputs;
	u32 supported_inputs;
	u32 supported_outputs;
	void *base_dev;
	const struct bst_virt_component_funcs *funcs;
};

struct bst_virt_pipe {
	struct drm_private_obj obj;
	int n_subdevs;
	struct bst_virt_device *subdevs[BST_VIRT_MAX_SUBDEV_OF_1PIPE];
	int n_dc_layers;
	struct bst_virt_layer *dc_layers[BST_VIRT_PIPELINE_MAX_LAYERS];
	struct bst_virt_layer *dc_wb_layer;
	struct bst_virt_dc_crtc *dc_crtc;
	struct bst_virt_connector *master_conn;
	const struct bst_virt_pipe_funcs *funcs;
	struct device_node *of_node;
	struct device_node *of_output_port;
	struct device_node *of_output_links[2];
	int pipe_id;
	bool dual_link;
	u32 avail_comps;
	u32 old_changed_comps;
	u32 standalone_disabled_comps;
#ifdef DISPLAY_SUPPORT_SCALE
	u8 scaler_num;
	//struct bst_virt_layer* scaler_slot[BST_VIRT_PIPELINE_MAX_SCALERS];
#endif
};

struct bst_virt_pipe_funcs {
	void (*dump_log)(struct bst_virt_pipe *pipe, struct seq_file *sf);
};

struct bst_virt_pipe_state {
	struct drm_private_state obj;
	struct bst_virt_pipe *pipe;
	struct drm_crtc *crtc;
	u32 active_comps;
};

#define to_comp(__c) (((__c) == NULL) ? NULL : &((__c)->base))
#define to_cpos(__c) ((struct bst_virt_component **)&(__c))

struct bst_virt_component_output {
	struct bst_virt_component *component;
	u8 output_port;
};

struct bst_virt_component_state {
	struct drm_private_state obj;
	struct bst_virt_component *component;
	union {
		struct drm_crtc *crtc;
		struct drm_plane *plane;
		struct drm_connector *wb_conn;
		void *binding_user;
	};
	u16 active_inputs;
	u16 changed_active_inputs;
	u16 affected_inputs;
	struct bst_virt_component_output inputs[BST_VIRT_COMPONENT_N_INPUTS];
};

struct bst_data_flow_cfg {
	struct bst_virt_component_output input;
	u16 in_x, in_y, in_w, in_h;
	u32 out_x, out_y, out_w, out_h;
	u16 total_in_h, total_in_w;
	u16 total_out_w;
	u16 left_crop, right_crop, overlap;
	u32 rot;
	int blending_zorder;
	u8 pixel_blend_mode, layer_alpha;
	u8 en_scaling : 1, en_img_enhancement : 1, en_split : 1, is_yuv : 1;
};

struct bst_virt_dc_crtc {
	struct bst_virt_component base;
	u32 supported_color_formats;
	u32 supported_color_depths;
	u8 supports_degamma : 1;
	u8 supports_csc : 1;
	u8 supports_gamma : 1;
	u8 supports_dual_link : 1;
	struct bstdc_range hsize, vsize;
};

struct bst_virt_crtc_input_cfg {
	u16 hsize, vsize;
	u16 hoffset, voffset;
	u8 pixel_blend_mode, layer_alpha;
};

struct bst_virt_dc_crtc_state {
	struct bst_virt_component_state base;
	u8 color_format, color_depth;
	u16 hsize, vsize;
	u32 fgamma_coeffs[BST_DRM_N_GAMMA_COEFFS];
	u32 ctm_coeffs[BST_DRM_N_CTM_COEFFS];
	u8 valid_input_ids[5];
};

struct bst_virt_layer {
	struct bst_virt_component base;
	struct bstdc_range hsize_in, vsize_in;
	u32 layer_type;
	u32 line_sz;
	u32 yuv_line_sz;
	u32 supported_rots;
	u32 supported_pix_fmt_stds;
	u32 supported_ctm_lut_stds;
	u32 supported_scale;
	u32 init_zpos;
	u32 max_upscaling;
	u32 max_downscaling;
	struct bst_virt_layer *right;
	struct bstdc_range scaler_hsize, scaler_vsize;
};

struct bst_scaler_cfg {
	u16 hsize_in, vsize_in;
	u16 hsize_out, vsize_out;
	u16 total_hsize_in, total_vsize_in;
	u16 total_hsize_out;
	u16 left_crop, right_crop;
	u8 en_scaling : 1,
		en_alpha : 1,
		en_img_enhancement : 1;
};

struct bst_afbc_crop_cfg {
	u16 afbc_crop_l;
	u16 afbc_crop_r;
	u16 afbc_crop_t;
	u16 afbc_crop_b;
	u16 crop_type;
};

struct bst_virt_layer_state {
	struct bst_virt_component_state base;
	u16 hsize, vsize;
	u32 rot;
	struct bst_afbc_crop_cfg afbc_crop_old;
	struct bst_afbc_crop_cfg afbc_crop;
	dma_addr_t addr[3];
	//struct bst_scaler_cfg scaler_old;
	struct bst_scaler_cfg scaler;
	struct bst_virt_crtc_input_cfg cin;
};

struct bst_virt_connector {
	struct bst_virt_component base;
	struct bst_connector *bconn;
	struct video_timing cur_timing;
	u8 edid[DEFAULT_EDID_BUFLEN * MAX_EDID_BUF_NUM];
	uint32_t supported_color_formats;
	uint32_t supported_color_depths;
	u8 video_format;
	u8 bpc;
	atomic_t connected;
	uint32_t rate;
	u8 lanes;
};

struct bst_virt_connector_state {
	struct bst_virt_component_state base;
};

struct bst_plane_state;
struct bst_crtc_state;
struct bst_crtc;

struct bst_virt_component *
bst_virt_component_add(struct bst_virt_pipe *pipe,
		       struct bst_virt_device *subdev, size_t comp_sz, u32 id,
		       u32 fw_id, const struct bst_virt_component_funcs *funcs,
		       u8 max_active_inputs, u32 supported_inputs,
		       u8 max_active_outputs, const char *name_fmt, ...);

struct bst_virt_component *
bst_virt_pipe_get_component(struct bst_virt_pipe *pipe, int id);
void bst_virt_pipe_destroy(struct bst_super_device *super_dev,
			   struct bst_virt_pipe *pipe);
void bst_virt_component_destroy(struct bst_super_device *super_dev,
				struct bst_virt_component *c);
struct bst_virt_pipe *
bst_virt_pipe_add(struct bst_super_device *super_dev, size_t size,
		  const struct bst_virt_pipe_funcs *funcs);
int bst_virt_assemble_pipe(struct bst_super_device *super_dev);
int bst_complete_data_flow_cfg(struct bst_virt_layer *layer,
				struct bst_data_flow_cfg *dflow,
				struct drm_framebuffer *fb);
void remove_scaler_from_slot(struct bst_virt_layer *layer);
int bst_build_layer_data_flow(struct bst_virt_layer *layer,
			      struct bst_plane_state *bplane_st,
			      struct bst_crtc_state *bcrtc_st,
			      struct bst_data_flow_cfg *dflow);
int bst_build_wb_data_flow(struct bst_virt_layer *wb_layer,
			   struct drm_connector_state *conn_st,
			   struct bst_crtc_state *bcrtc_st,
			   struct bst_data_flow_cfg *dflow);
int bst_build_display_data_flow(struct bst_crtc *bcrtc,
				struct bst_crtc_state *bcrtc_st);
int bst_release_unclaimed_resources(struct bst_virt_pipe *pipe,
				    struct bst_crtc_state *bcrtc_st);
void bst_virt_pipe_update(struct bst_virt_pipe *pipe,
			  struct drm_atomic_state *old_state);
bool bst_virt_pipe_disable(struct bst_virt_pipe *pipe,
			   struct drm_atomic_state *old_state);
void bst_virt_pipe_dc_crtc_size(struct bst_crtc_state *bcrtc_st, u16 *hsize,
			       u16 *vsize);

#endif /* _BST_VIRT_DRM_PIPELINE_H_*/
