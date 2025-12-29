// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef _BST_VIRT_DRM_KMS_H_
#define _BST_VIRT_DRM_KMS_H_

#include <linux/list.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_writeback.h>
#include <drm/drm_bridge.h>
#include <drm/drm_print.h>
#include <drm/display/drm_dp_helper.h>
#include "bst_virt_pipeline.h"

#define BST_DRM_MAX_PIPES 5

struct bst_virt_device;

struct bst_plane {
	struct drm_plane base;
	struct bst_virt_layer *layer;
	struct bst_virt_device *this_subdev;
};

struct bst_plane_state {
	struct drm_plane_state base;
	struct list_head zlist_node;
	u8 layer_split : 1;
};

struct bst_wb_connector {
	struct drm_writeback_connector base;
	struct bst_virt_layer *wb_layer;
};

struct bst_connector {
	struct drm_connector base;
	struct drm_encoder encoder;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct bst_virt_connector *virt_conn;
	struct backlight_device *bd;
};

struct bst_crtc {
	struct drm_crtc base;
	struct bst_virt_pipe *master;
	struct bst_connector *master_conn;
	struct bst_wb_connector *wb_conn;
	struct completion *disable_done;
	struct completion *force_wb_comp;
	int force_wb_flag;
	int force_wb_index;
};

struct bst_crtc_state {
	struct drm_crtc_state base;
	u32 affected_pipes;
	u32 active_pipes;
	u64 clock_ratio;
	u8 en_scaling;
};

struct bst_kms_dev {
	struct drm_device base;
	int n_crtcs;
	struct bst_crtc crtcs[BST_VIRT_MAX_PIPELINES];
};

static inline bool
is_only_changed_connector(struct drm_crtc_state *st, struct drm_connector *conn)
{
	struct drm_crtc_state *old_st;
	u32 changed_connectors;

	old_st = drm_atomic_get_old_crtc_state(st->state, st->crtc);
	changed_connectors = st->connector_mask ^ old_st->connector_mask;

	return BIT(drm_connector_index(conn)) == changed_connectors;
}

#define to_bplane(p) container_of(p, struct bst_plane, base)
#define to_bplane_st(p) container_of(p, struct bst_plane_state, base)
#define to_bwconn(p) container_of(p, struct bst_wb_connector, base)
#define to_bcrtc(p) container_of(p, struct bst_crtc, base)
#define to_bcrtc_st(p) container_of(p, struct bst_crtc_state, base)
#define to_wb_conn(x) container_of(x, struct drm_writeback_connector, base)
#define to_bconn(x) container_of(x, struct bst_connector, base)
#define to_kms_dev(p)	container_of(p, struct bst_kms_dev, base)

unsigned long bst_crtc_get_aclk(struct bst_crtc_state *bcrtc_st);
void bst_crtc_get_color_config(struct drm_crtc_state *crtc_st,
			       u32 *color_depths, u32 *color_formats);
int bst_kms_setup_crtcs(struct bst_kms_dev *kms,
			struct bst_super_device *super_dev);
int bst_kms_add_crtcs(struct bst_kms_dev *kms,
		      struct bst_super_device *super_dev);
int bst_kms_add_planes(struct bst_kms_dev *kms,
		       struct bst_super_device *super_dev);
int bst_kms_add_private_objs(struct bst_kms_dev *kms,
				struct bst_super_device *super_dev);
int bst_virt_connectors_add(struct bst_kms_dev *kms,
				struct bst_super_device *super_dev);
int bst_kms_add_wb_connectors(struct bst_kms_dev *kms);
void bst_kms_cleanup_private_objs(struct bst_kms_dev *kms);
struct bst_kms_dev *bst_kms_attach(struct bst_super_device *super_dev);
void bst_kms_detach(struct bst_kms_dev *kms);
void bst_crtc_handle_event(struct bst_crtc *bcrtc, struct bst_virt_events *evts);
void bst_crtc_wait_for_hw_flip_done(struct bst_crtc *bcrtc, struct completion *input_flip_done);
void bst_crtc_hw_flush(struct bst_crtc *bcrtc);

static inline u32 drm_dp_rate_from_firmware(u8 rate)
{
	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		rate = DP_LINK_BW_1_62;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		rate = DP_LINK_BW_2_7;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		rate = DP_LINK_BW_5_4;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		rate = DP_LINK_BW_8_1;
		break;
	}
	return drm_dp_bw_code_to_link_rate(rate);
}


static inline void drm_display_info_form_fw(struct drm_display_info *disp_info,
					    struct screen_state *fw_screen_info,
					    struct bst_virt_connector *v_conn)
{
	disp_info->width_mm = fw_screen_info->hszie_mm;
	disp_info->height_mm = fw_screen_info->vszie_mm;
	disp_info->bpc = fw_screen_info->pix_bpc;
	disp_info->color_formats = fw_screen_info->pix_format;
	switch (v_conn->video_format) {
	case RGB:
		disp_info->color_formats = DRM_COLOR_FORMAT_RGB444;
		break;
	case YCBCR420:
		disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR420;
		break;
	case YCBCR422:
		disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR422;
		break;
	case YCBCR444:
		disp_info->color_formats = DRM_COLOR_FORMAT_YCBCR444;
		break;
	case YONLY:
	case RAW:
		DRM_WARN_ONCE("not support YONLY/RAW yet\n");
		break;
	default:
		DRM_WARN_ONCE("invalid pixel_encode value\n");
	}
	v_conn->cur_timing.video_timing_id = fw_screen_info->timing_id;
}
#endif /*_BST_VIRT_DRM_KMS_H_*/
