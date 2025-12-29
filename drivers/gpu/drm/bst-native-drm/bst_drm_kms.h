/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#ifndef _BST_DRM_KMS_H_
#define _BST_DRM_KMS_H_

#include <linux/list.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_writeback.h>
#include <drm/drm_print.h>


struct bst_plane {
	struct drm_plane base;
	struct bst_layer *layer;
};

struct bst_plane_state {
	struct drm_plane_state base;
	struct list_head zlist_node;
	u8 layer_split : 1;
};

struct bst_wb_connector {
	struct drm_writeback_connector base;
	struct bst_layer *wb_layer;
};

struct bst_crtc {
	struct drm_crtc base;
	struct bst_pipeline *master;
	struct bst_pipeline *slave;
	u32 slave_planes;
	struct bst_wb_connector *wb_conn;
	struct completion *disable_done;
};

struct bst_crtc_state {
	struct drm_crtc_state base;
	u32 affected_pipes;
	u32 active_pipes;
	u64 clock_ratio;
	u32 max_slave_zorder;
};

struct bst_kms_dev {
	struct drm_device base;
	int n_crtcs;
	struct bst_crtc crtcs[BST_DRM_MAX_PIPELINES];
};

#define to_bplane(p)	container_of(p, struct bst_plane, base)
#define to_bplane_st(p)	container_of(p, struct bst_plane_state, base)
#define to_bconn(p)	container_of(p, struct bst_wb_connector, base)
#define to_bcrtc(p)	container_of(p, struct bst_crtc, base)
#define to_bcrtc_st(p)	container_of(p, struct bst_crtc_state, base)
#define to_bdev(p)	container_of(p, struct bst_kms_dev, base)
#define to_wb_conn(x)	container_of(x, struct drm_writeback_connector, base)

static inline bool is_writeback_only(struct drm_crtc_state *st)
{
	struct bst_wb_connector *wb_conn = to_bcrtc(st->crtc)->wb_conn;
	struct drm_connector *conn = wb_conn ? &wb_conn->base.base : NULL;

	return conn && (st->connector_mask == BIT(drm_connector_index(conn)));
}

static inline bool
is_only_changed_connector(struct drm_crtc_state *st, struct drm_connector *conn)
{
	struct drm_crtc_state *old_st;
	u32 changed_connectors;

	old_st = drm_atomic_get_old_crtc_state(st->state, st->crtc);
	changed_connectors = st->connector_mask ^ old_st->connector_mask;

	return BIT(drm_connector_index(conn)) == changed_connectors;
}

static inline bool has_flip_h(u32 rot)
{
	u32 rotation = drm_rotation_simplify(rot,
					     DRM_MODE_ROTATE_0 |
					     DRM_MODE_ROTATE_90 |
					     DRM_MODE_REFLECT_MASK);

	if (rotation & DRM_MODE_ROTATE_90)
		return !!(rotation & DRM_MODE_REFLECT_Y);
	else
		return !!(rotation & DRM_MODE_REFLECT_X);
}

void bst_crtc_get_color_config(struct drm_crtc_state *crtc_st,
				  u32 *color_depths, u32 *color_formats);
unsigned long bst_crtc_get_aclk(struct bst_crtc_state *bcrtc_st);

int bst_kms_setup_crtcs(struct bst_kms_dev *kms, struct bst_dev *mdev);

int bst_kms_add_crtcs(struct bst_kms_dev *kms, struct bst_dev *mdev);
int bst_kms_add_planes(struct bst_kms_dev *kms, struct bst_dev *mdev);
int bst_kms_add_private_objs(struct bst_kms_dev *kms,
				struct bst_dev *mdev);
int bst_kms_add_wb_connectors(struct bst_kms_dev *kms,
				 struct bst_dev *mdev);
void bst_kms_cleanup_private_objs(struct bst_kms_dev *kms);

void bst_crtc_handle_event(struct bst_crtc *bcrtc,
			      struct bst_events *evts);
void bst_crtc_flush_and_wait_for_flip_done(struct bst_crtc *bcrtc,
					 struct completion *input_flip_done);
struct bst_kms_dev *bst_kms_attach(struct bst_dev *mdev);
void bst_kms_detach(struct bst_kms_dev *kms);

#endif /*_BST_DRM_KMS_H_*/
