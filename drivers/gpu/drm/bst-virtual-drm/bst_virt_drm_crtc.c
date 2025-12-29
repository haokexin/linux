// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "bst_virt_drm_device.h"
#include "bst_virt_drm_kms.h"
#include "bst_virt_pipeline.h"

void bst_crtc_get_color_config(struct drm_crtc_state *crtc_st,
			       u32 *color_depths, u32 *color_formats)
{
	struct drm_connector *conn;
	struct drm_connector_state *conn_st;
	u32 conn_color_formats = ~0u;
	int i, min_bpc = 31, conn_bpc = 0;

	for_each_new_connector_in_state(crtc_st->state, conn, conn_st, i) {
		if (conn_st->crtc != crtc_st->crtc)
			continue;

		conn_bpc = conn->display_info.bpc ? conn->display_info.bpc : 8;
		conn_color_formats &= conn->display_info.color_formats;

		if (conn_bpc < min_bpc)
			min_bpc = conn_bpc;
	}

	if (!conn_color_formats)
		conn_color_formats = DRM_COLOR_FORMAT_RGB444;

	*color_depths = GENMASK(min_bpc, 0);
	*color_formats = conn_color_formats;
}

static void bst_crtc_update_clock_ratio(struct bst_crtc_state *bcrtc_st)
{
	u64 pxlclk, aclk;

	if (!bcrtc_st->base.active) {
		bcrtc_st->clock_ratio = 0;
		return;
	}

	pxlclk = bcrtc_st->base.adjusted_mode.crtc_clock * 1000ULL;
	aclk = bst_crtc_get_aclk(bcrtc_st);

	bcrtc_st->clock_ratio = div64_u64(aclk << 32, pxlclk);
}

static int
bst_crtc_atomic_check(struct drm_crtc *crtc,
			 struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(crtc_state);
	int err;

	if (drm_atomic_crtc_needs_modeset(crtc_state))
		bst_crtc_update_clock_ratio(bcrtc_st);

	if (crtc_state->active) {
		err = bst_build_display_data_flow(bcrtc, bcrtc_st);
		if (err)
			return err;
	}

	err = bst_release_unclaimed_resources(bcrtc->master, bcrtc_st);
	if (err)
		return err;

	return 0;
}

void bst_crtc_handle_event(struct bst_crtc *bcrtc, struct bst_virt_events *evts)
{
	struct drm_crtc *crtc = &bcrtc->base;
	u32 events = evts->pipes;

	if (events & BST_DRM_EVENT_VSYNC)
		drm_crtc_handle_vblank(crtc);

	if (events & BST_DRM_EVENT_EOW) {
		struct bst_wb_connector *wb_conn = bcrtc->wb_conn;

		DRM_DEBUG("EOW.\n");
		if(bcrtc->force_wb_flag) {
			bcrtc->force_wb_flag = 0;
			complete_all(bcrtc->force_wb_comp);
		} else {
			if (wb_conn) {
				drm_writeback_signal_completion(&wb_conn->base, 0);
				DRM_INFO("CRTC[%d]: EOW happen on wb_connector.\n",
								drm_crtc_index(&bcrtc->base));
			} else
				DRM_WARN("CRTC[%d]: EOW happen but no wb_connector.\n",
					drm_crtc_index(&bcrtc->base));
		}
	}

	if (events & BST_DRM_EVENT_FLIP) {
		unsigned long flags;
		struct drm_pending_vblank_event *event;

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		if (bcrtc->disable_done) {
			complete_all(bcrtc->disable_done);
			bcrtc->disable_done = NULL;
			DRM_DEBUG_DRIVER("CRTC[%d]: pipe%d FLIP is disable done!\n", drm_crtc_index(&bcrtc->base), bcrtc->master->pipe_id);
		} else if (crtc->state->event) {
			event = crtc->state->event;
			/*
			 * Consume event before notifying drm core that flip
			 * happened.
			 */
			crtc->state->event = NULL;
			drm_crtc_send_vblank_event(crtc, event);
		} else {
			DRM_DEBUG_DRIVER("CRTC[%d]: FLIP happened but no pending commit.\n",
				 drm_crtc_index(&bcrtc->base));
		}
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}

static void bst_crtc_do_flush(struct drm_crtc *crtc, struct drm_crtc_state *old)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(crtc->state);
	struct bst_virt_pipe *master = bcrtc->master;
	struct bst_wb_connector *wb_conn = bcrtc->wb_conn;
	struct drm_connector_state *conn_st;
	//struct bst_virt_device *dc_dev = master->subdevs[BST_VIRT_DC_IDX];

	DRM_DEBUG_ATOMIC("CRTC%d_FLUSH: active_pipes: 0x%x, affected: 0x%x.\n",
			 drm_crtc_index(crtc), bcrtc_st->active_pipes,
			 bcrtc_st->affected_pipes);

	if (has_bit(master->pipe_id, bcrtc_st->affected_pipes))
		bst_virt_pipe_update(master, old->state);

	conn_st = wb_conn ? wb_conn->base.base.state : NULL;
	if (conn_st && conn_st->writeback_job)
		drm_writeback_queue_job(&wb_conn->base, conn_st);

	//dc_dev->funcs->flush(dc_dev);
}

static void
bst_crtc_atomic_enable(struct drm_crtc *crtc,
			  struct drm_atomic_state *state)
{
	struct drm_crtc_state *old = drm_atomic_get_old_crtc_state(state,
								   crtc);

	pm_runtime_get_sync(crtc->dev->dev);
	drm_crtc_vblank_on(crtc);
	WARN_ON(drm_crtc_vblank_get(crtc));
	bst_crtc_do_flush(crtc, old);
}

void bst_crtc_hw_flush(struct bst_crtc *bcrtc)
{
	struct bst_virt_device *dc_dev = bcrtc->master->subdevs[BST_VIRT_DC_IDX];
	dc_dev->funcs->flush(dc_dev);
}

void bst_crtc_wait_for_hw_flip_done(struct bst_crtc *bcrtc,
					 struct completion *input_flip_done)
{
	struct drm_device *drm = bcrtc->base.dev;
	struct completion *flip_done;
	struct completion temp;
	int timeout;

	if (input_flip_done) {
		flip_done = input_flip_done;
	} else {
		init_completion(&temp);
		bcrtc->disable_done = &temp;
		flip_done = &temp;
	}

	timeout = wait_for_completion_timeout(flip_done, HZ * 0.3);
	if (timeout == 0) {
		unsigned long flags;
		struct drm_crtc *crtc = &bcrtc->base;
		struct drm_pending_vblank_event *event;

		DRM_INFO("[%s]wait pipe-%d flip done timeout 300ms, send evt.\n", __func__, bcrtc->master->pipe_id);
		spin_lock_irqsave(&drm->event_lock, flags);
		event = crtc->state->event;
		crtc->state->event = NULL;
		if (event)
			drm_crtc_send_vblank_event(crtc, event);

		if (!input_flip_done) {
			bcrtc->disable_done = NULL;
		}
		spin_unlock_irqrestore(&drm->event_lock, flags);
	}
}

static void
bst_crtc_flush_and_wait_for_flip_done(struct bst_crtc *bcrtc,
				      struct completion *input_flip_done)
{
	struct bst_virt_device *dc_dev = bcrtc->master->subdevs[BST_VIRT_DC_IDX];
	struct drm_device *drm = bcrtc->base.dev;
	struct completion *flip_done;
	struct completion temp;
	int timeout;

	if (input_flip_done) {
		flip_done = input_flip_done;
	} else {
		init_completion(&temp);
		bcrtc->disable_done = &temp;
		flip_done = &temp;
	}

	dc_dev->funcs->flush(dc_dev);
	timeout = wait_for_completion_timeout(flip_done, HZ);
	if (timeout == 0) {
		DRM_ERROR("wait pipe%d flip done timeout\n",
			  bcrtc->master->pipe_id);
		if (!input_flip_done) {
			unsigned long flags;

			spin_lock_irqsave(&drm->event_lock, flags);
			bcrtc->disable_done = NULL;
			spin_unlock_irqrestore(&drm->event_lock, flags);
		}
	}
}

static void
bst_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct drm_crtc_state *old = drm_atomic_get_old_crtc_state(state,
								   crtc);
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_crtc_state *old_st = to_bcrtc_st(old);
	struct bst_virt_pipe *master = bcrtc->master;
	struct completion *disable_done;
	bool needs_phase2 = false;

	DRM_DEBUG_ATOMIC("CRTC%d_DISABLE: active_pipes: 0x%x, affected: 0x%x\n",
			 drm_crtc_index(crtc), old_st->active_pipes,
			 old_st->affected_pipes);

	if (has_bit(master->pipe_id, old_st->active_pipes))
		needs_phase2 = bst_virt_pipe_disable(master, old->state);

	disable_done = (needs_phase2 || crtc->state->active) ?
				     NULL : &crtc->state->commit->flip_done;

	bst_crtc_flush_and_wait_for_flip_done(bcrtc, disable_done);

	if (needs_phase2) {
		bst_virt_pipe_disable(bcrtc->master, old->state);

		disable_done = crtc->state->active ?
					     NULL : &crtc->state->commit->flip_done;

		bst_crtc_flush_and_wait_for_flip_done(bcrtc, disable_done);
	}

	drm_crtc_vblank_put(crtc);
	drm_crtc_vblank_off(crtc);
	pm_runtime_put(crtc->dev->dev);
}

static void
bst_crtc_atomic_flush(struct drm_crtc *crtc,
			 struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct drm_crtc_state *old = drm_atomic_get_old_crtc_state(state,
								   crtc);

	if (drm_atomic_crtc_needs_modeset(crtc_state))
		return;

	bst_crtc_do_flush(crtc, old);
}

static unsigned long
bst_calc_min_aclk_rate(struct bst_crtc *bcrtc,
			  unsigned long pxlclk)
{
	if (bcrtc->master->dual_link)
		return pxlclk * 2;
	else
		return pxlclk;
}

#define DISPLAY_ACLK_800MHz     (800000000)
#define DISPLAY_ACLK_1000MHz    (1000000000)
unsigned long bst_crtc_get_aclk(struct bst_crtc_state *bcrtc_st)
{
	struct drm_crtc *crtc = bcrtc_st->base.crtc;
	unsigned long pxlclk = bcrtc_st->base.adjusted_mode.crtc_clock * 1000;
	unsigned long min_aclk;
	min_aclk = bst_calc_min_aclk_rate(to_bcrtc(crtc), pxlclk);
	if (min_aclk <= DISPLAY_ACLK_800MHz) {
		return DISPLAY_ACLK_800MHz;
	} else {
		return DISPLAY_ACLK_1000MHz;
	}
}

static enum drm_mode_status
bst_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *m)
{
	if (m->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	return MODE_OK;
}

static bool bst_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *m,
				struct drm_display_mode *adjusted_mode)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);

	drm_mode_set_crtcinfo(adjusted_mode, 0);
	if (bcrtc->master->dual_link) {
		adjusted_mode->crtc_clock /= 2;
		adjusted_mode->crtc_hdisplay /= 2;
		adjusted_mode->crtc_hsync_start /= 2;
		adjusted_mode->crtc_hsync_end /= 2;
		adjusted_mode->crtc_htotal /= 2;
	}

	return true;
}
static int bst_crtc_atomic_set_property(struct drm_crtc *crtc,
					 struct drm_crtc_state *state,
					 struct drm_property *property,
					 u64 val)
{
	drm_dbg_atomic(crtc->dev, "Unknown property [PROP:%d:%s]\n",
		       property->base.id, property->name);
	return 0;
}

static int bst_crtc_atomic_get_property(struct drm_crtc *crtc,
					 const struct drm_crtc_state *state,
					 struct drm_property *property,
					 u64 *val)
{
	return 0;
}
static const struct drm_crtc_helper_funcs bst_crtc_helper_funcs = {
	.atomic_check = bst_crtc_atomic_check,
	.atomic_flush = bst_crtc_atomic_flush,
	.atomic_enable = bst_crtc_atomic_enable,
	.atomic_disable = bst_crtc_atomic_disable,
	.mode_valid = bst_crtc_mode_valid,
	.mode_fixup = bst_crtc_mode_fixup,
};

static void bst_crtc_reset(struct drm_crtc *crtc)
{
	struct bst_crtc_state *state;

	if (crtc->state)
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

	kfree(to_bcrtc_st(crtc->state));
	crtc->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_crtc_reset(crtc, &state->base);
}

static struct drm_crtc_state *
bst_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct bst_crtc_state *old = to_bcrtc_st(crtc->state);
	struct bst_crtc_state *new;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &new->base);

	new->affected_pipes = old->active_pipes;
	new->clock_ratio = old->clock_ratio;
	new->en_scaling = old->en_scaling;

	return &new->base;
}

static void bst_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					  struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_bcrtc_st(state));
}

static int bst_crtc_vblank_enable(struct drm_crtc *crtc)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_virt_device *dc_dev = bcrtc->master->subdevs[BST_VIRT_DC_IDX];

	dc_dev->funcs->on_off_vblank(dc_dev, true, bcrtc);
	return 0;
}

static void bst_crtc_vblank_disable(struct drm_crtc *crtc)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_virt_device *dc_dev = bcrtc->master->subdevs[BST_VIRT_DC_IDX];

	dc_dev->funcs->on_off_vblank(dc_dev, false, bcrtc);
}

static const struct drm_crtc_funcs bst_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = bst_crtc_reset,
	.atomic_duplicate_state = bst_crtc_atomic_duplicate_state,
	.atomic_destroy_state = bst_crtc_atomic_destroy_state,
	.enable_vblank = bst_crtc_vblank_enable,
	.disable_vblank = bst_crtc_vblank_disable,
	.atomic_set_property = bst_crtc_atomic_set_property,
	.atomic_get_property = bst_crtc_atomic_get_property,
};

int bst_kms_setup_crtcs(struct bst_kms_dev *kms,
			struct bst_super_device *super_dev)
{
	struct bst_crtc *crtc;
	struct bst_virt_pipe *master;
	int i;

	kms->n_crtcs = 0;

	for (i = 0; i < super_dev->n_pipelines; i++) {
		crtc = &kms->crtcs[kms->n_crtcs];
		master = super_dev->pipelines[i];
		crtc->master = master;

		DRM_DEBUG("CRTC-%d: master(pipe-%d)\n", kms->n_crtcs,
			 master->pipe_id);

		kms->n_crtcs++;
	}

	return 0;
}

static struct drm_plane *
get_crtc_primary(struct bst_kms_dev *kms, struct bst_crtc *crtc)
{
	struct bst_plane *bplane;
	struct drm_plane *plane;

	drm_for_each_plane(plane, &kms->base) {
		if (plane->type != DRM_PLANE_TYPE_PRIMARY)
			continue;

		bplane = to_bplane(plane);
		if (bplane->layer->base.pipe == crtc->master)
			return plane;
	}

	return NULL;
}

static int bst_crtc_create_prop(struct bst_crtc *bcrtc,
				struct drm_mode_object *obj)
{
	return 0;
}

static int bst_crtc_add(struct bst_kms_dev *kms, struct bst_crtc *bcrtc)
{
	struct drm_crtc *crtc = &bcrtc->base;
	int err;

	err = drm_crtc_init_with_planes(&kms->base, crtc,
					get_crtc_primary(kms, bcrtc), NULL,
					&bst_crtc_funcs, NULL);
	if (err)
		return err;

	drm_crtc_helper_add(crtc, &bst_crtc_helper_funcs);

	crtc->port = bcrtc->master->of_output_port;

	drm_crtc_enable_color_mgmt(crtc, 0, true, BST_DRM_COLOR_LUT_SIZE);

	bst_crtc_create_prop(bcrtc, &crtc->base);

	return err;
}

int bst_kms_add_crtcs(struct bst_kms_dev *kms,
		      struct bst_super_device *super_dev)
{
	int i, err;

	for (i = 0; i < kms->n_crtcs; i++) {
		err = bst_crtc_add(kms, &kms->crtcs[i]);
		if (err)
			return err;
	}

	return 0;
}
