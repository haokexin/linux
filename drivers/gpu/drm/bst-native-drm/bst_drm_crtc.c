// SPDX-License-Identifier: GPL-2.0
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
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "bst_drm_dev.h"
#include "bst_drm_kms.h"
#include "bst_dpu_csr.h"

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

	err = bst_release_unclaimed_resources(bcrtc->slave, bcrtc_st);
	if (err)
		return err;

	err = bst_release_unclaimed_resources(bcrtc->master, bcrtc_st);
	if (err)
		return err;

	return 0;
}

static int
bst_crtc_prepare(struct bst_crtc *bcrtc)
{
	struct bst_dev *mdev = bcrtc->base.dev->dev_private;
	struct bst_pipeline *master = bcrtc->master;
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(bcrtc->base.state);
	struct drm_display_mode *mode = &bcrtc_st->base.adjusted_mode;
	u32 new_mode;
	int err;

	mutex_lock(&mdev->lock);

	new_mode = mdev->dpmode | BIT(master->id);
	if (WARN_ON(new_mode == mdev->dpmode)) {
		err = 0;
		goto unlock;
	}

	err = mdev->funcs->change_opmode(mdev, new_mode);
	if (err) {
		DRM_ERROR("failed to change opmode: 0x%x -> 0x%x.\n,",
			  mdev->dpmode, new_mode);
		goto unlock;
	}

	mdev->dpmode = new_mode;
	bst_csr_clear_frame_counter(mdev->dev, master->id);

	if(strstr(master->of_output_links[0]->full_name, "lvds")) {
        bst_clk_set_rate_only(master->pll_clk, mode->crtc_clock * 1000 * 7);
        bst_clk_set_rate_divider(master->div_clk, mode->crtc_clock * 1000);
	} else {
		clk_set_rate(master->pll_clk, 1188000000);
		clk_set_rate(master->div_clk,mode->crtc_clock * 1000);
	}

unlock:
	mutex_unlock(&mdev->lock);

	return err;
}

static int
bst_crtc_unprepare(struct bst_crtc *bcrtc)
{
	struct bst_dev *mdev = bcrtc->base.dev->dev_private;
	struct bst_pipeline *master = bcrtc->master;
	u32 new_mode;
	int err;

	mutex_lock(&mdev->lock);

	new_mode = mdev->dpmode & (~BIT(master->id));

	if (WARN_ON(new_mode == mdev->dpmode)) {
		err = 0;
		goto unlock;
	}

	err = mdev->funcs->change_opmode(mdev, new_mode);
	if (err) {
		DRM_ERROR("failed to change opmode: 0x%x -> 0x%x.\n,",
			  mdev->dpmode, new_mode);
		goto unlock;
	}

	mdev->dpmode = new_mode;

unlock:
	mutex_unlock(&mdev->lock);

	return err;
}

void bst_crtc_handle_event(struct bst_crtc   *bcrtc,
			      struct bst_events *evts)
{
	struct drm_crtc *crtc = &bcrtc->base;
	u32 events = evts->pipes[bcrtc->master->id];
	struct bst_dev *mdev = bcrtc->base.dev->dev_private;

	if (events & BST_DRM_EVENT_VSYNC) {
		mdev->frame_count[bcrtc->master->id]++;
		drm_crtc_handle_vblank(crtc);
	}

	if (events & BST_DRM_EVENT_EOW) {
		struct bst_wb_connector *wb_conn = bcrtc->wb_conn;

		if (wb_conn)
			drm_writeback_signal_completion(&wb_conn->base, 0);
		else
			DRM_WARN("CRTC[%d]: EOW happen but no wb_connector.\n",
				 drm_crtc_index(&bcrtc->base));
	}

	if (events & BST_DRM_EVENT_EOW)
		DRM_DEBUG("EOW.\n");

	if (events & BST_DRM_EVENT_FLIP) {
		unsigned long flags;
		struct drm_pending_vblank_event *event;

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		if (bcrtc->disable_done) {
			complete_all(bcrtc->disable_done);
			bcrtc->disable_done = NULL;
		} else if (crtc->state->event) {
			event = crtc->state->event;
			crtc->state->event = NULL;
			drm_crtc_send_vblank_event(crtc, event);
		} else {
			DRM_WARN("CRTC[%d]: FLIP happen but no pending commit.\n",
					drm_crtc_index(&bcrtc->base));
		}
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}
EXPORT_SYMBOL(bst_crtc_handle_event);

static void
bst_crtc_do_flush(struct drm_crtc *crtc,
		     struct drm_crtc_state *old)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(crtc->state);
	struct bst_pipeline *master = bcrtc->master;
	struct bst_pipeline *slave = bcrtc->slave;
	struct bst_wb_connector *wb_conn = bcrtc->wb_conn;
	struct drm_connector_state *conn_st;
	struct bst_dev *mdev = bcrtc->master->mdev;

	DRM_DEBUG_ATOMIC("CRTC%d_FLUSH: active_pipes: 0x%x, affected: 0x%x.\n",
			 drm_crtc_index(crtc),
			 bcrtc_st->active_pipes, bcrtc_st->affected_pipes);

	if (has_bit(master->id, bcrtc_st->affected_pipes))
		bst_pipeline_update(master, old->state);

	if (slave && has_bit(slave->id, bcrtc_st->affected_pipes))
		bst_pipeline_update(slave, old->state);

	conn_st = wb_conn ? wb_conn->base.base.state : NULL;
	if (conn_st && conn_st->writeback_job)
		drm_writeback_queue_job(&wb_conn->base, conn_st);

	mdev->funcs->flush(mdev, bcrtc->master->id, bcrtc_st->active_pipes);
}

static void
bst_crtc_atomic_enable(struct drm_crtc *crtc,
			  struct drm_atomic_state *state)
{
	struct drm_crtc_state *old = drm_atomic_get_old_crtc_state(state,
								   crtc);
	pm_runtime_get_sync(crtc->dev->dev);
	bst_crtc_prepare(to_bcrtc(crtc));
	drm_crtc_vblank_on(crtc);
	WARN_ON(drm_crtc_vblank_get(crtc));
	bst_crtc_do_flush(crtc, old);
}

void bst_crtc_flush_and_wait_for_flip_done(struct bst_crtc *bcrtc,
					 struct completion *input_flip_done)
{
	struct drm_device *drm = bcrtc->base.dev;
	struct bst_dev *mdev = bcrtc->master->mdev;
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

	mdev->funcs->flush(mdev, bcrtc->master->id, 0);
	timeout = wait_for_completion_timeout(flip_done, HZ);
	if (timeout == 0) {
		DRM_ERROR("wait pipe%d flip done timeout\n", bcrtc->master->id);
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
	struct bst_pipeline *master = bcrtc->master;
	struct bst_pipeline *slave  = bcrtc->slave;
	struct completion *disable_done;
	bool needs_phase2 = false;

	DRM_INFO("CRTC%d_DISABLE: active_pipes: 0x%x, affected: 0x%x\n",
			 drm_crtc_index(crtc),
			 old_st->active_pipes, old_st->affected_pipes);

	if (slave && has_bit(slave->id, old_st->active_pipes))
		bst_pipeline_disable(slave, old->state);

	if (has_bit(master->id, old_st->active_pipes))
		needs_phase2 = bst_pipeline_disable(master, old->state);

	disable_done = (needs_phase2 || crtc->state->active) ?
		       NULL : &crtc->state->commit->flip_done;

	bst_crtc_flush_and_wait_for_flip_done(bcrtc, disable_done);

	if (needs_phase2) {
		bst_pipeline_disable(bcrtc->master, old->state);

		disable_done = crtc->state->active ?
			       NULL : &crtc->state->commit->flip_done;

		bst_crtc_flush_and_wait_for_flip_done(bcrtc, disable_done);
	}

	drm_crtc_vblank_put(crtc);
	drm_crtc_vblank_off(crtc);
	bst_crtc_unprepare(bcrtc);
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
	unsigned long min_aclk;
	unsigned long pxlclk = bcrtc_st->base.adjusted_mode.crtc_clock * 1000;

	min_aclk = bst_calc_min_aclk_rate(to_bcrtc(crtc), pxlclk);
	if (min_aclk <= DISPLAY_ACLK_800MHz)
		return DISPLAY_ACLK_800MHz;
	 else
		return DISPLAY_ACLK_1000MHz;
}

static enum drm_mode_status
bst_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *m)
{
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_pipeline *master = bcrtc->master;
	unsigned long min_pxlclk, min_aclk;

	if (m->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	min_pxlclk = m->clock * 1000;
	if (master->dual_link)
		min_pxlclk /= 2;

	if (min_pxlclk != clk_round_rate(master->gate_pxlclk, min_pxlclk)) {
		DRM_DEBUG_ATOMIC("pxlclk doesn't support %lu %lu Hz\n", min_pxlclk, clk_round_rate(master->gate_pxlclk, min_pxlclk));

		return MODE_NOCLOCK;
	}

	min_aclk = bst_calc_min_aclk_rate(to_bcrtc(crtc), min_pxlclk);

	if (clk_round_rate(master->mux_aclk, min_aclk) < min_aclk) {
		DRM_DEBUG_ATOMIC("engine clk can't satisfy the requirement of %s-clk: %lu.\n",
				 m->name, min_pxlclk);

		return MODE_CLOCK_HIGH;
	}

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

static const struct drm_crtc_helper_funcs bst_crtc_helper_funcs = {
	.atomic_check	= bst_crtc_atomic_check,
	.atomic_flush	= bst_crtc_atomic_flush,
	.atomic_enable	= bst_crtc_atomic_enable,
	.atomic_disable	= bst_crtc_atomic_disable,
	.mode_valid	= bst_crtc_mode_valid,
	.mode_fixup	= bst_crtc_mode_fixup,
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
	new->max_slave_zorder = old->max_slave_zorder;

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
	struct bst_dev *mdev = crtc->dev->dev_private;
	struct bst_crtc *bcrtc = to_bcrtc(crtc);

	mdev->funcs->on_off_vblank(mdev, bcrtc->master->id, true, bcrtc);
	return 0;
}

static void bst_crtc_vblank_disable(struct drm_crtc *crtc)
{
	struct bst_dev *mdev = crtc->dev->dev_private;
	struct bst_crtc *bcrtc = to_bcrtc(crtc);

	mdev->funcs->on_off_vblank(mdev, bcrtc->master->id, false, bcrtc);
}

static const struct drm_crtc_funcs bst_crtc_funcs = {
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= bst_crtc_reset,
	.atomic_duplicate_state	= bst_crtc_atomic_duplicate_state,
	.atomic_destroy_state	= bst_crtc_atomic_destroy_state,
	.enable_vblank		= bst_crtc_vblank_enable,
	.disable_vblank		= bst_crtc_vblank_disable,
};

int bst_kms_setup_crtcs(struct bst_kms_dev *kms,
			   struct bst_dev *mdev)
{
	struct bst_crtc *crtc;
	struct bst_pipeline *master;
	char str[16];
	int i;

	kms->n_crtcs = 0;

	for (i = 0; i < mdev->n_pipelines; i++) {
		crtc = &kms->crtcs[kms->n_crtcs];
		master = mdev->pipelines[i];

		crtc->master = master;
		crtc->slave  = bst_pipeline_get_slave(master);

		if (crtc->slave)
			sprintf(str, "pipe-%d", crtc->slave->id);
		else
			sprintf(str, "None");

		DRM_INFO("CRTC-%d: master(pipe-%d) slave(%s).\n",
			 kms->n_crtcs, master->id, str);

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
		if (bplane->layer->base.pipeline == crtc->master)
			return plane;
	}

	return NULL;
}

static int bst_crtc_add(struct bst_kms_dev *kms,
			   struct bst_crtc *bcrtc)
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

	return err;
}

int bst_kms_add_crtcs(struct bst_kms_dev *kms, struct bst_dev *mdev)
{
	int i, err;

	for (i = 0; i < kms->n_crtcs; i++) {
		err = bst_crtc_add(kms, &kms->crtcs[i]);
		if (err)
			return err;
	}

	return 0;
}
