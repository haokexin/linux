// SPDX-License-Identifier: GPL-2.0
/*
 * (C) COPYRIGHT 2018 ARM Limited. All rights reserved.
 * Author: James.Qian.Wang <james.qian.wang@arm.com>
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
 
#include <linux/component.h>
#include <linux/interrupt.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "bst_drm_dev.h"
#include "bst_drm_framebuffer.h"
#include "bst_drm_kms.h"

DEFINE_DRM_GEM_DMA_FOPS(bst_cma_fops);

static int bst_gem_dma_dumb_create(struct drm_file *file,
				      struct drm_device *dev,
				      struct drm_mode_create_dumb *args)
{
	struct bst_dev *mdev = dev->dev_private;
	u32 pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	args->pitch = ALIGN(pitch, mdev->chip.bus_width);

	return drm_gem_dma_dumb_create_internal(file, dev, args);
}

static irqreturn_t bst_kms_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct bst_dev *mdev = drm->dev_private;
	struct bst_kms_dev *kms = to_bdev(drm);
	struct bst_events evts;
	irqreturn_t status;
	u32 i;

	memset(&evts, 0, sizeof(evts));
	if((mdev->funcs != NULL)&&(mdev->funcs->irq_handler != NULL))
	    status = mdev->funcs->irq_handler(mdev, &evts);

	bst_print_events(&evts, drm);

	for (i = 0; i < kms->n_crtcs; i++)
		bst_crtc_handle_event(&kms->crtcs[i], &evts);

	return status;
}

static struct drm_driver bst_kms_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.lastclose			= drm_fb_helper_lastclose,
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(bst_gem_dma_dumb_create),
	.fops = &bst_cma_fops,
	.name = "bst-drm",
	.desc = "BST Native Display Controllor driver",
	.date = "20231101",
	.major = 0,
	.minor = 1,
};

static void bst_kms_atomic_commit_hw_done(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;
	struct bst_kms_dev *kms = to_bdev(dev);
	int i;
	unsigned long flags;
	int wait_flag = 0;


	for (i = 0; i < kms->n_crtcs; i++) {
		struct bst_crtc *bcrtc = &kms->crtcs[i];

		if (bcrtc->base.state->active) {
			struct completion *flip_done = NULL;
			wait_flag = 0;

			spin_lock_irqsave(&dev->event_lock, flags);
			if (bcrtc->base.state->event){
				flip_done = bcrtc->base.state->event->base.completion;
				wait_flag = 1;
			}
			spin_unlock_irqrestore(&dev->event_lock, flags);

			if(1 == wait_flag) {
				bst_crtc_flush_and_wait_for_flip_done(bcrtc, flip_done);
			}
		}
	}

	drm_atomic_helper_commit_hw_done(state);
}

static void bst_kms_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *drm = old_state->dev;
	bool fence_cookie = dma_fence_begin_signalling();
	ktime_t start_time, end_time, diff_time;
	struct bst_dev *mdev = dev_to_mdev(drm->dev);

	start_time = ktime_get();

	drm_atomic_helper_commit_modeset_disables(drm, old_state);

	drm_atomic_helper_commit_planes(drm, old_state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_modeset_enables(drm, old_state);

	bst_kms_atomic_commit_hw_done(old_state);

	drm_atomic_helper_wait_for_flip_done(drm, old_state);

	dma_fence_end_signalling(fence_cookie);

	drm_atomic_helper_cleanup_planes(drm, old_state);

	end_time = ktime_get();
	diff_time = ktime_sub(end_time, start_time);
	mdev->cur_commit_time = ktime_to_ns(diff_time) / 1000;
	mdev->commit_counts++;
	mdev->commit_time_sum += mdev->cur_commit_time;
	mdev->avg_commit_time = mdev->commit_time_sum / mdev->commit_counts;
}

static const struct drm_mode_config_helper_funcs bst_mode_config_helpers = {
	.atomic_commit_tail = bst_kms_commit_tail,
};

static int bst_plane_state_list_add(struct drm_plane_state *plane_st,
				       struct list_head *zorder_list)
{
	struct bst_plane_state *new = to_bplane_st(plane_st);
	struct bst_plane_state *node, *last;

	last = list_empty(zorder_list) ?
	       NULL : list_last_entry(zorder_list, typeof(*last), zlist_node);

	if (!last || (new->base.zpos > last->base.zpos)) {
		list_add_tail(&new->zlist_node, zorder_list);
		return 0;
	}

	list_for_each_entry(node, zorder_list, zlist_node) {
		if (new->base.zpos < node->base.zpos) {
			list_add_tail(&new->zlist_node, &node->zlist_node);
			break;
		} else if (node->base.zpos == new->base.zpos) {
			struct drm_plane *a = node->base.plane;
			struct drm_plane *b = new->base.plane;

			DRM_DEBUG_ATOMIC("PLANE: %s and PLANE: %s are configured same zpos: %d.\n",
					 a->name, b->name, node->base.zpos);
			return -EINVAL;
		}
	}

	return 0;
}

static int bst_crtc_normalize_zpos(struct drm_crtc *crtc,
				      struct drm_crtc_state *crtc_st)
{
	struct drm_atomic_state *state = crtc_st->state;
	struct bst_crtc *bcrtc = to_bcrtc(crtc);
	struct bst_crtc_state *bcrtc_st = to_bcrtc_st(crtc_st);
	struct bst_plane_state *bplane_st;
	struct drm_plane_state *plane_st;
	struct drm_plane *plane;
	struct list_head zorder_list;
	int order = 0, err;

	DRM_DEBUG_ATOMIC("[CRTC:%d:%s] calculating normalized zpos values\n",
			 crtc->base.id, crtc->name);

	INIT_LIST_HEAD(&zorder_list);
	drm_for_each_plane_mask(plane, crtc->dev, crtc_st->plane_mask) {
		plane_st = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_st))
			return PTR_ERR(plane_st);

		err = bst_plane_state_list_add(plane_st, &zorder_list);
		if (err)
			return err;
	}

	bcrtc_st->max_slave_zorder = 0;

	list_for_each_entry(bplane_st, &zorder_list, zlist_node) {
		plane_st = &bplane_st->base;
		plane = plane_st->plane;

		plane_st->normalized_zpos = order++;
		if (to_bplane_st(plane_st)->layer_split)
			order++;

		DRM_DEBUG_ATOMIC("[PLANE:%d:%s] zpos:%d, normalized zpos: %d\n",
				 plane->base.id, plane->name,
				 plane_st->zpos, plane_st->normalized_zpos);

		if (has_bit(drm_plane_index(plane), bcrtc->slave_planes))
			bcrtc_st->max_slave_zorder =
				max(plane_st->normalized_zpos,
				    bcrtc_st->max_slave_zorder);
	}

	crtc_st->zpos_changed = true;

	return 0;
}

static int bst_kms_check(struct drm_device *dev,
			    struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_st;
	int i, err;

	err = drm_atomic_helper_check_modeset(dev, state);
	if (err)
		return err;

	for_each_new_crtc_in_state(state, crtc, new_crtc_st, i) {
		err = drm_atomic_add_affected_planes(state, crtc);
		if (err)
			return err;

		err = bst_crtc_normalize_zpos(crtc, new_crtc_st);
		if (err)
			return err;
	}

	err = drm_atomic_helper_check_planes(dev, state);
	if (err)
		return err;

	return 0;
}

static const struct drm_mode_config_funcs bst_mode_config_funcs = {
	.fb_create		= bst_fb_create,
	.atomic_check		= bst_kms_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void bst_kms_mode_config_init(struct bst_kms_dev *kms,
					struct bst_dev *mdev)
{
	struct drm_mode_config *config = &kms->base.mode_config;

	drm_mode_config_init(&kms->base);

	bst_kms_setup_crtcs(kms, mdev);

	config->min_width	= 0;
	config->min_height	= 0;
	config->max_width	= 4096;
	config->max_height	= 4096;

	config->funcs = &bst_mode_config_funcs;
	config->helper_private = &bst_mode_config_helpers;
}

struct bst_kms_dev *bst_kms_attach(struct bst_dev *mdev)
{
	struct bst_kms_dev *kms;
	struct drm_device *drm;
	int err;

	kms = devm_drm_dev_alloc(mdev->dev, &bst_kms_driver,
				 struct bst_kms_dev, base);
	if (IS_ERR(kms))
		return kms;

	drm = &kms->base;

	drm->dev_private = mdev;

	bst_kms_mode_config_init(kms, mdev);

	err = bst_kms_add_private_objs(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = bst_kms_add_planes(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = drm_vblank_init(drm, kms->n_crtcs);
	if (err)
		goto cleanup_mode_config;

	err = bst_kms_add_crtcs(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = bst_kms_add_wb_connectors(kms, mdev);
	if (err)
		goto cleanup_mode_config;

	err = component_bind_all(mdev->dev, kms);
	if (err)
		goto cleanup_mode_config;

	drm_mode_config_reset(drm);

	err = devm_request_irq(drm->dev, mdev->irq,
			       bst_kms_irq_handler, IRQF_SHARED,
			       drm->driver->name, drm);
	if (err)
		goto free_component_binding;

	drm_kms_helper_poll_init(drm);

	err = drm_dev_register(drm, 0);
	if (err)
		goto free_interrupts;

	return kms;

free_interrupts:
	drm_kms_helper_poll_fini(drm);
free_component_binding:
	component_unbind_all(mdev->dev, drm);
cleanup_mode_config:
	drm_mode_config_cleanup(drm);
	bst_kms_cleanup_private_objs(kms);
	drm->dev_private = NULL;
	return ERR_PTR(err);
}

void bst_kms_detach(struct bst_kms_dev *kms)
{
	struct drm_device *drm = &kms->base;
	struct bst_dev *mdev = drm->dev_private;

	//devm_free_irq(drm->dev, mdev->irq, drm);
	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	component_unbind_all(mdev->dev, drm);
	drm_mode_config_cleanup(drm);
	bst_kms_cleanup_private_objs(kms);
	drm->dev_private = NULL;
}
