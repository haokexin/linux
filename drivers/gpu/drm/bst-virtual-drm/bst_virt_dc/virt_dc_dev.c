// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include "bst_virt_drm_device.h"
#include <drm/drm_print.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
#include <uapi/drm/drm_fourcc.h>
#include <drm/drm_blend.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <drm/drm_vblank.h>
#include "virt_dc_dev.h"
#include "bst_virt_drm_kms.h"
#include "bst_display_global_api.h"
#include "bst_virt_drm_debugfs.h"

static irqreturn_t
virt_dc_irq_handler(struct bst_virt_device *vdev,
		    const struct bst_display_events_status status)
{
	struct virt_dc_dev *dc = vdev->virt_dev_data;
	struct bst_crtc *bcrtc = dc->bcrtc;
	struct bst_virt_events evts;
	u32 global_status = 0;
	u32 events_type = status.events & ~vdev->events_mask;

	memset(&evts, 0, sizeof(evts));

	if (events_type) {
		DRM_DEBUG_VBL("virt dev(%d) evts:%#x\n", vdev->device_type, events_type);
		if (events_type & BST_EVENT_VSYNC)
			evts.pipes |= BST_DRM_EVENT_VSYNC;
		if (events_type & BST_EVENT_EOW)
			evts.pipes |= BST_DRM_EVENT_EOW;

		//NOTE: using flip timer instead of fw BST_EVENT_FLIP event
		//if (events_type & BST_EVENT_FLIP)
		//	dc->pipes->flush_done = true;
		bst_crtc_handle_event(bcrtc, &evts);
	}

	return IRQ_RETVAL(global_status);
}

void virt_dc_events_handler(const struct bst_display_events_status status, void *ext)
{
	struct bst_virt_device *virt_dev = (struct bst_virt_device *)ext;
	virt_dev->funcs->irq_handler(virt_dev, status);
}

static int virt_dc_enable_irq(struct bst_virt_device *vdev)
{
	int ret;
	ret = bst_display_glb_cmd_subscribe_events(
		to_fw_subdev_type(vdev->device_type), virt_dc_events_handler,
		vdev);
	if (ret)
		DRM_ERROR("VIRT DC Enable irq, ret=%d!\n", ret);

	DRM_INFO("VIRT DC(%d) Enable irq!\n",
		to_fw_subdev_type(vdev->device_type));

	return ret;
}

static int virt_dc_disable_irq(struct bst_virt_device *vdev)
{
	DRM_INFO("VIRT DC(%d) Disable irq!\n",
		 to_fw_subdev_type(vdev->device_type));
	return bst_display_glb_cmd_unsubscribe_events(
		to_fw_subdev_type(vdev->device_type));
}

static uint8_t get_min_fw_layer_id(struct bst_display_dc_probed_info *info)
{
	uint8_t i, min_layer_id = info->submodule_ids[0];
	uint8_t num_layers = info->num_submodules - FIXED_DC_SUBMODULE_NUM;

	for (i = 1; i < num_layers; i++) {
		if (info->submodule_ids[i] < min_layer_id)
			min_layer_id = info->submodule_ids[i];
	}

	return min_layer_id;
}

static void virt_dc_cleanup(struct bst_virt_device *vdev)
{
	struct virt_dc_dev *dc = vdev->virt_dev_data;

	if (!dc)
		return;

	kfree(dc);
	vdev->virt_dev_data = NULL;
}

static int virt_dc_probe(struct bst_virt_device *vdev)
{
	struct virt_dc_dev *dc_dev;
	struct bst_display_submodule_header submodule_head = { 0 };
	struct bst_display_submodule_req submodule_req = { 0 };
	struct bst_display_dc_probed_info *dc_probed_info = vdev->dev_info.private;
	u32 i = 0, subdev_session, client_id;
	int err;

	dc_dev = devm_kzalloc(vdev->dev, sizeof(*dc_dev), GFP_KERNEL);
	if (!dc_dev)
		return -ENOMEM;
	vdev->virt_dev_data = dc_dev;

	dc_dev->base_dev = vdev;
	dc_dev->max_vsize = dc_probed_info->max_vsize;
	dc_dev->max_line_size = dc_probed_info->max_hsize;
	dc_dev->num_rich_layers = dc_probed_info->num_rich_layers;
	dc_dev->num_submodules = dc_probed_info->num_submodules;
	if ((dc_probed_info->supported_link_types & BIT(DC_LINK_TYPE_SPLIT)) ||
		(dc_probed_info->supported_link_types & BIT(DC_LINK_TYPE_SIDE_BY_SIDE)))
		dc_dev->support_dual_link = true;
	else
		dc_dev->support_dual_link = false;

	if (dc_probed_info->supported_smmu_types & BIT(DC_SMMU_TYPE_STAGE1))
		dc_dev->support_smmu_stage_1 = true;
	else
		dc_dev->support_smmu_stage_1 = false;

	dc_dev->min_fw_layer_id = get_min_fw_layer_id(dc_probed_info);
	dc_dev->max_scaler_num = dc_probed_info->max_scaler_num;
	vdev->this_pipe->max_scaler_num = dc_probed_info->max_scaler_num;
	subdev_session = vdev->subdev_session;
	client_id = vdev->dev_info.client_id;

	while (i < dc_probed_info->num_submodules) {
		memset(&submodule_head, 0, sizeof(submodule_head));
		memset(&submodule_req, 0, sizeof(submodule_req));
		submodule_req.client_id = client_id;
		submodule_req.submodule_id = dc_probed_info->submodule_ids[i];
		err = bst_display_dc_cmd_probe_submodule(subdev_session,
			&submodule_req, &submodule_head);
		if (err) {
			DRM_ERROR("probe dc submodules failed.\n");
			goto err_cleanup;
		}

		if (submodule_req.submodule_id ==
			SUBMODULE_INFO_SUBMODULE_ID(submodule_head.submodule_info) &&
		    submodule_req.submodule_id < SUBMODULE_ID_DC_MAX) {
			err = virt_dc_init_submodule(dc_dev, &submodule_head);
			if (err)
				goto err_cleanup;
		}
		i++;
	}

	dc_dev->new_flush = false;
	dc_dev->timer_inited = false;
	dc_dev->base_dev->first_flush = true;

	return 0;

err_cleanup:
	virt_dc_cleanup(vdev);
	return err;
}

static int virt_dc_update(struct bst_virt_device *virt_dev, void *properties)
{
	return 0;
}

static void virt_dc_flush(struct bst_virt_device *virt_dev)
{
	struct virt_dc_dev *dc = virt_dev->virt_dev_data;
	struct bst_virt_device_info * info = &dc->base_dev->dev_info;
	struct bst_display_flush_cfg dc_flush = {
							.client_id = info->client_id,
							.test_mode = dc->test_mode,
							.is_trust  = false};
	uint32_t subdev_session = info->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	ret = bst_display_dc_cmd_do_flush(subdev_session, &dc_flush, &reply);
	if (!ret && reply.status == DISP_COMM_REPLAY_OK)
		DRM_DEBUG("dc do flush ok!!\n");
	else
		DRM_ERROR("dc do flush falied!!\n");
	dc->new_flush = true;
}

static void virt_dc_debug_dump(struct bst_virt_device *virt_dev,
		struct seq_file *sf)
{
	struct virt_dc_dev *dc = virt_dev->virt_dev_data;
	struct bst_virt_device_info * info = &dc->base_dev->dev_info;
	struct bst_display_dev_dump dump_cfg = { .type = DC_MONITOR, .client_id = info->client_id };
	uint32_t subdev_session = info->subdev_session;
	struct bst_display_comm_reply reply = { 0 };
	int ret;

	dump_cfg.type = DC_MONITOR;
	ret = bst_display_dc_cmd_dump_debug_info(subdev_session, &dump_cfg, &reply);
	if (ret) {
		DRM_ERROR("dc debug dump falied!!\n");
		goto done;
	}
done:
	return;
}
#ifndef __DISPLAY_EVENTS_MGR__
static enum hrtimer_restart virt_dc_vblank_simulate(struct hrtimer *timer)
{
	struct virt_dc_dev *dc_dev =
		container_of(timer, struct virt_dc_dev, vblank_hrtimer);
	int ret_overrun;
	struct bst_virt_events evts;

	/* Call into the CHIP to recognize events */
	memset(&evts, 0, sizeof(evts));
	if (dc_dev->new_flush) {
		evts.pipes |= BST_DRM_EVENT_FLIP;
		dc_dev->new_flush = false;
	}

	if (dc_dev->new_writeback) {
		evts.pipes |= BST_DRM_EVENT_EOW;
		dc_dev->new_writeback = false;
	}

	evts.pipes |= BST_DRM_EVENT_VSYNC;
	bst_crtc_handle_event(dc_dev->bcrtc, &evts);

	ret_overrun =
		hrtimer_forward_now(&dc_dev->vblank_hrtimer, dc_dev->framedur_ns);

	return HRTIMER_RESTART;
}

static void virt_dc_on_off_vblank(struct bst_virt_device *virt_dev, bool on,
				  struct bst_crtc *bcrtc)
{
	struct virt_dc_dev *dc_dev = virt_dev->virt_dev_data;
	struct drm_device *dev = bcrtc->base.dev;
	struct drm_vblank_crtc *vblank = &dev->vblank[drm_crtc_index(&bcrtc->base)];
	drm_calc_timestamping_constants(&bcrtc->base, &bcrtc->base.mode);

	dc_dev->bcrtc = bcrtc;
	if (on) {
		hrtimer_init(&dc_dev->vblank_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		dc_dev->vblank_hrtimer.function = &virt_dc_vblank_simulate;
		dc_dev->framedur_ns = ktime_set(0, vblank->framedur_ns);
		hrtimer_start(&dc_dev->vblank_hrtimer, dc_dev->framedur_ns, HRTIMER_MODE_REL);
	} else {
		hrtimer_cancel(&dc_dev->vblank_hrtimer);
		dc_dev->bcrtc = NULL;
	}
}

#else

static enum hrtimer_restart flip_event_simulate(struct hrtimer *timer)
{
	struct virt_dc_dev *dc_dev =
		container_of(timer, struct virt_dc_dev, flip_hrtimer);
	int ret_overrun;
	struct bst_virt_events evts;

	memset(&evts, 0, sizeof(evts));

	if (dc_dev->new_flush) {
		evts.pipes |= BST_DRM_EVENT_FLIP;
		bst_crtc_handle_event(dc_dev->bcrtc, &evts);
		dc_dev->new_flush = false;
	}

	ret_overrun =
		hrtimer_forward_now(&dc_dev->flip_hrtimer, dc_dev->framedur_ns);

	return HRTIMER_RESTART;
}

static void virt_dc_on_off_vblank(struct bst_virt_device *virt_dev,
	bool on, struct bst_crtc* bcrtc)
{
	struct virt_dc_dev *dc_dev = virt_dev->virt_dev_data;
	struct drm_device *dev = bcrtc->base.dev;
	struct drm_vblank_crtc *vblank = &dev->vblank[drm_crtc_index(&bcrtc->base)];
	drm_calc_timestamping_constants(&bcrtc->base, &bcrtc->base.mode);

	dc_dev->bcrtc = bcrtc;
	if (on) {
		virt_dev->events_mask &= ~(BST_EVENT_VSYNC | BST_EVENT_EOW);
		if (!dc_dev->timer_inited) {
			hrtimer_init(&dc_dev->flip_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			dc_dev->flip_hrtimer.function = &flip_event_simulate;
			dc_dev->framedur_ns =
				ktime_set(0, vblank->framedur_ns / BST_DRM_HW_FLUSH_DELAY_RAITO);
			hrtimer_start(&dc_dev->flip_hrtimer, dc_dev->framedur_ns, HRTIMER_MODE_REL);
			dc_dev->timer_inited = true;
			DRM_INFO("pipe-%d flip simulate timer started, framedur_ns=%lld\n",
				bcrtc->master->pipe_id, dc_dev->framedur_ns);
		}

	} else {
		virt_dev->events_mask |= (BST_EVENT_VSYNC | BST_EVENT_EOW);
		if (dc_dev->timer_inited) {
			hrtimer_cancel(&dc_dev->flip_hrtimer);
			dc_dev->bcrtc = NULL;
			dc_dev->timer_inited = false;
			DRM_INFO("pipe-%d flip simulate timer stopped\n", bcrtc->master->pipe_id);
		}
	}
}
#endif

static const struct bst_virt_device_funcs virt_dc_dev_funcs = {
	.probe = virt_dc_probe,
	.cleanup = virt_dc_cleanup,
	.irq_handler = virt_dc_irq_handler,
	.enable_irq = virt_dc_enable_irq,
	.disable_irq = virt_dc_disable_irq,
	.update = virt_dc_update,
	.flush = virt_dc_flush,
	.on_off_vblank = virt_dc_on_off_vblank,
	.debug_dump = virt_dc_debug_dump,
};

static const struct bst_virt_device_funcs *
virt_dc_identify(struct device *dev, struct bst_virt_platform_info *plat_info,
		 struct bst_virt_device_info *dev_info)
{
	struct bst_subdev_probe_request request;
	struct bst_subdev_probe_response response;
	struct bst_display_dc_probed_info *probed_info;
	int ret = 0;

	probed_info = devm_kzalloc(dev, sizeof(*probed_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(probed_info))
		return NULL;

	request.client_id = plat_info->client_id;
	request.platform_id = plat_info->platform_id;
	request.want_subdev = to_fw_subdev_type(plat_info->device_type);
	request.want_layer_num = plat_info->want_layer_num;

	request.want_info_size = sizeof(response) + sizeof(*probed_info);
	ret = bst_display_glb_cmd_probe_subdev(&request, &response);
	if (!ret && response.status == SUBDEV_PROBE_STATUS_OK) {
		memcpy(probed_info, &response.probed_info[0],
		       sizeof(*probed_info));
		dev_info->subdev_session = response.subdev_session;
		dev_info->is_owner_device = response.is_owner;
		dev_info->client_id = response.client_id;
		dev_info->arch_id = probed_info->arch_id;
		dev_info->bus_width = probed_info->bus_width;
		dev_info->private = probed_info;
		dev_info->device_type = plat_info->device_type;
		dev_info->platform_id = request.platform_id;
		return &virt_dc_dev_funcs;
	}

	return NULL;
}

void bst_virt_dc_destroy(struct bst_virt_device *vdev)
{
	const struct bst_virt_device_funcs *funcs = vdev->funcs;

	if (funcs && funcs->cleanup)
		funcs->cleanup(vdev);
}

struct bst_virt_device *
bst_virt_dc_create(struct device *dev, struct bst_virt_platform_info *plat_info,
		   struct bst_virt_pipe *pipe)
{
	struct bst_virt_device *vdev;
	int err = 0;

	vdev = devm_kzalloc(dev, sizeof(*vdev), GFP_KERNEL);
	if (!vdev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&vdev->lock);
	vdev->dev = dev;
	vdev->funcs = virt_dc_identify(dev, plat_info, &vdev->dev_info);
	if (!vdev->funcs) {
		DRM_ERROR("Failed to identify the HW.\n");
		err = -ENODEV;
		goto err_cleanup;
	}

	vdev->subdev_session = vdev->dev_info.subdev_session;
	vdev->device_type = plat_info->device_type;
	vdev->this_pipe = pipe;
	vdev->events_mask = BST_EVENT_VSYNC | BST_EVENT_FLIP | BST_EVENT_EOW;

	DRM_INFO("Found BST-DC-%x, Client ID:0x%x, Device Session:%x, Device Role:%s, device_type=%d\n",
		 vdev->dev_info.arch_id, vdev->dev_info.client_id,
		 vdev->dev_info.subdev_session,
		 vdev->dev_info.is_owner_device ? "is_owner" : "not_owner",
		 vdev->device_type);

	err = vdev->funcs->probe(vdev);
	if (err) {
		DRM_ERROR("enumerate display resource failed.\n");
		goto err_cleanup;
	}

	return vdev;

err_cleanup:
	bst_virt_dc_destroy(vdev);
	return NULL;
}
