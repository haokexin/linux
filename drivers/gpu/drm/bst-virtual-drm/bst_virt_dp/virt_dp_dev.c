// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#define BST_VIRT_TAG "virt-dp-dev"

#include "bst_display_global_api.h"
#include "bst_virt_pipeline.h"
#include <drm/drm_print.h>
#include <linux/types.h>
#include <linux/slab.h>
#include "virt_dp_dev.h"
#include <drm/drm_print.h>

static uint32_t to_fw_dp_device_type(uint32_t virt_drm_dev_type)
{
	uint32_t fw_dp_dev_type;

	switch (virt_drm_dev_type) {
	case DEVICE_TYPE_VIRT_DP:
		fw_dp_dev_type = BST_SUBDEV_eDP;
		break;
	default:
		fw_dp_dev_type = BST_SUBDEV_INVAL;
	}

	return fw_dp_dev_type;
}

static irqreturn_t
virt_dp_irq_handler(struct bst_virt_device *vdev,
		    const struct bst_display_events_status status)
{
	struct virt_dp_dev *dp = vdev->virt_dev_data;
	struct bst_virt_connector *v_conn = vdev->this_pipe->master_conn;
	struct bst_virt_events evts;
	u32 global_status = 0;
	u32 events_type = status.events & ~vdev->events_mask;

	memset(&evts, 0, sizeof(evts));

	if (events_type) {
		DRM_INFO("virt dev(%d) evts:%#x\n", vdev->device_type,
			 events_type);
		if (events_type & BST_EVENT_HOTPLUG)
			atomic_set(&v_conn->connected, 1);
		if (events_type & BST_EVENT_UNHOTPLUG) {
			atomic_set(&v_conn->connected, 0);
			dp->trained = false;
		}
	}

	return IRQ_RETVAL(global_status);
}
void virt_dp_events_handler(const struct bst_display_events_status status,
			    void *ext)
{
	struct bst_virt_device *virt_dev = (struct bst_virt_device *)ext;
	virt_dev->funcs->irq_handler(virt_dev, status);
}

static int virt_dp_enable_irq(struct bst_virt_device *vdev)
{
	int ret;
	ret = bst_display_glb_cmd_subscribe_events(
		to_fw_dp_device_type(vdev->device_type), virt_dp_events_handler,
		vdev);
	if (ret)
		DRM_ERROR("VIRT DP Enable irq, ret=%d!\n", ret);
	return 0;
}

static int virt_dp_disable_irq(struct bst_virt_device *vdev)
{
	return bst_display_glb_cmd_unsubscribe_events(
		to_fw_dp_device_type(vdev->device_type));
}

static void virt_dp_cleanup(struct bst_virt_device *vdev)
{
	struct virt_dp_dev *dp = vdev->virt_dev_data;

	if (!dp)
		return;
	vdev->virt_dev_data = NULL;
}

static int virt_dp_probe(struct bst_virt_device *vdev)
{
	struct virt_dp_dev *dp_dev;
	struct bst_display_submodule_header submodule_head = { 0 };
	struct bst_display_submodule_req submodule_req = { 0 };
	struct bst_display_dp_probed_info *dp_probed_info = vdev->dev_info.private;
	u32 i = 0, subdev_session;
	int err;

	dp_dev = devm_kzalloc(vdev->dev, sizeof(*dp_dev), GFP_KERNEL);
	if (!dp_dev)
		return -ENOMEM;

	vdev->virt_dev_data = dp_dev;

	dp_dev->base_dev = vdev;
	subdev_session = vdev->subdev_session;

	if (dp_probed_info->num_submodules > SUBMODULE_ID_DP_MAX)
		return -EINVAL;

	while (i < dp_probed_info->num_submodules) {
		memset(&submodule_head, 0, sizeof(submodule_head));
		memset(&submodule_req, 0, sizeof(submodule_req));
		submodule_req.submodule_id = dp_probed_info->submodule_ids[i];
		err = bst_display_conn_cmd_probe_submodule(subdev_session, &submodule_req, &submodule_head);
		if (err) {
			DRM_ERROR("probe dp submodules failed.\n");
			goto err_cleanup;
		}
		if (submodule_req.submodule_id ==
			    SUBMODULE_INFO_SUBMODULE_ID(submodule_head.submodule_info) &&
		    submodule_req.submodule_id < SUBMODULE_ID_DP_MAX) {
			err = virt_dp_init_submodule(dp_dev, &submodule_head);
			if (err)
				goto err_cleanup;
		}
		i++;
	}

	return 0;

err_cleanup:
	virt_dp_cleanup(vdev);
	return err;
}

static int virt_dp_update(struct bst_virt_device *virt_dev, void *properties)
{
	return 0;
}

static void virt_dp_flush(struct bst_virt_device *virt_dev)
{
}

static const struct bst_virt_device_funcs virt_dp_dev_funcs = {
	.probe = virt_dp_probe,
	.cleanup = virt_dp_cleanup,
	.irq_handler = virt_dp_irq_handler,
	.enable_irq = virt_dp_enable_irq,
	.disable_irq = virt_dp_disable_irq,
	.update = virt_dp_update,
	.flush = virt_dp_flush,
};

static const struct bst_virt_device_funcs *
virt_dp_identify(struct device *dev, struct bst_virt_platform_info *plat_info,
		 struct bst_virt_device_info *dev_info)
{
	struct bst_subdev_probe_request request;
	struct bst_subdev_probe_response response;
	struct bst_display_dp_probed_info *probed_info;
	int ret = 0;

	if (sizeof(*probed_info) > SUBDEV_PROBE_INFO_MAX_SIZE) {
		DRM_ERROR("err! dp probe info size[%d] > max_size[%d].\n",
			(int32_t)sizeof(*probed_info), (int32_t)SUBDEV_PROBE_INFO_MAX_SIZE);
		return NULL;
	}

	probed_info = devm_kzalloc(dev, sizeof(*probed_info), GFP_KERNEL);
	if (!probed_info)
		return NULL;

	request.want_subdev = to_fw_dp_device_type(plat_info->device_type);
	request.want_info_size = sizeof(response);
	ret = bst_display_glb_cmd_probe_subdev(&request, &response);
	if (!ret && response.base.status == DISP_COMM_REPLAY_OK) {
		memcpy(probed_info, &response.probed_info[0], sizeof(*probed_info));
		dev_info->subdev_session = response.subdev_session;
		dev_info->is_owner_device = response.is_owner;
		dev_info->arch_id = probed_info->arch_id;
		dev_info->private = probed_info;
		dev_info->device_type = plat_info->device_type;
		//if (dev_info->is_owner_device)
			return &virt_dp_dev_funcs;
		//else
		//	return &virt_shared_conn_dev_funcs;
	}
	return NULL;
}

void bst_virt_dp_destroy(struct bst_virt_device *vdev)
{
	const struct bst_virt_device_funcs *funcs = vdev->funcs;

	if (funcs && funcs->cleanup)
		funcs->cleanup(vdev);
}

struct bst_virt_device *
bst_virt_dp_create(struct device *dev, struct bst_virt_platform_info *plat_info,
		   struct bst_virt_pipe *pipe)
{
	struct bst_virt_device *vdev;
	int err = 0;

	vdev = devm_kzalloc(dev, sizeof(*vdev), GFP_KERNEL);
	if (!vdev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&vdev->lock);
	vdev->funcs = virt_dp_identify(dev, plat_info, &vdev->dev_info);
	if (!vdev->funcs) {
		DRM_ERROR("Failed to identify the HW.\n");
		err = -ENODEV;
		goto err_cleanup;
	}
	vdev->dev = dev;
	vdev->subdev_session = vdev->dev_info.subdev_session;
	vdev->device_type = plat_info->device_type;
	vdev->this_pipe = pipe;

	DRM_INFO("Found BST-eDP-%x, Device Session:%x, Device Role:%s\n",
		 vdev->dev_info.arch_id,
		 vdev->dev_info.subdev_session,
		 vdev->dev_info.is_owner_device ? "is_owner" : "not_owner");

	err = vdev->funcs->probe(vdev);
	if (err) {
		DRM_ERROR("enumerate display resource failed.\n");
		goto err_cleanup;
	}

	return vdev;

err_cleanup:
	bst_virt_dp_destroy(vdev);
	return ERR_PTR(err);
}
