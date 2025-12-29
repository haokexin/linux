// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <drm/drm_print.h>
#include <linux/types.h>
#include <linux/slab.h>
#include "virt_lvds_dev.h"

static void virt_lvds_cleanup(struct bst_virt_device *vdev)
{
	struct virt_lvds_dev *lvds = vdev->virt_dev_data;

	if (!lvds)
		return;

	vdev->virt_dev_data = NULL;
}

static int virt_lvds_probe(struct bst_virt_device *vdev)
{
	struct virt_lvds_dev *lvds_dev;
	struct bst_display_submodule_header submodule_head = { 0 };
	struct bst_display_submodule_req submodule_req = { 0 };
	struct bst_display_lvds_probed_info *lvds_probed_info = vdev->dev_info.private;
	u32 i = 0, subdev_session;
	int err;

	lvds_dev = devm_kzalloc(vdev->dev, sizeof(*lvds_dev), GFP_KERNEL);
	if (!lvds_dev)
		return -ENOMEM;
	vdev->virt_dev_data = lvds_dev;
	lvds_dev->base_dev = vdev;
	subdev_session = vdev->subdev_session;

	if (lvds_probed_info->num_submodules > SUBMODULE_ID_LVDS_MAX)
		return -EINVAL;

	while (i < lvds_probed_info->num_submodules) {
		memset(&submodule_head, 0, sizeof(submodule_head));
		memset(&submodule_req, 0, sizeof(submodule_req));
		submodule_req.submodule_id = lvds_probed_info->submodule_ids[i];
		err = bst_display_conn_cmd_probe_submodule(subdev_session, &submodule_req, &submodule_head);
		if (err) {
			DRM_ERROR("probe lvds submodules failed.\n");
			goto err_cleanup;
		}
		if (submodule_req.submodule_id ==
			SUBMODULE_INFO_SUBMODULE_ID(submodule_head.submodule_info)) {
			err = virt_lvds_init_submodule(lvds_dev, &submodule_head);
			if (err)
				goto err_cleanup;
		}
		i++;
	}
	return 0;

err_cleanup:
	virt_lvds_cleanup(vdev);
	return err;
}

static int virt_lvds_update(struct bst_virt_device *virt_dev, void *properties)
{
	return 0;
}

static void virt_lvds_flush(struct bst_virt_device *virt_dev)
{
}

static const struct bst_virt_device_funcs virt_lvds_dev_funcs = {
	.probe = virt_lvds_probe,
	.cleanup = virt_lvds_cleanup,
	.update = virt_lvds_update,
	.flush = virt_lvds_flush,
};

static uint32_t to_fw_lvds_device_type(uint32_t virt_drm_dev_type)
{
	uint32_t fw_lvds_dev_type;

	switch (virt_drm_dev_type) {
	case DEVICE_TYPE_VIRT_LVDS0:
		fw_lvds_dev_type = BST_SUBDEV_LVDS0;
		break;
	case DEVICE_TYPE_VIRT_LVDS1:
		fw_lvds_dev_type = BST_SUBDEV_LVDS1;
		break;
	default:
		fw_lvds_dev_type = BST_SUBDEV_INVAL;
	}

	return fw_lvds_dev_type;
}

static const struct bst_virt_device_funcs *
virt_lvds_identify(struct device *dev, struct bst_virt_platform_info *plat_info,
		   struct bst_virt_device_info *dev_info)
{
	struct bst_subdev_probe_request request;
	struct bst_subdev_probe_response response;
	struct bst_display_lvds_probed_info *probed_info;
	int ret = 0;

	if (sizeof(*probed_info) > SUBDEV_PROBE_INFO_MAX_SIZE) {
		DRM_ERROR("err! lvds probe info size[%d] > max_size[%d].\n",
			(int32_t)sizeof(*probed_info), (int32_t)SUBDEV_PROBE_INFO_MAX_SIZE);
		return NULL;
	}

	probed_info = devm_kzalloc(dev, sizeof(*probed_info), GFP_KERNEL);
	if(IS_ERR_OR_NULL(probed_info))
		return NULL;

	request.want_subdev = to_fw_lvds_device_type(plat_info->device_type);
	request.want_info_size = sizeof(response);
	ret = bst_display_glb_cmd_probe_subdev(&request, &response);
	if (!ret && response.base.status == DISP_COMM_REPLAY_OK) {
		memcpy(probed_info, &response.probed_info[0],
		       sizeof(*probed_info));
		dev_info->subdev_session = response.subdev_session;
		dev_info->is_owner_device = response.is_owner;
		dev_info->arch_id = probed_info->arch_id;
		dev_info->private = probed_info;
		dev_info->device_type = plat_info->device_type;
		//if (dev_info->is_owner_device)
			return &virt_lvds_dev_funcs;
		//else
		//	return &virt_shared_conn_dev_funcs;
	}

	DRM_ERROR("virt_lvds_identify return null!\n");
	return NULL;
}

void bst_virt_lvds_destroy(struct bst_virt_device *vdev)
{
	const struct bst_virt_device_funcs *funcs = vdev->funcs;

	if (funcs && funcs->cleanup)
		funcs->cleanup(vdev);
}

struct bst_virt_device *
bst_virt_lvds_create(struct device *dev,
		     struct bst_virt_platform_info *plat_info,
		     struct bst_virt_pipe *pipe)
{
	struct bst_virt_device *vdev;
	int err = 0;

	vdev = devm_kzalloc(dev, sizeof(*vdev), GFP_KERNEL);
	if (!vdev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&vdev->lock);
	vdev->funcs = virt_lvds_identify(dev, plat_info, &vdev->dev_info);
	if (!vdev->funcs) {
		DRM_ERROR("Failed to identify lvds the HW.\n");
		err = -ENODEV;
		goto err_cleanup;
	}
	vdev->dev = dev;
	vdev->subdev_session = vdev->dev_info.subdev_session;
	vdev->device_type = plat_info->device_type;
	vdev->this_pipe = pipe;

	DRM_INFO("Found BST-LVDS-%x, Device Session:%x, Device Role:%s\n",
		 vdev->dev_info.arch_id,
		 vdev->dev_info.subdev_session,
		 vdev->dev_info.is_owner_device ? "is_owner" : "not_owner");

	err = vdev->funcs->probe(vdev);
	if (err) {
		DRM_ERROR("enumerate lvds resource failed.\n");
		goto err_cleanup;
	}
	return vdev;

err_cleanup:
	bst_virt_lvds_destroy(vdev);
	return ERR_PTR(err);
}
