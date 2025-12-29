/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_RX_H__
#define __BST_CSI_RX_H__

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

#include <bst/media-dev.h>
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
#include <linux/bst_samphore.h>
#endif

#include "csi_hw.h"
#include "csi_safety.h"

#define CSI_HW_INIT_RETRIES (3)
#define CSI_HW_LOCK_TIMEOUT (10)

struct csi_channel {
	struct camera_dev *cam_dev;
	struct csi_device *csi_dev;
	int vc;
};

struct csi_device {
	/* Common */
	struct device *dev;
	struct platform_device *pdev;
	struct mutex lock;
	struct csi_channel channels[MAX_VC_PER_CSI];
	int used_vcs;
	bool inited;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	u32 sem_master;
	u32 sem_bank;
	u32 sem_id;
	struct bst_samphore *hwlock;
	u32 uid;
#endif

	/* Properties */
	u32 id;
	u32 phy_if;
	u32 lane_num;
	u32 lane_speed;
	u32 eq;
	bool recoverable;
	u32 recover_threshold;
	u32 recover_window;

	/* As a subdev */
	struct v4l2_subdev subdev;
	struct fwnode_handle *fwnode;

	/* As an endpoint to connect remote */
	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev async_dev;
	struct v4l2_subdev *remote_sd;
	struct fwnode_handle *remote_fwnode;
	struct csi_tx_dev *tx_dev;

	/* Hardware */
	struct reset_control *rstc;
	void __iomem *ctrl_base;
	void __iomem *top_base;
	bool func_irq_enable;
	int func_irq;
	bool diag_irq_enable;
	int diag_irq;

	/* Safety */
	u32 recover_count;
	u32 error_total;
	u32 error_window;
	ktime_t error_start;
	union psm psm; /* Primary Safety Mechanism */
	int host_access_retries;
	int phy_access_retries;
};

#define subdev_to_csi_device(sd)   container_of(sd, struct csi_device, subdev)
#define notifier_to_csi_device(nf) container_of(nf, struct csi_device, notifier)
#define async_dev_to_csi_device(asd) \
	container_of(asd, struct csi_device, async_dev)

void csi_update_camera_status(struct csi_device *csi);

#endif /* __BST_CSI_RX_H__ */
