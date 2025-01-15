/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_CSI_H__
#define __BST_CSI_H__

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "../../i2c/bst/common_deser_hub.h"
#include "cam_entity.h"

#define MAX_VC_PER_CSI 4

enum {
	CSI_CHANNEL_SINK_PAD = 0,
	CSI_CHANNEL_SOURCE_PAD = 1,
	CSI_CHANNEL_PAD_NUM,
};

enum {
	CSI_STATE_INVALID,
	CSI_STATE_INITED,
};

// csi channel as a media entity
struct bst_csi_channel {
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_entity entity;
#endif
	struct media_pad pads[CSI_CHANNEL_PAD_NUM];
	struct camera_dev *cam_dev;
	struct bst_csi_device *csi;
	bool connected;
	int state;
	int csi_dev_id;
	int index;
	int csi_chn_id;
	int sn_in_all_csi;
	atomic_t is_streaming;
};

struct bst_mipi_config {
	unsigned int reg_val;
	unsigned int reg_val_offsetcal_wait_thresh;
	unsigned int reg_val_lpdcocal_timebase;
	unsigned int reg_val_twait_coarse_fine;
	unsigned int reg_val_ddlcal_timebase_target;
	unsigned int reg_val_ddlcal_start_delay;
	unsigned int reg_val_ddlcal_counter_ref;
	unsigned int max_phase;
	unsigned int reg_val_ddlcal_dll_fbk;
	unsigned int reg_val_ddlcal_ddl_coarse_bank;
	unsigned int reg_val_lanex_hsrx_cdphy_sel_fast;
	unsigned int reg_val_lanex_hsrx_hs_clk_div;
	unsigned int reg_val_hs_rx_thssettle;
	unsigned int reg_val_hs_rx_fjump_deskew;
	unsigned int reg_val_hs_rx_min_eye_opening_deskew;

	unsigned int reg_val_coarse_target_reg;
	unsigned int reg_val_delay_deass_thresh_reg;
	unsigned int reg_val_det_dly_thresh_val;
	unsigned int reg_val_post_rcvd_rst_val_thresh_reg;
	unsigned int phase_bound_reg;
	unsigned long reg_val_hsdcocal_nref;
	unsigned long reg_val_hsdcocal_nref_range;
};

struct bst_csi_device {
	struct device *dev;
	struct platform_device *pdev;
	struct deser_hub_dev *deser;
	char devname[32];
	struct v4l2_subdev subdev;
	struct v4l2_async_subdev async_dev;
	struct v4l2_async_notifier notifier;
	struct device_node *of_node;
	struct fwnode_handle *csi_fwnode;
	struct fwnode_handle *remote_fwnode;

	struct bst_mipi_config mipi_cfg;
	struct bst_csi_channel csi_vc[MAX_VC_PER_CSI];
	// struct fwnode_handle *cam_fwnode[MAX_VC_PER_CSI];
	struct mutex mutex; /* format and crop settings */
	int state;
	int sd_state;
	int csi_id;
	int lane_speed;
	int num_vc; // number of virtual channel
	int num_lanes;
	void __iomem *ctrl_base;
	void __iomem *top_base;
	int function_irq;
	int fmeda_irq;
	unsigned int error_count;
	unsigned long error_ts; /* error occured by jiffies */
	unsigned int recover_times;
	enum csi_phy_mode phy_mode_cfg;
	atomic_t refcount;
};

int update_camera_status_in_csi(struct bst_csi_device *pcsi_dev);

#endif // __BST_CSI_H__
