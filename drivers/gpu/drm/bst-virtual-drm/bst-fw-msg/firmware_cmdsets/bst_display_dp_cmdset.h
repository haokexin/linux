// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _BST_DISPLAY_DP_CMDSET_H_
#define _BST_DISPLAY_DP_CMDSET_H_

#include "bst_display_global_api.h"

#define DPTX_PHYIF_CTRL_RATE_RBR  0x0
#define DPTX_PHYIF_CTRL_RATE_HBR  0x1
#define DPTX_PHYIF_CTRL_RATE_HBR2 0x2
#define DPTX_PHYIF_CTRL_RATE_HBR3 0x3

#define DPTX_LANE_SPEED_RBR    1620
#define DPTX_LANE_SPEED_HBR    2700
#define DPTX_LANE_SPEED_HBR2   5400
#define DPTX_LANE_SPEED_HBR3   8100

enum dp_cmdid {
	DP_CMD_INVALED = 0x00,
	DP_CMD_PROBE_SUBMODULE,
	DP_CMD_GET_INFO,
	DP_CMD_SET_VIDEO_STREAM,
	DP_CMD_DO_LINK_TRAINING,
	DP_CMD_DO_LINK_CONNECT,
	DP_CMD_DISABLE_SUBMODULE,
	DP_CMD_DUMP_DEBUG_INFO
};

enum {
	DP_LINK_TRAINING,
	DP_FAST_LINK_TRAINING
};

enum {
	DP_IF,
	EDP_IF
};

struct bst_display_dp_training_req {
	uint32_t client_id;
	uint8_t video_format;
	uint8_t train_type;
	uint8_t rate;
	uint8_t lanes;
	uint8_t bpc;
	uint8_t colorimetry;
	uint8_t dynamic_range;
	struct video_timing timing;
};

struct bst_display_dp_training_status {
	uint32_t client_id;
	uint8_t rate;
	uint8_t lanes;
	uint8_t trained;
};

struct bst_display_dp_req {
	uint32_t client_id;
};

struct bst_display_dp_info {
	uint32_t client_id;
	uint32_t supported_color_formats;
	uint32_t supported_color_depths;
	uint8_t connected;
	bool trained;
	struct video_timing timing;
};

struct bst_display_dp_connect_req {
	uint32_t client_id;
	uint8_t connect;
	uint8_t lanes;
};

struct bst_display_dp_connect_status {
	uint32_t client_id;
	uint8_t connected;
};

struct bst_display_dp_set_video_req {
	uint32_t client_id;
	uint8_t enable;
	uint8_t lanes;
	uint8_t video_format;
	uint8_t bpc;
	uint8_t colorimetry;
	uint8_t dynamic_range;
	struct video_timing timing;
};

struct bst_display_dp_set_video_status {
	uint32_t client_id;
	uint8_t result; /* !0:Pass, 0:Failed */
};

int bst_display_dp_cmd_dump_debug_info(uint32_t dp_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_dp_cmd_disable_submodule(uint32_t dp_session,
				struct bst_display_submodule_disable *submodule_dis);
int bst_display_dp_cmd_probe_submodule(uint32_t dp_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_header);
int bst_display_dp_cmd_get_info(uint32_t dp_session,
				struct bst_display_dp_req *req,
				struct bst_display_dp_info *info);
int bst_display_dp_cmd_do_link_connect(uint32_t dp_session,
				struct bst_display_dp_connect_req *req,
				struct bst_display_dp_connect_status *status);
int bst_display_dp_cmd_set_video_stream(uint32_t dp_session,
				struct bst_display_dp_set_video_req *req,
				struct bst_display_dp_set_video_status *status);
int bst_display_dp_cmd_do_link_training(uint32_t dp_session,
				struct bst_display_dp_training_req *req,
				struct bst_display_dp_training_status *train);

#endif /* _BST_DISPLAY_DP_CMDSET_H_ */
