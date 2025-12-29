// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_CONN_API_H
#define BST_DISPLAY_CONN_API_H

#include "bst_display_cmdset_api.h"
#include "bst_display_global_api.h"
#include "bst_display_platform.h"

#define DPTX_PHYIF_CTRL_RATE_RBR  0x0
#define DPTX_PHYIF_CTRL_RATE_HBR  0x1
#define DPTX_PHYIF_CTRL_RATE_HBR2 0x2
#define DPTX_PHYIF_CTRL_RATE_HBR3 0x3

#define DPTX_LANE_SPEED_RBR    1620
#define DPTX_LANE_SPEED_HBR    2700
#define DPTX_LANE_SPEED_HBR2   5400
#define DPTX_LANE_SPEED_HBR3   8100

enum connctor_cmdid {
	CONN_CMD_INVALED = 0x00,
	CONN_CMD_PROBE_SUBMODULE,
	CONN_CMD_GET_SUBMODULE_INFO,
	CONN_CMD_GET_EDID,
	CONN_CMD_DO_LINK_TRAINING,
	CONN_CMD_DO_LINK_CONNECT,
	CONN_CMD_SET_VIDEO_STREAM,
	CONN_CMD_GET_CUR_VM,
	CONN_CMD_DISABLE_SUBMODULE,
	//CONN_CMD_DUMP_DEBUG_INFO,  //todo
};

enum {
	EDID_BLOCK_TOP,
	EDID_BLOCK_BOTTOM,
	EDID_EXT_BLOCK1_TOP,
	EDID_EXT_BLOCK1_BOTTOM,
	EDID_MAX_BLOCK_NUM,
	//EDID_EXT_BLOCK2_TOP,
	//EDID_EXT_BLOCK2_BOTTOM,
};

#define DEFAULT_EDID_BUFLEN			128U
#define MAX_EDID_BUF_NUM			((uint32_t)EDID_MAX_BLOCK_NUM / 2)
#define EDID_BLOCK_BUFLEN			64U

struct bst_display_edid_req {
	uint8_t type;
};

struct bst_display_edid_info {
	reply_base base;
	uint8_t edid[EDID_BLOCK_BUFLEN];
};

struct bst_display_video_subm_info {
	uint32_t supported_color_formats;
	uint32_t supported_color_depths;
	bool connected;
	bool trained;
	//struct video_timing timing;
};

struct bst_display_submodule_info {
	reply_base base;
	uint32_t submodule_type;
	union submodule_info {
		struct bst_display_video_subm_info video_info;
	} info;
};

struct edp_ext_video_param {
	uint8_t lanes;
	uint8_t video_format;
	uint8_t bpc;
	uint8_t colorimetry;
	uint8_t dynamic_range;
};

struct lvds_ext_video_param {
	uint8_t color_mapping; /* LVDS_VESA_30 ~ LVDS_JEIDA_24*/
};

struct mipi_ext_video_param {
	uint8_t reserve; /* LVDS_VESA_30 ~ LVDS_JEIDA_24*/
};

struct bst_display_set_video_stream_req {
	uint8_t enable;
	uint8_t enable_backlight;
	struct video_timing timing;

	union {
		struct edp_ext_video_param edp_param;
		struct lvds_ext_video_param lvds_param;
		struct mipi_ext_video_param mipi_param;
	} ext_info;
};

enum {
	DP_LINK_TRAINING,
	DP_FAST_LINK_TRAINING
};

enum {
	DP_IF,
	EDP_IF
};

struct bst_display_training_req {
	uint8_t video_format;
	uint8_t train_type;
	uint8_t rate;
	uint8_t lanes;
	uint8_t bpc;
	uint8_t colorimetry;
	uint8_t dynamic_range;
	struct video_timing timing;
};

struct bst_display_training_status {
	reply_base base;
	uint8_t rate;
	uint8_t lanes;
	uint8_t trained;
};

struct bst_display_connect_req {
	uint8_t connect;
	uint8_t lanes;
};

struct bst_display_connect_status {
	reply_base client_id;
	bool connected;
};

struct bst_display_vm_setting {
	reply_base base;
	uint16_t refresh_rate;
	struct video_timing timing;
};

struct bst_display_vm_req {
	uint8_t video_timing_id;
};

extern int bst_display_conn_cmd_probe_submodule(uint32_t conn_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head);
extern int bst_display_conn_cmd_get_submodule_info(uint32_t conn_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_info *submodule_info);
extern int bst_display_conn_cmd_get_edid(uint32_t conn_session,
				struct bst_display_edid_req *edid,
				struct bst_display_edid_info *info);
extern int bst_display_conn_cmd_link_training(uint32_t conn_session,
				struct bst_display_training_req *req,
				struct bst_display_training_status *train);
//extern int bst_display_conn_cmd_link_connect(uint32_t dp_session,
//				struct bst_display_connect_req *req,
//				struct bst_display_connect_status *status)
extern int bst_display_conn_cmd_set_video_stream(uint32_t conn_session,
				struct bst_display_set_video_stream_req *req,
				struct bst_display_comm_reply* reply);
extern int bst_display_conn_cmd_get_cur_video_mode(uint32_t conn_session,
				struct bst_display_vm_req *req,
				struct bst_display_vm_setting *info);
extern int bst_display_conn_cmd_disable_submodule(uint32_t lvds_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_comm_reply* reply);

#endif /* BST_DISPLAY_CONN_API_H */
