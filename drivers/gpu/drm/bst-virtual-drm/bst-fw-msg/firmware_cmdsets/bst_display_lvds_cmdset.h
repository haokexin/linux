// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef _BST_DISPLAY_LVDS_CMDSET_H_
#define _BST_DISPLAY_LVDS_CMDSET_H_

#include "bst_display_global_api.h"

enum lvds_cmdid {
	LVDS_CMD_INVALED = 0x00,
	LVDS_CMD_RESET,
	LVDS_CMD_GET_EVENTS,
	LVDS_CMD_PROBE_SUBMODULE,
	LVDS_CMD_GET_INFO,
	LVDS_CMD_SET_VIDEO_STREAM,
	LVDS_CMD_DISABLE_SUBMODULE,
	LVDS_CMD_DUMP_DEBUG_INFO
};

struct bst_display_lvds_req{
	uint32_t client_id;
	uint16_t refresh_rate;
	uint8_t display_protocol;
	uint8_t video_id;

};

struct bst_display_lvds_info {
	uint32_t client_id;
	uint32_t supported_color_formats;
	uint32_t supported_color_depths;
	bool connected;
};

struct bst_display_lvds_set_video_req {
	uint32_t client_id;
	uint8_t enable;
	uint8_t color_mapping; /* LVDS_VESA_30 ~ LVDS_JEIDA_24*/
	struct video_timing timing;
};


int bst_display_lvds_cmd_probe_submodule(
				uint32_t lvds_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head);
int bst_display_lvds_cmd_get_info(
				uint32_t lvds_session,
				struct bst_display_lvds_req *req,
				struct bst_display_lvds_info *info);
int bst_display_lvds_cmd_set_video_stream(uint32_t lvds_session,
				struct bst_display_lvds_set_video_req *req,
				struct bst_display_comm_reply* reply);
int bst_display_lvds_cmd_dump_debug_info(uint32_t lvds_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply);
int bst_display_lvds_cmd_disable_submodule(uint32_t lvds_session,
				struct bst_display_submodule_disable *submodule_dis,
				struct bst_display_comm_reply* reply);
#endif /* _BST_DISPLAY_LVDS_CMDSET_H_ */
