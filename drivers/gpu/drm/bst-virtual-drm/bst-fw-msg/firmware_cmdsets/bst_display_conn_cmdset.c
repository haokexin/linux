// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_conn_cmdset.h"

int bst_display_conn_cmd_get_edid(uint32_t conn_session,
				struct bst_display_edid_req *edid,
				struct bst_display_edid_info *info)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_GET_EDID,
			edid, sizeof(*edid), info, sizeof(*info));
	return ret;
}

int bst_display_conn_cmd_probe_submodule(uint32_t conn_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_PROBE_SUBMODULE,
			submodule_req, sizeof(*submodule_req), submodule_head, sizeof(*submodule_head));
	return ret;
}

int bst_display_conn_cmd_get_submodule_info(uint32_t conn_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_info *submodule_info)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_GET_SUBMODULE_INFO,
			submodule_req, sizeof(*submodule_req), submodule_info, sizeof(*submodule_info));
	return ret;
}


int bst_display_conn_cmd_set_video_stream(uint32_t conn_session,
				   struct bst_display_set_video_stream_req *req,
				   struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_SET_VIDEO_STREAM,
			req, sizeof(*req), reply, sizeof(*reply));
	return ret;
}

int bst_display_conn_cmd_disable_submodule(uint32_t conn_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_DISABLE_SUBMODULE,
		submodule_req, sizeof(*submodule_req), reply, sizeof(*reply));
	return ret;
}

int bst_display_conn_cmd_link_training(uint32_t conn_session,
				struct bst_display_training_req *req,
				struct bst_display_training_status *train)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_DO_LINK_TRAINING,
			req, sizeof(*req), train, sizeof(*train));
	return ret;
}

int bst_display_conn_cmd_get_cur_video_mode(uint32_t conn_session,
				struct bst_display_vm_req *req,
				struct bst_display_vm_setting *info)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, (int)BST_DISPLAY_CONN_SUBDEV, (int)CONN_CMD_GET_CUR_VM,
			req, sizeof(*req), info, sizeof(*info));
	return ret;
}

/*
int bst_display_conn_cmd_link_connect(uint32_t dp_session,
				struct bst_display_connect_req *req,
				struct bst_display_connect_status *status)
{
	int ret;

	ret = bst_display_do_cmd(dp_session, BST_DISPLAY_CONN_SUBDEV, CONN_CMD_DO_LINK_CONNECT,
			req, sizeof(*req), status, sizeof(*status));
	return ret;
}

int bst_display_conn_cmd_dump_debug_info(uint32_t conn_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(conn_session, BST_DISPLAY_CONN_SUBDEV, CONN_CMD_DUMP_DEBUG_INFO,
			dump_cfg, sizeof(*dump_cfg), reply, sizeof(*reply));
	return ret;
}
*/
