// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_dp_cmdset.h"

static void build_command(struct fw_msg_data *msg_data,int cmdid,
				uint32_t client_id, uint32_t subdev_session,
				uint32_t *usr_param, uint32_t size_param)
{
	msg_data->client_id = client_id;
	msg_data->cmdset = BST_DISPLAY_DP_SUBDEV;
	msg_data->cmdid = cmdid;
	msg_data->size_cmd = size_param;
	memcpy(&msg_data->user_cmd_data[0], usr_param, size_param);
	msg_data->subdev_session = subdev_session;
	msg_data->sync_mode = CMD_SYNC_MDDE;
	memset(&msg_data->user_ack_data[0], 0, sizeof(msg_data->user_ack_data));
}

static int build_reply(struct fw_msg_data *msg_data,
				uint32_t *usr_ack,
				uint32_t size_ack)
{
	uint32_t reply_client = msg_data->user_ack_data[0];
	uint32_t cmd_client = msg_data->client_id;
	if (reply_client == cmd_client)
		memcpy(usr_ack, &msg_data->user_ack_data[0], size_ack);
	else {
		DISP_ERR("build_reply failed!! (reply_client=0x%x != cmd_client = 0x%x)",
			reply_client,
			cmd_client);
		return -1;

	}

	return 0;
}

int bst_display_dp_cmd_probe_submodule(uint32_t dp_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head)
{
	struct fw_msg_data msg_data;
	uint32_t submodule_head_size = sizeof(*submodule_head);

	if (submodule_head_size > MAX_USR_DATA)
		return -1;

	build_command(&msg_data, DP_CMD_PROBE_SUBMODULE, submodule_req->client_id,
		      dp_session, (uint32_t *)submodule_req, sizeof(*submodule_req));
	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)submodule_head, submodule_head_size);
}

int bst_display_dp_cmd_disable_submodule(uint32_t dp_session,
				struct bst_display_submodule_disable *submodule_dis)
{
	struct fw_msg_data msg_data = { 0 };

	build_command(&msg_data, DP_CMD_DISABLE_SUBMODULE, submodule_dis->client_id,
		      dp_session, (uint32_t *)submodule_dis, sizeof(*submodule_dis));

	transfer_fw_msg(&msg_data);

	return 0;
}

int bst_display_dp_cmd_get_info(uint32_t dp_session,
				struct bst_display_dp_req *req,
				struct bst_display_dp_info *info)
{
	struct fw_msg_data msg_data;

	build_command(&msg_data, DP_CMD_GET_INFO, req->client_id,
		      dp_session, (uint32_t *)req, sizeof(*req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)info, sizeof(*info));
}

int bst_display_dp_cmd_do_link_training(uint32_t dp_session,
				struct bst_display_dp_training_req *req,
				struct bst_display_dp_training_status *train)
{
	struct fw_msg_data msg_data;

	build_command(&msg_data, DP_CMD_DO_LINK_TRAINING, req->client_id,
		      dp_session, (uint32_t *)req, sizeof(*req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)train, sizeof(*train));
}

int bst_display_dp_cmd_do_link_connect(uint32_t dp_session,
				struct bst_display_dp_connect_req *req,
				struct bst_display_dp_connect_status *status)
{
	struct fw_msg_data msg_data;

	build_command(&msg_data, DP_CMD_DO_LINK_CONNECT, req->client_id, dp_session,
		      (uint32_t *)req, sizeof(*req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)status, sizeof(*status));
}

int bst_display_dp_cmd_set_video_stream(uint32_t dp_session,
				struct bst_display_dp_set_video_req *req,
				struct bst_display_dp_set_video_status *status)

{
	struct fw_msg_data msg_data;

	build_command(&msg_data, DP_CMD_SET_VIDEO_STREAM, req->client_id,
		      dp_session, (uint32_t *)req, sizeof(*req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)status, sizeof(*status));
}

int bst_display_dp_cmd_dump_debug_info(uint32_t dp_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA) {
		DISP_ERR(" error !! reply size(%d) out of range(%d)",
			reply_size,
			MAX_ACK_DATA);
		return -1;
	}

	build_command(&msg_data, DP_CMD_DUMP_DEBUG_INFO, dump_cfg->client_id,
			dp_session, (uint32_t *)dump_cfg, sizeof(*dump_cfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}
