// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_cmdset_api.h"
#include "bst_display_global_api.h"

static uint32_t g_client_id;
static uint32_t g_platform_id;

void display_ipc_client_init(uint32_t client_id, uint32_t platform_id)
{
	g_client_id = client_id;
	g_platform_id = platform_id;
}

static int build_command(uint32_t subdev_session, int cmd_set, int cmd_id,
				struct fw_msg_data *msg_data, uint32_t *usr_param, uint32_t size_param)
{
	if (size_param > (uint32_t)MAX_USR_DATA) {
		DISP_ERR("cmd(%d-%d) error !! request size(%d) out of range(%d)", 
			cmd_set, cmd_id, size_param, MAX_USR_DATA);
		return -1;
	}

	msg_data->client_id = g_client_id;
	msg_data->platform_id = g_platform_id;
	msg_data->cmdset = (uint32_t)cmd_set;
	msg_data->cmdid = (uint32_t)cmd_id;
	msg_data->size_cmd = size_param;
	(void)memcpy(&msg_data->user_cmd_data[0], usr_param, (unsigned int)size_param);
	msg_data->subdev_session = subdev_session;
	msg_data->sync_mode = CMD_SYNC_MDDE;
	(void)memset(&msg_data->user_ack_data[0], 0, sizeof(msg_data->user_ack_data));

	return 0;
}

static int build_reply(struct fw_msg_data *msg_data, uint32_t *usr_ack, uint32_t size_ack)
{
	if (size_ack > MAX_ACK_DATA) {
		DISP_ERR("cmd(%d-%d) error !! reply size(%d) out of range(%d)", 
			msg_data->cmdset, msg_data->cmdid, size_ack, MAX_ACK_DATA);
		return -1;
	}
	(void)memcpy(usr_ack, &msg_data->user_ack_data[0], (unsigned int)size_ack);

	return 0;
}

int bst_display_do_cmd(uint32_t subdev_session, int cmd_set, int cmd_id,
			void *req, uint32_t req_size, void *reply, uint32_t reply_size)
{
	int ret;
	uint32_t cmd_client;
	reply_base *base;
	struct fw_msg_data msg_data;
	union {
		void *handle;
		uint32_t *req;
	} req_u;
	union {
		void *handle;
		uint32_t *reply;
	} reply_u;
	union {
		void *handle;
		reply_base *base;
	} reply_base_u;

	req_u.handle = req;
	reply_u.handle = reply;
	ret = build_command(subdev_session, cmd_set, cmd_id,
			&msg_data, (uint32_t *)req_u.req, req_size);
	if (0 == ret) {
		(void)transfer_fw_msg(&msg_data);
		ret = build_reply(&msg_data, (uint32_t *)reply_u.reply, reply_size);
	}

	if (0 == ret) {
		cmd_client = msg_data.client_id;
		reply_base_u.handle = &msg_data.user_ack_data[0];
		base = (reply_base *)(reply_base_u.base);
		if ((base->client_id != cmd_client) && (base->status != DISP_COMM_REPLAY_OK)) {
			DISP_ERR("cmd(%d-%d) build_reply failed!! reply_client[0x%x],cmd_client[0x%x],replay_status[%d])",
				msg_data.cmdset, msg_data.cmdid, cmd_client, base->client_id, base->status);
			return -1;
		}
	}
	return ret;
}
