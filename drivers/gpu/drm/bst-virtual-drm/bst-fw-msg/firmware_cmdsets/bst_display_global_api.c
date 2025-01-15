// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_global_api.h"

static void build_command(struct fw_msg_data *msg_data, int cmdid,
			  uint32_t client_id, uint32_t *usr_param,
			  uint32_t size_param)
{
	msg_data->client_id = client_id;
	msg_data->cmdset = BST_DISPLAY_GLB_SUBDEV;
	msg_data->cmdid = cmdid;
	msg_data->size_cmd = size_param;
	memcpy(&msg_data->user_cmd_data[0], usr_param, size_param);
	/* for FW probe the subdev_session fix to 0*/
	msg_data->subdev_session = 0;
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

int bst_display_glb_cmd_probe_subdev(struct bst_subdev_probe_request *request,
			  struct bst_subdev_probe_response *response)
{
	struct fw_msg_data msg_data;
	uint32_t req_size = sizeof(*request);
	uint32_t want_info_size = request->want_info_size;
	int ret = 0;

	if (req_size > MAX_USR_DATA) {
		DISP_ERR(" request size out of range(%d>%d)", req_size,
			 MAX_USR_DATA);
		return -1;
	}
	build_command(&msg_data, GLB_CMD_PROBE_SUBDEV, request->client_id,
 		      (uint32_t *)request, req_size);
	transfer_fw_msg(&msg_data);
	ret = build_reply(&msg_data, (uint32_t *)response, sizeof(*response));
	if (ret || want_info_size != response->probed_info_size) {
		DISP_ERR(
			" build_reply failed,ret=%d,want_info_size=%d, probed_info_size=%d\n",
			ret, want_info_size, response->probed_info_size);
		return -1;
	}

	return 0;
}

int bst_display_glb_cmd_is_valid_topology(
			struct bst_display_topology_info *topo_info,
			struct bst_display_topology_status *topo_status)
{
	struct fw_msg_data msg_data;
	uint32_t info_size = sizeof(*topo_info);
	int ret = 0;

	if (info_size > MAX_USR_DATA) {
		DISP_ERR(" request size out of range(%d>%d)", info_size,
			 MAX_USR_DATA);
		return -1;
	}
	build_command(&msg_data, GLB_CMD_IS_VALID_TOPO, topo_info->client_id,
		      (uint32_t *)topo_info, info_size);
	transfer_fw_msg(&msg_data);
	ret = build_reply(&msg_data, (uint32_t *)topo_status,
			  sizeof(*topo_status));
	if (ret)
		return -1;

	return 0;
}
int bst_display_glb_cmd_get_edid(struct bst_display_edid_req *edid,
				 struct bst_display_edid_info *info)
{
	struct fw_msg_data msg_data;
	uint32_t info_size = sizeof(*edid);

	if (info_size > MAX_USR_DATA) {
		DISP_ERR(" request size out of range(%d>%d)", info_size,
			 MAX_USR_DATA);
		return -1;
	}
	build_command(&msg_data, GLB_CMD_GET_EDID, edid->client_id,
		      (uint32_t *)edid, sizeof(*edid));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)info, sizeof(*info));
}

int bst_display_glb_cmd_get_cur_video_mode(struct bst_display_vm_req *req,
					   struct bst_display_vm_setting *info)
{
	struct fw_msg_data msg_data;
	uint32_t info_size = sizeof(*req);
	int ret = 0;

	if (info_size > MAX_USR_DATA) {
		DISP_ERR(" request size out of range(%d>%d)", info_size,
			 MAX_USR_DATA);
		return -1;
	}
	build_command(&msg_data, GLB_CMD_GET_CUR_VM, req->client_id,
		      (uint32_t *)req, info_size);
	transfer_fw_msg(&msg_data);
	ret = build_reply(&msg_data, (uint32_t *)info, sizeof(*info));
	if (ret)
		return -1;

	return 0;
}
int bst_display_glb_cmd_get_subdev_info(struct bst_subdev_info_req *request,
                          struct bst_subdev_info_result *response)
{
	struct fw_msg_data msg_data;
	uint32_t info_size = sizeof(*request);

	if (info_size > MAX_USR_DATA) {
		DISP_ERR(" request size out of range(%d>%d)", info_size,
			 MAX_USR_DATA);
		return -1;
	}
	build_command(&msg_data, GLB_CMD_GET_SUBDEV_INFO, request->client_id,
		      (uint32_t *)request, info_size);
	transfer_fw_msg(&msg_data);
	memcpy(response, &msg_data.user_ack_data[0], sizeof(*response));

	return 0;
}

int __attribute__((weak))
fw_msg_events_sub(uint32_t subdev, disp_event_callback_t cb, void *ext)
{
	DISP_ERR("fw_msg_events_sub NOT define!\n");
	return 0;
}
int __attribute__((weak)) fw_msg_events_unsub(uint32_t subdev)
{
	DISP_ERR("fw_msg_events_unsub NOT define!");
	return 0;
}

int bst_display_glb_cmd_subscribe_events(uint32_t subdev,
				      disp_event_callback_t cb, void *data)
{
	if (subdev >= BST_SUBDEV_MAX || subdev <= BST_SUBDEV_NONE) {
		DISP_ERR("Error, subdev(%d) is invalid!\n", subdev);
		return -1;
	}
	return fw_msg_events_sub(subdev, cb, data);
}
int bst_display_glb_cmd_unsubscribe_events(uint32_t subdev)
{
	if (subdev >= BST_SUBDEV_MAX || subdev <= BST_SUBDEV_NONE) {
		DISP_ERR("Error, subdev(%d) is invalid!\n", subdev);
		return -1;
	}
	return fw_msg_events_unsub(subdev);
}