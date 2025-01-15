// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_dc_cmdset.h"
static void build_command(struct fw_msg_data *msg_data, int cmdid,
				uint32_t client_id, uint32_t subdev_session,
				uint32_t *usr_param, uint32_t size_param)
{
	msg_data->client_id = client_id;
	msg_data->cmdset = BST_DISPLAY_DC_SUBDEV;
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
		// DISP_ERR("build_reply failed!! (reply_client=0x%x != cmd_client = 0x%x)",
		// 	reply_client,
		// 	cmd_client);
		return -1;

	}

	return 0;
}

int bst_display_dc_cmd_do_flush(uint32_t dc_session,
				struct bst_display_flush_cfg *flush_cfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA) {
		DISP_ERR("virt_dc_cmd_do_flush error !! reply size(%d) out of range(%d)",
			reply_size,
			MAX_ACK_DATA);
		return -1;
	}

	build_command(&msg_data, DC_CMD_DO_FLUSH, flush_cfg->client_id,
			dc_session, (uint32_t *)flush_cfg, sizeof(*flush_cfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_probe_submodule(uint32_t dc_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head)
{
	struct fw_msg_data msg_data;
	uint32_t submodule_head_size = sizeof(*submodule_head);

	if (submodule_head_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_PROBE_SUBMODULE, submodule_req->client_id,
		      dc_session, (uint32_t *)submodule_req, sizeof(*submodule_req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)submodule_head, submodule_head_size);
}

int bst_display_dc_cmd_get_layer_info(uint32_t dc_session,
				struct bst_display_layer_req *layer_req,
				struct bst_display_layer_info *linfo)
{
	struct fw_msg_data msg_data;
	uint32_t linfo_size = sizeof(*linfo);

	if (linfo_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_GET_LAYER_INFO, layer_req->client_id,
		      dc_session, (uint32_t *)layer_req, sizeof(*layer_req));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)linfo, linfo_size);
}

int bst_display_dc_cmd_update_composer(uint32_t dc_session,
				struct bst_display_composer_cfg *composer_cfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_COMPOSER, composer_cfg->client_id,
		      dc_session, (uint32_t *)composer_cfg, sizeof(*composer_cfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_update_layer(uint32_t dc_session,
				struct bst_display_layer_cfg *lcfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_LAYER, lcfg->client_id,
		      dc_session, (uint32_t *)lcfg, sizeof(*lcfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_get_composer_info(uint32_t dc_session,
				struct bst_display_composer_info *info)
{
	struct fw_msg_data msg_data;
	uint32_t info_size = sizeof(*info);

	if (info_size > MAX_USR_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_GET_COMPOSER_INFO, info->client_id,
		      dc_session, NULL, 0);

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)info, info_size);
}

int bst_display_dc_cmd_update_wb_layer(uint32_t dc_session,
				struct bst_display_wb_layer_cfg *wb_lcfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_WB_LAYER, wb_lcfg->client_id,
		      dc_session, (uint32_t *)wb_lcfg, sizeof(*wb_lcfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_update_trust_layer(uint32_t dc_session,
				struct bst_display_trust_layer_cfg *trust_lcfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_TRUST_LAYER, trust_lcfg->client_id,
		      dc_session, (uint32_t *)trust_lcfg, sizeof(*trust_lcfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_update_layer_scaler(uint32_t dc_session,
				struct bst_display_scaler_cfg *scfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_LAYER_SCALER, scfg->client_id,
		      dc_session, (uint32_t *)scfg, sizeof(*scfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int  bst_display_dc_cmd_update_layer_crop(uint32_t dc_session,
				struct  bst_display_crop_cfg *ccfg,
				struct  bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_LAYER_CROP, ccfg->client_id,
		      dc_session, (uint32_t *)ccfg, sizeof(*ccfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_update_coeffs_table(uint32_t dc_session,
				struct bst_display_coeffs_cfg *coeffs_cfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_UPDATE_COEFFS_TABLE,
			  coeffs_cfg->client_id, dc_session,
			  (uint32_t *)coeffs_cfg, sizeof(*coeffs_cfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_disable_submodule(uint32_t dc_session,
				struct bst_display_submodule_disable *submodule_dis,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data = { 0 };
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA)
		return -1;

	build_command(&msg_data, DC_CMD_DISABLE_SUBMODULE, submodule_dis->client_id,
		      dc_session, (uint32_t *)submodule_dis, sizeof(*submodule_dis));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}

int bst_display_dc_cmd_dump_debug_info(uint32_t dc_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply)
{
	struct fw_msg_data msg_data;
	uint32_t reply_size = sizeof(*reply);

	if (reply_size > MAX_ACK_DATA) {
		DISP_ERR("error !! reply size(%d) out of range(%d)",
			reply_size,
			MAX_ACK_DATA);
		return -1;
	}

	build_command(&msg_data, DC_CMD_DUMP_DEBUG_INFO, dump_cfg->client_id,
			dc_session, (uint32_t *)dump_cfg, sizeof(*dump_cfg));

	transfer_fw_msg(&msg_data);

	return build_reply(&msg_data, (uint32_t *)reply, reply_size);
}
