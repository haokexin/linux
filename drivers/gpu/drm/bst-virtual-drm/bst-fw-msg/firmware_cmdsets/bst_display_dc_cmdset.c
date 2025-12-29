// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_dc_cmdset.h"

int bst_display_dc_cmd_do_flush(uint32_t dc_session,
				struct bst_display_flush_cfg *flush_cfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_DO_FLUSH,
			flush_cfg, sizeof(*flush_cfg), reply, sizeof(* reply));
	return ret;
}

int bst_display_dc_cmd_probe_submodule(uint32_t dc_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_submodule_header *submodule_head)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_PROBE_SUBMODULE,
			submodule_req, sizeof(*submodule_req), submodule_head, sizeof(*submodule_head));
	return ret;
}

int bst_display_dc_cmd_get_layer_info(uint32_t dc_session,
				struct bst_display_layer_req *layer_req,
				struct bst_display_layer_info *linfo)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_GET_LAYER_INFO,
			layer_req, sizeof(*layer_req), linfo, sizeof(*linfo));
	return ret;
}

int bst_display_dc_cmd_update_composer(uint32_t dc_session,
				struct bst_display_composer_cfg *composer_cfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_UPDATE_COMPOSER,
			composer_cfg, sizeof(*composer_cfg), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_update_layer(uint32_t dc_session,
				struct bst_display_layer_cfg *lcfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_UPDATE_LAYER,
			lcfg, sizeof(*lcfg), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_get_composer_info(uint32_t dc_session,
				struct bst_display_composer_request *cfg,
				struct bst_display_composer_info *reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_GET_COMPOSER_INFO,
			cfg, sizeof(*cfg), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_update_wb_layer(uint32_t dc_session,
				struct bst_display_wb_layer_cfg *wb_lcfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_UPDATE_WB_LAYER,
			wb_lcfg, sizeof(*wb_lcfg), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_update_coeffs_table(uint32_t dc_session,
				struct bst_display_coeffs_cfg *coeffs_cfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_UPDATE_COEFFS_TABLE,
			coeffs_cfg, sizeof(*coeffs_cfg), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_disable_submodule(uint32_t dc_session,
				struct bst_display_submodule_req *submodule_req,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_DISABLE_SUBMODULE,
			submodule_req, sizeof(*submodule_req), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_set_plane_ids(uint32_t dc_session,
	struct bst_display_plane_ids *plane_ids,
	struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_SET_LAYER_PLANEID,
		plane_ids, sizeof(*plane_ids), reply, sizeof(*reply));
	return ret;
}

int bst_display_dc_cmd_dump_debug_info(uint32_t dc_session,
				struct bst_display_dev_dump* dump_cfg,
				struct bst_display_comm_reply* reply)
{
	int ret;

	ret = bst_display_do_cmd(dc_session, (int)BST_DISPLAY_DC_SUBDEV, (int)DC_CMD_DUMP_DEBUG_INFO,
			dump_cfg, sizeof(*dump_cfg), reply, sizeof(*reply));
	return ret;
}
