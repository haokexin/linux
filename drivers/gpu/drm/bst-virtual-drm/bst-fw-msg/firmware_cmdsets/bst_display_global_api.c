// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include "bst_display_global_api.h"

int bst_display_glb_cmd_probe_subdev(struct bst_subdev_probe_request *request,
				struct bst_subdev_probe_response *response)
{
	int ret;
	uint32_t want_info_size = request->want_info_size;

	ret = bst_display_do_cmd(0, (int)BST_DISPLAY_GLB_SUBDEV, (int)GLB_CMD_PROBE_SUBDEV,
			request, sizeof(*request), response, sizeof(*response));

	if ((ret == 0) && (want_info_size != response->probed_info_size)) {
		DISP_ERR("cmd(%d-%d), want_info_size=%d, probed_info_size=%d\n",
				BST_DISPLAY_GLB_SUBDEV, GLB_CMD_PROBE_SUBDEV, want_info_size, response->probed_info_size);
		return -1;
	}
	return ret;
}

int bst_display_glb_cmd_is_valid_topology(
			struct bst_display_topology_info *topo_info,
			struct bst_display_topology_status *topo_status)
{
	int ret = 0;

	ret = bst_display_do_cmd(0, (int)BST_DISPLAY_GLB_SUBDEV, (int)GLB_CMD_IS_VALID_TOPO,
			topo_info, sizeof(*topo_info), topo_status, sizeof(*topo_status));
	return ret;
}

int bst_display_glb_cmd_get_subdev_info(struct bst_subdev_info_req *request,
                          struct bst_subdev_info_result *response)
{
	int ret = 0;

	ret = bst_display_do_cmd(0, (int)BST_DISPLAY_GLB_SUBDEV, (int)GLB_CMD_GET_SUBDEV_INFO,
			request, sizeof(*request), response, sizeof(*response));
	return ret;
}

int bst_display_glb_cmd_get_all_subdev_topo(struct bst_all_subdev_topo_req *request,
                          struct bst_all_subdev_topo *response)
{
	int ret = 0;

	ret = bst_display_do_cmd(0, (int)BST_DISPLAY_GLB_SUBDEV, (int)GLB_CMD_GET_ALL_SUBDEV_TOPO,
			request, sizeof(*request), response, sizeof(*response));
	return ret;
}

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

int __attribute__((weak))
fw_msg_events_sub(uint32_t subdev, disp_event_callback_t cb, void *ext)
{
	UNUSED(subdev);
	UNUSED(cb);
	UNUSED(ext);
	DISP_ERR("fw_msg_events_sub NOT define!\n");
	return 0;
}
int __attribute__((weak)) fw_msg_events_unsub(uint32_t subdev)
{
	UNUSED(subdev);
	DISP_ERR("fw_msg_events_unsub NOT define!");
	return 0;
}

int bst_display_glb_cmd_subscribe_events(uint32_t subdev,
				      disp_event_callback_t cb, void *data)
{
	if (subdev >= (uint32_t)BST_SUBDEV_MAX || subdev <= (uint32_t)BST_SUBDEV_NONE) {
		DISP_ERR("Error, subdev(%d) is invalid!\n", subdev);
		return -1;
	}
	return fw_msg_events_sub(subdev, cb, data);
}
int bst_display_glb_cmd_unsubscribe_events(uint32_t subdev)
{
	if (subdev >= (uint32_t)BST_SUBDEV_MAX || subdev <= (uint32_t)BST_SUBDEV_NONE) {
		DISP_ERR("Error, subdev(%d) is invalid!\n", subdev);
		return -1;
	}
	return fw_msg_events_unsub(subdev);
}