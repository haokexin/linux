// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_DISPLAY_CMDSETS_API_H
#define BST_DISPLAY_CMDSETS_API_H

#include "bst_display_platform.h"

#define BST_COLOR_DEPTHS_8BIT	(1U<<8U)
#define BST_COLOR_DEPTHS_10BIT	(1U<<10U)

#define BST_DC_OUT_COLOR_FORMAT_RGB444		(1U << 0U)
#define BST_DC_OUT_COLOR_FORMAT_YCRCB444	(1U << 1U)
#define BST_DC_OUT_COLOR_FORMAT_YCRCB422	(1U << 2U)
#define BST_DC_OUT_COLOR_FORMAT_YCRCB420	(1U << 3U)

// cmdset definition
enum {
	BST_DISPLAY_GLB_SUBDEV = 1,
	BST_DISPLAY_DC_SUBDEV = 2,
	BST_DISPLAY_CONN_SUBDEV = 3,
};

typedef struct bst_display_reply_base {
	uint32_t client_id;
	uint8_t status;
}reply_base;

struct bst_display_comm_reply {
	reply_base base;
};

#define SUBMODULE_MAX_INPUT 9
#define SUBMODULE_MAX_OUTPUT 5

struct bst_display_submodule_req {
	uint32_t submodule_id;
};

struct bst_display_submodule_header {
	reply_base base;
	uint32_t submodule_info;
	uint32_t pipeline_info;
	uint16_t input_ids[SUBMODULE_MAX_INPUT];
	uint16_t output_ids[SUBMODULE_MAX_OUTPUT];
	uint8_t input_id_num;
	uint8_t output_id_num;
};

#define MAX_USR_DATA 			(35U * 4U)
#define MAX_ACK_DATA 			(43U * 4U)
#define CMD_SYNC_MDDE 			(0U)
#define CMD_ASYNC_MDDE 			(1U)

#define DISP_COMM_REPLAY_OK		(0x0U)
#define DISP_COMM_REPLAY_FAILED	(0x1U)

struct fw_msg_data {
	uint32_t client_id;
	uint32_t platform_id;
	uint32_t subdev_session;
	uint32_t cmdset;
	uint32_t cmdid;
	uint32_t size_cmd;
	uint32_t size_ack;
	uint32_t user_cmd_data[MAX_USR_DATA / 4];
	uint32_t user_ack_data[MAX_ACK_DATA / 4];
	uint32_t sync_mode;
};

extern void display_ipc_client_init(uint32_t client_id, uint32_t platform_id);
extern int bst_display_do_cmd(uint32_t subdev_session, int cmd_set, int cmd_id,
			void *req, uint32_t req_size, void *reply, uint32_t reply_size);

#endif /* BST_DISPLAY_DC_CMDSET_H */
