// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file   bstn_msg_manager.h
 * @brief   This is the header file of the session part in BSTN driver. It
 *      contains the structure definitions of the message manager and
 *      exchange node. It also has declarations of message handling
 *      functions as well as the initialization and cleanup functions.
 */

#ifndef BSTN_MSG_MANAGER_H
#define BSTN_MSG_MANAGER_H

#include "msgbx/bstn_client.h"

#define BSTN_MAX_DEV_NUM 1
#define BSTN_EXCHANGE_NODE_NUM 5 // 1 node for software BIST
// 1s for timeout
#define BSTN_RSP_TIMEOUT_MS \
	(BSTN_EXCHANGE_NODE_NUM * 1000) // exchange timeout in 10s
#define BSTN_RSP_TIMEOUT_JIFFIES (BSTN_RSP_TIMEOUT_MS * HZ / 1000)

#define BSTN_SW_BIST_WAIT_S (5)
#define BSTN_SW_BIST_PERIOD_MS (100)

#define BSTN_MSG_INTERFACE_IPC 0
#define BSTN_MSG_INTERFACE_MSGBOX 1

typedef enum {
	IPC_MSG_CMD_INVALID,
	IPC_MSG_CMD_BOOTDONE,
	IPC_MSG_CMD_LOOPBACK = 0x7f,
} ipc_msg_cmd;

struct bstn_req_msg {
	uint16_t nid;
	struct bsnn_request req;
	uint32_t target_net;
};

struct bstn_rsp_msg {
	uint16_t nid;
	struct bsnn_response rsp;
};

struct bstn_exchange_node {
	uint16_t nid;
	Q_NEW_LINK(bstn_exchange_node, link);
	struct bstn_req_msg *req_buf;
	struct bstn_rsp_msg *rsp_buf;
	struct completion complete;
};

struct bstn_msg_manager {
	int32_t ipc_session_id;

	bstn_client_data_t msgbx_data;
	bstn_client_t *msgbx_client;

	struct completion ipc_boot_complete;

	int32_t bist_thread_start;
	int32_t bstn_r5msg_enable;

	struct bstn_exchange_node *flist; //free exchange node list
	struct bstn_exchange_node
		*sw_bist_flist; //free exchange node list for software BIST
	struct mutex flist_lock;

	struct bstn_memblock *
		req_bufs; //an one-time allocated large buffer for all request buffers
	struct bstn_exchange_node exchange_nodes[BSTN_EXCHANGE_NODE_NUM];

	struct task_struct *msg_receiver_task;
	struct task_struct *msg_sw_bister_task;
};

int bstn_msg_manager_init(struct bstn_device *pbstn);
void bstn_msg_manager_exit(struct bstn_device *pbstn);
bool bstn_msg_is_bootdone(struct bstn_device *pbstn);
int bstn_msg_exchange(struct bstn_device *pbstn, struct bsnn_msg_exchange *msg);
int bstn_msg_start_sw_bist_thread(struct bstn_device *pbstn);
int bstn_msg_psm_enabled_status(struct bstn_device *pbstn);

#endif
