/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef IPC_SESSION_H
#define IPC_SESSION_H

#include <linux/list.h>
#include <linux/kfifo.h>

#include "ipc_communication_manager.h"
#include "user_head.h"



struct ipc_drv_msg;
struct ipc_client_info;

enum ipc_session_status {
	SESSION_STATE_NULL = 0x0,
	SESSION_INIT = 0x1,
	SESSION_READY = 0x2,
	SESSION_SENDING = 0x4,
	SESSION_RECEIVING = 0x10,
	SESSION_WAIT_RECEIVE = 0x20,
	SESSION_WAIT_SEND = 0x30,
	SESSION_SENT = 0x31,
	SESSION_RECEIVED = 0x32,
	SESSION_DESTROY = 0x40,
	SESSION_STATUS_MAX = INT_MAX
};
#define IPC_SESSION_STATUS enum ipc_session_status

struct ipc_session {
	uint32_t id; // id of this session
	struct fasync_struct *ipc_fasync;
	pid_t pid_num; // for user space
	IPC_SESSION_STATUS status;
	struct kfifo recv_msg_fifo;
	enum ipc_core_e src;
	enum ipc_core_e dest;
	struct completion rx_complete;
	struct completion tx_complete;
	struct mutex session_mutex;
	uint32_t waiting_reply_msg_token; //now is deprecated
	struct ipc_client_info *cl_info;
	uint16_t token;
};

// reserved
// get session information
struct ipc_session *get_session_by_id(uint32_t id);
struct ipc_session *ipc_get_session_by_coreid(enum ipc_core_e coreid);
bool ipc_get_dst_by_session_id(uint32_t session_id, uint32_t *src, uint32_t *dst);

// session message fifo manipulation
int32_t ipc_session_msg_in(uint32_t session_id, struct ipc_drv_msg msg);
int32_t ipc_session_msg_out(uint32_t session_id, struct ipc_drv_msg *recv_msg);

// session destroy
int32_t ipc_session_destroy_by_id(uint32_t session_id);
int32_t ipc_session_destroy_by_pid(pid_t pid);

// session state control
IPC_SESSION_STATUS get_session_status(uint32_t session_id);
int32_t set_session_status(uint32_t session_id, IPC_SESSION_STATUS status);
int32_t ipc_register_session(enum ipc_core_e src, enum ipc_core_e dst);
bool ipc_session_valid(uint32_t id);

#endif
