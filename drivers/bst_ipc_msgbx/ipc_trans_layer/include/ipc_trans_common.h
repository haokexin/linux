/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 *
 * This program is also distributed under the terms of the Apache 2.0
 * License.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _IPC_TRANS_COMMON_H
#define _IPC_TRANS_COMMON_H

#include <bst/ipc_hw_layer.h>
#include "config.h"
#include <bst/ipc_lflist_siso.h>

struct _debug_info_t {
	uint8_t role;
	uint8_t cid;
	uint8_t ccid;
#ifdef DEBUG_MODE_ENABLE
	uint8_t send_fail_cnt;
	uint32_t send_msg_cnt;
	uint32_t send_rw_msg_cnt;
	uint32_t recv_rw_msg_cnt;
	uint32_t in_rw_msg_cnt;
	uint32_t recv_msg_1_cnt;
	uint32_t recv_msg_2_cnt;
	uint32_t res;
#endif
#ifdef TIMESTAMP_DEBUG_ENABLE
	uint64_t send_start_time;
	uint64_t send_end_time;
	uint64_t get_msg_time;
	uint64_t max_send_time;
#endif
};
#define debug_info_t struct _debug_info_t

struct _ses_base_t {
	uint8_t role;
	uint8_t fid;
	uint8_t sid;
	uint8_t cid;
#if defined(MSGBX_HW_TYPE_A2000)
	uint8_t ccid;
	uint8_t res[3];
#endif
};
#define ses_base_t struct _ses_base_t

enum _ses_status_t {
	SES_STATE_INVALID = 0,
	SES_STATE_NOT_AVAIL,
	SES_STATE_DESTROY,
	SES_STATE_AVAIL,
	SES_STATE_DST_CHANGED,
};
#define ses_status_t enum _ses_status_t

struct _ipc_ses_t {
	ses_base_t info;
	uint8_t status;
	uint8_t update_flag;
	uint8_t cid_idx;
	uint8_t res;
#if defined(MSGBX_HW_TYPE_A2000)
	uint8_t res2[4];
#endif
#ifdef DEBUG_MODE_ENABLE
	debug_info_t debug_info;
#endif
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	bst_msg_queue_t msg_queue;
#else
	bst_msg_queue_t *msg_queue;
#endif
};
#define ipc_ses_t struct _ipc_ses_t
#define SES_UPDATE_STS_BIT 1
#define SES_UPDATE_MSG_BIT 2

#define CMD_MAX_COUNT 255U
#define MSGBX_SID_MAX 16U

struct _msgbx_end_device_t {
	msgbx_hw_info_t hw_info;
	uint8_t g_ipc_cpuid;
	uint8_t g_ipc_pid;
	uint8_t g_ipc_flt_cnt;
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	int16_t method_register_map[CMD_MAX_COUNT];
#endif
#ifndef REMOVE_STS_MGT
	int16_t g_update_ses_list[CHANNEL_COUNT * SESSION_COUNT];
	uint8_t g_req_endmap_cid;
	sts_endmap_t g_end_sts_map;
#endif
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	uint8_t g_log_tok;
#endif
	ipc_ses_t ses_map[CHANNEL_COUNT * SESSION_COUNT];
};
#define msgbx_end_device_t struct _msgbx_end_device_t

#define TRANS_PROTO_CMD_ENDMAP_OFFLINE 0
#define TRANS_PROTO_CMD_ENDMAP_ONLINE 1
#define TRANS_PROTO_CMD_LOG 2
#define TRANS_PROTO_CMD_GET_ENDMAP 3

extern msgbx_end_device_t *g_ipc_end_array[NR_CPUS];
extern struct platform_device *g_ipc_msgbx_pdev;
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
extern struct task_struct *loopback;
#endif
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
extern int32_t ipc_trans_complete_test(const uint8_t fid, const rw_msg_t *msg);
#endif

extern u8 msgbx_get_start_pid(void);
extern uint64_t msgbx_get_max_send_time(uint8_t endid, uint8_t handle);
// extern void msgbx_clr_max_send_time(uint8_t endid, uint8_t handle);
#endif
