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

/**
 * @file  ipc_trans_routing.c
 * @brief file would use api from ipc_hw_layer
 * @details feature list
 * 1. receive message and put it into coreesponding session
 * 2. receive error message from hw_layer(if defined)
 * 3. parse request and transfer to hw_layer to send message (different protocol is reserved)
 */

#include "ipc_trans_routing.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_ses_mgt.h"

int32_t recv_dispatch(void *addr, const uint8_t fid, const rw_msg_t *msg)
{
	int32_t ret = -1;
	uint8_t session_id = 0;
	msgbx_end_device_t *module = NULL;
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	int16_t ses_id = 0;
#endif
#ifndef REMOVE_STS_MGT
	uint8_t cnt = 0;
	sts_endmap_t endmap = {0};
#endif

	// ignore check for addr and msg, already checked in caller.
	// if (!addr || !msg)
	// 	return ret;

	module = (msgbx_end_device_t *)addr;

	if (msg->header.cid != module->g_ipc_pid) {
		IPC_LOG_ERR("dispatch msg endid %d != g_pid %d", msg->header.cid, module->g_ipc_pid);
		return -1;
	}

	IPC_LOG_DEBUG("dispatch msg pid: %d, cid: %d, sid: %d, fid: %d",
		      msg->header.pid, msg->header.cid, msg->header.sid,
		      msg->header.fid);
	IPC_LOG_DEBUG("msg type: %d, cmd: %d, tok: %d, idx: %d, is_eof: %u",
		      msg->header.typ, msg->header.cmd, msg->header.tok,
		      msg->header.idx, msg->header.is_eof);

	if (msg->header.typ == MSGBX_MSG_TYPE_REPLY ||
	    msg->header.typ == MSGBX_MSG_TYPE_BROADCAST ||
	    (msg->header.typ >= MSGBX_MSG_TYPE_USERDEFINED &&
	     msg->header.typ < MSGBX_MSG_TYPE_HARDWARE)) {
		session_id = session_comb(msg->header.sid, msg->header.fid);
		ret = session_msg_in(session_id, msg, addr);
	} else if (msg->header.typ == MSGBX_MSG_TYPE_METHOD) {
#if ((SESSION_COUNT == 1 && CHANNEL_COUNT == 1) ||                             \
     defined(BAREMETAL_VERSION_TRUNCATE))
		ret = session_msg_in(DEFAULT_SES_ID, msg, addr);
#else
		ses_id = module->method_register_map[msg->header.cmd];
		ret = session_msg_in(ses_id, msg, addr);
		session_id = (uint8_t)ses_id;
#endif
	} else if (msg->header.typ == MSGBX_MSG_TYPE_PROTOCOL) {
		ret = 0;
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
		if (msg->header.cmd == TRANS_PROTO_CMD_LOG) {
			ret = session_msg_in(REMOTE_LOG_SES_ID, msg, addr);
			session_id = (uint8_t)REMOTE_LOG_SES_ID;
		}
#endif
#ifndef REMOVE_STS_MGT
		if (msg->header.cmd == TRANS_PROTO_CMD_GET_ENDMAP &&
		    module->g_ipc_pid == CENTRAL_MONITOR_END_ID) {
			module->g_req_endmap_cid = msg->header.pid;
			ret = 2;
		} else {
			endmap.lo_map = msg->payload[0];
			endmap.hi_map = msg->payload[1];
			ret = ipc_trans_end_sts_update(msg->header.pid, msg->header.cmd, &endmap, addr);
		}
#endif
	} else {
		IPC_LOG_DEBUG(
			"discard msg pid: %d, cid: %d, sid: %d, fid: %d",
			msg->header.pid, msg->header.cid, msg->header.sid,
			msg->header.fid);
		IPC_LOG_DEBUG("msg type: %d, cmd: %d, tok: %d, idx: %d",
				msg->header.typ, msg->header.cmd,
				msg->header.tok, msg->header.idx);
		return ret;
	}

	// ret processing
	if (ret == 0) {
		if (msg->header.is_eof == 1)
			ipc_trans_complete(module->g_ipc_cpuid, session_id);
		return 0;
	}
#ifndef REMOVE_STS_MGT
	if (ret == 1) {
		if (module->g_ipc_pid == CENTRAL_MONITOR_END_ID)
			ipc_trans_complete_sts(module->g_ipc_cpuid);

		for (cnt = 0; cnt < CHANNEL_COUNT * SESSION_COUNT; ++cnt) {
			if (module->g_update_ses_list[cnt] == -1)
				break;
			ipc_trans_complete(module->g_ipc_cpuid,
					   module->g_update_ses_list[cnt]);
		}
		ipc_memset(&module->g_update_ses_list, -1,
			   sizeof(module->g_update_ses_list));
		return 1;
	}

	if (ret == 2) {
		ipc_trans_complete_sts(module->g_ipc_cpuid);
		return 2;
	}
#endif

	// note: add can not dispatch method error code loopback feature at here.
	// in Linux kernel should use a special method to reply this error code
#ifdef ENABLE_SERVER_ERROR_CODE_REPLY
	if (ret == -2 || ret == -6) {
		rw_msg_t reply = *msg;
		reply.header.pid = msg->header.cid;
		reply.header.cid = msg->header.pid;
		reply.header.typ = MSGBX_MSG_TYPE_REPLY;
		reply.header.is_eof = 1;
		reply.payload[0] = (ret == -2) ? ERR_APP_SERVER_IS_NOT_AVAIL : ERR_APP_SERVER_IS_FULL;
		ipc_hw_layer_send_msg(module->g_ipc_cpuid, &reply);
	}
#endif

	IPC_LOG_DEBUG(
		"ret: %d, discard msg pid: %d, cid: %d, sid: %d, fid: %d", ret,
		msg->header.pid, msg->header.cid, msg->header.sid,
		msg->header.fid);
	IPC_LOG_DEBUG("msg type: %d, cmd: %d, tok: %d, idx: %d push in fail",
			msg->header.typ, msg->header.cmd, msg->header.tok,
			msg->header.idx);
	return ret;
}

int endmap_dispatch(void *addr, const sts_endmap_t *endmap)
{
#if defined(MSGBX_HW_TYPE_A2000)
#ifndef REMOVE_STS_MGT
	int32_t ret = -1;
	uint8_t cnt = 0;
	msgbx_end_device_t *module = NULL;
#if defined(MULTI_DIE_HW_VERSION)
	uint8_t updated_chipid = 0;
#endif

	if (!addr || !endmap)
		return ret;


	module = (msgbx_end_device_t *)addr;
	ret = ipc_trans_end_sts_update(0, 0, endmap, addr);

	if (ret < 0)
		return ret;

#if defined(MULTI_DIE_HW_VERSION)
	if (module->g_ipc_pid == CENTRAL_MONITOR_END_ID) {
		updated_chipid = module->hw_info.chipid ^= 1;
		ipc_hw_layer_update_endmap(module->g_ipc_cpuid, updated_chipid);
	}
#endif

	for (cnt = 0; cnt < CHANNEL_COUNT * SESSION_COUNT; ++cnt) {
		if (module->g_update_ses_list[cnt] == -1)
			break;
		ipc_trans_complete(module->g_ipc_cpuid,
					module->g_update_ses_list[cnt]);
	}
	ipc_memset(&module->g_update_ses_list, -1,
			sizeof(module->g_update_ses_list));
	return 1;
#endif
#endif
	return 0;
}

int32_t ipc_trans_read_msg(uint8_t fid, uint8_t mode, void *addr)
{
	int8_t ret = 0;

#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;
	rw_msg_t recv_msg = { 0 };
	uint8_t cnt = 0;

	if (!addr)
		return -1;
	module = (msgbx_end_device_t *)addr;

	if (mode == 0) {
		for (cnt = 0; cnt < CHANNEL_COUNT; ++cnt) {
			ret = ipc_hw_layer_get_msg(module->g_ipc_cpuid, &recv_msg,
						   cnt);
			if (ret == 0)
				ret = recv_dispatch(addr, cnt, &recv_msg);
		}
	} else if (mode == 1) {
		ret = ipc_hw_layer_get_msg(module->g_ipc_cpuid, &recv_msg, fid);
		if (ret == 0)
			ret = recv_dispatch(addr, fid, &recv_msg);
	} else {
		ret = -2;
	}
#endif
	return ret;
}

int8_t ipc_trans_routing_init(void *addr)
{
	int8_t ret = 0;

#if (SESSION_COUNT > 1)
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;

	if (!addr)
		return -1;
	module = (msgbx_end_device_t *)addr;
	ipc_memset(&module->method_register_map, -1,
		   sizeof(module->method_register_map));
#endif
#endif

#if (MSGBX_RECV_MODE == 1)
	ret = ipc_hw_layer_recv_ntf_register(addr, recv_dispatch);
	if (ret < 0) {
		IPC_LOG_ERR("register recv func fail, ret: %d", ret);
		return ret;
	}
#endif

#if defined(MSGBX_HW_TYPE_A2000)
	ret = ipc_hw_layer_endmap_ntf_register(addr, endmap_dispatch);
	if (ret < 0)
		IPC_LOG_ERR("register endmap func fail, ret: %d", ret);
#endif
	return ret;
}
