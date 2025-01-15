// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

/**
 * @file  ipc_trans_routing.c
 * @brief file would use api from ipc_hw_layer
 * @details feature list
 * 1. receive message and put it into coreesponding session
 * 2. receive error message from hw_layer(if defined)
 * 3. parse request and transfer to hw_layer to send message (different protocol is reserved)
 */
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_trans_layer.h>
#include "ipc_trans_routing.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_sts_mgt.h"

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
#endif

	// according to dispatching rules, push messages into corresponding session message queue
	if (!addr || !msg)
		return ret;

	module = (msgbx_end_device_t *)addr;

	if (msg->header.cid != module->g_ipc_pid) {
		IPC_LOG_ERR("dispatch msg endid %d != g_pid %d", msg->header.cid, module->g_ipc_pid);
		return -1;
	}

	// message dispatching
	IPC_LOG_DEBUG("dispatch msg pid: %d, cid: %d, sid: %d, fid: %d",
		      msg->header.pid, msg->header.cid, msg->header.sid,
		      msg->header.fid);
	IPC_LOG_DEBUG("msg type: %d, cmd: %d, tok: %d, idx: %d",
		      msg->header.typ, msg->header.cmd, msg->header.tok,
		      msg->header.idx);

	if (msg->header.typ == MSGBX_MSG_TYPE_REPLY ||
	    msg->header.typ == MSGBX_MSG_TYPE_BROADCAST ||
	    (msg->header.typ >= MSGBX_MSG_TYPE_USERDEFINED &&
	     msg->header.typ < MSGBX_MSG_TYPE_HARDWARE)) {
		session_comb(msg->header.sid, msg->header.fid, &session_id);
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
		} else
			ret = ipc_end_sts_update(msg->header.pid,
						 msg->header.cmd,
						 msg->payload[0], addr);
#endif
	} else {
		IPC_LOG_WARNING(
			"discard msg pid: %d, cid: %d, sid: %d, fid: %d",
			msg->header.pid, msg->header.cid, msg->header.sid,
			msg->header.fid);
		IPC_LOG_WARNING("msg type: %d, cmd: %d, tok: %d, idx: %d",
				msg->header.typ, msg->header.cmd,
				msg->header.tok, msg->header.idx);
		return ret;
	}

	// ret processing
	if (ret == 0) {
		ipc_trans_complete(module->ops.cpuid, session_id);
		return 0;
	}
#ifndef REMOVE_STS_MGT
	if (ret == 1) {
		if (module->g_ipc_pid == CENTRAL_MONITOR_END_ID)
			ipc_trans_complete_sts(module->ops.cpuid);

		for (cnt = 0; cnt < CHANNEL_COUNT * SESSION_COUNT; ++cnt) {
			if (module->g_update_ses_list[cnt] == -1)
				break;
			ipc_trans_complete(module->ops.cpuid,
					   module->g_update_ses_list[cnt]);
		}
		ipc_memset(&module->g_update_ses_list, -1,
			   sizeof(module->g_update_ses_list));
		return 1;
	}

	if (ret == 2) {
		ipc_trans_complete_sts(module->ops.cpuid);
		return 2;
	}
#endif

	IPC_LOG_WARNING(
		"ret: %d, discard msg pid: %d, cid: %d, sid: %d, fid: %d", ret,
		msg->header.pid, msg->header.cid, msg->header.sid,
		msg->header.fid);
	IPC_LOG_WARNING("msg type: %d, cmd: %d, tok: %d, idx: %d push in fail",
			msg->header.typ, msg->header.cmd, msg->header.tok,
			msg->header.idx);
	return ret;
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
	// check mode
	// 0 means get messages from every filter
	// 1 means get message from specific filter

	if (mode == 0) {
		// means get all flt

		for (cnt = 0; cnt < CHANNEL_COUNT; ++cnt) {
			ret = ipc_hw_layer_get_msg(module->ops.cpuid, &recv_msg,
						   cnt);
			if (ret == 0)
				ret = recv_dispatch(addr, cnt, &recv_msg);
		}
	} else if (mode == 1) {
		ret = ipc_hw_layer_get_msg(module->ops.cpuid, &recv_msg, fid);
		if (ret == 0)
			ret = recv_dispatch(addr, fid, &recv_msg);
	} else {
		ret = -2;
	}
#endif
	return ret;
}

int32_t ipc_trans_register_method(const uint8_t session_id, const uint8_t cmd,
				  void *dev_info)
{
#if (SESSION_COUNT > 1)
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;
	int8_t ret = -1;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ret = session_isvalid(session_id, dev_info);
	if (ret != 0)
		return -ERR_SES_IS_INVALID;

	// spec note: even cmd value is over 256, compiler will optimize and truncate it.
	if (module->method_register_map[cmd] > 0)
		return -ERR_REGISTER_METHOD_REPEATE;

	module->method_register_map[cmd] = session_id;
	IPC_LOG_INFO("ses %d register cmd %d success", session_id, cmd);
#endif
#endif
	return 0;
}

int32_t ipc_trans_unregister_method(const uint8_t session_id, void *dev_info)
{
#if (SESSION_COUNT > 1)
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;
	int8_t ret = -1;
	uint8_t cnt = 0;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ret = session_isvalid(session_id, dev_info);
	if (ret != 0)
		return -ERR_SES_IS_INVALID;

	for (cnt = 0; cnt < CMD_MAX_COUNT; ++cnt)
		if (module->method_register_map[cnt] == session_id)
			module->method_register_map[cnt] = -1;
#endif
#endif
	return 0;
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

#if (IPC_RECV_MODE == 1)
	ret = ipc_hw_layer_recv_ntf_register(addr, recv_dispatch);
	if (ret < 0)
		IPC_LOG_ERR("register recv func fail, ret: %d", ret);
#endif
	return ret;
}
