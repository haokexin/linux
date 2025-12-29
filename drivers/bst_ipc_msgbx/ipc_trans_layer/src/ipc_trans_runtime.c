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
 * @file  ipc_trans_runtime.c
 * @brief This file serves as the API for the IPC transferring layer,
 * facilitating direct usage by ipc_app_layer or enabling driver developers to
 * implement ipc_trans_impl.c for OS-specific adaptations.
 * @note
 * @details feature list
 * 1. trans layer core api provider
 */
#include <bst/ipc_trans_layer.h>
#include "ipc_trans_runtime.h"
#include "ipc_trans_routing.h"
#include "ipc_trans_sts_mgt.h"
#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_msg_mgt.h"
#include "ipc_trans_utl.h"
#include "ipc_trans_cfg.h"

/**
 * @name  ipc_trans_init
 * @param in :role, err_func, dev_ifno
 * @param out:
 * @return result of initiating trans layer
 * @details
 * 1. Initialize MsgBx hardware layer.
 * 2. Establish trans layer data structure.
 * 3. Signal availability of MsgBx end message.
 */
int32_t ipc_trans_init(const uint8_t role, err_msg_ntf err_func,
		       void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	ipc_init_params_t init_params = { 0 };

	if (!dev_info)
		return ret;
	module = (msgbx_end_device_t *)dev_info;

	init_params.mbx_device = IPC_HW_MSGBX_MODE;
	init_params.msgbx_end_mgt_flag = MSG_DEF_FLT_MGT_CONFIG;

	ret = ipc_hw_layer_init(module->g_ipc_cpuid, &init_params, &module->hw_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	// update global info
	module->g_ipc_pid = module->hw_info.mbx_end_id;
	module->g_ipc_flt_cnt = module->hw_info.mbx_flt_cnt;

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	module->g_log_tok = 0;
#endif
	pr_info("init msgbx filter cnt = %u, pid = %u, cpuid %u\n",
		     module->g_ipc_flt_cnt, module->g_ipc_pid, module->g_ipc_cpuid);
#if defined(MSGBX_HW_TYPE_A2000)
	IPC_LOG_INFO("chipid = %u", module->hw_info.chipid);
#elif defined(MSGBX_HW_TYPE_C1200)
	module->hw_info.chipid = 0;
#endif

	if (module->g_ipc_flt_cnt < CHANNEL_COUNT) {
		IPC_LOG_ERR(
			"hw filter count %d is less than sw define %d, please check your definition",
			module->g_ipc_flt_cnt, CHANNEL_COUNT);
		return -ERR_TRANS_INIT_FAIL;
	}

	ret = ipc_trans_routing_init(dev_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	ret = session_mgt_init(dev_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	ret = flt_cfg_init(dev_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	// state management setting
#ifdef IPC_STATE_MGT_ENABLE
	ret = ipc_hw_layer_err_msg_register(module->g_ipc_cpuid, err_func);
	if (ret < 0) {
		IPC_LOG_WARNING("init err msg handle fail, ret = %d", ret);
		return -ERR_TRANS_INIT_FAIL;
	}
#endif

	ret = ipc_hw_layer_start();
	if (ret < 0) {
		IPC_LOG_WARNING("hw start fail, ret: %d", ret);
		return -ERR_TRANS_INIT_START_FAIL;
	}

	ret = ipc_end_register(module->g_ipc_pid, dev_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	return RESULT_SUCCESS;
}

int32_t ipc_trans_reinit(void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	msgbx_hw_info_t hw_info = { 0 };
	ipc_init_params_t init_params = {
		.mbx_device = IPC_HW_MSGBX_MODE,
		.msgbx_end_mgt_flag = MSG_DEF_FLT_MGT_CONFIG,
	};

	if (!dev_info)
		return ret;
	module = (msgbx_end_device_t *)dev_info;

	ret = ipc_hw_layer_init(module->g_ipc_cpuid, &init_params, &hw_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	ret = ipc_end_register(module->g_ipc_pid, dev_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_deinit
 * @param in : dev_info
 * @param out:
 * @return result of deinit msgbx driver
 * @details
 */
int32_t ipc_trans_deinit(void *dev_info)
{
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;

	if (!dev_info)
		return -ERR_DEINIT_FAIL;

	module = (msgbx_end_device_t *)dev_info;
	ret = ipc_end_unregister(module->g_ipc_pid, dev_info);
	if (ret < 0)
		return -ERR_DEINIT_FAIL;

	return ipc_hw_layer_deinit(module->g_ipc_cpuid);
}

/**
 * @name  ipc_trans_create_session
 * @param in :sid, fid, cid, role, dev_info
 * @param out:ses_id
 * @return result of creating session
 * @details
 * 1. Validate input parameters
 * 2. Register session and assign buffer for message queue
 */
int32_t ipc_trans_create_session(const uint8_t sid, const uint8_t fid,
				 const uint8_t cid, const uint8_t ccid, const uint8_t role,
				 uint8_t *ses_id, void *dev_info)
{
	int8_t ret = -1;
	ses_base_t ses_info = { 0 };

	if (!dev_info || !ses_id)
		return ret;

	IPC_LOG_DEBUG("create session fid: %u, sid: %u, cid: %u, role: %u, ccid: %u", fid,
		      sid, cid, role, ccid);

	if (sid >= SESSION_COUNT || fid >= CHANNEL_COUNT) {
		IPC_LOG_WARNING(
			"create session sid or fid overrange, sid %u, fid %u",
			sid, fid);
		return -ERR_CREATE_SES_OUT_RANGE;
	}
#if defined(MULTI_DIE_HW_VERSION)
	if (ccid > MULTI_DIE_CHIP_1) {
		IPC_LOG_WARNING(
			"create session ccid %u overrange", ccid);
		return -ERR_CREATE_SES_OUT_RANGE;
	}
#endif
	if (role >= MSGBX_SES_ROLE_MAX) {
		IPC_LOG_WARNING("create session is invalid role %u", role);
		return -ERR_CREATE_SES_ROLE_INVALID;
	}

	ses_info.sid = sid;
	ses_info.fid = fid;
	ses_info.cid = cid;
	ses_info.role = role;
#if defined(MSGBX_HW_TYPE_A2000)
	ses_info.ccid = ccid;
#endif

	ret = session_register(ses_info, ses_id, dev_info);
	if (ret < 0) {
		IPC_LOG_WARNING("create session fail, ret:%d", ret);
		return -ERR_CREATE_SES_FAIL;
	}
	IPC_LOG_DEBUG("create session %d success", *ses_id);
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_send_msg
 * @param in: ses_id, msg, type, dev_info
 * @return send result
 * @details
 * 1. Validate inputs
 * 2. Transmit message
 */
int32_t ipc_trans_send_msg(const uint8_t ses_id, serdes_t *msg,
			   const uint8_t type, void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = (msgbx_end_device_t *)dev_info;
	ipc_ses_t *ses = NULL;
	uint8_t cnt = 0;
	rw_msg_t *rwmsg = NULL;
	uint8_t cpuid = 0;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *dbg_info = NULL;
#endif

	if (!module || !msg)
		return ret;

	if (type >= MSGBX_MSG_TYPE_MAX)
		return -ERR_TYP_IS_INVALID;

	if (msg->header.pid != module->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// note: remove pid can not equal cid check senario for message loopback feature
	if (end_is_valid(msg->header.cid) == 0)
		return -ERR_CID_IS_INVALID;

	ses = get_session(ses_id, dev_info);
	if (!ses) {
		IPC_LOG_INFO("ses %d is invalid", ses_id);
		return -ERR_SES_IS_INVALID;
	}
#ifndef REMOVE_STS_MGT
	if (msg->header.typ != MSGBX_MSG_TYPE_REPLY) {
#if defined(MSGBX_HW_TYPE_C1200)
		ret = ipc_end_is_ready(msg->header.cid, 0, dev_info);
#elif defined(MSGBX_HW_TYPE_A2000)
		ret = ipc_end_is_ready(msg->header.cid, msg->header.chip_cid, dev_info);
#endif
		if (ret < 0) {
			IPC_LOG_INFO("ses %d send msg dst %d is not ready", ses_id,
					msg->header.cid);
			return -ERR_DES_IS_OFFLINE;
		}
	}
#endif

	// set local variables.
	rwmsg = &msg->msg_pool[0];
	cpuid = module->g_ipc_cpuid;
	IPC_LOG_DEBUG("ses %d send msg typ: %d, cmd: %d, tok: %d, idx cnt: %d",
		      ses_id, type, msg->header.cmd, msg->header.tok,
		      msg->index);

#ifdef DEBUG_MODE_ENABLE
	dbg_info = &ses->debug_info;
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(cpuid, &dbg_info->send_start_time);
#endif
#endif

	for (cnt = 0; cnt <= msg->index; ++cnt) {
		ret = ipc_hw_layer_send_msg(cpuid, rwmsg);
		if (ret < 0) {
			IPC_LOG_WARNING("ses id %d hw send msg fail ret: %d",
					ses_id, ret);
#ifdef DEBUG_MODE_ENABLE
			++dbg_info->send_fail_cnt;
#endif
			return -ERR_SEND_MSG_FAIL;
		}
#ifdef DEBUG_MODE_ENABLE
		++dbg_info->send_rw_msg_cnt;
#endif
		if (rwmsg->header.is_eof == 1)
			break;
		++rwmsg;
	}

#ifdef DEBUG_MODE_ENABLE
	ATOMIC_FETCH_ADD(&dbg_info->send_msg_cnt, 1, __ATOMIC_SEQ_CST);
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(cpuid, &dbg_info->send_end_time);
#endif
#endif
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_get_msg
 * @param in: ses_id, msg_type, dev_info
 * @param out: msg
 * @return receive message result
 * @details
 * 1. Validate inputs
 * 2. Get message from session message queue
 */
int32_t ipc_trans_get_msg(const uint8_t ses_id, serdes_t *msg, void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = (msgbx_end_device_t *)dev_info;
	ipc_ses_t *ses = NULL;
	uint8_t cpuid = 0;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *dbg_info = NULL;
#endif

	if (!msg || !module)
		return ret;

	ses = get_session(ses_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	cpuid = module->g_ipc_cpuid;
#ifdef DEBUG_MODE_ENABLE
	dbg_info = &ses->debug_info;
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(cpuid, &dbg_info->get_msg_time);
#endif
#endif

	ret = session_msg_out(ses, msg);
	if (ret < 0)
		return -ERR_RECV_MSG_FAIL;

#ifdef DEBUG_MODE_ENABLE
	if (msg->header.typ == MSGBX_MSG_TYPE_METHOD || msg->header.typ == MSGBX_MSG_TYPE_REPLY)
		++dbg_info->recv_msg_1_cnt;
	else
		++dbg_info->recv_msg_2_cnt;
#endif

	ipc_hw_layer_get_time(cpuid, &msg->recv_get_time);
	IPC_LOG_DEBUG("ses id %d get msg  typ: %d, tok: %d, cmd: %d, idx: %d",
		      ses_id, msg->header.typ, msg->header.tok, msg->header.cmd,
		      msg->rcv_index);
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_close_session
 * @param in: ses_id, dev_info
 * @param out:
 * @return receive message result
 * @details
 * 1. Validate inputs
 * 2. Close session and clear relative buffer
 */
int32_t ipc_trans_close_session(const uint8_t ses_id, void *dev_info)
{
	int8_t ret = -1;
	ipc_ses_t *ses = NULL;

	if (!dev_info)
		return ret;

	ses = get_session(ses_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	ret = session_destroy(ses);
	if (ret < 0) {
		IPC_LOG_WARNING("sid %d destroy fail, ret: %d", ses_id, ret);
		return -ERR_SES_CLOSE_FAIL;
	}

	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_err_hdl
 * @param in: type, id, hdl, dev_info
 * @param out:
 * @return fault handle
 * @details
 * 1. Send command to hardware layer
 */
int32_t ipc_trans_err_hdl(const uint8_t type, const uint8_t id,
			  const uint32_t hdl, void *dev_info)
{
#ifdef IPC_STATE_MGT_ENABLE
	msgbx_end_device_t *module = NULL;
	int8_t ret = -1;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ret = ipc_hw_layer_err_hdl(module->g_ipc_cpuid, id, hdl);
	if (ret < 0) {
		IPC_LOG_INFO("hw err handle fail ret: %d", ret);
		return -ERR_ERR_HANDLE_FAIL;
	}
#endif
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_get_debug_info
 * @param in: ses_id, dev_info
 * @param out: info
 * @return get session debug information
 * @details
 * 1. Validate inputs
 */
int32_t ipc_trans_get_debug_info(const uint8_t ses_id, debug_info_t *info,
				 void *dev_info)
{
#ifdef DEBUG_MODE_ENABLE
	ipc_ses_t * ses = NULL;
	if (!info || !dev_info)
		return -1;

	ses = get_session(ses_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	*info = ses->debug_info;
#endif
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_transmit_log
 * @param in: msg, cid, dev_info
 * @param out:
 * @return special requirement for log transmission to central msgend
 * @details
 * 1. Validate inputs
 */
int32_t ipc_trans_transmit_log(const uint8_t cid, const char *log,
			       void *dev_info)
{
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	serdes_t msg = { 0 };
	uint8_t cnt = 0;

	if (!dev_info || !log)
		return ret;

	module = (msgbx_end_device_t *)dev_info;
	ipc_ser_init(&msg);
	msg.header.res = 0;
	msg.header.typ = MSGBX_MSG_TYPE_PROTOCOL;
	msg.header.cid = cid;
	msg.header.pid = module->g_ipc_pid;
	msg.header.sid = 0;
	msg.header.fid = 0;
	msg.header.cmd = TRANS_PROTO_CMD_LOG;

	ret = ipc_ser_put_string(&msg, log);
	if (ret < 0)
		IPC_LOG_INFO("transmit log ipc_ser_put_string fail\n");
	msg.header.tok = (module->g_log_tok++) & 15;
	ipc_ser_finish(&msg);

	if (cid == module->g_ipc_pid || end_is_valid(cid) == 0)
		return -ERR_CID_IS_INVALID;

	for (cnt = 0; cnt <= msg.index; ++cnt) {
		ret = ipc_hw_layer_send_msg(module->g_ipc_cpuid,
					    &msg.msg_pool[cnt]);
		if (ret < 0) {
			IPC_LOG_WARNING("trans log hw send msg fail ret: %d",
					ret);
			return -ERR_SEND_MSG_FAIL;
		}
	}
#endif
	return RESULT_SUCCESS;
}

/**
 * @name  ipc_trans_send_rwmsg
 * @param in: ses_id, msg, dev_info
 * @return send result
 * @details
 * 1. Validate inputs
 * 2. Transmit message
 */
int32_t ipc_trans_send_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			     void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = (msgbx_end_device_t *)dev_info;
	ipc_ses_t * ses = NULL;
	uint8_t cpuid = 0;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *dbg_info = NULL;
#endif

	if (!module || !msg)
		return ret;

	if (msg->header.pid != module->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	if (end_is_valid(msg->header.cid) == 0)
		return -ERR_CID_IS_INVALID;

#ifndef REMOVE_STS_MGT
#if defined(MSGBX_HW_TYPE_C1200)
	ret = ipc_end_is_ready(msg->header.cid, 0, dev_info);
#elif defined(MSGBX_HW_TYPE_A2000)
	ret = ipc_end_is_ready(msg->header.cid, msg->header.chip_cid, dev_info);
#endif
	if (ret < 0) {
		IPC_LOG_INFO("ses %d send msg dst %d is not ready", ses_id,
			     msg->header.cid);
		return -ERR_DES_IS_OFFLINE;
	}
#endif

	ses = get_session(ses_id, dev_info);
	if (!ses) {
		IPC_LOG_INFO("ses %d is invalid", ses_id);
		return -ERR_SES_IS_INVALID;
	}

	// set local variables.
	cpuid = module->g_ipc_cpuid;

	IPC_LOG_DEBUG("ses %d send msg cmd: %d, tok: %d fid: %d, sid: %d",
		      ses_id, msg->header.cmd, msg->header.tok, msg->header.fid,
		      msg->header.sid);

#ifdef DEBUG_MODE_ENABLE
	dbg_info = &ses->debug_info;
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(cpuid, &dbg_info->send_start_time);
#endif
#endif

	ret = ipc_hw_layer_send_msg(cpuid, msg);
	if (ret < 0) {
		IPC_LOG_WARNING("ses id %d hw send msg fail ret: %d", ses_id,
				ret);
#ifdef DEBUG_MODE_ENABLE
		// ATOMIC_FETCH_ADD(&dbg_info->send_fail_cnt, 1, __ATOMIC_SEQ_CST);
		++dbg_info->send_fail_cnt;
#endif
		return -ERR_SEND_MSG_FAIL;
	}
#ifdef DEBUG_MODE_ENABLE
	// ATOMIC_FETCH_ADD(&dbg_info->send_rw_msg_cnt, 1, __ATOMIC_SEQ_CST);
	++dbg_info->send_rw_msg_cnt;
#endif

#ifdef DEBUG_MODE_ENABLE
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(module->g_ipc_cpuid,
			      &ses->debug_info.send_end_time);
#endif
#endif

	return RESULT_SUCCESS;
}

int32_t ipc_trans_get_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			    uint64_t *timestamp, void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = (msgbx_end_device_t *)dev_info;
	ipc_ses_t *ses = NULL;
	uint8_t cpuid = 0;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *dbg_info = NULL;
#endif

	if (!msg || !module)
		return ret;

	ses = get_session(ses_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	cpuid = module->g_ipc_cpuid;
#ifdef DEBUG_MODE_ENABLE
	dbg_info = &ses->debug_info;
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(cpuid, &dbg_info->get_msg_time);
#endif
#endif

	ret = session_rwmsg_out(ses, msg);
	if (ret < 0)
		return -ERR_RECV_MSG_FAIL;

#ifdef DEBUG_MODE_ENABLE
	++dbg_info->recv_msg_1_cnt;
#endif

	ipc_hw_layer_get_time(cpuid, timestamp);
	IPC_LOG_DEBUG("ses id %d get msg  typ: %d, tok: %d, cmd: %d, pid: %d",
		      ses_id, msg->header.typ, msg->header.tok, msg->header.cmd,
		      msg->header.pid);
	return RESULT_SUCCESS;
}

int32_t ipc_trans_get_hw_count(const uint8_t fid, msgbox_hw_counter_t* hw_cnt, 
				void *dev_info)
{
#if defined(MSGBX_HW_TYPE_A2000)
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	if (!dev_info || !hw_cnt)
		return ret;

	if (fid >= module->g_ipc_flt_cnt)
		return ret;

	module = (msgbx_end_device_t *)dev_info;
	ret = ipc_hw_layer_get_hw_counter(module->g_ipc_cpuid, fid, hw_cnt);
	return ret;
#endif
	return RESULT_SUCCESS;
}

int32_t ipc_trans_clr_hw_count(const uint8_t fid, const uint8_t clr_mask, 
				void *dev_info)
{
#if defined(MSGBX_HW_TYPE_A2000)
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	if (!dev_info)
		return ret;

	if (fid >= module->g_ipc_flt_cnt)
		return ret;

	module = (msgbx_end_device_t *)dev_info;
	ret = ipc_hw_layer_clr_hw_counter(module->g_ipc_cpuid, fid, clr_mask);
	return ret;
#endif
	return RESULT_SUCCESS;
}

int32_t ipc_trans_register_method(const uint8_t session_id, const uint8_t cmd,
				  void *dev_info)
{
#if (SESSION_COUNT > 1)
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;
	ipc_ses_t *ses = NULL;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ses = get_session(session_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	// spec note: even cmd value is over 256, compiler will optimize and truncate it.
	if (module->method_register_map[cmd] >= 0)
		return -ERR_REGISTER_METHOD_REPEATE;

	module->method_register_map[cmd] = session_id;
	IPC_LOG_DEBUG("ses %d register cmd %d success", session_id, cmd);
#endif
#endif
	return RESULT_SUCCESS;
}

int32_t ipc_trans_unregister_method(const uint8_t session_id, void *dev_info)
{
#if (SESSION_COUNT > 1)
#if !defined(BAREMETAL_VERSION_TRUNCATE)
	msgbx_end_device_t *module = NULL;
	ipc_ses_t *ses = NULL;
	uint8_t cnt = 0;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ses = get_session(session_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	for (cnt = 0; cnt < CMD_MAX_COUNT; ++cnt)
		if (module->method_register_map[cnt] == session_id)
			module->method_register_map[cnt] = -1;
#endif
#endif
	return RESULT_SUCCESS;
}

int32_t ipc_trans_map_session(const uint8_t ses_id, void **msg_queue_addr, void *dev_info)
{
	ipc_ses_t *ses = NULL;

	if (!dev_info)
		return -1;

	ses = get_session(ses_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	*msg_queue_addr = ses->msg_queue;
	ses->msg_queue->k_addr = (uintptr_t)ses->msg_queue;
#else
	*msg_queue_addr = &ses->msg_queue;
	ses->msg_queue.k_addr = (uintptr_t)&ses->msg_queue;
#endif
	return RESULT_SUCCESS;
}

int32_t ipc_trans_msg_queue_alloc(const uint8_t session_id, const void* addr, void* dev_info)
{
#if defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses = NULL;

	if (!dev_info || !addr)
		return -ERR_CREATE_SES_ALLOC_FAIL;

	ses = get_session(session_id, dev_info);
	if (!ses)
		return -ERR_SES_IS_INVALID;

	ses->msg_queue = (bst_msg_queue_t*)addr;
	ses->msg_queue->k_addr = (uintptr_t)addr;
	msg_queue_init(ses->msg_queue);
#endif
	return RESULT_SUCCESS;
}