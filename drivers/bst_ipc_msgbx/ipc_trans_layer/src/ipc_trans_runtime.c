// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * This program is free software; you can redistribute it and/or modify
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
 * @file  ipc_trans_runtime.c
 * @brief This file serves as the API for the IPC transferring layer,
 * facilitating direct usage by ipc_app_layer or enabling driver developers to
 * implement ipc_trans_impl.c for OS-specific adaptations.
 * @note
 * @details feature list
 * 1.
 */
#include <bst/ipc_trans_layer.h>
#include "config.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_routing.h"
#include "ipc_trans_sts_mgt.h"
#include "ipc_trans_utl.h"

/**
 * @name  ipc_trans_init
 * @param in :role, err_func, dev_ifno
 * @param out:
 * @return result of initiating trans layer
 * @details
 * 1. Initialize MsgBx hardware layer.
 * 2. Configure state management, if required.
 * 3. Establish trans layer data structure.
 * 4. Signal availability of MsgBx end message.
 */
int32_t ipc_trans_init(const uint8_t role, err_msg_callback err_func,
		       void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	ipc_init_params_t init_params = { 0 };
	msgbx_hw_info_t hw_info = { 0 };

	if (!dev_info)
		return ret;
	module = (msgbx_end_device_t *)dev_info;

	init_params.mbx_device = IPC_HW_MSGBX_MODE;
#ifdef IPC_STATE_MGT_ENABLE
	init_params.msgbx_end_mgt_flag = MSG_END_MGT_CONFIG;
#else
	init_params.msgbx_end_mgt_flag = 0;
#endif

/* Note: This is a demo-specific configuration that designates the receiver.
 * In a practical scenario, this parameter might not be necessary.
 */
	if (role == 0)
		init_params.mbx_type = MSGBX_ROLE_SERVER_ONLY;
	else
		init_params.mbx_type = MSGBX_ROLE_CLIENT_ONLY;

	ret = ipc_hw_layer_init(module->ops.cpuid, &init_params, &hw_info);
	if (ret < 0)
		return -ERR_TRANS_INIT_FAIL;

	// update global info
	module->g_ipc_pid = hw_info.mbx_end_id;
	module->g_ipc_flt_cnt = hw_info.mbx_flt_cnt;
#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	module->g_log_tok = 0;
#endif
	IPC_LOG_INFO("init msgbx filter cnt = %d, pid = %d",
		     module->g_ipc_flt_cnt, module->g_ipc_pid);

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
	ret = ipc_hw_layer_err_msg_register(module->ops.cpuid, err_func);
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
#ifdef IPC_STATE_MGT_ENABLE
		.msgbx_end_mgt_flag = MSG_END_MGT_CONFIG,
#else
		.msgbx_end_mgt_flag = 0,
#endif
		.mbx_type = MSGBX_ROLE_SERVER_ONLY,
	};

	if (unlikely(!dev_info))
		return ret;
	module = (msgbx_end_device_t *)dev_info;

	ret = ipc_hw_layer_init(module->ops.cpuid, &init_params, &hw_info);
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

	return ipc_hw_layer_deinit(module->ops.cpuid);
}

/**
 * @name  ipc_trans_create_session
 * @param in :sid, fid, cid, role, dev_info
 * @param out:ses_id
 * @return result of creating session
 * @details
 * 1. Validate input parameters
 * 2. Register session and allocate buffer for message queue
 * 3. Define message box filter dispatch rules
 */
int32_t ipc_trans_create_session(const uint8_t sid, const uint8_t fid,
				 const uint8_t cid, const uint8_t role,
				 uint8_t *ses_id, void *dev_info)
{
	int8_t ret = -1;
	ses_base_t ses_info = { 0 };

	if (!dev_info)
		return ret;

	IPC_LOG_DEBUG("create session fid: %d, sid: %d, cid: %d, role: %d", fid,
		      sid, cid, role);
	// input param check
	if (sid >= SESSION_COUNT || fid >= CHANNEL_COUNT) {
		IPC_LOG_WARNING(
			"create session sid or fid overrange, sid %d, fid %d",
			sid, fid);
		return -ERR_CREATE_SES_OUT_RANGE;
	}

	if (role >= MSGBX_SES_ROLE_MAX) {
		IPC_LOG_WARNING("create session is invalid role %d", role);
		return -ERR_CREATE_SES_ROLE_INVALID;
	}

	// check filter rule setting
	ret = set_flt_rules(fid, dev_info);
	if (ret < 0)
		return -ERR_SET_FIL_RULE_FAIL;

	// create session buffer
	ses_info.sid = sid;
	ses_info.fid = fid;
	ses_info.cid = cid;
	ses_info.role = role;
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
	msgbx_end_device_t *module = NULL;
	uint8_t cnt = 0;
#ifdef DEBUG_MODE_ENABLE
	uint8_t fid = 0, sid = 0;
	session_dist(ses_id, &sid, &fid);
#endif

	if (!dev_info || !msg)
		return ret;

	module = (msgbx_end_device_t *)dev_info;

	if (type >= MSGBX_MSG_TYPE_MAX)
		return -ERR_TYP_IS_INVALID;

	if (msg->header.pid != module->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	// note: remove this check senario for message loopback feature
	// if (msg->header.pid == msg->header.cid)
	//	return -ERR_CID_EQUAL_PID;

	if (end_is_valid(msg->header.cid) == 0)
		return -ERR_CID_IS_INVALID;

	// input param check
	ret = session_isvalid(ses_id, dev_info);
	if (ret < 0) {
		IPC_LOG_INFO("ses %d is invalid, ret: %d", ses_id, ret);
		return -ERR_SES_IS_INVALID;
	}
#if 0
	ret = ipc_end_is_ready(msg->header.cid, dev_info);
	if (ret < 0) {
		IPC_LOG_INFO("ses %d send msg dst %d is not ready", ses_id,
			     msg->header.cid);
		return -ERR_DES_IS_OFFLINE;
	}
#endif

	IPC_LOG_DEBUG("ses %d send msg typ: %d, cmd: %d, tok: %d, idx cnt: %d",
		      ses_id, type, msg->header.cmd, msg->header.tok,
		      msg->index);

#if defined(TIMESTAMP_DEBUG_ENABLE) && defined(DEBUG_MODE_ENABLE)
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].send_start_time);
#endif

	// send package msg

	for (cnt = 0; cnt <= msg->index; ++cnt) {
		msg->msg_pool[cnt].header.ver = TRANS_LAYER_VERSION;
		msg->msg_pool[cnt].header.typ = type;
		msg->msg_pool[cnt].header.cid = msg->header.cid;
		msg->msg_pool[cnt].header.pid = msg->header.pid;

		// call hw_layer to send
		ret = ipc_hw_layer_send_msg(module->ops.cpuid,
					    &msg->msg_pool[cnt]);
		if (ret < 0) {
			IPC_LOG_WARNING("ses id %d hw send msg fail ret: %d",
					ses_id, ret);
#ifdef DEBUG_MODE_ENABLE
			ATOMIC_FETCH_ADD(
				&(module->debug_info[fid][sid].send_fail_cnt),
				1, __ATOMIC_SEQ_CST);
#endif
			return -ERR_SEND_MSG_FAIL;
		}
#ifdef DEBUG_MODE_ENABLE
		ATOMIC_FETCH_ADD(
			&(module->debug_info[fid][sid].send_rw_msg_cnt), 1,
			__ATOMIC_SEQ_CST);
#endif
		if (msg->msg_pool[cnt].header.is_eof == 1)
			break;
	}

#ifdef DEBUG_MODE_ENABLE
	ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].send_msg_cnt), 1,
			 __ATOMIC_SEQ_CST);
	module->debug_info[fid][sid].send_frame_cnt = cnt;
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].send_end_time);
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
int32_t ipc_trans_get_msg(const uint8_t ses_id, const ipc_msg_type_t msg_typ,
			  serdes_t *msg, void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
#ifdef DEBUG_MODE_ENABLE
	uint8_t sid = 0, fid = 0;
	session_dist(ses_id, &sid, &fid);
#endif

	if (!msg || !dev_info)
		return ret;

	if (msg_typ > MSGBX_MSG_TYPE_BROADCAST)
		return -ERR_TYP_IS_INVALID;

	if (session_isvalid(ses_id, dev_info) < 0)
		return -ERR_SES_IS_INVALID;

	module = (msgbx_end_device_t *)dev_info;
#if defined(TIMESTAMP_DEBUG_ENABLE) && defined(DEBUG_MODE_ENABLE)
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].get_msg_time);
#endif

	ret = session_msg_out(ses_id, msg_typ, msg, dev_info);
	if (ret < 0)
		return -ERR_RECV_MSG_FAIL;

#ifdef DEBUG_MODE_ENABLE
	if (msg_typ == MSGBX_MSG_TYPE_METHOD || msg_typ == MSGBX_MSG_TYPE_REPLY)
		ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].recv_msg_1_cnt), 1,
			__ATOMIC_SEQ_CST);
	else
		ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].recv_msg_2_cnt), 1,
			__ATOMIC_SEQ_CST);
#endif

	ipc_hw_layer_get_time(module->ops.cpuid, &msg->recv_get_time);
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

	if (!dev_info)
		return ret;

	if (session_isvalid(ses_id, dev_info) < 0)
		return -ERR_SES_IS_INVALID;

	ret = session_destroy(ses_id, dev_info);
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
	ret = ipc_hw_layer_err_hdl(module->ops.cpuid, type, id, hdl);
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
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	uint8_t sid = 0, fid = 0;

	if (!dev_info)
		return ret;

	if (session_isvalid(ses_id, dev_info) < 0)
		return -ERR_SES_IS_INVALID;

	module = (msgbx_end_device_t *)dev_info;
	session_dist(ses_id, &sid, &fid);
	*info = module->debug_info[fid][sid];
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
	msg.header.ver = TRANS_LAYER_VERSION;
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

	// send package msg
	for (cnt = 0; cnt <= msg.index; ++cnt) {
		// call hw_layer to send
		ret = ipc_hw_layer_send_msg(module->ops.cpuid,
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
	msgbx_end_device_t *module = NULL;
#ifdef DEBUG_MODE_ENABLE
	uint8_t fid = 0, sid = 0;
	session_dist(ses_id, &sid, &fid);
#endif

	if (!dev_info || !msg)
		return ret;

	module = (msgbx_end_device_t *)dev_info;

	if (msg->header.pid != module->g_ipc_pid)
		return -ERR_PID_IS_INVALID;

	if (end_is_valid(msg->header.cid) == 0)
		return -ERR_CID_IS_INVALID;

#ifndef REMOVE_STS_MGT
	ret = ipc_end_is_ready(msg->header.cid, dev_info);
	if (ret < 0) {
		IPC_LOG_INFO("ses %d send msg dst %d is not ready", ses_id,
			     msg->header.cid);
		return -ERR_DES_IS_OFFLINE;
	}
#endif

	// input param check
	ret = session_isvalid(ses_id, dev_info);
	if (ret < 0) {
		IPC_LOG_INFO("ses %d is invalid, ret: %d", ses_id, ret);
		return -ERR_SES_IS_INVALID;
	}

	IPC_LOG_DEBUG("ses %d send msg cmd: %d, tok: %d fid: %d, sid: %d",
		      ses_id, msg->header.cmd, msg->header.tok, msg->header.fid,
		      msg->header.sid);

#if defined(TIMESTAMP_DEBUG_ENABLE) && defined(DEBUG_MODE_ENABLE)
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].send_start_time);
#endif

	// send package msg
	msg->header.ver = TRANS_LAYER_VERSION;
	// call hw_layer to send
	ret = ipc_hw_layer_send_msg(module->ops.cpuid, msg);
	if (ret < 0) {
		IPC_LOG_WARNING("ses id %d hw send msg fail ret: %d", ses_id,
				ret);
#ifdef DEBUG_MODE_ENABLE
		ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].send_fail_cnt),
				 1, __ATOMIC_SEQ_CST);
#endif
		return -ERR_SEND_MSG_FAIL;
	}
#ifdef DEBUG_MODE_ENABLE
	ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].send_rw_msg_cnt), 1,
			 __ATOMIC_SEQ_CST);
#endif

#ifdef DEBUG_MODE_ENABLE
#ifdef TIMESTAMP_DEBUG_ENABLE
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].send_end_time);
#endif
#endif

	return RESULT_SUCCESS;
}

int32_t ipc_trans_get_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			    uint64_t *timestamp, void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
#ifdef DEBUG_MODE_ENABLE
	uint8_t sid = 0, fid = 0;
	session_dist(ses_id, &sid, &fid);
#endif

	if (!msg || !dev_info)
		return ret;

	if (session_isvalid(ses_id, dev_info) < 0)
		return -ERR_SES_IS_INVALID;

	module = (msgbx_end_device_t *)dev_info;
#if defined(TIMESTAMP_DEBUG_ENABLE) && defined(DEBUG_MODE_ENABLE)
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].get_msg_time);
#endif

	ret = session_rwmsg_out(ses_id, msg, dev_info);
	if (ret < 0)
		return -ERR_RECV_MSG_FAIL;

#ifdef DEBUG_MODE_ENABLE
	ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].recv_msg_1_cnt), 1,
			 __ATOMIC_SEQ_CST);
#endif

	ipc_hw_layer_get_time(module->ops.cpuid, timestamp);
	IPC_LOG_DEBUG("ses id %d get msg  typ: %d, tok: %d, cmd: %d, pid: %d",
		      ses_id, msg->header.typ, msg->header.tok, msg->header.cmd,
		      msg->header.pid);
	return RESULT_SUCCESS;
}
