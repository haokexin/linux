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
 * @file  ipc_trans_ses_mgt.c
 * @brief this file is used as session management
 * @note
 * @details feature list
 */
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>
#include "ipc_trans_ses_mgt.h"
#include "../include/config.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_utl.h"

int8_t session_mgt_init(void *addr)
{
	msgbx_end_device_t *module = NULL;
	uint8_t fid_cnt = 0, sid_cnt = 0;
	ipc_ses_t **ses_map = NULL;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	for (fid_cnt = 0; fid_cnt < CHANNEL_COUNT; ++fid_cnt) {
		for (sid_cnt = 0; sid_cnt < SESSION_COUNT; ++sid_cnt) {
			ipc_memset(&ses_map[fid_cnt][sid_cnt].info,
				   SES_INFO_INVALID_VAL, sizeof(ses_base_t));
			ses_map[fid_cnt][sid_cnt].cid_idx = 0;
			ses_map[fid_cnt][sid_cnt].update_flag = 0;
			ses_map[fid_cnt][sid_cnt].status =
				SES_STATE_INVALID;
			ses_map[fid_cnt][sid_cnt].ses_type =
				MSGBX_SES_ROLE_INVALID;
#ifdef DEBUG_MODE_ENABLE
			ipc_memset(&module->debug_info[fid_cnt][sid_cnt], 0,
				   sizeof(debug_info_t));
#endif
		}
	}

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	ses_map[REMOTE_LOG_SES_FID][REMOTE_LOG_SES_SID].info.role =
		MSGBX_SES_ROLE_SERVER;
	ses_map[REMOTE_LOG_SES_FID][REMOTE_LOG_SES_SID].info.sid =
		REMOTE_LOG_SES_SID;
	msg_queue_init(&ses_map[REMOTE_LOG_SES_FID][REMOTE_LOG_SES_SID]
				.server_buf.method_buf);
#endif

	return 0;
}

// note: sid valid range is from 0 ~ 15
int8_t session_isvalid(const uint8_t session_id, void *addr)
{
	msgbx_end_device_t *module = NULL;
	uint8_t sid = 0, fid = 0;
		ipc_ses_t **ses_map = NULL;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	session_dist(session_id, &sid, &fid);
	if (sid >= SESSION_COUNT || fid >= CHANNEL_COUNT)
		return -1;
	if (ses_map[fid][sid].ses_type == MSGBX_SES_ROLE_INVALID)
		return -2;
	if (ses_map[fid][sid].status < SES_STATE_AVAIL)
		return 1;

	return 0;
}

int8_t session_register(const ses_base_t info, uint8_t *session_id, void *addr)
{
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	// sid valid check
	if (info.sid >= SESSION_COUNT || info.fid >= CHANNEL_COUNT)
		return -2;

	if (ses_map[info.fid][info.sid].status != SES_STATE_INVALID) {
		IPC_LOG_WARNING("sid: %d, fid: %d is occupied\n", info.sid,
			     info.fid);
		return -3;
	}

	ses_map[info.fid][info.sid].info.sid = info.sid;
	ses_map[info.fid][info.sid].info.fid = info.fid;
	ses_map[info.fid][info.sid].info.cid = info.cid;
	ses_map[info.fid][info.sid].info.role = info.role;
	ses_map[info.fid][info.sid].cid_idx =
		query_end_id_idx(info.cid);
	if (info.role == MSGBX_SES_ROLE_SERVER ||
	    info.role == MSGBX_SES_ROLE_FASTPATH) {
		ses_map[info.fid][info.sid].status = SES_STATE_AVAIL;
		msg_queue_init(&ses_map[info.fid][info.sid]
					.server_buf.method_buf);
	} else {
#ifndef REMOVE_STS_MGT
		ses_map[info.fid][info.sid].status =
			ipc_end_is_ready(info.cid, addr) < 0 ?
				SES_STATE_NOT_AVAIL :
				SES_STATE_AVAIL;
		if (ses_map[info.fid][info.sid].status ==
		    SES_STATE_AVAIL)
			ses_map[info.fid][info.sid].update_flag |=
				(1 << SES_UPDATE_STS_BIT);
#else
		ses_map[info.fid][info.sid].status = SES_STATE_AVAIL;
#endif
		if (info.cid == 0)
			ses_map[info.fid][info.sid].status =
				SES_STATE_AVAIL;
		msg_queue_init(&ses_map[info.fid][info.sid]
					.client_buf.reply_buf);
		msg_queue_init(&ses_map[info.fid][info.sid]
					.client_buf.signal_buf);
	}
	ses_map[info.fid][info.sid].ses_type = info.role;
#ifdef DEBUG_MODE_ENABLE
	module->debug_info[info.fid][info.sid].role = info.role;
#endif
	session_comb(info.sid, info.fid, session_id);

	return 0;
}

static int8_t session_map_clear(uint8_t sid, uint8_t fid, void *addr)
{
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	ipc_memset(&ses_map[fid][sid].info, SES_INFO_INVALID_VAL,
		   sizeof(ses_base_t));
	ses_map[fid][sid].cid_idx = 0;
	ses_map[fid][sid].status = SES_STATE_INVALID;
	ses_map[fid][sid].ses_type = MSGBX_SES_ROLE_INVALID;
	ses_map[fid][sid].update_flag = 0;

#ifdef DEBUG_MODE_ENABLE
	ipc_memset(&module->debug_info[fid][sid], 0, sizeof(debug_info_t));
#endif
	return 0;
}

int8_t session_destroy(const uint8_t session_id, void *addr)
{
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
	uint8_t sid = 0, fid = 0;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	if (session_isvalid(session_id, addr) < 0)
		return -2;

	session_dist(session_id, &sid, &fid);
	ses_map[fid][sid].status = SES_STATE_DESTROY;
	if (ses_map[fid][sid].ses_type == MSGBX_SES_ROLE_CLIENT) {
		msg_queue_init(&ses_map[fid][sid].client_buf.reply_buf);
		msg_queue_init(
			&ses_map[fid][sid].client_buf.signal_buf);
	} else {
		msg_queue_init(
			&ses_map[fid][sid].server_buf.method_buf);
	}

	session_map_clear(sid, fid, addr);
	return 0;
}

int8_t session_msg_in(const uint8_t session_id, const rw_msg_t *msg, void *addr)
{
	int32_t ret = -1;
	uint64_t timestamp = 0;
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
	uint8_t sid = 0, fid = 0;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return ret;

	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	if (session_isvalid(session_id, addr) < 0)
		return -2;

	session_dist(session_id, &sid, &fid);
#ifdef DEBUG_MODE_ENABLE
	ATOMIC_FETCH_ADD(&(module->debug_info[fid][sid].recv_rw_msg_cnt), 1,
			 __ATOMIC_SEQ_CST);
#endif

	ipc_hw_layer_get_time(module->ops.cpuid, &timestamp);

	if (ses_map[fid][sid].status == SES_STATE_DESTROY)
		return -3;

	ses_map[fid][sid].update_flag |= (1 << SES_UPDATE_MSG_BIT);
	if (ses_map[fid][sid].info.role == MSGBX_SES_ROLE_CLIENT) {
		// put msg in different queue according to msg type
		if (msg->header.typ == MSGBX_MSG_TYPE_REPLY) {
			ret = msg_queue_in(&(ses_map[fid][sid]
						     .client_buf.reply_buf),
					   (rw_msg_t *)msg, timestamp);
			return ((ret != 0) ? -4 : 0);
		}
		if (msg->header.typ == MSGBX_MSG_TYPE_BROADCAST) {
			ret = msg_queue_in(&(ses_map[fid][sid]
						     .client_buf.signal_buf),
					   (rw_msg_t *)msg, timestamp);
			return ((ret != 0) ? -5 : 0);
		}
	} else {
		ret = msg_queue_in(
			&(ses_map[fid][sid].server_buf.method_buf),
			(rw_msg_t *)msg, timestamp);
		return ((ret != 0) ? -6 : 0);
	}

	return 0;
}

int8_t session_msg_out(const uint8_t session_id, const uint8_t type,
		       serdes_t *msg, void *addr)
{
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
	uint8_t sid = 0, fid = 0;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return ret;

	if (session_isvalid(session_id, addr) < 0)
		return -2;

	session_dist(session_id, &sid, &fid);
	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	if (ses_map[fid][sid].status == SES_STATE_DESTROY)
		return -3;

	if (ses_map[fid][sid].info.role == MSGBX_SES_ROLE_CLIENT) {
		// put msg in different queue according to msg type
		if (type == MSGBX_MSG_TYPE_REPLY) {
			ret = msg_queue_out(&(ses_map[fid][sid]
						      .client_buf.reply_buf),
					    msg);
			return ((ret != 0) ? -3 : 0);
		}
		if (type == MSGBX_MSG_TYPE_BROADCAST) {
			ret = msg_queue_out(&(ses_map[fid][sid]
						      .client_buf.signal_buf),
					    msg);
			return ((ret != 0) ? -4 : 0);
		} else
			return -5;
	} else {
		ret = msg_queue_out(
			&(ses_map[fid][sid].server_buf.method_buf),
			msg);
		return ((ret != 0) ? -6 : 0);
	}

	return 0;
}

int32_t ipc_trans_get_avail_info(const uint8_t session_id, uint8_t *type,
				 void *dev_info)
{
	int8_t ret = -1;
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
	uint8_t sid = 0, fid = 0;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!dev_info)
		return ret;

	if (session_isvalid(session_id, dev_info) < 0) {
		*type = 0;
		return -ERR_SES_IS_INVALID;
	}

	session_dist(session_id, &sid, &fid);
	module = (msgbx_end_device_t *)dev_info;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

#if defined(TIMESTAMP_DEBUG_ENABLE) && defined(DEBUG_MODE_ENABLE)
	ipc_hw_layer_get_time(module->ops.cpuid,
			      &module->debug_info[fid][sid].collate_time);
#endif

#ifndef REMOVE_STS_MGT
	if (ses_map[fid][sid].update_flag & (1 << SES_UPDATE_STS_BIT)) {
		ret = ses_map[fid][sid].status == SES_STATE_AVAIL ?
			      QUERY_INFO_DST_STS_ONLINE :
			      QUERY_INFO_DST_STS_OFFLINE;
		ses_map[fid][sid].update_flag &=
			~(1 << SES_UPDATE_STS_BIT);
		return ret;
	}
#endif

	if (ses_map[fid][sid].info.role == MSGBX_SES_ROLE_CLIENT) {
		if (!msg_queue_collate(
			    &(ses_map[fid][sid].client_buf.reply_buf),
			    0)) {
			*type = MSGBX_MSG_TYPE_REPLY;
			ret = QUERY_INFO_HAS_AVAIL_RECV_MSG;
		}
		if (!msg_queue_collate(
			    &(ses_map[fid][sid].client_buf.signal_buf),
			    0)) {
			*type = MSGBX_MSG_TYPE_BROADCAST;
			ret = QUERY_INFO_HAS_AVAIL_RECV_MSG;
		}
	} else if (ses_map[fid][sid].info.role == MSGBX_SES_ROLE_SERVER) {
		if (!msg_queue_collate(
			    &(ses_map[fid][sid].server_buf.method_buf),
			    0)) {
			*type = MSGBX_MSG_TYPE_METHOD;
			ret = QUERY_INFO_HAS_AVAIL_RECV_MSG;
		}
	} else
		ret = -ERR_TYP_IS_INVALID;

	return ret;
}

int8_t session_rwmsg_out(const uint8_t session_id, rw_msg_t *msg, void *addr)
{
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;
	ipc_ses_t **ses_map = NULL;
	uint8_t sid = 0, fid = 0;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	ipc_ses_t *ses_map_tmp[CHANNEL_COUNT];
	int i = 0;
#endif

	if (!addr)
		return ret;

	if (session_isvalid(session_id, addr) < 0)
		return -2;

	session_dist(session_id, &sid, &fid);
	module = (msgbx_end_device_t *)addr;

#if defined(USE_EXTERNAL_MSG_BUFFER)
	if (!module->ses_map)
		return -1;
	ses_map = (ipc_ses_t **)module->ses_map;
#else
    for (i = 0; i < CHANNEL_COUNT; i++) {
        ses_map_tmp[i] = module->ses_map[i];
    }
	ses_map = ses_map_tmp;
#endif

	if (ses_map[fid][sid].status == SES_STATE_DESTROY)
		return -3;

	ret = rwmsg_queue_out(
		&(ses_map[fid][sid].server_buf.method_buf), msg);

	return ((ret != 0) ? -4 : 0);
}
