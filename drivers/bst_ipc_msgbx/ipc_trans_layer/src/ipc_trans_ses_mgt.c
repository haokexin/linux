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
 * @file  ipc_trans_ses_mgt.c
 * @brief this file is used as session management
 * @note
 * @details feature list
 * 1. session map init
 * 2. session create and destroy
 * 3. push and pop messages in or from the corresponding session map
 */
#include "ipc_trans_ses_mgt.h"
#include "ipc_trans_msg_mgt.h"
#include "ipc_trans_sts_mgt.h"
#include "ipc_trans_utl.h"

int8_t session_mgt_init(void *addr)
{
#if !defined(DEV_MEM_INIT_OUTSIDE) || defined(ENABLE_REMOTE_LOG_PROCESS_FUNC)
	msgbx_end_device_t *module = NULL;
	ipc_ses_t *ses_map = NULL;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	ses_map = module->ses_map;

#if !defined(DEV_MEM_INIT_OUTSIDE) && !defined(USE_EXTERNAL_MSG_BUFFER)
	// memset the whole session map, to achieve better performance.
	ipc_memset(ses_map, 0, CHANNEL_COUNT * SESSION_COUNT * sizeof(ipc_ses_t));
#endif

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
	ses_map[REMOTE_LOG_SES_ID].info.role = MSGBX_SES_ROLE_SERVER;
	ses_map[REMOTE_LOG_SES_ID].info.sid = REMOTE_LOG_SES_SID;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	msg_queue_init(ses_map[REMOTE_LOG_SES_ID].msg_queue);
#else
	msg_queue_init(&ses_map[REMOTE_LOG_SES_ID].msg_queue);
#endif
#endif
#endif
	return 0;
}

int8_t session_register(const ses_base_t info, uint8_t *session_id, void *addr)
{
	msgbx_end_device_t *module = (msgbx_end_device_t *)addr;
	ipc_ses_t *ses_map = NULL;
	uint8_t ses_id = session_comb(info.sid, info.fid);

	if (!module || info.fid >= CHANNEL_COUNT || info.sid >= SESSION_COUNT || 
			info.role == MSGBX_SES_ROLE_INVALID || info.role >= MSGBX_SES_ROLE_MAX)
		return -1;

	ses_map = module->ses_map;

	if (ses_map[ses_id].status != SES_STATE_INVALID) {
		IPC_LOG_WARNING("sid: %d, fid: %d is occupied\n", info.sid,
			     info.fid);
		return -3;
	}
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	msg_queue_init(&ses_map[ses_id].msg_queue);
#endif

	ses_map[ses_id].info = info;
	ses_map[ses_id].cid_idx = query_end_id_idx(info.cid) + module->hw_info.chipid * MULTI_DIE_ENDMAP_OFFSET;

#ifndef REMOVE_STS_MGT
	if (info.role == MSGBX_SES_ROLE_CLIENT) {
		if (info.cid == MULTI_DST_CLIENT)
			ses_map[ses_id].status = SES_STATE_DST_CHANGED;
		else
			ses_map[ses_id].status = ipc_end_is_ready(info.cid, module->hw_info.chipid, addr) < 0 ? SES_STATE_NOT_AVAIL : SES_STATE_AVAIL;
		ses_map[ses_id].update_flag |= (1 << SES_UPDATE_STS_BIT);
	} else {
		ses_map[ses_id].status = SES_STATE_AVAIL;
	}
#else
	ses_map[ses_id].status = SES_STATE_AVAIL;
#endif

#ifdef DEBUG_MODE_ENABLE
	ipc_memset(&ses_map[ses_id].debug_info, 0, sizeof(debug_info_t));
	ses_map[ses_id].debug_info.role = info.role;
#if defined(MSGBX_HW_TYPE_A2000)
	ses_map[ses_id].debug_info.ccid = info.ccid;
#endif
	ses_map[ses_id].debug_info.cid = info.cid;
#endif

	*session_id = ses_id;
	return 0;
}

int8_t session_destroy(ipc_ses_t *ses)
{
	if (!ses)
		return -1;

	ses->status = SES_STATE_DESTROY;
#if !defined(USE_EXTERNAL_MSG_BUFFER)
	msg_queue_init(&ses->msg_queue);
#endif
	ipc_memset(&ses->info, 0, sizeof(ses_base_t));
	ses->cid_idx = 0;
	ses->update_flag = 0;
	ses->status = SES_STATE_INVALID;

	return 0;
}

int8_t session_msg_in(const uint8_t session_id, const rw_msg_t *msg, void *addr)
{
	int32_t ret = -1;
	uint64_t timestamp = 0;
	msgbx_end_device_t *module = (msgbx_end_device_t *)addr;
	ipc_ses_t *ses = module->ses_map + session_id;
	uint8_t cpuid = module->g_ipc_cpuid;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *debug = NULL;
#endif

	if (session_id >= CHANNEL_COUNT * SESSION_COUNT)
		return -3;

	if (ses->status == SES_STATE_DESTROY)
		return -4;

#ifdef DEBUG_MODE_ENABLE
	debug = &ses->debug_info;
	++(debug->recv_rw_msg_cnt);
#endif

	ipc_hw_layer_get_time(cpuid, &timestamp);
#if defined(USE_EXTERNAL_MSG_BUFFER)
	ret = msg_queue_in(ses->msg_queue, (rw_msg_t *)msg, timestamp);
#else
	ret = msg_queue_in(&(ses->msg_queue), (rw_msg_t *)msg, timestamp);
#endif

#ifdef DEBUG_MODE_ENABLE
	if (ret == 0)
		++(debug->in_rw_msg_cnt);
#endif
	return ret;
}

int8_t session_msg_out(ipc_ses_t *ses, serdes_t *msg)
{
	if (!ses)
		return -1;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	return msg_queue_out(ses->msg_queue, msg);
#else
	return msg_queue_out(&(ses->msg_queue), msg);
#endif
}

int32_t ipc_trans_get_avail_info(const uint8_t session_id, void *dev_info)
{
	int8_t ret = -1;
	ipc_ses_t *ses = NULL;

	if (!dev_info)
		return ret;

	ses = get_session(session_id, dev_info);
	if (!ses) {
		return -ERR_SES_IS_INVALID;
	}

	if (ses->status == SES_STATE_DESTROY)
		return -4;

	ret = msg_queue_recycle(ses);
	if (ret < 0)
		return ret;

#ifndef REMOVE_STS_MGT
	if (ses->update_flag & (1 << SES_UPDATE_STS_BIT)) {
		if (ses->status == SES_STATE_DST_CHANGED)
			ret = QUERY_INFO_DST_STS_CHANGED;
		else
			ret = ses->status == SES_STATE_AVAIL ?
			      QUERY_INFO_DST_STS_ONLINE :
			      QUERY_INFO_DST_STS_OFFLINE;
		ses->update_flag &= ~(1 << SES_UPDATE_STS_BIT);
		return ret;
	}
#endif
#if defined(USE_EXTERNAL_MSG_BUFFER)
	ret = msg_queue_collate(ses->msg_queue, 0);
#else
	ret = msg_queue_collate(&(ses->msg_queue), 0);
#endif
	return ret;
}

int8_t session_rwmsg_out(ipc_ses_t *ses, rw_msg_t *msg)
{
	int32_t ret = -1;

	if (!ses)
		return -2;

	if (ses->status == SES_STATE_DESTROY || ses->status == SES_STATE_INVALID)
		return -3;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	ret = rwmsg_queue_out(ses->msg_queue, msg);
#else
	ret = rwmsg_queue_out(&(ses->msg_queue), msg);	
#endif
	return ((ret != 0) ? -4 : 0);
}
