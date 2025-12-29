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
 * @file  ipc_trans_sts_mgt.c
 * @brief this file is used as ipc state management, and it is configurable.
 * @note you can use IPC_STATE_MGT_ENABLE flag in bstipc_cfg.h to enable or disable these functions.
 * @details feature list
 * 1. receive error message from ipc_trans_routing
 * 2. provide notification to ipc_trans_impl
 * 3. support default or user-defined process mechanism
 * 4. provide inquire api to other modules in ipc_trans_layer
 */

#include "ipc_trans_sts_mgt.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_utl.h"

#define MSGBX_END_MONITOR_CNT (MSGBX_END_COUNT_MAX - 1)

int32_t ipc_end_register(const uint8_t pid, void *addr)
{
	int32_t ret = 0;
	msgbx_end_device_t *module = NULL;
#if defined(MSGBX_HW_TYPE_C1200)
	rw_msg_t notify_msg = { 0 };
#elif defined(MSGBX_HW_TYPE_A2000)
	int32_t idx = 0;
#endif

	if (!addr)
		return -1;
	module = (msgbx_end_device_t *)addr;

#if defined(MSGBX_HW_TYPE_C1200)
	if (pid == CENTRAL_MONITOR_END_ID)
		return 0;
	module = (msgbx_end_device_t *)addr;
	notify_msg.header.cid = CENTRAL_MONITOR_END_ID;
	notify_msg.header.pid = pid;
	notify_msg.header.len = 0;
	notify_msg.header.is_eof = 1;
	notify_msg.header.fid = 0;
	notify_msg.header.sid = 0;
	notify_msg.header.typ = MSGBX_MSG_TYPE_PROTOCOL;
	notify_msg.header.cmd = TRANS_PROTO_CMD_ENDMAP_ONLINE;
	notify_msg.header.tok = 0;
	notify_msg.header.idx = 0;
	notify_msg.header.res = 0;
	ret = ipc_hw_layer_send_msg(module->g_ipc_cpuid, &notify_msg);
#elif defined(MSGBX_HW_TYPE_A2000)
	idx = query_end_id_idx(pid);
	if (idx < 0 || idx > MSGBX_END_COUNT_MAX)
		return -2;
	if (module->hw_info.chipid == MULTI_DIE_CHIP_1)
		idx += MULTI_DIE_ENDMAP_OFFSET;
	ret = ipc_hw_layer_set_endmap(module->g_ipc_cpuid, idx, TRANS_PROTO_CMD_ENDMAP_ONLINE);
#endif
	return ret;
}

int32_t ipc_end_unregister(const uint8_t pid, void *addr)
{
	int32_t ret = 0;
	msgbx_end_device_t *module = NULL;
#if defined(MSGBX_HW_TYPE_C1200)
	rw_msg_t notify_msg = { 0 };
#elif defined(MSGBX_HW_TYPE_A2000)
	int32_t idx = 0;
#endif

	if (!addr)
		return -1;
	module = (msgbx_end_device_t *)addr;
#if defined(MSGBX_HW_TYPE_C1200)
	if (pid == CENTRAL_MONITOR_END_ID)
		return 0;
	module = (msgbx_end_device_t *)addr;
	notify_msg.header.cid = CENTRAL_MONITOR_END_ID;
	notify_msg.header.pid = pid;
	notify_msg.header.len = 0;
	notify_msg.header.is_eof = 1;
	notify_msg.header.fid = 0;
	notify_msg.header.sid = 0;
	notify_msg.header.typ = MSGBX_MSG_TYPE_PROTOCOL;
	notify_msg.header.cmd = TRANS_PROTO_CMD_ENDMAP_OFFLINE;
	notify_msg.header.tok = 0;
	notify_msg.header.idx = 0;
	notify_msg.header.res = 0;
	ret = ipc_hw_layer_send_msg(module->g_ipc_cpuid, &notify_msg);
#elif defined(MSGBX_HW_TYPE_A2000)
	idx = query_end_id_idx(pid);
	if (idx < 0 || idx > MSGBX_END_COUNT_MAX)
		return -2;
	if (module->hw_info.chipid == MULTI_DIE_CHIP_1)
		idx += MULTI_DIE_ENDMAP_OFFSET;
	ret = ipc_hw_layer_set_endmap(module->g_ipc_cpuid, idx, TRANS_PROTO_CMD_ENDMAP_OFFLINE);
#endif
	return ret;
}


int32_t ipc_trans_end_sts_update(const uint8_t end_id, const uint8_t status,
			   const sts_endmap_t *map, void *addr)
{
#ifndef REMOVE_STS_MGT
#if defined(MSGBX_HW_TYPE_A2000)
	msgbx_end_device_t *module = NULL;
	uint8_t i = 0;
	uint8_t cnt = 0;
	uint8_t update_result = SES_STATE_INVALID;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
#if defined (IPC_NO_ATOMIC)
	module->g_end_sts_map = *map;
#else
	ATOMIC_STORE(&(module->g_end_sts_map.lo_map), map->lo_map, __ATOMIC_RELEASE);
	ATOMIC_STORE(&(module->g_end_sts_map.hi_map), map->hi_map, __ATOMIC_RELEASE);
#endif

	ipc_memset(&module->g_update_ses_list, -1, sizeof(module->g_update_ses_list));

	for (i = 0; i < CHANNEL_COUNT * SESSION_COUNT; ++i) {
		if (module->ses_map[i].info.role == MSGBX_SES_ROLE_CLIENT) {
			if (module->ses_map[i].info.cid == MULTI_DST_CLIENT) {
				module->ses_map[i].status = SES_STATE_DST_CHANGED;
				module->ses_map[i].update_flag |= (1ULL << SES_UPDATE_STS_BIT);
				module->g_update_ses_list[cnt++] = i;
			} else {
				if (module->ses_map[i].cid_idx < MULTI_DIE_ENDMAP_OFFSET)
					update_result = (module->g_end_sts_map.lo_map &
						(1ULL << module->ses_map[i] .cid_idx)) != 0 ?
						SES_STATE_AVAIL : SES_STATE_NOT_AVAIL;
				else
					update_result = (module->g_end_sts_map.hi_map &
						(1ULL << (module->ses_map[i].cid_idx -
						MULTI_DIE_ENDMAP_OFFSET))) != 0 ?
						SES_STATE_AVAIL : SES_STATE_NOT_AVAIL;
				if (module->ses_map[i].status != update_result) {
					module->ses_map[i].status = update_result;
					module->ses_map[i].update_flag |= (1ULL << SES_UPDATE_STS_BIT);
					module->g_update_ses_list[cnt++] = i;
				}
			}
		}
	}
	return 1;
#elif defined(MSGBX_HW_TYPE_C1200)
	msgbx_end_device_t *module = NULL;
	uint8_t i = 0;
	uint8_t cnt = 0;
	uint8_t update_result = SES_STATE_INVALID;
	int32_t idx = 0;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	if (module->g_ipc_pid != CENTRAL_MONITOR_END_ID) {
#if defined (IPC_NO_ATOMIC)
		module->g_end_sts_map.lo_map = map->lo_map;
#else
		ATOMIC_STORE(&module->g_end_sts_map.lo_map, map->lo_map, __ATOMIC_RELEASE);
#endif
	} else {
		idx = query_end_id_idx(end_id);
		if (idx < 0)
			return -1;

		if (status)
#if defined (IPC_NO_ATOMIC)
			module->g_end_sts_map.lo_map |= 1ULL << idx;
#else
			ATOMIC_FETCH_OR(&module->g_end_sts_map.lo_map, 1ULL << idx,
					__ATOMIC_RELEASE);
#endif
		else
#if defined IPC_RTE_BAREMETAL
			module->g_end_sts_map.lo_map &= ~(1ULL << idx);
#else
			ATOMIC_FETCH_XOR(&module->g_end_sts_map.lo_map, 1ULL << idx,
					 __ATOMIC_RELEASE);
#endif
	}

	ipc_memset(&module->g_update_ses_list, -1, sizeof(module->g_update_ses_list));
	for (i = 0; i < CHANNEL_COUNT * SESSION_COUNT; ++i) {
		if (module->ses_map[i].info.role == MSGBX_SES_ROLE_CLIENT) {
			if (module->ses_map[i].info.cid == MULTI_DST_CLIENT) {
				module->ses_map[i].status = SES_STATE_DST_CHANGED;
				module->ses_map[i].update_flag |= (1ULL << SES_UPDATE_STS_BIT);
				module->g_update_ses_list[cnt++] = i;
			} else {
				update_result = (module->g_end_sts_map.lo_map &
					(1ULL << module->ses_map[i].cid_idx)) != 0 ?
					SES_STATE_AVAIL :
					SES_STATE_NOT_AVAIL;
				if (module->ses_map[i].status != update_result) {
					module->ses_map[i].status = update_result;
					module->ses_map[i].update_flag |= (1ULL << SES_UPDATE_STS_BIT);
					module->g_update_ses_list[cnt++] = i;
				}
			}
		}
	}
	return 1;
#endif
#endif
	return 0;
}

int32_t ipc_end_is_ready(const uint8_t end_id, const uint8_t chipid, void *addr)
{
#ifndef REMOVE_STS_MGT
	msgbx_end_device_t *module = NULL;
	int32_t idx = query_end_id_idx(end_id);
	uint64_t endmap = 0;

	if (!addr)
		return -1;

	if (idx < 0)
		return -2;

	module = (msgbx_end_device_t *)addr;
#if defined(MSGBX_HW_TYPE_A2000)
	if (chipid == MULTI_DIE_CHIP_0)
		endmap = ATOMIC_LOAD(&module->g_end_sts_map.lo_map, __ATOMIC_ACQUIRE);
	else
		endmap = ATOMIC_LOAD(&module->g_end_sts_map.hi_map, __ATOMIC_ACQUIRE);
#elif defined(MSGBX_HW_TYPE_C1200)
	endmap = ATOMIC_LOAD(&module->g_end_sts_map.lo_map, __ATOMIC_ACQUIRE);
#endif
	return ((endmap & (1ULL << idx)) != 0 ? 0 : -1);
#endif
	return 0;
}

#if defined(MSGBX_HW_TYPE_C1200)
#ifndef REMOVE_STS_MGT
static uint8_t get_all_1bit(uint8_t id[], void *addr)
{
	uint8_t cnt = 0;
	uint64_t map = 0;
	uint8_t next_psn = 0;
	msgbx_end_device_t *module = NULL;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	map = ATOMIC_LOAD(&module->g_end_sts_map.lo_map, __ATOMIC_ACQUIRE);

	while (map > 0) {
		next_psn = __builtin_ffsll(map) - 1;
		map &= (map - 1);
		id[cnt++] = next_psn;
	}
	return cnt;
}
#endif

int32_t ipc_trans_end_sts_broadcast(void *addr)
{
#ifndef REMOVE_STS_MGT
	msgbx_end_device_t *module = NULL;
	uint8_t ids[MSGBX_END_MONITOR_CNT] = { 0 };
	int32_t ret = -1;
	uint8_t ids_cnt = 0;
	uint64_t map = 0;
	uint8_t monitor_idx = 0;
	rw_msg_t notify_msg = { 0 };
	uint8_t cnt = 0;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	if (module->g_ipc_pid != CENTRAL_MONITOR_END_ID)
		return -2;

	map = ATOMIC_LOAD(&module->g_end_sts_map.lo_map, __ATOMIC_ACQUIRE);
	if (module->g_req_endmap_cid != 0) {
		ids_cnt = 1;
		ids[0] = query_end_id_idx(module->g_req_endmap_cid);
		module->g_req_endmap_cid = 0;
	} else
		ids_cnt = get_all_1bit(ids, addr);

	ret = query_end_id_idx(CENTRAL_MONITOR_END_ID);
	if (ret < 0)
		return -1;

	monitor_idx = ret;
	notify_msg.header.pid = CENTRAL_MONITOR_END_ID;
	notify_msg.header.len = 1;
	notify_msg.header.fid = 0;
	notify_msg.header.sid = 0;
	notify_msg.header.is_eof = 1;
	notify_msg.header.typ = MSGBX_MSG_TYPE_PROTOCOL;
	notify_msg.header.cmd = TRANS_PROTO_CMD_ENDMAP_ONLINE;
	notify_msg.header.ver = TRANS_LAYER_VERSION;
	notify_msg.payload[0] = map | (1ULL << monitor_idx);

	for (cnt = 0; cnt < ids_cnt; ++cnt) {
		notify_msg.header.cid = end_idx_to_id(ids[cnt]);
		IPC_LOG_DEBUG("broadcast to endid %d new status map %llx",
			      notify_msg.header.cid, notify_msg.payload[0]);
		ret = ipc_hw_layer_send_msg(module->g_ipc_cpuid, &notify_msg);
		if (ret < 0)
			return -1;
	}
#endif
	return 0;
}

int32_t ipc_trans_query_remote_endmap(void *dev_info)
{
	int32_t ret = 0;
#ifndef REMOVE_STS_MGT
	msgbx_end_device_t *module = NULL;
	rw_msg_t notify_msg = { 0 };

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	if (module->g_ipc_pid == CENTRAL_MONITOR_END_ID)
		return 0;

	notify_msg.header.cid = CENTRAL_MONITOR_END_ID;
	notify_msg.header.pid = module->g_ipc_pid;
	notify_msg.header.len = 0;
	notify_msg.header.is_eof = 1;
	notify_msg.header.fid = 0;
	notify_msg.header.sid = 0;
	notify_msg.header.typ = MSGBX_MSG_TYPE_PROTOCOL;
	notify_msg.header.cmd = TRANS_PROTO_CMD_GET_ENDMAP;
	notify_msg.header.tok = 0;
	notify_msg.header.idx = 0;
	notify_msg.header.res = 0;
	notify_msg.header.ver = TRANS_LAYER_VERSION;

	ret = ipc_hw_layer_send_msg(module->g_ipc_cpuid, &notify_msg);
#endif
	return ret;
}
#elif defined(MSGBX_HW_TYPE_A2000)
int32_t ipc_trans_end_sts_broadcast(void *addr)
{
#ifndef REMOVE_STS_MGT
	int32_t ret = -1;
	uint8_t chipid = MULTI_DIE_CHIP_0;
	msgbx_end_device_t *module = NULL;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;

	if (module->g_ipc_pid != CENTRAL_MONITOR_END_ID)
		return -2;
	if (module->hw_info.chipid == MULTI_DIE_CHIP_0)
		chipid = MULTI_DIE_CHIP_1;
	ret = ipc_hw_layer_update_endmap(module->g_ipc_cpuid, chipid);
	return ret;
#endif
	return 0;
}

int32_t ipc_trans_query_remote_endmap(void *dev_info)
{
#ifndef REMOVE_STS_MGT
	int32_t ret = -1;
	msgbx_end_device_t *module = NULL;

	if (!dev_info)
		return -1;

	module = (msgbx_end_device_t *)dev_info;
	ret = ipc_hw_layer_get_endmap(module->g_ipc_cpuid, &module->g_end_sts_map);
	return ret;
#endif
	return 0;
}
#endif
