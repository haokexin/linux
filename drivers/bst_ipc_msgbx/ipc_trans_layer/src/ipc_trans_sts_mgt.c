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
 * @file  ipc_trans_sts_mgt.c
 * @brief this file is used as ipc state management, and it is configurable.
 * @note you can use IPC_STATE_MGT_ENABLE flag in bstipc_cfg.h to enable or disable these functions.
 * @details feature list
 * 1. receive error message from ipc_trans_routing
 * 2. provide notification to ipc_trans_impl
 * 3. support default or user-defined process mechanism
 * 4. provide inquire api to other modules in ipc_trans_layer
 */
#include <bst/ipc_hw_layer.h>
#include <bst/ipc_trans_common.h>
#include "ipc_trans_sts_mgt.h"
#include "../include/config.h"
#include "ipc_trans_runtime.h"
#include "ipc_trans_utl.h"
#include "ipc_trans_lflist.h"

#define MSGBX_END_MONITOR_CNT (MSGBX_END_COUNT_MAX - 1)

int32_t ipc_end_register(const uint8_t pid, void *addr)
{
	int32_t ret = 0;
	msgbx_end_device_t *module = NULL;
	rw_msg_t notify_msg = { 0 };

	if (!addr)
		return -1;

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
	notify_msg.header.ver = TRANS_LAYER_VERSION;

	ret = ipc_hw_layer_send_msg(module->ops.cpuid, &notify_msg);
	return ret;
}

int32_t ipc_end_unregister(const uint8_t pid, void *addr)
{
	int32_t ret = 0;
	msgbx_end_device_t *module = NULL;
	rw_msg_t notify_msg = { 0 };

	if (!addr)
		return -1;

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
	notify_msg.header.ver = TRANS_LAYER_VERSION;

	ret = ipc_hw_layer_send_msg(module->ops.cpuid, &notify_msg);
	return ret;
}

int32_t ipc_end_sts_update(const uint8_t end_id, const uint8_t status,
			   const uint64_t map, void *addr)
{
#ifndef REMOVE_STS_MGT
	msgbx_end_device_t *module = NULL;
	int32_t idx = 0;
	uint8_t fid_cnt = 0;
	uint8_t sid_cnt = 0;
	uint8_t cnt = 0;
	ses_status_t update_result = SES_STATE_INVALID;
	uint64_t updated_map = 0;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	if (module->g_ipc_pid != CENTRAL_MONITOR_END_ID) {
#if defined IPC_RTE_BAREMETAL
		module->g_end_sts_map = map;
#else
		ATOMIC_STORE(&module->g_end_sts_map, map, __ATOMIC_RELEASE);
#endif
	} else {
		idx = query_end_id_idx(end_id);
		if (idx < 0)
			return -1;

		// status: 1 online / 0 offline
		if (status)
#if defined IPC_RTE_BAREMETAL
			module->g_end_sts_map |= 1ULL << idx;
#else
			ATOMIC_FETCH_OR(&module->g_end_sts_map, 1ULL << idx,
					__ATOMIC_RELEASE);
#endif
		else
#if defined IPC_RTE_BAREMETAL
			module->g_end_sts_map &= ~(1ULL << idx);
#else
			ATOMIC_FETCH_XOR(&module->g_end_sts_map, 1ULL << idx,
					 __ATOMIC_RELEASE);
#endif
	}

	// update all related sessions status
	ipc_memset(&module->g_update_ses_list, -1,
		   sizeof(module->g_update_ses_list));
#if defined IPC_RTE_BAREMETAL
	updated_map = module->g_end_sts_map;
#else
	updated_map = ATOMIC_LOAD(&module->g_end_sts_map, __ATOMIC_ACQUIRE);
#endif

	for (fid_cnt = 0; fid_cnt < CHANNEL_COUNT; ++fid_cnt) {
		for (sid_cnt = 0; sid_cnt < SESSION_COUNT; ++sid_cnt) {
			if (module->ses_map[fid_cnt][sid_cnt].ses_type ==
				    MSGBX_SES_ROLE_CLIENT &&
			    module->ses_map[fid_cnt][sid_cnt].info.cid != 0) {
				update_result =
					(updated_map &
					 (1ULL
					  << module->ses_map[fid_cnt][sid_cnt]
						     .cid_idx)) != 0 ?
						SES_STATE_AVAIL :
						SES_STATE_NOT_AVAIL;
				if (module->ses_map[fid_cnt][sid_cnt].status !=
				    update_result) {
					module->ses_map[fid_cnt][sid_cnt]
						.status = update_result;
					module->ses_map[fid_cnt][sid_cnt]
						.update_flag |=
						(1ULL << SES_UPDATE_STS_BIT);
					module->g_update_ses_list[cnt++] =
						((sid_cnt << 4) | fid_cnt);
				}
			}
		}
	}
	return 1;
#endif
	return 0;
}

int32_t ipc_end_is_ready(const uint8_t end_id, void *addr)
{
#ifndef REMOVE_STS_MGT
	msgbx_end_device_t *module = NULL;
	int32_t idx = query_end_id_idx(end_id);
	uint64_t status = 0;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	status = ATOMIC_LOAD(&module->g_end_sts_map, __ATOMIC_ACQUIRE);

	if (idx < 0)
		return -2;
	return ((status & (1ULL << idx)) != 0 ? 0 : -1);
#endif
	return 0;
}

#if !defined(BAREMETAL_VERSION_TRUNCATE)
static uint8_t get_all_1bit(uint8_t id[], void *addr)
{
	uint8_t cnt = 0;
	uint64_t map = 0;
	uint8_t next_psn = 0;
	msgbx_end_device_t *module = NULL;

	if (!addr)
		return -1;

	module = (msgbx_end_device_t *)addr;
	map = ATOMIC_LOAD(&module->g_end_sts_map, __ATOMIC_ACQUIRE);

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
#if !defined(BAREMETAL_VERSION_TRUNCATE)
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

	map = ATOMIC_LOAD(&module->g_end_sts_map, __ATOMIC_ACQUIRE);
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
		ret = ipc_hw_layer_send_msg(module->ops.cpuid, &notify_msg);
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

	ret = ipc_hw_layer_send_msg(module->ops.cpuid, &notify_msg);
#endif
	return ret;
}
