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
 * @file ipc_app_svr_utils.h
 * @brief Utilities for IPC application layer server.
 *
 * This file contains utility functions for the IPC server.
 * It provides functions for adding and removing registry entries.
 */

#ifndef IPC_APP_LAYER_SERVER_UTILITIES_H
#define IPC_APP_LAYER_SERVER_UTILITIES_H

#include "ipc_app_common.h"
#include "ipc_trans_layer.h"

#ifndef IPC_RTE_BAREMETAL
#if defined IPC_RTE_POSIX
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#elif defined IPC_RTE_KERNEL
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Broadcast sub/unsub callback function.
 * 
 * @param pid the subscriber's messagebox end id
 * @param fid the subscriber's filter channel id
 * @param sid the subscriber's session id
 * @return 0 accept, -1 reject
 */
typedef int32_t (*broadcast_sub_t)(uint8_t pid, uint8_t fid, uint8_t sid);

#define IPC_REG_MAP_BLOCK_SIZE 4U

struct __attribute__ ((aligned(4))) _broadcast_reg_entry_t {
	uint8_t pid;
	uint8_t fid;
	uint8_t sid;
	uint8_t res;
};
#define broadcast_reg_entry_t struct _broadcast_reg_entry_t

struct _broadcast_registry_t {
	broadcast_reg_entry_t entries[IPC_MAX_SUBSCRIPTION];
};
#define broadcast_registry_t struct _broadcast_registry_t

struct _com_server_data_t {
	COM_DATA_ELE
};
#define com_server_data_t struct _com_server_data_t

/**
 * @brief Adds a registration entry to the broadcast registry.
 *
 * @param regs The broadcast registry.
 * @param pid The process ID.
 * @param fid The function ID.
 * @param sid The service ID.
 * @return 0 if successful, -1 otherwise.
 */
static inline int32_t add_registration(broadcast_registry_t *regs, uint8_t pid,
				       uint8_t fid, uint8_t sid)
{
	int32_t index = 0;
	int32_t insert_index = -1;
	broadcast_reg_entry_t *entry = NULL;
	broadcast_reg_entry_t src = {0};
	broadcast_reg_entry_t dest = { pid, fid, sid };

	if (!regs || pid == 0)
		return -1;
	// find if already exists.
	entry = regs->entries;
	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++entry) {
		__atomic_load(entry, &src, __ATOMIC_ACQUIRE);
		if (src.pid == pid && src.fid == fid && src.sid == sid)
			return 0;
		if (insert_index < 0 && src.pid == 0)
			insert_index = index;
	}
	if (insert_index < 0)
		return -1;
	// insert the entry.
	entry = &regs->entries[insert_index];
	__atomic_store(entry, &dest, __ATOMIC_RELEASE);

	return 0;
}

/**
 * @brief Removes a registration entry from the broadcast registry.
 *
 * @param regs The broadcast registry.
 * @param pid The process ID.
 * @param fid The function ID.
 * @param sid The service ID.
 * @return 0 if successful, -1 otherwise.
 */
static inline int32_t remove_registration(broadcast_registry_t *regs,
					  uint8_t pid, uint8_t fid, uint8_t sid)
{
	int32_t index = 0;
	broadcast_reg_entry_t *entry = NULL;
	broadcast_reg_entry_t src = {0};
	broadcast_reg_entry_t dest = {0};

	if (!regs || pid == 0)
		return -1;
	entry = regs->entries;
	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++entry) {
		__atomic_load(entry, &src, __ATOMIC_ACQUIRE);
		if (src.pid == pid && src.fid == fid && src.sid == sid) {
			//clear entry
			__atomic_store(entry, &dest, __ATOMIC_RELEASE);

			return 0;
		}
	}
	return 0;
}

static inline void increase_token(com_server_data_t *data)
{
	if (!data)
		return;

	data->token = (data->token + 1) % IPC_TOKEN_NUM;
}

static inline int32_t send_broadcast(com_server_data_t *data, serdes_t *ser, broadcast_reg_entry_t *reg, uint8_t cmd, uint32_t end)
{
	int32_t ret = 0;
	int32_t send_ret = 0;
	int32_t index = 0;
	rw_msg_header_t header = { 0 };
	broadcast_reg_entry_t entry = {0};
	
	if (!data || !ser || !reg)
		return -ERR_APP_PARAM;

	header.pid = data->pid;
	header.cmd = cmd;
	header.typ = MSGBX_MSG_TYPE_BROADCAST;

	for (index = 0; index < end; ++index, ++reg) {
		__atomic_load(reg, &entry, __ATOMIC_ACQUIRE);
		if (entry.pid != 0) {
			header.cid = entry.pid;
			header.fid = entry.fid;
			header.sid = entry.sid;
			header.tok = data->token;
			ipc_ser_set_header(ser, header);
			ret = ipc_ser_finish(ser);
			if (ret < 0)
				continue;
			IPC_MUTEX_LOCK(&data->send_mtx);
			send_ret = ipc_trans_layer_stub_send_broadcast(data->pid, data->handle, ser);
			IPC_MUTEX_UNLOCK(&data->send_mtx);
			if (send_ret < 0)
				IPC_LOG_WARNING("send broadcast fail %d.\n", send_ret);
			else
				++ret;
		}
	}
	++data->token;

	return ret;
}

static inline int32_t send_reply(com_server_data_t *data, serdes_t *ser)
{
	int32_t ret = 0;

	if (!data || !ser)
		return -ERR_APP_PARAM;

	IPC_MUTEX_LOCK(&data->send_mtx);
	ret = ipc_trans_layer_stub_send_reply_msg(data->pid, data->handle, ser);
	IPC_MUTEX_UNLOCK(&data->send_mtx);

	return ret;
}

static inline int32_t get_subscribed_size(broadcast_reg_entry_t *reg)
{
	int32_t ret = 0;
	int32_t index = 0;

	if (!reg)
		return -ERR_APP_PARAM;

	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++reg) {
		if (reg->pid != 0) {
			++ret;
		}
	}
	return ret;
}

static inline int32_t export_registry(broadcast_reg_entry_t *reg, uint8_t *reg_map, uint8_t cmd)
{
	int32_t ret = 0;
	int32_t index = 0;
	broadcast_reg_entry_t src = {0};

	if (!reg || !reg_map)
		return -ERR_APP_PARAM;

	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++reg) {
		__atomic_load(reg, &src, __ATOMIC_ACQUIRE);
		if (src.pid != 0) {
			*(reg_map++) = cmd;
			*(reg_map++) = src.pid;
			*(reg_map++) = src.fid;
			*(reg_map++) = src.sid;
			++ret;
		}
	}
	return ret;
}

#ifdef __cplusplus
}
#endif

static inline int32_t des_registry(uint8_t *reg_map, uint32_t *pos, uint8_t *pid, uint8_t *fid, uint8_t *sid, uint8_t *cmd)
{
	uint8_t *ptr;

	if (!reg_map || !pos || !pid || !fid || !sid || !cmd)
		return -ERR_APP_PARAM;

	ptr = (uint8_t *)reg_map + *pos;
	*cmd = *(ptr++);
	*pid = *(ptr++);
	*fid = *(ptr++);
	*sid = *(ptr++);
	*pos += 4;

	return 0;
}
#endif
//\ No newline at end of file
