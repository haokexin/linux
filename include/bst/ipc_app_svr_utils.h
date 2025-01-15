/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
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
 * @file ipc_app_svr_utils.h
 * @brief Utilities for IPC application layer server.
 *
 * This file contains utility functions for the IPC server.
 * It provides functions for adding and removing registry entries.
 */

#ifndef IPC_APP_LAYER_SERVER_UTILITIES_H
#define IPC_APP_LAYER_SERVER_UTILITIES_H

#include <bst/ipc_app_common.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>

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

struct _broadcast_reg_entry_t {
	uint8_t pid;
	uint8_t fid;
	uint8_t sid;
	uint8_t res;
};
#define broadcast_reg_entry_t struct _broadcast_reg_entry_t

struct _broadcast_registry_t {
	broadcast_reg_entry_t entries[IPC_MAX_SUBSCRIPTION];
	int32_t start;
	int32_t end;
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

	if (!regs || pid == 0)
		return -1;
	// find if already exists.
	entry = regs->entries;
	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++entry) {
		if (entry->pid == pid && entry->fid == fid && entry->sid == sid)
			return 0;
		if (insert_index < 0 && entry->pid == 0)
			insert_index = index;
	}
	if (insert_index < 0)
		return -1;
	// insert the entry.
	entry = &regs->entries[insert_index];
	entry->pid = pid;
	entry->fid = fid;
	entry->sid = sid;
	if (index < regs->start)
		regs->start = index;
	if (index >= regs->end)
		regs->end = index + 1;
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

	if (!regs || pid == 0)
		return -1;
	entry = regs->entries;
	for (index = regs->start, entry += regs->start; index < regs->end;
	     ++index, ++entry) {
		if (entry->pid == pid && entry->fid == fid &&
		    entry->sid == sid) {
			entry->pid = 0;
			entry->fid = 0;
			entry->sid = 0;
			return 0;
		}
	}
	return -1;
}

static inline void increase_token(com_server_data_t *data)
{
	if (!data)
		return;

	//Since IPC_TOKEN_NUM is 256, no need to check for overflow.
	++data->token;
}

static inline int32_t send_broadcast(com_server_data_t *data, serdes_t *ser, broadcast_reg_entry_t *reg, uint8_t cmd, uint32_t end)
{
	int32_t ret = 0;
	int32_t send_ret = 0;
	int32_t index = 0;
	rw_msg_header_t header = { 0 };
	
	if (!data || !ser || !reg)
		return -ERR_APP_PARAM;

	header.pid = data->pid;
	header.cmd = cmd;
	header.typ = MSGBX_MSG_TYPE_BROADCAST;

	for (index = 0; index < end; ++index, ++reg) {
		if (reg->pid != 0) {
			header.cid = reg->pid;
			header.fid = reg->fid;
			header.sid = reg->sid;
			header.tok = data->token;
			ipc_ser_set_header(ser, header);
			ipc_ser_finish(ser);
			IPC_MUTEX_LOCK(&data->send_mtx);
			send_ret = ipc_trans_layer_stub_send_broadcast(data->pid, data->handle, ser);
			IPC_MUTEX_UNLOCK(&data->send_mtx);
			if (send_ret < 0)
				IPC_LOG_ERR("send broadcast fail %d.\n", send_ret);
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

	if (!reg || !reg_map)
		return -ERR_APP_PARAM;

	for (index = 0; index < IPC_MAX_SUBSCRIPTION; ++index, ++reg) {
		if (reg->pid != 0) {
			*(reg_map++) = cmd;
			*(reg_map++) = reg->pid;
			*(reg_map++) = reg->fid;
			*(reg_map++) = reg->sid;
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
