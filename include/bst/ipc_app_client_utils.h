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
 * @file ipc_app_client_utils.h
 * @brief Utilities for IPC application layer client.
 *
 * This file contains utility functions for the IPC application layer client.
 * It provides functions for initializing, adding, clearing, and destroying registry entries.
 * It also provides functions for waiting, notifying, and printing registry entries.
 */

#ifndef IPC_APP_LAYER_CLIENT_UTILITIES_H
#define IPC_APP_LAYER_CLIENT_UTILITIES_H

#include "ipc_app_common.h"
#include "ipc_trans_layer.h"

#ifndef IPC_RTE_BAREMETAL
#if defined IPC_RTE_POSIX

#include <time.h>
#include <semaphore.h>
#include <errno.h>
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
 * @brief Structure representing a callback registration entry.
 */
struct _callback_registration_t {
	bool busy;
	uint8_t res[7];
	void *cb;
	void *ext;
	des_buf_t *ext_buf;
};
#define callback_registration_t struct _callback_registration_t

struct _com_client_data_t {
COM_DATA_ELE
	callback_registration_t common_registry[IPC_TOKEN_NUM];
};
#define com_client_data_t struct _com_client_data_t

/**
 * @brief Initializes a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @return 0 if successful, -1 if reg is NULL, -2 if initialization fails.
 */
static inline int32_t init_registry(callback_registration_t *reg)
{
	if (!reg)
		return -1;
	reg->busy = false;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
	return 0;
}

/**
 * @brief Initializes a list of callback registry entries.
 *
 * @param list Pointer to the list of callback registry entries.
 * @param size Size of the list.
 * @return Total number of initialization failures.
 */
static inline int32_t init_registry_list(callback_registration_t *list,
					 uint32_t size)
{
	int32_t ret = 0;

	if (!list)
		return -1;

	for (int32_t i = 0; i < size; ++i)
		ret += init_registry(&list[i]);

	return ret;
}

/**
 * @brief Clears a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 */
static inline void clear_registry(callback_registration_t *reg)
{
	if (!reg)
		return;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
	reg->busy = false; //busy must be last to avoid race condition.
}
/**
 * @brief Sets information to a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @param cb Pointer to the callback function.
 * @param ext Pointer to the external data.
 * @param ext_buf Pointer to the external buffer.
 * @return 0 if successful, -1 if reg is NULL.
 */
static inline int32_t set_registry(callback_registration_t *reg, void *cb,
	void *ext, des_buf_t *ext_buf)
{
	if (!reg)
		return -1;

	reg->cb = cb;
	reg->ext = ext;
	reg->ext_buf = ext_buf;
	return 0;
}

/**
 * @brief Adds information to a callback registry entry.
 *
 * @param list Pointer to the callback registry list.
 * @param tok Returns the token used.
 * @param cb Pointer to the callback function.
 * @param ext Pointer to the external data.
 * @param ext_buf Pointer to the external buffer.
 * @return 0 if successful, -1 if fail.
 */
static inline int32_t add_registry(callback_registration_t *list,
	uint8_t *tok, void *cb, void *ext, des_buf_t *ext_buf)
{
	bool expected = false;
	if (!list)
		return -1;

	for (int i = 0; i < IPC_TOKEN_NUM; ++i) {
		// if same, reuse this registration.
		if (list[i].busy && list[i].cb == cb && list[i].ext == ext && list[i].ext_buf == ext_buf) {
			*tok = i;
			return 0;
		}
		// find empty slot.
		expected = false;
		if (__atomic_compare_exchange_n(&list[i].busy, &expected, true, true, __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
			*tok = i;
			list[i].cb = cb;
			list[i].ext = ext;
			list[i].ext_buf = ext_buf;
			return 0;
		}
	}

	return -1;
}

/**
 * @brief Destroys a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 */
static inline void destroy_registry(callback_registration_t *reg)
{
	if (!reg)
		return;
	reg->busy = false;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
}

/**
 * @brief Destroys a list of callback registry entries.
 *
 * @param list Pointer to the list of callback registry entries.
 * @param size Size of the list.
 */
static inline void destroy_registry_list(callback_registration_t *list,
					 uint32_t size)
{
	if (!list)
		return;
	for (int32_t i = 0; i < size; ++i)
		destroy_registry(&list[i]);
}

#ifndef IPC_NO_DEBUG
/**
 * @brief Prints the information of a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 */
static inline void print_registry(callback_registration_t *reg)
{
	if (!reg)
		return;
	IPC_LOG_INFO("busy: %d, cb: %p, ext: %p, ext_buf: %p", reg->busy,
		     reg->cb, reg->ext, reg->ext_buf);
}

/**
 * @brief Prints the information of a list of callback registry entries.
 *
 * @param list Pointer to the list of callback registry entries.
 * @param size Size of the list.
 */
static inline void print_registry_list(callback_registration_t *list,
				       uint32_t size)
{
	if (!list)
		return;
	for (int32_t i = 0; i < size; ++i)
		print_registry(&list[i]);
}
#endif

/**
 * @brief Sends a request to the server.
 *
 * @param data Pointer to the client data.
 * @param registry Pointer to the callback registry entry.
 * @param ser Pointer to the serialization buffer.
 * @param cid Command ID.
 * @param cmd Command type.
 * @param cb Callback function pointer.
 * @param ext External data pointer.
 * @param ext_buf External buffer for external data.
 * @return Token if successful, -1 if fail.
 */
static inline int32_t send_request(com_client_data_t *data,
		callback_registration_t *registry, serdes_t *ser, int8_t cid,
		int8_t cmd, void *cb, void *ext, des_buf_t *ext_buf)
{
	int32_t ret = 0;
	uint8_t tok = 0;

	if (!ser || !data)
		return -ERR_APP_PARAM;

	// prepare message
	ser->header.pid = data->pid;
	ser->header.fid = data->fid;
	ser->header.sid = data->sid;
	ser->header.cid = cid;
	ser->header.cmd = cmd;
	ser->header.typ = MSGBX_MSG_TYPE_METHOD;

	// take and set registry
	ret = add_registry(registry, &tok, cb, ext, ext_buf);
	if (ret < 0)
		return -ERR_APP_TOK;
	ser->header.tok = tok;
	ret = ipc_ser_finish(ser);
	if (ret < 0)
		return -ERR_APP_SERDES;

	// send message
	IPC_MUTEX_LOCK(&data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(data->pid, data->handle, ser);
	IPC_MUTEX_UNLOCK(&data->send_mtx);
	if (ret < 0) {
		clear_registry(&registry[tok]);
		return ret;
	}
	return tok;
}

#ifndef IPC_NO_FIRE_AND_FORGET
/**
 * @brief Sends a fire-and-forget request to the server.
 * @param data Pointer to the client data.
 * @param ser Pointer to the serialization buffer.
 * @param cid Command ID.
 * @param cmd Command type.
 * @return 0 if successful, -1 if fail.
 */
static inline int32_t send_fire_and_forget_request(com_client_data_t *data,
			serdes_t *ser, int8_t cid, int8_t cmd)
{
	int32_t ret = 0;

	if (!ser || !data)
		return -ERR_APP_PARAM;

	// prepare message
	ser->header.pid = data->pid;
	ser->header.fid = data->fid;
	ser->header.sid = data->sid;
	ser->header.cid = cid;
	ser->header.cmd = cmd;
	ser->header.typ = MSGBX_MSG_TYPE_METHOD;
	ser->header.tok = 0;
	ret = ipc_ser_finish(ser);
	if (ret < 0)
		return -ERR_APP_SERDES;

	// send message
	IPC_MUTEX_LOCK(&data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(data->pid, data->handle, ser);
	IPC_MUTEX_UNLOCK(&data->send_mtx);

	return ret;
}
#endif

/**
 * @brief Calls the broadcast subscription/unsubscription callback.
 * @param data Pointer to the client data.
 * @param des Pointer to the deserialization buffer.
 * @return 0 if successful, -1 if fail.
 */
static inline int32_t call_broadcast_sub_unsub_callback(com_client_data_t *data,
			des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	int32_t err = 0;

	if (!data || !des)
		return -ERR_APP_PARAM;

	reg = &data->common_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	if (des->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;
	ret = deserialize_32(des, (uint32_t *)&err);
	if (ret < 0)
		return -ERR_APP_SERDES;

	if (reg->busy) {
		broadcast_sub_unsub_callback_t cb = (broadcast_sub_unsub_callback_t)(reg->cb);

		if (cb)
			cb(err, reg->ext, &data->info);
		clear_registry(reg);
	}

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif
