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
 * @file ipc_app_client_utils.h
 * @brief Utilities for IPC application layer client.
 *
 * This file contains utility functions for the IPC application layer client.
 * It provides functions for initializing, adding, clearing, and destroying registry entries.
 * It also provides functions for waiting, notifying, and printing registry entries.
 */

#ifndef IPC_APP_LAYER_CLIENT_UTILITIES_H
#define IPC_APP_LAYER_CLIENT_UTILITIES_H

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

#include <bst/ipc_app_common.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure representing a callback registration entry.
 */
struct _callback_registration_t {
	_Atomic bool busy;
	uint8_t cmd;
	bool disable_gc;
	uint8_t wait_times;
	uint8_t res[4];
	void *cb;
	void *ext;
	des_buf_t *ext_buf;
	DECL_SEM(sem)
};
#define callback_registration_t struct _callback_registration_t

struct _com_client_data_t {
COM_DATA_ELE
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t method_registry[IPC_TOKEN_NUM];
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
	reg->cmd = 0;
	reg->disable_gc = false;
	reg->wait_times = 0;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
	IPC_SEM_INIT(&reg->sem, 0);
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
	reg->busy = false;
	reg->cmd = 0;
	reg->disable_gc = false;
	reg->wait_times = 0;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
}

/**
 * @brief Takes a callback registry entry from the list.
 *
 * @param list Pointer to the list of callback registry entries.
 * @param index Pointer to the index of the current entry.
 * @param out Pointer to store the index of the taken entry.
 * @return Pointer to the taken callback registry entry, or NULL if the entry is busy.
 */
static inline callback_registration_t *
take_registry(callback_registration_t *list, _Atomic uint8_t *index,
	      uint8_t *out)
{
	uint8_t curr = 0;
	uint32_t times = 0;

	if (!list || !index)
		return NULL;

	while (true) {
#ifdef IPC_NO_ATOMIC
		curr = (*index)++;
#else
		curr = ATOMIC_FETCH_ADD(index, 1, __ATOMIC_ACQUIRE);
#endif
#if (IPC_TOKEN_NUM != 256)
		curr = curr % IPC_TOKEN_NUM;
#endif
		if (!list[curr].busy)
			break;
		// check wait times for garbage collection.
		if (!list[curr].disable_gc && ++list[curr].wait_times > IPC_TOKEN_GC_TIMES) {
			clear_registry(&list[curr]);
			IPC_LOG_INFO("GC Happens. tok %d.", curr);
			break;
		}
		if (++times >= IPC_TOKEN_NUM)
			return NULL;
	}

	list[curr].busy = true;
	if (out)
		*out = curr;
	return &list[curr];
}

/**
 * @brief Adds information to a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @param cb Pointer to the callback function.
 * @param ext Pointer to the external data.
 * @param ext_buf Pointer to the external buffer.
 * @return 0 if successful, -1 if reg is NULL.
 */
static inline int32_t add_registry(callback_registration_t *reg, void *cb,
				   void *ext, des_buf_t *ext_buf)
{
	if (!reg)
		return -1;

	reg->cmd = 0;
	reg->disable_gc = false;
	reg->wait_times = 0;
	reg->cb = cb;
	reg->ext = ext;
	reg->ext_buf = ext_buf;
	IPC_SEM_RESET(&reg->sem);
	return 0;
}

/**
 * @brief Adds information to a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @param cb Pointer to the callback function.
 * @param ext Pointer to the external data.
 * @param ext_buf Pointer to the external buffer.
 * @return 0 if successful, -1 if reg is NULL.
 */
static inline int32_t add_registry_ext(callback_registration_t *reg, void *cb,
				   void *ext, des_buf_t *ext_buf, uint8_t cmd, bool disable_gc)
{
	if (!reg)
		return -1;

	reg->cmd = cmd;
	reg->disable_gc = disable_gc;
	reg->cb = cb;
	reg->ext = ext;
	reg->ext_buf = ext_buf;
	IPC_SEM_RESET(&reg->sem);
	return 0;
}

#ifndef IPC_RTE_BAREMETAL
/**
 * @brief Waits on a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @return 0 if successful, -1 if reg is NULL.
 */
static inline int32_t wait_on_registry(callback_registration_t *reg)
{
	int32_t ret = 0;

	if (!reg)
		return -1;
#ifndef IPC_RTE_KERNEL
	ret = IPC_SEM_WAIT(&reg->sem);
	if (ret < 0)
		return -1;
#else
	IPC_SEM_WAIT(&reg->sem);
#endif
	return ret;

}

/**
 * @brief Waits on a callback registry entry with a timeout.
 *
 * @param reg Pointer to the callback registry entry.
 * @param timeout_ms Timeout in milliseconds.
 * @return 0 if successful, -1 if reg is NULL or timeout_ms is invalid.
 */
static inline int32_t timedwait_on_registry(callback_registration_t *reg,
					    int64_t timeout_ms)
{
	int32_t ret = 0;

	if (!reg || timeout_ms <= 0)
		return -1;

	IPC_SEM_TIMED_WAIT(&reg->sem, timeout_ms);
	return ret;
}

/**
 * @brief Notifies a callback registry entry.
 *
 * @param reg Pointer to the callback registry entry.
 * @return 0 if successful, -1 if reg is NULL.
 */
static inline int32_t notify_callback_registry(callback_registration_t *reg)
{
	int32_t ret = 0;

	if (!reg)
		return -1;

	if (reg->busy)
		IPC_SEM_POST(&reg->sem);

	return ret;
}
#endif

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
	reg->disable_gc = false;
	reg->cmd = 0;
	reg->wait_times = 0;
	reg->cb = NULL;
	reg->ext = NULL;
	reg->ext_buf = NULL;
	IPC_SEM_DESTROY(&reg->sem);
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

static inline int32_t send_request(com_client_data_t *data, serdes_t *ser,
			int8_t cid, int8_t cmd, void *cb, void *ext, des_buf_t *ext_buf)
{
	int32_t ret = 0;
	uint8_t tok = 0;
	callback_registration_t *reg = NULL;

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
	reg = take_registry(data->method_registry, &data->token, &tok);
	if (!reg)
		return -ERR_APP_TOK;
	(void)add_registry_ext(reg, (void *)cb, ext, ext_buf, cmd, false);
	ser->header.tok = tok;
	(void)ipc_ser_finish(ser);

	// send message
	IPC_MUTEX_LOCK(&data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(data->pid, data->handle, ser);
	IPC_MUTEX_UNLOCK(&data->send_mtx);
	if (ret < 0) {
		clear_registry(reg);
		return ret;
	}
	return tok;
}

#ifndef IPC_NO_FIRE_AND_FORGET
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
	(void)ipc_ser_finish(ser);

	// send message
	IPC_MUTEX_LOCK(&data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(data->pid, data->handle, ser);
	IPC_MUTEX_UNLOCK(&data->send_mtx);

	return ret;
}
#endif

static inline int32_t call_broadcast_sub_unsub_callback(com_client_data_t *data,
			serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	callback_registration_t *reg = NULL;
	uint32_t len = 0;
	int32_t err = 0;

	if (!data || !des)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	buf = &data->des_buf;
	clear_des_buf(buf);
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;
	ret = deserialize_32(buf, (uint32_t *)&err);
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

static inline int32_t reg_avail_changed_cb(com_client_data_t *data, avail_changed_callback_t cb, void *ext)
{
	if (!data)
		return -ERR_APP_PARAM;

	data->avail_changed_cb = cb;
	data->avail_ext = ext;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif
