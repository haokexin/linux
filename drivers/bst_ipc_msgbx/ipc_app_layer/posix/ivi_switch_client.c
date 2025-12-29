// SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
/*
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

/* This file is auto generated for message box v2.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 */

#include "ivi_switch_client.h"

// macro definitions
#define CID SWITCH_1
#define CCID 0
#define CID_MASK (0x1ULL << 19)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_VMAC_METHOD 5U

#define CMD_METHOD_SUB_NOTIFY_VMAC_REINIT 10U
#define CMD_METHOD_UNSUB_NOTIFY_VMAC_REINIT 11U
#define CMD_BROADCAST_NOTIFY_VMAC_REINIT 1U

// local variables
static com_client_data_t *s_data;
static ivi_switch_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _vmac_method_out_t {
	DECL_SEM(sem)
	ivi_switch_ErrorEnum_t *err;
};
#define vmac_method_out_t struct _vmac_method_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_vmac_method(
				serdes_t *ser,
				const ivi_switch_MyArray_t vmac_msg,
				const uint32_t flag
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_ivi_switch_MyArray(ser, &vmac_msg);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&flag);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void vmac_method_sync_callback(
				const ivi_switch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	vmac_method_out_t *out = (vmac_method_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_vmac_method_sync(const ivi_switch_MyArray_t vmac_msg,
				const uint32_t flag,
				ivi_switch_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	vmac_method_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_vmac_method(ser, vmac_msg, flag);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vmac_method_registry, ser, s_ext->cid,
			CMD_METHOD_VMAC_METHOD, vmac_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->vmac_method_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_vmac_method_async(const ivi_switch_MyArray_t vmac_msg,
				const uint32_t flag,
				ivi_switch_vmac_method_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_vmac_method(ser, vmac_msg, flag);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vmac_method_registry, ser, s_ext->cid,
			CMD_METHOD_VMAC_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_vmac_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	ivi_switch_vmac_method_callback_t cb = NULL;
	ivi_switch_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->vmac_method_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_ivi_switch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (ivi_switch_vmac_method_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe notify_vmac_reinit
static int32_t subscribe_notify_vmac_reinit(
				ivi_switch_notify_vmac_reinit_callback_t cb,
				void *ext,
				des_buf_t *ext_buf,
				broadcast_sub_unsub_callback_t cb2,
				void *ext2
				)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// set registry
	s_ext->notify_vmac_reinit_registry.busy = true;
	(void)set_registry(&s_ext->notify_vmac_reinit_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_NOTIFY_VMAC_REINIT, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->notify_vmac_reinit_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe notify_vmac_reinit
static int32_t unsubscribe_notify_vmac_reinit(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->notify_vmac_reinit_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_NOTIFY_VMAC_REINIT, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_notify_vmac_reinit_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	ivi_switch_notify_vmac_reinit_callback_t cb = NULL;
	uint32_t channel = 0;
	uint8_t cmd = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->notify_vmac_reinit_registry;
	if (!reg->busy || !reg->cb) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return RESULT_SUCCESS;
	}

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&channel);
	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&cmd);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (ivi_switch_notify_vmac_reinit_callback_t)(reg->cb);
	cb(channel, cmd, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_NOTIFY_VMAC_REINIT:
		ret = call_notify_vmac_reinit_callback(des);
		break;
	default:
		ret = -ERR_APP_UNKNOWN_CMD;
		break;
	}

	return ret;
}

// dispatch_reply
static inline int32_t dispatch_reply(des_buf_t *des)
{
	int32_t ret = 0;
	com_client_data_t *data = s_data;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_VMAC_METHOD:
		ret = call_vmac_method_callback(des);
		break;
	case CMD_METHOD_SUB_NOTIFY_VMAC_REINIT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_NOTIFY_VMAC_REINIT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->notify_vmac_reinit_registry);
		break;
	default:
		ret = -ERR_APP_UNKNOWN_CMD;
		break;
	}

	return ret;
}

// register availablity changed callback function
static int32_t register_avail_changed_cb(avail_changed_callback_t cb, void *ext)
{
	if (!s_ext)
		return -ERR_APP_PARAM;

	s_ext->avail_changed_cb = cb;
	s_ext->avail_ext = ext;
	return 0;
}

// initialize client
int32_t ivi_switch_client_init(com_client_data_t *data, ivi_switch_client_t *client,
			ivi_switch_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->vmac_method_sync = call_vmac_method_sync;
#endif
	client->vmac_method_async = call_vmac_method_async;
(void)init_registry(ext->vmac_method_registry);

	client->notify_vmac_reinit_sub = subscribe_notify_vmac_reinit;
	client->notify_vmac_reinit_unsub = unsubscribe_notify_vmac_reinit;
	(void)init_registry(&ext->notify_vmac_reinit_registry);

	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	ext->cid = CID;
	ext->ccid = CCID;
	ext->cid_mask = CID_MASK;
	ext->status = false;

	return 0;
}
// destroy client
void ivi_switch_client_destroy(void)
{
	destroy_registry(&s_ext->notify_vmac_reinit_registry);

	s_data = NULL;
	s_ext = NULL;
}
