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

#include "net_safety_client.h"

// macro definitions
#define CID SAFETY_0
#define CCID 0
#define CID_MASK (0x1U << 26)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_FUSAENABLE_METHOD 77U


// local variables
static com_client_data_t *s_data;
static net_safety_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _fusaenable_method_out_t {
	DECL_SEM(sem)
	uint8_t *block_id_out;
	net_safety_UInt32Array4_t **psm_id;
	net_safety_ErrorEnum_t *err;
};
#define fusaenable_method_out_t struct _fusaenable_method_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_fusaenable_method(
				serdes_t *ser,
				const uint8_t block_id
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&block_id);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void fusaenable_method_sync_callback(
				const uint8_t block_id_out,
				const net_safety_UInt32Array4_t *psm_id,
				const net_safety_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	fusaenable_method_out_t *out = (fusaenable_method_out_t *)ext;

	if (!out)
		return;
	*out->block_id_out = block_id_out;
	*out->psm_id = (net_safety_UInt32Array4_t *)psm_id;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_fusaenable_method_sync(const uint8_t block_id,
				uint8_t *block_id_out,
				net_safety_UInt32Array4_t **psm_id,
				net_safety_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	fusaenable_method_out_t out = {.block_id_out = block_id_out,
				.psm_id = psm_id,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_fusaenable_method(ser, block_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->fusaenable_method_registry, ser, s_ext->cid,
			CMD_METHOD_FUSAENABLE_METHOD, fusaenable_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->fusaenable_method_registry[ret];
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

static int32_t call_fusaenable_method_async(const uint8_t block_id,
				net_safety_fusaenable_method_callback_t cb,
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

	ret = serialize_fusaenable_method(ser, block_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->fusaenable_method_registry, ser, s_ext->cid,
			CMD_METHOD_FUSAENABLE_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_fusaenable_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	net_safety_fusaenable_method_callback_t cb = NULL;
	uint8_t block_id_out = 0;
	net_safety_UInt32Array4_t *psm_id = NULL;
	net_safety_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->fusaenable_method_registry[des->header.tok];
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
		ret = deserialize_net_safety_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == NET_SAFETY_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&block_id_out);
		if (ret >= 0)
			ret = deserialize_net_safety_UInt32Array4(buf, &psm_id);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (net_safety_fusaenable_method_callback_t)(reg->cb);
	if (cb)
		cb(block_id_out, psm_id, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {

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

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_FUSAENABLE_METHOD:
		ret = call_fusaenable_method_callback(des);
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
int32_t net_safety_client_init(com_client_data_t *data, net_safety_client_t *client,
			net_safety_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->fusaenable_method_sync = call_fusaenable_method_sync;
#endif
	client->fusaenable_method_async = call_fusaenable_method_async;
(void)init_registry(ext->fusaenable_method_registry);


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
void net_safety_client_destroy(void)
{

	s_data = NULL;
	s_ext = NULL;
}
