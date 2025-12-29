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

#include "backlight_client.h"

// macro definitions
#define CID SAFETY_0
#define CCID 0
#define CID_MASK (0x1U << 26)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_VIRT_BL_REQUEST 80U

#define CMD_METHOD_SUB_VIRT_BL_BROADCAST 86U
#define CMD_METHOD_UNSUB_VIRT_BL_BROADCAST 87U
#define CMD_BROADCAST_VIRT_BL_BROADCAST 88U

// local variables
static com_client_data_t *s_data;
static backlight_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _virt_bl_request_out_t {
	DECL_SEM(sem)
	backlight_virt_bl_msg_t *rsp;
	backlight_ErrorEnum_t *err;
};
#define virt_bl_request_out_t struct _virt_bl_request_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_virt_bl_request(
				serdes_t *ser,
				const backlight_virt_bl_msg_t req
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_backlight_virt_bl_msg(ser, &req);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void virt_bl_request_sync_callback(
				const backlight_virt_bl_msg_t rsp,
				const backlight_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	virt_bl_request_out_t *out = (virt_bl_request_out_t *)ext;

	if (!out)
		return;
	out->rsp->msg_len = rsp.msg_len;
	out->rsp->cmd_id = rsp.cmd_id;
	out->rsp->content.size = rsp.content.size;
	out->rsp->content.data = rsp.content.data;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_virt_bl_request_sync(const backlight_virt_bl_msg_t req,
				backlight_virt_bl_msg_t *rsp,
				backlight_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	virt_bl_request_out_t out = {.rsp = rsp,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_virt_bl_request(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->virt_bl_request_registry, ser, s_ext->cid,
			CMD_METHOD_VIRT_BL_REQUEST, virt_bl_request_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->virt_bl_request_registry[ret];
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

static int32_t call_virt_bl_request_async(const backlight_virt_bl_msg_t req,
				backlight_virt_bl_request_callback_t cb,
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

	ret = serialize_virt_bl_request(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->virt_bl_request_registry, ser, s_ext->cid,
			CMD_METHOD_VIRT_BL_REQUEST, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_virt_bl_request_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	backlight_virt_bl_request_callback_t cb = NULL;
	backlight_virt_bl_msg_t rsp = { 0 };
	backlight_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->virt_bl_request_registry[des->header.tok];
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
		ret = deserialize_backlight_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == BACKLIGHT_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_backlight_virt_bl_msg(buf, &rsp);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (backlight_virt_bl_request_callback_t)(reg->cb);
	if (cb)
		cb(rsp, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe virt_bl_broadcast
static int32_t subscribe_virt_bl_broadcast(
				backlight_virt_bl_broadcast_callback_t cb,
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
	s_ext->virt_bl_broadcast_registry.busy = true;
	(void)set_registry(&s_ext->virt_bl_broadcast_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_VIRT_BL_BROADCAST, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->virt_bl_broadcast_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe virt_bl_broadcast
static int32_t unsubscribe_virt_bl_broadcast(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->virt_bl_broadcast_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_VIRT_BL_BROADCAST, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_virt_bl_broadcast_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	backlight_virt_bl_broadcast_callback_t cb = NULL;
	backlight_virt_bl_msg_t evt = { 0 };

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->virt_bl_broadcast_registry;
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
		ret = deserialize_backlight_virt_bl_msg(buf, &evt);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (backlight_virt_bl_broadcast_callback_t)(reg->cb);
	cb(evt, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_VIRT_BL_BROADCAST:
		ret = call_virt_bl_broadcast_callback(des);
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
	case CMD_METHOD_VIRT_BL_REQUEST:
		ret = call_virt_bl_request_callback(des);
		break;
	case CMD_METHOD_SUB_VIRT_BL_BROADCAST:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_VIRT_BL_BROADCAST:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->virt_bl_broadcast_registry);
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
int32_t backlight_client_init(com_client_data_t *data, backlight_client_t *client,
			backlight_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->virt_bl_request_sync = call_virt_bl_request_sync;
#endif
	client->virt_bl_request_async = call_virt_bl_request_async;
(void)init_registry(ext->virt_bl_request_registry);

	client->virt_bl_broadcast_sub = subscribe_virt_bl_broadcast;
	client->virt_bl_broadcast_unsub = unsubscribe_virt_bl_broadcast;
	(void)init_registry(&ext->virt_bl_broadcast_registry);

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
void backlight_client_destroy(void)
{
	destroy_registry(&s_ext->virt_bl_broadcast_registry);

	s_data = NULL;
	s_ext = NULL;
}
