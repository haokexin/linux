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

#include "bst_touch_client.h"

// macro definitions
#define CID DMA_0
#define CCID 0
#define CID_MASK (0x1U << 16)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT 90U
#define CMD_METHOD_SET_TOUCH_CALIBRATION 91U
#define CMD_METHOD_GET_TOUCH_CALIBRATION 92U
#define CMD_METHOD_TOUCH_DEBUG_CMD 93U

#define CMD_METHOD_SUB_LOCATION_INFO 95U
#define CMD_METHOD_UNSUB_LOCATION_INFO 96U
#define CMD_BROADCAST_LOCATION_INFO 36U

// local variables
static com_client_data_t *s_data;
static bst_touch_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _client_request_location_init_out_t {
	DECL_SEM(sem)
	uint64_t *client_uuid;
	bst_touch_hw_info_t **screen_hwinfo;
	bst_touch_ErrorEnum_t *err;
};
#define client_request_location_init_out_t struct _client_request_location_init_out_t

struct _set_touch_calibration_out_t {
	DECL_SEM(sem)
	bst_touch_ErrorEnum_t *err;
};
#define set_touch_calibration_out_t struct _set_touch_calibration_out_t

struct _get_touch_calibration_out_t {
	DECL_SEM(sem)
	bst_touch_calibration_info_t **cali_info;
	bst_touch_ErrorEnum_t *err;
};
#define get_touch_calibration_out_t struct _get_touch_calibration_out_t

struct _touch_debug_cmd_out_t {
	DECL_SEM(sem)
	char **result_str;
	bst_touch_ErrorEnum_t *err;
};
#define touch_debug_cmd_out_t struct _touch_debug_cmd_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_client_request_location_init(
				serdes_t *ser,
				const uint32_t client_id,
				const bst_touch_request_info_t *req_info
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&client_id);
	if (ret >= 0)
		ret = serialize_bst_touch_request_info(ser, req_info);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void client_request_location_init_sync_callback(
				const uint64_t client_uuid,
				const bst_touch_hw_info_t *screen_hwinfo,
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	client_request_location_init_out_t *out = (client_request_location_init_out_t *)ext;

	if (!out)
		return;
	*out->client_uuid = client_uuid;
	*out->screen_hwinfo = (bst_touch_hw_info_t *)screen_hwinfo;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_client_request_location_init_sync(const uint32_t client_id,
				const bst_touch_request_info_t *req_info,
				uint64_t *client_uuid,
				bst_touch_hw_info_t **screen_hwinfo,
				bst_touch_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	client_request_location_init_out_t out = {.client_uuid = client_uuid,
				.screen_hwinfo = screen_hwinfo,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_client_request_location_init(ser, client_id, req_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->client_request_location_init_registry, ser, s_ext->cid,
			CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT, client_request_location_init_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->client_request_location_init_registry[ret];
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

static int32_t call_client_request_location_init_async(const uint32_t client_id,
				const bst_touch_request_info_t *req_info,
				bst_touch_client_request_location_init_callback_t cb,
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

	ret = serialize_client_request_location_init(ser, client_id, req_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->client_request_location_init_registry, ser, s_ext->cid,
			CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_client_request_location_init_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	bst_touch_client_request_location_init_callback_t cb = NULL;
	uint64_t client_uuid = 0;
	bst_touch_hw_info_t *screen_hwinfo = NULL;
	bst_touch_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->client_request_location_init_registry[des->header.tok];
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
		ret = deserialize_bst_touch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == BST_TOUCH_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_64(buf, (uint64_t *)&client_uuid);
		if (ret >= 0)
			ret = deserialize_bst_touch_hw_info(buf, &screen_hwinfo);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (bst_touch_client_request_location_init_callback_t)(reg->cb);
	if (cb)
		cb(client_uuid, screen_hwinfo, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_set_touch_calibration(
				serdes_t *ser,
				const uint32_t screen_id,
				const bst_touch_calibration_info_t *cali_info
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&screen_id);
	if (ret >= 0)
		ret = serialize_bst_touch_calibration_info(ser, cali_info);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void set_touch_calibration_sync_callback(
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	set_touch_calibration_out_t *out = (set_touch_calibration_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_set_touch_calibration_sync(const uint32_t screen_id,
				const bst_touch_calibration_info_t *cali_info,
				bst_touch_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	set_touch_calibration_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_set_touch_calibration(ser, screen_id, cali_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->set_touch_calibration_registry, ser, s_ext->cid,
			CMD_METHOD_SET_TOUCH_CALIBRATION, set_touch_calibration_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->set_touch_calibration_registry[ret];
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

static int32_t call_set_touch_calibration_async(const uint32_t screen_id,
				const bst_touch_calibration_info_t *cali_info,
				bst_touch_set_touch_calibration_callback_t cb,
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

	ret = serialize_set_touch_calibration(ser, screen_id, cali_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->set_touch_calibration_registry, ser, s_ext->cid,
			CMD_METHOD_SET_TOUCH_CALIBRATION, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_set_touch_calibration_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	bst_touch_set_touch_calibration_callback_t cb = NULL;
	bst_touch_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->set_touch_calibration_registry[des->header.tok];
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
		ret = deserialize_bst_touch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (bst_touch_set_touch_calibration_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_get_touch_calibration(
				serdes_t *ser,
				const uint32_t screen_id
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&screen_id);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void get_touch_calibration_sync_callback(
				const bst_touch_calibration_info_t *cali_info,
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	get_touch_calibration_out_t *out = (get_touch_calibration_out_t *)ext;

	if (!out)
		return;
	*out->cali_info = (bst_touch_calibration_info_t *)cali_info;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_get_touch_calibration_sync(const uint32_t screen_id,
				bst_touch_calibration_info_t **cali_info,
				bst_touch_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	get_touch_calibration_out_t out = {.cali_info = cali_info,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_get_touch_calibration(ser, screen_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->get_touch_calibration_registry, ser, s_ext->cid,
			CMD_METHOD_GET_TOUCH_CALIBRATION, get_touch_calibration_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->get_touch_calibration_registry[ret];
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

static int32_t call_get_touch_calibration_async(const uint32_t screen_id,
				bst_touch_get_touch_calibration_callback_t cb,
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

	ret = serialize_get_touch_calibration(ser, screen_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->get_touch_calibration_registry, ser, s_ext->cid,
			CMD_METHOD_GET_TOUCH_CALIBRATION, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_get_touch_calibration_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	bst_touch_get_touch_calibration_callback_t cb = NULL;
	bst_touch_calibration_info_t *cali_info = NULL;
	bst_touch_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->get_touch_calibration_registry[des->header.tok];
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
		ret = deserialize_bst_touch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == BST_TOUCH_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_bst_touch_calibration_info(buf, &cali_info);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (bst_touch_get_touch_calibration_callback_t)(reg->cb);
	if (cb)
		cb(cali_info, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_touch_debug_cmd(
				serdes_t *ser,
				const char *cmd_str
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_string(ser, cmd_str);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void touch_debug_cmd_sync_callback(
				const char *result_str,
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	touch_debug_cmd_out_t *out = (touch_debug_cmd_out_t *)ext;

	if (!out)
		return;
	*out->result_str = (char *)result_str;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_touch_debug_cmd_sync(const char *cmd_str,
				char **result_str,
				bst_touch_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	touch_debug_cmd_out_t out = {.result_str = result_str,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_touch_debug_cmd(ser, cmd_str);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->touch_debug_cmd_registry, ser, s_ext->cid,
			CMD_METHOD_TOUCH_DEBUG_CMD, touch_debug_cmd_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->touch_debug_cmd_registry[ret];
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

static int32_t call_touch_debug_cmd_async(const char *cmd_str,
				bst_touch_touch_debug_cmd_callback_t cb,
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

	ret = serialize_touch_debug_cmd(ser, cmd_str);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->touch_debug_cmd_registry, ser, s_ext->cid,
			CMD_METHOD_TOUCH_DEBUG_CMD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_touch_debug_cmd_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	bst_touch_touch_debug_cmd_callback_t cb = NULL;
	char *result_str = NULL;
	bst_touch_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->touch_debug_cmd_registry[des->header.tok];
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
		ret = deserialize_bst_touch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == BST_TOUCH_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(buf, &result_str);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (bst_touch_touch_debug_cmd_callback_t)(reg->cb);
	if (cb)
		cb(result_str, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe location_info
static int32_t subscribe_location_info(
				bst_touch_location_info_callback_t cb,
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
	s_ext->location_info_registry.busy = true;
	(void)set_registry(&s_ext->location_info_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_LOCATION_INFO, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->location_info_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe location_info
static int32_t unsubscribe_location_info(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->location_info_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_LOCATION_INFO, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_location_info_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	bst_touch_location_info_callback_t cb = NULL;
	uint32_t screen_id = 0;
	uint32_t locinfo_offset = 0;
	uint32_t locinfo_size = 0;
	uint32_t locinfo_chksum = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->location_info_registry;
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
		ret = deserialize_32(buf, (uint32_t *)&screen_id);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&locinfo_offset);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&locinfo_size);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&locinfo_chksum);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (bst_touch_location_info_callback_t)(reg->cb);
	cb(screen_id, locinfo_offset, locinfo_size, locinfo_chksum, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_LOCATION_INFO:
		ret = call_location_info_callback(des);
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
	case CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT:
		ret = call_client_request_location_init_callback(des);
		break;
	case CMD_METHOD_SET_TOUCH_CALIBRATION:
		ret = call_set_touch_calibration_callback(des);
		break;
	case CMD_METHOD_GET_TOUCH_CALIBRATION:
		ret = call_get_touch_calibration_callback(des);
		break;
	case CMD_METHOD_TOUCH_DEBUG_CMD:
		ret = call_touch_debug_cmd_callback(des);
		break;
	case CMD_METHOD_SUB_LOCATION_INFO:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_LOCATION_INFO:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->location_info_registry);
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
int32_t bst_touch_client_init(com_client_data_t *data, bst_touch_client_t *client,
			bst_touch_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->client_request_location_init_sync = call_client_request_location_init_sync;
#endif
	client->client_request_location_init_async = call_client_request_location_init_async;
(void)init_registry(ext->client_request_location_init_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->set_touch_calibration_sync = call_set_touch_calibration_sync;
#endif
	client->set_touch_calibration_async = call_set_touch_calibration_async;
(void)init_registry(ext->set_touch_calibration_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->get_touch_calibration_sync = call_get_touch_calibration_sync;
#endif
	client->get_touch_calibration_async = call_get_touch_calibration_async;
(void)init_registry(ext->get_touch_calibration_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->touch_debug_cmd_sync = call_touch_debug_cmd_sync;
#endif
	client->touch_debug_cmd_async = call_touch_debug_cmd_async;
(void)init_registry(ext->touch_debug_cmd_registry);

	client->location_info_sub = subscribe_location_info;
	client->location_info_unsub = unsubscribe_location_info;
	(void)init_registry(&ext->location_info_registry);

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
void bst_touch_client_destroy(void)
{
	destroy_registry(&s_ext->location_info_registry);

	s_data = NULL;
	s_ext = NULL;
}
