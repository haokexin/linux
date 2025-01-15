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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 797e374 msgbx_ipc c468e33
 */

#include "bst_touch_client.h"

// macro definitions
#define CID SAFETY_0
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT 90U
#define CMD_METHOD_SET_TOUCH_CALIBRATION 91U
#define CMD_METHOD_GET_TOUCH_CALIBRATION 92U

#define CMD_METHOD_SUB_LOCATION_INFO 95U
#define CMD_METHOD_UNSUB_LOCATION_INFO 96U
#define CMD_BROADCAST_LOCATION_INFO 36U

// local variables
static com_client_data_t *s_data;
static bst_touch_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _client_request_location_init_out_t {
	uint64_t *client_uuid;
	bst_touch_hw_info_t **screen_hwinfo;
	bst_touch_ErrorEnum_t *err;
};
#define client_request_location_init_out_t struct _client_request_location_init_out_t

struct _set_touch_calibration_out_t {
	bst_touch_ErrorEnum_t *err;
};
#define set_touch_calibration_out_t struct _set_touch_calibration_out_t

struct _get_touch_calibration_out_t {
	bst_touch_calibration_info_t **cali_info;
	bst_touch_ErrorEnum_t *err;
};
#define get_touch_calibration_out_t struct _get_touch_calibration_out_t

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
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_client_request_location_init(ser, client_id, req_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT,
				client_request_location_init_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
	reg->disable_gc = true;
	if (timeout_ms <= 0)
		ret = wait_on_registry(reg);
	else
		ret = timedwait_on_registry(reg, timeout_ms);
	if (ret < 0) {
		clear_registry(reg);
		IPC_LOG_ERR("wait timeout\n");
	}

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
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_client_request_location_init(ser, client_id, req_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_client_request_location_init_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	uint64_t client_uuid = 0;
	bst_touch_hw_info_t *screen_hwinfo = NULL;
	bst_touch_ErrorEnum_t err = 0;
	bst_touch_client_request_location_init_callback_t cb;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_CLIENT_REQUEST_LOCATION_INIT) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	buf = reg->ext_buf ? reg->ext_buf : &data->des_buf;
	clear_des_buf(buf);
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	// deserialize arguments
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

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
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

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
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_set_touch_calibration(ser, screen_id, cali_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SET_TOUCH_CALIBRATION,
				set_touch_calibration_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
	reg->disable_gc = true;
	if (timeout_ms <= 0)
		ret = wait_on_registry(reg);
	else
		ret = timedwait_on_registry(reg, timeout_ms);
	if (ret < 0) {
		clear_registry(reg);
		IPC_LOG_ERR("wait timeout\n");
	}

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
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_set_touch_calibration(ser, screen_id, cali_info);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SET_TOUCH_CALIBRATION,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_set_touch_calibration_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	bst_touch_ErrorEnum_t err = 0;
	bst_touch_set_touch_calibration_callback_t cb;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_SET_TOUCH_CALIBRATION) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	buf = reg->ext_buf ? reg->ext_buf : &data->des_buf;
	clear_des_buf(buf);
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	// deserialize arguments
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

	if (ret >= 0)
		ret = deserialize_bst_touch_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == BST_TOUCH_NO_ERROR) {
	
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (bst_touch_set_touch_calibration_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

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
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_get_touch_calibration(ser, screen_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_GET_TOUCH_CALIBRATION,
				get_touch_calibration_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
	reg->disable_gc = true;
	if (timeout_ms <= 0)
		ret = wait_on_registry(reg);
	else
		ret = timedwait_on_registry(reg, timeout_ms);
	if (ret < 0) {
		clear_registry(reg);
		IPC_LOG_ERR("wait timeout\n");
	}

	return ret;
}
#endif

static int32_t call_get_touch_calibration_async(const uint32_t screen_id,
				bst_touch_get_touch_calibration_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_get_touch_calibration(ser, screen_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_GET_TOUCH_CALIBRATION,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_get_touch_calibration_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	bst_touch_calibration_info_t *cali_info = NULL;
	bst_touch_ErrorEnum_t err = 0;
	bst_touch_get_touch_calibration_callback_t cb;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_GET_TOUCH_CALIBRATION) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	buf = reg->ext_buf ? reg->ext_buf : &data->des_buf;
	clear_des_buf(buf);
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	// deserialize arguments
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

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
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

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

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_LOCATION_INFO,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->location_info_registry.busy = true;
	(void)add_registry(&s_ext->location_info_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe location_info
static int32_t unsubscribe_location_info(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_LOCATION_INFO,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_location_info_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
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
	data->info.timestamp = des->recv_end_time;
	buf = reg->ext_buf ? reg->ext_buf : &data->des_buf;
	clear_des_buf(buf);
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

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
static inline int32_t dispatch_broadcast(serdes_t *des)
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
		IPC_LOG_ERR("unknown broadcast message %d.\n", des->header.cmd);
		break;
	}

	return ret;
}

// dispatch_reply
static inline int32_t dispatch_reply(serdes_t *des)
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
		IPC_LOG_ERR("unknown reply message %d.\n", des->header.cmd);
		break;
	}

	return ret;
}

// register availablity changed callback function
static int32_t register_avail_changed_cb(avail_changed_callback_t cb, void *ext)
{
	return reg_avail_changed_cb(s_data, cb, ext);
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
#ifndef IPC_RTE_BAREMETAL
	client->set_touch_calibration_sync = call_set_touch_calibration_sync;
#endif
	client->set_touch_calibration_async = call_set_touch_calibration_async;
#ifndef IPC_RTE_BAREMETAL
	client->get_touch_calibration_sync = call_get_touch_calibration_sync;
#endif
	client->get_touch_calibration_async = call_get_touch_calibration_async;

	client->location_info_sub = subscribe_location_info;
	client->location_info_unsub = unsubscribe_location_info;
	(void)init_registry(&ext->location_info_registry);
	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	if (ext->cid == 0)
		ext->cid = CID;

	return 0;
}
// destroy client
void bst_touch_client_destroy(void)
{
	destroy_registry(&s_ext->location_info_registry);
	s_data = NULL;
	s_ext = NULL;
}
