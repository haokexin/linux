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

/* This file is auto generated for message box v1.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl b083072 msgbx_ipc ad2552b
 */

#include "st_public_client.h"

// macro definitions
#define CID SAFETY_0
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_TIMESYNC_METHOD 1U
#define CMD_METHOD_QSPI_METHOD 4U
#define CMD_METHOD_SCMI_METHOD 5U
#define CMD_METHOD_GETTEMP_METHOD 6U


// local variables
static com_client_data_t *s_data;
static st_public_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _qspi_method_out_t {
	st_public_ErrorEnum_t *err;
};
#define qspi_method_out_t struct _qspi_method_out_t

struct _scmi_method_out_t {
	st_public_ErrorEnum_t *err;
};
#define scmi_method_out_t struct _scmi_method_out_t

struct _gettemp_method_out_t {
	uint32_t *reply_temp;
	st_public_ErrorEnum_t *err;
};
#define gettemp_method_out_t struct _gettemp_method_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_timesync_method(
				serdes_t *ser,
				const uint32_t sec,
				const uint32_t nsec
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&sec);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&nsec);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_timesync_method_fire_and_forget(const uint32_t sec,
				const uint32_t nsec)
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

	ret = serialize_timesync_method(ser, sec, nsec);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_TIMESYNC_METHOD);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_qspi_method(
				serdes_t *ser,
				const st_public_qspi_cmd_head_t head_msg
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_st_public_qspi_cmd_head(ser, &head_msg);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void qspi_method_sync_callback(
				const st_public_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	qspi_method_out_t *out = (qspi_method_out_t *)ext;

	if (!out)
		return;
	*out->err = err;
}

static int32_t call_qspi_method_sync(const st_public_qspi_cmd_head_t head_msg,
				st_public_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	qspi_method_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_qspi_method(ser, head_msg);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_QSPI_METHOD,
				qspi_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
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

static int32_t call_qspi_method_async(const st_public_qspi_cmd_head_t head_msg,
				st_public_qspi_method_callback_t cb,
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

	ret = serialize_qspi_method(ser, head_msg);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_QSPI_METHOD,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_qspi_method_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	st_public_ErrorEnum_t err = { 0 };


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy) {
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
		ret = deserialize_st_public_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == ST_PUBLIC_NO_ERROR) {
	
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		st_public_qspi_method_callback_t cb = (st_public_qspi_method_callback_t)(reg->cb);

		if (cb)
			cb(err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
		notify_callback_registry(reg);
#endif
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
}

static inline int32_t serialize_scmi_method(
				serdes_t *ser,
				const uint32_t addr
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&addr);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void scmi_method_sync_callback(
				const st_public_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	scmi_method_out_t *out = (scmi_method_out_t *)ext;

	if (!out)
		return;
	*out->err = err;
}

static int32_t call_scmi_method_sync(const uint32_t addr,
				st_public_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	scmi_method_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_scmi_method(ser, addr);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SCMI_METHOD,
				scmi_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
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

static int32_t call_scmi_method_async(const uint32_t addr,
				st_public_scmi_method_callback_t cb,
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

	ret = serialize_scmi_method(ser, addr);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SCMI_METHOD,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_scmi_method_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	st_public_ErrorEnum_t err = { 0 };


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy) {
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
		ret = deserialize_st_public_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == ST_PUBLIC_NO_ERROR) {
	
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		st_public_scmi_method_callback_t cb = (st_public_scmi_method_callback_t)(reg->cb);

		if (cb)
			cb(err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
		notify_callback_registry(reg);
#endif
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
}

static inline int32_t serialize_gettemp_method(
				serdes_t *ser,
				const uint32_t temp_index
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&temp_index);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void gettemp_method_sync_callback(
				const uint32_t reply_temp,
				const st_public_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	gettemp_method_out_t *out = (gettemp_method_out_t *)ext;

	if (!out)
		return;
	*out->reply_temp = reply_temp;
	*out->err = err;
}

static int32_t call_gettemp_method_sync(const uint32_t temp_index,
				uint32_t *reply_temp,
				st_public_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	gettemp_method_out_t out = {.reply_temp = reply_temp,
.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_gettemp_method(ser, temp_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_GETTEMP_METHOD,
				gettemp_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	//wait for reply
	reg = &data->method_registry[ret];
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

static int32_t call_gettemp_method_async(const uint32_t temp_index,
				st_public_gettemp_method_callback_t cb,
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

	ret = serialize_gettemp_method(ser, temp_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_GETTEMP_METHOD,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_gettemp_method_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	uint32_t reply_temp = 0;
	st_public_ErrorEnum_t err = { 0 };


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy) {
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
		ret = deserialize_st_public_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == ST_PUBLIC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&reply_temp);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		st_public_gettemp_method_callback_t cb = (st_public_gettemp_method_callback_t)(reg->cb);

		if (cb)
			cb(reply_temp, err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
		notify_callback_registry(reg);
#endif
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
}

// broadcast

// dispatch_broadcast
static inline int32_t dispatch_broadcast(serdes_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {

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

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_QSPI_METHOD:
		ret = call_qspi_method_callback(des);
		break;
	case CMD_METHOD_SCMI_METHOD:
		ret = call_scmi_method_callback(des);
		break;
	case CMD_METHOD_GETTEMP_METHOD:
		ret = call_gettemp_method_callback(des);
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
int32_t st_public_client_init(com_client_data_t *data, st_public_client_t *client,
			st_public_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
	client->timesync_method_fire_and_forget = call_timesync_method_fire_and_forget;
#ifndef IPC_RTE_BAREMETAL
	client->qspi_method_sync = call_qspi_method_sync;
#endif
	client->qspi_method_async = call_qspi_method_async;
#ifndef IPC_RTE_BAREMETAL
	client->scmi_method_sync = call_scmi_method_sync;
#endif
	client->scmi_method_async = call_scmi_method_async;
#ifndef IPC_RTE_BAREMETAL
	client->gettemp_method_sync = call_gettemp_method_sync;
#endif
	client->gettemp_method_async = call_gettemp_method_async;


	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	if (ext->cid == 0)
		ext->cid = CID;

	return 0;
}
// destroy client
void st_public_client_destroy(void)
{

	s_data = NULL;
	s_ext = NULL;
}
