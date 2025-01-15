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

#include "cvdsp2_client.h"

// macro definitions
#define CID ISPCV_2
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_DISP_REQ 0U
#define CMD_METHOD_DISP_RUN 1U


// local variables
static com_client_data_t *s_data;
static cvdsp2_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _disp_req_out_t {
	uint32_t *rep;
	cvdsp2_ErrorEnum_t *err;
};
#define disp_req_out_t struct _disp_req_out_t

struct _disp_run_out_t {
	uint32_t *status;
	uint32_t *perf_us;
	cvdsp2_ErrorEnum_t *err;
};
#define disp_run_out_t struct _disp_run_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_disp_req(
				serdes_t *ser,
				const uint32_t req
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&req);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void disp_req_sync_callback(
				const uint32_t rep,
				const cvdsp2_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	disp_req_out_t *out = (disp_req_out_t *)ext;

	if (!out)
		return;
	*out->rep = rep;
	*out->err = err;
}

static int32_t call_disp_req_sync(const uint32_t req,
				uint32_t *rep,
				cvdsp2_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	disp_req_out_t out = {.rep = rep,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_disp_req(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_DISP_REQ,
				disp_req_sync_callback, &out, ext_buf);
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

static int32_t call_disp_req_async(const uint32_t req,
				cvdsp2_disp_req_callback_t cb,
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

	ret = serialize_disp_req(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_DISP_REQ,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_disp_req_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	uint32_t rep = 0;
	cvdsp2_ErrorEnum_t err = 0;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_DISP_REQ) {
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
		ret = deserialize_cvdsp2_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == CVDSP2_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&rep);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	{
	// call callback function
	cvdsp2_disp_req_callback_t cb = (cvdsp2_disp_req_callback_t)(reg->cb);
	if (cb)
		cb(rep, err, reg->ext, &data->info);
	}
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_disp_run(
				serdes_t *ser,
				const uint32_t opcode,
				const uint32_t opdata
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&opcode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&opdata);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void disp_run_sync_callback(
				const uint32_t status,
				const uint32_t perf_us,
				const cvdsp2_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	disp_run_out_t *out = (disp_run_out_t *)ext;

	if (!out)
		return;
	*out->status = status;
	*out->perf_us = perf_us;
	*out->err = err;
}

static int32_t call_disp_run_sync(const uint32_t opcode,
				const uint32_t opdata,
				uint32_t *status,
				uint32_t *perf_us,
				cvdsp2_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	disp_run_out_t out = {.status = status,
				.perf_us = perf_us,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_disp_run(ser, opcode, opdata);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_DISP_RUN,
				disp_run_sync_callback, &out, ext_buf);
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

static int32_t call_disp_run_async(const uint32_t opcode,
				const uint32_t opdata,
				cvdsp2_disp_run_callback_t cb,
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

	ret = serialize_disp_run(ser, opcode, opdata);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_DISP_RUN,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_disp_run_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	uint32_t status = 0;
	uint32_t perf_us = 0;
	cvdsp2_ErrorEnum_t err = 0;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_DISP_RUN) {
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
		ret = deserialize_cvdsp2_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == CVDSP2_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&status);
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&perf_us);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	{
	cvdsp2_disp_run_callback_t cb = (cvdsp2_disp_run_callback_t)(reg->cb);
	if (cb)
		cb(status, perf_us, err, reg->ext, &data->info);
	}
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

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
	case CMD_METHOD_DISP_REQ:
		ret = call_disp_req_callback(des);
		break;
	case CMD_METHOD_DISP_RUN:
		ret = call_disp_run_callback(des);
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
int32_t cvdsp2_client_init(com_client_data_t *data, cvdsp2_client_t *client,
			cvdsp2_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->disp_req_sync = call_disp_req_sync;
#endif
	client->disp_req_async = call_disp_req_async;
#ifndef IPC_RTE_BAREMETAL
	client->disp_run_sync = call_disp_run_sync;
#endif
	client->disp_run_async = call_disp_run_async;


	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	if (ext->cid == 0)
		ext->cid = CID;

	return 0;
}
// destroy client
void cvdsp2_client_destroy(void)
{

	s_data = NULL;
	s_ext = NULL;
}
