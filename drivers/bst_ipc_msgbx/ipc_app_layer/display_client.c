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

/* This file is auto generated for message box v0.2.2.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: f27fcbb
 */

#define IPC_APP_RTE_LINUX_KERNEL
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>
#include "display_client.h"

// macro definitions
#define PID CPU_6
#define CID CPU_7
#define FID DEF
#define SID 1U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_COMPLEX_METHOD 2U
#define CMD_METHOD_NO_REPLY_METHOD 3U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// internal data structure
struct _test_client_data_t {
	test_client_t client;
	ipc_inf_version_t version;
	uint8_t handle;
	bool initialized;
	des_buf_t des_buf;
	callback_registration_t method_registry[IPC_TOKEN_NUM];
	callback_registration_t heartbeat_registry;
	serdes_t serializer;
	serdes_t deserializer;
	struct task_struct *route_task;

	_Atomic bool bRunning;
	struct mutex send_mtx;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	ext_info_t info;
};
#define test_client_data_t struct _test_client_data_t

// local variables
static test_client_data_t *s_data;
static _Atomic uint8_t s_token;
static test_client_data_t s_internal_data;
// private data structure

struct _hello_out_t {
	char **message;
	test_ErrorEnum_t *err;
};
#define hello_out_t struct _hello_out_t

struct _complex_method_out_t {
	uint32_t *out1;
	char **out2;
	byte_buffer_t *out3;
	test_MyArray_t *out4;
	test_MyStruct_t *out5;
	test_MyUnion_t *out6;
	test_ErrorEnum_t *err;
};
#define complex_method_out_t struct _complex_method_out_t

// interface implementation
static inline int32_t send_request(serdes_t *ser, int8_t cmd, void *cb,
				void *ext, des_buf_t *ext_buf)
{
	int32_t ret = 0;
	uint8_t tok = 0;
	callback_registration_t *reg = NULL;

	if (!ser || !s_data)
		return -ERR_APP_PARAM;

	// prepare message
	ser->header.pid = PID;
	ser->header.cid = CID;
	ser->header.fid = FID;
	ser->header.sid = SID;
	ser->header.cmd = cmd;
	ser->header.typ = IPC_MSG_TYPE_METHOD;

	// take and set registry
	reg = take_registry(s_data->method_registry, &s_token, &tok);
	if (!reg)
		return -ERR_APP_TOK;
	(void)add_registry(reg, (void *)cb, ext, ext_buf);
	ser->header.tok = tok;
	(void)ipc_ser_finish(ser);

	// send message
	mutex_lock(&s_data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(PID, s_data->handle, ser);
	mutex_unlock(&s_data->send_mtx);
	if (ret < 0) {
		clear_registry(reg);
		return ret;
	}
	return tok;
}

static inline int32_t send_fire_and_forget_request(serdes_t *ser, int8_t cmd)
{
	int32_t ret = 0;

	if (!ser || !s_data)
		return -ERR_APP_PARAM;

	// prepare message
	ser->header.pid = PID;
	ser->header.cid = CID;
	ser->header.fid = FID;
	ser->header.sid = SID;
	ser->header.cmd = cmd;
	ser->header.typ = IPC_MSG_TYPE_METHOD;
	ser->header.tok = 0;
	(void)ipc_ser_finish(ser);

	// send message
	mutex_lock(&s_data->send_mtx);
	ret = ipc_trans_layer_proxy_send_method(PID, s_data->handle, ser);
	mutex_unlock(&s_data->send_mtx);

	return ret;
}

// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { 0 };

	return s_data ? s_data->version : ret;
}

// method

static void hello_sync_callback(
				const char *message,
				const test_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	hello_out_t *out = (hello_out_t *)ext;

	if (!out)
		return;
	*out->message = (char *)message;
	*out->err = err;
}

static inline int32_t serialize_hello(
				serdes_t *ser,
				const char *name
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_string(ser, name);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_hello_sync(const char *name,
				char **message,
				test_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	hello_out_t out = {.message = message, .err = err};
	callback_registration_t *reg = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		printf("%s: serialize hello fail.\n", __func__);
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(ser, CMD_METHOD_HELLO, (void *)hello_sync_callback, (void *)&out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send method hello fail %d.\n", __func__, ret);
		return ret;
	}

	//wait for reply
	reg = &s_data->method_registry[ret];
	if (timeout_ms <= 0)
		ret = wait_on_registry(reg);
	else
		ret = timedwait_on_registry(reg, timeout_ms);
	if (ret < 0) {
		clear_registry(reg);
		printf("%s: wait timeout\n", __func__);
	}

	return ret;
}

static int32_t call_hello_async(const char *name,
				test_hello_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		printf("%s: serialize hello fail.\n", __func__);
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(ser, CMD_METHOD_HELLO, (void *)cb, (void *)ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send method hello fail %d.\n", __func__, ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hello_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	char *message = NULL;
	test_ErrorEnum_t err = { 0 };


	if (!des || !s_data)
		return -ERR_APP_PARAM;

	reg = &s_data->method_registry[des->header.tok];
	if (!reg->busy) {
		printf("%s: callback registry is invalid.\n", __func__);
		return -ERR_APP_TOK;
	}
	buf = reg->ext_buf ? reg->ext_buf : &s_data->des_buf;
	clear_des_buf(buf);
	s_data->info.uuid = ipc_msg_get_uuid(des->header);
	s_data->info.timestamp = des->recv_end_time;

	// deserialize arguments
	if (ret >= 0)
		ret = deserialize_ErrorEnum(des, &err, buf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(des, &message, buf);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);

		if (cb)
			cb(message, err, reg->ext, &s_data->info);
		notify_callback_registry(reg);
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
}

static void complex_method_sync_callback(
				const uint32_t out1,
				const char *out2,
				const byte_buffer_t out3,
				const test_MyArray_t out4,
				const test_MyStruct_t out5,
				const test_MyUnion_t out6,
				const test_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	complex_method_out_t *out = (complex_method_out_t *)ext;

	if (!out)
		return;
	*out->out1 = out1;
	*out->out2 = (char *)out2;
	out->out3->size = out3.size;
	out->out3->data = out3.data;
	out->out4->size = out4.size;
	out->out4->data = out4.data;
	out->out5->m1 = out5.m1;
	out->out5->m2 = out5.m2;
	out->out5->m3.size = out5.m3.size;
	out->out5->m3.data = out5.m3.data;
	out->out5->m4 = (char *)out5.m4;
	out->out5->m5.size = out5.m5.size;
	out->out5->m5.data = out5.m5.data;
	*out->out6 = out6;
	*out->err = err;
}

static inline int32_t serialize_complex_method(
				serdes_t *ser,
				const uint32_t in1,
				const char *in2,
				const byte_buffer_t in3,
				const test_MyArray_t in4,
				const test_MyStruct_t in5,
				const test_MyUnion_t in6
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)&in1, sizeof(uint32_t));
	if (ret >= 0)
		ret = serialize_string(ser, in2);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in3);
	if (ret >= 0)
		ret = serialize_MyArray(ser, &in4);
	if (ret >= 0)
		ret = serialize_MyStruct(ser, &in5);
	if (ret >= 0)
		ret = serialize_MyUnion(ser, &in6);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_complex_method_sync(const uint32_t in1,
				const char *in2,
				const byte_buffer_t in3,
				const test_MyArray_t in4,
				const test_MyStruct_t in5,
				const test_MyUnion_t in6,
				uint32_t *out1,
				char **out2,
				byte_buffer_t *out3,
				test_MyArray_t *out4,
				test_MyStruct_t *out5,
				test_MyUnion_t *out6,
				test_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	complex_method_out_t out = {.out1 = out1, .out2 = out2, .out3 = out3, .out4 = out4, .out5 = out5, .out6 = out6, .err = err};
	callback_registration_t *reg = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_complex_method(ser, in1, in2, in3, in4, in5, in6);
	if (ret != 0) {
		printf("%s: serialize complex_method fail.\n", __func__);
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(ser, CMD_METHOD_COMPLEX_METHOD, (void *)complex_method_sync_callback, (void *)&out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send method complex_method fail %d.\n", __func__, ret);
		return ret;
	}

	//wait for reply
	reg = &s_data->method_registry[ret];
	if (timeout_ms <= 0)
		ret = wait_on_registry(reg);
	else
		ret = timedwait_on_registry(reg, timeout_ms);
	if (ret < 0) {
		clear_registry(reg);
		printf("%s: wait timeout\n", __func__);
	}

	return ret;
}

static int32_t call_complex_method_async(const uint32_t in1,
				const char *in2,
				const byte_buffer_t in3,
				const test_MyArray_t in4,
				const test_MyStruct_t in5,
				const test_MyUnion_t in6,
				test_complex_method_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_complex_method(ser, in1, in2, in3, in4, in5, in6);
	if (ret != 0) {
		printf("%s: serialize complex_method fail.\n", __func__);
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(ser, CMD_METHOD_COMPLEX_METHOD, (void *)cb, (void *)ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send method complex_method fail %d.\n", __func__, ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_complex_method_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t out1 = 0;
	char *out2 = NULL;
	byte_buffer_t out3 = { 0 };
	test_MyArray_t out4 = { 0 };
	test_MyStruct_t out5 = { 0 };
	test_MyUnion_t out6 = { 0 };
	test_ErrorEnum_t err = { 0 };


	if (!des || !s_data)
		return -ERR_APP_PARAM;

	reg = &s_data->method_registry[des->header.tok];
	if (!reg->busy) {
		printf("%s: callback registry is invalid.\n", __func__);
		return -ERR_APP_TOK;
	}
	buf = reg->ext_buf ? reg->ext_buf : &s_data->des_buf;
	clear_des_buf(buf);
	s_data->info.uuid = ipc_msg_get_uuid(des->header);
	s_data->info.timestamp = des->recv_end_time;

	// deserialize arguments
	if (ret >= 0)
		ret = deserialize_ErrorEnum(des, &err, buf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == NO_ERROR) {
		if (ret >= 0)
			ret = ipc_des_get(des, (uint8_t *)&out1, sizeof(uint32_t));
		if (ret >= 0)
			ret = deserialize_string(des, &out2, buf);
		if (ret >= 0)
			ret = deserialize_byte_buffer(des, &out3, buf);
		if (ret >= 0)
			ret = deserialize_MyArray(des, &out4, buf);
		if (ret >= 0)
			ret = deserialize_MyStruct(des, &out5, buf);
		if (ret >= 0)
			ret = deserialize_MyUnion(des, &out6, buf);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		test_complex_method_callback_t cb = (test_complex_method_callback_t)(reg->cb);

		if (cb)
			cb(out1, out2, out3, out4, out5, out6, err, reg->ext, &s_data->info);
		notify_callback_registry(reg);
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
}

static inline int32_t serialize_no_reply_method(
				serdes_t *ser,
				const uint8_t status
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)&status, sizeof(uint8_t));

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_no_reply_method_fire_and_forget(const uint8_t status)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_no_reply_method(ser, status);
	if (ret != 0) {
		printf("%s: serialize no_reply_method fail.\n", __func__);
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(ser, CMD_METHOD_NO_REPLY_METHOD);
	if (ret < 0) {
		printf("%s: send method no_reply_method fail %d.\n", __func__, ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

// broadcast

// subscribe heartbeat
static int32_t subscribe_heartbeat(
				test_heartbeat_callback_t cb,
				void *ext,
				des_buf_t *ext_buf,
				test_heartbeat_sub_callback_t cb2,
				void *ext2
				)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;

	ser = &s_data->serializer;

	// send request
	ret = send_request(ser, CMD_METHOD_SUB_HEARTBEAT, (void *)cb2, (void *)ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send fail %d.\n", __func__, ret);
		return ret;
	}

	// set registry
	s_data->heartbeat_registry.busy = true;
	(void)add_registry(&s_data->heartbeat_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(test_heartbeat_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;

	if (!s_data)
		return -ERR_APP_PARAM;

	ser = &s_data->serializer;

	// send request
	ret = send_request(ser, CMD_METHOD_UNSUB_HEARTBEAT, (void *)cb, (void *)ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		printf("%s: send fail %d.\n", __func__, ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_heartbeat_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	callback_registration_t *reg = NULL;
	test_heartbeat_callback_t cb = NULL;
	uint8_t status = 0;


	if (!des || !s_data)
		return -ERR_APP_PARAM;

	reg = &s_data->heartbeat_registry;
	if (!reg->busy || !reg->cb) {
		printf("%s: heartbeat callback registry is invalid.\n", __func__);
		return RESULT_SUCCESS;
	}

	s_data->info.uuid = ipc_msg_get_uuid(des->header);
	s_data->info.timestamp = des->recv_end_time;
	buf = reg->ext_buf ? reg->ext_buf : &s_data->des_buf;
	clear_des_buf(buf);
	if (ret >= 0)
		ret = ipc_des_get(des, (uint8_t *)&status, sizeof(uint8_t));

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (test_heartbeat_callback_t)(reg->cb);
	cb(status, reg->ext, &s_data->info);

	return RESULT_SUCCESS;
}

// receive messages
static int32_t receive_message(void)
{
	int32_t ret = 0;

	if (!s_data)
		return -ERR_APP_PARAM;

	ret = ipc_trans_layer_query_info(PID, s_data->handle);

	// check if availability changed
	if (s_data->avail_changed_cb) {
		if (ret == QUERY_INFO_DST_STS_OFFLINE)
			s_data->avail_changed_cb(false, s_data->avail_ext);
		else if (ret == QUERY_INFO_DST_STS_ONLINE)
			s_data->avail_changed_cb(true, s_data->avail_ext);
	}

	return ret;
}

// dispatch messages
static int32_t dispatch_message(void)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	serdes_t *des = NULL;
	bool has_message = false;

	if (!s_data)
		return -ERR_APP_PARAM;
	ser = &s_data->serializer;
	des = &s_data->deserializer;
	while (true) {
		has_message = false;
		if (ipc_trans_layer_proxy_get_broadcast_msg(PID, s_data->handle, des) >= 0) {
			has_message = true;
			switch (des->header.cmd) {
			case CMD_BROADCAST_HEARTBEAT:
				ret = call_heartbeat_callback(des);
				break;
			default:
				break;
			}
			(void)ipc_des_init(des);
		}
		if (ipc_trans_layer_proxy_get_reply_msg(PID, s_data->handle, des) >= 0) {
			has_message = true;
			switch (des->header.cmd) {
			case CMD_METHOD_HELLO:
				ret = call_hello_callback(des);
				break;
			case CMD_METHOD_COMPLEX_METHOD:
				ret = call_complex_method_callback(des);
				break;
			case CMD_METHOD_SUB_HEARTBEAT:
			{
				int32_t err = 0;

				ret = ipc_des_get(des, (uint8_t *)&err, sizeof(err));
				if (ret >= 0) {
					callback_registration_t *reg = &s_data->method_registry[des->header.tok];
					test_heartbeat_sub_callback_t cb = (test_heartbeat_sub_callback_t)(reg->cb);

					if (cb)
						cb(err, reg->ext, &s_data->info);
					clear_registry(reg);
				}
				break;
			}
			case CMD_METHOD_UNSUB_HEARTBEAT:
			{
				int32_t err = 0;

				ret = ipc_des_get(des, (uint8_t *)&err, sizeof(err));
				if (ret >= 0) {
					callback_registration_t *reg = &s_data->method_registry[des->header.tok];
					test_heartbeat_unsub_callback_t cb = (test_heartbeat_unsub_callback_t)(reg->cb);

					if (cb)
						cb(err, reg->ext, &s_data->info);
					clear_registry(reg);
					clear_registry(&s_data->heartbeat_registry);
				}
				break;
			}
			default:
				break;
			}
			(void)ipc_des_init(des);
			if (ret < 0)
				printf("%s: deserialization failed.\n", __func__);
		}
		if (!has_message)
			break;
	}
	return ret;
}

static int router_func(void *arg)
{
	int32_t ret = 0;

	while (unlikely(!kthread_should_stop())) {
		ret = receive_message();
		if (ret < 0)
			continue;

		ret = dispatch_message();
		if (ret < 0)
			continue;
	}

	return RESULT_SUCCESS;
}

// start message router
static int32_t start(void)
{
	if (s_data->bRunning)
		return RESULT_SUCCESS;

	s_data->route_task = kthread_run(router_func, NULL, "test_client_thread");
	if (unlikely(!s_data->route_task))
		return -ERR_APP_START;

	s_data->bRunning = true;

	return RESULT_SUCCESS;
}

// stop message router.
static int32_t stop(void)
{
	if (!s_data->bRunning)
		return RESULT_SUCCESS;

	//sleep 1 seconds.
	msleep(1000);
	if (likely(s_data->route_task)) {
		int32_t ret = 0;

		ipc_trans_layer_release_recv_wait(PID, s_data->handle);
		ret = kthread_stop(s_data->route_task);
		if (unlikely(ret))
			return -ERR_APP_STOP;
	}
	s_data->bRunning = false;

	return RESULT_SUCCESS;
}

// register availablity changed callback function
static int32_t register_avail_changed_cb(avail_changed_callback_t cb, void *ext)
{
	if (!s_data)
		return -ERR_APP_PARAM;

	s_data->avail_changed_cb = cb;
	s_data->avail_ext = ext;
	return 0;
}

// initialize client
test_client_t *test_client_init(void)
{
	int32_t ret = 0;

	if (s_data && s_data->initialized) {
		printf("%s: already initialized.\n", __func__);
		return &s_data->client;
	}

	s_data = &s_internal_data;
	ipc_memset(s_data, 0, sizeof(test_client_data_t));

	// create client handle.
	ret = ipc_trans_layer_proxy_create_handle(PID, FID, SID, CID,
				&s_data->handle);
	if (ret < 0)
		return NULL;

	// set version
	s_data->version.major = 1;
	s_data->version.minor = 0;

	// set client
	s_data->client.version = get_ipc_inf_version;
	s_data->client.register_avail_changed = register_avail_changed_cb;
	s_data->client.hello_sync = call_hello_sync;
	s_data->client.hello_async = call_hello_async;
	s_data->client.complex_method_sync = call_complex_method_sync;
	s_data->client.complex_method_async = call_complex_method_async;
	s_data->client.no_reply_method_fire_and_forget = call_no_reply_method_fire_and_forget;

	s_data->client.heartbeat_sub = subscribe_heartbeat;
	s_data->client.heartbeat_unsub = unsubscribe_heartbeat;
	init_registry(&s_data->heartbeat_registry);
	s_data->client.start = start;
	s_data->client.stop = stop;

	mutex_init(&s_data->send_mtx);
	init_registry_list(s_data->method_registry, IPC_TOKEN_NUM);
	s_data->initialized = true;
	return &s_data->client;
}

// destroy client
int32_t test_client_destroy(void)
{
	int32_t ret = 0;

	// if s_data is NULL, just return SUCCESS.
	if (!s_data)
		return RESULT_SUCCESS;

	ret = ipc_trans_layer_destroy_handle(PID, s_data->handle);
	if (ret < 0)
		return ret;


	destroy_registry(&s_data->heartbeat_registry);
	destroy_registry_list(s_data->method_registry, IPC_TOKEN_NUM);
	ipc_memset(s_data, 0, sizeof(test_client_data_t));
	s_data = NULL;

	return ret;
}
