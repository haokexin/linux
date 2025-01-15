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
 * Generator Version: francaidl 77a2400 msgbx_ipc eb42a92
 */

#include "test_client.h"

// macro definitions
#define CID CPU_7
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_COMPLEX_METHOD 2U
#define CMD_METHOD_NO_REPLY_METHOD 3U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static com_client_data_t *s_data;
static test_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

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

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

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
#ifndef IPC_RTE_BAREMETAL

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

static int32_t call_hello_sync(const char *name,
				char **message,
				test_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	hello_out_t out = {.message = message,
.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_HELLO,
				hello_sync_callback, &out, ext_buf);
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

static int32_t call_hello_async(const char *name,
				test_hello_callback_t cb,
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

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_HELLO,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hello_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	char *message = NULL;
	test_ErrorEnum_t err = { 0 };


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
		ret = deserialize_test_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == TEST_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(buf, &message);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);

		if (cb)
			cb(message, err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
		notify_callback_registry(reg);
#endif
		clear_registry(reg);
	}
	return RESULT_SUCCESS;
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
		ret = ipc_ser_put_32(ser, (uint32_t *)&in1);
	if (ret >= 0)
		ret = serialize_string(ser, in2);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in3);
	if (ret >= 0)
		ret = serialize_test_MyArray(ser, &in4);
	if (ret >= 0)
		ret = serialize_test_MyStruct(ser, &in5);
	if (ret >= 0)
		ret = serialize_test_MyUnion(ser, &in6);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

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
	complex_method_out_t out = {.out1 = out1,
.out2 = out2,
.out3 = out3,
.out4 = out4,
.out5 = out5,
.out6 = out6,
.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_complex_method(ser, in1, in2, in3, in4, in5, in6);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_COMPLEX_METHOD,
				complex_method_sync_callback, &out, ext_buf);
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

	ret = serialize_complex_method(ser, in1, in2, in3, in4, in5, in6);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_COMPLEX_METHOD,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_complex_method_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	uint32_t out1 = 0;
	char *out2 = NULL;
	byte_buffer_t out3 = { 0 };
	test_MyArray_t out4 = { 0 };
	test_MyStruct_t out5 = { 0 };
	test_MyUnion_t out6 = { 0 };
	test_ErrorEnum_t err = { 0 };


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
		ret = deserialize_test_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == TEST_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&out1);
		if (ret >= 0)
			ret = deserialize_string(buf, &out2);
		if (ret >= 0)
			ret = deserialize_byte_buffer(buf, &out3);
		if (ret >= 0)
			ret = deserialize_test_MyArray(buf, &out4);
		if (ret >= 0)
			ret = deserialize_test_MyStruct(buf, &out5);
		if (ret >= 0)
			ret = deserialize_test_MyUnion(buf, &out6);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	if (reg->busy) {
		test_complex_method_callback_t cb = (test_complex_method_callback_t)(reg->cb);

		if (cb)
			cb(out1, out2, out3, out4, out5, out6, err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
		notify_callback_registry(reg);
#endif
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
		ret = ipc_ser_put_8(ser, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_no_reply_method_fire_and_forget(const uint8_t status)
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

	ret = serialize_no_reply_method(ser, status);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NO_REPLY_METHOD);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_HEARTBEAT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->heartbeat_registry.busy = true;
	(void)add_registry(&s_ext->heartbeat_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_HEARTBEAT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_heartbeat_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	test_heartbeat_callback_t cb = NULL;
	uint8_t status = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->heartbeat_registry;
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
		ret = deserialize_8(buf, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (test_heartbeat_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(serdes_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_HEARTBEAT:
		ret = call_heartbeat_callback(des);
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
	case CMD_METHOD_HELLO:
		ret = call_hello_callback(des);
		break;
	case CMD_METHOD_COMPLEX_METHOD:
		ret = call_complex_method_callback(des);
		break;
	case CMD_METHOD_SUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->heartbeat_registry);
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
int32_t test_client_init(com_client_data_t *data, test_client_t *client,
			test_client_ext_t *ext)
{
	int32_t ret = 0;

	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->hello_sync = call_hello_sync;
#endif
	client->hello_async = call_hello_async;
#ifndef IPC_RTE_BAREMETAL
	client->complex_method_sync = call_complex_method_sync;
#endif
	client->complex_method_async = call_complex_method_async;
	client->no_reply_method_fire_and_forget = call_no_reply_method_fire_and_forget;

	client->heartbeat_sub = subscribe_heartbeat;
	client->heartbeat_unsub = unsubscribe_heartbeat;
	init_registry(&ext->heartbeat_registry);
	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	if (ext->cid == 0)
		ext->cid = CID;

	return 0;
}
// destroy client
void test_client_destroy(void)
{
	destroy_registry(&s_ext->heartbeat_registry);
	s_data = NULL;
	s_ext = NULL;
}
