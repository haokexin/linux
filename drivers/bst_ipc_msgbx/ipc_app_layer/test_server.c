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

#include "test_server.h"
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>

// macro definitions
#define MAJOR 1U
#define MINOR 0U

#define MAX_METHOD_NUM 10U
#define MAX_BROADCAST_NUM 10U

#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_COMPLEX_METHOD 2U
#define CMD_METHOD_NO_REPLY_METHOD 3U

#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static com_server_data_t *s_data;
static test_server_ext_t *s_ext;

// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static int32_t register_hello(test_hello_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->hello_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t call_hello(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_server_data_t *data = s_data;
	char *name = NULL;


	if (!des || !data || !s_ext || !s_ext->hello_ptr)
		return -ERR_APP_PARAM;

	buf = &data->des_buf;
	clear_des_buf(buf);
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

	if (ret >= 0)
		ret = deserialize_string(buf, &name);

	if (ret < 0)
		return -ERR_APP_SERDES;

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	(*s_ext->hello_ptr)(name, *(uint64_t *)&des->header, &data->info);

	return RESULT_SUCCESS;
}

static int32_t reply_hello(
				const char *message,
				const test_ErrorEnum_t err,
				const uint64_t context)
{
	int32_t ret = 0;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_server_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	ret = ipc_ser_init(ser);
	if (ret >= 0)
		ret = serialize_test_ErrorEnum(ser, &err);
	if (ret >= 0)
		ret = serialize_string(ser, message);

	if (ret < 0) {
		int32_t _err = -1;
		(void)ipc_ser_init(ser);
		ret = ipc_ser_put_32(ser, (uint32_t *)&_err);
		IPC_LOG_ERR("serialization failed.\n");
	}

	if (ret >= 0) {
		ser->header = *(rw_msg_header_t *)&context;
		ser->header.cid = ser->header.pid;
		ser->header.pid = data->pid;
		ser->header.typ = IPC_MSG_TYPE_REPLY;
		ret = ipc_ser_finish(ser);
	}

	if (ret >= 0)
		ret = ipc_trans_layer_stub_send_reply_msg(data->pid, data->handle, ser);

	if (ret < 0) {
		IPC_LOG_ERR("send reply fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static int32_t register_complex_method(test_complex_method_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->complex_method_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t call_complex_method(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_server_data_t *data = s_data;
	uint32_t in1 = 0;
	char *in2 = NULL;
	byte_buffer_t in3 = { 0 };
	test_MyArray_t in4 = { 0 };
	test_MyStruct_t in5 = { 0 };
	test_MyUnion_t in6 = { 0 };


	if (!des || !data || !s_ext || !s_ext->complex_method_ptr)
		return -ERR_APP_PARAM;

	buf = &data->des_buf;
	clear_des_buf(buf);
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&in1);
	if (ret >= 0)
		ret = deserialize_string(buf, &in2);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &in3);
	if (ret >= 0)
		ret = deserialize_test_MyArray(buf, &in4);
	if (ret >= 0)
		ret = deserialize_test_MyStruct(buf, &in5);
	if (ret >= 0)
		ret = deserialize_test_MyUnion(buf, &in6);

	if (ret < 0)
		return -ERR_APP_SERDES;

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	(*s_ext->complex_method_ptr)(in1, in2, in3, in4, in5, in6, *(uint64_t *)&des->header, &data->info);

	return RESULT_SUCCESS;
}

static int32_t reply_complex_method(
				const uint32_t out1,
				const char *out2,
				const byte_buffer_t out3,
				const test_MyArray_t out4,
				const test_MyStruct_t out5,
				const test_MyUnion_t out6,
				const test_ErrorEnum_t err,
				const uint64_t context)
{
	int32_t ret = 0;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_server_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	ret = ipc_ser_init(ser);
	if (ret >= 0)
		ret = serialize_test_ErrorEnum(ser, &err);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&out1);
	if (ret >= 0)
		ret = serialize_string(ser, out2);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &out3);
	if (ret >= 0)
		ret = serialize_test_MyArray(ser, &out4);
	if (ret >= 0)
		ret = serialize_test_MyStruct(ser, &out5);
	if (ret >= 0)
		ret = serialize_test_MyUnion(ser, &out6);

	if (ret < 0) {
		int32_t _err = -1;
		(void)ipc_ser_init(ser);
		ret = ipc_ser_put_32(ser, (uint32_t *)&_err);
		IPC_LOG_ERR("serialization failed.\n");
	}

	if (ret >= 0) {
		ser->header = *(rw_msg_header_t *)&context;
		ser->header.cid = ser->header.pid;
		ser->header.pid = data->pid;
		ser->header.typ = IPC_MSG_TYPE_REPLY;
		ret = ipc_ser_finish(ser);
	}

	if (ret >= 0)
		ret = ipc_trans_layer_stub_send_reply_msg(data->pid, data->handle, ser);

	if (ret < 0) {
		IPC_LOG_ERR("send reply fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static int32_t register_no_reply_method(test_no_reply_method_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->no_reply_method_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t call_no_reply_method(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_server_data_t *data = s_data;
	uint8_t status = 0;


	if (!des || !data || !s_ext || !s_ext->no_reply_method_ptr)
		return -ERR_APP_PARAM;

	buf = &data->des_buf;
	clear_des_buf(buf);
	len = ipc_des_get_all(des, (uint8_t *)buf->data_buf);
	if (len <= 0)
		return -ERR_APP_SERDES;
	buf->unavail_data_size = IPC_MAX_DATA_SIZE - len;

	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->recv_end_time;

	(*s_ext->no_reply_method_ptr)(status, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

static int32_t register_heartbeat_subcribed(broadcast_sub_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->heartbeat_sub_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t register_heartbeat_unsubcribed(broadcast_sub_t func)
{
	if (!s_ext)
		return -ERR_APP_PARAM;
	s_ext->heartbeat_unsub_ptr = func;
	return RESULT_SUCCESS;
}

static int32_t heartbeat(uint8_t status)
{
	int32_t ret = 0;
	int32_t send_ret = 0;
	int32_t index = 0;
	rw_msg_header_t header = { 0 };
	broadcast_reg_entry_t *entry = NULL;
	broadcast_registry_t *reg = NULL;
#ifdef IPC_RTE_BAREMETAL
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_server_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_RTE_BAREMETAL
	ser = &data->serializer;
#endif
	header.pid = data->pid;
	header.cmd = CMD_BROADCAST_HEARTBEAT;
	header.typ = IPC_MSG_TYPE_BROADCAST;
	ret = ipc_ser_init(ser);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	reg = &s_ext->heartbeat_registry;
	entry = reg->entries + reg->start;
	for (index = reg->start; index < reg->end; ++index, ++entry) {
		if (entry->pid != 0) {
			header.cid = entry->pid;
			header.fid = entry->fid;
			header.sid = entry->sid;
			header.tok = data->token;
			ipc_ser_set_header(ser, header);
			ipc_ser_finish(ser);
			send_ret = ipc_trans_layer_stub_send_broadcast(data->pid, data->handle, ser);
			if (send_ret < 0)
				IPC_LOG_ERR("send broadcast fail %d.\n", send_ret);
			else
				++ret;
		}
	}
	increase_token(data);
	return ret;
}

// dispatch_request
static int32_t dispatch_request(serdes_t *des, bool *reply)
{
	int32_t ret = 0;

	if (!des)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_HELLO:
		ret = call_hello(des);
		if (ret >= 0)
			*reply = false;
		else
			*reply = true;
		return ret;
	case CMD_METHOD_COMPLEX_METHOD:
		ret = call_complex_method(des);
		if (ret >= 0)
			*reply = false;
		else
			*reply = true;
		return ret;
	case CMD_METHOD_NO_REPLY_METHOD:
		ret = call_no_reply_method(des);
		*reply = false;
		return ret;
	case CMD_METHOD_SUB_HEARTBEAT:
		if (s_ext->heartbeat_sub_ptr)
			ret = (*s_ext->heartbeat_sub_ptr)((uint8_t)des->header.pid,
					(uint8_t)des->header.fid, (uint8_t)des->header.sid);
		if (ret >= 0)
			ret = add_registration(&s_ext->heartbeat_registry,
					(uint8_t)des->header.pid, (uint8_t)des->header.fid,
					(uint8_t)des->header.sid);
		*reply = true;
		return ret;
	case CMD_METHOD_UNSUB_HEARTBEAT:
		if (s_ext->heartbeat_unsub_ptr)
			ret = (*s_ext->heartbeat_unsub_ptr)((uint8_t)des->header.pid,
					(uint8_t)des->header.fid, (uint8_t)des->header.sid);
		if (ret >= 0)
			ret = remove_registration(&s_ext->heartbeat_registry,
					(uint8_t)des->header.pid, (uint8_t)des->header.fid,
					(uint8_t)des->header.sid);
		*reply = true;
		return ret;
	default:
		break;
	}
	ret = -ERR_APP_UNKNOWN_CMD;
	*reply = false;
	return ret;
}

// initialize server
int32_t test_server_init(com_server_data_t *data, test_server_t *server,
			test_server_ext_t *ext)
{
	int32_t ret = 0;

	if (!data || !server || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// register CMDs.
	ret = ipc_trans_layer_register_method(data->pid, data->handle, CMD_METHOD_HELLO);
	if (ret < 0)
		return -1;
	ret = ipc_trans_layer_register_method(data->pid, data->handle, CMD_METHOD_COMPLEX_METHOD);
	if (ret < 0)
		return -1;
	ret = ipc_trans_layer_register_method(data->pid, data->handle, CMD_METHOD_NO_REPLY_METHOD);
	if (ret < 0)
		return -1;
	ret = ipc_trans_layer_register_method(data->pid, data->handle, CMD_METHOD_SUB_HEARTBEAT);
	if (ret < 0)
		return -1;
	ret = ipc_trans_layer_register_method(data->pid, data->handle, CMD_METHOD_UNSUB_HEARTBEAT);
	if (ret < 0)
		return -1;

	// set server
	server->version = get_ipc_inf_version;
	ext->hello_ptr = NULL;
	server->register_hello = register_hello;
	server->reply_hello = reply_hello;
	ext->complex_method_ptr = NULL;
	server->register_complex_method = register_complex_method;
	server->reply_complex_method = reply_complex_method;
	ext->no_reply_method_ptr = NULL;
	server->register_no_reply_method = register_no_reply_method;

	server->heartbeat = heartbeat;
	server->register_heartbeat_subcribed = register_heartbeat_subcribed;
	server->register_heartbeat_unsubcribed = register_heartbeat_unsubcribed;

	server->dispatch_request = dispatch_request;

	return 0;
}

// destroy client
void test_server_destroy(void)
{
	s_data = NULL;
	s_ext = NULL;
}
