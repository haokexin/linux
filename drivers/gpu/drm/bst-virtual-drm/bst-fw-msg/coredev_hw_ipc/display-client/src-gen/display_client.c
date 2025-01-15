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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 8957426 msgbx_ipc 1964fef
 */

#include "display_client.h"

// macro definitions
#define CID SAFETY_0
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_FW_MSG_SEND 8U

#define CMD_METHOD_SUB_DC_PIPE0_EVENT 10U
#define CMD_METHOD_UNSUB_DC_PIPE0_EVENT 11U
#define CMD_BROADCAST_DC_PIPE0_EVENT 0U
#define CMD_METHOD_SUB_DC_PIPE1_EVENT 12U
#define CMD_METHOD_UNSUB_DC_PIPE1_EVENT 13U
#define CMD_BROADCAST_DC_PIPE1_EVENT 1U
#define CMD_METHOD_SUB_DC_PIPE2_EVENT 14U
#define CMD_METHOD_UNSUB_DC_PIPE2_EVENT 15U
#define CMD_BROADCAST_DC_PIPE2_EVENT 2U
#define CMD_METHOD_SUB_DC_PIPE3_EVENT 16U
#define CMD_METHOD_UNSUB_DC_PIPE3_EVENT 17U
#define CMD_BROADCAST_DC_PIPE3_EVENT 3U
#define CMD_METHOD_SUB_DC_PIPE4_EVENT 18U
#define CMD_METHOD_UNSUB_DC_PIPE4_EVENT 19U
#define CMD_BROADCAST_DC_PIPE4_EVENT 4U
#define CMD_METHOD_SUB_EDP_EVENT 20U
#define CMD_METHOD_UNSUB_EDP_EVENT 21U
#define CMD_BROADCAST_EDP_EVENT 5U
#define CMD_METHOD_SUB_DSI0_EVENT 22U
#define CMD_METHOD_UNSUB_DSI0_EVENT 23U
#define CMD_BROADCAST_DSI0_EVENT 6U
#define CMD_METHOD_SUB_DSI1_EVENT 24U
#define CMD_METHOD_UNSUB_DSI1_EVENT 25U
#define CMD_BROADCAST_DSI1_EVENT 7U
#define CMD_METHOD_SUB_LVDS0_EVENT 26U
#define CMD_METHOD_UNSUB_LVDS0_EVENT 27U
#define CMD_BROADCAST_LVDS0_EVENT 8U
#define CMD_METHOD_SUB_LVDS1_EVENT 28U
#define CMD_METHOD_UNSUB_LVDS1_EVENT 29U
#define CMD_BROADCAST_LVDS1_EVENT 9U

// local variables
static com_client_data_t *s_data;
static display_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _fw_msg_send_out_t {
	display_Array_Uint32_t *user_ack_data;
	display_ErrorEnum_t *err;
};
#define fw_msg_send_out_t struct _fw_msg_send_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_fw_msg_send(
				serdes_t *ser,
				const display_Array_Uint32_t user_cmd_data,
				const display_bst_display_cmd_head_t *fw_cmd_msg
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_display_Array_Uint32(ser, &user_cmd_data);
	if (ret >= 0)
		ret = serialize_display_bst_display_cmd_head(ser, fw_cmd_msg);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void fw_msg_send_sync_callback(
				const display_Array_Uint32_t user_ack_data,
				const display_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	fw_msg_send_out_t *out = (fw_msg_send_out_t *)ext;

	if (!out)
		return;
	out->user_ack_data->size = user_ack_data.size;
	out->user_ack_data->data = user_ack_data.data;
	*out->err = err;
}

static int32_t call_fw_msg_send_sync(const display_Array_Uint32_t user_cmd_data,
				const display_bst_display_cmd_head_t *fw_cmd_msg,
				display_Array_Uint32_t *user_ack_data,
				display_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	fw_msg_send_out_t out = {.user_ack_data = user_ack_data,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_fw_msg_send(ser, user_cmd_data, fw_cmd_msg);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_FW_MSG_SEND,
				fw_msg_send_sync_callback, &out, ext_buf);
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

static int32_t call_fw_msg_send_async(const display_Array_Uint32_t user_cmd_data,
				const display_bst_display_cmd_head_t *fw_cmd_msg,
				display_fw_msg_send_callback_t cb,
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

	ret = serialize_fw_msg_send(ser, user_cmd_data, fw_cmd_msg);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_FW_MSG_SEND,
				cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_fw_msg_send_callback(serdes_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	display_fw_msg_send_callback_t cb = NULL;
	display_Array_Uint32_t user_ack_data = { 0 };
	display_ErrorEnum_t err = 0;


	if (!des || !data)
		return -ERR_APP_PARAM;

	reg = &data->method_registry[des->header.tok];
	if (!reg->busy || reg->cmd != CMD_METHOD_FW_MSG_SEND) {
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
		ret = deserialize_display_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == DISPLAY_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_display_Array_Uint32(buf, &user_ack_data);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (display_fw_msg_send_callback_t)(reg->cb);
	if (cb)
		cb(user_ack_data, err, reg->ext, &data->info);
#ifndef IPC_RTE_BAREMETAL
	notify_callback_registry(reg);
#endif
	clear_registry(reg);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe dc_pipe0_event
static int32_t subscribe_dc_pipe0_event(
				display_dc_pipe0_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DC_PIPE0_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dc_pipe0_event_registry.busy = true;
	(void)add_registry(&s_ext->dc_pipe0_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dc_pipe0_event
static int32_t unsubscribe_dc_pipe0_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DC_PIPE0_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dc_pipe0_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dc_pipe0_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dc_pipe0_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dc_pipe0_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dc_pipe1_event
static int32_t subscribe_dc_pipe1_event(
				display_dc_pipe1_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DC_PIPE1_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dc_pipe1_event_registry.busy = true;
	(void)add_registry(&s_ext->dc_pipe1_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dc_pipe1_event
static int32_t unsubscribe_dc_pipe1_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DC_PIPE1_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dc_pipe1_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dc_pipe1_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dc_pipe1_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dc_pipe1_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dc_pipe2_event
static int32_t subscribe_dc_pipe2_event(
				display_dc_pipe2_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DC_PIPE2_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dc_pipe2_event_registry.busy = true;
	(void)add_registry(&s_ext->dc_pipe2_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dc_pipe2_event
static int32_t unsubscribe_dc_pipe2_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DC_PIPE2_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dc_pipe2_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dc_pipe2_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dc_pipe2_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dc_pipe2_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dc_pipe3_event
static int32_t subscribe_dc_pipe3_event(
				display_dc_pipe3_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DC_PIPE3_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dc_pipe3_event_registry.busy = true;
	(void)add_registry(&s_ext->dc_pipe3_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dc_pipe3_event
static int32_t unsubscribe_dc_pipe3_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DC_PIPE3_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dc_pipe3_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dc_pipe3_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dc_pipe3_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dc_pipe3_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dc_pipe4_event
static int32_t subscribe_dc_pipe4_event(
				display_dc_pipe4_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DC_PIPE4_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dc_pipe4_event_registry.busy = true;
	(void)add_registry(&s_ext->dc_pipe4_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dc_pipe4_event
static int32_t unsubscribe_dc_pipe4_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DC_PIPE4_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dc_pipe4_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dc_pipe4_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dc_pipe4_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dc_pipe4_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe edp_event
static int32_t subscribe_edp_event(
				display_edp_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_EDP_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->edp_event_registry.busy = true;
	(void)add_registry(&s_ext->edp_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe edp_event
static int32_t unsubscribe_edp_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_EDP_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_edp_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_edp_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->edp_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_edp_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dsi0_event
static int32_t subscribe_dsi0_event(
				display_dsi0_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DSI0_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dsi0_event_registry.busy = true;
	(void)add_registry(&s_ext->dsi0_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dsi0_event
static int32_t unsubscribe_dsi0_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DSI0_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dsi0_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dsi0_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dsi0_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dsi0_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe dsi1_event
static int32_t subscribe_dsi1_event(
				display_dsi1_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_DSI1_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->dsi1_event_registry.busy = true;
	(void)add_registry(&s_ext->dsi1_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe dsi1_event
static int32_t unsubscribe_dsi1_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_DSI1_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_dsi1_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_dsi1_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->dsi1_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_dsi1_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe lvds0_event
static int32_t subscribe_lvds0_event(
				display_lvds0_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_LVDS0_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->lvds0_event_registry.busy = true;
	(void)add_registry(&s_ext->lvds0_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe lvds0_event
static int32_t unsubscribe_lvds0_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_LVDS0_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_lvds0_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_lvds0_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->lvds0_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_lvds0_event_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe lvds1_event
static int32_t subscribe_lvds1_event(
				display_lvds1_event_callback_t cb,
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
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_SUB_LVDS1_EVENT,
				cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	// set registry
	s_ext->lvds1_event_registry.busy = true;
	(void)add_registry(&s_ext->lvds1_event_registry, (void *)cb, ext, ext_buf);

	return RESULT_SUCCESS;
}

// unsubscribe lvds1_event
static int32_t unsubscribe_lvds1_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, ser, s_ext->cid, CMD_METHOD_UNSUB_LVDS1_EVENT,
				cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %d.\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_lvds1_event_callback(serdes_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	uint32_t len = 0;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	display_lvds1_event_callback_t cb = NULL;
	display_event_status_t *status = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->lvds1_event_registry;
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
		ret = deserialize_display_event_status(buf, &status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (display_lvds1_event_callback_t)(reg->cb);
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
	case CMD_BROADCAST_DC_PIPE0_EVENT:
		ret = call_dc_pipe0_event_callback(des);
		break;
	case CMD_BROADCAST_DC_PIPE1_EVENT:
		ret = call_dc_pipe1_event_callback(des);
		break;
	case CMD_BROADCAST_DC_PIPE2_EVENT:
		ret = call_dc_pipe2_event_callback(des);
		break;
	case CMD_BROADCAST_DC_PIPE3_EVENT:
		ret = call_dc_pipe3_event_callback(des);
		break;
	case CMD_BROADCAST_DC_PIPE4_EVENT:
		ret = call_dc_pipe4_event_callback(des);
		break;
	case CMD_BROADCAST_EDP_EVENT:
		ret = call_edp_event_callback(des);
		break;
	case CMD_BROADCAST_DSI0_EVENT:
		ret = call_dsi0_event_callback(des);
		break;
	case CMD_BROADCAST_DSI1_EVENT:
		ret = call_dsi1_event_callback(des);
		break;
	case CMD_BROADCAST_LVDS0_EVENT:
		ret = call_lvds0_event_callback(des);
		break;
	case CMD_BROADCAST_LVDS1_EVENT:
		ret = call_lvds1_event_callback(des);
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
	case CMD_METHOD_FW_MSG_SEND:
		ret = call_fw_msg_send_callback(des);
		break;
	case CMD_METHOD_SUB_DC_PIPE0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DC_PIPE0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dc_pipe0_event_registry);
		break;
	case CMD_METHOD_SUB_DC_PIPE1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DC_PIPE1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dc_pipe1_event_registry);
		break;
	case CMD_METHOD_SUB_DC_PIPE2_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DC_PIPE2_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dc_pipe2_event_registry);
		break;
	case CMD_METHOD_SUB_DC_PIPE3_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DC_PIPE3_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dc_pipe3_event_registry);
		break;
	case CMD_METHOD_SUB_DC_PIPE4_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DC_PIPE4_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dc_pipe4_event_registry);
		break;
	case CMD_METHOD_SUB_EDP_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_EDP_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->edp_event_registry);
		break;
	case CMD_METHOD_SUB_DSI0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DSI0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dsi0_event_registry);
		break;
	case CMD_METHOD_SUB_DSI1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DSI1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->dsi1_event_registry);
		break;
	case CMD_METHOD_SUB_LVDS0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_LVDS0_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->lvds0_event_registry);
		break;
	case CMD_METHOD_SUB_LVDS1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_LVDS1_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->lvds1_event_registry);
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
int32_t display_client_init(com_client_data_t *data, display_client_t *client,
			display_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->fw_msg_send_sync = call_fw_msg_send_sync;
#endif
	client->fw_msg_send_async = call_fw_msg_send_async;

	client->dc_pipe0_event_sub = subscribe_dc_pipe0_event;
	client->dc_pipe0_event_unsub = unsubscribe_dc_pipe0_event;
	(void)init_registry(&ext->dc_pipe0_event_registry);	client->dc_pipe1_event_sub = subscribe_dc_pipe1_event;
	client->dc_pipe1_event_unsub = unsubscribe_dc_pipe1_event;
	(void)init_registry(&ext->dc_pipe1_event_registry);	client->dc_pipe2_event_sub = subscribe_dc_pipe2_event;
	client->dc_pipe2_event_unsub = unsubscribe_dc_pipe2_event;
	(void)init_registry(&ext->dc_pipe2_event_registry);	client->dc_pipe3_event_sub = subscribe_dc_pipe3_event;
	client->dc_pipe3_event_unsub = unsubscribe_dc_pipe3_event;
	(void)init_registry(&ext->dc_pipe3_event_registry);	client->dc_pipe4_event_sub = subscribe_dc_pipe4_event;
	client->dc_pipe4_event_unsub = unsubscribe_dc_pipe4_event;
	(void)init_registry(&ext->dc_pipe4_event_registry);	client->edp_event_sub = subscribe_edp_event;
	client->edp_event_unsub = unsubscribe_edp_event;
	(void)init_registry(&ext->edp_event_registry);	client->dsi0_event_sub = subscribe_dsi0_event;
	client->dsi0_event_unsub = unsubscribe_dsi0_event;
	(void)init_registry(&ext->dsi0_event_registry);	client->dsi1_event_sub = subscribe_dsi1_event;
	client->dsi1_event_unsub = unsubscribe_dsi1_event;
	(void)init_registry(&ext->dsi1_event_registry);	client->lvds0_event_sub = subscribe_lvds0_event;
	client->lvds0_event_unsub = unsubscribe_lvds0_event;
	(void)init_registry(&ext->lvds0_event_registry);	client->lvds1_event_sub = subscribe_lvds1_event;
	client->lvds1_event_unsub = unsubscribe_lvds1_event;
	(void)init_registry(&ext->lvds1_event_registry);
	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	if (ext->cid == 0)
		ext->cid = CID;

	return 0;
}
// destroy client
void display_client_destroy(void)
{
	destroy_registry(&s_ext->dc_pipe0_event_registry);destroy_registry(&s_ext->dc_pipe1_event_registry);destroy_registry(&s_ext->dc_pipe2_event_registry);destroy_registry(&s_ext->dc_pipe3_event_registry);destroy_registry(&s_ext->dc_pipe4_event_registry);destroy_registry(&s_ext->edp_event_registry);destroy_registry(&s_ext->dsi0_event_registry);destroy_registry(&s_ext->dsi1_event_registry);destroy_registry(&s_ext->lvds0_event_registry);destroy_registry(&s_ext->lvds1_event_registry);
	s_data = NULL;
	s_ext = NULL;
}
