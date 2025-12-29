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

#include "audio_ipc_client.h"

// macro definitions
#define CID MEDIA_0
#define CCID 0
#define CID_MASK (0x1ULL << 34)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_SLT_METHOD 2U
#define CMD_METHOD_NO_REPLY_METHOD 3U
#define CMD_METHOD_XRP_SHMEM_ADDR_METHOD 4U
#define CMD_METHOD_SET_SCENARIO 5U
#define CMD_METHOD_GET_CUR_SCENARIO 6U
#define CMD_METHOD_AUDIO_CONFIG 7U
#define CMD_METHOD_SEND_DMA_BUFF_GLOBAL_FD 8U
#define CMD_METHOD_DATA_READY_IN_PINGPONG_BUFF 9U
#define CMD_METHOD_CTRL_CMD 10U
#define CMD_METHOD_START_STOP_TO_PLAY 11U

#define CMD_METHOD_SUB_HEARTBEAT 20U
#define CMD_METHOD_UNSUB_HEARTBEAT 21U
#define CMD_BROADCAST_HEARTBEAT 1U
#define CMD_METHOD_SUB_KWS_TRIGGERED 22U
#define CMD_METHOD_UNSUB_KWS_TRIGGERED 23U
#define CMD_BROADCAST_KWS_TRIGGERED 2U
#define CMD_METHOD_SUB_DATA_COMSUMED_EVENT 24U
#define CMD_METHOD_UNSUB_DATA_COMSUMED_EVENT 25U
#define CMD_BROADCAST_DATA_COMSUMED_EVENT 3U

// local variables
static com_client_data_t *s_data;
static audio_ipc_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _hello_out_t {
	DECL_SEM(sem)
	char **message;
	audio_ipc_ErrorEnum_t *err;
};
#define hello_out_t struct _hello_out_t

struct _slt_method_out_t {
	DECL_SEM(sem)
	uint32_t *reply_result;
	audio_ipc_ErrorEnum_t *err;
};
#define slt_method_out_t struct _slt_method_out_t

struct _xrp_shmem_addr_method_out_t {
	DECL_SEM(sem)
	audio_ipc_XrpDspCmd_t *out_cmd;
	audio_ipc_ErrorEnum_t *err;
};
#define xrp_shmem_addr_method_out_t struct _xrp_shmem_addr_method_out_t

struct _set_scenario_out_t {
	DECL_SEM(sem)
	audio_ipc_ErrorEnum_t *err;
};
#define set_scenario_out_t struct _set_scenario_out_t

struct _get_cur_scenario_out_t {
	DECL_SEM(sem)
	char **scenario_name;
	audio_ipc_ErrorEnum_t *err;
};
#define get_cur_scenario_out_t struct _get_cur_scenario_out_t

struct _audio_config_out_t {
	DECL_SEM(sem)
	uint8_t *result;
	audio_ipc_ErrorEnum_t *err;
};
#define audio_config_out_t struct _audio_config_out_t

struct _send_dma_buff_global_fd_out_t {
	DECL_SEM(sem)
	uint8_t *result;
	audio_ipc_ErrorEnum_t *err;
};
#define send_dma_buff_global_fd_out_t struct _send_dma_buff_global_fd_out_t

struct _data_ready_in_pingpong_buff_out_t {
	DECL_SEM(sem)
	uint8_t *result;
	audio_ipc_ErrorEnum_t *err;
};
#define data_ready_in_pingpong_buff_out_t struct _data_ready_in_pingpong_buff_out_t

struct _ctrl_cmd_out_t {
	DECL_SEM(sem)
	uint8_t *result;
	audio_ipc_ErrorEnum_t *err;
};
#define ctrl_cmd_out_t struct _ctrl_cmd_out_t

struct _start_stop_to_play_out_t {
	DECL_SEM(sem)
	uint8_t *result;
	audio_ipc_ErrorEnum_t *err;
};
#define start_stop_to_play_out_t struct _start_stop_to_play_out_t

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
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	hello_out_t *out = (hello_out_t *)ext;

	if (!out)
		return;
	*out->message = (char *)message;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_hello_sync(const char *name,
				char **message,
				audio_ipc_ErrorEnum_t *err,
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
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hello_registry, ser, s_ext->cid,
			CMD_METHOD_HELLO, hello_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->hello_registry[ret];
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

static int32_t call_hello_async(const char *name,
				audio_ipc_hello_callback_t cb,
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

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hello_registry, ser, s_ext->cid,
			CMD_METHOD_HELLO, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hello_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_hello_callback_t cb = NULL;
	char *message = NULL;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->hello_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(buf, &message);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_hello_callback_t)(reg->cb);
	if (cb)
		cb(message, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_slt_method(
				serdes_t *ser,
				const uint32_t bin_index
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&bin_index);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void slt_method_sync_callback(
				const uint32_t reply_result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	slt_method_out_t *out = (slt_method_out_t *)ext;

	if (!out)
		return;
	*out->reply_result = reply_result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_slt_method_sync(const uint32_t bin_index,
				uint32_t *reply_result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	slt_method_out_t out = {.reply_result = reply_result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_slt_method(ser, bin_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->slt_method_registry, ser, s_ext->cid,
			CMD_METHOD_SLT_METHOD, slt_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->slt_method_registry[ret];
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

static int32_t call_slt_method_async(const uint32_t bin_index,
				audio_ipc_slt_method_callback_t cb,
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

	ret = serialize_slt_method(ser, bin_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->slt_method_registry, ser, s_ext->cid,
			CMD_METHOD_SLT_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_slt_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_slt_method_callback_t cb = NULL;
	uint32_t reply_result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->slt_method_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&reply_result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_slt_method_callback_t)(reg->cb);
	if (cb)
		cb(reply_result, err, reg->ext, &data->info);

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

	ret = serialize_no_reply_method(ser, status);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NO_REPLY_METHOD);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_xrp_shmem_addr_method(
				serdes_t *ser,
				const audio_ipc_XrpDspCmd_t in_cmd
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_audio_ipc_XrpDspCmd(ser, &in_cmd);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void xrp_shmem_addr_method_sync_callback(
				const audio_ipc_XrpDspCmd_t out_cmd,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	xrp_shmem_addr_method_out_t *out = (xrp_shmem_addr_method_out_t *)ext;

	if (!out)
		return;
	out->out_cmd->flags = out_cmd.flags;
	out->out_cmd->in_data_size = out_cmd.in_data_size;
	out->out_cmd->out_data_size = out_cmd.out_data_size;
	out->out_cmd->buffer_size = out_cmd.buffer_size;
	out->out_cmd->in_data_addr = out_cmd.in_data_addr;
	out->out_cmd->in_data.size = out_cmd.in_data.size;
	out->out_cmd->in_data.data = out_cmd.in_data.data;
	out->out_cmd->out_data_addr = out_cmd.out_data_addr;
	out->out_cmd->out_data.size = out_cmd.out_data.size;
	out->out_cmd->out_data.data = out_cmd.out_data.data;
	out->out_cmd->buffer_addr = out_cmd.buffer_addr;
	out->out_cmd->buffer_data.size = out_cmd.buffer_data.size;
	out->out_cmd->buffer_data.data = out_cmd.buffer_data.data;
	out->out_cmd->buffer_alignment.size = out_cmd.buffer_alignment.size;
	out->out_cmd->buffer_alignment.data = out_cmd.buffer_alignment.data;
	out->out_cmd->nsid.size = out_cmd.nsid.size;
	out->out_cmd->nsid.data = out_cmd.nsid.data;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_xrp_shmem_addr_method_sync(const audio_ipc_XrpDspCmd_t in_cmd,
				audio_ipc_XrpDspCmd_t *out_cmd,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	xrp_shmem_addr_method_out_t out = {.out_cmd = out_cmd,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_xrp_shmem_addr_method(ser, in_cmd);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->xrp_shmem_addr_method_registry, ser, s_ext->cid,
			CMD_METHOD_XRP_SHMEM_ADDR_METHOD, xrp_shmem_addr_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->xrp_shmem_addr_method_registry[ret];
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

static int32_t call_xrp_shmem_addr_method_async(const audio_ipc_XrpDspCmd_t in_cmd,
				audio_ipc_xrp_shmem_addr_method_callback_t cb,
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

	ret = serialize_xrp_shmem_addr_method(ser, in_cmd);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->xrp_shmem_addr_method_registry, ser, s_ext->cid,
			CMD_METHOD_XRP_SHMEM_ADDR_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_xrp_shmem_addr_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_xrp_shmem_addr_method_callback_t cb = NULL;
	audio_ipc_XrpDspCmd_t out_cmd = { 0 };
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->xrp_shmem_addr_method_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_audio_ipc_XrpDspCmd(buf, &out_cmd);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_xrp_shmem_addr_method_callback_t)(reg->cb);
	if (cb)
		cb(out_cmd, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_set_scenario(
				serdes_t *ser,
				const char *scenario_name
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_string(ser, scenario_name);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void set_scenario_sync_callback(
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	set_scenario_out_t *out = (set_scenario_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_set_scenario_sync(const char *scenario_name,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	set_scenario_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_set_scenario(ser, scenario_name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->set_scenario_registry, ser, s_ext->cid,
			CMD_METHOD_SET_SCENARIO, set_scenario_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->set_scenario_registry[ret];
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

static int32_t call_set_scenario_async(const char *scenario_name,
				audio_ipc_set_scenario_callback_t cb,
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

	ret = serialize_set_scenario(ser, scenario_name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->set_scenario_registry, ser, s_ext->cid,
			CMD_METHOD_SET_SCENARIO, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_set_scenario_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_set_scenario_callback_t cb = NULL;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->set_scenario_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (audio_ipc_set_scenario_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void get_cur_scenario_sync_callback(
				const char *scenario_name,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	get_cur_scenario_out_t *out = (get_cur_scenario_out_t *)ext;

	if (!out)
		return;
	*out->scenario_name = (char *)scenario_name;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_get_cur_scenario_sync(char **scenario_name,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	get_cur_scenario_out_t out = {.scenario_name = scenario_name,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	// send request
	ret = send_request(data, s_ext->get_cur_scenario_registry, ser, s_ext->cid,
			CMD_METHOD_GET_CUR_SCENARIO, get_cur_scenario_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->get_cur_scenario_registry[ret];
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

static int32_t call_get_cur_scenario_async(audio_ipc_get_cur_scenario_callback_t cb,
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

	// send request
	ret = send_request(data, s_ext->get_cur_scenario_registry, ser, s_ext->cid,
			CMD_METHOD_GET_CUR_SCENARIO, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_get_cur_scenario_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_get_cur_scenario_callback_t cb = NULL;
	char *scenario_name = NULL;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->get_cur_scenario_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(buf, &scenario_name);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_get_cur_scenario_callback_t)(reg->cb);
	if (cb)
		cb(scenario_name, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_audio_config(
				serdes_t *ser,
				const uint8_t channel,
				const uint32_t sample_rate,
				const uint8_t data_width,
				const uint8_t direction
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&channel);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&sample_rate);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&data_width);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&direction);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void audio_config_sync_callback(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	audio_config_out_t *out = (audio_config_out_t *)ext;

	if (!out)
		return;
	*out->result = result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_audio_config_sync(const uint8_t channel,
				const uint32_t sample_rate,
				const uint8_t data_width,
				const uint8_t direction,
				uint8_t *result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	audio_config_out_t out = {.result = result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_audio_config(ser, channel, sample_rate, data_width, direction);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->audio_config_registry, ser, s_ext->cid,
			CMD_METHOD_AUDIO_CONFIG, audio_config_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->audio_config_registry[ret];
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

static int32_t call_audio_config_async(const uint8_t channel,
				const uint32_t sample_rate,
				const uint8_t data_width,
				const uint8_t direction,
				audio_ipc_audio_config_callback_t cb,
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

	ret = serialize_audio_config(ser, channel, sample_rate, data_width, direction);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->audio_config_registry, ser, s_ext->cid,
			CMD_METHOD_AUDIO_CONFIG, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_audio_config_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_audio_config_callback_t cb = NULL;
	uint8_t result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->audio_config_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_audio_config_callback_t)(reg->cb);
	if (cb)
		cb(result, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_send_dma_buff_global_fd(
				serdes_t *ser,
				const int64_t gFd,
				const int64_t size
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_64(ser, (uint64_t *)&gFd);
	if (ret >= 0)
		ret = ipc_ser_put_64(ser, (uint64_t *)&size);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void send_dma_buff_global_fd_sync_callback(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	send_dma_buff_global_fd_out_t *out = (send_dma_buff_global_fd_out_t *)ext;

	if (!out)
		return;
	*out->result = result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_send_dma_buff_global_fd_sync(const int64_t gFd,
				const int64_t size,
				uint8_t *result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	send_dma_buff_global_fd_out_t out = {.result = result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_send_dma_buff_global_fd(ser, gFd, size);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->send_dma_buff_global_fd_registry, ser, s_ext->cid,
			CMD_METHOD_SEND_DMA_BUFF_GLOBAL_FD, send_dma_buff_global_fd_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->send_dma_buff_global_fd_registry[ret];
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

static int32_t call_send_dma_buff_global_fd_async(const int64_t gFd,
				const int64_t size,
				audio_ipc_send_dma_buff_global_fd_callback_t cb,
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

	ret = serialize_send_dma_buff_global_fd(ser, gFd, size);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->send_dma_buff_global_fd_registry, ser, s_ext->cid,
			CMD_METHOD_SEND_DMA_BUFF_GLOBAL_FD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_send_dma_buff_global_fd_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_send_dma_buff_global_fd_callback_t cb = NULL;
	uint8_t result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->send_dma_buff_global_fd_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_send_dma_buff_global_fd_callback_t)(reg->cb);
	if (cb)
		cb(result, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_data_ready_in_pingpong_buff(
				serdes_t *ser,
				const bool isPing
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&isPing);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void data_ready_in_pingpong_buff_sync_callback(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	data_ready_in_pingpong_buff_out_t *out = (data_ready_in_pingpong_buff_out_t *)ext;

	if (!out)
		return;
	*out->result = result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_data_ready_in_pingpong_buff_sync(const bool isPing,
				uint8_t *result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	data_ready_in_pingpong_buff_out_t out = {.result = result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_data_ready_in_pingpong_buff(ser, isPing);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->data_ready_in_pingpong_buff_registry, ser, s_ext->cid,
			CMD_METHOD_DATA_READY_IN_PINGPONG_BUFF, data_ready_in_pingpong_buff_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->data_ready_in_pingpong_buff_registry[ret];
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

static int32_t call_data_ready_in_pingpong_buff_async(const bool isPing,
				audio_ipc_data_ready_in_pingpong_buff_callback_t cb,
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

	ret = serialize_data_ready_in_pingpong_buff(ser, isPing);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->data_ready_in_pingpong_buff_registry, ser, s_ext->cid,
			CMD_METHOD_DATA_READY_IN_PINGPONG_BUFF, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_data_ready_in_pingpong_buff_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_data_ready_in_pingpong_buff_callback_t cb = NULL;
	uint8_t result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->data_ready_in_pingpong_buff_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_data_ready_in_pingpong_buff_callback_t)(reg->cb);
	if (cb)
		cb(result, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_ctrl_cmd(
				serdes_t *ser,
				const uint8_t channel,
				const uint8_t func_id,
				const byte_buffer_t in_data
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&channel);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&func_id);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in_data);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void ctrl_cmd_sync_callback(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	ctrl_cmd_out_t *out = (ctrl_cmd_out_t *)ext;

	if (!out)
		return;
	*out->result = result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_ctrl_cmd_sync(const uint8_t channel,
				const uint8_t func_id,
				const byte_buffer_t in_data,
				uint8_t *result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	ctrl_cmd_out_t out = {.result = result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_ctrl_cmd(ser, channel, func_id, in_data);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->ctrl_cmd_registry, ser, s_ext->cid,
			CMD_METHOD_CTRL_CMD, ctrl_cmd_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->ctrl_cmd_registry[ret];
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

static int32_t call_ctrl_cmd_async(const uint8_t channel,
				const uint8_t func_id,
				const byte_buffer_t in_data,
				audio_ipc_ctrl_cmd_callback_t cb,
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

	ret = serialize_ctrl_cmd(ser, channel, func_id, in_data);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->ctrl_cmd_registry, ser, s_ext->cid,
			CMD_METHOD_CTRL_CMD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_ctrl_cmd_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_ctrl_cmd_callback_t cb = NULL;
	uint8_t result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->ctrl_cmd_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_ctrl_cmd_callback_t)(reg->cb);
	if (cb)
		cb(result, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_start_stop_to_play(
				serdes_t *ser,
				const uint8_t chn,
				const uint8_t sts,
				const uint8_t direction
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&chn);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&sts);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&direction);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void start_stop_to_play_sync_callback(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	start_stop_to_play_out_t *out = (start_stop_to_play_out_t *)ext;

	if (!out)
		return;
	*out->result = result;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_start_stop_to_play_sync(const uint8_t chn,
				const uint8_t sts,
				const uint8_t direction,
				uint8_t *result,
				audio_ipc_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	start_stop_to_play_out_t out = {.result = result,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_start_stop_to_play(ser, chn, sts, direction);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->start_stop_to_play_registry, ser, s_ext->cid,
			CMD_METHOD_START_STOP_TO_PLAY, start_stop_to_play_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->start_stop_to_play_registry[ret];
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

static int32_t call_start_stop_to_play_async(const uint8_t chn,
				const uint8_t sts,
				const uint8_t direction,
				audio_ipc_start_stop_to_play_callback_t cb,
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

	ret = serialize_start_stop_to_play(ser, chn, sts, direction);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->start_stop_to_play_registry, ser, s_ext->cid,
			CMD_METHOD_START_STOP_TO_PLAY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_start_stop_to_play_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	audio_ipc_start_stop_to_play_callback_t cb = NULL;
	uint8_t result = 0;
	audio_ipc_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->start_stop_to_play_registry[des->header.tok];
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
		ret = deserialize_audio_ipc_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == AUDIO_IPC_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_8(buf, (uint8_t *)&result);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (audio_ipc_start_stop_to_play_callback_t)(reg->cb);
	if (cb)
		cb(result, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe heartbeat
static int32_t subscribe_heartbeat(
				audio_ipc_heartbeat_callback_t cb,
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
	s_ext->heartbeat_registry.busy = true;
	(void)set_registry(&s_ext->heartbeat_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_HEARTBEAT, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->heartbeat_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->heartbeat_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_HEARTBEAT, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_heartbeat_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	audio_ipc_heartbeat_callback_t cb = NULL;
	uint8_t status = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->heartbeat_registry;
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
		ret = deserialize_8(buf, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (audio_ipc_heartbeat_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe kws_triggered
static int32_t subscribe_kws_triggered(
				audio_ipc_kws_triggered_callback_t cb,
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
	s_ext->kws_triggered_registry.busy = true;
	(void)set_registry(&s_ext->kws_triggered_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_KWS_TRIGGERED, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->kws_triggered_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe kws_triggered
static int32_t unsubscribe_kws_triggered(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->kws_triggered_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_KWS_TRIGGERED, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_kws_triggered_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	audio_ipc_kws_triggered_callback_t cb = NULL;
	uint8_t kws_result = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->kws_triggered_registry;
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
		ret = deserialize_8(buf, (uint8_t *)&kws_result);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (audio_ipc_kws_triggered_callback_t)(reg->cb);
	cb(kws_result, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe data_comsumed_event
static int32_t subscribe_data_comsumed_event(
				audio_ipc_data_comsumed_event_callback_t cb,
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
	s_ext->data_comsumed_event_registry.busy = true;
	(void)set_registry(&s_ext->data_comsumed_event_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_DATA_COMSUMED_EVENT, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->data_comsumed_event_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe data_comsumed_event
static int32_t unsubscribe_data_comsumed_event(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->data_comsumed_event_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_DATA_COMSUMED_EVENT, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_data_comsumed_event_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	audio_ipc_data_comsumed_event_callback_t cb = NULL;
	uint32_t count = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->data_comsumed_event_registry;
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
		ret = deserialize_32(buf, (uint32_t *)&count);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (audio_ipc_data_comsumed_event_callback_t)(reg->cb);
	cb(count, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_HEARTBEAT:
		ret = call_heartbeat_callback(des);
		break;
	case CMD_BROADCAST_KWS_TRIGGERED:
		ret = call_kws_triggered_callback(des);
		break;
	case CMD_BROADCAST_DATA_COMSUMED_EVENT:
		ret = call_data_comsumed_event_callback(des);
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
	case CMD_METHOD_HELLO:
		ret = call_hello_callback(des);
		break;
	case CMD_METHOD_SLT_METHOD:
		ret = call_slt_method_callback(des);
		break;
	case CMD_METHOD_XRP_SHMEM_ADDR_METHOD:
		ret = call_xrp_shmem_addr_method_callback(des);
		break;
	case CMD_METHOD_SET_SCENARIO:
		ret = call_set_scenario_callback(des);
		break;
	case CMD_METHOD_GET_CUR_SCENARIO:
		ret = call_get_cur_scenario_callback(des);
		break;
	case CMD_METHOD_AUDIO_CONFIG:
		ret = call_audio_config_callback(des);
		break;
	case CMD_METHOD_SEND_DMA_BUFF_GLOBAL_FD:
		ret = call_send_dma_buff_global_fd_callback(des);
		break;
	case CMD_METHOD_DATA_READY_IN_PINGPONG_BUFF:
		ret = call_data_ready_in_pingpong_buff_callback(des);
		break;
	case CMD_METHOD_CTRL_CMD:
		ret = call_ctrl_cmd_callback(des);
		break;
	case CMD_METHOD_START_STOP_TO_PLAY:
		ret = call_start_stop_to_play_callback(des);
		break;
	case CMD_METHOD_SUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->heartbeat_registry);
		break;
	case CMD_METHOD_SUB_KWS_TRIGGERED:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_KWS_TRIGGERED:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->kws_triggered_registry);
		break;
	case CMD_METHOD_SUB_DATA_COMSUMED_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_DATA_COMSUMED_EVENT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->data_comsumed_event_registry);
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
int32_t audio_ipc_client_init(com_client_data_t *data, audio_ipc_client_t *client,
			audio_ipc_client_ext_t *ext)
{
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
(void)init_registry(ext->hello_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->slt_method_sync = call_slt_method_sync;
#endif
	client->slt_method_async = call_slt_method_async;
(void)init_registry(ext->slt_method_registry);
		client->no_reply_method_fire_and_forget = call_no_reply_method_fire_and_forget;
#ifndef IPC_RTE_BAREMETAL
	client->xrp_shmem_addr_method_sync = call_xrp_shmem_addr_method_sync;
#endif
	client->xrp_shmem_addr_method_async = call_xrp_shmem_addr_method_async;
(void)init_registry(ext->xrp_shmem_addr_method_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->set_scenario_sync = call_set_scenario_sync;
#endif
	client->set_scenario_async = call_set_scenario_async;
(void)init_registry(ext->set_scenario_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->get_cur_scenario_sync = call_get_cur_scenario_sync;
#endif
	client->get_cur_scenario_async = call_get_cur_scenario_async;
(void)init_registry(ext->get_cur_scenario_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->audio_config_sync = call_audio_config_sync;
#endif
	client->audio_config_async = call_audio_config_async;
(void)init_registry(ext->audio_config_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->send_dma_buff_global_fd_sync = call_send_dma_buff_global_fd_sync;
#endif
	client->send_dma_buff_global_fd_async = call_send_dma_buff_global_fd_async;
(void)init_registry(ext->send_dma_buff_global_fd_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->data_ready_in_pingpong_buff_sync = call_data_ready_in_pingpong_buff_sync;
#endif
	client->data_ready_in_pingpong_buff_async = call_data_ready_in_pingpong_buff_async;
(void)init_registry(ext->data_ready_in_pingpong_buff_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->ctrl_cmd_sync = call_ctrl_cmd_sync;
#endif
	client->ctrl_cmd_async = call_ctrl_cmd_async;
(void)init_registry(ext->ctrl_cmd_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->start_stop_to_play_sync = call_start_stop_to_play_sync;
#endif
	client->start_stop_to_play_async = call_start_stop_to_play_async;
(void)init_registry(ext->start_stop_to_play_registry);

	client->heartbeat_sub = subscribe_heartbeat;
	client->heartbeat_unsub = unsubscribe_heartbeat;
	(void)init_registry(&ext->heartbeat_registry);
	client->kws_triggered_sub = subscribe_kws_triggered;
	client->kws_triggered_unsub = unsubscribe_kws_triggered;
	(void)init_registry(&ext->kws_triggered_registry);
	client->data_comsumed_event_sub = subscribe_data_comsumed_event;
	client->data_comsumed_event_unsub = unsubscribe_data_comsumed_event;
	(void)init_registry(&ext->data_comsumed_event_registry);

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
void audio_ipc_client_destroy(void)
{
	destroy_registry(&s_ext->heartbeat_registry);
	destroy_registry(&s_ext->kws_triggered_registry);
	destroy_registry(&s_ext->data_comsumed_event_registry);

	s_data = NULL;
	s_ext = NULL;
}
