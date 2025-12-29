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

#include "switch0_can_gateway_client.h"

// macro definitions
#define CID SWITCH_0
#define CCID 0
#define CID_MASK (0x1U << 18)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_NOTIFY_ETH_2_CAN_ENQUEUE 1U
#define CMD_METHOD_NOTIFY_ETH_2_CAN_DEQUEUE 2U
#define CMD_METHOD_NOTIFY_CAN_2_ETH_ENQUEUE 3U
#define CMD_METHOD_NOTIFY_CAN_2_ETH_DEQUEUE 4U
#define CMD_METHOD_VMAC_METHOD 5U
#define CMD_METHOD_GET_ETH_STATISTICS 6U
#define CMD_METHOD_VCAN_METHOD 7U
#define CMD_METHOD_CAN_GATEWAY_SEND 8U
#define CMD_METHOD_CAN_CONFIG_QUERY 9U
#define CMD_METHOD_ADAS_VCAN_NOTIFY_SQBUF_ADDR 14U
#define CMD_METHOD_ADAS_VCAN_DRIVER_REMOVE 15U
#define CMD_METHOD_VCAN_PORT_METHOD 16U

#define CMD_METHOD_SUB_CAN_GATEWAY_RECV 64U
#define CMD_METHOD_UNSUB_CAN_GATEWAY_RECV 65U
#define CMD_BROADCAST_CAN_GATEWAY_RECV 1U
#define CMD_METHOD_SUB_CAN_STATUS_NOTIFY 12U
#define CMD_METHOD_UNSUB_CAN_STATUS_NOTIFY 13U
#define CMD_BROADCAST_CAN_STATUS_NOTIFY 2U
#define CMD_METHOD_SUB_VCAN_PORT_BROADCAST 17U
#define CMD_METHOD_UNSUB_VCAN_PORT_BROADCAST 18U
#define CMD_BROADCAST_VCAN_PORT_BROADCAST 3U

// local variables
static com_client_data_t *s_data;
static switch0_can_gateway_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _vmac_method_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_ErrorEnum_t *err;
};
#define vmac_method_out_t struct _vmac_method_out_t

struct _get_ETH_statistics_out_t {
	DECL_SEM(sem)
	uint32_t *statistics_data_ptr;
	switch0_can_gateway_ErrorEnum_t *err;
};
#define get_ETH_statistics_out_t struct _get_ETH_statistics_out_t

struct _vcan_method_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_ErrorEnum_t *err;
};
#define vcan_method_out_t struct _vcan_method_out_t

struct _can_gateway_send_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_ErrorEnum_t *err;
};
#define can_gateway_send_out_t struct _can_gateway_send_out_t

struct _can_config_query_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_can_config_t **config_info;
	switch0_can_gateway_ErrorEnum_t *err;
};
#define can_config_query_out_t struct _can_config_query_out_t

struct _adas_vcan_notify_sqbuf_addr_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_ErrorEnum_t *err;
};
#define adas_vcan_notify_sqbuf_addr_out_t struct _adas_vcan_notify_sqbuf_addr_out_t

struct _adas_vcan_driver_remove_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_ErrorEnum_t *err;
};
#define adas_vcan_driver_remove_out_t struct _adas_vcan_driver_remove_out_t

struct _vcan_port_method_out_t {
	DECL_SEM(sem)
	switch0_can_gateway_vcan_port_msg_t *rsp;
	switch0_can_gateway_ErrorEnum_t *err;
};
#define vcan_port_method_out_t struct _vcan_port_method_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_notify_Eth_2_Can_enqueue(
				serdes_t *ser,
				const uint32_t element_ptr,
				const uint16_t element_count
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&element_ptr);
	if (ret >= 0)
		ret = ipc_ser_put_16(ser, (uint16_t *)&element_count);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_notify_Eth_2_Can_enqueue_fire_and_forget(const uint32_t element_ptr,
				const uint16_t element_count)
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

	ret = serialize_notify_Eth_2_Can_enqueue(ser, element_ptr, element_count);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NOTIFY_ETH_2_CAN_ENQUEUE);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_notify_Eth_2_Can_dequeue(
				serdes_t *ser,
				const uint32_t element_ptr,
				const uint16_t element_count
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&element_ptr);
	if (ret >= 0)
		ret = ipc_ser_put_16(ser, (uint16_t *)&element_count);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_notify_Eth_2_Can_dequeue_fire_and_forget(const uint32_t element_ptr,
				const uint16_t element_count)
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

	ret = serialize_notify_Eth_2_Can_dequeue(ser, element_ptr, element_count);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NOTIFY_ETH_2_CAN_DEQUEUE);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_notify_Can_2_Eth_enqueue(
				serdes_t *ser,
				const uint32_t element_ptr,
				const uint16_t element_count
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&element_ptr);
	if (ret >= 0)
		ret = ipc_ser_put_16(ser, (uint16_t *)&element_count);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_notify_Can_2_Eth_enqueue_fire_and_forget(const uint32_t element_ptr,
				const uint16_t element_count)
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

	ret = serialize_notify_Can_2_Eth_enqueue(ser, element_ptr, element_count);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NOTIFY_CAN_2_ETH_ENQUEUE);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_notify_Can_2_Eth_dequeue(
				serdes_t *ser,
				const uint32_t element_ptr,
				const uint16_t element_count
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&element_ptr);
	if (ret >= 0)
		ret = ipc_ser_put_16(ser, (uint16_t *)&element_count);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}

static int32_t call_notify_Can_2_Eth_dequeue_fire_and_forget(const uint32_t element_ptr,
				const uint16_t element_count)
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

	ret = serialize_notify_Can_2_Eth_dequeue(ser, element_ptr, element_count);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_fire_and_forget_request(data, ser, s_ext->cid, CMD_METHOD_NOTIFY_CAN_2_ETH_DEQUEUE);
	if (ret < 0) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t serialize_vmac_method(
				serdes_t *ser,
				const switch0_can_gateway_MyArray_t vmac_msg,
				const uint32_t flag
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_switch0_can_gateway_MyArray(ser, &vmac_msg);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&flag);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void vmac_method_sync_callback(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	vmac_method_out_t *out = (vmac_method_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_vmac_method_sync(const switch0_can_gateway_MyArray_t vmac_msg,
				const uint32_t flag,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	vmac_method_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_vmac_method(ser, vmac_msg, flag);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vmac_method_registry, ser, s_ext->cid,
			CMD_METHOD_VMAC_METHOD, vmac_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->vmac_method_registry[ret];
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

static int32_t call_vmac_method_async(const switch0_can_gateway_MyArray_t vmac_msg,
				const uint32_t flag,
				switch0_can_gateway_vmac_method_callback_t cb,
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

	ret = serialize_vmac_method(ser, vmac_msg, flag);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vmac_method_registry, ser, s_ext->cid,
			CMD_METHOD_VMAC_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_vmac_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_vmac_method_callback_t cb = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->vmac_method_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (switch0_can_gateway_vmac_method_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void get_ETH_statistics_sync_callback(
				const uint32_t statistics_data_ptr,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	get_ETH_statistics_out_t *out = (get_ETH_statistics_out_t *)ext;

	if (!out)
		return;
	*out->statistics_data_ptr = statistics_data_ptr;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_get_ETH_statistics_sync(uint32_t *statistics_data_ptr,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	get_ETH_statistics_out_t out = {.statistics_data_ptr = statistics_data_ptr,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	// send request
	ret = send_request(data, s_ext->get_ETH_statistics_registry, ser, s_ext->cid,
			CMD_METHOD_GET_ETH_STATISTICS, get_ETH_statistics_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->get_ETH_statistics_registry[ret];
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

static int32_t call_get_ETH_statistics_async(switch0_can_gateway_get_ETH_statistics_callback_t cb,
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
	ret = send_request(data, s_ext->get_ETH_statistics_registry, ser, s_ext->cid,
			CMD_METHOD_GET_ETH_STATISTICS, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_get_ETH_statistics_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_get_ETH_statistics_callback_t cb = NULL;
	uint32_t statistics_data_ptr = 0;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->get_ETH_statistics_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SWITCH0_CAN_GATEWAY_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&statistics_data_ptr);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (switch0_can_gateway_get_ETH_statistics_callback_t)(reg->cb);
	if (cb)
		cb(statistics_data_ptr, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_vcan_method(
				serdes_t *ser,
				const uint8_t command,
				const uint32_t param
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&command);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&param);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void vcan_method_sync_callback(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	vcan_method_out_t *out = (vcan_method_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_vcan_method_sync(const uint8_t command,
				const uint32_t param,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	vcan_method_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_vcan_method(ser, command, param);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vcan_method_registry, ser, s_ext->cid,
			CMD_METHOD_VCAN_METHOD, vcan_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->vcan_method_registry[ret];
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

static int32_t call_vcan_method_async(const uint8_t command,
				const uint32_t param,
				switch0_can_gateway_vcan_method_callback_t cb,
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

	ret = serialize_vcan_method(ser, command, param);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vcan_method_registry, ser, s_ext->cid,
			CMD_METHOD_VCAN_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_vcan_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_vcan_method_callback_t cb = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->vcan_method_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (switch0_can_gateway_vcan_method_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_can_gateway_send(
				serdes_t *ser,
				const switch0_can_gateway_UInt8Array128_t *x2can
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_switch0_can_gateway_UInt8Array128(ser, x2can);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void can_gateway_send_sync_callback(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	can_gateway_send_out_t *out = (can_gateway_send_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_can_gateway_send_sync(const switch0_can_gateway_UInt8Array128_t *x2can,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	can_gateway_send_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_can_gateway_send(ser, x2can);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->can_gateway_send_registry, ser, s_ext->cid,
			CMD_METHOD_CAN_GATEWAY_SEND, can_gateway_send_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->can_gateway_send_registry[ret];
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

static int32_t call_can_gateway_send_async(const switch0_can_gateway_UInt8Array128_t *x2can,
				switch0_can_gateway_can_gateway_send_callback_t cb,
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

	ret = serialize_can_gateway_send(ser, x2can);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->can_gateway_send_registry, ser, s_ext->cid,
			CMD_METHOD_CAN_GATEWAY_SEND, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_can_gateway_send_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_can_gateway_send_callback_t cb = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->can_gateway_send_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (switch0_can_gateway_can_gateway_send_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_can_config_query(
				serdes_t *ser,
				const uint8_t can_bus_id
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&can_bus_id);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void can_config_query_sync_callback(
				const switch0_can_gateway_can_config_t *config_info,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	can_config_query_out_t *out = (can_config_query_out_t *)ext;

	if (!out)
		return;
	*out->config_info = (switch0_can_gateway_can_config_t *)config_info;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_can_config_query_sync(const uint8_t can_bus_id,
				switch0_can_gateway_can_config_t **config_info,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	can_config_query_out_t out = {.config_info = config_info,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_can_config_query(ser, can_bus_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->can_config_query_registry, ser, s_ext->cid,
			CMD_METHOD_CAN_CONFIG_QUERY, can_config_query_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->can_config_query_registry[ret];
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

static int32_t call_can_config_query_async(const uint8_t can_bus_id,
				switch0_can_gateway_can_config_query_callback_t cb,
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

	ret = serialize_can_config_query(ser, can_bus_id);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->can_config_query_registry, ser, s_ext->cid,
			CMD_METHOD_CAN_CONFIG_QUERY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_can_config_query_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_can_config_query_callback_t cb = NULL;
	switch0_can_gateway_can_config_t *config_info = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->can_config_query_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SWITCH0_CAN_GATEWAY_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_switch0_can_gateway_can_config(buf, &config_info);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (switch0_can_gateway_can_config_query_callback_t)(reg->cb);
	if (cb)
		cb(config_info, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_adas_vcan_notify_sqbuf_addr(
				serdes_t *ser,
				const uint32_t pa_low,
				const uint32_t pa_high,
				const uint32_t size
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&pa_low);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&pa_high);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&size);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void adas_vcan_notify_sqbuf_addr_sync_callback(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	adas_vcan_notify_sqbuf_addr_out_t *out = (adas_vcan_notify_sqbuf_addr_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_adas_vcan_notify_sqbuf_addr_sync(const uint32_t pa_low,
				const uint32_t pa_high,
				const uint32_t size,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	adas_vcan_notify_sqbuf_addr_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_adas_vcan_notify_sqbuf_addr(ser, pa_low, pa_high, size);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->adas_vcan_notify_sqbuf_addr_registry, ser, s_ext->cid,
			CMD_METHOD_ADAS_VCAN_NOTIFY_SQBUF_ADDR, adas_vcan_notify_sqbuf_addr_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->adas_vcan_notify_sqbuf_addr_registry[ret];
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

static int32_t call_adas_vcan_notify_sqbuf_addr_async(const uint32_t pa_low,
				const uint32_t pa_high,
				const uint32_t size,
				switch0_can_gateway_adas_vcan_notify_sqbuf_addr_callback_t cb,
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

	ret = serialize_adas_vcan_notify_sqbuf_addr(ser, pa_low, pa_high, size);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->adas_vcan_notify_sqbuf_addr_registry, ser, s_ext->cid,
			CMD_METHOD_ADAS_VCAN_NOTIFY_SQBUF_ADDR, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_adas_vcan_notify_sqbuf_addr_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_adas_vcan_notify_sqbuf_addr_callback_t cb = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->adas_vcan_notify_sqbuf_addr_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (switch0_can_gateway_adas_vcan_notify_sqbuf_addr_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void adas_vcan_driver_remove_sync_callback(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	adas_vcan_driver_remove_out_t *out = (adas_vcan_driver_remove_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_adas_vcan_driver_remove_sync(switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	adas_vcan_driver_remove_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	// send request
	ret = send_request(data, s_ext->adas_vcan_driver_remove_registry, ser, s_ext->cid,
			CMD_METHOD_ADAS_VCAN_DRIVER_REMOVE, adas_vcan_driver_remove_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->adas_vcan_driver_remove_registry[ret];
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

static int32_t call_adas_vcan_driver_remove_async(switch0_can_gateway_adas_vcan_driver_remove_callback_t cb,
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
	ret = send_request(data, s_ext->adas_vcan_driver_remove_registry, ser, s_ext->cid,
			CMD_METHOD_ADAS_VCAN_DRIVER_REMOVE, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_adas_vcan_driver_remove_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_adas_vcan_driver_remove_callback_t cb = NULL;
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->adas_vcan_driver_remove_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (switch0_can_gateway_adas_vcan_driver_remove_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_vcan_port_method(
				serdes_t *ser,
				const switch0_can_gateway_vcan_port_msg_t req
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_switch0_can_gateway_vcan_port_msg(ser, &req);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void vcan_port_method_sync_callback(
				const switch0_can_gateway_vcan_port_msg_t rsp,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	vcan_port_method_out_t *out = (vcan_port_method_out_t *)ext;

	if (!out)
		return;
	out->rsp->msg_len = rsp.msg_len;
	out->rsp->cmd_id = rsp.cmd_id;
	out->rsp->content.size = rsp.content.size;
	out->rsp->content.data = rsp.content.data;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_vcan_port_method_sync(const switch0_can_gateway_vcan_port_msg_t req,
				switch0_can_gateway_vcan_port_msg_t *rsp,
				switch0_can_gateway_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	vcan_port_method_out_t out = {.rsp = rsp,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_vcan_port_method(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vcan_port_method_registry, ser, s_ext->cid,
			CMD_METHOD_VCAN_PORT_METHOD, vcan_port_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->vcan_port_method_registry[ret];
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

static int32_t call_vcan_port_method_async(const switch0_can_gateway_vcan_port_msg_t req,
				switch0_can_gateway_vcan_port_method_callback_t cb,
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

	ret = serialize_vcan_port_method(ser, req);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->vcan_port_method_registry, ser, s_ext->cid,
			CMD_METHOD_VCAN_PORT_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_vcan_port_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	switch0_can_gateway_vcan_port_method_callback_t cb = NULL;
	switch0_can_gateway_vcan_port_msg_t rsp = { 0 };
	switch0_can_gateway_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->vcan_port_method_registry[des->header.tok];
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
		ret = deserialize_switch0_can_gateway_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SWITCH0_CAN_GATEWAY_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_switch0_can_gateway_vcan_port_msg(buf, &rsp);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (switch0_can_gateway_vcan_port_method_callback_t)(reg->cb);
	if (cb)
		cb(rsp, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe can_gateway_recv
static int32_t subscribe_can_gateway_recv(
				switch0_can_gateway_can_gateway_recv_callback_t cb,
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
	s_ext->can_gateway_recv_registry.busy = true;
	(void)set_registry(&s_ext->can_gateway_recv_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_CAN_GATEWAY_RECV, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->can_gateway_recv_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe can_gateway_recv
static int32_t unsubscribe_can_gateway_recv(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->can_gateway_recv_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_CAN_GATEWAY_RECV, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_can_gateway_recv_callback(des_buf_t *des)
{
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	switch0_can_gateway_can_gateway_recv_callback_t cb = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->can_gateway_recv_registry;
	if (!reg->busy || !reg->cb) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return RESULT_SUCCESS;
	}

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	cb = (switch0_can_gateway_can_gateway_recv_callback_t)(reg->cb);
	cb(reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe can_status_notify
static int32_t subscribe_can_status_notify(
				switch0_can_gateway_can_status_notify_callback_t cb,
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
	s_ext->can_status_notify_registry.busy = true;
	(void)set_registry(&s_ext->can_status_notify_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_CAN_STATUS_NOTIFY, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->can_status_notify_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe can_status_notify
static int32_t unsubscribe_can_status_notify(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->can_status_notify_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_CAN_STATUS_NOTIFY, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_can_status_notify_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	switch0_can_gateway_can_status_notify_callback_t cb = NULL;
	switch0_can_gateway_can_status_t *status_info = NULL;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->can_status_notify_registry;
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
		ret = deserialize_switch0_can_gateway_can_status(buf, &status_info);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (switch0_can_gateway_can_status_notify_callback_t)(reg->cb);
	cb(status_info, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// subscribe vcan_port_broadcast
static int32_t subscribe_vcan_port_broadcast(
				switch0_can_gateway_vcan_port_broadcast_callback_t cb,
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
	s_ext->vcan_port_broadcast_registry.busy = true;
	(void)set_registry(&s_ext->vcan_port_broadcast_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_VCAN_PORT_BROADCAST, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->vcan_port_broadcast_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe vcan_port_broadcast
static int32_t unsubscribe_vcan_port_broadcast(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->vcan_port_broadcast_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_VCAN_PORT_BROADCAST, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_vcan_port_broadcast_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	switch0_can_gateway_vcan_port_broadcast_callback_t cb = NULL;
	switch0_can_gateway_vcan_port_msg_t evt = { 0 };

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->vcan_port_broadcast_registry;
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
		ret = deserialize_switch0_can_gateway_vcan_port_msg(buf, &evt);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (switch0_can_gateway_vcan_port_broadcast_callback_t)(reg->cb);
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
	case CMD_BROADCAST_CAN_GATEWAY_RECV:
		ret = call_can_gateway_recv_callback(des);
		break;
	case CMD_BROADCAST_CAN_STATUS_NOTIFY:
		ret = call_can_status_notify_callback(des);
		break;
	case CMD_BROADCAST_VCAN_PORT_BROADCAST:
		ret = call_vcan_port_broadcast_callback(des);
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
	case CMD_METHOD_VMAC_METHOD:
		ret = call_vmac_method_callback(des);
		break;
	case CMD_METHOD_GET_ETH_STATISTICS:
		ret = call_get_ETH_statistics_callback(des);
		break;
	case CMD_METHOD_VCAN_METHOD:
		ret = call_vcan_method_callback(des);
		break;
	case CMD_METHOD_CAN_GATEWAY_SEND:
		ret = call_can_gateway_send_callback(des);
		break;
	case CMD_METHOD_CAN_CONFIG_QUERY:
		ret = call_can_config_query_callback(des);
		break;
	case CMD_METHOD_ADAS_VCAN_NOTIFY_SQBUF_ADDR:
		ret = call_adas_vcan_notify_sqbuf_addr_callback(des);
		break;
	case CMD_METHOD_ADAS_VCAN_DRIVER_REMOVE:
		ret = call_adas_vcan_driver_remove_callback(des);
		break;
	case CMD_METHOD_VCAN_PORT_METHOD:
		ret = call_vcan_port_method_callback(des);
		break;
	case CMD_METHOD_SUB_CAN_GATEWAY_RECV:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_CAN_GATEWAY_RECV:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->can_gateway_recv_registry);
		break;
	case CMD_METHOD_SUB_CAN_STATUS_NOTIFY:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_CAN_STATUS_NOTIFY:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->can_status_notify_registry);
		break;
	case CMD_METHOD_SUB_VCAN_PORT_BROADCAST:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_VCAN_PORT_BROADCAST:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->vcan_port_broadcast_registry);
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
int32_t switch0_can_gateway_client_init(com_client_data_t *data, switch0_can_gateway_client_t *client,
			switch0_can_gateway_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
	client->notify_Eth_2_Can_enqueue_fire_and_forget = call_notify_Eth_2_Can_enqueue_fire_and_forget;
	client->notify_Eth_2_Can_dequeue_fire_and_forget = call_notify_Eth_2_Can_dequeue_fire_and_forget;
	client->notify_Can_2_Eth_enqueue_fire_and_forget = call_notify_Can_2_Eth_enqueue_fire_and_forget;
	client->notify_Can_2_Eth_dequeue_fire_and_forget = call_notify_Can_2_Eth_dequeue_fire_and_forget;
#ifndef IPC_RTE_BAREMETAL
	client->vmac_method_sync = call_vmac_method_sync;
#endif
	client->vmac_method_async = call_vmac_method_async;
(void)init_registry(ext->vmac_method_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->get_ETH_statistics_sync = call_get_ETH_statistics_sync;
#endif
	client->get_ETH_statistics_async = call_get_ETH_statistics_async;
(void)init_registry(ext->get_ETH_statistics_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->vcan_method_sync = call_vcan_method_sync;
#endif
	client->vcan_method_async = call_vcan_method_async;
(void)init_registry(ext->vcan_method_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->can_gateway_send_sync = call_can_gateway_send_sync;
#endif
	client->can_gateway_send_async = call_can_gateway_send_async;
(void)init_registry(ext->can_gateway_send_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->can_config_query_sync = call_can_config_query_sync;
#endif
	client->can_config_query_async = call_can_config_query_async;
(void)init_registry(ext->can_config_query_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->adas_vcan_notify_sqbuf_addr_sync = call_adas_vcan_notify_sqbuf_addr_sync;
#endif
	client->adas_vcan_notify_sqbuf_addr_async = call_adas_vcan_notify_sqbuf_addr_async;
(void)init_registry(ext->adas_vcan_notify_sqbuf_addr_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->adas_vcan_driver_remove_sync = call_adas_vcan_driver_remove_sync;
#endif
	client->adas_vcan_driver_remove_async = call_adas_vcan_driver_remove_async;
(void)init_registry(ext->adas_vcan_driver_remove_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->vcan_port_method_sync = call_vcan_port_method_sync;
#endif
	client->vcan_port_method_async = call_vcan_port_method_async;
(void)init_registry(ext->vcan_port_method_registry);

	client->can_gateway_recv_sub = subscribe_can_gateway_recv;
	client->can_gateway_recv_unsub = unsubscribe_can_gateway_recv;
	(void)init_registry(&ext->can_gateway_recv_registry);
	client->can_status_notify_sub = subscribe_can_status_notify;
	client->can_status_notify_unsub = unsubscribe_can_status_notify;
	(void)init_registry(&ext->can_status_notify_registry);
	client->vcan_port_broadcast_sub = subscribe_vcan_port_broadcast;
	client->vcan_port_broadcast_unsub = unsubscribe_vcan_port_broadcast;
	(void)init_registry(&ext->vcan_port_broadcast_registry);

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
void switch0_can_gateway_client_destroy(void)
{
	destroy_registry(&s_ext->can_gateway_recv_registry);
	destroy_registry(&s_ext->can_status_notify_registry);
	destroy_registry(&s_ext->vcan_port_broadcast_registry);

	s_data = NULL;
	s_ext = NULL;
}
