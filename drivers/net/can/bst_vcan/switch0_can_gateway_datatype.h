/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
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

#ifndef SWITCH0_CAN_GATEWAY_DATATYPE_H
#define SWITCH0_CAN_GATEWAY_DATATYPE_H

#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_common.h>
#else
#include "ipc_app_common.h"
#endif

#if __has_include(<inttypes.h>)
  #include <inttypes.h>
#else
#ifndef PRId32
#define PRId32 "d"
#endif
#ifndef PRIu8
#define PRIu8 "u"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
struct _switch0_can_gateway_MyArray_t {
	uint8_t *data;
	uint32_t size;
};
#define switch0_can_gateway_MyArray_t struct _switch0_can_gateway_MyArray_t

enum _switch0_can_gateway_ErrorEnum_t {
	SWITCH0_CAN_GATEWAY_NO_ERROR = 0,
	SWITCH0_CAN_GATEWAY_SERVER_FAIL = -1,
	SWITCH0_CAN_GATEWAY_ERROR_2 = -2,
	SWITCH0_CAN_GATEWAY_ERROR_3 = -3
};
#define switch0_can_gateway_ErrorEnum_t enum _switch0_can_gateway_ErrorEnum_t

struct _switch0_can_gateway_time_segment_type_t {
	uint16_t presdiv;
	uint8_t propseg;
	uint8_t phaseseg1;
	uint8_t phaseseg2;
};
#define switch0_can_gateway_time_segment_type_t struct _switch0_can_gateway_time_segment_type_t

struct _switch0_can_gateway_can_config_t {
	uint8_t can_bus_id;
	uint8_t canfd_enable;
	uint32_t nominal_baudrate;
	uint32_t data_baudrate;
	switch0_can_gateway_time_segment_type_t nominal_timing;
	switch0_can_gateway_time_segment_type_t data_timing;
};
#define switch0_can_gateway_can_config_t struct _switch0_can_gateway_can_config_t

enum _switch0_can_gateway_sw_status_enum_t {
	SWITCH0_CAN_GATEWAY_SW_STATUS_OK = 0,
	SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNINIT = 1,
	SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_DISABLED = 2,
	SWITCH0_CAN_GATEWAY_SW_STATUS_CANIP_UNKOWN = 4
};
#define switch0_can_gateway_sw_status_enum_t enum _switch0_can_gateway_sw_status_enum_t

struct _switch0_can_gateway_can_status_t {
	uint8_t can_bus_id;
	uint8_t sw_status;
	uint32_t reg_esr1;
	uint32_t reg_ecr;
};
#define switch0_can_gateway_can_status_t struct _switch0_can_gateway_can_status_t

struct _switch0_can_gateway_UInt32Array_t {
	uint32_t *data;
	uint32_t size;
};
#define switch0_can_gateway_UInt32Array_t struct _switch0_can_gateway_UInt32Array_t

struct _switch0_can_gateway_vcan_port_msg_t {
	uint32_t msg_len;
	uint32_t cmd_id;
	switch0_can_gateway_UInt32Array_t content;
};
#define switch0_can_gateway_vcan_port_msg_t struct _switch0_can_gateway_vcan_port_msg_t
typedef uint8_t switch0_can_gateway_UInt8Array128_t[128];

// constants


// type serialize / deserialize functions
/**
 * Serialize switch0_can_gateway_MyArray_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_MyArray(
							serdes_t *ser,
							const switch0_can_gateway_MyArray_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)in->data,
				in->size * sizeof(uint8_t), 4);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_MyArray_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_MyArray(
							des_buf_t *buf,
							switch0_can_gateway_MyArray_t *out)
{
	uint32_t size = 0;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint8_t);
	out->data = (uint8_t *)alloc_data(buf, size, 4);
	if (size > 0 && !out->data)
		return -1;

	return 0;
}

/**
 * Serialize switch0_can_gateway_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_ErrorEnum(
							serdes_t *ser,
							const switch0_can_gateway_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_ErrorEnum(
							des_buf_t *buf,
							switch0_can_gateway_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (switch0_can_gateway_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize switch0_can_gateway_time_segment_type_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_time_segment_type(
							serdes_t *ser,
							const switch0_can_gateway_time_segment_type_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(switch0_can_gateway_time_segment_type_t), 2);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_time_segment_type_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_time_segment_type(
							des_buf_t *buf,
							switch0_can_gateway_time_segment_type_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (switch0_can_gateway_time_segment_type_t *)alloc_data(buf, sizeof(switch0_can_gateway_time_segment_type_t), 2);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize switch0_can_gateway_can_config_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_can_config(
							serdes_t *ser,
							const switch0_can_gateway_can_config_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(switch0_can_gateway_can_config_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_can_config_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_can_config(
							des_buf_t *buf,
							switch0_can_gateway_can_config_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (switch0_can_gateway_can_config_t *)alloc_data(buf, sizeof(switch0_can_gateway_can_config_t), 4);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize switch0_can_gateway_sw_status_enum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_sw_status_enum(
							serdes_t *ser,
							const switch0_can_gateway_sw_status_enum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_sw_status_enum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_sw_status_enum(
							des_buf_t *buf,
							switch0_can_gateway_sw_status_enum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (switch0_can_gateway_sw_status_enum_t)(*ptr);
	return 0;
}

/**
 * Serialize switch0_can_gateway_can_status_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_can_status(
							serdes_t *ser,
							const switch0_can_gateway_can_status_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(switch0_can_gateway_can_status_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_can_status_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_can_status(
							des_buf_t *buf,
							switch0_can_gateway_can_status_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (switch0_can_gateway_can_status_t *)alloc_data(buf, sizeof(switch0_can_gateway_can_status_t), 4);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize switch0_can_gateway_UInt32Array_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_UInt32Array(
							serdes_t *ser,
							const switch0_can_gateway_UInt32Array_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)in->data,
				in->size * sizeof(uint32_t), 4);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_UInt32Array_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_UInt32Array(
							des_buf_t *buf,
							switch0_can_gateway_UInt32Array_t *out)
{
	uint32_t size = 0;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint32_t);
	out->data = (uint32_t *)alloc_data(buf, size, 4);
	if (size > 0 && !out->data)
		return -1;

	return 0;
}

/**
 * Serialize switch0_can_gateway_vcan_port_msg_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_vcan_port_msg(
							serdes_t *ser,
							const switch0_can_gateway_vcan_port_msg_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->msg_len);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->cmd_id);
	if (ret >= 0)
		ret = serialize_switch0_can_gateway_UInt32Array(ser, &in->content);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_vcan_port_msg_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_vcan_port_msg(
							des_buf_t *buf,
							switch0_can_gateway_vcan_port_msg_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->msg_len);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->cmd_id);
	if (ret >= 0)
		ret = deserialize_switch0_can_gateway_UInt32Array(buf, &out->content);

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize switch0_can_gateway_UInt8Array128_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_switch0_can_gateway_UInt8Array128(
							serdes_t *ser,
							const switch0_can_gateway_UInt8Array128_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				128 * sizeof(uint8_t), 1);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize switch0_can_gateway_UInt8Array128_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_switch0_can_gateway_UInt8Array128(
							des_buf_t *buf,
							switch0_can_gateway_UInt8Array128_t **out)
{
	uint32_t size = 128 * sizeof(uint8_t);

	if (!out || !buf)
		return -1;

	*out = (switch0_can_gateway_UInt8Array128_t *)alloc_data(buf, size, 1);
	if (!*out)
		return -1;

	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
