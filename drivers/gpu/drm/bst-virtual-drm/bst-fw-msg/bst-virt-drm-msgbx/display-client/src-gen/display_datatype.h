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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 8957426 msgbx_ipc 1964fef
 */

#ifndef DISPLAY_DATATYPE_H
#define DISPLAY_DATATYPE_H

#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_common.h>
#else
#include "ipc_app_common.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
struct _display_Array_Uint8_t {
	uint8_t *data;
	uint32_t size;
};
#define display_Array_Uint8_t struct _display_Array_Uint8_t

struct _display_Array_Uint16_t {
	uint16_t *data;
	uint32_t size;
};
#define display_Array_Uint16_t struct _display_Array_Uint16_t

struct _display_Array_Uint32_t {
	uint32_t *data;
	uint32_t size;
};
#define display_Array_Uint32_t struct _display_Array_Uint32_t
typedef uint8_t display_magic_data_t[4];
typedef uint8_t display_reserve_data_t[11];

enum _display_ErrorEnum_t {
	DISPLAY_NO_ERROR = 0,
	DISPLAY_SERVER_FAIL = -1,
	DISPLAY_ERROR_2 = -2,
	DISPLAY_ERROR_3 = -3
};
#define display_ErrorEnum_t enum _display_ErrorEnum_t

struct _display_bst_display_cmd_head_t {
	uint32_t client_id;
	uint32_t subdev_session;
	uint8_t cmdset;
	uint8_t cmdid;
	display_magic_data_t magic;
	display_reserve_data_t reserve;
};
#define display_bst_display_cmd_head_t struct _display_bst_display_cmd_head_t

struct _display_event_status_t {
	uint32_t client_id;
	uint32_t events_type;
};
#define display_event_status_t struct _display_event_status_t

// constants


// type serialize / deserialize functions
/**
 * Serialize display_Array_Uint8_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_Array_Uint8(
							serdes_t *ser,
							const display_Array_Uint8_t *in)
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
 * Deserialize display_Array_Uint8_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_Array_Uint8(
							des_buf_t *buf,
							display_Array_Uint8_t *out)
{
	int32_t ret = 0;
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

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize display_Array_Uint16_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_Array_Uint16(
							serdes_t *ser,
							const display_Array_Uint16_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)in->data,
				in->size * sizeof(uint16_t), 4);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_Array_Uint16_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_Array_Uint16(
							des_buf_t *buf,
							display_Array_Uint16_t *out)
{
	int32_t ret = 0;
	uint32_t size = 0;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint16_t);
	out->data = (uint16_t *)alloc_data(buf, size, 4);
	if (size > 0 && !out->data)
		return -1;

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize display_Array_Uint32_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_Array_Uint32(
							serdes_t *ser,
							const display_Array_Uint32_t *in)
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
 * Deserialize display_Array_Uint32_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_Array_Uint32(
							des_buf_t *buf,
							display_Array_Uint32_t *out)
{
	int32_t ret = 0;
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

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize display_magic_data_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_magic_data(
							serdes_t *ser,
							const display_magic_data_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				4 * sizeof(uint8_t), 1);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_magic_data_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_magic_data(
							des_buf_t *buf,
							display_magic_data_t **out)
{
	int32_t ret = 0;
	uint32_t size = 4 * sizeof(uint8_t);

	if (!out || !buf)
		return -1;

	*out = (display_magic_data_t *)alloc_data(buf, size, 1);
	if (!*out)
		return -1;

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize display_reserve_data_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_reserve_data(
							serdes_t *ser,
							const display_reserve_data_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				11 * sizeof(uint8_t), 1);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_reserve_data_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_reserve_data(
							des_buf_t *buf,
							display_reserve_data_t **out)
{
	int32_t ret = 0;
	uint32_t size = 11 * sizeof(uint8_t);

	if (!out || !buf)
		return -1;

	*out = (display_reserve_data_t *)alloc_data(buf, size, 1);
	if (!*out)
		return -1;

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize display_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_ErrorEnum(
							serdes_t *ser,
							const display_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_ErrorEnum(
							des_buf_t *buf,
							display_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (display_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize display_bst_display_cmd_head_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_bst_display_cmd_head(
							serdes_t *ser,
							const display_bst_display_cmd_head_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(display_bst_display_cmd_head_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_bst_display_cmd_head_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_bst_display_cmd_head(
							des_buf_t *buf,
							display_bst_display_cmd_head_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (display_bst_display_cmd_head_t *)alloc_data(buf, sizeof(display_bst_display_cmd_head_t), 4);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize display_event_status_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_display_event_status(
							serdes_t *ser,
							const display_event_status_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(display_event_status_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize display_event_status_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_display_event_status(
							des_buf_t *buf,
							display_event_status_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (display_event_status_t *)alloc_data(buf, sizeof(display_event_status_t), 4);
	if (!*out)
		return -1;

	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
