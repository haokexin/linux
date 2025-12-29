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

#ifndef AUDIO_IPC_DATATYPE_H
#define AUDIO_IPC_DATATYPE_H

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
struct _audio_ipc_MyArray_t {
	uint16_t *data;
	uint32_t size;
};
#define audio_ipc_MyArray_t struct _audio_ipc_MyArray_t

enum _audio_ipc_ErrorEnum_t {
	AUDIO_IPC_NO_ERROR = 0,
	AUDIO_IPC_SERVER_FAIL = -1,
	AUDIO_IPC_ERROR_2 = -2,
	AUDIO_IPC_ERROR_3 = -3
};
#define audio_ipc_ErrorEnum_t enum _audio_ipc_ErrorEnum_t

enum _audio_ipc_DataReadyTypeEnum_t {
	AUDIO_IPC_DATA_READY_IN_PING = 0,
	AUDIO_IPC_DATA_READY_IN_PANG = 1
};
#define audio_ipc_DataReadyTypeEnum_t enum _audio_ipc_DataReadyTypeEnum_t

struct _audio_ipc_MyStruct_t {
	uint8_t m1;
	bool m2;
	audio_ipc_MyArray_t m3;
	char *m4;
	byte_buffer_t m5;
};
#define audio_ipc_MyStruct_t struct _audio_ipc_MyStruct_t

union _audio_ipc_MyUnion_t {
	uint8_t m1;
	uint16_t m2;
	uint32_t m3;
};
#define audio_ipc_MyUnion_t union _audio_ipc_MyUnion_t

struct _audio_ipc_XrpDspCmd_t {
	uint32_t flags;
	uint32_t in_data_size;
	uint32_t out_data_size;
	uint32_t buffer_size;
	uint32_t in_data_addr;
	byte_buffer_t in_data;
	uint32_t out_data_addr;
	byte_buffer_t out_data;
	uint32_t buffer_addr;
	byte_buffer_t buffer_data;
	byte_buffer_t buffer_alignment;
	byte_buffer_t nsid;
};
#define audio_ipc_XrpDspCmd_t struct _audio_ipc_XrpDspCmd_t

// constants
static const bool b1 = true;
static const uint32_t MAX_COUNT = 10000;
static const uint16_t SOME_ID = 40971;
static const uint8_t BYTE_ME = 51;
static const double pi = 3.141500;

// type serialize / deserialize functions
/**
 * Serialize audio_ipc_MyArray_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_audio_ipc_MyArray(
							serdes_t *ser,
							const audio_ipc_MyArray_t *in)
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
 * Deserialize audio_ipc_MyArray_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_audio_ipc_MyArray(
							des_buf_t *buf,
							audio_ipc_MyArray_t *out)
{
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

	return 0;
}

/**
 * Serialize audio_ipc_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_audio_ipc_ErrorEnum(
							serdes_t *ser,
							const audio_ipc_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize audio_ipc_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_audio_ipc_ErrorEnum(
							des_buf_t *buf,
							audio_ipc_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (audio_ipc_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize audio_ipc_DataReadyTypeEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_audio_ipc_DataReadyTypeEnum(
							serdes_t *ser,
							const audio_ipc_DataReadyTypeEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize audio_ipc_DataReadyTypeEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_audio_ipc_DataReadyTypeEnum(
							des_buf_t *buf,
							audio_ipc_DataReadyTypeEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (audio_ipc_DataReadyTypeEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize audio_ipc_MyStruct_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_audio_ipc_MyStruct(
							serdes_t *ser,
							const audio_ipc_MyStruct_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&in->m1);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&in->m2);
	if (ret >= 0)
		ret = serialize_audio_ipc_MyArray(ser, &in->m3);
	if (ret >= 0)
		ret = serialize_string(ser, in->m4);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->m5);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize audio_ipc_MyStruct_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_audio_ipc_MyStruct(
							des_buf_t *buf,
							audio_ipc_MyStruct_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&out->m1);
	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&out->m2);
	if (ret >= 0)
		ret = deserialize_audio_ipc_MyArray(buf, &out->m3);
	if (ret >= 0)
		ret = deserialize_string(buf, &out->m4);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->m5);

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize audio_ipc_MyUnion_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 * @note union size greater than 64 bit is NOT supported, return -1.
 */
static inline int32_t serialize_audio_ipc_MyUnion(
							serdes_t *ser,
							const audio_ipc_MyUnion_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (uint8_t *)in, sizeof(audio_ipc_MyUnion_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize audio_ipc_MyUnion_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 * @note union size greater than 64 bit is NOT supported, return -1.
 */
static inline int32_t deserialize_audio_ipc_MyUnion(
							des_buf_t *buf,
							audio_ipc_MyUnion_t *out)
{
	audio_ipc_MyUnion_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (audio_ipc_MyUnion_t *)alloc_data(buf, sizeof(audio_ipc_MyUnion_t), 4);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

/**
 * Serialize audio_ipc_XrpDspCmd_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_audio_ipc_XrpDspCmd(
							serdes_t *ser,
							const audio_ipc_XrpDspCmd_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->flags);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->in_data_size);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->out_data_size);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->buffer_size);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->in_data_addr);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->in_data);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->out_data_addr);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->out_data);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->buffer_addr);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->buffer_data);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->buffer_alignment);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->nsid);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize audio_ipc_XrpDspCmd_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_audio_ipc_XrpDspCmd(
							des_buf_t *buf,
							audio_ipc_XrpDspCmd_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->flags);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->in_data_size);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->out_data_size);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->buffer_size);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->in_data_addr);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->in_data);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->out_data_addr);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->out_data);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->buffer_addr);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->buffer_data);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->buffer_alignment);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->nsid);

	return ret >= 0 ? 0 : -1;
}



#ifdef __cplusplus
}
#endif

#endif
