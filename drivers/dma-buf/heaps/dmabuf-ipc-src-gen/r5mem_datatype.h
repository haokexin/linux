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

#ifndef R5MEM_DATATYPE_H
#define R5MEM_DATATYPE_H

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
enum _r5mem_ErrorEnum_t {
	R5MEM_NO_ERROR = 0,
	R5MEM_SERVER_FAIL = -1
};
#define r5mem_ErrorEnum_t enum _r5mem_ErrorEnum_t

enum _r5mem_Consts_t {
	R5MEM_SHARED_BUF_SIZE = 2,
	R5MEM_MAX_FD = 256,
	R5MEM_MAX_POOL_NUM_PER_MEM_TYPE = 4
};
#define r5mem_Consts_t enum _r5mem_Consts_t

enum _r5mem_CmdType_t {
	R5MEM_ALLOC = 0,
	R5MEM_ALLOC_AND_ATTACH = 1,
	R5MEM_SET_RANGE = 2,
	R5MEM_GET_USAGE = 3,
	R5MEM_CONFIG_SECURE_MEM = 4,
	R5MEM_FREE = 5,
	R5MEM_ATTACH = 6,
	R5MEM_DETACH = 7,
	R5MEM_GET_PHYS_ADDR = 8,
	R5MEM_GET_PHYS_ADDR_AND_ATTACH = 9,
	R5MEM_CMD_MAX
};
#define r5mem_CmdType_t enum _r5mem_CmdType_t

enum _r5mem_MemType_t {
	R5MEM_TYPE_NORMAL = 0,
	R5MEM_TYPE_SECURE = 1,
	R5MEM_TYPE_HIFI = 2,
	R5MEM_TYPE_MAX
};
#define r5mem_MemType_t enum _r5mem_MemType_t

enum _r5mem_MasterId_t {
	R5MEM_COREIP = 0,
	R5MEM_NET,
	R5MEM_CV,
	R5MEM_DISPLAY,
	R5MEM_DISPLAY0,
	R5MEM_DISPLAY1,
	R5MEM_DISPLAY2,
	R5MEM_MEDIA_CODEC,
	R5MEM_GPU,
	R5MEM_MASTER_ID_MAX
};
#define r5mem_MasterId_t enum _r5mem_MasterId_t

struct _r5mem_UInt64Array_t {
	uint64_t *data;
	uint32_t size;
};
#define r5mem_UInt64Array_t struct _r5mem_UInt64Array_t

struct _r5mem_driver_ipc_msg_t {
	int32_t cmd;
	int32_t mem_type;
	r5mem_UInt64Array_t payload;
};
#define r5mem_driver_ipc_msg_t struct _r5mem_driver_ipc_msg_t

// constants


// type serialize / deserialize functions
/**
 * Serialize r5mem_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_ErrorEnum(
							serdes_t *ser,
							const r5mem_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_ErrorEnum(
							des_buf_t *buf,
							r5mem_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (r5mem_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize r5mem_Consts_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_Consts(
							serdes_t *ser,
							const r5mem_Consts_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_Consts_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_Consts(
							des_buf_t *buf,
							r5mem_Consts_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (r5mem_Consts_t)(*ptr);
	return 0;
}

/**
 * Serialize r5mem_CmdType_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_CmdType(
							serdes_t *ser,
							const r5mem_CmdType_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_CmdType_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_CmdType(
							des_buf_t *buf,
							r5mem_CmdType_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (r5mem_CmdType_t)(*ptr);
	return 0;
}

/**
 * Serialize r5mem_MemType_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_MemType(
							serdes_t *ser,
							const r5mem_MemType_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_MemType_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_MemType(
							des_buf_t *buf,
							r5mem_MemType_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (r5mem_MemType_t)(*ptr);
	return 0;
}

/**
 * Serialize r5mem_MasterId_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_MasterId(
							serdes_t *ser,
							const r5mem_MasterId_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_MasterId_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_MasterId(
							des_buf_t *buf,
							r5mem_MasterId_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (r5mem_MasterId_t)(*ptr);
	return 0;
}

/**
 * Serialize r5mem_UInt64Array_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_UInt64Array(
							serdes_t *ser,
							const r5mem_UInt64Array_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)in->data,
				in->size * sizeof(uint64_t), 8);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_UInt64Array_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_UInt64Array(
							des_buf_t *buf,
							r5mem_UInt64Array_t *out)
{
	uint32_t size = 0;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint64_t);
	out->data = (uint64_t *)alloc_data(buf, size, 8);
	if (size > 0 && !out->data)
		return -1;

	return 0;
}

/**
 * Serialize r5mem_driver_ipc_msg_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_r5mem_driver_ipc_msg(
							serdes_t *ser,
							const r5mem_driver_ipc_msg_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->cmd);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->mem_type);
	if (ret >= 0)
		ret = serialize_r5mem_UInt64Array(ser, &in->payload);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize r5mem_driver_ipc_msg_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_r5mem_driver_ipc_msg(
							des_buf_t *buf,
							r5mem_driver_ipc_msg_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->cmd);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->mem_type);
	if (ret >= 0)
		ret = deserialize_r5mem_UInt64Array(buf, &out->payload);

	return ret >= 0 ? 0 : -1;
}



#ifdef __cplusplus
}
#endif

#endif
