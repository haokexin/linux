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

#ifndef BACKLIGHT_DATATYPE_H
#define BACKLIGHT_DATATYPE_H

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
enum _backlight_ErrorEnum_t {
	BACKLIGHT_NO_ERROR = 0,
	BACKLIGHT_SERVER_FAIL = -1
};
#define backlight_ErrorEnum_t enum _backlight_ErrorEnum_t

struct _backlight_UInt32Array_t {
	uint32_t *data;
	uint32_t size;
};
#define backlight_UInt32Array_t struct _backlight_UInt32Array_t

struct _backlight_virt_bl_msg_t {
	uint32_t msg_len;
	uint32_t cmd_id;
	backlight_UInt32Array_t content;
};
#define backlight_virt_bl_msg_t struct _backlight_virt_bl_msg_t

// constants


// type serialize / deserialize functions
/**
 * Serialize backlight_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_backlight_ErrorEnum(
							serdes_t *ser,
							const backlight_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize backlight_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_backlight_ErrorEnum(
							des_buf_t *buf,
							backlight_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (backlight_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize backlight_UInt32Array_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_backlight_UInt32Array(
							serdes_t *ser,
							const backlight_UInt32Array_t *in)
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
 * Deserialize backlight_UInt32Array_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_backlight_UInt32Array(
							des_buf_t *buf,
							backlight_UInt32Array_t *out)
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
 * Serialize backlight_virt_bl_msg_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_backlight_virt_bl_msg(
							serdes_t *ser,
							const backlight_virt_bl_msg_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->msg_len);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&in->cmd_id);
	if (ret >= 0)
		ret = serialize_backlight_UInt32Array(ser, &in->content);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize backlight_virt_bl_msg_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_backlight_virt_bl_msg(
							des_buf_t *buf,
							backlight_virt_bl_msg_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->msg_len);
	if (ret >= 0)
		ret = deserialize_32(buf, (uint32_t *)&out->cmd_id);
	if (ret >= 0)
		ret = deserialize_backlight_UInt32Array(buf, &out->content);

	return ret >= 0 ? 0 : -1;
}



#ifdef __cplusplus
}
#endif

#endif
