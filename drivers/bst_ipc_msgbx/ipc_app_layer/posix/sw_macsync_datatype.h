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

#ifndef SW_MACSYNC_DATATYPE_H
#define SW_MACSYNC_DATATYPE_H

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
struct _sw_macsync_MyArray_t {
	uint8_t *data;
	uint32_t size;
};
#define sw_macsync_MyArray_t struct _sw_macsync_MyArray_t

enum _sw_macsync_ErrorEnum_t {
	SW_MACSYNC_NO_ERROR = 0,
	SW_MACSYNC_SERVER_FAIL = -1,
	SW_MACSYNC_ERROR_2 = -2,
	SW_MACSYNC_ERROR_3 = -3
};
#define sw_macsync_ErrorEnum_t enum _sw_macsync_ErrorEnum_t

// constants


// type serialize / deserialize functions
/**
 * Serialize sw_macsync_MyArray_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_sw_macsync_MyArray(
							serdes_t *ser,
							const sw_macsync_MyArray_t *in)
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
 * Deserialize sw_macsync_MyArray_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_sw_macsync_MyArray(
							des_buf_t *buf,
							sw_macsync_MyArray_t *out)
{
	// int32_t ret = 0;
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
	// return ret >= 0 ? 0 : -1;
}

/**
 * Serialize sw_macsync_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_sw_macsync_ErrorEnum(
							serdes_t *ser,
							const sw_macsync_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize sw_macsync_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_sw_macsync_ErrorEnum(
							des_buf_t *buf,
							sw_macsync_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (sw_macsync_ErrorEnum_t)(*ptr);
	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
