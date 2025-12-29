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

#ifndef SDEMMC_DATATYPE_H
#define SDEMMC_DATATYPE_H

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
enum _sdemmc_ErrorEnum_t {
	SDEMMC_NO_ERROR = 0,
	SDEMMC_SERVER_FAIL = -1,
	SDEMMC_ERROR_2 = -2,
	SDEMMC_ERROR_3 = -3
};
#define sdemmc_ErrorEnum_t enum _sdemmc_ErrorEnum_t
typedef uint32_t sdemmc_UInt32Array4_t[4];

// constants


// type serialize / deserialize functions
/**
 * Serialize sdemmc_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_sdemmc_ErrorEnum(
							serdes_t *ser,
							const sdemmc_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize sdemmc_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_sdemmc_ErrorEnum(
							des_buf_t *buf,
							sdemmc_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (sdemmc_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize sdemmc_UInt32Array4_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_sdemmc_UInt32Array4(
							serdes_t *ser,
							const sdemmc_UInt32Array4_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				4 * sizeof(uint32_t), 4);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize sdemmc_UInt32Array4_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_sdemmc_UInt32Array4(
							des_buf_t *buf,
							sdemmc_UInt32Array4_t **out)
{
	uint32_t size = 4 * sizeof(uint32_t);

	if (!out || !buf)
		return -1;

	*out = (sdemmc_UInt32Array4_t *)alloc_data(buf, size, 4);
	if (!*out)
		return -1;

	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
