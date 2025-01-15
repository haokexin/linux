/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

/* This file is auto generated for message box v1.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 77a2400 msgbx_ipc eb42a92
 */

#ifndef SW_MACSYNC_DATATYPE_H
#define SW_MACSYNC_DATATYPE_H

#include <bst/ipc_app_common.h>

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
				in->size * sizeof(uint8_t),
				sizeof(uint8_t) > 4 ? 8 : 4);

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
	int32_t ret = 0;
	uint32_t size = 0;
	uint32_t align = sizeof(uint8_t) > 4 ? 8 : 4;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint8_t);
	out->data = (uint8_t *)alloc_data(buf, size, align);
	if (!out->data)
		return -1;

	return ret >= 0 ? 0 : -1;
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

	ret = ipc_ser_put_32(ser, (uint32_t *)in);
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
	sw_macsync_ErrorEnum_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (sw_macsync_ErrorEnum_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
