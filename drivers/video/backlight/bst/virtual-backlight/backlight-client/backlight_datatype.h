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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 797e374 msgbx_ipc c468e33
 */

#ifndef BACKLIGHT_DATATYPE_H
#define BACKLIGHT_DATATYPE_H

#include "ipc_app_common.h"

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
