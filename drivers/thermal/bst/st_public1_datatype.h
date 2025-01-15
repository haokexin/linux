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
 * Generator Version: francaidl 3a7f767 msgbx_ipc 001bddd
 */

#ifndef ST_PUBLIC1_DATATYPE_H
#define ST_PUBLIC1_DATATYPE_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
enum _st_public1_ErrorEnum_t {
	ST_PUBLIC1_NO_ERROR = 0,
	ST_PUBLIC1_SERVER_FAIL = -1,
	ST_PUBLIC1_ERROR_2 = -2,
	ST_PUBLIC1_ERROR_3 = -3
};
#define st_public1_ErrorEnum_t enum _st_public1_ErrorEnum_t

struct _st_public1_qspi_cmd_head_t {
	uint32_t addr;
	uint32_t maxsize;
	uint32_t offset;
	uint32_t size;
	uint32_t bus_num;
	uint32_t flag;
};
#define st_public1_qspi_cmd_head_t struct _st_public1_qspi_cmd_head_t

// constants


// type serialize / deserialize functions
/**
 * Serialize st_public1_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_st_public1_ErrorEnum(
							serdes_t *ser,
							const st_public1_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize st_public1_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_st_public1_ErrorEnum(
							des_buf_t *buf,
							st_public1_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (st_public1_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize st_public1_qspi_cmd_head_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_st_public1_qspi_cmd_head(
							serdes_t *ser,
							const st_public1_qspi_cmd_head_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(st_public1_qspi_cmd_head_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize st_public1_qspi_cmd_head_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_st_public1_qspi_cmd_head(
							des_buf_t *buf,
							st_public1_qspi_cmd_head_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (st_public1_qspi_cmd_head_t *)alloc_data(buf, sizeof(st_public1_qspi_cmd_head_t), 4);
	if (!*out)
		return -1;

	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
