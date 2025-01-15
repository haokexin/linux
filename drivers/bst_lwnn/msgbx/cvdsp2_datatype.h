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

#ifndef CVDSP2_DATATYPE_H
#define CVDSP2_DATATYPE_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
enum _cvdsp2_ErrorEnum_t {
	CVDSP2_NO_ERROR = 0,
	CVDSP2_SERVER_FAIL = -1,
	CVDSP2_ERROR_PARA = -2,
	CVDSP2_ERROR_SYS = -3
};
#define cvdsp2_ErrorEnum_t enum _cvdsp2_ErrorEnum_t

// constants


// type serialize / deserialize functions
/**
 * Serialize cvdsp2_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_cvdsp2_ErrorEnum(
							serdes_t *ser,
							const cvdsp2_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize cvdsp2_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_cvdsp2_ErrorEnum(
							des_buf_t *buf,
							cvdsp2_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (cvdsp2_ErrorEnum_t)(*ptr);
	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
