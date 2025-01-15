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

/* This file is auto generated for message box v0.2.2.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: f27fcbb
 */

#ifndef TEST_DATATYPE_H
#define TEST_DATATYPE_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C" {
#endif

// user defined types
struct _test_MyArray_t {
	uint16_t *data;
	uint32_t size;
};
#define test_MyArray_t struct _test_MyArray_t

enum _test_ErrorEnum_t {
	NO_ERROR = 0,
	SERVER_FAIL = -1,
	ERROR_2 = -2,
	ERROR_3 = -3
};
#define test_ErrorEnum_t enum _test_ErrorEnum_t

struct _test_MyStruct_t {
	uint8_t m1;
	bool m2;
	test_MyArray_t m3;
	char *m4;
	byte_buffer_t m5;
};
#define test_MyStruct_t struct _test_MyStruct_t

union _test_MyUnion_t {
	uint8_t m1;
	uint16_t m2;
	uint32_t m3;
};
#define test_MyUnion_t union _test_MyUnion_t

// constants
static const bool b1 = true;
static const uint32_t MAX_COUNT = 10000;
static const uint16_t SOME_ID = 40971;
static const uint8_t BYTE_ME = 51;
static const char *foo = "bar";
static const double pi = 3.141500;

// type serialize / deserialize functions
/**
 * Serialize test_MyArray_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_MyArray(
							serdes_t *ser,
							const test_MyArray_t *in
							)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)&in->size, sizeof(uint32_t));

	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)in->data, in->size * sizeof(uint16_t));

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize test_MyArray_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_MyArray(
							serdes_t *des,
							test_MyArray_t *out,
							des_buf_t *buf
							)
{
	int32_t ret = 0;
	uint32_t size = 0;

	if (!des || !out || !buf)
		return -1;

	ret = ipc_des_get(des, (uint8_t *)&out->size, sizeof(uint32_t));
	if (ret < 0)
		return -1;

	size = out->size * sizeof(uint16_t);
	out->data = (uint16_t *)alloc_data(buf, size);
	if (!out->data)
		return -1;

	ret = ipc_des_get(des, (uint8_t *)out->data, size);

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize test_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_ErrorEnum(
							serdes_t *ser,
							const test_ErrorEnum_t *in
							)
{
	int32_t ret = 0;

	ret = ipc_ser_put(ser, (uint8_t *)in, sizeof(int32_t));
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize test_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_ErrorEnum(
							serdes_t *des,
							test_ErrorEnum_t *out,
							des_buf_t *buf
							)
{
	int32_t _out = 0;
	int32_t ret = 0;

	if (!des || !out || !buf)
		return -1;

	ret = ipc_des_get(des, (uint8_t *)&_out, sizeof(int32_t));
	*out = _out;
	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize test_MyStruct_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_MyStruct(
							serdes_t *ser,
							const test_MyStruct_t *in
							)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)&in->m1, sizeof(uint8_t));
	if (ret >= 0)
		ret = ipc_ser_put(ser, (uint8_t *)&in->m2, sizeof(bool));
	if (ret >= 0)
		ret = serialize_MyArray(ser, &in->m3);
	if (ret >= 0)
		ret = serialize_string(ser, in->m4);
	if (ret >= 0)
		ret = serialize_byte_buffer(ser, &in->m5);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize test_MyStruct_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_MyStruct(
							serdes_t *des,
							test_MyStruct_t *out,
							des_buf_t *buf
							)
{
	int32_t ret = 0;

	if (!des || !out || !buf)
		return -1;

	if (ret >= 0)
		ret = ipc_des_get(des, (uint8_t *)&out->m1, sizeof(uint8_t));
	if (ret >= 0)
		ret = ipc_des_get(des, (uint8_t *)&out->m2, sizeof(bool));
	if (ret >= 0)
		ret = deserialize_MyArray(des, &out->m3, buf);
	if (ret >= 0)
		ret = deserialize_string(des, &out->m4, buf);
	if (ret >= 0)
		ret = deserialize_byte_buffer(des, &out->m5, buf);

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize test_MyUnion_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_MyUnion(
							serdes_t *ser,
							const test_MyUnion_t *in
							)
{
	int32_t ret = 0;

	ret = ipc_ser_put(ser, (uint8_t *)in, sizeof(test_MyUnion_t));
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize test_MyUnion_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_MyUnion(
							serdes_t *des,
							test_MyUnion_t *out,
							des_buf_t *buf
							)
{
	int32_t ret = 0;

	if (!des || !out || !buf)
		return -1;

	ret = ipc_des_get(des, (uint8_t *)out, sizeof(test_MyUnion_t));
	return ret >= 0 ? 0 : -1;
}



#ifdef __cplusplus
}
#endif

#endif
