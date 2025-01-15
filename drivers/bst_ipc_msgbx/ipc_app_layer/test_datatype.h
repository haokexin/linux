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
	TEST_NO_ERROR = 0,
	TEST_SERVER_FAIL = -1,
	TEST_ERROR_2 = -2,
	TEST_ERROR_3 = -3
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
static const double pi = 3.141500;

// type serialize / deserialize functions
/**
 * Serialize test_MyArray_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_test_MyArray(
							serdes_t *ser,
							const test_MyArray_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, &in->size);

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)in->data,
				in->size * sizeof(uint16_t),
				sizeof(uint16_t) > 4 ? 8 : 4);

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
static inline int32_t deserialize_test_MyArray(
							des_buf_t *buf,
							test_MyArray_t *out)
{
	int32_t ret = 0;
	uint32_t size = 0;
	uint32_t align = sizeof(uint16_t) > 4 ? 8 : 4;
	uint32_t *size_ptr = NULL;

	if (!out || !buf)
		return -1;

	size_ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!size_ptr)
		return -1;

	out->size = *size_ptr;
	size = out->size * sizeof(uint16_t);
	out->data = (uint16_t *)alloc_data(buf, size, align);
	if (!out->data)
		return -1;

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize test_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_test_ErrorEnum(
							serdes_t *ser,
							const test_ErrorEnum_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_32(ser, (uint32_t *)in);
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
static inline int32_t deserialize_test_ErrorEnum(
							des_buf_t *buf,
							test_ErrorEnum_t *out)
{
	test_ErrorEnum_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (test_ErrorEnum_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}

/**
 * Serialize test_MyStruct_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_test_MyStruct(
							serdes_t *ser,
							const test_MyStruct_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&in->m1);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&in->m2);
	if (ret >= 0)
		ret = serialize_test_MyArray(ser, &in->m3);
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
static inline int32_t deserialize_test_MyStruct(
							des_buf_t *buf,
							test_MyStruct_t *out)
{
	int32_t ret = 0;

	if (!out || !buf)
		return -1;

	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&out->m1);
	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&out->m2);
	if (ret >= 0)
		ret = deserialize_test_MyArray(buf, &out->m3);
	if (ret >= 0)
		ret = deserialize_string(buf, &out->m4);
	if (ret >= 0)
		ret = deserialize_byte_buffer(buf, &out->m5);

	return ret >= 0 ? 0 : -1;
}

/**
 * Serialize test_MyUnion_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 * @note union size greater than 64 bit is NOT supported, return -1.
 */
static inline int32_t serialize_test_MyUnion(
							serdes_t *ser,
							const test_MyUnion_t *in)
{
	int32_t ret = 0;

	if (sizeof(test_MyUnion_t) > 8)
		return -1;

	ret = ipc_ser_put_align(ser, (uint8_t *)in, sizeof(test_MyUnion_t), sizeof(test_MyUnion_t));
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize test_MyUnion_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 * @note union size greater than 64 bit is NOT supported, return -1.
 */
static inline int32_t deserialize_test_MyUnion(
							des_buf_t *buf,
							test_MyUnion_t *out)
{
	test_MyUnion_t *ptr = NULL;

	if (!out || !buf || sizeof(test_MyUnion_t) > 8)
		return -1;

	ptr = (test_MyUnion_t *)alloc_data(buf, sizeof(test_MyUnion_t), sizeof(test_MyUnion_t));
	if (!ptr)
		return -1;
	*out = *ptr;
	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
