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

#ifndef BST_TOUCH_DATATYPE_H
#define BST_TOUCH_DATATYPE_H

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
enum _bst_touch_ErrorEnum_t {
	BST_TOUCH_NO_ERROR = 0,
	BST_TOUCH_SERVER_FAIL = -1,
	BST_TOUCH_ERROR_2 = -2,
	BST_TOUCH_ERROR_3 = -3,
	BST_TOUCH_ERROR_COMMON = 120,
	BST_TOUCH_ERROR_SCREEN_ID_INVALID = 128,
	BST_TOUCH_ERROR_SCREEN_NUM_OVERLIMIT = 129,
	BST_TOUCH_ERROR_CLIENT_NUM_OVERLIMIT,
	BST_TOUCH_ERROR_RESEVERD
};
#define bst_touch_ErrorEnum_t enum _bst_touch_ErrorEnum_t

enum _bst_touch_MacroEnum_t {
	BST_TOUCH_MAX_SCREEN = 5,
	BST_TOUCH_MAX_CLIENT = 10,
	BST_TOUCH_MAX_POINT_NUM = 10,
	BST_TOUCH_SCREENID_LVDS0 = 9,
	BST_TOUCH_SCREENID_LVDS1 = 10,
	BST_TOUCH_SCREENID_DSI0 = 7,
	BST_TOUCH_SCREENID_DSI1 = 8,
	BST_TOUCH_SCREENID_EDP = 6,
	BST_TOUCH_VENDOR_ID_SYNA = 1,
	BST_TOUCH_VENDOR_ID_HIMAX = 2,
	BST_TOUCH_PRODUCT_ID_SYNA_DEF = 1,
	BST_TOUCH_PRODUCT_ID_HIMAX_DEF = 1,
	BST_TOUCH_PRODUCT_ID_HIMAX_83192A = 2,
	BST_TOUCH_PRODUCT_ID_HIMAX_83192D = 3,
	BST_TOUCH_PRODUCT_ID_HIMAX_83193A = 4,
	BST_TOUCH_PRODUCT_ID_HIMAX_83180A = 5,
	BST_TOUCH_VERSION_ID_SYNA_DEF = 1,
	BST_TOUCH_VERSION_ID_HIMAX_DEF = 1
};
#define bst_touch_MacroEnum_t enum _bst_touch_MacroEnum_t

enum _bst_touch_TransportProtoEnum_t {
	BST_TOUCH_TRANSPORT_PROTOTYPE_FULL = 0,
	BST_TOUCH_TRANSPORT_PROTOTYPE_CHANGE
};
#define bst_touch_TransportProtoEnum_t enum _bst_touch_TransportProtoEnum_t
typedef uint8_t bst_touch_UInt8Array32_t[32];

struct _bst_touch_hw_info_t {
	uint32_t screen_id;
	uint16_t vendor;
	uint16_t product;
	uint16_t versions;
	uint16_t x_min;
	uint16_t x_max;
	uint16_t y_min;
	uint16_t y_max;
	uint16_t pressure_min;
	uint16_t pressure_max;
	uint16_t touch_num_max;
	uint8_t transport_proto;
	uint8_t connected;
	bst_touch_UInt8Array32_t name;
	bst_touch_UInt8Array32_t phys;
};
#define bst_touch_hw_info_t struct _bst_touch_hw_info_t

struct _bst_touch_request_info_t {
	uint32_t screen_id;
	uint32_t shmem_size;
	uint64_t shmem_paddr;
};
#define bst_touch_request_info_t struct _bst_touch_request_info_t

struct _bst_touch_calibration_info_t {
	uint32_t a_factor;
	uint32_t b_factor;
	uint32_t c_factor;
	uint32_t d_factor;
	uint32_t e_factor;
	uint32_t f_factor;
	uint32_t s_factor;
};
#define bst_touch_calibration_info_t struct _bst_touch_calibration_info_t

struct _bst_touch_point_data_t {
	uint16_t pressure;
	int16_t x_pos;
	int16_t y_pos;
	uint16_t distance;
	uint16_t orientation;
	uint16_t touch_major;
	uint16_t touch_minor;
	uint16_t width_major;
	uint16_t width_minor;
	uint8_t tool_type;
	uint8_t tracking_id;
};
#define bst_touch_point_data_t struct _bst_touch_point_data_t
typedef bst_touch_point_data_t bst_touch_point_dataArray10_t[10];

struct _bst_touch_point_info_t {
	uint32_t screen_id;
	uint16_t slots;
	bst_touch_point_dataArray10_t point_lists;
};
#define bst_touch_point_info_t struct _bst_touch_point_info_t

// constants


// type serialize / deserialize functions
/**
 * Serialize bst_touch_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_ErrorEnum(
							serdes_t *ser,
							const bst_touch_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_ErrorEnum(
							des_buf_t *buf,
							bst_touch_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (bst_touch_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize bst_touch_MacroEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_MacroEnum(
							serdes_t *ser,
							const bst_touch_MacroEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_MacroEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_MacroEnum(
							des_buf_t *buf,
							bst_touch_MacroEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (bst_touch_MacroEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize bst_touch_TransportProtoEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_TransportProtoEnum(
							serdes_t *ser,
							const bst_touch_TransportProtoEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_TransportProtoEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_TransportProtoEnum(
							des_buf_t *buf,
							bst_touch_TransportProtoEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (bst_touch_TransportProtoEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize bst_touch_UInt8Array32_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_UInt8Array32(
							serdes_t *ser,
							const bst_touch_UInt8Array32_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				32 * sizeof(uint8_t), 1);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_UInt8Array32_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_UInt8Array32(
							des_buf_t *buf,
							bst_touch_UInt8Array32_t **out)
{
	uint32_t size = 32 * sizeof(uint8_t);

	if (!out || !buf)
		return -1;

	*out = (bst_touch_UInt8Array32_t *)alloc_data(buf, size, 1);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_hw_info_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_hw_info(
							serdes_t *ser,
							const bst_touch_hw_info_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(bst_touch_hw_info_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_hw_info_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_hw_info(
							des_buf_t *buf,
							bst_touch_hw_info_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (bst_touch_hw_info_t *)alloc_data(buf, sizeof(bst_touch_hw_info_t), 4);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_request_info_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_request_info(
							serdes_t *ser,
							const bst_touch_request_info_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(bst_touch_request_info_t), 8);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_request_info_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_request_info(
							des_buf_t *buf,
							bst_touch_request_info_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (bst_touch_request_info_t *)alloc_data(buf, sizeof(bst_touch_request_info_t), 8);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_calibration_info_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_calibration_info(
							serdes_t *ser,
							const bst_touch_calibration_info_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(bst_touch_calibration_info_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_calibration_info_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_calibration_info(
							des_buf_t *buf,
							bst_touch_calibration_info_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (bst_touch_calibration_info_t *)alloc_data(buf, sizeof(bst_touch_calibration_info_t), 4);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_point_data_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_point_data(
							serdes_t *ser,
							const bst_touch_point_data_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(bst_touch_point_data_t), 2);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_point_data_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_point_data(
							des_buf_t *buf,
							bst_touch_point_data_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (bst_touch_point_data_t *)alloc_data(buf, sizeof(bst_touch_point_data_t), 2);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_point_dataArray10_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_point_dataArray10(
							serdes_t *ser,
							const bst_touch_point_dataArray10_t *in)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_align(ser, (uint8_t *)(*in),
				10 * sizeof(bst_touch_point_data_t), 2);

	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_point_dataArray10_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_point_dataArray10(
							des_buf_t *buf,
							bst_touch_point_dataArray10_t **out)
{
	uint32_t size = 10 * sizeof(bst_touch_point_data_t);

	if (!out || !buf)
		return -1;

	*out = (bst_touch_point_dataArray10_t *)alloc_data(buf, size, 2);
	if (!*out)
		return -1;

	return 0;
}

/**
 * Serialize bst_touch_point_info_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_bst_touch_point_info(
							serdes_t *ser,
							const bst_touch_point_info_t *in)
{
	int32_t ret = 0;

	ret = ipc_ser_put_align(ser, (const uint8_t *)in,
				sizeof(bst_touch_point_info_t), 4);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize bst_touch_point_info_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_bst_touch_point_info(
							des_buf_t *buf,
							bst_touch_point_info_t **out)
{
	if (!out || !buf)
		return -1;

	*out = (bst_touch_point_info_t *)alloc_data(buf, sizeof(bst_touch_point_info_t), 4);
	if (!*out)
		return -1;

	return 0;
}



#ifdef __cplusplus
}
#endif

#endif
