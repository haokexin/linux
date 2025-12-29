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

#ifndef SERVICE_DATATYPE_H
#define SERVICE_DATATYPE_H

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
enum _service_ErrorEnum_t {
	SERVICE_NO_ERROR = 0,
	SERVICE_SERVER_FAIL = -1,
	SERVICE_ERROR_TRNG = -2,
	SERVICE_ERROR_HASH = -3,
	SERVICE_ERROR_HMAC = -4,
	SERVICE_ERROR_SM3 = -5,
	SERVICE_ERROR_CRC32 = -6,
	SERVICE_ERROR_AES = -7,
	SERVICE_ERROR_SM4 = -8,
	SERVICE_ERROR_RSA_SIGN_OR_VERIFY = -9,
	SERVICE_ERROR_ECC_SIGN_OR_VERIFY = -10,
	SERVICE_ERROR_SM2_SIGN_OR_VERIFY = -11,
	SERVICE_ERROR_SEIP_KEY_SET = -12,
	SERVICE_ERROR_SEIP_KEY_STATUS = -13,
	SERVICE_ERROR_LIFE_CYCLE_SET = -14,
	SERVICE_ERROR_SRAM_KEY_SET = -15,
	SERVICE_ERROR_KEY_EX_OP = -16,
	SERVICE_ERROR_RSA_EN = -17,
	SERVICE_ERROR_RSA_DE = -18,
	SERVICE_ERROR_PUB_KEY = -19,
	SERVICE_ERROR_KEY_NOT_SET = -20,
	SERVICE_ERROR_OTP_SET = -21,
	SERVICE_ERROR_KEY_ID = -22,
	SERVICE_ERROR_MALLOC = -23,
	SERVICE_ERROR_CBC_MAC = -24,
	SERVICE_ERROR_CMAC = -25,
	SERVICE_ERROR_OTP_INFO_INVALID = -26,
	SERVICE_ERROR_AUTH_FAIL = -27,
	SERVICE_ERROR_CONDITION = -28,
	SERVICE_ERROR_SEQUENCE = -29,
	SERVICE_ERROR_OUT_OF_RANGE = -30,
	SERVICE_ERROR_SRV_NOT_SUPPORT = -31,
	SERVICE_ERROR_TIMEOUT = -32,
	SERVICE_ERROR_CARRY_INFO = -33,
	SERVICE_ERROR_GEN_KEY = -34
};
#define service_ErrorEnum_t enum _service_ErrorEnum_t

enum _service_AesEnum_t {
	SERVICE_AES_128_ECB = 48,
	SERVICE_AES_128_CBC = 49,
	SERVICE_AES_128_CFB = 50,
	SERVICE_AES_128_OFB = 51,
	SERVICE_AES_128_CTR = 52,
	SERVICE_AES_128_XTS = 53,
	SERVICE_AES_192_ECB = 56,
	SERVICE_AES_192_CBC = 57,
	SERVICE_AES_192_CFB = 58,
	SERVICE_AES_192_OFB = 59,
	SERVICE_AES_192_CTR = 60,
	SERVICE_AES_192_XTS = 61,
	SERVICE_AES_256_ECB = 64,
	SERVICE_AES_256_CBC = 65,
	SERVICE_AES_256_CFB = 66,
	SERVICE_AES_256_OFB = 67,
	SERVICE_AES_256_CTR = 68,
	SERVICE_AES_256_XTS = 69
};
#define service_AesEnum_t enum _service_AesEnum_t

enum _service_Sm4Enum_t {
	SERVICE_SM4_ECB = 72,
	SERVICE_SM4_CBC = 73,
	SERVICE_SM4_CFB = 74,
	SERVICE_SM4_OFB = 75,
	SERVICE_SM4_CTR = 76,
	SERVICE_SM4_XTS = 77
};
#define service_Sm4Enum_t enum _service_Sm4Enum_t

enum _service_GcmEnum_t {
	SERVICE_AES_128_GCM = 54,
	SERVICE_AES_192_GCM = 62,
	SERVICE_AES_256_GCM = 70,
	SERVICE_SM4_GCM = 78
};
#define service_GcmEnum_t enum _service_GcmEnum_t

enum _service_CcmEnum_t {
	SERVICE_AES_128_CCM = 55,
	SERVICE_AES_192_CCM = 63,
	SERVICE_AES_256_CCM = 71,
	SERVICE_SM4_CCM = 79
};
#define service_CcmEnum_t enum _service_CcmEnum_t

enum _service_HashEnum_t {
	SERVICE_HASH_SHA256 = 1,
	SERVICE_HASH_SHA224 = 4,
	SERVICE_HASH_SHA1 = 7
};
#define service_HashEnum_t enum _service_HashEnum_t

enum _service_HmacEnum_t {
	SERVICE_HMAC_SHA256 = 14,
	SERVICE_HMAC_SHA224 = 17,
	SERVICE_HMAC_SHA1 = 20
};
#define service_HmacEnum_t enum _service_HmacEnum_t

enum _service_CmacEnum_t {
	SERVICE_AES_128_CMAC = 29,
	SERVICE_AES_192_CMAC = 30,
	SERVICE_AES_256_CMAC = 31,
	SERVICE_SM4_CMAC = 32
};
#define service_CmacEnum_t enum _service_CmacEnum_t

enum _service_CbcMacEnum_t {
	SERVICE_AES_128_CBC_MAC = 25,
	SERVICE_AES_192_CBC_MAC = 26,
	SERVICE_AES_256_CBC_MAC = 27,
	SERVICE_SM4_CBC_MAC = 28
};
#define service_CbcMacEnum_t enum _service_CbcMacEnum_t

enum _service_CryptoEnum_t {
	SERVICE_ENCRYPT = 0,
	SERVICE_DECRYPT = 1
};
#define service_CryptoEnum_t enum _service_CryptoEnum_t

enum _service_SignEnum_t {
	SERVICE_SIGN = 0,
	SERVICE_VERIFY = 1
};
#define service_SignEnum_t enum _service_SignEnum_t

enum _service_OtpStatus_t {
	SERVICE_OTP_STATUS_INIT = 0,
	SERVICE_OTP_STATUS_SET = 1
};
#define service_OtpStatus_t enum _service_OtpStatus_t

enum _service_SramKeyId_t {
	SERVICE_SRAM_KEY_0 = 305397760,
	SERVICE_SRAM_KEY_1,
	SERVICE_SRAM_KEY_2,
	SERVICE_SRAM_KEY_3,
	SERVICE_SRAM_KEY_4,
	SERVICE_SRAM_KEY_5,
	SERVICE_SRAM_KEY_6,
	SERVICE_SRAM_KEY_7,
	SERVICE_SRAM_KEY_8,
	SERVICE_SRAM_KEY_9,
	SERVICE_SRAM_KEY_10,
	SERVICE_SRAM_KEY_11,
	SERVICE_SRAM_KEY_12,
	SERVICE_SRAM_KEY_13,
	SERVICE_SRAM_KEY_14,
	SERVICE_SRAM_KEY_15,
	SERVICE_SRAM_KEY_16 = 2882338817,
	SERVICE_SRAM_KEY_17
};
#define service_SramKeyId_t enum _service_SramKeyId_t

enum _service_LifeCycleEnum_t {
	SERVICE_LC_CHIP_TEST = 0,
	SERVICE_LC_CHIP_DEV = 1,
	SERVICE_LC_DEVICE_DESIGN = 2,
	SERVICE_LC_DEVICE_USING = 3,
	SERVICE_LC_CHIP_FAIL = 4,
	SERVICE_LC_CHIP_END = 5
};
#define service_LifeCycleEnum_t enum _service_LifeCycleEnum_t

enum _service_OtpUseOptionEnum_t {
	SERVICE_OTP_USE_REQUEST_RB = 0,
	SERVICE_OTP_USE_HANDLE_TOKEN = 1
};
#define service_OtpUseOptionEnum_t enum _service_OtpUseOptionEnum_t

enum _service_OtpInfoEnum_t {
	SERVICE_OTP_INFO_CHIP_ID_DERIVATIVE = 0,
	SERVICE_OTP_INFO_ROLLBACK_DATA = 1,
	SERVICE_OTP_INFO_VER_CTRL_INFO = 2,
	SERVICE_OTP_INFO_IC_TYPE_INFO = 3,
	SERVICE_OTP_INFO_PARTIAL_GOOD_INFO = 4
};
#define service_OtpInfoEnum_t enum _service_OtpInfoEnum_t

enum _service_BinVerifyOpEnum_t {
	SERVICE_BVO_VERIFY_PUB_KEY = 0,
	SERVICE_BVO_VERIFY_BIN = 1,
	SERVICE_BVO_UPDATE_HMAC_KEY = 2
};
#define service_BinVerifyOpEnum_t enum _service_BinVerifyOpEnum_t

struct _service_UInt8Array_t {
	uint8_t *data;
	uint32_t size;
};
#define service_UInt8Array_t struct _service_UInt8Array_t

// constants


// type serialize / deserialize functions
/**
 * Serialize service_ErrorEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_ErrorEnum(
							serdes_t *ser,
							const service_ErrorEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_ErrorEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_ErrorEnum(
							des_buf_t *buf,
							service_ErrorEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_ErrorEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_AesEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_AesEnum(
							serdes_t *ser,
							const service_AesEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_AesEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_AesEnum(
							des_buf_t *buf,
							service_AesEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_AesEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_Sm4Enum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_Sm4Enum(
							serdes_t *ser,
							const service_Sm4Enum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_Sm4Enum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_Sm4Enum(
							des_buf_t *buf,
							service_Sm4Enum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_Sm4Enum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_GcmEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_GcmEnum(
							serdes_t *ser,
							const service_GcmEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_GcmEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_GcmEnum(
							des_buf_t *buf,
							service_GcmEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_GcmEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_CcmEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_CcmEnum(
							serdes_t *ser,
							const service_CcmEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_CcmEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_CcmEnum(
							des_buf_t *buf,
							service_CcmEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_CcmEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_HashEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_HashEnum(
							serdes_t *ser,
							const service_HashEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_HashEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_HashEnum(
							des_buf_t *buf,
							service_HashEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_HashEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_HmacEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_HmacEnum(
							serdes_t *ser,
							const service_HmacEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_HmacEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_HmacEnum(
							des_buf_t *buf,
							service_HmacEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_HmacEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_CmacEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_CmacEnum(
							serdes_t *ser,
							const service_CmacEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_CmacEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_CmacEnum(
							des_buf_t *buf,
							service_CmacEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_CmacEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_CbcMacEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_CbcMacEnum(
							serdes_t *ser,
							const service_CbcMacEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_CbcMacEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_CbcMacEnum(
							des_buf_t *buf,
							service_CbcMacEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_CbcMacEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_CryptoEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_CryptoEnum(
							serdes_t *ser,
							const service_CryptoEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_CryptoEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_CryptoEnum(
							des_buf_t *buf,
							service_CryptoEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_CryptoEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_SignEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_SignEnum(
							serdes_t *ser,
							const service_SignEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_SignEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_SignEnum(
							des_buf_t *buf,
							service_SignEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_SignEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_OtpStatus_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_OtpStatus(
							serdes_t *ser,
							const service_OtpStatus_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_OtpStatus_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_OtpStatus(
							des_buf_t *buf,
							service_OtpStatus_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_OtpStatus_t)(*ptr);
	return 0;
}

/**
 * Serialize service_SramKeyId_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_SramKeyId(
							serdes_t *ser,
							const service_SramKeyId_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_SramKeyId_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_SramKeyId(
							des_buf_t *buf,
							service_SramKeyId_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_SramKeyId_t)(*ptr);
	return 0;
}

/**
 * Serialize service_LifeCycleEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_LifeCycleEnum(
							serdes_t *ser,
							const service_LifeCycleEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_LifeCycleEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_LifeCycleEnum(
							des_buf_t *buf,
							service_LifeCycleEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_LifeCycleEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_OtpUseOptionEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_OtpUseOptionEnum(
							serdes_t *ser,
							const service_OtpUseOptionEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_OtpUseOptionEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_OtpUseOptionEnum(
							des_buf_t *buf,
							service_OtpUseOptionEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_OtpUseOptionEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_OtpInfoEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_OtpInfoEnum(
							serdes_t *ser,
							const service_OtpInfoEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_OtpInfoEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_OtpInfoEnum(
							des_buf_t *buf,
							service_OtpInfoEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_OtpInfoEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_BinVerifyOpEnum_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_BinVerifyOpEnum(
							serdes_t *ser,
							const service_BinVerifyOpEnum_t *in)
{
	int32_t ret = 0;
	uint32_t val = (uint32_t)(*in);

	ret = ipc_ser_put_32(ser, (uint32_t *)&val);
	return ret >= 0 ? 0 : -1;
}

/**
 * Deserialize service_BinVerifyOpEnum_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_BinVerifyOpEnum(
							des_buf_t *buf,
							service_BinVerifyOpEnum_t *out)
{
	uint32_t *ptr = NULL;

	if (!out || !buf)
		return -1;

	ptr = (uint32_t *)alloc_data(buf, 4, 4);
	if (!ptr)
		return -1;
	*out = (service_BinVerifyOpEnum_t)(*ptr);
	return 0;
}

/**
 * Serialize service_UInt8Array_t.
 *
 * @param ser Pointer to serdes_t, which stores the serialized data.
 * @param in The input data to be serialized.
 * @return 0 if success, negative if fail.
 */
static inline int32_t serialize_service_UInt8Array(
							serdes_t *ser,
							const service_UInt8Array_t *in)
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
 * Deserialize service_UInt8Array_t.
 *
 * @param des Pointer to serdes_t, which stores the serialized data.
 * @param out The output of deserialized data.
 * @param buf The buffer used to make the deserialized objects.
 * @return 0 if success, negative if fail.
 */
static inline int32_t deserialize_service_UInt8Array(
							des_buf_t *buf,
							service_UInt8Array_t *out)
{
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
}



#ifdef __cplusplus
}
#endif

#endif
