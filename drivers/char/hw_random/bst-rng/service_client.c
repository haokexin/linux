// SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
/*
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

#include "service_client.h"

// macro definitions
#define CID SECURE_0
#define CCID 0
#define CID_MASK (0x1U << 24)
#define MAJOR 1U
#define MINOR 0U


#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_OTP_INFO 2U
#define CMD_METHOD_TRNG 3U
#define CMD_METHOD_HASH 4U
#define CMD_METHOD_HMAC 5U
#define CMD_METHOD_SM3 6U
#define CMD_METHOD_CRC32 7U
#define CMD_METHOD_AES 8U
#define CMD_METHOD_SM4 9U
#define CMD_METHOD_RSA_SIGN_OR_VERIFY 10U
#define CMD_METHOD_ECC_SIGN_OR_VERIFY 11U
#define CMD_METHOD_SM2_SIGN_OR_VERIFY 12U
#define CMD_METHOD_SEIP_KEY_STATUS 13U
#define CMD_METHOD_LIFE_CYCLE_STATUS 14U
#define CMD_METHOD_OTP_USE_WITH_AUTH 15U
#define CMD_METHOD_BIN_VERIFY 16U
#define CMD_METHOD_CBC_MAC 17U
#define CMD_METHOD_CMAC 18U
#define CMD_METHOD_SLT_METHOD 19U
#define CMD_METHOD_CONFIG_TZC400 20U

#define CMD_METHOD_SUB_HEARTBEAT 64U
#define CMD_METHOD_UNSUB_HEARTBEAT 65U
#define CMD_BROADCAST_HEARTBEAT 1U

// local variables
static com_client_data_t *s_data;
static service_client_ext_t *s_ext;

#ifndef IPC_RTE_BAREMETAL

struct _hello_out_t {
	DECL_SEM(sem)
	char **message;
	service_ErrorEnum_t *err;
};
#define hello_out_t struct _hello_out_t

struct _otp_info_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define otp_info_out_t struct _otp_info_out_t

struct _trng_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define trng_out_t struct _trng_out_t

struct _hash_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define hash_out_t struct _hash_out_t

struct _hmac_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define hmac_out_t struct _hmac_out_t

struct _sm3_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define sm3_out_t struct _sm3_out_t

struct _crc32_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define crc32_out_t struct _crc32_out_t

struct _aes_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define aes_out_t struct _aes_out_t

struct _sm4_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define sm4_out_t struct _sm4_out_t

struct _rsa_sign_or_verify_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define rsa_sign_or_verify_out_t struct _rsa_sign_or_verify_out_t

struct _ecc_sign_or_verify_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define ecc_sign_or_verify_out_t struct _ecc_sign_or_verify_out_t

struct _sm2_sign_or_verify_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define sm2_sign_or_verify_out_t struct _sm2_sign_or_verify_out_t

struct _seip_key_status_out_t {
	DECL_SEM(sem)
	service_OtpStatus_t *keyStatus;
	service_ErrorEnum_t *err;
};
#define seip_key_status_out_t struct _seip_key_status_out_t

struct _life_cycle_status_out_t {
	DECL_SEM(sem)
	service_LifeCycleEnum_t *lifeCycle;
	service_ErrorEnum_t *err;
};
#define life_cycle_status_out_t struct _life_cycle_status_out_t

struct _otp_use_with_auth_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define otp_use_with_auth_out_t struct _otp_use_with_auth_out_t

struct _bin_verify_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define bin_verify_out_t struct _bin_verify_out_t

struct _cbc_mac_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define cbc_mac_out_t struct _cbc_mac_out_t

struct _cmac_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define cmac_out_t struct _cmac_out_t

struct _slt_method_out_t {
	DECL_SEM(sem)
	uint32_t *reply_result;
	uint32_t *recv_bin_index;
	byte_buffer_t *result_descrption;
	service_ErrorEnum_t *err;
};
#define slt_method_out_t struct _slt_method_out_t

struct _config_tzc400_out_t {
	DECL_SEM(sem)
	service_ErrorEnum_t *err;
};
#define config_tzc400_out_t struct _config_tzc400_out_t

#endif
// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
	ipc_inf_version_t ret = { .major = MAJOR, .minor = MINOR };

	return ret;
}

// method

static inline int32_t serialize_hello(
				serdes_t *ser,
				const char *name
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_string(ser, name);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void hello_sync_callback(
				const char *message,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	hello_out_t *out = (hello_out_t *)ext;

	if (!out)
		return;
	*out->message = (char *)message;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_hello_sync(const char *name,
				char **message,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	hello_out_t out = {.message = message,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hello_registry, ser, s_ext->cid,
			CMD_METHOD_HELLO, hello_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->hello_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_hello_async(const char *name,
				service_hello_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_hello(ser, name);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hello_registry, ser, s_ext->cid,
			CMD_METHOD_HELLO, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hello_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_hello_callback_t cb = NULL;
	char *message = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->hello_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SERVICE_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_string(buf, &message);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (service_hello_callback_t)(reg->cb);
	if (cb)
		cb(message, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_otp_info(
				serdes_t *ser,
				const service_OtpInfoEnum_t infoType,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_OtpInfoEnum(ser, &infoType);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void otp_info_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	otp_info_out_t *out = (otp_info_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_otp_info_sync(const service_OtpInfoEnum_t infoType,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	otp_info_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_otp_info(ser, infoType, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->otp_info_registry, ser, s_ext->cid,
			CMD_METHOD_OTP_INFO, otp_info_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->otp_info_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_otp_info_async(const service_OtpInfoEnum_t infoType,
				const uint32_t outBuf,
				service_otp_info_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_otp_info(ser, infoType, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->otp_info_registry, ser, s_ext->cid,
			CMD_METHOD_OTP_INFO, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_otp_info_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_otp_info_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->otp_info_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_otp_info_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_trng(
				serdes_t *ser,
				const uint32_t trngLen,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&trngLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void trng_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	trng_out_t *out = (trng_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_trng_sync(const uint32_t trngLen,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	trng_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_trng(ser, trngLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->trng_registry, ser, s_ext->cid,
			CMD_METHOD_TRNG, trng_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->trng_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_trng_async(const uint32_t trngLen,
				const uint32_t outBuf,
				service_trng_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_trng(ser, trngLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->trng_registry, ser, s_ext->cid,
			CMD_METHOD_TRNG, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_trng_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_trng_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->trng_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_trng_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_hash(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const service_HashEnum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = serialize_service_HashEnum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void hash_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	hash_out_t *out = (hash_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_hash_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const service_HashEnum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	hash_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hash(ser, msgBuf, msgLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hash_registry, ser, s_ext->cid,
			CMD_METHOD_HASH, hash_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->hash_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_hash_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const service_HashEnum_t mode,
				const uint32_t outBuf,
				service_hash_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_hash(ser, msgBuf, msgLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hash_registry, ser, s_ext->cid,
			CMD_METHOD_HASH, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hash_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_hash_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->hash_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_hash_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_hmac(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_HmacEnum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyPara);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyLen);
	if (ret >= 0)
		ret = serialize_service_HmacEnum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void hmac_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	hmac_out_t *out = (hmac_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_hmac_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_HmacEnum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	hmac_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_hmac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hmac_registry, ser, s_ext->cid,
			CMD_METHOD_HMAC, hmac_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->hmac_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_hmac_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_HmacEnum_t mode,
				const uint32_t outBuf,
				service_hmac_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_hmac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->hmac_registry, ser, s_ext->cid,
			CMD_METHOD_HMAC, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_hmac_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_hmac_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->hmac_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_hmac_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_sm3(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void sm3_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	sm3_out_t *out = (sm3_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_sm3_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	sm3_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_sm3(ser, msgBuf, msgLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm3_registry, ser, s_ext->cid,
			CMD_METHOD_SM3, sm3_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->sm3_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_sm3_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf,
				service_sm3_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_sm3(ser, msgBuf, msgLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm3_registry, ser, s_ext->cid,
			CMD_METHOD_SM3, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_sm3_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_sm3_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->sm3_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_sm3_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_crc32(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void crc32_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	crc32_out_t *out = (crc32_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_crc32_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	crc32_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_crc32(ser, msgBuf, msgLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->crc32_registry, ser, s_ext->cid,
			CMD_METHOD_CRC32, crc32_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->crc32_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_crc32_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t outBuf,
				service_crc32_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_crc32(ser, msgBuf, msgLen, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->crc32_registry, ser, s_ext->cid,
			CMD_METHOD_CRC32, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_crc32_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_crc32_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->crc32_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_crc32_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_aes(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_AesEnum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&ivBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyPara);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = serialize_service_CryptoEnum(ser, &isDecrypt);
	if (ret >= 0)
		ret = serialize_service_AesEnum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void aes_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	aes_out_t *out = (aes_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_aes_sync(const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_AesEnum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	aes_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_aes(ser, msgBuf, ivBuf, keyPara, msgLen, isDecrypt, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->aes_registry, ser, s_ext->cid,
			CMD_METHOD_AES, aes_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->aes_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_aes_async(const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_AesEnum_t mode,
				const uint32_t outBuf,
				service_aes_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_aes(ser, msgBuf, ivBuf, keyPara, msgLen, isDecrypt, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->aes_registry, ser, s_ext->cid,
			CMD_METHOD_AES, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_aes_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_aes_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->aes_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_aes_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_sm4(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_Sm4Enum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&ivBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyPara);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = serialize_service_CryptoEnum(ser, &isDecrypt);
	if (ret >= 0)
		ret = serialize_service_Sm4Enum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void sm4_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	sm4_out_t *out = (sm4_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_sm4_sync(const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_Sm4Enum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	sm4_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_sm4(ser, msgBuf, ivBuf, keyPara, msgLen, isDecrypt, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm4_registry, ser, s_ext->cid,
			CMD_METHOD_SM4, sm4_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->sm4_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_sm4_async(const uint32_t msgBuf,
				const uint32_t ivBuf,
				const uint32_t keyPara,
				const uint32_t msgLen,
				const service_CryptoEnum_t isDecrypt,
				const service_Sm4Enum_t mode,
				const uint32_t outBuf,
				service_sm4_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_sm4(ser, msgBuf, ivBuf, keyPara, msgLen, isDecrypt, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm4_registry, ser, s_ext->cid,
			CMD_METHOD_SM4, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_sm4_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_sm4_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->sm4_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_sm4_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_rsa_sign_or_verify(
				serdes_t *ser,
				const service_SignEnum_t isVerify,
				const uint32_t msgBuf,
				const uint32_t eBuf,
				const uint32_t nBuf,
				const uint32_t dBuf,
				const uint32_t nBitLen,
				const uint32_t signBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_SignEnum(ser, &isVerify);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&eBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&nBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&dBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&nBitLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&signBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void rsa_sign_or_verify_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	rsa_sign_or_verify_out_t *out = (rsa_sign_or_verify_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_rsa_sign_or_verify_sync(const service_SignEnum_t isVerify,
				const uint32_t msgBuf,
				const uint32_t eBuf,
				const uint32_t nBuf,
				const uint32_t dBuf,
				const uint32_t nBitLen,
				const uint32_t signBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	rsa_sign_or_verify_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_rsa_sign_or_verify(ser, isVerify, msgBuf, eBuf, nBuf, dBuf, nBitLen, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->rsa_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_RSA_SIGN_OR_VERIFY, rsa_sign_or_verify_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->rsa_sign_or_verify_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_rsa_sign_or_verify_async(const service_SignEnum_t isVerify,
				const uint32_t msgBuf,
				const uint32_t eBuf,
				const uint32_t nBuf,
				const uint32_t dBuf,
				const uint32_t nBitLen,
				const uint32_t signBuf,
				service_rsa_sign_or_verify_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_rsa_sign_or_verify(ser, isVerify, msgBuf, eBuf, nBuf, dBuf, nBitLen, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->rsa_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_RSA_SIGN_OR_VERIFY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_rsa_sign_or_verify_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_rsa_sign_or_verify_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->rsa_sign_or_verify_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_rsa_sign_or_verify_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_ecc_sign_or_verify(
				serdes_t *ser,
				const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t eBitlen,
				const uint32_t keyBuf,
				const uint32_t signBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_SignEnum(ser, &isVerify);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&eBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&eBitlen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&signBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void ecc_sign_or_verify_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	ecc_sign_or_verify_out_t *out = (ecc_sign_or_verify_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_ecc_sign_or_verify_sync(const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t eBitlen,
				const uint32_t keyBuf,
				const uint32_t signBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	ecc_sign_or_verify_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_ecc_sign_or_verify(ser, isVerify, eBuf, eBitlen, keyBuf, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->ecc_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_ECC_SIGN_OR_VERIFY, ecc_sign_or_verify_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->ecc_sign_or_verify_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_ecc_sign_or_verify_async(const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t eBitlen,
				const uint32_t keyBuf,
				const uint32_t signBuf,
				service_ecc_sign_or_verify_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_ecc_sign_or_verify(ser, isVerify, eBuf, eBitlen, keyBuf, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->ecc_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_ECC_SIGN_OR_VERIFY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_ecc_sign_or_verify_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_ecc_sign_or_verify_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->ecc_sign_or_verify_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_ecc_sign_or_verify_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_sm2_sign_or_verify(
				serdes_t *ser,
				const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t keyBuf,
				const uint32_t signBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_SignEnum(ser, &isVerify);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&eBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&signBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void sm2_sign_or_verify_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	sm2_sign_or_verify_out_t *out = (sm2_sign_or_verify_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_sm2_sign_or_verify_sync(const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t keyBuf,
				const uint32_t signBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	sm2_sign_or_verify_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_sm2_sign_or_verify(ser, isVerify, eBuf, keyBuf, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm2_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_SM2_SIGN_OR_VERIFY, sm2_sign_or_verify_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->sm2_sign_or_verify_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_sm2_sign_or_verify_async(const service_SignEnum_t isVerify,
				const uint32_t eBuf,
				const uint32_t keyBuf,
				const uint32_t signBuf,
				service_sm2_sign_or_verify_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_sm2_sign_or_verify(ser, isVerify, eBuf, keyBuf, signBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->sm2_sign_or_verify_registry, ser, s_ext->cid,
			CMD_METHOD_SM2_SIGN_OR_VERIFY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_sm2_sign_or_verify_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_sm2_sign_or_verify_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->sm2_sign_or_verify_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_sm2_sign_or_verify_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_seip_key_status(
				serdes_t *ser,
				const uint32_t keyId
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyId);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void seip_key_status_sync_callback(
				const service_OtpStatus_t keyStatus,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	seip_key_status_out_t *out = (seip_key_status_out_t *)ext;

	if (!out)
		return;
	*out->keyStatus = keyStatus;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_seip_key_status_sync(const uint32_t keyId,
				service_OtpStatus_t *keyStatus,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	seip_key_status_out_t out = {.keyStatus = keyStatus,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_seip_key_status(ser, keyId);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->seip_key_status_registry, ser, s_ext->cid,
			CMD_METHOD_SEIP_KEY_STATUS, seip_key_status_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->seip_key_status_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_seip_key_status_async(const uint32_t keyId,
				service_seip_key_status_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_seip_key_status(ser, keyId);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->seip_key_status_registry, ser, s_ext->cid,
			CMD_METHOD_SEIP_KEY_STATUS, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_seip_key_status_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_seip_key_status_callback_t cb = NULL;
	service_OtpStatus_t keyStatus = 0;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->seip_key_status_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SERVICE_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_service_OtpStatus(buf, &keyStatus);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (service_seip_key_status_callback_t)(reg->cb);
	if (cb)
		cb(keyStatus, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void life_cycle_status_sync_callback(
				const service_LifeCycleEnum_t lifeCycle,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	life_cycle_status_out_t *out = (life_cycle_status_out_t *)ext;

	if (!out)
		return;
	*out->lifeCycle = lifeCycle;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_life_cycle_status_sync(service_LifeCycleEnum_t *lifeCycle,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	life_cycle_status_out_t out = {.lifeCycle = lifeCycle,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	// send request
	ret = send_request(data, s_ext->life_cycle_status_registry, ser, s_ext->cid,
			CMD_METHOD_LIFE_CYCLE_STATUS, life_cycle_status_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->life_cycle_status_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_life_cycle_status_async(service_life_cycle_status_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	// send request
	ret = send_request(data, s_ext->life_cycle_status_registry, ser, s_ext->cid,
			CMD_METHOD_LIFE_CYCLE_STATUS, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_life_cycle_status_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_life_cycle_status_callback_t cb = NULL;
	service_LifeCycleEnum_t lifeCycle = 0;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->life_cycle_status_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SERVICE_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_service_LifeCycleEnum(buf, &lifeCycle);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (service_life_cycle_status_callback_t)(reg->cb);
	if (cb)
		cb(lifeCycle, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_otp_use_with_auth(
				serdes_t *ser,
				const service_OtpUseOptionEnum_t op,
				const uint32_t cmdWordBuf,
				const uint32_t tokenBuf,
				const uint32_t rbBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_OtpUseOptionEnum(ser, &op);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&cmdWordBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&tokenBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&rbBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void otp_use_with_auth_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	otp_use_with_auth_out_t *out = (otp_use_with_auth_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_otp_use_with_auth_sync(const service_OtpUseOptionEnum_t op,
				const uint32_t cmdWordBuf,
				const uint32_t tokenBuf,
				const uint32_t rbBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	otp_use_with_auth_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_otp_use_with_auth(ser, op, cmdWordBuf, tokenBuf, rbBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->otp_use_with_auth_registry, ser, s_ext->cid,
			CMD_METHOD_OTP_USE_WITH_AUTH, otp_use_with_auth_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->otp_use_with_auth_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_otp_use_with_auth_async(const service_OtpUseOptionEnum_t op,
				const uint32_t cmdWordBuf,
				const uint32_t tokenBuf,
				const uint32_t rbBuf,
				service_otp_use_with_auth_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_otp_use_with_auth(ser, op, cmdWordBuf, tokenBuf, rbBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->otp_use_with_auth_registry, ser, s_ext->cid,
			CMD_METHOD_OTP_USE_WITH_AUTH, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_otp_use_with_auth_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_otp_use_with_auth_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->otp_use_with_auth_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_otp_use_with_auth_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_bin_verify(
				serdes_t *ser,
				const service_BinVerifyOpEnum_t op,
				const uint32_t inBuff,
				const uint32_t length,
				const uint32_t signBuf,
				const uint32_t resultBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = serialize_service_BinVerifyOpEnum(ser, &op);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&inBuff);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&length);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&signBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&resultBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void bin_verify_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	bin_verify_out_t *out = (bin_verify_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_bin_verify_sync(const service_BinVerifyOpEnum_t op,
				const uint32_t inBuff,
				const uint32_t length,
				const uint32_t signBuf,
				const uint32_t resultBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	bin_verify_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_bin_verify(ser, op, inBuff, length, signBuf, resultBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->bin_verify_registry, ser, s_ext->cid,
			CMD_METHOD_BIN_VERIFY, bin_verify_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->bin_verify_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_bin_verify_async(const service_BinVerifyOpEnum_t op,
				const uint32_t inBuff,
				const uint32_t length,
				const uint32_t signBuf,
				const uint32_t resultBuf,
				service_bin_verify_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_bin_verify(ser, op, inBuff, length, signBuf, resultBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->bin_verify_registry, ser, s_ext->cid,
			CMD_METHOD_BIN_VERIFY, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_bin_verify_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_bin_verify_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->bin_verify_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_bin_verify_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_cbc_mac(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CbcMacEnum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyPara);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyLen);
	if (ret >= 0)
		ret = serialize_service_CbcMacEnum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void cbc_mac_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	cbc_mac_out_t *out = (cbc_mac_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_cbc_mac_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CbcMacEnum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	cbc_mac_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_cbc_mac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->cbc_mac_registry, ser, s_ext->cid,
			CMD_METHOD_CBC_MAC, cbc_mac_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->cbc_mac_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_cbc_mac_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CbcMacEnum_t mode,
				const uint32_t outBuf,
				service_cbc_mac_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_cbc_mac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->cbc_mac_registry, ser, s_ext->cid,
			CMD_METHOD_CBC_MAC, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_cbc_mac_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_cbc_mac_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->cbc_mac_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_cbc_mac_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_cmac(
				serdes_t *ser,
				const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CmacEnum_t mode,
				const uint32_t outBuf
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgBuf);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&msgLen);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyPara);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&keyLen);
	if (ret >= 0)
		ret = serialize_service_CmacEnum(ser, &mode);
	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&outBuf);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void cmac_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	cmac_out_t *out = (cmac_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_cmac_sync(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CmacEnum_t mode,
				const uint32_t outBuf,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	cmac_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_cmac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->cmac_registry, ser, s_ext->cid,
			CMD_METHOD_CMAC, cmac_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->cmac_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_cmac_async(const uint32_t msgBuf,
				const uint32_t msgLen,
				const uint32_t keyPara,
				const uint32_t keyLen,
				const service_CmacEnum_t mode,
				const uint32_t outBuf,
				service_cmac_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_cmac(ser, msgBuf, msgLen, keyPara, keyLen, mode, outBuf);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->cmac_registry, ser, s_ext->cid,
			CMD_METHOD_CMAC, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_cmac_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_cmac_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->cmac_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_cmac_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_slt_method(
				serdes_t *ser,
				const uint32_t bin_index
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_32(ser, (uint32_t *)&bin_index);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void slt_method_sync_callback(
				const uint32_t reply_result,
				const uint32_t recv_bin_index,
				const byte_buffer_t result_descrption,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	slt_method_out_t *out = (slt_method_out_t *)ext;

	if (!out)
		return;
	*out->reply_result = reply_result;
	*out->recv_bin_index = recv_bin_index;
	out->result_descrption->size = result_descrption.size;
	out->result_descrption->data = result_descrption.data;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_slt_method_sync(const uint32_t bin_index,
				uint32_t *reply_result,
				uint32_t *recv_bin_index,
				byte_buffer_t *result_descrption,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	slt_method_out_t out = {.reply_result = reply_result,
				.recv_bin_index = recv_bin_index,
				.result_descrption = result_descrption,
				.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_slt_method(ser, bin_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->slt_method_registry, ser, s_ext->cid,
			CMD_METHOD_SLT_METHOD, slt_method_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->slt_method_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_slt_method_async(const uint32_t bin_index,
				service_slt_method_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_slt_method(ser, bin_index);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->slt_method_registry, ser, s_ext->cid,
			CMD_METHOD_SLT_METHOD, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_slt_method_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_slt_method_callback_t cb = NULL;
	uint32_t reply_result = 0;
	uint32_t recv_bin_index = 0;
	byte_buffer_t result_descrption = { 0 };
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->slt_method_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;
	if (err == SERVICE_NO_ERROR) {
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&reply_result);
		if (ret >= 0)
			ret = deserialize_32(buf, (uint32_t *)&recv_bin_index);
		if (ret >= 0)
			ret = deserialize_byte_buffer(buf, &result_descrption);
		if (ret < 0)
			return -ERR_APP_SERDES;
	}

	// call callback function
	cb = (service_slt_method_callback_t)(reg->cb);
	if (cb)
		cb(reply_result, recv_bin_index, result_descrption, err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

static inline int32_t serialize_config_tzc400(
				serdes_t *ser,
				const uint64_t client_id,
				const service_UInt8Array_t master_name,
				const uint8_t status
				)
{
	int32_t ret = 0;

	if (ret >= 0)
		ret = ipc_ser_put_64(ser, (uint64_t *)&client_id);
	if (ret >= 0)
		ret = serialize_service_UInt8Array(ser, &master_name);
	if (ret >= 0)
		ret = ipc_ser_put_8(ser, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;
	else
		return RESULT_SUCCESS;
}
#ifndef IPC_RTE_BAREMETAL

static void config_tzc400_sync_callback(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				)
{
	config_tzc400_out_t *out = (config_tzc400_out_t *)ext;

	if (!out)
		return;
	*out->err = err;

	IPC_SEM_POST(&out->sem);
}

static int32_t call_config_tzc400_sync(const uint64_t client_id,
				const service_UInt8Array_t master_name,
				const uint8_t status,
				service_ErrorEnum_t *err,
				int64_t timeout_ms,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
	serdes_t serdes = { 0 };
	serdes_t *ser = NULL;
	config_tzc400_out_t out = {.err = err};
	callback_registration_t *reg = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
	IPC_SEM_INIT(&out.sem, 0);
	ser = &serdes;
	(void)ipc_ser_init(ser);

	ret = serialize_config_tzc400(ser, client_id, master_name, status);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->config_tzc400_registry, ser, s_ext->cid,
			CMD_METHOD_CONFIG_TZC400, config_tzc400_sync_callback, &out, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	//wait for reply
	reg = &s_ext->config_tzc400_registry[ret];
	if (timeout_ms <= 0)
		IPC_SEM_WAIT(&out.sem);
	else
		IPC_SEM_TIMED_WAIT(&out.sem, timeout_ms);
	if (ret < 0)
		IPC_LOG_ERR("wait timeout\n");
	clear_registry(reg);
	IPC_SEM_DESTROY(&out.sem);

	return ret;
}
#endif

static int32_t call_config_tzc400_async(const uint64_t client_id,
				const service_UInt8Array_t master_name,
				const uint8_t status,
				service_config_tzc400_callback_t cb,
				void *ext,
				des_buf_t *ext_buf)
{
	int32_t ret = 0;
#ifdef IPC_SHARED_SERIALIZER
	serdes_t *ser = NULL;
#else
	serdes_t serdes = { 0 };
	serdes_t *ser = &serdes;
#endif
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;
#ifdef IPC_SHARED_SERIALIZER
	ser = &data->serializer;
#endif
	(void)ipc_ser_init(ser);

	ret = serialize_config_tzc400(ser, client_id, master_name, status);
	if (ret != 0) {
		IPC_LOG_ERR("serialize fail.\n");
		return -ERR_APP_SERDES;
	}

	// send request
	ret = send_request(data, s_ext->config_tzc400_registry, ser, s_ext->cid,
			CMD_METHOD_CONFIG_TZC400, cb, ext, ext_buf);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send method fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_config_tzc400_callback(des_buf_t *des)
{
	int32_t ret = 0;
	callback_registration_t *reg = NULL;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	service_config_tzc400_callback_t cb = NULL;
	service_ErrorEnum_t err = 0;


	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->config_tzc400_registry[des->header.tok];
	if (!reg->busy) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return -ERR_APP_TOK;
	}
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	// set info (for callback function)
	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;

	// deserialize arguments
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_service_ErrorEnum(buf, &err);

	if (ret < 0)
		return -ERR_APP_SERDES;


	// call callback function
	cb = (service_config_tzc400_callback_t)(reg->cb);
	if (cb)
		cb(err, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// broadcast

// subscribe heartbeat
static int32_t subscribe_heartbeat(
				service_heartbeat_callback_t cb,
				void *ext,
				des_buf_t *ext_buf,
				broadcast_sub_unsub_callback_t cb2,
				void *ext2
				)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// set registry
	s_ext->heartbeat_registry.busy = true;
	(void)set_registry(&s_ext->heartbeat_registry, (void *)cb, ext, ext_buf);

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_SUB_HEARTBEAT, cb2, ext2, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		clear_registry(&s_ext->heartbeat_registry);
		return ret;
	}

	return RESULT_SUCCESS;
}

// unsubscribe heartbeat
static int32_t unsubscribe_heartbeat(broadcast_sub_unsub_callback_t cb, void *ext)
{
	int32_t ret = 0;
	serdes_t *ser = NULL;
	com_client_data_t *data = s_data;

	if (!data || !s_ext || !s_ext->heartbeat_registry.busy)
		return -ERR_APP_PARAM;

	ser = &data->serializer;

	// send request
	ret = send_request(data, data->common_registry, ser, s_ext->cid,
			CMD_METHOD_UNSUB_HEARTBEAT, cb, ext, NULL);
	if (ret < 0 || ret >= IPC_TOKEN_NUM) {
		IPC_LOG_ERR("send fail %" PRId32 ".\n", ret);
		return ret;
	}

	return RESULT_SUCCESS;
}

static inline int32_t call_heartbeat_callback(des_buf_t *des)
{
	int32_t ret = 0;
	des_buf_t *buf = NULL;
	com_client_data_t *data = s_data;
	callback_registration_t *reg = NULL;
	service_heartbeat_callback_t cb = NULL;
	uint8_t status = 0;

	if (!des || !data || !s_ext)
		return -ERR_APP_PARAM;

	reg = &s_ext->heartbeat_registry;
	if (!reg->busy || !reg->cb) {
		IPC_LOG_ERR("callback registry is invalid.\n");
		return RESULT_SUCCESS;
	}

	data->info.uuid = ipc_msg_get_uuid(des->header);
	data->info.timestamp = des->timestamp;
	if (reg->ext_buf) {
		buf = reg->ext_buf;
		(void)ipc_memcpy(buf, des, sizeof(des_buf_t));
	}
	else
		buf = des;
	if (buf->unavail_data_size >= IPC_MAX_DATA_SIZE)
		return -ERR_APP_SERDES;

	if (ret >= 0)
		ret = deserialize_8(buf, (uint8_t *)&status);

	if (ret < 0)
		return -ERR_APP_SERDES;

	cb = (service_heartbeat_callback_t)(reg->cb);
	cb(status, reg->ext, &data->info);

	return RESULT_SUCCESS;
}

// dispatch_broadcast
static inline int32_t dispatch_broadcast(des_buf_t *des)
{
	int32_t ret = 0;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_BROADCAST_HEARTBEAT:
		ret = call_heartbeat_callback(des);
		break;
	default:
		ret = -ERR_APP_UNKNOWN_CMD;
		break;
	}

	return ret;
}

// dispatch_reply
static inline int32_t dispatch_reply(des_buf_t *des)
{
	int32_t ret = 0;
	com_client_data_t *data = s_data;

	if (!des || des->header.pid != s_ext->cid)
		return -ERR_APP_PARAM;

	switch (des->header.cmd) {
	case CMD_METHOD_HELLO:
		ret = call_hello_callback(des);
		break;
	case CMD_METHOD_OTP_INFO:
		ret = call_otp_info_callback(des);
		break;
	case CMD_METHOD_TRNG:
		ret = call_trng_callback(des);
		break;
	case CMD_METHOD_HASH:
		ret = call_hash_callback(des);
		break;
	case CMD_METHOD_HMAC:
		ret = call_hmac_callback(des);
		break;
	case CMD_METHOD_SM3:
		ret = call_sm3_callback(des);
		break;
	case CMD_METHOD_CRC32:
		ret = call_crc32_callback(des);
		break;
	case CMD_METHOD_AES:
		ret = call_aes_callback(des);
		break;
	case CMD_METHOD_SM4:
		ret = call_sm4_callback(des);
		break;
	case CMD_METHOD_RSA_SIGN_OR_VERIFY:
		ret = call_rsa_sign_or_verify_callback(des);
		break;
	case CMD_METHOD_ECC_SIGN_OR_VERIFY:
		ret = call_ecc_sign_or_verify_callback(des);
		break;
	case CMD_METHOD_SM2_SIGN_OR_VERIFY:
		ret = call_sm2_sign_or_verify_callback(des);
		break;
	case CMD_METHOD_SEIP_KEY_STATUS:
		ret = call_seip_key_status_callback(des);
		break;
	case CMD_METHOD_LIFE_CYCLE_STATUS:
		ret = call_life_cycle_status_callback(des);
		break;
	case CMD_METHOD_OTP_USE_WITH_AUTH:
		ret = call_otp_use_with_auth_callback(des);
		break;
	case CMD_METHOD_BIN_VERIFY:
		ret = call_bin_verify_callback(des);
		break;
	case CMD_METHOD_CBC_MAC:
		ret = call_cbc_mac_callback(des);
		break;
	case CMD_METHOD_CMAC:
		ret = call_cmac_callback(des);
		break;
	case CMD_METHOD_SLT_METHOD:
		ret = call_slt_method_callback(des);
		break;
	case CMD_METHOD_CONFIG_TZC400:
		ret = call_config_tzc400_callback(des);
		break;
	case CMD_METHOD_SUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		break;
	case CMD_METHOD_UNSUB_HEARTBEAT:
		ret = call_broadcast_sub_unsub_callback(data, des);
		if (ret >= 0)
			clear_registry(&s_ext->heartbeat_registry);
		break;
	default:
		ret = -ERR_APP_UNKNOWN_CMD;
		break;
	}

	return ret;
}

// register availablity changed callback function
static int32_t register_avail_changed_cb(avail_changed_callback_t cb, void *ext)
{
	if (!s_ext)
		return -ERR_APP_PARAM;

	s_ext->avail_changed_cb = cb;
	s_ext->avail_ext = ext;
	return 0;
}

// initialize client
int32_t service_client_init(com_client_data_t *data, service_client_t *client,
			service_client_ext_t *ext)
{
	if (!data || !client || !ext)
		return -1;

	s_data = data;
	s_ext = ext;

	// set client
	client->version = get_ipc_inf_version;
	client->register_avail_changed = register_avail_changed_cb;
#ifndef IPC_RTE_BAREMETAL
	client->hello_sync = call_hello_sync;
#endif
	client->hello_async = call_hello_async;
(void)init_registry(ext->hello_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->otp_info_sync = call_otp_info_sync;
#endif
	client->otp_info_async = call_otp_info_async;
(void)init_registry(ext->otp_info_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->trng_sync = call_trng_sync;
#endif
	client->trng_async = call_trng_async;
(void)init_registry(ext->trng_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->hash_sync = call_hash_sync;
#endif
	client->hash_async = call_hash_async;
(void)init_registry(ext->hash_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->hmac_sync = call_hmac_sync;
#endif
	client->hmac_async = call_hmac_async;
(void)init_registry(ext->hmac_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->sm3_sync = call_sm3_sync;
#endif
	client->sm3_async = call_sm3_async;
(void)init_registry(ext->sm3_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->crc32_sync = call_crc32_sync;
#endif
	client->crc32_async = call_crc32_async;
(void)init_registry(ext->crc32_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->aes_sync = call_aes_sync;
#endif
	client->aes_async = call_aes_async;
(void)init_registry(ext->aes_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->sm4_sync = call_sm4_sync;
#endif
	client->sm4_async = call_sm4_async;
(void)init_registry(ext->sm4_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->rsa_sign_or_verify_sync = call_rsa_sign_or_verify_sync;
#endif
	client->rsa_sign_or_verify_async = call_rsa_sign_or_verify_async;
(void)init_registry(ext->rsa_sign_or_verify_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->ecc_sign_or_verify_sync = call_ecc_sign_or_verify_sync;
#endif
	client->ecc_sign_or_verify_async = call_ecc_sign_or_verify_async;
(void)init_registry(ext->ecc_sign_or_verify_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->sm2_sign_or_verify_sync = call_sm2_sign_or_verify_sync;
#endif
	client->sm2_sign_or_verify_async = call_sm2_sign_or_verify_async;
(void)init_registry(ext->sm2_sign_or_verify_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->seip_key_status_sync = call_seip_key_status_sync;
#endif
	client->seip_key_status_async = call_seip_key_status_async;
(void)init_registry(ext->seip_key_status_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->life_cycle_status_sync = call_life_cycle_status_sync;
#endif
	client->life_cycle_status_async = call_life_cycle_status_async;
(void)init_registry(ext->life_cycle_status_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->otp_use_with_auth_sync = call_otp_use_with_auth_sync;
#endif
	client->otp_use_with_auth_async = call_otp_use_with_auth_async;
(void)init_registry(ext->otp_use_with_auth_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->bin_verify_sync = call_bin_verify_sync;
#endif
	client->bin_verify_async = call_bin_verify_async;
(void)init_registry(ext->bin_verify_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->cbc_mac_sync = call_cbc_mac_sync;
#endif
	client->cbc_mac_async = call_cbc_mac_async;
(void)init_registry(ext->cbc_mac_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->cmac_sync = call_cmac_sync;
#endif
	client->cmac_async = call_cmac_async;
(void)init_registry(ext->cmac_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->slt_method_sync = call_slt_method_sync;
#endif
	client->slt_method_async = call_slt_method_async;
(void)init_registry(ext->slt_method_registry);
	#ifndef IPC_RTE_BAREMETAL
	client->config_tzc400_sync = call_config_tzc400_sync;
#endif
	client->config_tzc400_async = call_config_tzc400_async;
(void)init_registry(ext->config_tzc400_registry);

	client->heartbeat_sub = subscribe_heartbeat;
	client->heartbeat_unsub = unsubscribe_heartbeat;
	(void)init_registry(&ext->heartbeat_registry);

	client->dispatch_broadcast = dispatch_broadcast;
	client->dispatch_reply = dispatch_reply;

	// set ext
	ext->cid = CID;
	ext->ccid = CCID;
	ext->cid_mask = CID_MASK;
	ext->status = false;

	return 0;
}
// destroy client
void service_client_destroy(void)
{
	destroy_registry(&s_ext->heartbeat_registry);

	s_data = NULL;
	s_ext = NULL;
}
