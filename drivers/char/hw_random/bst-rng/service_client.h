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

#ifndef SERVICE_CLIENT_H
#define SERVICE_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "service_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for hello_async method.
 *
 * @param message The output argument returned by hello_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_hello_callback_t)(
				const char *message,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for otp_info_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_otp_info_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for trng_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_trng_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for hash_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_hash_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for hmac_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_hmac_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for crc32_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_crc32_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for sm3_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_sm3_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for aes_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_aes_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for sm4_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_sm4_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for rsa_sign_or_verify_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_rsa_sign_or_verify_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for ecc_sign_or_verify_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_ecc_sign_or_verify_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for sm2_sign_or_verify_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_sm2_sign_or_verify_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for seip_key_status_async method.
 *
 * @param keyStatus The output argument returned by seip_key_status_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_seip_key_status_callback_t)(
				const service_OtpStatus_t keyStatus,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for life_cycle_status_async method.
 *
 * @param lifeCycle The output argument returned by life_cycle_status_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_life_cycle_status_callback_t)(
				const service_LifeCycleEnum_t lifeCycle,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for otp_use_with_auth_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_otp_use_with_auth_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for bin_verify_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_bin_verify_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for cbc_mac_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_cbc_mac_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for cmac_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_cmac_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for slt_method_async method.
 *
 * @param reply_result The output argument returned by slt_method_async.
 * @param recv_bin_index The output argument returned by slt_method_async.
 * @param result_descrption The output argument returned by slt_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_slt_method_callback_t)(
				const uint32_t reply_result,
				const uint32_t recv_bin_index,
				const byte_buffer_t result_descrption,
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for config_tzc400_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_config_tzc400_callback_t)(
				const service_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for broadcast heartbeat.
 *
 * @param status The output argument returned by broadcast heartbeat.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*service_heartbeat_callback_t)(
				const uint8_t status,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _service_client_t {
	/**
	 * Get the version of the interface.
	 *
	 * @return The version struct, containing major and minor.
	 */
	ipc_inf_version_t (*version)(void);

	/**
	 * Register server availability changed callback.
	 *
	 * @param cb The callback function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_avail_changed)(avail_changed_callback_t cb,
					void *ext);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param name The input argument of method hello.
	 * @param message The output argument of method hello.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hello_sync)(
					const char *name,
					char **message,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the hello method.
	 *
	 * @param name The input argument of method hello.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hello_async)(
					const char *name,
					service_hello_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param infoType The input argument of method otp_info.
	 * @param outBuf The input argument of method otp_info.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*otp_info_sync)(
					const service_OtpInfoEnum_t infoType,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the otp_info method.
	 *
	 * @param infoType The input argument of method otp_info.
	 * @param outBuf The input argument of method otp_info.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*otp_info_async)(
					const service_OtpInfoEnum_t infoType,
					const uint32_t outBuf,
					service_otp_info_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param trngLen The input argument of method trng.
	 * @param outBuf The input argument of method trng.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*trng_sync)(
					const uint32_t trngLen,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the trng method.
	 *
	 * @param trngLen The input argument of method trng.
	 * @param outBuf The input argument of method trng.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*trng_async)(
					const uint32_t trngLen,
					const uint32_t outBuf,
					service_trng_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method hash.
	 * @param msgLen The input argument of method hash.
	 * @param mode The input argument of method hash.
	 * @param outBuf The input argument of method hash.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hash_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const service_HashEnum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the hash method.
	 *
	 * @param msgBuf The input argument of method hash.
	 * @param msgLen The input argument of method hash.
	 * @param mode The input argument of method hash.
	 * @param outBuf The input argument of method hash.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hash_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const service_HashEnum_t mode,
					const uint32_t outBuf,
					service_hash_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method hmac.
	 * @param msgLen The input argument of method hmac.
	 * @param keyPara The input argument of method hmac.
	 * @param keyLen The input argument of method hmac.
	 * @param mode The input argument of method hmac.
	 * @param outBuf The input argument of method hmac.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hmac_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_HmacEnum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the hmac method.
	 *
	 * @param msgBuf The input argument of method hmac.
	 * @param msgLen The input argument of method hmac.
	 * @param keyPara The input argument of method hmac.
	 * @param keyLen The input argument of method hmac.
	 * @param mode The input argument of method hmac.
	 * @param outBuf The input argument of method hmac.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hmac_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_HmacEnum_t mode,
					const uint32_t outBuf,
					service_hmac_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method crc32.
	 * @param msgLen The input argument of method crc32.
	 * @param outBuf The input argument of method crc32.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*crc32_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the crc32 method.
	 *
	 * @param msgBuf The input argument of method crc32.
	 * @param msgLen The input argument of method crc32.
	 * @param outBuf The input argument of method crc32.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*crc32_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t outBuf,
					service_crc32_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method sm3.
	 * @param msgLen The input argument of method sm3.
	 * @param outBuf The input argument of method sm3.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm3_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the sm3 method.
	 *
	 * @param msgBuf The input argument of method sm3.
	 * @param msgLen The input argument of method sm3.
	 * @param outBuf The input argument of method sm3.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm3_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t outBuf,
					service_sm3_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method aes.
	 * @param ivBuf The input argument of method aes.
	 * @param keyPara The input argument of method aes.
	 * @param msgLen The input argument of method aes.
	 * @param isDecrypt The input argument of method aes.
	 * @param mode The input argument of method aes.
	 * @param outBuf The input argument of method aes.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*aes_sync)(
					const uint32_t msgBuf,
					const uint32_t ivBuf,
					const uint32_t keyPara,
					const uint32_t msgLen,
					const service_CryptoEnum_t isDecrypt,
					const service_AesEnum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the aes method.
	 *
	 * @param msgBuf The input argument of method aes.
	 * @param ivBuf The input argument of method aes.
	 * @param keyPara The input argument of method aes.
	 * @param msgLen The input argument of method aes.
	 * @param isDecrypt The input argument of method aes.
	 * @param mode The input argument of method aes.
	 * @param outBuf The input argument of method aes.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*aes_async)(
					const uint32_t msgBuf,
					const uint32_t ivBuf,
					const uint32_t keyPara,
					const uint32_t msgLen,
					const service_CryptoEnum_t isDecrypt,
					const service_AesEnum_t mode,
					const uint32_t outBuf,
					service_aes_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method sm4.
	 * @param ivBuf The input argument of method sm4.
	 * @param keyPara The input argument of method sm4.
	 * @param msgLen The input argument of method sm4.
	 * @param isDecrypt The input argument of method sm4.
	 * @param mode The input argument of method sm4.
	 * @param outBuf The input argument of method sm4.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm4_sync)(
					const uint32_t msgBuf,
					const uint32_t ivBuf,
					const uint32_t keyPara,
					const uint32_t msgLen,
					const service_CryptoEnum_t isDecrypt,
					const service_Sm4Enum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the sm4 method.
	 *
	 * @param msgBuf The input argument of method sm4.
	 * @param ivBuf The input argument of method sm4.
	 * @param keyPara The input argument of method sm4.
	 * @param msgLen The input argument of method sm4.
	 * @param isDecrypt The input argument of method sm4.
	 * @param mode The input argument of method sm4.
	 * @param outBuf The input argument of method sm4.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm4_async)(
					const uint32_t msgBuf,
					const uint32_t ivBuf,
					const uint32_t keyPara,
					const uint32_t msgLen,
					const service_CryptoEnum_t isDecrypt,
					const service_Sm4Enum_t mode,
					const uint32_t outBuf,
					service_sm4_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param isVerify The input argument of method rsa_sign_or_verify.
	 * @param msgBuf The input argument of method rsa_sign_or_verify.
	 * @param eBuf The input argument of method rsa_sign_or_verify.
	 * @param nBuf The input argument of method rsa_sign_or_verify.
	 * @param dBuf The input argument of method rsa_sign_or_verify.
	 * @param nBitLen The input argument of method rsa_sign_or_verify.
	 * @param signBuf The input argument of method rsa_sign_or_verify.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*rsa_sign_or_verify_sync)(
					const service_SignEnum_t isVerify,
					const uint32_t msgBuf,
					const uint32_t eBuf,
					const uint32_t nBuf,
					const uint32_t dBuf,
					const uint32_t nBitLen,
					const uint32_t signBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the rsa_sign_or_verify method.
	 *
	 * @param isVerify The input argument of method rsa_sign_or_verify.
	 * @param msgBuf The input argument of method rsa_sign_or_verify.
	 * @param eBuf The input argument of method rsa_sign_or_verify.
	 * @param nBuf The input argument of method rsa_sign_or_verify.
	 * @param dBuf The input argument of method rsa_sign_or_verify.
	 * @param nBitLen The input argument of method rsa_sign_or_verify.
	 * @param signBuf The input argument of method rsa_sign_or_verify.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*rsa_sign_or_verify_async)(
					const service_SignEnum_t isVerify,
					const uint32_t msgBuf,
					const uint32_t eBuf,
					const uint32_t nBuf,
					const uint32_t dBuf,
					const uint32_t nBitLen,
					const uint32_t signBuf,
					service_rsa_sign_or_verify_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param isVerify The input argument of method ecc_sign_or_verify.
	 * @param eBuf The input argument of method ecc_sign_or_verify.
	 * @param eBitlen The input argument of method ecc_sign_or_verify.
	 * @param keyBuf The input argument of method ecc_sign_or_verify.
	 * @param signBuf The input argument of method ecc_sign_or_verify.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*ecc_sign_or_verify_sync)(
					const service_SignEnum_t isVerify,
					const uint32_t eBuf,
					const uint32_t eBitlen,
					const uint32_t keyBuf,
					const uint32_t signBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the ecc_sign_or_verify method.
	 *
	 * @param isVerify The input argument of method ecc_sign_or_verify.
	 * @param eBuf The input argument of method ecc_sign_or_verify.
	 * @param eBitlen The input argument of method ecc_sign_or_verify.
	 * @param keyBuf The input argument of method ecc_sign_or_verify.
	 * @param signBuf The input argument of method ecc_sign_or_verify.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*ecc_sign_or_verify_async)(
					const service_SignEnum_t isVerify,
					const uint32_t eBuf,
					const uint32_t eBitlen,
					const uint32_t keyBuf,
					const uint32_t signBuf,
					service_ecc_sign_or_verify_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param isVerify The input argument of method sm2_sign_or_verify.
	 * @param eBuf The input argument of method sm2_sign_or_verify.
	 * @param keyBuf The input argument of method sm2_sign_or_verify.
	 * @param signBuf The input argument of method sm2_sign_or_verify.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm2_sign_or_verify_sync)(
					const service_SignEnum_t isVerify,
					const uint32_t eBuf,
					const uint32_t keyBuf,
					const uint32_t signBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the sm2_sign_or_verify method.
	 *
	 * @param isVerify The input argument of method sm2_sign_or_verify.
	 * @param eBuf The input argument of method sm2_sign_or_verify.
	 * @param keyBuf The input argument of method sm2_sign_or_verify.
	 * @param signBuf The input argument of method sm2_sign_or_verify.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*sm2_sign_or_verify_async)(
					const service_SignEnum_t isVerify,
					const uint32_t eBuf,
					const uint32_t keyBuf,
					const uint32_t signBuf,
					service_sm2_sign_or_verify_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param keyId The input argument of method seip_key_status.
	 * @param keyStatus The output argument of method seip_key_status.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*seip_key_status_sync)(
					const uint32_t keyId,
					service_OtpStatus_t *keyStatus,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the seip_key_status method.
	 *
	 * @param keyId The input argument of method seip_key_status.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*seip_key_status_async)(
					const uint32_t keyId,
					service_seip_key_status_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *

	 * @param lifeCycle The output argument of method life_cycle_status.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*life_cycle_status_sync)(
					service_LifeCycleEnum_t *lifeCycle,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the life_cycle_status method.
	 *

	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*life_cycle_status_async)(
					service_life_cycle_status_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param op The input argument of method otp_use_with_auth.
	 * @param cmdWordBuf The input argument of method otp_use_with_auth.
	 * @param tokenBuf The input argument of method otp_use_with_auth.
	 * @param rbBuf The input argument of method otp_use_with_auth.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*otp_use_with_auth_sync)(
					const service_OtpUseOptionEnum_t op,
					const uint32_t cmdWordBuf,
					const uint32_t tokenBuf,
					const uint32_t rbBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the otp_use_with_auth method.
	 *
	 * @param op The input argument of method otp_use_with_auth.
	 * @param cmdWordBuf The input argument of method otp_use_with_auth.
	 * @param tokenBuf The input argument of method otp_use_with_auth.
	 * @param rbBuf The input argument of method otp_use_with_auth.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*otp_use_with_auth_async)(
					const service_OtpUseOptionEnum_t op,
					const uint32_t cmdWordBuf,
					const uint32_t tokenBuf,
					const uint32_t rbBuf,
					service_otp_use_with_auth_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param op The input argument of method bin_verify.
	 * @param inBuff The input argument of method bin_verify.
	 * @param length The input argument of method bin_verify.
	 * @param signBuf The input argument of method bin_verify.
	 * @param resultBuf The input argument of method bin_verify.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*bin_verify_sync)(
					const service_BinVerifyOpEnum_t op,
					const uint32_t inBuff,
					const uint32_t length,
					const uint32_t signBuf,
					const uint32_t resultBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the bin_verify method.
	 *
	 * @param op The input argument of method bin_verify.
	 * @param inBuff The input argument of method bin_verify.
	 * @param length The input argument of method bin_verify.
	 * @param signBuf The input argument of method bin_verify.
	 * @param resultBuf The input argument of method bin_verify.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*bin_verify_async)(
					const service_BinVerifyOpEnum_t op,
					const uint32_t inBuff,
					const uint32_t length,
					const uint32_t signBuf,
					const uint32_t resultBuf,
					service_bin_verify_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method cbc_mac.
	 * @param msgLen The input argument of method cbc_mac.
	 * @param keyPara The input argument of method cbc_mac.
	 * @param keyLen The input argument of method cbc_mac.
	 * @param mode The input argument of method cbc_mac.
	 * @param outBuf The input argument of method cbc_mac.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*cbc_mac_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_CbcMacEnum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the cbc_mac method.
	 *
	 * @param msgBuf The input argument of method cbc_mac.
	 * @param msgLen The input argument of method cbc_mac.
	 * @param keyPara The input argument of method cbc_mac.
	 * @param keyLen The input argument of method cbc_mac.
	 * @param mode The input argument of method cbc_mac.
	 * @param outBuf The input argument of method cbc_mac.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*cbc_mac_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_CbcMacEnum_t mode,
					const uint32_t outBuf,
					service_cbc_mac_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param msgBuf The input argument of method cmac.
	 * @param msgLen The input argument of method cmac.
	 * @param keyPara The input argument of method cmac.
	 * @param keyLen The input argument of method cmac.
	 * @param mode The input argument of method cmac.
	 * @param outBuf The input argument of method cmac.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*cmac_sync)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_CmacEnum_t mode,
					const uint32_t outBuf,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the cmac method.
	 *
	 * @param msgBuf The input argument of method cmac.
	 * @param msgLen The input argument of method cmac.
	 * @param keyPara The input argument of method cmac.
	 * @param keyLen The input argument of method cmac.
	 * @param mode The input argument of method cmac.
	 * @param outBuf The input argument of method cmac.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*cmac_async)(
					const uint32_t msgBuf,
					const uint32_t msgLen,
					const uint32_t keyPara,
					const uint32_t keyLen,
					const service_CmacEnum_t mode,
					const uint32_t outBuf,
					service_cmac_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param bin_index The input argument of method slt_method.
	 * @param reply_result The output argument of method slt_method.
	 * @param recv_bin_index The output argument of method slt_method.
	 * @param result_descrption The output argument of method slt_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*slt_method_sync)(
					const uint32_t bin_index,
					uint32_t *reply_result,
					uint32_t *recv_bin_index,
					byte_buffer_t *result_descrption,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the slt_method method.
	 *
	 * @param bin_index The input argument of method slt_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*slt_method_async)(
					const uint32_t bin_index,
					service_slt_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param client_id The input argument of method config_tzc400.
	 * @param master_name The input argument of method config_tzc400.
	 * @param status The input argument of method config_tzc400.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*config_tzc400_sync)(
					const uint64_t client_id,
					const service_UInt8Array_t master_name,
					const uint8_t status,
					service_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the config_tzc400 method.
	 *
	 * @param client_id The input argument of method config_tzc400.
	 * @param master_name The input argument of method config_tzc400.
	 * @param status The input argument of method config_tzc400.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*config_tzc400_async)(
					const uint64_t client_id,
					const service_UInt8Array_t master_name,
					const uint8_t status,
					service_config_tzc400_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Subscribe to the heartbeat broadcast.
	 *
	 * @param cb The callback function called when the broadcast received.
	 * @param ext The user-defined data passed to the broadcast callback.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @param cb2 The callback function called when the subscription is complete.
	 * @param ext2 The user-defined data passed to the subscription callback.
	 * @note all the data are stored in ext_buf passed to async call.
	 * If ext_buf is NULL, internal buffer will be used.
	 * Please note that, the internal buffer is shared by all callbacks.
	 * The data MAY CHANGED after leaving the callback function.
	 * @note subscribe multiple times will result in multiple callbacks, while
	 * the broadcast registry will be overwritten by the last subscription. This
	 * means the cb, ext, ext_buf will be overwritten by the last subscription.
	 * cb2 and ext2 will not be affected.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*heartbeat_sub)(
					service_heartbeat_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the heartbeat broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*heartbeat_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);

	/**
	 * Dispatch broadcast messages.
	 *
	 * @param des The received message package.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_broadcast)(des_buf_t *des);

	/**
	 * Dispatch reply messages.
	 *
	 * @param des The received message package.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_reply)(des_buf_t *des);
};
#define service_client_t struct _service_client_t

/**
 *  The extend data used by the client.
 */
struct _service_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t hello_registry[IPC_TOKEN_NUM];
	callback_registration_t otp_info_registry[IPC_TOKEN_NUM];
	callback_registration_t trng_registry[IPC_TOKEN_NUM];
	callback_registration_t hash_registry[IPC_TOKEN_NUM];
	callback_registration_t hmac_registry[IPC_TOKEN_NUM];
	callback_registration_t crc32_registry[IPC_TOKEN_NUM];
	callback_registration_t sm3_registry[IPC_TOKEN_NUM];
	callback_registration_t aes_registry[IPC_TOKEN_NUM];
	callback_registration_t sm4_registry[IPC_TOKEN_NUM];
	callback_registration_t rsa_sign_or_verify_registry[IPC_TOKEN_NUM];
	callback_registration_t ecc_sign_or_verify_registry[IPC_TOKEN_NUM];
	callback_registration_t sm2_sign_or_verify_registry[IPC_TOKEN_NUM];
	callback_registration_t seip_key_status_registry[IPC_TOKEN_NUM];
	callback_registration_t life_cycle_status_registry[IPC_TOKEN_NUM];
	callback_registration_t otp_use_with_auth_registry[IPC_TOKEN_NUM];
	callback_registration_t bin_verify_registry[IPC_TOKEN_NUM];
	callback_registration_t cbc_mac_registry[IPC_TOKEN_NUM];
	callback_registration_t cmac_registry[IPC_TOKEN_NUM];
	callback_registration_t slt_method_registry[IPC_TOKEN_NUM];
	callback_registration_t config_tzc400_registry[IPC_TOKEN_NUM];
	callback_registration_t heartbeat_registry;
};
#define service_client_ext_t struct _service_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for service_client_t
 * @param ext The data for service_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t service_client_init(com_client_data_t *data,
			service_client_t *client,
			service_client_ext_t *ext);

/**
 * Destroys the client.
 */
void service_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // SERVICE_CLIENT_H
