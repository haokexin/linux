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

#ifndef AUDIO_IPC_CLIENT_H
#define AUDIO_IPC_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "audio_ipc_datatype.h"

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
typedef void (*audio_ipc_hello_callback_t)(
				const char *message,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for slt_method_async method.
 *
 * @param reply_result The output argument returned by slt_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_slt_method_callback_t)(
				const uint32_t reply_result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for xrp_shmem_addr_method_async method.
 *
 * @param out_cmd The output argument returned by xrp_shmem_addr_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_xrp_shmem_addr_method_callback_t)(
				const audio_ipc_XrpDspCmd_t out_cmd,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for set_scenario_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_set_scenario_callback_t)(
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for get_cur_scenario_async method.
 *
 * @param scenario_name The output argument returned by get_cur_scenario_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_get_cur_scenario_callback_t)(
				const char *scenario_name,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for audio_config_async method.
 *
 * @param result The output argument returned by audio_config_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_audio_config_callback_t)(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for ctrl_cmd_async method.
 *
 * @param result The output argument returned by ctrl_cmd_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_ctrl_cmd_callback_t)(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for start_stop_to_play_async method.
 *
 * @param result The output argument returned by start_stop_to_play_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_start_stop_to_play_callback_t)(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for send_dma_buff_global_fd_async method.
 *
 * @param result The output argument returned by send_dma_buff_global_fd_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_send_dma_buff_global_fd_callback_t)(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for data_ready_in_pingpong_buff_async method.
 *
 * @param result The output argument returned by data_ready_in_pingpong_buff_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_data_ready_in_pingpong_buff_callback_t)(
				const uint8_t result,
				const audio_ipc_ErrorEnum_t err,
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
typedef void (*audio_ipc_heartbeat_callback_t)(
				const uint8_t status,
				void *ext,
				const ext_info_t *info
				);
/**
 * Callback function for broadcast kws_triggered.
 *
 * @param kws_result The output argument returned by broadcast kws_triggered.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_kws_triggered_callback_t)(
				const uint8_t kws_result,
				void *ext,
				const ext_info_t *info
				);
/**
 * Callback function for broadcast data_comsumed_event.
 *
 * @param count The output argument returned by broadcast data_comsumed_event.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*audio_ipc_data_comsumed_event_callback_t)(
				const uint32_t count,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _audio_ipc_client_t {
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
					audio_ipc_ErrorEnum_t *err,
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
					audio_ipc_hello_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param bin_index The input argument of method slt_method.
	 * @param reply_result The output argument of method slt_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*slt_method_sync)(
					const uint32_t bin_index,
					uint32_t *reply_result,
					audio_ipc_ErrorEnum_t *err,
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
					audio_ipc_slt_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param status The input argument of method no_reply_method.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*no_reply_method_fire_and_forget)(
					const uint8_t status
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param in_cmd The input argument of method xrp_shmem_addr_method.
	 * @param out_cmd The output argument of method xrp_shmem_addr_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*xrp_shmem_addr_method_sync)(
					const audio_ipc_XrpDspCmd_t in_cmd,
					audio_ipc_XrpDspCmd_t *out_cmd,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the xrp_shmem_addr_method method.
	 *
	 * @param in_cmd The input argument of method xrp_shmem_addr_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*xrp_shmem_addr_method_async)(
					const audio_ipc_XrpDspCmd_t in_cmd,
					audio_ipc_xrp_shmem_addr_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param scenario_name The input argument of method set_scenario.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*set_scenario_sync)(
					const char *scenario_name,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the set_scenario method.
	 *
	 * @param scenario_name The input argument of method set_scenario.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*set_scenario_async)(
					const char *scenario_name,
					audio_ipc_set_scenario_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *

	 * @param scenario_name The output argument of method get_cur_scenario.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_cur_scenario_sync)(
					char **scenario_name,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the get_cur_scenario method.
	 *

	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_cur_scenario_async)(
					audio_ipc_get_cur_scenario_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param channel The input argument of method audio_config.
	 * @param sample_rate The input argument of method audio_config.
	 * @param data_width The input argument of method audio_config.
	 * @param direction The input argument of method audio_config.
	 * @param result The output argument of method audio_config.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*audio_config_sync)(
					const uint8_t channel,
					const uint32_t sample_rate,
					const uint8_t data_width,
					const uint8_t direction,
					uint8_t *result,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the audio_config method.
	 *
	 * @param channel The input argument of method audio_config.
	 * @param sample_rate The input argument of method audio_config.
	 * @param data_width The input argument of method audio_config.
	 * @param direction The input argument of method audio_config.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*audio_config_async)(
					const uint8_t channel,
					const uint32_t sample_rate,
					const uint8_t data_width,
					const uint8_t direction,
					audio_ipc_audio_config_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param channel The input argument of method ctrl_cmd.
	 * @param func_id The input argument of method ctrl_cmd.
	 * @param in_data The input argument of method ctrl_cmd.
	 * @param result The output argument of method ctrl_cmd.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*ctrl_cmd_sync)(
					const uint8_t channel,
					const uint8_t func_id,
					const byte_buffer_t in_data,
					uint8_t *result,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the ctrl_cmd method.
	 *
	 * @param channel The input argument of method ctrl_cmd.
	 * @param func_id The input argument of method ctrl_cmd.
	 * @param in_data The input argument of method ctrl_cmd.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*ctrl_cmd_async)(
					const uint8_t channel,
					const uint8_t func_id,
					const byte_buffer_t in_data,
					audio_ipc_ctrl_cmd_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param chn The input argument of method start_stop_to_play.
	 * @param sts The input argument of method start_stop_to_play.
	 * @param direction The input argument of method start_stop_to_play.
	 * @param result The output argument of method start_stop_to_play.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*start_stop_to_play_sync)(
					const uint8_t chn,
					const uint8_t sts,
					const uint8_t direction,
					uint8_t *result,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the start_stop_to_play method.
	 *
	 * @param chn The input argument of method start_stop_to_play.
	 * @param sts The input argument of method start_stop_to_play.
	 * @param direction The input argument of method start_stop_to_play.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*start_stop_to_play_async)(
					const uint8_t chn,
					const uint8_t sts,
					const uint8_t direction,
					audio_ipc_start_stop_to_play_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param gFd The input argument of method send_dma_buff_global_fd.
	 * @param size The input argument of method send_dma_buff_global_fd.
	 * @param result The output argument of method send_dma_buff_global_fd.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*send_dma_buff_global_fd_sync)(
					const int64_t gFd,
					const int64_t size,
					uint8_t *result,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the send_dma_buff_global_fd method.
	 *
	 * @param gFd The input argument of method send_dma_buff_global_fd.
	 * @param size The input argument of method send_dma_buff_global_fd.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*send_dma_buff_global_fd_async)(
					const int64_t gFd,
					const int64_t size,
					audio_ipc_send_dma_buff_global_fd_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param isPing The input argument of method data_ready_in_pingpong_buff.
	 * @param result The output argument of method data_ready_in_pingpong_buff.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*data_ready_in_pingpong_buff_sync)(
					const bool isPing,
					uint8_t *result,
					audio_ipc_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the data_ready_in_pingpong_buff method.
	 *
	 * @param isPing The input argument of method data_ready_in_pingpong_buff.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*data_ready_in_pingpong_buff_async)(
					const bool isPing,
					audio_ipc_data_ready_in_pingpong_buff_callback_t cb,
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
					audio_ipc_heartbeat_callback_t cb,
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
	 * Subscribe to the kws_triggered broadcast.
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
	int32_t (*kws_triggered_sub)(
					audio_ipc_kws_triggered_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the kws_triggered broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*kws_triggered_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);
	/**
	 * Subscribe to the data_comsumed_event broadcast.
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
	int32_t (*data_comsumed_event_sub)(
					audio_ipc_data_comsumed_event_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the data_comsumed_event broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*data_comsumed_event_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);

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
#define audio_ipc_client_t struct _audio_ipc_client_t

/**
 *  The extend data used by the client.
 */
struct _audio_ipc_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t hello_registry[IPC_TOKEN_NUM];
	callback_registration_t slt_method_registry[IPC_TOKEN_NUM];
	callback_registration_t xrp_shmem_addr_method_registry[IPC_TOKEN_NUM];
	callback_registration_t set_scenario_registry[IPC_TOKEN_NUM];
	callback_registration_t get_cur_scenario_registry[IPC_TOKEN_NUM];
	callback_registration_t audio_config_registry[IPC_TOKEN_NUM];
	callback_registration_t ctrl_cmd_registry[IPC_TOKEN_NUM];
	callback_registration_t start_stop_to_play_registry[IPC_TOKEN_NUM];
	callback_registration_t send_dma_buff_global_fd_registry[IPC_TOKEN_NUM];
	callback_registration_t data_ready_in_pingpong_buff_registry[IPC_TOKEN_NUM];
	callback_registration_t heartbeat_registry;
	callback_registration_t kws_triggered_registry;
	callback_registration_t data_comsumed_event_registry;
};
#define audio_ipc_client_ext_t struct _audio_ipc_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for audio_ipc_client_t
 * @param ext The data for audio_ipc_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t audio_ipc_client_init(com_client_data_t *data,
			audio_ipc_client_t *client,
			audio_ipc_client_ext_t *ext);

/**
 * Destroys the client.
 */
void audio_ipc_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_IPC_CLIENT_H
