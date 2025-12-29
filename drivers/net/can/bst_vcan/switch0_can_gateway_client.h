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

#ifndef SWITCH0_CAN_GATEWAY_CLIENT_H
#define SWITCH0_CAN_GATEWAY_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "switch0_can_gateway_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for get_ETH_statistics_async method.
 *
 * @param statistics_data_ptr The output argument returned by get_ETH_statistics_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_get_ETH_statistics_callback_t)(
				const uint32_t statistics_data_ptr,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for vmac_method_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_vmac_method_callback_t)(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for vcan_method_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_vcan_method_callback_t)(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for can_gateway_send_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_can_gateway_send_callback_t)(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for can_config_query_async method.
 *
 * @param config_info The output argument returned by can_config_query_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_can_config_query_callback_t)(
				const switch0_can_gateway_can_config_t *config_info,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for adas_vcan_notify_sqbuf_addr_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_adas_vcan_notify_sqbuf_addr_callback_t)(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for adas_vcan_driver_remove_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_adas_vcan_driver_remove_callback_t)(
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for vcan_port_method_async method.
 *
 * @param rsp The output argument returned by vcan_port_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_vcan_port_method_callback_t)(
				const switch0_can_gateway_vcan_port_msg_t rsp,
				const switch0_can_gateway_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for broadcast can_status_notify.
 *
 * @param status_info The output argument returned by broadcast can_status_notify.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_can_status_notify_callback_t)(
				const switch0_can_gateway_can_status_t *status_info,
				void *ext,
				const ext_info_t *info
				);
/**
 * Callback function for broadcast can_gateway_recv.
 *

 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_can_gateway_recv_callback_t)(
				void *ext,
				const ext_info_t *info
				);
/**
 * Callback function for broadcast vcan_port_broadcast.
 *
 * @param evt The output argument returned by broadcast vcan_port_broadcast.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*switch0_can_gateway_vcan_port_broadcast_callback_t)(
				const switch0_can_gateway_vcan_port_msg_t evt,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _switch0_can_gateway_client_t {
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

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param element_ptr The input argument of method notify_Eth_2_Can_enqueue.
	 * @param element_count The input argument of method notify_Eth_2_Can_enqueue.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*notify_Eth_2_Can_enqueue_fire_and_forget)(
					const uint32_t element_ptr,
					const uint16_t element_count
					);

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param element_ptr The input argument of method notify_Eth_2_Can_dequeue.
	 * @param element_count The input argument of method notify_Eth_2_Can_dequeue.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*notify_Eth_2_Can_dequeue_fire_and_forget)(
					const uint32_t element_ptr,
					const uint16_t element_count
					);

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param element_ptr The input argument of method notify_Can_2_Eth_enqueue.
	 * @param element_count The input argument of method notify_Can_2_Eth_enqueue.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*notify_Can_2_Eth_enqueue_fire_and_forget)(
					const uint32_t element_ptr,
					const uint16_t element_count
					);

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param element_ptr The input argument of method notify_Can_2_Eth_dequeue.
	 * @param element_count The input argument of method notify_Can_2_Eth_dequeue.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*notify_Can_2_Eth_dequeue_fire_and_forget)(
					const uint32_t element_ptr,
					const uint16_t element_count
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *

	 * @param statistics_data_ptr The output argument of method get_ETH_statistics.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_ETH_statistics_sync)(
					uint32_t *statistics_data_ptr,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the get_ETH_statistics method.
	 *

	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_ETH_statistics_async)(
					switch0_can_gateway_get_ETH_statistics_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param vmac_msg The input argument of method vmac_method.
	 * @param flag The input argument of method vmac_method.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vmac_method_sync)(
					const switch0_can_gateway_MyArray_t vmac_msg,
					const uint32_t flag,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the vmac_method method.
	 *
	 * @param vmac_msg The input argument of method vmac_method.
	 * @param flag The input argument of method vmac_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vmac_method_async)(
					const switch0_can_gateway_MyArray_t vmac_msg,
					const uint32_t flag,
					switch0_can_gateway_vmac_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param command The input argument of method vcan_method.
	 * @param param The input argument of method vcan_method.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vcan_method_sync)(
					const uint8_t command,
					const uint32_t param,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the vcan_method method.
	 *
	 * @param command The input argument of method vcan_method.
	 * @param param The input argument of method vcan_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vcan_method_async)(
					const uint8_t command,
					const uint32_t param,
					switch0_can_gateway_vcan_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param x2can The input argument of method can_gateway_send.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*can_gateway_send_sync)(
					const switch0_can_gateway_UInt8Array128_t *x2can,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the can_gateway_send method.
	 *
	 * @param x2can The input argument of method can_gateway_send.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*can_gateway_send_async)(
					const switch0_can_gateway_UInt8Array128_t *x2can,
					switch0_can_gateway_can_gateway_send_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param can_bus_id The input argument of method can_config_query.
	 * @param config_info The output argument of method can_config_query.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*can_config_query_sync)(
					const uint8_t can_bus_id,
					switch0_can_gateway_can_config_t **config_info,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the can_config_query method.
	 *
	 * @param can_bus_id The input argument of method can_config_query.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*can_config_query_async)(
					const uint8_t can_bus_id,
					switch0_can_gateway_can_config_query_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param pa_low The input argument of method adas_vcan_notify_sqbuf_addr.
	 * @param pa_high The input argument of method adas_vcan_notify_sqbuf_addr.
	 * @param size The input argument of method adas_vcan_notify_sqbuf_addr.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*adas_vcan_notify_sqbuf_addr_sync)(
					const uint32_t pa_low,
					const uint32_t pa_high,
					const uint32_t size,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the adas_vcan_notify_sqbuf_addr method.
	 *
	 * @param pa_low The input argument of method adas_vcan_notify_sqbuf_addr.
	 * @param pa_high The input argument of method adas_vcan_notify_sqbuf_addr.
	 * @param size The input argument of method adas_vcan_notify_sqbuf_addr.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*adas_vcan_notify_sqbuf_addr_async)(
					const uint32_t pa_low,
					const uint32_t pa_high,
					const uint32_t size,
					switch0_can_gateway_adas_vcan_notify_sqbuf_addr_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *


	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*adas_vcan_driver_remove_sync)(
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the adas_vcan_driver_remove method.
	 *

	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*adas_vcan_driver_remove_async)(
					switch0_can_gateway_adas_vcan_driver_remove_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param req The input argument of method vcan_port_method.
	 * @param rsp The output argument of method vcan_port_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vcan_port_method_sync)(
					const switch0_can_gateway_vcan_port_msg_t req,
					switch0_can_gateway_vcan_port_msg_t *rsp,
					switch0_can_gateway_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the vcan_port_method method.
	 *
	 * @param req The input argument of method vcan_port_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*vcan_port_method_async)(
					const switch0_can_gateway_vcan_port_msg_t req,
					switch0_can_gateway_vcan_port_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Subscribe to the can_status_notify broadcast.
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
	int32_t (*can_status_notify_sub)(
					switch0_can_gateway_can_status_notify_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the can_status_notify broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*can_status_notify_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);
	/**
	 * Subscribe to the can_gateway_recv broadcast.
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
	int32_t (*can_gateway_recv_sub)(
					switch0_can_gateway_can_gateway_recv_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the can_gateway_recv broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*can_gateway_recv_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);
	/**
	 * Subscribe to the vcan_port_broadcast broadcast.
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
	int32_t (*vcan_port_broadcast_sub)(
					switch0_can_gateway_vcan_port_broadcast_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the vcan_port_broadcast broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*vcan_port_broadcast_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);

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
#define switch0_can_gateway_client_t struct _switch0_can_gateway_client_t

/**
 *  The extend data used by the client.
 */
struct _switch0_can_gateway_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t get_ETH_statistics_registry[IPC_TOKEN_NUM];
	callback_registration_t vmac_method_registry[IPC_TOKEN_NUM];
	callback_registration_t vcan_method_registry[IPC_TOKEN_NUM];
	callback_registration_t can_gateway_send_registry[IPC_TOKEN_NUM];
	callback_registration_t can_config_query_registry[IPC_TOKEN_NUM];
	callback_registration_t adas_vcan_notify_sqbuf_addr_registry[IPC_TOKEN_NUM];
	callback_registration_t adas_vcan_driver_remove_registry[IPC_TOKEN_NUM];
	callback_registration_t vcan_port_method_registry[IPC_TOKEN_NUM];
	callback_registration_t can_status_notify_registry;
	callback_registration_t can_gateway_recv_registry;
	callback_registration_t vcan_port_broadcast_registry;
};
#define switch0_can_gateway_client_ext_t struct _switch0_can_gateway_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for switch0_can_gateway_client_t
 * @param ext The data for switch0_can_gateway_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t switch0_can_gateway_client_init(com_client_data_t *data,
			switch0_can_gateway_client_t *client,
			switch0_can_gateway_client_ext_t *ext);

/**
 * Destroys the client.
 */
void switch0_can_gateway_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // SWITCH0_CAN_GATEWAY_CLIENT_H
