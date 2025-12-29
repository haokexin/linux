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

#ifndef BACKLIGHT_CLIENT_H
#define BACKLIGHT_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "backlight_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for virt_bl_request_async method.
 *
 * @param rsp The output argument returned by virt_bl_request_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*backlight_virt_bl_request_callback_t)(
				const backlight_virt_bl_msg_t rsp,
				const backlight_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for broadcast virt_bl_broadcast.
 *
 * @param evt The output argument returned by broadcast virt_bl_broadcast.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*backlight_virt_bl_broadcast_callback_t)(
				const backlight_virt_bl_msg_t evt,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _backlight_client_t {
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
	 * @param req The input argument of method virt_bl_request.
	 * @param rsp The output argument of method virt_bl_request.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*virt_bl_request_sync)(
					const backlight_virt_bl_msg_t req,
					backlight_virt_bl_msg_t *rsp,
					backlight_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the virt_bl_request method.
	 *
	 * @param req The input argument of method virt_bl_request.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*virt_bl_request_async)(
					const backlight_virt_bl_msg_t req,
					backlight_virt_bl_request_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Subscribe to the virt_bl_broadcast broadcast.
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
	int32_t (*virt_bl_broadcast_sub)(
					backlight_virt_bl_broadcast_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the virt_bl_broadcast broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 * @note if the subscription is not found, return -1.
	 * @note unsubscribe multiple times will result in multiple callbacks.
	 * cb and ext will not be affected by multiple unsubscriptions.
	 */
	int32_t (*virt_bl_broadcast_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);

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
#define backlight_client_t struct _backlight_client_t

/**
 *  The extend data used by the client.
 */
struct _backlight_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t virt_bl_request_registry[IPC_TOKEN_NUM];
	callback_registration_t virt_bl_broadcast_registry;
};
#define backlight_client_ext_t struct _backlight_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for backlight_client_t
 * @param ext The data for backlight_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t backlight_client_init(com_client_data_t *data,
			backlight_client_t *client,
			backlight_client_ext_t *ext);

/**
 * Destroys the client.
 */
void backlight_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // BACKLIGHT_CLIENT_H
