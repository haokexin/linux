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

#ifndef NET_SAFETY_CLIENT_H
#define NET_SAFETY_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "net_safety_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for fusaenable_method_async method.
 *
 * @param block_id_out The output argument returned by fusaenable_method_async.
 * @param psm_id The output argument returned by fusaenable_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*net_safety_fusaenable_method_callback_t)(
				const uint8_t block_id_out,
				const net_safety_UInt32Array4_t *psm_id,
				const net_safety_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);


// Interface client
struct _net_safety_client_t {
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
	 * @param block_id The input argument of method fusaenable_method.
	 * @param block_id_out The output argument of method fusaenable_method.
	 * @param psm_id The output argument of method fusaenable_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*fusaenable_method_sync)(
					const uint8_t block_id,
					uint8_t *block_id_out,
					net_safety_UInt32Array4_t **psm_id,
					net_safety_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the fusaenable_method method.
	 *
	 * @param block_id The input argument of method fusaenable_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*fusaenable_method_async)(
					const uint8_t block_id,
					net_safety_fusaenable_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);


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
#define net_safety_client_t struct _net_safety_client_t

/**
 *  The extend data used by the client.
 */
struct _net_safety_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t fusaenable_method_registry[IPC_TOKEN_NUM];

};
#define net_safety_client_ext_t struct _net_safety_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for net_safety_client_t
 * @param ext The data for net_safety_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t net_safety_client_init(com_client_data_t *data,
			net_safety_client_t *client,
			net_safety_client_ext_t *ext);

/**
 * Destroys the client.
 */
void net_safety_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // NET_SAFETY_CLIENT_H
