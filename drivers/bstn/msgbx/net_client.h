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

#ifndef NET_CLIENT_H
#define NET_CLIENT_H

#define IPC_RTE_KERNEL
#ifdef IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#else
#include "ipc_app_client_utils.h"
#endif
#include "net_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for disp_req_async method.
 *
 * @param rep The output argument returned by disp_req_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*net_disp_req_callback_t)(
				const uint32_t rep,
				const net_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for disp_run_async method.
 *
 * @param status The output argument returned by disp_run_async.
 * @param perf_us The output argument returned by disp_run_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*net_disp_run_callback_t)(
				const uint32_t status,
				const uint32_t perf_us,
				const net_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);


// Interface client
struct _net_client_t {
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
	 * @param req The input argument of method disp_req.
	 * @param rep The output argument of method disp_req.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*disp_req_sync)(
					const uint32_t req,
					uint32_t *rep,
					net_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the disp_req method.
	 *
	 * @param req The input argument of method disp_req.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*disp_req_async)(
					const uint32_t req,
					net_disp_req_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param opcode The input argument of method disp_run.
	 * @param opdata The input argument of method disp_run.
	 * @param status The output argument of method disp_run.
	 * @param perf_us The output argument of method disp_run.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*disp_run_sync)(
					const uint32_t opcode,
					const uint32_t opdata,
					uint32_t *status,
					uint32_t *perf_us,
					net_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the disp_run method.
	 *
	 * @param opcode The input argument of method disp_run.
	 * @param opdata The input argument of method disp_run.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*disp_run_async)(
					const uint32_t opcode,
					const uint32_t opdata,
					net_disp_run_callback_t cb,
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
#define net_client_t struct _net_client_t

/**
 *  The extend data used by the client.
 */
struct _net_client_ext_t {
	uint8_t cid;
	uint8_t ccid;
	uint8_t status;
	uint8_t res[5];
	uint64_t cid_mask;
	avail_changed_callback_t avail_changed_cb;
	void *avail_ext;
	callback_registration_t disp_req_registry[IPC_TOKEN_NUM];
	callback_registration_t disp_run_registry[IPC_TOKEN_NUM];

};
#define net_client_ext_t struct _net_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for net_client_t
 * @param ext The data for net_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t net_client_init(com_client_data_t *data,
			net_client_t *client,
			net_client_ext_t *ext);

/**
 * Destroys the client.
 */
void net_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // NET_CLIENT_H
