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

/* This file is auto generated for message box v1.1.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 797e374 msgbx_ipc c468e33
 */

#ifndef CVDSP1_CLIENT_H
#define CVDSP1_CLIENT_H

#define IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#include "cvdsp1_datatype.h"

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
typedef void (*cvdsp1_disp_req_callback_t)(
				const uint32_t rep,
				const cvdsp1_ErrorEnum_t err,
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
typedef void (*cvdsp1_disp_run_callback_t)(
				const uint32_t status,
				const uint32_t perf_us,
				const cvdsp1_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);


// Interface client
struct _cvdsp1_client_t {
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
					cvdsp1_ErrorEnum_t *err,
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
					cvdsp1_disp_req_callback_t cb,
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
					cvdsp1_ErrorEnum_t *err,
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
					cvdsp1_disp_run_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);


	/**
	 * Dispatch broadcast messages.
	 *
	 * @param des The received message package.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_broadcast)(serdes_t *des);

	/**
	 * Dispatch reply messages.
	 *
	 * @param des The received message package.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_reply)(serdes_t *des);
};
#define cvdsp1_client_t struct _cvdsp1_client_t

/**
 *  The extend data used by the client.
 */
struct _cvdsp1_client_ext_t {
	uint8_t cid;
	uint8_t res[7];

};
#define cvdsp1_client_ext_t struct _cvdsp1_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for cvdsp1_client_t
 * @param ext The data for cvdsp1_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t cvdsp1_client_init(com_client_data_t *data,
			cvdsp1_client_t *client,
			cvdsp1_client_ext_t *ext);

/**
 * Destroys the client.
 */
void cvdsp1_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // CVDSP1_CLIENT_H
