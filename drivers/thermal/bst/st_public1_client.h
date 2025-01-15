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

/* This file is auto generated for message box v1.0.0.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: francaidl 3a7f767 msgbx_ipc 001bddd
 */

#ifndef ST_PUBLIC1_CLIENT_H
#define ST_PUBLIC1_CLIENT_H

#define IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#include "st_public1_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for qspi_method_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*st_public1_qspi_method_callback_t)(
				const st_public1_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for scmi_method_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*st_public1_scmi_method_callback_t)(
				const st_public1_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for gettemp_method_async method.
 *
 * @param reply_temp The output argument returned by gettemp_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*st_public1_gettemp_method_callback_t)(
				const uint32_t reply_temp,
				const st_public1_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for slt_method_async method.
 *
 * @param name_out The output argument returned by slt_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*st_public1_slt_method_callback_t)(
				const char *name_out,
				const st_public1_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);


// Interface client
struct _st_public1_client_t {
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
	 * @param sec The input argument of method timesync_method.
	 * @param nsec The input argument of method timesync_method.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*timesync_method_fire_and_forget)(
					const uint32_t sec,
					const uint32_t nsec
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param head_msg The input argument of method qspi_method.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*qspi_method_sync)(
					const st_public1_qspi_cmd_head_t *head_msg,
					st_public1_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the qspi_method method.
	 *
	 * @param head_msg The input argument of method qspi_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*qspi_method_async)(
					const st_public1_qspi_cmd_head_t *head_msg,
					st_public1_qspi_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param addr The input argument of method scmi_method.
	 * @param index The input argument of method scmi_method.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*scmi_method_sync)(
					const uint32_t addr,
					const uint32_t index,
					st_public1_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the scmi_method method.
	 *
	 * @param addr The input argument of method scmi_method.
	 * @param index The input argument of method scmi_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*scmi_method_async)(
					const uint32_t addr,
					const uint32_t index,
					st_public1_scmi_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param temp_index The input argument of method gettemp_method.
	 * @param reply_temp The output argument of method gettemp_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*gettemp_method_sync)(
					const uint32_t temp_index,
					uint32_t *reply_temp,
					st_public1_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the gettemp_method method.
	 *
	 * @param temp_index The input argument of method gettemp_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*gettemp_method_async)(
					const uint32_t temp_index,
					st_public1_gettemp_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param name_in The input argument of method slt_method.
	 * @param name_out The output argument of method slt_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*slt_method_sync)(
					const char *name_in,
					char **name_out,
					st_public1_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the slt_method method.
	 *
	 * @param name_in The input argument of method slt_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*slt_method_async)(
					const char *name_in,
					st_public1_slt_method_callback_t cb,
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
#define st_public1_client_t struct _st_public1_client_t

/**
 *  The extend data used by the client.
 */
struct _st_public1_client_ext_t {
	uint8_t cid;
	uint8_t res[7];

};
#define st_public1_client_ext_t struct _st_public1_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for st_public1_client_t
 * @param ext The data for st_public1_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t st_public1_client_init(com_client_data_t *data,
			st_public1_client_t *client,
			st_public1_client_ext_t *ext);

/**
 * Destroys the client.
 */
void st_public1_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // ST_PUBLIC1_CLIENT_H
