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

#ifndef BST_TOUCH_CLIENT_H
#define BST_TOUCH_CLIENT_H

#define IPC_RTE_KERNEL
#include "ipc_app_client_utils.h"
#include "bst_touch_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for client_request_location_init_async method.
 *
 * @param client_uuid The output argument returned by client_request_location_init_async.
 * @param screen_hwinfo The output argument returned by client_request_location_init_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*bst_touch_client_request_location_init_callback_t)(
				const uint64_t client_uuid,
				const bst_touch_hw_info_t *screen_hwinfo,
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for set_touch_calibration_async method.
 *

 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*bst_touch_set_touch_calibration_callback_t)(
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for get_touch_calibration_async method.
 *
 * @param cali_info The output argument returned by get_touch_calibration_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*bst_touch_get_touch_calibration_callback_t)(
				const bst_touch_calibration_info_t *cali_info,
				const bst_touch_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for broadcast location_info.
 *
 * @param screen_id The output argument returned by broadcast location_info.
 * @param locinfo_offset The output argument returned by broadcast location_info.
 * @param locinfo_size The output argument returned by broadcast location_info.
 * @param locinfo_chksum The output argument returned by broadcast location_info.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*bst_touch_location_info_callback_t)(
				const uint32_t screen_id,
				const uint32_t locinfo_offset,
				const uint32_t locinfo_size,
				const uint32_t locinfo_chksum,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _bst_touch_client_t {
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
	 * @param client_id The input argument of method client_request_location_init.
	 * @param req_info The input argument of method client_request_location_init.
	 * @param client_uuid The output argument of method client_request_location_init.
	 * @param screen_hwinfo The output argument of method client_request_location_init.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*client_request_location_init_sync)(
					const uint32_t client_id,
					const bst_touch_request_info_t *req_info,
					uint64_t *client_uuid,
					bst_touch_hw_info_t **screen_hwinfo,
					bst_touch_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the client_request_location_init method.
	 *
	 * @param client_id The input argument of method client_request_location_init.
	 * @param req_info The input argument of method client_request_location_init.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*client_request_location_init_async)(
					const uint32_t client_id,
					const bst_touch_request_info_t *req_info,
					bst_touch_client_request_location_init_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param screen_id The input argument of method set_touch_calibration.
	 * @param cali_info The input argument of method set_touch_calibration.

	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*set_touch_calibration_sync)(
					const uint32_t screen_id,
					const bst_touch_calibration_info_t *cali_info,
					bst_touch_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the set_touch_calibration method.
	 *
	 * @param screen_id The input argument of method set_touch_calibration.
	 * @param cali_info The input argument of method set_touch_calibration.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*set_touch_calibration_async)(
					const uint32_t screen_id,
					const bst_touch_calibration_info_t *cali_info,
					bst_touch_set_touch_calibration_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	#ifndef IPC_RTE_BAREMETAL
	/**
	 * Synchronously call the hello method.
	 *
	 * @param screen_id The input argument of method get_touch_calibration.
	 * @param cali_info The output argument of method get_touch_calibration.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_touch_calibration_sync)(
					const uint32_t screen_id,
					bst_touch_calibration_info_t **cali_info,
					bst_touch_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);
	#endif

	/**
	 * Asynchronously call the get_touch_calibration method.
	 *
	 * @param screen_id The input argument of method get_touch_calibration.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*get_touch_calibration_async)(
					const uint32_t screen_id,
					bst_touch_get_touch_calibration_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Subscribe to the location_info broadcast.
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
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*location_info_sub)(
					bst_touch_location_info_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					broadcast_sub_unsub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the location_info broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*location_info_unsub)(broadcast_sub_unsub_callback_t cb, void *ext);

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
#define bst_touch_client_t struct _bst_touch_client_t

/**
 *  The extend data used by the client.
 */
struct _bst_touch_client_ext_t {
	uint8_t cid;
	uint8_t res[7];
	callback_registration_t location_info_registry;
};
#define bst_touch_client_ext_t struct _bst_touch_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for bst_touch_client_t
 * @param ext The data for bst_touch_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t bst_touch_client_init(com_client_data_t *data,
			bst_touch_client_t *client,
			bst_touch_client_ext_t *ext);

/**
 * Destroys the client.
 */
void bst_touch_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // BST_TOUCH_CLIENT_H
