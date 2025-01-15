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
 * Generator Version: francaidl a8fd4f2 msgbx_ipc aa5ce6c
 */

#ifndef REALTIME_CLIENT_H
#define REALTIME_CLIENT_H

#define IPC_RTE_KERNEL
#include <bst/ipc_app_client_utils.h>
#include "realtime_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif


// Interface client
struct _realtime_client_t {
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
	 * @param sec The input argument of method realtime_timesync.
	 * @param nsec The input argument of method realtime_timesync.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*realtime_timesync_fire_and_forget)(
					const uint32_t sec,
					const uint32_t nsec
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
#define realtime_client_t struct _realtime_client_t

/**
 *  The extend data used by the client.
 */
struct _realtime_client_ext_t {
	uint8_t cid;
	uint8_t res[7];

};
#define realtime_client_ext_t struct _realtime_client_ext_t

/**
 * Initializes the client.
 *
 * @param data The data for com_client_data_t
 * @param client The data for realtime_client_t
 * @param ext The data for realtime_client_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t realtime_client_init(com_client_data_t *data,
			realtime_client_t *client,
			realtime_client_ext_t *ext);

/**
 * Destroys the client.
 */
void realtime_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // REALTIME_CLIENT_H
