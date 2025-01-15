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
 * Generator Version: francaidl 77a2400 msgbx_ipc eb42a92
 */

#ifndef MACSYNCCLIENT_H
#define MACSYNCCLIENT_H

#include "sw_macsync_client.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _MacSyncClient_t {
	sw_macsync_client_t sw_macsync_client;

#ifdef IPC_RTE_BAREMETAL
	/**
	 * Receive a message from the server.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*receive_message)(void);

	/**
	 * Dispatch a message to the server.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_message)(void);
#else
	/**
	 * Start the message router.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*start)(void);

	/**
	 * Stop the message router.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*stop)(void);
#endif
};
#define MacSyncClient_t struct _MacSyncClient_t

/**
 *  The internal data used by the client.
 *  Users should define an instance and pass it to the initialization function.
 */
struct _MacSyncClient_data_t {
	com_client_data_t com_data;
	MacSyncClient_t client;
	sw_macsync_client_ext_t sw_macsync_ext;
};
#define MacSyncClient_data_t struct _MacSyncClient_data_t

/**
 * Initializes the client.
 *
 * @param data The data to be used by the client.
 * @return A pointer to the initialized client, NULL if fail.
 */
MacSyncClient_t *MacSyncClient_init(MacSyncClient_data_t *ins);

/**
 * Destroys the client.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t MacSyncClient_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // MACSYNCCLIENT_H
