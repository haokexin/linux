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

#ifndef TIMESYNCCLIENT_H
#define TIMESYNCCLIENT_H

#include "safety_client.h"
#include "realtime_client.h"
#include "switch_client.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _TimeSyncClient_t {
	safety_client_t safety_client;
	realtime_client_t realtime_client;
	switch_client_t switch_client;

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
#define TimeSyncClient_t struct _TimeSyncClient_t

/**
 *  The internal data used by the client.
 *  Users should define an instance and pass it to the initialization function.
 */
struct _TimeSyncClient_data_t {
	com_client_data_t com_data;
	TimeSyncClient_t client;
	safety_client_ext_t safety_ext;
	realtime_client_ext_t realtime_ext;
	switch_client_ext_t switch_ext;
};
#define TimeSyncClient_data_t struct _TimeSyncClient_data_t

/**
 * Initializes the client.
 *
 * @param data The data to be used by the client.
 * @return A pointer to the initialized client, NULL if fail.
 */
TimeSyncClient_t *TimeSyncClient_init(TimeSyncClient_data_t *ins);

/**
 * Destroys the client.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t TimeSyncClient_destroy(void);

#ifdef __cplusplus
}
#endif

extern TimeSyncClient_t *ts_client;
extern TimeSyncClient_data_t ts_data;

#endif // TIMESYNCCLIENT_H
