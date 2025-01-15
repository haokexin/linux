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

#ifndef LWNN_CLIENT_H
#define LWNN_CLIENT_H

#include "cvdsp0_client.h"
#include "cvdsp1_client.h"
#include "cvdsp2_client.h"
#include "cvdsp3_client.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _lwnn_client_t {
	cvdsp0_client_t cvdsp0_client;
	cvdsp1_client_t cvdsp1_client;
	cvdsp2_client_t cvdsp2_client;
	cvdsp3_client_t cvdsp3_client;

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
#define lwnn_client_t struct _lwnn_client_t

/**
 *  The internal data used by the client.
 *  Users should define an instance and pass it to the initialization function.
 */
struct _lwnn_client_data_t {
	com_client_data_t com_data;
	lwnn_client_t client;
	cvdsp0_client_ext_t cvdsp0_ext;
	cvdsp1_client_ext_t cvdsp1_ext;
	cvdsp2_client_ext_t cvdsp2_ext;
	cvdsp3_client_ext_t cvdsp3_ext;
};
#define lwnn_client_data_t struct _lwnn_client_data_t

/**
 * Initializes the client.
 *
 * @param data The data to be used by the client.
 * @return A pointer to the initialized client, NULL if fail.
 */
lwnn_client_t *lwnn_client_init(lwnn_client_data_t *ins);

/**
 * Destroys the client.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t lwnn_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // LWNN_CLIENT_H
