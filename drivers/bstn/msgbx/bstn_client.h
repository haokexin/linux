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

#ifndef BSTN_CLIENT_H
#define BSTN_CLIENT_H

#include "net_client.h"
#include "net_safety_client.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _bstn_client_t {
	net_client_t net_client;
	net_safety_client_t net_safety_client;

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
#define bstn_client_t struct _bstn_client_t

/**
 *  The internal data used by the client.
 *  Users should define an instance and pass it to the initialization function.
 */
struct _bstn_client_data_t {
	com_client_data_t com_data;
	bstn_client_t client;
	net_client_ext_t net_ext;
	net_safety_client_ext_t net_safety_ext;
};
#define bstn_client_data_t struct _bstn_client_data_t

/**
 * Initializes the client.
 *
 * @param data The data to be used by the client.
 * @return A pointer to the initialized client, NULL if fail.
 */
bstn_client_t *bstn_client_init(bstn_client_data_t *ins);

/**
 * Destroys the client.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t bstn_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // BSTN_CLIENT_H
