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

#ifndef SAMPLE_SERVER_H
#define SAMPLE_SERVER_H

#include "test_server.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _sample_server_t {
	test_server_t test_server;

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
#define sample_server_t struct _sample_server_t


/**
 *  The internal data used by the server.
 *  Users should define an instance and pass it to the server initialization function.
 */
struct _sample_server_data_t {
	com_server_data_t com_data;
	sample_server_t server;
	test_server_ext_t test_ext;
};
#define sample_server_data_t struct _sample_server_data_t

/**
 * Initializes the server.
 *
 * @param ins The instance to be initialized.
 * @return A pointer to the initialized server, NULL if fail.
 */
sample_server_t *sample_server_init(sample_server_data_t *ins);
/**
 * Destroys the server.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t sample_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // SAMPLE_SERVER_H
