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

#ifndef TEST_SERVER_H
#define TEST_SERVER_H

#define IPC_RTE_KERNEL
#include <bst/ipc_app_svr_utils.h>
#include "test_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Stub function for method hello.
 *
 * @param name The input argument of method hello.
 * @param context The message context, to be passed to the reply function.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*test_hello_t)(
				const char *name,
				const uint64_t context,
				const ext_info_t *info
				);
/**
 * Stub function for method complex_method.
 *
 * @param in1 The input argument of method complex_method.
 * @param in2 The input argument of method complex_method.
 * @param in3 The input argument of method complex_method.
 * @param in4 The input argument of method complex_method.
 * @param in5 The input argument of method complex_method.
 * @param in6 The input argument of method complex_method.
 * @param context The message context, to be passed to the reply function.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*test_complex_method_t)(
				const uint32_t in1,
				const char *in2,
				const byte_buffer_t in3,
				const test_MyArray_t in4,
				const test_MyStruct_t in5,
				const test_MyUnion_t in6,
				const uint64_t context,
				const ext_info_t *info
				);
/**
 * Stub function for method no_reply_method.
 *
 * @param status The input argument of method no_reply_method.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*test_no_reply_method_t)(
				const uint8_t status,
				const ext_info_t *info
				);

// Interface server
struct _test_server_t {
	/**
	 * Get the version of the interface.
	 *
	 * @return The version struct, containing major and minor.
	 */
	ipc_inf_version_t (*version)(void);

	/**
	 * Register stub function for method hello.
	 *
	 * @param func The stub function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_hello)(test_hello_t func);

	/**
	 * Send the reply message of method hello.
	 *
	 * @param message The output argument of method hello.
	 * @param err The error code returned to the client.
	 * @param context The message context, got from the method stub function.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*reply_hello)(
					const char *message,
					const test_ErrorEnum_t err,
					const uint64_t context);

	/**
	 * Register stub function for method complex_method.
	 *
	 * @param func The stub function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_complex_method)(test_complex_method_t func);

	/**
	 * Send the reply message of method complex_method.
	 *
	 * @param out1 The output argument of method complex_method.
	 * @param out2 The output argument of method complex_method.
	 * @param out3 The output argument of method complex_method.
	 * @param out4 The output argument of method complex_method.
	 * @param out5 The output argument of method complex_method.
	 * @param out6 The output argument of method complex_method.
	 * @param err The error code returned to the client.
	 * @param context The message context, got from the method stub function.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*reply_complex_method)(
					const uint32_t out1,
					const char *out2,
					const byte_buffer_t out3,
					const test_MyArray_t out4,
					const test_MyStruct_t out5,
					const test_MyUnion_t out6,
					const test_ErrorEnum_t err,
					const uint64_t context);

	/**
	 * Register stub function for method no_reply_method.
	 *
	 * @param func The stub function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_no_reply_method)(test_no_reply_method_t func);

	/**
	 * Send broadcast heartbeat to all subscribers.
	 *
	 * @param status The output argument of broadcast heartbeat.
	 */
	int32_t (*heartbeat)(uint8_t status);

	/**
	 * Register subscribed callback function for broadcast heartbeat
	 *
	 * @param func The callback function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_heartbeat_subcribed)(broadcast_sub_t func);

	/**
	 * Register unsubscribed callback function for broadcast heartbeat
	 *
	 * @param func The callback function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_heartbeat_unsubcribed)(broadcast_sub_t func);

	/**
	 * Dispatch server messages.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_request)(serdes_t *des, bool *reply);
};
#define test_server_t struct _test_server_t

/**
 *  The extend data used by the server.
 */
struct _test_server_ext_t {
	test_hello_t hello_ptr;
	test_complex_method_t complex_method_ptr;
	test_no_reply_method_t no_reply_method_ptr;
	broadcast_sub_t heartbeat_sub_ptr;
	broadcast_sub_t heartbeat_unsub_ptr;
	broadcast_registry_t heartbeat_registry;
};
#define test_server_ext_t struct _test_server_ext_t

/**
 * Initializes the server.
 *
 * @param data The data for com_server_data_t
 * @param server The data for test_server_t
 * @param ext The data for test_server_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t test_server_init(com_server_data_t *data,
			test_server_t *server,
			test_server_ext_t *ext);

/**
 * Destroys the test client.
 */
void test_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_SERVER_H
