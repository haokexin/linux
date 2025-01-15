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

/* This file is auto generated for message box v0.2.2.
 * All manual modifications will be LOST by next generation.
 * It is recommended NOT modify it.
 * Generator Version: f27fcbb
 */

#ifndef TEST_CLIENT_H
#define TEST_CLIENT_H

#include <bst/ipc_app_client_utils.h>
#include "test_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback function for hello_async method.
 *
 * @param message The output argument returned by hello_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*test_hello_callback_t)(
				const char *message,
				const test_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for complex_method_async method.
 *
 * @param out1 The output argument returned by complex_method_async.
 * @param out2 The output argument returned by complex_method_async.
 * @param out3 The output argument returned by complex_method_async.
 * @param out4 The output argument returned by complex_method_async.
 * @param out5 The output argument returned by complex_method_async.
 * @param out6 The output argument returned by complex_method_async.
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*test_complex_method_callback_t)(
				const uint32_t out1,
				const char *out2,
				const byte_buffer_t out3,
				const test_MyArray_t out4,
				const test_MyStruct_t out5,
				const test_MyUnion_t out6,
				const test_ErrorEnum_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for broadcast heartbeat.
 *
 * @param status The output argument returned by broadcast heartbeat.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 * @note all the data are stored in ext_buf passed to async call.
 * If ext_buf is NULL, internal buffer will be used.
 * Please note that, the internal buffer is shared by all callbacks.
 * The data MAY CHANGED after leaving the callback function.
 */
typedef void (*test_heartbeat_callback_t)(
				const uint8_t status,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for heartbeat_sub method.
 *
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*test_heartbeat_sub_callback_t)(
				int32_t err,
				void *ext,
				const ext_info_t *info
				);

/**
 * Callback function for heartbeat_unsub method.
 *
 * @param err The error code returned by the method.
 * @param ext The user-defined data passed to the method.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*test_heartbeat_unsub_callback_t)(
				int32_t err,
				void *ext,
				const ext_info_t *info
				);

// Interface client
struct _test_client_t {
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
	 * Synchronously call the hello method.
	 *
	 * @param name The input argument of method hello.
	 * @param message The output argument of method hello.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hello_sync)(
					const char *name,
					char **message,
					test_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);

	/**
	 * Asynchronously call the hello method.
	 *
	 * @param name The input argument of method hello.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*hello_async)(
					const char *name,
					test_hello_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Synchronously call the hello method.
	 *
	 * @param in1 The input argument of method complex_method.
	 * @param in2 The input argument of method complex_method.
	 * @param in3 The input argument of method complex_method.
	 * @param in4 The input argument of method complex_method.
	 * @param in5 The input argument of method complex_method.
	 * @param in6 The input argument of method complex_method.
	 * @param out1 The output argument of method complex_method.
	 * @param out2 The output argument of method complex_method.
	 * @param out3 The output argument of method complex_method.
	 * @param out4 The output argument of method complex_method.
	 * @param out5 The output argument of method complex_method.
	 * @param out6 The output argument of method complex_method.
	 * @param err The error code returned by the method.
	 * @param timeout_ms The timeout for the method call in milliseconds, less or equal to 0 means wait forever.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*complex_method_sync)(
					const uint32_t in1,
					const char *in2,
					const byte_buffer_t in3,
					const test_MyArray_t in4,
					const test_MyStruct_t in5,
					const test_MyUnion_t in6,
					uint32_t *out1,
					char **out2,
					byte_buffer_t *out3,
					test_MyArray_t *out4,
					test_MyStruct_t *out5,
					test_MyUnion_t *out6,
					test_ErrorEnum_t *err,
					int64_t timeout_ms,
					des_buf_t *ext_buf
					);

	/**
	 * Asynchronously call the complex_method method.
	 *
	 * @param in1 The input argument of method complex_method.
	 * @param in2 The input argument of method complex_method.
	 * @param in3 The input argument of method complex_method.
	 * @param in4 The input argument of method complex_method.
	 * @param in5 The input argument of method complex_method.
	 * @param in6 The input argument of method complex_method.
	 * @param cb The callback function to be called when the method returns.
	 * @param ext The user-defined data passed to the method.
	 * @param ext_buf The buffer to store the user-defined data.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*complex_method_async)(
					const uint32_t in1,
					const char *in2,
					const byte_buffer_t in3,
					const test_MyArray_t in4,
					const test_MyStruct_t in5,
					const test_MyUnion_t in6,
					test_complex_method_callback_t cb,
					void *ext,
					des_buf_t *ext_buf
					);

	/**
	 * Fire and forget call to the no_reply_method.
	 * This is one way method call. The server will NOT return.
	 *
	 * @param status The input argument of method no_reply_method.
	 * @return 0 if success, negative if fail.
	 * @note This is unreliable transmission, be used ONLY if message losing is accepted.
	 */
	int32_t (*no_reply_method_fire_and_forget)(
					const uint8_t status
					);

	/**
	 * Subscribe to the heartbeat broadcast.
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
	int32_t (*heartbeat_sub)(
					test_heartbeat_callback_t cb,
					void *ext,
					des_buf_t *ext_buf,
					test_heartbeat_sub_callback_t cb2,
					void *ext2
					);

	/**
	 * Unsubscribe from the heartbeat broadcast.
	 *
	 * @param cb The callback function called when the unsubscription is complete.
	 * @param ext The user-defined data passed to the callback.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*heartbeat_unsub)(test_heartbeat_unsub_callback_t cb, void *ext);

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
};
#define test_client_t struct _test_client_t

/**
 * Initializes the test client.
 *
 * @return A pointer to the initialized client, NULL if fail.
 */
test_client_t *test_client_init(void);

/**
 * Destroys the test client.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t test_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_CLIENT_H
