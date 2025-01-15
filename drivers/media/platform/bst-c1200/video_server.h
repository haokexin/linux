/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef VIDEO_SERVER_H
#define VIDEO_SERVER_H

#define IPC_RTE_KERNEL
#include <bst/ipc_app_svr_utils.h>
#include "video_datatype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Stub function for method isp2arm.
 *
 * @param msgAddr The input argument of method isp2arm.
 * @param context The message context, to be passed to the reply function.
 * @param info The extended information, containing uuid and timestamp.
 */
typedef void (*video_isp2arm_t)(const uint32_t msgAddr, const uint64_t context,
				const ext_info_t *info);

// Interface server
struct _video_server_t {
	/**
	 * Get the version of the interface.
	 *
	 * @return The version struct, containing major and minor.
	 */
	ipc_inf_version_t (*version)(void);

	/**
	 * Register stub function for method isp2arm.
	 *
	 * @param func The stub function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_isp2arm)(video_isp2arm_t func);

	/**
	 * Send the reply message of method isp2arm.
	 *
	 * @param result The output argument of method isp2arm.
	 * @param err The error code returned to the client.
	 * @param context The message context, got from the method stub
	 * function.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*reply_isp2arm)(const uint32_t result,
				 const video_error_e_t err,
				 const uint64_t context);

	/**
	 * Send broadcast arm2isp to all subscribers.
	 *
	 * @param msgAddr The output argument of broadcast arm2isp.
	 */
	int32_t (*arm2isp)(uint32_t msgAddr);

	/**
	 * Register subscribed callback function for broadcast arm2isp
	 *
	 * @param func The callback function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_arm2isp_subcribed)(broadcast_sub_t func);

	/**
	 * Register unsubscribed callback function for broadcast arm2isp
	 *
	 * @param func The callback function to be registered.
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*register_arm2isp_unsubcribed)(broadcast_sub_t func);

	/**
	 * Dispatch server messages.
	 *
	 * @return 0 if success, negative if fail.
	 */
	int32_t (*dispatch_request)(serdes_t *des, bool *reply);
};

#define video_server_t struct _video_server_t

/**
 *  The extend data used by the server.
 */
struct _video_server_ext_t {
	video_isp2arm_t isp2arm_ptr;
	broadcast_sub_t arm2isp_sub_ptr;
	broadcast_sub_t arm2isp_unsub_ptr;
	broadcast_registry_t arm2isp_registry;
};

#define video_server_ext_t struct _video_server_ext_t

/**
 * Initializes the server.
 *
 * @param data The data for com_server_data_t
 * @param server The data for test_server_t
 * @param ext The data for test_server_ext_t
 * @return 0 if success, negative if fail.
 */
int32_t video_server_init(com_server_data_t *data, video_server_t *server,
			  video_server_ext_t *ext);

/**
 * Destroys the test client.
 */
void video_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // VIDEO_SERVER_H
