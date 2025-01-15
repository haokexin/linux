/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef ISP_MSGBX_SERVER_H
#define ISP_MSGBX_SERVER_H

#include "video_server.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _isp_msgbx_server_t {
	video_server_t video_server;

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

#define isp_msgbx_server_t struct _isp_msgbx_server_t

/**
 *  The internal data used by the server.
 *  Users should define an instance and pass it to the server initialization
 * function.
 */
struct _isp_msgbx_server_data_t {
	com_server_data_t com_data;
	isp_msgbx_server_t server;
	video_server_ext_t video_ext;
};

#define isp_msgbx_server_data_t struct _isp_msgbx_server_data_t

/**
 * Initializes the server.
 *
 * @param ins The instance to be initialized.
 * @return A pointer to the initialized server, NULL if fail.
 */
isp_msgbx_server_t *isp_msgbx_server_init(isp_msgbx_server_data_t *ins);
/**
 * Destroys the server.
 *
 * @return 0 if success, negetive if fail.
 */
int32_t isp_msgbx_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // ISP_MSGBX_SERVER_H
