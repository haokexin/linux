/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
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

#ifndef _IPC_TRANS_CONFIG_H
#define _IPC_TRANS_CONFIG_H

// developer config value ---------------------------------------------------------------------
#include <bst/bstipc_cfg.h>

/*
 *   define the central monitor end id
 */
#define CENTRAL_MONITOR_END_ID SAFETY_0 // release setting

/* state management function enable flag
 * note: you could select which error interrupt message will be notified to trans_layer
 * note: below is full function definition
 */
#ifdef IPC_STATE_MGT_ENABLE
#define MSG_END_MGT_CONFIG                                                     \
	(MSGBX_ECC_RX_MULTIP_EN_BIT | MSGBX_ECC_RX_DETECT_EN_BIT |             \
	 MSGBX_PARITY_HWDATA_EN_BIT | MSGBX_PARITY_HADDR_EN_BIT)
#define MSG_DEF_FLT_MGT_CONFIG                                                 \
	(MSGBX_TX_OVERFLOW_EN_BIT | MSGBX_RX_OVERFLOW_EN_BIT |                 \
	 MSGBX_RX_UNDERFLOW_EN_BIT)
#define MSG_FLT_MGT_CONFIG                                                     \
	(MSGBX_RX_OVERFLOW_EN_BIT | MSGBX_RX_UNDERFLOW_EN_BIT)
#endif

/* buffer size for each session
 * MUST: IF YOU USE THIS MODE, YOUR SERVER SID MUST BE 0
 * note: channel_count * session_count = support max sessions
 */
#ifdef BAREMETAL_VERSION_TRUNCATE
#define SESSION_MSG_BUFFER_COUNT 8
#define SESSION_PACKET_MSG_BUFFER_COUNT 1
#define CHANNEL_COUNT 1 // rw
#define SESSION_COUNT 2 // rw
#else
#define CHANNEL_COUNT CONFIG_MSGBOX_CHANNEL_COUNT   // rw
#define SESSION_COUNT 16 // rw
#define SESSION_MSG_BUFFER_COUNT 900
#define SESSION_PACKET_MSG_BUFFER_COUNT 800
#define ENABLE_MSG_ERR_PROCESS
#define DEBUG_MODE_ENABLE
#define TIMESTAMP_DEBUG_ENABLE
#ifdef CONFIG_BST_C1200_ADAS
#define ENABLE_REMOTE_LOG_PROCESS_FUNC
#endif
#define USE_EXTERNAL_MSG_BUFFER
#endif

/* Define the IPC receive mode
 * 1 means interrupt; 2 means polling
 */
#define IPC_RECV_MODE 1

/* Define the maximum count for session error message processing
 */
#define SESSION_ERR_MSG_PROCESS_MAX_COUNT 512U

#endif
