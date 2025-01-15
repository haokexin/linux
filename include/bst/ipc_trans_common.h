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

#ifndef _IPC_TRANS_COMMON_H
#define _IPC_TRANS_COMMON_H

// this header file define common structure of ipc transferring layer
enum _ipc_msg_type_t {
	MSGBX_MSG_TYPE_INVALID = 0,
	MSGBX_MSG_TYPE_METHOD,
	MSGBX_MSG_TYPE_REPLY,
	MSGBX_MSG_TYPE_BROADCAST,
	MSGBX_MSG_TYPE_PROTOCOL,
	MSGBX_MSG_TYPE_USERDEFINED = 10,
	MSGBX_MSG_TYPE_HARDWARE = 15,
	MSGBX_MSG_TYPE_MAX = 16,
};
#define ipc_msg_type_t enum _ipc_msg_type_t

enum _ipc_ses_role_t {
	MSGBX_SES_ROLE_INVALID = 0,
	MSGBX_SES_ROLE_CLIENT = 1,
	MSGBX_SES_ROLE_SERVER = 2,
	MSGBX_SES_ROLE_FASTPATH = 3,
	MSGBX_SES_ROLE_MAX
};
#define ipc_ses_role_t enum _ipc_ses_role_t

#define MSGBX_END_COUNT_MAX 35U
#define CMD_MAX_COUNT 255U // ro
#define MSGBX_SID_MAX 16U
#endif
