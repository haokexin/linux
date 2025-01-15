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

#ifndef _IPC_TRANS_SES_MGT_H
#define _IPC_TRANS_SES_MGT_H

#include <bst/bstipc_cfg.h>
#include <bst/ipc_serdes.h>
#include <bst/ipc_trans_common.h>
#include "ipc_trans_msg_mgt.h"
#ifdef __cplusplus
extern "C" {
#endif
#define DEFAULT_SES_ID 0
#define SES_UPDATE_STS_BIT 1
#define SES_UPDATE_MSG_BIT 2
#define SES_INFO_INVALID_VAL 16

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
#define REMOTE_LOG_SES_SID (SESSION_COUNT - 1)
#define REMOTE_LOG_SES_FID (CHANNEL_COUNT - 1)
#define REMOTE_LOG_SES_ID (((REMOTE_LOG_SES_SID) << 4) | (REMOTE_LOG_SES_FID))
#endif

struct _ses_base_t {
	uint8_t sid;
	uint8_t fid;
	uint8_t role;
	uint8_t cid;
};
#define ses_base_t struct _ses_base_t

enum _ses_status_t {
	SES_STATE_INVALID = 0,
	SES_STATE_NOT_AVAIL,
	SES_STATE_DESTROY,
	SES_STATE_AVAIL,
};
#define ses_status_t enum _ses_status_t

struct _client_msg_buf_t {
	bst_msg_queue_t reply_buf;
	bst_msg_queue_t signal_buf;
};
#define client_msg_buf_t struct _client_msg_buf_t

struct _server_msg_buf_t {
	bst_msg_queue_t method_buf;
};
#define server_msg_buf_t struct _server_msg_buf_t

struct _ipc_ses_t {
	ses_base_t info;
	ses_status_t status;
	ipc_ses_role_t ses_type;
	uint8_t update_flag;
	uint8_t cid_idx;
	union {
		client_msg_buf_t client_buf;
		server_msg_buf_t server_buf;
	};
};
#define ipc_ses_t struct _ipc_ses_t

static inline int8_t session_comb(uint8_t sid, uint8_t fid, uint8_t *info)
{
	*info = ((sid << 4) | fid);
	return 0;
}

int8_t session_mgt_init(void *addr);
int8_t session_register(const ses_base_t info, uint8_t *session_id, void *addr);
int8_t session_destroy(const uint8_t session_id, void *addr);
int8_t session_msg_in(const uint8_t session_id, const rw_msg_t *msg,
		      void *addr);
int8_t session_msg_out(const uint8_t session_id, const uint8_t type,
		       serdes_t *msg, void *addr);
int8_t session_rwmsg_out(const uint8_t session_id, rw_msg_t *msg, void *addr);

#ifdef __cplusplus
}
#endif
#endif
