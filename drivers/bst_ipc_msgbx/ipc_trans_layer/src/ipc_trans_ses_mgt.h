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

#ifndef _IPC_TRANS_SES_MGT_H
#define _IPC_TRANS_SES_MGT_H

#include <bst/ipc_serdes.h>
#include "ipc_trans_common.h"

#ifdef __cplusplus
extern "C" {
#endif
#define DEFAULT_SES_ID 0
#define SES_INFO_INVALID_VAL 16

#ifdef ENABLE_REMOTE_LOG_PROCESS_FUNC
#define REMOTE_LOG_SES_SID (SESSION_COUNT - 1)
#define REMOTE_LOG_SES_FID (CHANNEL_COUNT - 1)
#define REMOTE_LOG_SES_ID (((REMOTE_LOG_SES_SID) << CHANNEL_COUNT_BITS) | (REMOTE_LOG_SES_FID))
#endif

static inline uint8_t session_comb(uint8_t sid, uint8_t fid)
{
	return ((sid << CHANNEL_COUNT_BITS) | fid);
}

static inline ipc_ses_t *get_session(const uint8_t session_id, void *addr)
{
	// ignore addr check, checking it outside.
	msgbx_end_device_t *module = (msgbx_end_device_t *)addr;
	ipc_ses_t *ses = module->ses_map + session_id;
	if (ses->info.role == MSGBX_SES_ROLE_INVALID)
		return NULL;

	return ses;
}

int8_t session_mgt_init(void *addr);
int8_t session_register(const ses_base_t info, uint8_t *session_id, void *addr);
int8_t session_destroy(ipc_ses_t *ses);
int8_t session_msg_in(const uint8_t session_id, const rw_msg_t *msg,
		      void *addr);
int8_t session_msg_out(ipc_ses_t *ses, serdes_t *msg);
int8_t session_rwmsg_out(ipc_ses_t *ses, rw_msg_t *msg);

#ifdef __cplusplus
}
#endif
#endif
