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

/**
 * @file  ipc_trans_runtime.h
 * @brief this file is used as the api to access ipc tranferring layer.
 * this api may be used by ipc_app_layer directly, or related driver developers
 * may implement ipc_trans_impl.c to adapt to different OSs.
 * @note
 * @details feature list
 */
#ifndef _IPC_TRANS_RUNTIME_H
#define _IPC_TRANS_RUNTIME_H

#include <bst/ipc_serdes.h>
#include "../include/ipc_trans_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRANS_LAYER_VERSION 3
/********************* extern global function *******************/
extern int32_t ipc_trans_complete(const uint8_t cpuid, const uint8_t ses_id);
extern int32_t ipc_trans_complete_sts(const uint8_t cpuid);

int32_t ipc_trans_layer_start(const uint8_t endid, uint8_t role);
int32_t ipc_trans_layer_stop(const uint8_t endid);

int32_t ipc_trans_init(const uint8_t role, err_msg_ntf err_func,
		       void *dev_info);
int32_t ipc_trans_deinit(void *dev_info);
int32_t ipc_trans_reinit(void *dev_info);

int32_t ipc_trans_create_session(const uint8_t sid, const uint8_t fid,
				 const uint8_t cid, const uint8_t ccid, const uint8_t role,
				 uint8_t *ses_id, void *dev_info);
int32_t ipc_trans_close_session(const uint8_t ses_id, void *dev_info);

int32_t ipc_trans_read_msg(const uint8_t fid, const uint8_t mode,
			   void *dev_info);

int32_t ipc_trans_send_msg(const uint8_t ses_id, serdes_t *msg,
			   const uint8_t type, void *dev_info);
int32_t ipc_trans_get_msg(const uint8_t ses_id, serdes_t *msg, void *dev_info);

int32_t ipc_trans_register_method(const uint8_t session_id, const uint8_t cmd,
				  void *dev_info);
int32_t ipc_trans_unregister_method(const uint8_t session_id, void *dev_info);

int32_t ipc_trans_get_avail_info(const uint8_t session_id, void *dev_info);

int32_t ipc_trans_err_hdl(const uint8_t type, const uint8_t id,
			  const uint32_t hdl, void *dev_info);

int32_t ipc_trans_get_debug_info(const uint8_t ses_id, debug_info_t *info,
				 void *dev_info);
int32_t ipc_trans_end_sts_broadcast(void *addr);

int32_t ipc_trans_transmit_log(const uint8_t cid, const char *log,
			       void *dev_info);
int32_t ipc_trans_query_remote_endmap(void *dev_info);

int32_t ipc_trans_send_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			     void *dev_info);
int32_t ipc_trans_get_rwmsg(const uint8_t ses_id, rw_msg_t *msg,
			    uint64_t *timestamp, void *dev_info);

int32_t ipc_trans_get_hw_count(const uint8_t fid, msgbox_hw_counter_t* hw_cnt,
				void *dev_info);
int32_t ipc_trans_clr_hw_count(const uint8_t fid, const uint8_t clr_mask, void *dev_info);

int32_t ipc_trans_end_sts_update(const uint8_t end_id, const uint8_t status,
			   const sts_endmap_t *map, void *addr);

int32_t ipc_trans_msg_queue_alloc(const uint8_t session_id, const void* addr, void* dev_info);
int32_t ipc_trans_map_session(const uint8_t ses_id, void **msg_queue_addr, void *dev_info);

#ifdef __cplusplus
}
#endif
#endif
