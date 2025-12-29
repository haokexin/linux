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

#ifndef _IPC_TRANS_LAYER_H
#define _IPC_TRANS_LAYER_H

#include "ipc_serdes.h"

// this header file is used by ipc_app_layer
int32_t ipc_trans_layer_mmap_session(const uint8_t endid, const uint8_t handle, void** msg_queue_addr);

// server api only
int32_t ipc_trans_layer_stub_create_handle(const uint8_t endid,
					   const uint8_t fid, const uint8_t sid,
					   const uint8_t cid, uint8_t *handle);
int32_t ipc_trans_layer_stub_send_reply_msg(const uint8_t endid,
					    const uint8_t handle,
					    serdes_t *msg);
int32_t ipc_trans_layer_stub_send_broadcast(const uint8_t endid,
					    const uint8_t handle,
					    serdes_t *msg);

int32_t ipc_trans_layer_register_method(const uint8_t endid,
					const uint8_t handle,
					const uint8_t cmd);
int32_t ipc_trans_layer_unregister_method(const uint8_t endid,
					  const uint8_t handle);

// client api only
int32_t ipc_trans_layer_proxy_create_handle(const uint8_t endid,
					    const uint8_t fid, const uint8_t sid,
					    const uint8_t cid, const uint8_t ccid, uint8_t *handle);
int32_t ipc_trans_layer_get_msg(const uint8_t endid,
					    const uint8_t handle,
					    serdes_t *msg);
int32_t ipc_trans_layer_proxy_send_method(const uint8_t endid,
					  const uint8_t handle, serdes_t *msg);

int32_t ipc_trans_layer_destroy_handle(const uint8_t endid,
				       const uint8_t handle);
int32_t ipc_trans_layer_query_info(const uint8_t endid, const uint8_t handle,
						const uint32_t polling_times, const uint8_t is_from_user);
int32_t ipc_trans_layer_release_recv_wait(const uint8_t endid,
					  const uint8_t handle);

// fast path app only
int32_t ipc_trans_layer_create_handle(const uint8_t endid, const uint8_t fid,
				      const uint8_t sid, const uint8_t cid,
				      uint8_t *handle);
int32_t ipc_trans_layer_send_msg(const uint8_t endid, const uint8_t handle,
				 rw_msg_t *msg);
int32_t ipc_trans_layer_get_rwmsg(const uint8_t endid, const uint8_t handle,
				const int32_t timeout, rw_msg_t *msg,
				uint64_t *timestamp);

// endmap mgt spec
int32_t ipc_trans_layer_update_endmap(const uint8_t update_endid, const uint8_t status);
int32_t ipc_trans_layer_get_endmap(const uint8_t endid, sts_endmap_t *map);

// debug
int32_t ipc_trans_layer_get_debug_info(const uint8_t endid, const uint8_t handle, msgbox_debug_info_t *info);

#endif
