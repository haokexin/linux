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

#ifndef _IPC_TRANS_LAYER_H
#define _IPC_TRANS_LAYER_H

#include <linux/completion.h>
#include <bst/bstipc_cfg.h>
#include <bst/ipc_serdes.h>

// this header file is used by ipc_app_layer
// common api
uint8_t get_cpuid_by_endid(uint8_t end_id);
#define CPUID_ERR 0xff

#define ENDID_TO_CPUID(_end_id) ({\
	uint8_t _cpuid = get_cpuid_by_endid(_end_id);\
\
	if (_cpuid == CPUID_ERR) \
		return -1; \
	_cpuid; \
})

// server api only
int32_t ipc_trans_layer_stub_create_handle(const uint8_t endid,
					   const uint8_t fid, const uint8_t sid,
					   const uint8_t cid, uint8_t *handle);
int32_t ipc_trans_layer_stub_get_method_msg(const uint8_t endid,
					    const uint8_t handle,
					    serdes_t *msg);
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
					    const uint8_t fid,
					    const uint8_t sid,
					    const uint8_t cid, uint8_t *handle);
int32_t ipc_trans_layer_proxy_get_reply_msg(const uint8_t endid,
					    const uint8_t handle,
					    serdes_t *msg);
int32_t ipc_trans_layer_proxy_get_broadcast_msg(const uint8_t endid,
						const uint8_t handle,
						serdes_t *msg);
int32_t ipc_trans_layer_proxy_send_method(const uint8_t endid,
					  const uint8_t handle, serdes_t *msg);

int32_t ipc_trans_layer_destroy_handle(const uint8_t endid,
				       const uint8_t handle);
int32_t ipc_trans_layer_query_info(const uint8_t endid, const uint8_t handle, ...);
int32_t ipc_trans_layer_release_recv_wait(const uint8_t endid,
					  const uint8_t handle);
extern void completion_all_init(void);

// fast path app only
int32_t ipc_trans_layer_create_handle(const uint8_t endid, const uint8_t fid,
				      const uint8_t sid, const uint8_t cid,
				      uint8_t *handle);
int32_t ipc_trans_layer_send_msg(const uint8_t endid, const uint8_t handle,
				 rw_msg_t *msg);
int32_t ipc_trans_layer_get_msg(const uint8_t endid, const uint8_t handle,
				const int32_t timeout, rw_msg_t *msg,
				uint64_t *timestamp);

#define QUERY_INFO_HAS_AVAIL_RECV_MSG 0
#define QUERY_INFO_DST_STS_OFFLINE 1
#define QUERY_INFO_DST_STS_ONLINE 2
#endif
