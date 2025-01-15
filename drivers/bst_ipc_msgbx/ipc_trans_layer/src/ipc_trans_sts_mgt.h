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

#ifndef _IPC_TRANS_STS_MGT_H
#define _IPC_TRANS_STS_MGT_H

#include <bst/bstipc_cfg.h>
#include <bst/ipc_hw_layer.h>
#ifdef __cplusplus
extern "C" {
#endif

#define TRANS_PROTO_CMD_ENDMAP_OFFLINE 0
#define TRANS_PROTO_CMD_ENDMAP_ONLINE 1
#define TRANS_PROTO_CMD_LOG 2
#define TRANS_PROTO_CMD_GET_ENDMAP 3

int32_t ipc_end_register(const uint8_t pid, void *addr);
int32_t ipc_end_unregister(const uint8_t pid, void *addr);
int32_t ipc_end_is_ready(const uint8_t end_id, void *addr);
int32_t ipc_end_sts_update(const uint8_t end_id, const uint8_t status,
			   const uint64_t map, void *addr);
#ifdef __cplusplus
}
#endif
#endif
