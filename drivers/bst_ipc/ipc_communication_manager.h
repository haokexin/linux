/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef IPC_COMMUNICATION_H
#define IPC_COMMUNICATION_H

#include <bst/ipc_interface.h>
#include <linux/kfifo.h>

#include "ipc_common.h"

extern uint64_t signal_map[SUBSCRIPTION_MAP_MAX][MSG_CMD_MAX_IDX];
extern uint16_t register_list[REGISTER_MAP_MAX_IDX][MSG_CMD_MAX_IDX];

// global interface definition
int32_t ipc_drv_send(enum ipc_core_e src, enum ipc_core_e dest,
		     int32_t session_id, void *buf, uint32_t len);
int32_t ipc_drv_recv(enum ipc_core_e src, enum ipc_core_e dest, void *buf,
		     uint32_t len);

int32_t ipc_communication_create(void);
int32_t ipc_communication_close(void);

// use for ipc_sys
extern int32_t core_addr_init(void);
extern int32_t core_addr_exit(void);
#endif
