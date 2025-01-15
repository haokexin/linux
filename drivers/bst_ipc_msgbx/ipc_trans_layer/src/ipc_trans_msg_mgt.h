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

#ifndef _IPC_TRANS_MSG_MGT_H
#define _IPC_TRANS_MSG_MGT_H

#include <bst/bstipc_cfg.h>
#include <bst/ipc_serdes.h>
#include "../include/config.h"
#include "ipc_trans_lflist.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _msg_object_t {
	rw_msg_t msg;
	uint64_t timestamp;
	uint16_t err_cnt;
	uint8_t res[6];
};
#define msg_object_t struct _msg_object_t

struct _packet_object_t {
	lflist_t packed_list;
	uint64_t mask;
	uint8_t completed_flag;
	uint8_t last_idx;
	uint16_t err_cnt;
	uint8_t res[4];
};
#define packet_object_t struct _packet_object_t

struct _bst_msg_queue_t {
	packet_object_t packet_obj[SESSION_PACKET_MSG_BUFFER_COUNT];
	msg_object_t msg_obj[SESSION_MSG_BUFFER_COUNT];
	lflist_node_t msg_nodes[SESSION_MSG_BUFFER_COUNT];
	lflist_node_t packet_nodes[SESSION_PACKET_MSG_BUFFER_COUNT];
	lflist_t free_list;
	lflist_t used_list;
#ifdef ENABLE_MSG_ERR_PROCESS
	lflist_t err_list;
#endif
	lflist_t free_pack_list;
	lflist_t used_pack_list;
};
#define bst_msg_queue_t struct _bst_msg_queue_t

int8_t msg_queue_init(bst_msg_queue_t *msg_queue);
int8_t msg_queue_in(bst_msg_queue_t *msg_queue, rw_msg_t *msg,
		    uint64_t timestamp);
int8_t msg_queue_out(bst_msg_queue_t *msg_queue, serdes_t *out);
int8_t rwmsg_queue_out(bst_msg_queue_t *msg_queue, rw_msg_t *out);
int8_t msg_queue_collate(bst_msg_queue_t *msg_queue, uint8_t is_err_handle);

#ifdef __cplusplus
}
#endif
#endif
