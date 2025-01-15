// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
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

/**
 * @file  ipc_trans_msg_mgt.c
 * @brief This file contains the implementation of the IPC message management
 * layer.
 *
 * @details Feature list:
 * 1. Message Queue Initialization
 * 2. Message Queue Input
 * 3. Message Queue Output
 * 4. Message Queue Collation
 *
 * @note This file is part of the IPC transport layer. It is responsible for managing the message queues used for
 * communication.
 */
#include "../include/config.h"
#include "ipc_trans_msg_mgt.h"

/********************* local variables ***************************/
#define MSG_HEAD_EXC_IDX_BIT_OFFSET 0xFFF00FFFFFB0FFFF
#define GET_HEADER_MASK(header) (header & MSG_HEAD_EXC_IDX_BIT_OFFSET)

union _header_union {
	rw_msg_header_t header;
	uint64_t as_uint64_t;
};
#define header_union union _header_union

int8_t msg_queue_init(bst_msg_queue_t *msg_queue)
{
	uint16_t cnt = 0;

	if (!msg_queue)
		return -1;

	lflist_init(&msg_queue->free_list);
	lflist_init(&msg_queue->used_list);
	lflist_init(&msg_queue->free_pack_list);
	lflist_init(&msg_queue->used_pack_list);
#ifdef ENABLE_MSG_ERR_PROCESS
	lflist_init(&msg_queue->err_list);
#endif

	for (cnt = 0; cnt < SESSION_MSG_BUFFER_COUNT; ++cnt) {
		ipc_memset(&msg_queue->msg_obj[cnt], 0, sizeof(msg_object_t));
		msg_queue->msg_nodes[cnt].data = &msg_queue->msg_obj[cnt];
		lflist_enqueue(&msg_queue->free_list,
			       &msg_queue->msg_nodes[cnt]);
	}

	for (cnt = 0; cnt < SESSION_PACKET_MSG_BUFFER_COUNT; ++cnt) {
		ipc_memset(&msg_queue->packet_obj[cnt], 0,
			   sizeof(packet_object_t));
		lflist_init(&msg_queue->packet_obj[cnt].packed_list);
		msg_queue->packet_nodes[cnt].data = &msg_queue->packet_obj[cnt];
		lflist_enqueue(&msg_queue->free_pack_list,
			       &msg_queue->packet_nodes[cnt]);
	}
	return 0;
}

int8_t msg_queue_in(bst_msg_queue_t *msg_queue, rw_msg_t *msg,
		    uint64_t timestamp)
{
	lflist_node_t *node = NULL;
	msg_object_t *pval = NULL;

	if (!msg_queue || !msg)
		return -1;

	// take a free node
	node = lflist_dequeue(&msg_queue->free_list);
	if (!node)
		return -2;

	// set value
	pval = (msg_object_t *)node->data;
	pval->msg = *msg;
	pval->timestamp = timestamp;
	pval->err_cnt = 0;

	// put to used list
	lflist_enqueue(&msg_queue->used_list, node);
	return 0;
}

int8_t msg_queue_out(bst_msg_queue_t *msg_queue, serdes_t *out)
{
	int8_t ret = -1;
	rw_msg_t *msg = NULL;
	lflist_node_t *pre_node = NULL;
	lflist_node_t *next_node = NULL;
	lflist_node_t *result = NULL;
	packet_object_t *obj = NULL;
#ifdef ENABLE_MSG_ERR_PROCESS
	lflist_node_t *err_node = NULL;
#endif
	packet_object_t *out_obj = NULL;
	lflist_node_t *node = NULL;
	msg_object_t *msg_obj = NULL;
	uint8_t first_frame_flag = 0;

	if (!msg_queue || !out)
		return ret;

	ipc_des_init(out);

	// poll used_pack_listc
	pre_node = msg_queue->used_pack_list.tail;
	next_node = pre_node->next;

	while (next_node) {
		obj = (packet_object_t *)next_node->data;
		if (!obj)
			break;
		if (obj->completed_flag == 1) {
			// take out this node
			pre_node->next = next_node->next;
			if (!pre_node->next)
				msg_queue->used_pack_list.head = pre_node;
			result = next_node;
			result->next = ((void *)0);
			ret = 0;
			break;
		}
#ifdef ENABLE_MSG_ERR_PROCESS
		// err handle
		obj->err_cnt++;
		if (obj->err_cnt == SESSION_ERR_MSG_PROCESS_MAX_COUNT) {
			IPC_LOG_WARNING(
				"packet msg mask %llu reach max process count",
				obj->mask);

			pre_node->next = next_node->next;
			if (!pre_node->next)
				msg_queue->used_pack_list.head = pre_node;

			while (1) {
				err_node = lflist_dequeue(&obj->packed_list);
				if (!err_node)
					break;
				lflist_enqueue(&msg_queue->free_list, err_node);
			}
			lflist_enqueue(&msg_queue->free_pack_list, next_node);
		}
#endif
		pre_node = next_node;
		next_node = next_node->next;
	};

	if (ret == 0) {
		// msg serialization
		while (1) {
			out_obj = (packet_object_t *)result->data;
			node = lflist_dequeue(&out_obj->packed_list);
			if (!node)
				break;

			// pop out node
			msg = ipc_des_get_current_msg(out);
			msg_obj = node->data;
			if (!first_frame_flag) {
				out->recv_start_time = msg_obj->timestamp;
				first_frame_flag = 1;
			}
			*msg = msg_obj->msg;
			ret = ipc_des_validate_msg(out);
			if (ret == -2)
				IPC_LOG_WARNING(
					"msg out validate fail, drop msg header is %llu",
					obj->mask);

			out->recv_end_time = msg_obj->timestamp;
			// put to free list
			lflist_enqueue(&msg_queue->free_list, node);
		}
		// put to free pakcet list
		lflist_enqueue(&msg_queue->free_pack_list, result);
		return ret;
	}
	// do not have available message
	return -3;
}

int8_t msg_queue_collate(bst_msg_queue_t *msg_queue, uint8_t is_err_handle)
{
	uint8_t complete_flag = 0;
	int8_t ret = -1;
	lflist_node_t *node = NULL;
	msg_object_t *msg_obj = NULL;
	rw_msg_t *msg = NULL;
	lflist_node_t *packet_node = NULL;
	packet_object_t *pval = NULL;
	uint64_t msg_head = 0;
	header_union tmp_value = { 0 };
	lflist_node_t *pre_node = NULL;
	uint64_t mask = 0;
	lflist_node_t *next_node = NULL;
	packet_object_t *obj = NULL;

	if (!msg_queue)
		return ret;

	// empty used_msg_list
	while (1) {
		// take a used node
#ifdef ENABLE_MSG_ERR_PROCESS
		if (!is_err_handle)
			node = lflist_dequeue(&msg_queue->used_list);
		else
			node = lflist_dequeue(&msg_queue->err_list);
#else
		node = lflist_dequeue(&msg_queue->used_list);
#endif
		if (!node)
			break;

		// msg collation
		msg_obj = node->data;
		msg = &msg_obj->msg;
		do {
			if (msg->header.idx == 0) {
				// new node
				packet_node = lflist_dequeue(
					&msg_queue->free_pack_list);
				if (!packet_node) {
					IPC_LOG_WARNING(
						"msg queue don't have empty pack node, drop msg pid: %d, typ: %d, tok: %d, cmd: %d, idx: %d\n",
						msg->header.pid,
						msg->header.typ,
						msg->header.tok,
						msg->header.cmd,
						msg->header.idx);
					ret = -2;
					break;
				}

				// set value
				pval = packet_node->data;
				tmp_value.header = msg->header;
				msg_head = tmp_value.as_uint64_t;
				pval->mask = GET_HEADER_MASK(msg_head);
				pval->last_idx = msg->header.idx;
				pval->err_cnt = 0;
				lflist_enqueue(&pval->packed_list, node);
				if (msg->header.is_eof == 1) {
					pval->completed_flag = 1;
					complete_flag = 1;
				} else
					pval->completed_flag = 0;

				// put to packet used list
				lflist_enqueue(&msg_queue->used_pack_list,
					       packet_node);
				ret = 0;
				break;
			}
			// remaining frame search node
			tmp_value.header = msg->header;
			msg_head = tmp_value.as_uint64_t;
			mask = GET_HEADER_MASK(msg_head);
			pre_node = msg_queue->used_pack_list.tail;
			next_node = pre_node->next;
			while (next_node) {
				obj = (packet_object_t *)next_node->data;
				if (obj->mask == mask &&
				    obj->last_idx + 1 == msg->header.idx) {
					// add node to pakced_list
					obj->last_idx = msg->header.idx;
					lflist_enqueue(&obj->packed_list, node);

					// check completed flag
					if (msg->header.is_eof == 1) {
						obj->completed_flag = 1;
						complete_flag = 1;
					}
					ret = 0;
					break;
				}
				pre_node = next_node;
				next_node = next_node->next;
			}
		} while (0);

#ifdef ENABLE_MSG_ERR_PROCESS
		if (ret < 0) {
			// enter err msg list
			IPC_LOG_INFO(
				"msg pid: %d, typ: %d, tok: %d, cmd: %d idx: %d enter err list\n",
				msg->header.pid, msg->header.typ,
				msg->header.tok, msg->header.cmd,
				msg->header.idx);
			msg_obj->err_cnt++;
			if (msg_obj->err_cnt ==
			    SESSION_ERR_MSG_PROCESS_MAX_COUNT) {
				IPC_LOG_WARNING(
					"msg pid: %d, typ: %d, cmd: %d, tok: %d, idx: %d reach max process count, we will drop this message\n",
					msg->header.pid, msg->header.typ,
					msg->header.cmd, msg->header.tok,
					msg->header.idx);
				lflist_enqueue(&msg_queue->free_list, node);
			} else {
				lflist_enqueue(&msg_queue->err_list, node);
			}
		}
#endif
	}
	if (complete_flag == 1)
		return 0;
	else
		return -2;
}
int8_t rwmsg_queue_out(bst_msg_queue_t *msg_queue, rw_msg_t *out)
{
	lflist_node_t *node = NULL;
	msg_object_t *msg_obj = NULL;

	node = lflist_dequeue(&msg_queue->used_list);
	if (!node)
		return -1;

	msg_obj = node->data;
	*out = msg_obj->msg;
	return 0;
}
