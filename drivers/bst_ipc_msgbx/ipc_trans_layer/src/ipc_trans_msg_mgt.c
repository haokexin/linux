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
#include "ipc_trans_msg_mgt.h"

/********************* local variables ***************************/
#if defined(MSGBX_HW_TYPE_C1200)
// except idx, res, is_eof, len
#define MSG_HEAD_EXC_IDX_BIT_OFFSET 0xFFF00FFFFFB0FFFF
// include typ, fid, sid ,pid
#define MSG_HEAD_COLLATE_OFFSET 0xF00FF0000FF
#elif defined(MSGBX_HW_TYPE_A2000)
#define MSG_HEAD_EXC_IDX_BIT_OFFSET 0xFFFF7FFFFF0FFFF
#define MSG_HEAD_COLLATE_OFFSET 0x7FF00C000FF
#endif
#define GET_HEADER_UUID(header) (header & MSG_HEAD_COLLATE_OFFSET)
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
	simplist_init(&msg_queue->free_pack_list);
	simplist_init(&msg_queue->uncompleted_pack_list);
	simplist_init(&msg_queue->completed_pack_list);

	for (cnt = 0; cnt < SESSION_MSG_BUFFER_COUNT; ++cnt) {
		// the objects are inited outside.
		// ipc_memset(&msg_queue->msg_obj[cnt], 0, sizeof(msg_object_t));
		msg_queue->msg_nodes[cnt].data = &msg_queue->msg_obj[cnt];
		lflist_enqueue(&msg_queue->free_list,
			       &msg_queue->msg_nodes[cnt]);
	}

	for (cnt = 0; cnt < SESSION_PACKET_MSG_BUFFER_COUNT; ++cnt) {
		// ipc_memset(&msg_queue->packet_obj[cnt], 0,
		// 	   sizeof(packet_object_t));
		simplist_init(&msg_queue->packet_obj[cnt].packed_list);
		simplist_enqueue(&msg_queue->free_pack_list,
				 &msg_queue->packet_obj[cnt].node);
	}
	return 0;
}

int8_t msg_queue_in(bst_msg_queue_t *msg_queue, rw_msg_t *msg,
		    uint64_t timestamp)
{
	lflist_node_t *node = NULL;
	msg_object_t *obj = NULL;

	if (!msg_queue || !msg)
		return -1;

	node = lflist_dequeue(&msg_queue->free_list);
	if (!node)
		return -2;

	obj = (msg_object_t *)node->data;
	obj->msg = *msg;
	obj->timestamp = timestamp;

	lflist_enqueue(&msg_queue->used_list, node);
	return 0;
}

int8_t msg_queue_out(bst_msg_queue_t *msg_queue, serdes_t *out)
{
	packet_object_t *packet_obj = NULL;
	lflist_node_t *msg_node = NULL;
	msg_object_t *msg_obj = NULL;
	rw_msg_t *msg = NULL;
	uint8_t first_frame_flag = 0;

	if (!msg_queue || !out)
		return -1;

	ipc_des_init(out);
	packet_obj = (packet_object_t *)simplist_dequeue(&msg_queue->completed_pack_list);
	if (!packet_obj)
		return -2;

	while (1) {
		msg_node = simplist_dequeue(&packet_obj->packed_list);
		if (!msg_node)
			break;
		msg_obj = msg_node->data;
		if (!first_frame_flag) {
			out->recv_start_time = msg_obj->timestamp;
			first_frame_flag = 1;
		}
		msg = &out->msg_pool[msg_obj->msg.header.idx];
		*msg = msg_obj->msg;
		if (msg->header.is_eof == 1) {
			out->rcv_index = msg->header.idx;
			out->header = msg->header;
			out->recv_end_time = msg_obj->timestamp;
		}
		lflist_enqueue(&msg_queue->free_list, msg_node);
	}
	simplist_enqueue(&msg_queue->free_pack_list, (lflist_node_t *)packet_obj);
	return 0;
}

int8_t msg_queue_recycle(void *ses)
{
	packet_object_t *packet_obj = NULL;
	lflist_node_t *msg_node = NULL;
	bst_msg_queue_t *msg_queue = NULL;
	ipc_ses_t *ses_addr = NULL;
#ifdef DEBUG_MODE_ENABLE
	debug_info_t *dbg_info = NULL;
#endif

	if (!ses)
		return -1;

	ses_addr = (ipc_ses_t *)ses;
#if defined(USE_EXTERNAL_MSG_BUFFER)
	msg_queue = ses_addr->msg_queue;
#else
	msg_queue = &ses_addr->msg_queue;
#endif
#ifdef DEBUG_MODE_ENABLE
	dbg_info = &ses_addr->debug_info;
#endif

	if (!msg_queue)
		return -1;

	while (1) {
		packet_obj = (packet_object_t *)simplist_dequeue(&msg_queue->completed_pack_list);
		if (!packet_obj)
			break;	
		while (1) {
			msg_node = simplist_dequeue(&packet_obj->packed_list);
			if (!msg_node)
				break;
			lflist_enqueue(&msg_queue->free_list, msg_node);
		}
		simplist_enqueue(&msg_queue->free_pack_list, (lflist_node_t *)packet_obj);
#ifdef DEBUG_MODE_ENABLE
		if (packet_obj->msg_typ == MSGBX_MSG_TYPE_METHOD || packet_obj->msg_typ == MSGBX_MSG_TYPE_REPLY)
			++dbg_info->recv_msg_1_cnt;
		else
			++dbg_info->recv_msg_2_cnt;
#endif
	}
	return 0;
}

void release_node(bst_msg_queue_t *msg_queue, lflist_node_t *release_node, lflist_node_t *release_pre_node, lflist_node_t **pre_node)
{
	packet_object_t *drop_packet_obj = NULL;
	lflist_node_t *drop_node = NULL;

	if (!release_node || !msg_queue || !*pre_node)
		return;

    if (release_pre_node) {
        release_pre_node->next = release_node->next;
		if (!release_pre_node->next)
			msg_queue->uncompleted_pack_list.head = release_pre_node;
    } else {
        simplist_dequeue(&msg_queue->uncompleted_pack_list);
    }
    release_node->next = NULL;

    if (*pre_node == release_node) {
        *pre_node = release_pre_node;
    }

	drop_packet_obj = (packet_object_t *)release_node;
	while (1) {
		drop_node = simplist_dequeue(&drop_packet_obj->packed_list);
		if (!drop_node)
			break;
		lflist_enqueue(&msg_queue->free_list, drop_node);
	}
    simplist_enqueue(&msg_queue->free_pack_list, release_node);
}

int8_t msg_queue_collate(bst_msg_queue_t *msg_queue, uint8_t is_err_handle)
{
	int8_t ret = -1;
	lflist_node_t *msg_node = NULL;
	lflist_node_t *drop_node = NULL;
	msg_object_t *msg_obj = NULL;
	rw_msg_t *msg = NULL;
	packet_object_t *packet_obj = NULL;
	uint64_t msg_head = 0;
	header_union tmp_value = { 0 };
	uint8_t complete_flag = 0;
	lflist_node_t *pre_node = NULL;
	lflist_node_t *next_node = NULL;
	lflist_node_t *found_pre_node = NULL;
	lflist_node_t *found_node = NULL;
	uint64_t mask = 0;
	uint64_t check_uuid = 0;
	uint64_t uuid = 0;

	if (!msg_queue)
		return ret;

	while(1) {
		msg_node = lflist_dequeue(&msg_queue->used_list);
		if (!msg_node)
			break;

		msg_obj = msg_node->data;
		msg = &msg_obj->msg;
		tmp_value.header = msg->header;
		msg_head = tmp_value.as_uint64_t;
		if (msg->header.idx == 0) {
			packet_obj = (packet_object_t *)simplist_dequeue(&msg_queue->free_pack_list);
			if (!packet_obj) {
				IPC_LOG_INFO(
					"msg queue don't have empty pack node, drop msg pid: %d, typ: %d, tok: %d, cmd: %d, idx: %d\n",
					msg->header.pid, msg->header.typ, msg->header.tok, msg->header.cmd, msg->header.idx);
				lflist_enqueue(&msg_queue->free_list, msg_node);
				continue;
			}

			packet_obj->mask = GET_HEADER_MASK(msg_head);
			packet_obj->last_idx = msg->header.idx;
			packet_obj->msg_typ = msg->header.typ;
			simplist_enqueue(&packet_obj->packed_list, msg_node);
			if (msg->header.is_eof == 1) {
				simplist_enqueue(&msg_queue->completed_pack_list, (lflist_node_t *)packet_obj);
				complete_flag = 1;
			} else {
				simplist_enqueue(&msg_queue->uncompleted_pack_list, (lflist_node_t *)packet_obj);
			}
		} else {
			ret = -1;
			mask = GET_HEADER_MASK(msg_head);
			pre_node = NULL;
			found_pre_node = NULL;
			found_node = NULL;
			next_node = msg_queue->uncompleted_pack_list.tail;
			// poll uncompleted_pack_list
			while (next_node) {
				packet_obj = (packet_object_t *)next_node;
				if (packet_obj->mask == mask) {
					ret = 0;
					if (packet_obj->last_idx + 1 == msg->header.idx) {
						// mask match, idx match
						if (found_node) {
							//free found node
							release_node(msg_queue, found_node, found_pre_node, &pre_node);
						}
						// update new found node
						found_node = next_node;
						found_pre_node = pre_node;
					} else {
						// mask match, idx not match
						if (found_node) {
							// free found node
							release_node(msg_queue, found_node, found_pre_node, &pre_node);
							// clear found node
							found_node = NULL;
							found_pre_node = NULL;
						}

						// free current node
						if (pre_node) {
							pre_node->next = next_node->next;
							if (!pre_node->next)
								msg_queue->uncompleted_pack_list.head = pre_node;	
						} else {
							simplist_dequeue(&msg_queue->uncompleted_pack_list);
						}
						next_node->next = NULL;
						while (1) {
							drop_node = simplist_dequeue(&packet_obj->packed_list);
							if (!drop_node)
								break;
							lflist_enqueue(&msg_queue->free_list, drop_node);
						}
						simplist_enqueue(&msg_queue->free_pack_list, next_node);

						if (pre_node)
							next_node = pre_node->next;
						else
							next_node = msg_queue->uncompleted_pack_list.tail;
						continue;
					}
				} else {
					uuid = GET_HEADER_UUID(mask);
					check_uuid = GET_HEADER_UUID(packet_obj->mask);
					// uuid match
					if (check_uuid == uuid) {
						if (pre_node) {
							pre_node->next = next_node->next;
							if (!pre_node->next)
								msg_queue->uncompleted_pack_list.head = pre_node;	
						} else {
							simplist_dequeue(&msg_queue->uncompleted_pack_list);
						}
						next_node->next = NULL;
						while (1) {
							drop_node = simplist_dequeue(&packet_obj->packed_list);
							if (!drop_node)
								break;
							lflist_enqueue(&msg_queue->free_list, drop_node);
						}
						simplist_enqueue(&msg_queue->free_pack_list, next_node);

						if (pre_node)
							next_node = pre_node->next;
						else
							next_node = msg_queue->uncompleted_pack_list.tail;
						continue;
					}
				}
				pre_node = next_node;
				next_node = next_node->next;
			}

			// check found_node and push msg in it
			if (found_node) {
				packet_obj = (packet_object_t *)found_node;
				packet_obj->last_idx = msg->header.idx;
				simplist_enqueue(&packet_obj->packed_list, msg_node);
				if (msg->header.is_eof == 1) {
					if (found_pre_node) {
						found_pre_node->next = found_node->next;
						if (!found_pre_node->next)
							msg_queue->uncompleted_pack_list.head = found_pre_node;
					} else {
						simplist_dequeue(&msg_queue->uncompleted_pack_list);
					}
					found_node->next = NULL;
					simplist_enqueue(&msg_queue->completed_pack_list, found_node);
					complete_flag = 1;
				}
			} else {
				lflist_enqueue(&msg_queue->free_list, msg_node);
			}
		}
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
	lflist_enqueue(&msg_queue->free_list, node);
	return 0;
}
