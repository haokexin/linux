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

#ifndef C_LOCKFREE_LIST_SISO_H
#define C_LOCKFREE_LIST_SISO_H

#include "bstipc_cfg.h"

struct _lflist_node_t {
	void *data;
	struct _lflist_node_t *next;
};
#define lflist_node_t struct _lflist_node_t

struct _lflist_t {
#if !defined(__riscv)
	__attribute__ ((aligned(64))) lflist_node_t* head;
    uint32_t total_in;
	uint32_t in_cnt;
	__attribute__ ((aligned(64))) lflist_node_t* tail;
    uint32_t total_out;
	uint32_t out_cnt;
	uint32_t pad;
#else
	lflist_node_t *head;
	lflist_node_t *tail;
#endif
	lflist_node_t head_node;
};
#define lflist_t struct _lflist_t

struct _simplist_t {
	lflist_node_t *head;
	lflist_node_t *tail;
};
#define simplist_t struct _simplist_t

struct _msg_object_t {
	rw_msg_t msg;
	uint64_t timestamp;
	uint8_t res[8];
};
#define msg_object_t struct _msg_object_t

struct _packet_object_t {
	lflist_node_t node;
	simplist_t packed_list;
	uint64_t mask;
	uint8_t last_idx;
	uint8_t msg_typ;
	uint8_t res[6];
};
#define packet_object_t struct _packet_object_t

struct _bst_msg_queue_t {
	uint64_t k_addr;
	uint64_t rsv[7];
	lflist_t free_list;
	lflist_t used_list;
	simplist_t free_pack_list;
	simplist_t uncompleted_pack_list;
	simplist_t completed_pack_list;
	packet_object_t packet_obj[SESSION_PACKET_MSG_BUFFER_COUNT];
	msg_object_t msg_obj[SESSION_MSG_BUFFER_COUNT];
	lflist_node_t msg_nodes[SESSION_MSG_BUFFER_COUNT];
};
#define bst_msg_queue_t struct _bst_msg_queue_t

static inline void lflist_init_node(lflist_node_t* node, void* data)
{
    if (!node || !data)
        return;
    node->data = data;
    node->next = NULL;
}

static inline void lflist_init(lflist_t *list)
{
	if (!list)
		return;

	list->head_node.next = NULL;
	list->head = &list->head_node;
	list->tail = &list->head_node;
	list->total_in = 0;
	list->total_out = 0;
	list->in_cnt = 0;
	list->out_cnt = 0;
}

static inline void lflist_enqueue(lflist_t *list, lflist_node_t *node)
{
	lflist_node_t *prevHead = NULL;

	if (!list || !node)
		return;

	node->next = NULL;
	prevHead = ATOMIC_LOAD(&list->head, __ATOMIC_RELAXED);
	ATOMIC_STORE(&list->head, node, __ATOMIC_RELAXED);
	++list->in_cnt;
#ifdef __aarch64__
	__asm__ __volatile__("dmb st" ::: "memory");
#endif
	prevHead->next = node;
}

static inline lflist_node_t *lflist_dequeue(lflist_t *list)
{
	lflist_node_t *tail = NULL;
	lflist_node_t *next = NULL;

	if(!list)
		return NULL;

	tail = ATOMIC_LOAD(&list->tail, __ATOMIC_RELAXED);
	next = ATOMIC_LOAD(&tail->next, __ATOMIC_RELAXED);
	if (next == NULL)
        return NULL;
	ATOMIC_STORE(&list->tail, next, __ATOMIC_RELAXED);
	++list->out_cnt;
	// ATOMIC_STORE(&list->total_out, list->out_cnt, __ATOMIC_RELAXED);

	tail->data = next->data;
	tail->next = NULL;
	return tail;
}

static inline uint32_t lflist_size(lflist_t *list)
{
    // return ATOMIC_LOAD(&list->total_in, __ATOMIC_RELAXED) -
    //        ATOMIC_LOAD(&list->total_out, __ATOMIC_RELAXED);
	return list->in_cnt - list->out_cnt;
}

static inline void simplist_init(simplist_t *list)
{
	if (!list)
		return;

	list->head = NULL;
	list->tail = NULL;
}

static inline void simplist_enqueue(simplist_t *list, lflist_node_t *node)
{
    if (!list || !node)
        return;

    node->next = NULL;
    if (list->head == NULL)
    {
        list->head = list->tail = node;
        return;
    }

    list->head->next = node;
    list->head = node;
}

static inline lflist_node_t *simplist_dequeue(simplist_t *list)
{
	lflist_node_t* node = NULL;

    if (!list || !list->tail)
        return NULL;

    node = list->tail;
    list->tail = list->tail->next;
    if (list->tail == NULL)
        list->head = NULL;

    node->next = NULL;
    return node;
}

static inline uint32_t simplist_size(simplist_t *list)
{
	lflist_node_t *node = NULL;
	uint32_t cnt = 0;

	if (!list)
		return 0;

	node = list->tail;
	while (node) {
		node = node->next;
		++cnt;
	}
	return cnt;
}

#endif
