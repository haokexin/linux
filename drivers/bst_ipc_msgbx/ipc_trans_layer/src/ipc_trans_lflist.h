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

#ifndef C_LOCKFREE_LIST_H
#define C_LOCKFREE_LIST_H

#include <bst/bstipc_cfg.h>

struct _lflist_node_t {
	void *data;
	struct _lflist_node_t *next;
};
#define lflist_node_t struct _lflist_node_t

struct _lflist_t {
#if !defined(__riscv)
	_Atomic(lflist_node_t *) head;
	_Atomic(lflist_node_t *) tail;
#else
	lflist_node_t *head;
	lflist_node_t *tail;
#endif
	lflist_node_t head_node;
	uint8_t initialized;
};
#define lflist_t struct _lflist_t

static inline void lflist_init(lflist_t *list)
{
	if (!list)
		return;

	list->head_node.next = NULL;
	ATOMIC_STORE(&(list->head), &list->head_node, __ATOMIC_RELAXED);
	ATOMIC_STORE(&list->tail, &list->head_node, __ATOMIC_RELAXED);
	list->initialized = 1;
}

static inline void lflist_enqueue(lflist_t *list, lflist_node_t *node)
{
	lflist_node_t *prevHead = NULL;

	if (!list || !node || list->initialized != 1)
		return;

	node->next = NULL;
	prevHead = ATOMIC_LOAD(&list->head, __ATOMIC_ACQUIRE);
#if defined(__riscv) || defined(__XTENSA__)
	ATOMIC_STORE(&list->head, node, __ATOMIC_RELEASE);
#else
	do {
		prevHead = ATOMIC_LOAD(&list->head, __ATOMIC_ACQUIRE);
	} while (!ATOMIC_COMPARE_EXCHANGE_STRONG(&list->head, &prevHead, node,
						 __ATOMIC_RELEASE,
						 __ATOMIC_RELAXED));
#endif
	prevHead->next = node;
}

static inline lflist_node_t *lflist_dequeue(lflist_t *list)
{
	lflist_node_t *tail = NULL;
	lflist_node_t *next = NULL;
	void *data = NULL;

	if(!list || list->initialized != 1)
		return NULL;

#if defined(__riscv) || defined(__XTENSA__)
	tail = ATOMIC_LOAD(&list->tail, __ATOMIC_ACQUIRE);
	next = tail->next;
	if (next == NULL)
		return NULL; // Queue is empty
	ATOMIC_STORE(&list->tail, next, __ATOMIC_RELEASE);
	data = next->data;
#else
	do {
		tail = ATOMIC_LOAD(&list->tail, __ATOMIC_ACQUIRE);
		next = tail->next;
		if (next == NULL)
			return NULL; // Queue is empty
		data = next->data;
	} while (!ATOMIC_COMPARE_EXCHANGE_STRONG(
		&list->tail, &tail, next, __ATOMIC_RELEASE, __ATOMIC_RELAXED));
#endif

	tail->data = data;
	tail->next = NULL;
	return tail;
}

#endif
