// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#include "queue.h"
#include "linux/mutex.h"
#include "linux/stddef.h"

// Initialize the Descriptor ring queue
void init_queue(struct virtnet_queue *queue, void *desc_base, u32 desc_num)
{
	queue->desc = desc_base;
	queue->desc_num = desc_num;
	atomic_set(&queue->owned_index, -1);
	atomic_set(&queue->free_index, -1);
	atomic_set(&queue->maintain_index, -1);
	memset(queue->status, 0, sizeof(queue->status));
	spin_lock_init(&queue->lock);
}

void init_all_desc(struct virtnet_queue *queue, bool default_owner)
{
	int i;

	for (i = 0; i < queue->desc_num; i++) {
		queue->desc[i].index = i;
		queue->desc[i].owned = default_owner;
		queue->desc[i].skb = NULL;
	}
}
