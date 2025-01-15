/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Black Sesame Technologies, Inc.
 *
 * Author: Xuran Yang <xuran.yang@bst.ai>
 */

#ifndef _DESCRIPTOR_QUEUE_H
#define _DESCRIPTOR_QUEUE_H

#include "linux/printk.h"
#include <linux/spinlock.h>
#include <linux/types.h>

#define O_TAKEN 0
#define F_TAKEN 1

// Define the Descriptor structure
struct virtnet_desc {
	u64 index;
	u64 owned;
	u64 fg;

	void *skb;
	dma_addr_t data;
	size_t size;

	void *skb1;
};

// Define the Descriptor ring queue structure
struct virtnet_queue {
	atomic_t owned_index;
	atomic_t free_index;
	atomic_t maintain_index;
	u32 desc_num;
	spinlock_t lock;	// lock the queue operate
	unsigned long status[32];
	struct virtnet_desc *desc; // Array of descriptors
};

// Initialize the Descriptor ring queue
void init_queue(struct virtnet_queue *queue, void *desc_base, u32 desc_num);

/* Initialize all descriptor */
void init_all_desc(struct virtnet_queue *queue, bool default_owner);

// Get a free Descriptor with Owner Bit set to 0
static inline struct virtnet_desc *get_free_descriptor(struct virtnet_queue *queue)
{
	int index;
	unsigned long flags;
	struct virtnet_desc *desc = NULL;

	spin_lock_irqsave(&queue->lock, flags);
	index = atomic_inc_return(&queue->free_index) % queue->desc_num;
	if (!queue->desc[index].owned && !test_and_set_bit(F_TAKEN, &queue->status[index]))
		desc = &queue->desc[index];
	else
		atomic_dec(&queue->free_index);

	spin_unlock_irqrestore(&queue->lock, flags);
	return desc; // No free descriptor available
}

// Get an owned Descriptor with Owner Bit set to 1
static inline struct virtnet_desc *get_owned_descriptor(struct virtnet_queue *queue)
{
	int index;
	unsigned long flags;
	struct virtnet_desc *desc = NULL;

	spin_lock_irqsave(&queue->lock, flags);
	index = atomic_inc_return(&queue->owned_index) % queue->desc_num;
	if (queue->desc[index].owned && !test_and_set_bit(O_TAKEN, &queue->status[index]))
		desc = &queue->desc[index];
	else
		atomic_dec(&queue->owned_index);
	spin_unlock_irqrestore(&queue->lock, flags);
	return desc; // No free descriptor available
}

// Get a free Descriptor with Owner Bit set to 0 and skb is null
static inline struct virtnet_desc *get_free_skb_descriptor(struct virtnet_queue *queue)
{
	int index;
	unsigned long flags;
	struct virtnet_desc *desc = NULL;

	spin_lock_irqsave(&queue->lock, flags);
	index = atomic_inc_return(&queue->free_index) % queue->desc_num;
	if (!queue->desc[index].owned &&
	    !test_bit(F_TAKEN, &queue->status[index])) {
		if (!queue->desc[index].skb) {
			set_bit(F_TAKEN, &queue->status[index]);
			desc = &queue->desc[index];
		} else {
			atomic_dec(&queue->free_index);
		}
	} else {
		atomic_dec(&queue->free_index);
	}
	spin_unlock_irqrestore(&queue->lock, flags);
	return desc; // No free descriptor available
}

// Get a free Descriptor with Owner Bit set to 0 and skb is not null
static inline struct virtnet_desc *get_maintain_descriptor(struct virtnet_queue *queue)
{
	int index;
	unsigned long flags;
	struct virtnet_desc *desc = NULL;

	spin_lock_irqsave(&queue->lock, flags);
	index = atomic_inc_return(&queue->maintain_index) % queue->desc_num;
	if (!queue->desc[index].owned &&
	    !test_bit(F_TAKEN, &queue->status[index])) {
		if (queue->desc[index].skb) {
			set_bit(F_TAKEN, &queue->status[index]);
			desc = &queue->desc[index];
		} else {
			atomic_dec(&queue->maintain_index);
		}
	} else {
		atomic_dec(&queue->maintain_index);
	}
	spin_unlock_irqrestore(&queue->lock, flags);
	return desc; // No free descriptor available
}

// Set the Owner Bit of a Descriptor
static inline void set_desc_owner_bit(struct virtnet_queue *queue, struct virtnet_desc *desc,
				      bool owned)
{
	unsigned long flags;

	spin_lock_irqsave(&queue->lock, flags);
	desc->owned = owned;
	if (owned)
		clear_bit(F_TAKEN, &queue->status[desc->index]);
	else
		clear_bit(O_TAKEN, &queue->status[desc->index]);
	spin_unlock_irqrestore(&queue->lock, flags);
}

static inline void set_desc_taken_bit(struct virtnet_queue *queue, struct virtnet_desc *desc,
				      int taken_type, bool val)
{
	unsigned long flags;

	spin_lock_irqsave(&queue->lock, flags);
	if (val)
		set_bit(taken_type, &queue->status[desc->index]);
	else
		clear_bit(taken_type, &queue->status[desc->index]);
	spin_unlock_irqrestore(&queue->lock, flags);
}

static inline struct virtnet_desc *get_each_desc(struct virtnet_queue *queue, int *p)
{
	if (++(*p) < queue->desc_num)
		return &queue->desc[*p];
	else
		return NULL;
}

#endif
