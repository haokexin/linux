/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DMA BUF PagePool implementation
 * Based on earlier ION code by Google
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2020 Linaro Ltd.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _SYSTEM_HEAP_IPC_H
#define _SYSTEM_HEAP_IPC_H

#include "dmabuf-ipc-src-gen/dmabuf_ipc_client.h"

#define LOW_ORDER_GFP (GFP_HIGHUSER | __GFP_ZERO | __GFP_COMP)
#define MID_ORDER_GFP (LOW_ORDER_GFP | __GFP_NOWARN)
#define HIGH_ORDER_GFP  (((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN \
				| __GFP_NORETRY) & ~__GFP_RECLAIM) \
				| __GFP_COMP)

typedef enum {
	R5_SUCCESS = 0,
	R5_FAIL = -1,
} R5_RET;
typedef struct ipc_msg
{
	int32_t cmd;
	int32_t mem_type;
	uint64_t  payload[R5MEM_SHARED_BUF_SIZE];
} ipc_msg;

union ipc_alloc_info
{
	unsigned long len;
	unsigned long global_fd;
};

struct system_heap_buffer {
	struct dma_heap *heap;
	struct list_head attachments;
	struct mutex lock;
	unsigned long len;
	struct sg_table sg_table;
	int vmap_cnt;
	void *vaddr;

	unsigned long global_fd;
	uint64_t phy_addr;
	r5mem_MemType_t mem_type;

	bool uncached;
};

struct dma_heap_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
	bool mapped;

	bool uncached;
};

struct blocks_stats {
	uint16_t fd:12;     // fd is [0, 255]
	uint16_t free:4;    // whether the memory block is used or not
	uint16_t ref;       // block reference count
	uint32_t size;      // block size
	uint64_t addr:60;   // block start physical addr
	uint8_t mem_type:4; // memory type
};

struct global_dmabuf_info {
	uint64_t total_memory;
	uint64_t used_memory;
};

#endif /* _SYSTEM_HEAP_IPC_H */
