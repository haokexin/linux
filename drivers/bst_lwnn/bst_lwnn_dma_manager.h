// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

#ifndef _BST_LWNN_DMA_MANAGER_H_
#define _BST_LWNN_DMA_MANAGER_H_

#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/mm_types.h>

struct bst_lwnn_vmarea_handler {
	refcount_t *refcount;
	void (*put)(void *arg);
	void *arg;
};

typedef struct bst_lwnn_memblock {
	struct device *dev;
	void *pbst_lwnn;
	void *vaddr;
	union {
		void *cookie;
		void *kern_addr;
	};
	dma_addr_t dma_addr;
	unsigned long size;
	unsigned long attrs;
	enum dma_data_direction dma_dir;
	struct sg_table *dma_sgt;
	struct frame_vector *vec;

	/* MMAP related */
	struct bst_lwnn_vmarea_handler handler;
	struct sg_table *sgt_base;
	refcount_t refcount;

	/* DMABUF related */
	struct dma_buf_attachment *db_attach;
} bst_lwnn_memblock_t;

struct bst_lwnn_dma_ops {
	void *(*alloc)(struct device *dev, unsigned long attrs,
		       unsigned long size, enum dma_data_direction dma_dir,
		       gfp_t gfp_flags);
	void (*put)(void *buffer_priv);

	struct dma_buf *(*get_dmabuf)(void *buffer_priv, unsigned long flags);

	void (*prepare)(void *buffer_priv);
	void (*finish)(void *buffer_priv);

	void *(*attach_dmabuf)(struct device *dev, struct dma_buf *dbuf,
			       unsigned long size,
			       enum dma_data_direction dma_dir);
	void (*detach_dmabuf)(void *buffer_priv);
	int (*map_dmabuf)(void *buffer_priv);
	void (*unmap_dmabuf)(void *buffer_priv);

	void *(*vaddr)(void *buffer_priv);
	void *(*cookie)(void *buffer_priv);

	unsigned int (*num_users)(void *buffer_priv);

	int (*mmap)(void *buffer_priv, struct vm_area_struct *vma);
};

extern const struct bst_lwnn_dma_ops bst_lwnn_dma_memops;

int bst_lwnn_dma_contig_set_max_seg_size(struct device *dev, unsigned int size);

#endif