/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2010 Samsung Electronics
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_DMA_BUF_H__
#define __BST_HWCV_DMA_BUF_H__

#include <linux/dma-buf.h>

struct hwcv_mem_ops {
	void *(*alloc)(struct device *dev, unsigned long attrs,
		       unsigned long size, enum dma_data_direction dma_dir,
		       gfp_t gfp_flags);
	void (*put)(void *buf_priv);
	struct dma_buf *(*get_dmabuf)(void *buf_priv, unsigned long flags);

	void (*prepare)(void *buf_priv);
	void (*finish)(void *buf_priv);
	void (*prepare_own)(void *buf_priv);
	void (*finish_own)(void *buf_priv);

	void *(*attach_dmabuf)(struct device *dev, struct dma_buf *dbuf,
			       unsigned long size,
			       enum dma_data_direction dma_dir);
	void (*detach_dmabuf)(void *buf_priv);
	int (*map_dmabuf)(void *buf_priv);
	void (*unmap_dmabuf)(void *buf_priv);

	void *(*vaddr)(void *buf_priv);
	void *(*cookie)(void *buf_priv);

	unsigned int (*num_users)(void *buf_priv);

	int (*mmap)(void *buf_priv, struct vm_area_struct *vma);
};

extern const struct hwcv_mem_ops hwcv_dma_contig_memops;
int hwcv_dma_contig_set_max_seg_size(struct device *dev, unsigned int size);

#endif
