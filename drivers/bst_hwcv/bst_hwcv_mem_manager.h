/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_MEM_MANAGER_H__
#define __BST_HWCV_MEM_MANAGER_H__
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/hashtable.h>
#include "bst_hwcv_dma_contig.h"

enum hwcv_buf_type {
	MEM,
	DMA
};

struct bst_hwcv_buf {
	void *mem_priv;
	struct dma_buf *dbuf;
	dma_addr_t iova;
	unsigned int bytesused;
	unsigned int length;
	int fd;
	enum hwcv_buf_type type;
	struct hlist_node node;
};

struct bst_hwcv_mem_manager {
	struct device *dev;
	struct mutex lock;
	struct list_head mem_ctx_list;
	int mem_ctx_id;
	const struct hwcv_mem_ops *ops;
	bool use_smmu;
};

/*------------------------------------------------------------------------------------------------------*/

struct bst_hwcv_mem_ctx {
	int id;
	struct file *filp;
	DECLARE_HASHTABLE(mm_ht, 12);
	struct list_head node;
};

struct bst_hwcv_buf *find_buf_by_iova(struct file *filp,
				      struct bst_hwcv_mem_manager *mman,
				      dma_addr_t iova);

struct bst_hwcv_buf *bst_hwcv_buf_alloc(struct bst_hwcv_mem_manager *mman,
					unsigned int size);
void bst_hwcv_buf_free(struct bst_hwcv_mem_manager *mman,
		       struct bst_hwcv_buf *buf);

struct bst_hwcv_buf *bst_hwcv_buf_import(struct bst_hwcv_mem_manager *mman,
					 int fd, unsigned int size);
void bst_hwcv_buf_return(struct bst_hwcv_mem_manager *mman,
			 struct bst_hwcv_buf *buf);

int bst_hwcv_add_buf_to_ctx(struct file *filp,
			    struct bst_hwcv_mem_manager *mman,
			    struct bst_hwcv_buf *buf);
struct bst_hwcv_buf *
bst_hwcv_del_buf_from_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman,
			  dma_addr_t iova);
int bst_hwcv_add_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman);
int bst_hwcv_del_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman);

int bst_hwcv_mem_manager_init(struct device *dev,
			      struct bst_hwcv_mem_manager *mman);
void bst_hwcv_mem_manager_exit(struct bst_hwcv_mem_manager *mman);

#endif
