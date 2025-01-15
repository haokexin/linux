/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP
 * author: AI Tools Team, BST Ltd.
 *
 * @file    bst_cv_mem_manager.h
 * @brief   This file is the header file of the memory manager part of the bst_cv
 *          driver. It contains structure definitions, function declarations and
 *          macros related to the memory and address management.
 */

#ifndef BST_CV_MEM_MANAGER_H
#define BST_CV_MEM_MANAGER_H

#include "bst_cv.h"
#include "bst_cv_dma_manager.h"

/*******************************************************************************
 * Memory Address Related Convertion Macros
 ******************************************************************************/
#define addr_truncate(bst_cv_addr)                                             \
  (dsp_ptr)((unsigned long)(bst_cv_addr)&0xFFFFFFFF)

#define bus_to_phys(baddr)                                                     \
  ((((phys_addr_t)(baddr) & 0xC0000000ULL) << 4) |                             \
    ((phys_addr_t)(baddr) & 0x3FFFFFFFULL))

#define phys_to_bus(paddr)                                                     \
  ((((phys_addr_t)(paddr) & 0xC00000000ULL) >> 4) |                            \
    ((phys_addr_t)(paddr) & 0x3FFFFFFFULL))

#define bus_to_dma(baddr) ((bst_cv_mem_usingsmmu == 0)                         \
   ? ((dma_addr_t)bus_to_phys(baddr))                                          \
   : ((dma_addr_t)baddr))

#define dma_to_bus(daddr) ((bst_cv_mem_usingsmmu == 0)                         \
   ? ((dsp_ptr)phys_to_bus(daddr))                                             \
   : ((dsp_ptr)addr_truncate(daddr)))

#define bus_to_kern(pbst_cv, baddr, blk)                                       \
  (blk->kern_addr + ((dsp_ptr)(baddr) - dma_to_bus((blk)->dma_addr)))

#define kern_to_bus(pbst_cv, kaddr, blk)                                       \
  (dma_to_bus((blk)->dma_addr +                                                \
              (phys_addr_t)((void *)(kaddr) - (blk)->kern_addr)))


//the structure to represent a physically continuous buffer allocated for userspace
struct bst_cv_buffer {
	bst_cv_memblock_t * block;
	uint64_t            user_addr;
	dsp_ptr             bus_addr;
	struct dma_buf     *dbuf;
	int                 type;
	int                 fd;
	struct hlist_node   node;
};

//the structure to manage the buffers allocated for the same process in userspace
struct bst_cv_mem_ctx {
	struct file     *filp;
	DECLARE_HASHTABLE(ht, 12);
	struct list_head link;
};

struct bst_cv_mem_ops {
	bst_cv_memblock_t *(*alloc)(struct bst_cv     *pbst_cv,
									   uint32_t    size,
									   uint32_t    align,
								unsigned long      attr);

	void        (*iommu_bypass)(struct bst_cv      *pbst_cv,
									   uint32_t     size,
									   uint32_t     align,
									   phys_addr_t  addr,
								unsigned long       attr);

	void        (*iommu_map)   (struct bst_cv      *pbst_cv,
									   uint32_t     size,
									   uint32_t     align,
									   phys_addr_t  addr,
									   dsp_ptr      iova,
								unsigned long       attr);

	void        (*iommu_free)  (struct bst_cv      *pbst_cv,
									   uint32_t     size,
									   uint32_t     align,
									   dsp_ptr      iova);

	void        (*free)        (bst_cv_memblock_t* memblock);
};


struct bst_cv_mem_manager {
	bool enable_smmu;
	struct device               *pdev;    // memory manager dev for dsp
	const struct bst_cv_dma_ops *dma_ops; // memory manager internal ops api
	const struct bst_cv_mem_ops *ops;     // memory manager external ops api

	/*
	   We use mutex to lock any operation on process contexts in order to deal
	   with unexpected execution of ioctl syscalls at the same time as close
	   syscall.
	 */
	struct mutex mm_mutex;
	struct list_head mem_ctx_list;

	struct mutex dma_buf_mutex;
	DECLARE_HASHTABLE(dma_buf_ht, 10);
};


int bst_cv_mem_ctx_add(struct bst_cv *pbst_cv, struct file *filp);
int bst_cv_mem_ctx_del(struct bst_cv *pbst_cv, struct file *filp);

int bst_cv_user_buffer_alloc(struct file *filp,
							 struct bst_cv *pbst_cv,
							 struct xrp_ioctl_alloc *ualloc);

int bst_cv_user_buffer_free (struct file *filp,
							 struct bst_cv *pbst_cv,
							 struct xrp_ioctl_alloc *ualloc);

int bst_cv_dma_buf_import(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf);
int bst_cv_dma_buf_return(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf);


int  bst_cv_mem_manager_init(struct bst_cv *pbst_cv);
void bst_cv_mem_manager_exit(struct bst_cv *pbst_cv);

#endif
