// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_mem_manager.h
 * @brief   This file is the header file of the memory manager part of the BSTN
 *          driver. It contains structure definitions, function declarations and
 *          macros related to the memory and address management.
 */

#ifndef BSTN_MEM_MANAGER_H
#define BSTN_MEM_MANAGER_H

#include <linux/iommu.h>
#include <linux/iova.h>
#include "bstn.h"
#include "bstn_user.h"
#include "bstn_dma_manager.h"

/*******************************************************************************
 * Memory Address Related Convertion Macros
 ******************************************************************************/
#define _FW_IOVA_BASE_              ( 0x61400000U)
#define _FW_IOVA_END_               ( 0x63400000U)

#define _FW_PA_BASE_                (0x810000000LLU)
#define _FW_PA_END_                 (0x812000000LLU)
#define _FW_IOVA_OFFSET_(iova)      (iova - _FW_IOVA_BASE_)
#define _FW_PA_OFFSET_(pa)          (pa   - _FW_PA_BASE_)

/* iova => pa */
#define bus_to_phys(iova) (phys_addr_t)(_FW_IOVA_OFFSET_(iova) + _FW_PA_BASE_)
/* pa => iova */
#define phys_to_bus(pa) (dma_addr_t)(_FW_PA_OFFSET_(pa) + _FW_IOVA_BASE_)

#define addr_truncate(bstn_addr) \
	(dsp_ptr)((unsigned long)(bstn_addr)&0xFFFFFFFF)

#define bus_to_dma(baddr)                                               \
	((bstn_mem_usingsmmu == 0) ? ((dma_addr_t)bus_to_phys(baddr)) : \
				     ((dma_addr_t)baddr))

#define dma_to_bus(daddr)                                            \
	((bstn_mem_usingsmmu == 0) ? ((dsp_ptr)phys_to_bus(daddr)) : \
				     ((dsp_ptr)addr_truncate(daddr)))

#define bus_to_kern(pbstn, baddr, blk) \
	(blk->kern_addr + ((dsp_ptr)(baddr)-dma_to_bus((blk)->dma_addr)))

#define kern_to_bus(pbstn, kaddr, blk) \
	(dma_to_bus((blk)->dma_addr +  \
		    (phys_addr_t)((void *)(kaddr) - (blk)->kern_addr)))

//the structure to represent a physically continuous buffer allocated for userspace
struct bstn_buffer {
	bstn_memblock_t *block;
	uint64_t user_addr;
	dsp_ptr bus_addr;
	struct dma_buf *dbuf;
	int type;
	int fd;
	size_t size;
	struct hlist_node node;
};

//the structure to manage the buffers allocated for the same process in userspace
struct bstn_mem_ctx {
	struct file *filp;
	DECLARE_HASHTABLE(ht, 12);
	struct list_head link;
};

struct bstn_mem_ops {
	bstn_memblock_t *(*alloc)(struct bstn_device *pbstn, uint32_t size,
				  uint32_t align, unsigned long attr);

	dma_addr_t (*iommu_map_fw)(struct bstn_device *pbstn, uint32_t size,
				   uint32_t align, phys_addr_t addr);

	void (*iommu_map)(struct bstn_device *pbstn, uint32_t size,
			  uint32_t align, phys_addr_t addr, dsp_ptr *iova,
			  unsigned long attr);

	void (*iommu_free)(struct bstn_device *pbstn, uint32_t size,
			   uint32_t align, dsp_ptr iova);

	void (*free)(bstn_memblock_t *memblock);
};

struct bstn_mem_manager {
	phys_addr_t rmem_base;
	phys_addr_t rmem_size;

	bool enable_smmu;
	struct device *pdev; // memory manager dev for dsp
	const struct bstn_dma_ops *dma_ops; // memory manager internal ops api
	const struct bstn_mem_ops *ops; // memory manager external ops api
	struct iommu_domain *domain;
	struct iommu_group *group;

	/*
	   We use mutex to lock any operation on process contexts in order to deal
	   with unexpected execution of ioctl syscalls at the same time as close
	   syscall.
	 */
	struct mutex mm_mutex;
	struct list_head mem_ctx_list;
};

int bstn_mem_ctx_add(struct bstn_device *pbstn, struct file *filp);
int bstn_mem_ctx_del(struct bstn_device *pbstn, struct file *filp);

int bsnn_buffer_alloc(struct file *filp, struct bstn_device *pbstn,
		      struct bsnn_buffer *pbuffer);

int bsnn_buffer_free(struct file *filp, struct bstn_device *pbstn,
		     struct bsnn_buffer *pbuffer);

int bstn_dma_buf_import(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf);
int bstn_dma_buf_return(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf);
int bstn_cma_buf_import(struct file *filp, struct bstn_device *pbstn,
			struct bstn_cma_buf *buf);
int bstn_cma_buf_return(struct file *filp, struct bstn_device *pbstn,
			struct bstn_cma_buf *buf);
int bstn_dma_buf_export(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf);

int bstn_dma_buf_flush(struct file *filp, struct bstn_device *pbstn,
		       struct bsnn_buffer *pbuffer);
int bstn_dma_buf_invalidate(struct file *filp, struct bstn_device *pbstn,
			    struct bsnn_buffer *pbuffer);
int bstn_dma_buf_sync(struct file *filp, struct bstn_device *pbstn,
		      struct bstnpu_mem_sync *pbuffer);

int bstn_mem_manager_init(struct bstn_device *pbstn);
void bstn_mem_manager_exit(struct bstn_device *pbstn);

#endif
