// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BST_LWNN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_mem_manager.c
 * @brief   This file is the source code file of the memory manager of the BST_LWNN
 *          driver. It contains definitions of actual memory manager operation
 *          functions as well as the initialization and cleanup functions.
 * @note    Because the current buffer and memory structures have not been
 *          finalized yet, further implementation for reliability like garbage
 *          collection is either commented out or not completed.
 */
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/sizes.h>
#include <bst/smmu_safety_map.h>
// #include <linux/audit.h>
#include "bst_lwnn.h"
#include "bst_lwnn_mem_manager.h"
#include "bst_lwnn_dma_manager.h"

extern struct device *dev_cvsmm;

typedef enum {
	DMA_BUFF_ALLOC,
	DMA_BUFF_IMPORT,
	CMA_BUFF_IMPORT,
} bst_lwnn_buf_type_e;

int close_fd(unsigned fd);

/******************************************************************************
* bst_lwnn mem ops: alloc, iommu_bypass, free...
*******************************************************************************/
static bst_lwnn_memblock_t *bst_lwnn_alloc(struct bst_lwnn *pbst_lwnn,
					   uint32_t size, uint32_t align,
					   unsigned long attr);

static void bst_lwnn_iommu_map(struct bst_lwnn *pbst_lwnn, uint32_t size,
			       uint32_t align, phys_addr_t addr, dsp_ptr *iova,
			       unsigned long attr);

static dma_addr_t bst_lwnn_iommu_bypass(struct bst_lwnn *pbst_lwnn,
					uint32_t size, uint32_t align,
					phys_addr_t addr, unsigned long attr);

static dma_addr_t bst_lwnn_iommu_bypass_iova(struct bst_lwnn *pbst_lwnn,
					     uint32_t size, uint32_t align,
					     phys_addr_t addr, dma_addr_t iova,
					     unsigned long attr);

static void bst_lwnn_iommu_free(struct bst_lwnn *pbst_lwnn, uint32_t size,
				uint32_t align, dsp_ptr iova);

static void bst_lwnn_free(bst_lwnn_memblock_t *block);

static struct bst_lwnn_mem_ops bst_lwnn_cma_memops = {
	.alloc = bst_lwnn_alloc,
	.iommu_bypass = bst_lwnn_iommu_bypass,
	.iommu_bypass_iova = bst_lwnn_iommu_bypass_iova,
	.iommu_map = bst_lwnn_iommu_map,
	.iommu_free = bst_lwnn_iommu_free,
	.free = bst_lwnn_free,
};

/*
 * @func    bst_lwnn_alloc
 * @brief   This function allocates a requested continuous memory block.
 * @params  mem_manager - the pointer to the memory mem_manager
 *          size - the requested size
 * @return  the pointer to the memory block - success
 *          NULL - failure
 */
static bst_lwnn_memblock_t *bst_lwnn_alloc(struct bst_lwnn *pbst_lwnn,
					   uint32_t size, uint32_t align,
					   unsigned long attr)
{
	struct bst_lwnn_mem_manager *pmman;
	struct bst_lwnn_memblock *block;
	struct device *pdev;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;
	block = NULL;
	if (0 == size) {
		goto exit;
	}
	size = ALIGN(size, align <= HPAGE_SIZE ? HPAGE_SIZE : align);
	attr |= DMA_ATTR_FORCE_CONTIGUOUS;
	block = pmman->dma_ops->alloc(pmman->pdev, attr, size,
				      DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (IS_ERR_OR_NULL(block))
		goto exit;
	block->pbst_lwnn = (void *)pbst_lwnn;

exit:
	if (NULL == block) {
		BST_LWNN_DEV_ERR(pdev, "bst_lwnn_alloc fail.");
	} else {
		BST_LWNN_TRACE_PRINTK("bst_lwnn_alloc ok");
	}
	return block;
}

static void bst_lwnn_iommu_map(struct bst_lwnn *pbst_lwnn, uint32_t size,
			       uint32_t align, phys_addr_t addr, dsp_ptr *iova,
			       unsigned long attr)
{
	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct device *pdev;
	struct iova_domain *iovad;
	dma_addr_t new_iova;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BST_LWNN_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}
	iovad = (struct iova_domain *)((void *)domain->iova_cookie +
				       sizeof(uint64_t));
	shift = iova_shift(iovad);
	// iovad->start_pfn = iova >> shift;
	BST_LWNN_TRACE_PRINTK(
		"dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
		(unsigned long long)pmman->pdev, (unsigned long long)domain,
		(unsigned long long)iovad, (unsigned long)shift);
	new_iova = alloc_iova_fast(iovad, size >> shift,
				   DMA_BIT_MASK(32) >> shift, true);
	new_iova <<= shift;
	if (iommu_map(domain, new_iova, addr, size, attr)) {
		BST_LWNN_DEV_ERR(pdev, "iommu_map 0x%llx fail.", addr);
		return;
	}
	*iova = (dsp_ptr)new_iova;
	// iovad->start_pfn = 0x80000000 >> shift; // iova must be bigger than 0x80000000
	BST_LWNN_TRACE_PRINTK(
		"bst_lwnn_iommu_map ok. iova: 0x%08x size: 0x%08x",
		(dsp_ptr)new_iova, size);
}

static dma_addr_t bst_lwnn_iommu_bypass(struct bst_lwnn *pbst_lwnn,
					uint32_t size, uint32_t align,
					phys_addr_t addr, unsigned long attr)
{
#if 0
	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct device *pdev;
	struct iova_domain *iovad;
	dma_addr_t iova;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);

	iovad = (struct iova_domain *)((void *)domain->iova_cookie +
				       sizeof(uint64_t));
	shift = iova_shift(iovad);
	iovad->start_pfn = phys_to_bus(addr) >> shift;
	BST_LWNN_TRACE_PRINTK(
		"dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
		(unsigned long long)pmman->pdev, (unsigned long long)domain,
		(unsigned long long)iovad, (unsigned long)shift);
	iova = alloc_iova_fast(iovad, size >> shift,
			       (phys_to_bus(addr) + size) >> shift, true);
	iova <<= shift;
	if (iommu_map(domain, iova, addr, size, attr)) {
		BST_LWNN_DEV_ERR(pdev, "iommu_map fail.");
		return DMA_MAPPING_ERROR;
	}
	iovad->start_pfn = 0x80000000 >> shift;
	BST_LWNN_STAGE_PRINTK(
		"bst_lwnn_iommu_bypass OK. pa: 0x%llx iova: 0x%x size: 0x%x",
		addr, (dsp_ptr)iova, size);
#endif

	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct iommu_group  *group;
	struct device *pdev;
	dma_addr_t iova;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	group = iommu_group_get(pmman->pdev);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	iova_cache_get();
	iommu_attach_group(domain, group);
	/* for bstn_mem_manager_exit release */
	pbst_lwnn->mem_manager.group = group;
	pbst_lwnn->mem_manager.domain = domain;

	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	BST_LWNN_STAGE_PRINTK("dev 0x%llx domain 0x%llx shift 0x%lx",
			  (unsigned long long)pmman->pdev,
			  (unsigned long long)domain,
			  (unsigned long)shift);
	iova = phys_to_bus(addr);
	if(iommu_map_by_proxy(COREIP_CV_DSP_SID, iova, addr, size)) {
		BST_LWNN_DEV_ERR(pdev, "iommu_map_by_proxy fail.");
		return DMA_MAPPING_ERROR;
	}
	BST_LWNN_STAGE_PRINTK(
		"%s OK. pa: 0x%llx iova: 0x%x size: 0x%x", __func__, addr,
		(dsp_ptr)iova, size);

	return iova;
}

static dma_addr_t bst_lwnn_iommu_bypass_iova(struct bst_lwnn *pbst_lwnn,
					     uint32_t size, uint32_t align,
					     phys_addr_t addr, dma_addr_t iova,
					     unsigned long attr)
{
#if 0
	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct device *pdev;
	struct iova_domain *iovad;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);

	iovad = (struct iova_domain *)((void *)domain->iova_cookie +
				       sizeof(uint64_t));
	shift = iova_shift(iovad);
	iovad->start_pfn = iova >> shift;
	BST_LWNN_TRACE_PRINTK(
		"dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
		(unsigned long long)pmman->pdev, (unsigned long long)domain,
		(unsigned long long)iovad, (unsigned long)shift);
	iova = alloc_iova_fast(iovad, size >> shift, (iova + size) >> shift,
			       true);
	iova <<= shift;
	if (iommu_map(domain, iova, addr, size, attr)) {
		BST_LWNN_DEV_ERR(pdev, "iommu_map fail.");
		return DMA_MAPPING_ERROR;
	}
	iovad->start_pfn = 0x80000000 >> shift;
	BST_LWNN_STAGE_PRINTK(
		"bst_lwnn_iommu_bypass OK. pa: 0x%llx iova: 0x%x size: 0x%x",
		addr, (dsp_ptr)iova, size);
#endif

	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct iommu_group  *group;
	struct device *pdev;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	group = iommu_group_get(pmman->pdev);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	iova_cache_get();
	iommu_attach_group(domain, group);
	/* for bstn_mem_manager_exit release */
	pbst_lwnn->mem_manager.group = group;
	pbst_lwnn->mem_manager.domain = domain;

	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	BST_LWNN_STAGE_PRINTK("dev 0x%llx domain 0x%llx shift 0x%lx",
			  (unsigned long long)pmman->pdev,
			  (unsigned long long)domain,
			  (unsigned long)shift);
	if(iommu_map_by_proxy(COREIP_CV_DSP_SID, iova, addr, size)) {
		BST_LWNN_DEV_ERR(pdev, "iommu_map_by_proxy fail.");
		return DMA_MAPPING_ERROR;
	}
	BST_LWNN_STAGE_PRINTK(
		"%s OK. pa: 0x%llx iova: 0x%x size: 0x%x", __func__, addr,
		(dsp_ptr)iova, size);

	return iova;
}

static void bst_lwnn_iommu_free(struct bst_lwnn *pbst_lwnn, uint32_t size,
				uint32_t align, dsp_ptr iova)
{
	struct bst_lwnn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct device *pdev;
	struct iova_domain *iovad;
	unsigned long shift;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BST_LWNN_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}
	iovad = (struct iova_domain *)((void *)domain->iova_cookie +
				       sizeof(uint64_t));
	shift = iova_shift(iovad);
	iommu_unmap(domain, (unsigned long)iova, size);
	free_iova_fast(iovad, (unsigned long)iova >> shift,
		       (unsigned long)size >> shift);

	BST_LWNN_STAGE_PRINTK(
		"bst_lwnn_iommu_free ok. iova: 0x%08x size: 0x%08x",
		(dsp_ptr)iova, size);
}

/*
 * @func    bst_lwnn_cma_free
 * @brief   This function frees the target continuous memory block.
 * @params  block - the pointer to the memory block
 * @return  void
 */
static void bst_lwnn_free(bst_lwnn_memblock_t *block)
{
	struct bst_lwnn *pbst_lwnn;
	struct bst_lwnn_mem_manager *pmman;
	if (NULL == block)
		return;

	pbst_lwnn = (struct bst_lwnn *)block->pbst_lwnn;
	pmman = &pbst_lwnn->mem_manager;

	pmman->dma_ops->put(block);
	return;
}

/***********************************************************************
* bst_lwnn mem ctx manger: add, delete, find ...
************************************************************************/
static struct bst_lwnn_mem_ctx *_find_mem_ctx(struct bst_lwnn *pbst_lwnn,
					      struct file *filp)
{
	struct bst_lwnn_mem_ctx *ctx;
	struct list_head *cur;
	struct bst_lwnn_mem_manager *pmman;

	pmman = &pbst_lwnn->mem_manager;
	list_for_each(cur, &pmman->mem_ctx_list) {
		ctx = container_of(cur, struct bst_lwnn_mem_ctx, link);
		if (ctx->filp == filp) {
			return ctx;
		}
	}
	return NULL;
}

int bst_lwnn_mem_ctx_add(struct bst_lwnn *pbst_lwnn, struct file *filp)
{
	struct bst_lwnn_mem_ctx *ctx, *search;
	struct device *pdev;
	struct bst_lwnn_mem_manager *pmman;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	ctx = devm_kzalloc(pdev, sizeof(*ctx), GFP_KERNEL);
	if (NULL == ctx) {
		BST_LWNN_DEV_ERR(pdev, "kmalloc failed");
		return -ENOMEM;
	}
	ctx->filp = filp;
	hash_init(ctx->ht);

	mutex_lock(&pmman->mm_mutex);
	search = _find_mem_ctx(pbst_lwnn, filp);
	if (NULL != search) {
		mutex_unlock(&pmman->mm_mutex);
		return -EINVAL;
	}

	list_add(&(ctx->link), &pmman->mem_ctx_list);
	mutex_unlock(&pmman->mm_mutex);
	BST_LWNN_TRACE_PRINTK("ctx %px added for filp %px", ctx, filp);
	return 0;
}

int bst_lwnn_mem_ctx_del(struct bst_lwnn *pbst_lwnn, struct file *filp)
{
	int i;
	struct bst_lwnn_mem_ctx *ctx;
	struct bst_lwnn_mem_manager *pmman;
	struct bst_lwnn_buffer *buffer;
	struct hlist_node *tmp;
	struct device *pdev;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (NULL == ctx) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(
			pdev,
			"fatal error: memory context with filp=%px not found",
			filp);
		return -EFAULT;
	}

	list_del(&ctx->link);

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		hash_del(&buffer->node);
		BST_LWNN_TRACE_PRINTK("buffer %px", buffer);

		switch (buffer->type) {
		case DMA_BUFF_ALLOC:
			pmman->ops->free(buffer->block);
			devm_kfree(pdev, buffer);
			break;

		case CMA_BUFF_IMPORT:
			pmman->ops->iommu_free(pbst_lwnn, buffer->size, 0,
					       buffer->bus_addr);
			devm_kfree(pdev, buffer);
			break;

		case DMA_BUFF_IMPORT:
			pmman->dma_ops->unmap_dmabuf(buffer->block);
			pmman->dma_ops->detach_dmabuf(buffer->block);
			dma_buf_put(buffer->dbuf);
			devm_kfree(pdev, buffer);
			break;

		default:
			BST_LWNN_DEV_ERR(pdev, "type %#x invalid\n",
					 buffer->type);
			break;
		}
	}
	devm_kfree(&pbst_lwnn->pdev->dev, ctx);
	mutex_unlock(&pmman->mm_mutex);
	BST_LWNN_TRACE_PRINTK("ctx %px removed for filp %px", ctx, filp);
	return 0;
}

/*
 * @func    lwnn_buffer_alloc
 * @brief   This function allocates a requested memory chunk to be used as a
 *          user buffer.
 * @params  filp - the misc device file pointer
 *          pbst_lwnn - the pointer to the bst_lwnn device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int lwnn_buffer_alloc(struct file *filp, struct bst_lwnn *pbst_lwnn,
		      struct bst_lwnn_user_buffer *pbuffer)
{
	int ret;
	struct bst_lwnn_buffer *buffer;
	struct bst_lwnn_mem_ctx *ctx;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	struct file *filp_mmap;
	// struct vm_area_struct   *vma;
	ret = -ENOENT;
	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BST_LWNN_DEV_ERR(pdev, "kmalloc failed!");
		ret = -ENOMEM;
		goto fail_before_struct_alloc;
	}
	//allocate the buffer
	buffer->block = pmman->ops->alloc(pbst_lwnn, pbuffer->size,
					  pbuffer->align, DMA_ATTR_PRIVILEGED);
	if (IS_ERR_OR_NULL(buffer->block)) {
		BST_LWNN_DEV_ERR(pdev, "alloc failed!");
		ret = -ENOMEM;
		goto fail_before_buffer_alloc;
	}
	if (pmman->enable_smmu) {
		buffer->bus_addr = (dsp_ptr)addr_truncate(
			pmman->dma_ops->cookie(buffer->block));
	} else {
		/*
		buffer->bus_addr = phys_to_bus(
			dma_to_phys(
				pmman->dma_ops->cookie(buffer->block)
			)
		);
		*/
		buffer->bus_addr = (dsp_ptr)phys_to_bus(
			pmman->dma_ops->cookie(buffer->block));
	}

	buffer->dbuf = pmman->dma_ops->get_dmabuf(buffer->block, O_RDWR);
	if (IS_ERR(buffer->dbuf)) {
		BST_LWNN_DEV_ERR(pdev, "Failed to get dmabuf");
		ret = PTR_ERR(buffer->dbuf);
		goto fail_after_buffer_alloc;
	}

	buffer->fd = dma_buf_fd(buffer->dbuf, O_CLOEXEC);
	if (buffer->fd < 0) {
		BST_LWNN_DEV_ERR(pdev, "Failed to get fd by dmabuf");
		ret = buffer->fd;
		goto fail_after_buffer_alloc;
	}
	buffer->size = buffer->block->size;
	buffer->type = DMA_BUFF_ALLOC;

	if (pmman->enable_smmu) {
		// audit_mmap_fd(buffer->fd, MAP_SHARED);
		filp_mmap = fget(buffer->fd);
		if (!filp_mmap) {
			BST_LWNN_DEV_ERR(pdev, "filp_mmap get null.");
			ret = -EINVAL;
			goto fail_after_buffer_alloc;
		}
		buffer->user_addr =
			vm_mmap(filp_mmap, 0 /* buffer->block->dma_addr */,
				buffer->block->size, PROT_READ | PROT_WRITE,
				MAP_SHARED, 0);
		fput(filp_mmap);
	} else {
		buffer->user_addr = vm_mmap(filp, 0, buffer->block->size,
					    PROT_READ | PROT_WRITE, MAP_SHARED,
					    buffer->block->dma_addr);
	}
	BST_LWNN_TRACE_PRINTK("user_addr: 0x%llx",
			      (unsigned long long)buffer->user_addr);

	pbuffer->bus_addr = buffer->bus_addr;
	pbuffer->user_addr = (void *)buffer->user_addr;
	pbuffer->align = pbuffer->align > HPAGE_SIZE ? pbuffer->align :
						       HPAGE_SIZE;
	BST_LWNN_TRACE_PRINTK("addr: %x, size: %x", pbuffer->bus_addr,
			      pbuffer->size);
	BST_LWNN_TRACE_PRINTK("buffer: %px", buffer);

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		goto fail_buffer_mmap;
	}
	//add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);
	return 0;

fail_buffer_mmap:
	BST_LWNN_DEV_ERR(pdev, "mmap fail");
	// vm_area_free(vma);
fail_after_buffer_alloc:
	BST_LWNN_DEV_ERR(pdev, "after buffer alloc fail");
	pmman->ops->free(buffer->block);
fail_before_buffer_alloc:
	BST_LWNN_DEV_ERR(pdev, "before buffer alloc fail");
	devm_kfree(pdev, buffer);
fail_before_struct_alloc:
	BST_LWNN_DEV_ERR(pdev, "before struct alloc fail");
	return ret;
}

/*
 * @func    lwnn_buffer_alloc
 * @brief   This function frees a user buffer.
 * @params  pbst_lwnn - the pointer to the bst_lwnn device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int lwnn_buffer_free(struct file *filp, struct bst_lwnn *pbst_lwnn,
		     struct bst_lwnn_user_buffer *pbuffer)
{
	struct bst_lwnn_buffer *buffer;
	struct bst_lwnn_mem_ctx *ctx;
	struct hlist_node *tmp;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	// struct vm_area_struct   *vma;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node,
				    pbuffer->bus_addr) {
		if ((buffer->type == DMA_BUFF_ALLOC) &&
		    (pbuffer->bus_addr == buffer->bus_addr)) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BST_LWNN_TRACE_PRINTK("buffer: %px", buffer);

			vm_munmap((unsigned long)buffer->user_addr,
				  buffer->block->size);
			if (buffer->fd != 0) {
				close_fd(buffer->fd);
			}
			if (buffer->dbuf != NULL) {
				dma_buf_put(buffer->dbuf);
			}
			//free the buffer
			pmman->ops->free(buffer->block);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);
	BST_LWNN_DEV_ERR(pdev, "no buffer @ baddr=0x%x", pbuffer->bus_addr);
	return -ENOENT;
}

int bst_lwnn_cma_buf_import(struct file *filp, struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_cma_buf *buf)
{
	struct bst_lwnn_buffer *buffer;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	phys_addr_t pa;
	int ret = 0;
	struct bst_lwnn_mem_ctx *ctx;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BST_LWNN_DEV_ERR(pdev, "fail to allocate bst_lwnn buff.");
		ret = -ENOMEM;
		goto cma_buf_import_fail_at_allocate_bst_lwnn;
	}

	pa = buf->pa;
	if (((pa & ~(SZ_4G - 1)) == 0) || (buf->size == 0)) {
		BST_LWNN_DEV_ERR(pdev, "import phy address error.");
		ret = -ENOMEM;
		goto cma_buf_import_fail_at_allocate_bst_lwnn;
	}

	pmman->ops->iommu_map(pbst_lwnn, buf->size, PAGE_SIZE, pa,
			      &buffer->bus_addr, IOMMU_READ | IOMMU_WRITE);
	buf->bus_addr = buffer->bus_addr;
	buffer->size = buf->size;
	buffer->type = CMA_BUFF_IMPORT;

	BST_LWNN_TRACE_PRINTK("pa 0x%llx, iova 0x%x, size 0x%x", buf->pa,
			      buf->bus_addr, buf->size);

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		ret = -EINVAL;
		goto cma_buf_ctx_fail;
	}
	// add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);
	return 0;

cma_buf_ctx_fail:
	pmman->ops->iommu_free(pbst_lwnn, buf->size, 0, buf->bus_addr);
cma_buf_import_fail_at_allocate_bst_lwnn:
	if (buffer)
		devm_kfree(pdev, buffer);
	return ret;
}

int bst_lwnn_cma_buf_return(struct file *filp, struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_cma_buf *buf)
{
	struct bst_lwnn_buffer *buffer;
	struct hlist_node *tmp;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	struct bst_lwnn_mem_ctx *ctx;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, buf->bus_addr) {
		if ((buffer->type == CMA_BUFF_IMPORT) &&
		    (buffer->bus_addr == buf->bus_addr)) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BST_LWNN_TRACE_PRINTK("buffer: %px", buffer);
			pmman->ops->iommu_free(pbst_lwnn, buf->size, 0,
					       buf->bus_addr);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BST_LWNN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
			 buf->bus_addr);
	return -ENOENT;
}

/*!
 * @brief           This function imports a dma-buf
 * @param[in]       pbst_lwnn The pointer to the bst_lwnn_device
 * @param[in,out]   buf The imported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
int bst_lwnn_dma_buf_import(struct file *filp, struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_dma_buf *buf)
{
	struct bst_lwnn_buffer *buffer;
	struct bst_lwnn_memblock *block;
	struct dma_buf *dmabuf = NULL;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	int ret = 0;
	struct bst_lwnn_mem_ctx *ctx;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BST_LWNN_DEV_ERR(pdev, "fail to allocate bst_lwnn buff.");
		ret = -ENOMEM;
		goto dma_buf_import_fail_at_allocate_bst_lwnn;
	}

	BST_LWNN_TRACE_PRINTK("get fd %d", buf->fd);
	dmabuf = dma_buf_get(buf->fd);
	buffer->dbuf = dmabuf;
	if (IS_ERR(dmabuf)) {
		BST_LWNN_DEV_ERR(pdev, "failed to get dma_buf(fd=%d)", buf->fd);
		ret = PTR_ERR(dmabuf);
		goto dma_buf_import_fail_at_dma_buf_get;
	}

	block = pmman->dma_ops->attach_dmabuf(pmman->pdev, dmabuf, buf->size,
					      DMA_BIDIRECTIONAL);
	buffer->block = block;
	if (IS_ERR(block)) {
		BST_LWNN_DEV_ERR(pmman->pdev, "failed to attach dma_buf(fd=%d)",
				 buf->fd);
		ret = PTR_ERR(block);
		goto dma_buf_import_fail_at_dma_buf_attach;
	}
	BST_LWNN_TRACE_PRINTK("attach dmabuf done");

	/* get the associated scatterlist for this buffer */
	ret = pmman->dma_ops->map_dmabuf(block);
	if (ret) {
		goto dma_buf_import_fail_at_dma_buf_map_attachment;
	}
	BST_LWNN_TRACE_PRINTK("map dmabuf done");

	if (pmman->enable_smmu) {
		buffer->bus_addr =
			(dsp_ptr)addr_truncate(pmman->dma_ops->cookie(block));
	} else {
		buffer->bus_addr =
			(dsp_ptr)phys_to_bus(pmman->dma_ops->cookie(block));
	}
	if (!buffer->bus_addr) {
		ret = -EINVAL;
		goto dma_buf_import_fial_at_dma_buf_unmap;
	}
	buffer->type = DMA_BUFF_IMPORT;
	buffer->fd = buf->fd;
	buffer->size = buffer->block->size;
	buf->bus_addr = buffer->bus_addr;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		ret = -EINVAL;
		goto dma_buf_import_fial_at_dma_buf_unmap;
	}
	// add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);

	return ret;

dma_buf_import_fial_at_dma_buf_unmap:
	pmman->dma_ops->unmap_dmabuf(block);
dma_buf_import_fail_at_dma_buf_map_attachment:
	pmman->dma_ops->detach_dmabuf(block);
dma_buf_import_fail_at_dma_buf_attach:
	dma_buf_put(buffer->dbuf);
dma_buf_import_fail_at_dma_buf_get:
	devm_kfree(pdev, buffer);
dma_buf_import_fail_at_allocate_bst_lwnn:

	return ret;
}

/*!
 * @brief       This function imports a dma-buf
 * @param[in]   pbst_lwnn The pointer to the bst_lwnn_device
 * @param[in]   buf The returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_dma_buf_return(struct file *filp, struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_dma_buf *buf)
{
	struct bst_lwnn_buffer *buffer;
	struct hlist_node *tmp;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;
	struct bst_lwnn_mem_ctx *ctx;
	int i;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}

	/* Lookup each hash elements to find fd. 
	Maybe more reasonable to use hash key latter. */
	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		if ((buffer->type == DMA_BUFF_IMPORT) &&
		    (buffer->fd == buf->fd)) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BST_LWNN_TRACE_PRINTK("buffer: %px", buffer);
			pmman->dma_ops->unmap_dmabuf(buffer->block);
			pmman->dma_ops->detach_dmabuf(buffer->block);
			dma_buf_put(buffer->dbuf);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BST_LWNN_DEV_ERR(pdev, "buffer for dma_buf(fd=%d) not found", buf->fd);
	return -ENOENT;
}

/*!
 * @brief       This function export a dma-buf
 * @param[in]   pbst_lwnn The pointer to the bst_lwnn_device
 * @param[in]   buf The returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_dma_buf_export(struct file *filp, struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_dma_buf *buf)
{
	struct bst_lwnn_buffer *buffer;
	struct bst_lwnn_mem_ctx *ctx;
	struct hlist_node *tmp;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}
	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, buf->bus_addr) {
		if ((buffer->type == DMA_BUFF_ALLOC) &&
		    (buffer->bus_addr == buf->bus_addr)) {
			mutex_unlock(&pmman->mm_mutex);
			BST_LWNN_TRACE_PRINTK("buffer: %px", buffer);
			buf->fd = buffer->fd;
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);
	BST_LWNN_DEV_ERR(pdev, "buffer fd for bus_addr 0x%08x not found",
			 buf->bus_addr);
	return -ENOENT;
}

int bst_lwnn_dma_buf_flush(struct file *filp, struct bst_lwnn *pbst_lwnn,
			   struct bst_lwnn_user_buffer *pbuffer)
{
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;

	struct bst_lwnn_buffer *lwnn_buffer;
	struct bst_lwnn_mem_ctx *ctx;
	struct hlist_node *tmp;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, lwnn_buffer, tmp, node,
				    pbuffer->bus_addr) {
		if (pbuffer->bus_addr == lwnn_buffer->bus_addr) {
			mutex_unlock(&pmman->mm_mutex);
			if (lwnn_buffer->type == DMA_BUFF_ALLOC) {
				pmman->dma_ops->prepare(
					(void *)lwnn_buffer->block);
			} else if ((lwnn_buffer->type == CMA_BUFF_IMPORT) ||
				   (lwnn_buffer->type == DMA_BUFF_IMPORT)) {
				dma_sync_single_for_device(pmman->pdev,
							   pbuffer->bus_addr,
							   pbuffer->size,
							   DMA_TO_DEVICE);
			}
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BST_LWNN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
			 pbuffer->bus_addr);
	return -ENOENT;
}

int bst_lwnn_dma_buf_invalidate(struct file *filp, struct bst_lwnn *pbst_lwnn,
				struct bst_lwnn_user_buffer *pbuffer)
{
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;

	struct bst_lwnn_buffer *lwnn_buffer;
	struct bst_lwnn_mem_ctx *ctx;
	struct hlist_node *tmp;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_lwnn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_LWNN_DEV_ERR(pdev, "memory context with filp=%px not found",
				 filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, lwnn_buffer, tmp, node,
				    pbuffer->bus_addr) {
		if (pbuffer->bus_addr == lwnn_buffer->bus_addr) {
			mutex_unlock(&pmman->mm_mutex);
			if (lwnn_buffer->type == DMA_BUFF_ALLOC) {
				pmman->dma_ops->finish(
					(void *)lwnn_buffer->block);
			} else if ((lwnn_buffer->type == CMA_BUFF_IMPORT) ||
				   (lwnn_buffer->type == DMA_BUFF_IMPORT)) {
				dma_sync_single_for_cpu(pmman->pdev,
							pbuffer->bus_addr,
							pbuffer->size,
							DMA_FROM_DEVICE);
			}
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BST_LWNN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
			 pbuffer->bus_addr);
	return -ENOENT;
}

/*
 * @func    bst_lwnn_mem_manager_init
 * @brief   This is the initialization function of the memory manager. It sets
 *          up the reseved memory and DMA configs of the device.
 * @params  pbst_lwnn - the pointer to the BST_LWNN device
 * @return  0 - success
 *          error code - failure
 */
int bst_lwnn_mem_manager_init(struct bst_lwnn *pbst_lwnn)
{
	int ret;
	struct device *pdev;
	struct bst_lwnn_mem_manager *pmman;

	struct iommu_domain *domain;
	struct iommu_group *group;
	struct iova_domain *iovad;

	/* tips: In some drvs, pdev may diff with pmman->pdev */
	pdev = &pbst_lwnn->pdev->dev;
	pmman = &pbst_lwnn->mem_manager;
	if (pmman->enable_smmu && dev_cvsmm != NULL) {
		pmman->pdev = dev_cvsmm;
		BST_LWNN_STAGE_PRINTK("memory manager using dev_cvsmm...");
	} else {
		pmman->pdev = pdev;
		BST_LWNN_STAGE_PRINTK("memory manager using dev_lwnn ...");
	};

	pmman->dma_ops = &bst_lwnn_dma_memops;
	pmman->ops = &bst_lwnn_cma_memops;

	if (pmman->enable_smmu) {
		// set dma mask and coherent mask
		ret = dma_set_mask_and_coherent(pmman->pdev, DMA_BIT_MASK(32));
		if (ret) {
			BST_LWNN_DEV_ERR(pmman->pdev,
					 "dma_set_coherent_mask fail, ret %d",
					 ret);
			return -ENODEV;
		}
		BST_LWNN_TRACE_PRINTK("dma_set_coherent_mask OK.");
	} else {
		// set dma mask and coherent mask
		ret = dma_set_mask_and_coherent(pmman->pdev, DMA_BIT_MASK(36));
		if (ret) {
			BST_LWNN_DEV_ERR(pmman->pdev,
					 "dma_set_coherent_mask fail, ret %d",
					 ret);
			return -ENODEV;
		}
		BST_LWNN_TRACE_PRINTK("dma_set_coherent_mask OK.");

		// init reserved memory
		ret = of_reserved_mem_device_init(pdev);
		if (ret < 0) {
			BST_LWNN_DEV_ERR(
				pdev,
				"of_reserved_mem_device_init fail, ret: %d",
				ret);
			return -ENODEV;
		}
		BST_LWNN_TRACE_PRINTK("of_reserved_mem_device_init OK.");
	}
	bst_lwnn_dma_contig_set_max_seg_size(pmman->pdev, UINT_MAX);

	if (pmman->enable_smmu) {
		group = iommu_group_get(pmman->pdev);
		domain = iommu_get_domain_for_dev(pmman->pdev);
		iova_cache_get();
		iommu_attach_group(domain, group);
		// iommu_setup_dma_ops(pmman->pdev, 0, DMA_BIT_MASK(32));

		pbst_lwnn->mem_manager.group = group;
		pbst_lwnn->mem_manager.domain = domain;

		iovad = (struct iova_domain *)((void *)domain->iova_cookie +
					       sizeof(uint64_t));
		pbst_lwnn->mem_manager.iovad = iovad;
	}

	INIT_LIST_HEAD(&pmman->mem_ctx_list);
	mutex_init(&pmman->mm_mutex);

	return 0;
}

/*
 * @func    bst_lwnn_mem_manager_exit
 * @brief   This is the cleanup function of the memory manager. It frees all
 *          allocated memory blocks including the assigned memory of the DSP.
 * @params  pbst_lwnn - the pointer to the BST_LWNN device
 * @return  void
 */
void bst_lwnn_mem_manager_exit(struct bst_lwnn *pbst_lwnn)
{
	struct bst_lwnn_mem_manager *pmman;
	struct bst_lwnn_mem_ctx *ctx;
	struct list_head *tmp, *cur;
	pmman = &pbst_lwnn->mem_manager;
	BST_LWNN_TRACE_PRINTK("Exit bst_lwnn mem manager.");

	if (pbst_lwnn->mem_manager.enable_smmu) {
		if (pbst_lwnn->mem_manager.iova_resv_dummy) {
			__free_iova(pbst_lwnn->mem_manager.iovad,
				    pbst_lwnn->mem_manager.iova_resv_dummy);
		}
		iommu_detach_group(pbst_lwnn->mem_manager.domain,
				   pbst_lwnn->mem_manager.group);
		iova_cache_put();
		iommu_group_put(pbst_lwnn->mem_manager.group);
	}

	list_for_each_safe(cur, tmp, &pmman->mem_ctx_list) {
		list_del(cur);
		ctx = container_of(cur, struct bst_lwnn_mem_ctx, link);
		// bst_lwnn_del_ctx();
		bst_lwnn_mem_ctx_del(pbst_lwnn, ctx->filp);
	}
	if (!pmman->enable_smmu) {
		of_reserved_mem_device_release(pmman->pdev);
	}
	return;
}
