// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_mem_manager.c
 * @brief   This file is the source code file of the memory manager of the BSTN
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
#include "bstn.h"
#include "bstn_mem_manager.h"
#include "bstn_dma_manager.h"

typedef enum {
	DMA_BUFF_ALLOC,
	DMA_BUFF_IMPORT,
	CMA_BUFF_IMPORT,
} bstn_buf_type_e;

int close_fd(unsigned fd);

/******************************************************************************
* bstn mem ops: alloc, iommu_map_fw, free...
*******************************************************************************/
static bstn_memblock_t *bstn_alloc(struct bstn_device *pbstn, uint32_t size,
				   uint32_t align, unsigned long attr);

static void bstn_iommu_map(struct bstn_device *pbstn, uint32_t size,
			   uint32_t align, phys_addr_t addr, dsp_ptr *iova,
			   unsigned long attr);

static dma_addr_t bstn_iommu_map_fw(struct bstn_device *pbstn, uint32_t size,
				    uint32_t align, phys_addr_t addr);

static void bstn_iommu_free(struct bstn_device *pbstn, uint32_t size,
			    uint32_t align, dsp_ptr iova);

static void bstn_free(bstn_memblock_t *block);

static struct bstn_mem_ops bstn_cma_memops = {
	.alloc = bstn_alloc,
	.iommu_map_fw = bstn_iommu_map_fw,
	.iommu_map = bstn_iommu_map,
	.iommu_free = bstn_iommu_free,
	.free = bstn_free,
};

/*
 * @func    bstn_alloc
 * @brief   This function allocates a requested continuous memory block.
 * @params  mem_manager - the pointer to the memory mem_manager
 *          size - the requested size
 * @return  the pointer to the memory block - success
 *          NULL - failure
 */
static bstn_memblock_t *bstn_alloc(struct bstn_device *pbstn, uint32_t size,
				   uint32_t align, unsigned long attr)
{
	struct bstn_mem_manager *pmman;
	struct bstn_memblock *block;
	struct device *pdev;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;
	block = NULL;
	if (0 == size) {
		goto exit;
	}
	size = ALIGN(size, align <= HPAGE_SIZE ? HPAGE_SIZE : align);
	attr |= DMA_ATTR_FORCE_CONTIGUOUS;
	block = pmman->dma_ops->dma_alloc(pmman->pdev, attr, size,
					  DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (IS_ERR_OR_NULL(block))
		goto exit;
	block->pbstn = (void *)pbstn;

exit:
	if (NULL == block) {
		BSTN_DEV_ERR(pdev, "bstn_alloc fail.");
	} else {
		BSTN_TRACE_PRINTK("bstn_alloc ok");
	}
	return block;
}

static void bstn_iommu_map(struct bstn_device *pbstn, uint32_t size,
			   uint32_t align, phys_addr_t addr, dsp_ptr *iova,
			   unsigned long attr)
{
	struct bstn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	struct device *pdev;
	dma_addr_t new_iova;
	unsigned long shift;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BSTN_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}

	iovad = (struct iova_domain *)((void *)domain->iova_cookie + sizeof(uint64_t));
	shift = iova_shift(iovad);

	BSTN_TRACE_PRINTK("dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
			  (unsigned long long)pmman->pdev,
			  (unsigned long long)domain, (unsigned long long)iovad,
			  (unsigned long)shift);
	new_iova = alloc_iova_fast(iovad, size >> shift,
				   DMA_BIT_MASK(32) >> shift, true);
	new_iova <<= shift;
	if (iommu_map(domain, new_iova, addr, size, attr)) {
		BSTN_DEV_ERR(pdev, "iommu_map 0x%llx fail.", addr);
		return;
	}
	*iova = (dsp_ptr)new_iova;
	BSTN_TRACE_PRINTK("bstn_iommu_map ok. iova: 0x%08x size: 0x%08x",
			  (dsp_ptr)new_iova, size);
}

static dma_addr_t bstn_iommu_map_fw(struct bstn_device *pbstn, uint32_t size,
				    uint32_t align, phys_addr_t addr)
{
	struct bstn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct iommu_group  *group;
	struct device *pdev;
	dma_addr_t iova;
	unsigned long shift;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	group = iommu_group_get(pmman->pdev);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	iova_cache_get();
	iommu_attach_group(domain, group);
	/* for bstn_mem_manager_exit release */
	pbstn->mem_manager.group = group;
	pbstn->mem_manager.domain = domain;

	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	BSTN_TRACE_PRINTK("dev 0x%llx domain 0x%llx shift 0x%lx",
			  (unsigned long long)pmman->pdev,
			  (unsigned long long)domain,
			  (unsigned long)shift);
	iova = _FW_IOVA_BASE_;
	if(iommu_map_by_proxy(COREIP_NET_BTMEM_SID, iova, addr, size)) {
		BSTN_DEV_ERR(pdev, "iommu_map_by_proxy fail.");
		return DMA_MAPPING_ERROR;
	}
	BSTN_STAGE_PRINTK(
		"bstn_iommu_map_fw OK. pa: 0x%llx iova: 0x%x size: 0x%x", addr,
		(dsp_ptr)iova, size);

	return iova;
}

static void bstn_iommu_free(struct bstn_device *pbstn, uint32_t size,
			    uint32_t align, dsp_ptr iova)
{
	struct bstn_mem_manager *pmman;
	struct iommu_domain *domain;
	struct device *pdev;
	struct iova_domain *iovad;
	unsigned long shift;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;
	size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BSTN_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}

	iovad = (struct iova_domain *)((void *)domain->iova_cookie + sizeof(uint64_t));
	shift = iova_shift(iovad);

	iommu_unmap(domain, (unsigned long)iova, size);
	free_iova_fast(iovad, (unsigned long)iova >> shift,
		       (unsigned long)size >> shift);

	BSTN_STAGE_PRINTK("bstn_iommu_free OK. iova: 0x%08x size: 0x%08x",
			  (dsp_ptr)iova, size);
}

/*
 * @func    bstn_cma_free
 * @brief   This function frees the target continuous memory block.
 * @params  block - the pointer to the memory block
 * @return  void
 */
static void bstn_free(bstn_memblock_t *block)
{
	struct bstn_device *pbstn;
	struct bstn_mem_manager *pmman;

	if (NULL == block)
		return;

	pbstn = (struct bstn_device *)block->pbstn;
	pmman = &pbstn->mem_manager;

	pmman->dma_ops->put(block);
	return;
}

/***********************************************************************
* bstn mem ctx manger: add, delete, find ...
************************************************************************/
static struct bstn_mem_ctx *_find_mem_ctx(struct bstn_device *pbstn,
					  struct file *filp)
{
	struct bstn_mem_ctx *ctx;
	struct list_head *cur;
	struct bstn_mem_manager *pmman;

	pmman = &pbstn->mem_manager;
	list_for_each(cur, &pmman->mem_ctx_list) {
		ctx = container_of(cur, struct bstn_mem_ctx, link);
		if (ctx->filp == filp) {
			return ctx;
		}
	}
	return NULL;
}

int bstn_mem_ctx_add(struct bstn_device *pbstn, struct file *filp)
{
	struct bstn_mem_ctx *ctx, *search;
	struct device *pdev;
	struct bstn_mem_manager *pmman;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	ctx = devm_kzalloc(pdev, sizeof(*ctx), GFP_KERNEL);
	if (NULL == ctx) {
		BSTN_DEV_ERR(pdev, "kmalloc failed");
		return -ENOMEM;
	}
	ctx->filp = filp;
	hash_init(ctx->ht);

	mutex_lock(&pmman->mm_mutex);
	search = _find_mem_ctx(pbstn, filp);
	if (NULL != search) {
		mutex_unlock(&pmman->mm_mutex);
		return -EINVAL;
	}

	list_add(&(ctx->link), &pmman->mem_ctx_list);
	mutex_unlock(&pmman->mm_mutex);
	BSTN_TRACE_PRINTK("ctx %px added for filp %px", ctx, filp);
	return 0;
}

int bstn_mem_ctx_del(struct bstn_device *pbstn, struct file *filp)
{
	int i;
	struct bstn_mem_ctx *ctx;
	struct bstn_mem_manager *pmman;
	struct bstn_buffer *buffer;
	struct hlist_node *tmp;
	struct device *pdev;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (NULL == ctx) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(
			pdev,
			"fatal error: memory context with filp=%px not found",
			filp);
		return -EFAULT;
	}

	list_del(&ctx->link);

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		hash_del(&buffer->node);
		BSTN_TRACE_PRINTK("buffer %px", buffer);

		switch (buffer->type) {
		case DMA_BUFF_ALLOC:
			pmman->ops->free(buffer->block);
			devm_kfree(pdev, buffer);
			break;

		case CMA_BUFF_IMPORT:
			pmman->ops->iommu_free(pbstn, buffer->size, 0,
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
			BSTN_DEV_ERR(pdev, "type %#x invalid\n", buffer->type);
			break;
		}
	}
	devm_kfree(&pbstn->pdev->dev, ctx);
	mutex_unlock(&pmman->mm_mutex);
	BSTN_TRACE_PRINTK("ctx %px removed for filp %px", ctx, filp);
	return 0;
}

/*
 * @func    bsnn_buffer_alloc
 * @brief   This function allocates a requested memory chunk to be used as a
 *          user buffer.
 * @params  filp - the misc device file pointer
 *          pbstn - the pointer to the bstn device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int bsnn_buffer_alloc(struct file *filp, struct bstn_device *pbstn,
		      struct bsnn_buffer *pbuffer)
{
	int ret;
	struct bstn_buffer *buffer;
	struct bstn_mem_ctx *ctx;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	struct file *filp_mmap;
	struct iommu_domain *domain;
	phys_addr_t phys_addr;

	ret = -ENOENT;
	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BSTN_DEV_ERR(pdev, "kmalloc failed!");
		ret = -ENOMEM;
		goto fail_before_struct_alloc;
	}
	//allocate the buffer
	buffer->block =
		pmman->ops->alloc(pbstn, pbuffer->size, pbuffer->align, 0);
	if (IS_ERR_OR_NULL(buffer->block)) {
		BSTN_DEV_ERR(pdev, "alloc failed!");
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
		BSTN_DEV_ERR(pdev, "Failed to get dmabuf");
		ret = PTR_ERR(buffer->dbuf);
		goto fail_after_buffer_alloc;
	}

	buffer->fd = dma_buf_fd(buffer->dbuf, O_CLOEXEC);
	if (buffer->fd < 0) {
		BSTN_DEV_ERR(pdev, "Failed to get fd by dmabuf");
		ret = buffer->fd;
		goto fail_after_buffer_alloc;
	}
	buffer->size = buffer->block->size;
	buffer->type = DMA_BUFF_ALLOC;

	if (pmman->enable_smmu) {
		// audit_mmap_fd(buffer->fd, MAP_SHARED);
		filp_mmap = fget(buffer->fd);
		if (!filp_mmap) {
			BSTN_DEV_ERR(pdev, "filp_mmap get null.");
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
	BSTN_TRACE_PRINTK("user_addr: 0x%llx",
			  (unsigned long long)buffer->user_addr);

	pbuffer->baddr = buffer->bus_addr;
	pbuffer->uaddr = (void *)buffer->user_addr;
	pbuffer->align = pbuffer->align > HPAGE_SIZE ? pbuffer->align :
						       HPAGE_SIZE;
	BSTN_TRACE_PRINTK("addr: %x, size: %x", pbuffer->baddr, pbuffer->size);
	BSTN_TRACE_PRINTK("buffer: %px, dbuf %px, dbuf_size: %lx", buffer,
			  buffer->dbuf, buffer->dbuf->size);

	// TODO: workaround
	if (pmman->enable_smmu) {
		domain = iommu_get_domain_for_dev(pmman->pdev);
		phys_addr = iommu_iova_to_phys(domain, buffer->block->dma_addr);
		pbuffer->handle = (void *)phys_addr;
	} else {
		phys_addr = bus_to_phys(pbuffer->baddr);
		pbuffer->handle = (void *)phys_addr;
	}
	BSTN_TRACE_PRINTK("phys_addr: %p", pbuffer->handle);

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		goto fail_buffer_mmap;
	}
	//add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);
	return 0;

fail_buffer_mmap:
	BSTN_DEV_ERR(pdev, "mmap fail");
	// vm_area_free(vma);
fail_after_buffer_alloc:
	BSTN_DEV_ERR(pdev, "after buffer alloc fail");
	pmman->ops->free(buffer->block);
fail_before_buffer_alloc:
	BSTN_DEV_ERR(pdev, "before buffer alloc fail");
	devm_kfree(pdev, buffer);
fail_before_struct_alloc:
	BSTN_DEV_ERR(pdev, "before struct alloc fail");
	return ret;
}

/*
 * @func    bsnn_buffer_alloc
 * @brief   This function frees a user buffer.
 * @params  pbstn - the pointer to the bstn device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int bsnn_buffer_free(struct file *filp, struct bstn_device *pbstn,
		     struct bsnn_buffer *pbuffer)
{
	struct bstn_buffer *buffer;
	struct bstn_mem_ctx *ctx;
	struct hlist_node *tmp;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	// struct vm_area_struct   *vma;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node,
				    pbuffer->baddr) {
		if ((buffer->type == DMA_BUFF_ALLOC) &&
		    (pbuffer->baddr == buffer->bus_addr)) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BSTN_TRACE_PRINTK("buffer: %px", buffer);

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
	BSTN_DEV_ERR(pdev, "no buffer @ baddr=0x%x", pbuffer->baddr);
	return -ENOENT;
}

/*!
 * @brief           This function imports a dma-buf
 * @param[in]				filp the misc device file pointer
 * @param[in]       pbstn The pointer to the bstn_device
 * @param[in,out]   buf The imported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
int bstn_dma_buf_import(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf)
{
	struct bstn_buffer *buffer;
	struct bstn_memblock *block;
	struct dma_buf *dmabuf = NULL;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	int ret = 0;
	struct bstn_mem_ctx *ctx;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BSTN_DEV_ERR(pdev, "fail to allocate bstn buff.");
		ret = -ENOMEM;
		goto dma_buf_import_fail_at_allocate_bstn;
	}

	BSTN_TRACE_PRINTK("get fd %d", buf->fd);
	dmabuf = dma_buf_get(buf->fd);
	buffer->dbuf = dmabuf;
	if (IS_ERR(dmabuf)) {
		BSTN_DEV_ERR(pdev, "failed to get dma_buf(fd=%d)", buf->fd);
		ret = PTR_ERR(dmabuf);
		goto dma_buf_import_fail_at_dma_buf_get;
	}

	block = pmman->dma_ops->attach_dmabuf(pmman->pdev, dmabuf, buf->size,
					      DMA_BIDIRECTIONAL);
	buffer->block = block;
	if (IS_ERR(block)) {
		BSTN_DEV_ERR(pmman->pdev, "failed to attach dma_buf(fd=%d)",
			     buf->fd);
		ret = PTR_ERR(block);
		goto dma_buf_import_fail_at_dma_buf_attach;
	}
	BSTN_TRACE_PRINTK("attach dmabuf done");

	/* get the associated scatterlist for this buffer */
	ret = pmman->dma_ops->map_dmabuf(block);
	if (ret) {
		goto dma_buf_import_fail_at_dma_buf_map_attachment;
	}
	BSTN_TRACE_PRINTK("map dmabuf done");

	if (pmman->enable_smmu) {
		buffer->bus_addr = buffer->block->dma_addr;
	} else {
		buffer->bus_addr =
			(dsp_ptr)phys_to_bus(pmman->dma_ops->cookie(block));
	}
	BSTN_TRACE_PRINTK("bus_addr 0x%x, buf->size  0x%lx", buffer->bus_addr,
			  buffer->block->size);

	if (!buffer->bus_addr) {
		ret = -EINVAL;
		goto dma_buf_import_fial_at_dma_buf_unmap;
	}
	buffer->type = DMA_BUFF_IMPORT;
	buffer->fd = buf->fd;
	buffer->size = buffer->block->size;
	buf->bus_addr = buffer->bus_addr;
	buf->size = buffer->block->size;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
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
dma_buf_import_fail_at_allocate_bstn:

	return ret;
}

/*!
 * @brief       This function imports a dma-buf
 * @param[in]   pbstn The pointer to the bstn_device
 * @param[in]   buf The returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
int bstn_dma_buf_return(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf)
{
	struct bstn_buffer *buffer;
	struct hlist_node *tmp;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	struct bstn_mem_ctx *ctx;
	int i;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
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
			BSTN_TRACE_PRINTK("buffer: %px", buffer);
			pmman->dma_ops->unmap_dmabuf(buffer->block);
			pmman->dma_ops->detach_dmabuf(buffer->block);
			dma_buf_put(buffer->dbuf);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BSTN_DEV_ERR(pdev, "buffer for dma_buf(fd=%d) not found", buf->fd);
	return -ENOENT;
}

int bstn_cma_buf_import(struct file *filp, struct bstn_device *pbstn,
			struct bstn_cma_buf *buf)
{
	struct bstn_buffer *buffer;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	phys_addr_t pa;
	int ret = 0;
	struct bstn_mem_ctx *ctx;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BSTN_DEV_ERR(pdev, "fail to allocate bstn buff.");
		ret = -ENOMEM;
		goto cma_buf_import_fail_at_allocate_bstn;
	}

	pa = buf->pa;
	if (((pa & ~(SZ_4G - 1)) == 0) || (buf->size == 0)) {
		BSTN_DEV_ERR(pdev, "import phy address error.");
		ret = -ENOMEM;
		goto cma_buf_import_fail_at_allocate_bstn;
	}

	pmman->ops->iommu_map(pbstn, buf->size, PAGE_SIZE, pa,
			      &buffer->bus_addr, IOMMU_READ | IOMMU_WRITE);
	buf->bus_addr = buffer->bus_addr;
	buffer->size = buf->size;
	buffer->type = CMA_BUFF_IMPORT;

	BSTN_TRACE_PRINTK("pa 0x%llx, iova 0x%x, size 0x%x", buf->pa,
			  buf->bus_addr, buf->size);

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		ret = -EINVAL;
		goto cma_buf_ctx_fail;
	}
	// add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);
	return 0;

cma_buf_ctx_fail:
	pmman->ops->iommu_free(pbstn, buf->size, 0, buf->bus_addr);
cma_buf_import_fail_at_allocate_bstn:
	if (buffer)
		devm_kfree(pdev, buffer);
	return ret;
}

int bstn_cma_buf_return(struct file *filp, struct bstn_device *pbstn,
			struct bstn_cma_buf *buf)
{
	struct bstn_buffer *buffer;
	struct hlist_node *tmp;
	struct bstn_mem_manager *pmman;
	struct device *pdev;
	struct bstn_mem_ctx *ctx;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, buf->bus_addr) {
		if ((buffer->type == CMA_BUFF_IMPORT) &&
		    (buffer->bus_addr == buf->bus_addr)) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BSTN_TRACE_PRINTK("buffer: %px", buffer);
			pmman->ops->iommu_free(pbstn, buf->size, 0,
					       buf->bus_addr);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BSTN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found", buf->bus_addr);
	return -ENOENT;
}
/*!
 * @brief       This function export a dma-buf
 * @param[in]   pbstn The pointer to the bstn_device
 * @param[in]   buf The returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
int bstn_dma_buf_export(struct file *filp, struct bstn_device *pbstn,
			struct bstn_dma_buf *buf)
{
	struct bstn_buffer *buffer;
	struct bstn_mem_ctx *ctx;
	struct hlist_node *tmp;
	struct bstn_mem_manager *pmman;
	struct device *pdev;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}
	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, buf->bus_addr) {
		if ((buffer->type == DMA_BUFF_ALLOC) &&
		    (buffer->bus_addr == buf->bus_addr)) {
			mutex_unlock(&pmman->mm_mutex);
			BSTN_TRACE_PRINTK("buffer: %px", buffer);
			buf->fd = buffer->fd;
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);
	BSTN_DEV_ERR(pdev, "buffer fd for bus_addr 0x%08x not found",
		     buf->bus_addr);
	return -ENOENT;
}

int bstn_dma_buf_flush(struct file *filp, struct bstn_device *pbstn,
		       struct bsnn_buffer *pbuffer)
{
	struct bstn_mem_manager *pmman;
	struct device *pdev;

	struct bstn_buffer *bstn_buffer;
	struct bstn_mem_ctx *ctx;
	struct hlist_node *tmp;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, bstn_buffer, tmp, node,
				    pbuffer->baddr) {
		if (pbuffer->baddr == bstn_buffer->bus_addr) {
			mutex_unlock(&pmman->mm_mutex);
			if (bstn_buffer->type == DMA_BUFF_ALLOC) {
				pmman->dma_ops->prepare(
					(void *)bstn_buffer->block);
			} else if ((bstn_buffer->type == CMA_BUFF_IMPORT) ||
				   (bstn_buffer->type == DMA_BUFF_IMPORT)) {
				dma_sync_single_for_device(pmman->pdev,
							   pbuffer->baddr,
							   pbuffer->size,
							   DMA_TO_DEVICE);
			}
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BSTN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
		     pbuffer->baddr);
	return -ENOENT;
}

int bstn_dma_buf_invalidate(struct file *filp, struct bstn_device *pbstn,
			    struct bsnn_buffer *pbuffer)
{
	struct bstn_mem_manager *pmman;
	struct device *pdev;

	struct bstn_buffer *bstn_buffer;
	struct bstn_mem_ctx *ctx;
	struct hlist_node *tmp;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, bstn_buffer, tmp, node,
				    pbuffer->baddr) {
		if (pbuffer->baddr == bstn_buffer->bus_addr) {
			mutex_unlock(&pmman->mm_mutex);
			if (bstn_buffer->type == DMA_BUFF_ALLOC) {
				pmman->dma_ops->finish(
					(void *)bstn_buffer->block);
			} else if ((bstn_buffer->type == CMA_BUFF_IMPORT) ||
				   (bstn_buffer->type == DMA_BUFF_IMPORT)) {
				dma_sync_single_for_cpu(pmman->pdev,
							pbuffer->baddr,
							pbuffer->size,
							DMA_FROM_DEVICE);
			}
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BSTN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
		     pbuffer->baddr);
	return -ENOENT;
}

int bstn_dma_buf_sync(struct file *filp, struct bstn_device *pbstn,
		      struct bstnpu_mem_sync *pbuffer)
{
	struct bstn_mem_manager *pmman;
	struct device *pdev;

	struct bstn_buffer *bstn_buffer;
	struct bstn_mem_ctx *ctx;
	struct hlist_node *tmp;

	pmman = &pbstn->mem_manager;
	pdev = &pbstn->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbstn, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BSTN_DEV_ERR(pdev, "memory context with filp=%px not found",
			     filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, bstn_buffer, tmp, node,
				    pbuffer->baddr) {
		if (pbuffer->baddr == bstn_buffer->bus_addr) {
			mutex_unlock(&pmman->mm_mutex);
			switch (bstn_buffer->type) {
			case DMA_BUFF_ALLOC:
				if (pbuffer->flags & BSTN_MEM_SYNC_TO_DEVICE) {
					pmman->dma_ops->dma_sync(
						(void *)bstn_buffer->block,
						pbuffer->offset, pbuffer->size,
						DMA_TO_DEVICE);
				}
				if (pbuffer->flags &
				    BSTN_MEM_SYNC_FROM_DEVICE) {
					pmman->dma_ops->dma_sync(
						(void *)bstn_buffer->block,
						pbuffer->offset, pbuffer->size,
						DMA_FROM_DEVICE);
				}
				break;

			case CMA_BUFF_IMPORT:
			case DMA_BUFF_IMPORT:
				if (pbuffer->flags & BSTN_MEM_SYNC_TO_DEVICE) {
					dma_sync_single_for_device(
						pmman->pdev,
						pbuffer->baddr +
							pbuffer->offset,
						pbuffer->size, DMA_TO_DEVICE);
				}
				if (pbuffer->flags &
				    BSTN_MEM_SYNC_FROM_DEVICE) {
					dma_sync_single_for_cpu(
						pmman->pdev,
						pbuffer->baddr +
							pbuffer->offset,
						pbuffer->size, DMA_FROM_DEVICE);
				}
				break;

			default:
				BSTN_DEV_ERR(pdev, "type %#x invalid\n",
					     bstn_buffer->type);
				break;
			}
			return 0;
		}
	}
	mutex_unlock(&pmman->mm_mutex);

	BSTN_DEV_ERR(pdev, "buffer for bus_addr(%#x) not found",
		     pbuffer->baddr);
	return -ENOENT;
}

/*
 * @func    bstn_mem_manager_init
 * @brief   This is the initialization function of the memory manager. It sets
 *          up the reseved memory and DMA configs of the device.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_mem_manager_init(struct bstn_device *pbstn)
{
	int ret;
	struct device *pdev;
	struct bstn_mem_manager *pmman;

	/* tips: In some drvs, pdev may diff with pmman->pdev */
	pdev = &pbstn->pdev->dev;
	pmman = &pbstn->mem_manager;
	pmman->pdev = &pbstn->pdev->dev;

	pmman->dma_ops = &bstn_dma_memops;
	pmman->ops = &bstn_cma_memops;

	pmman->group  = NULL;
	pmman->domain = NULL;

	if (pmman->enable_smmu) {
		// set dma mask and coherent mask
		ret = dma_set_mask_and_coherent(pmman->pdev, DMA_BIT_MASK(32));
		if (ret) {
			BSTN_DEV_ERR(pmman->pdev,
				     "dma_set_coherent_mask fail, ret %d", ret);
			return -ENODEV;
		}
		BSTN_TRACE_PRINTK("dma_set_coherent_mask OK.");
	} else {
		ret = device_property_read_u64_array(pdev, "rmem-base",
						     &pmman->rmem_base, 1);
		if (ret < 0) {
			BSTN_DEV_ERR(pdev, "no rmem-base property, ret %d",
				     ret);
			return -ENOENT;
		}
		ret = device_property_read_u64_array(pdev, "rmem-size",
						     &pmman->rmem_size, 1);
		if (ret < 0) {
			BSTN_DEV_ERR(pdev, "no rmem-size property, ret %d",
				     ret);
			return -ENOENT;
		}
		BSTN_TRACE_PRINTK("reserved memory: base 0x%llx size 0x%llx",
				  pmman->rmem_base, pmman->rmem_size);

		// set dma mask and coherent mask
		ret = dma_set_mask_and_coherent(pmman->pdev, DMA_BIT_MASK(36));
		if (ret) {
			BSTN_DEV_ERR(pmman->pdev,
				     "dma_set_coherent_mask fail, ret %d", ret);
			return -ENODEV;
		}
		BSTN_TRACE_PRINTK("dma_set_coherent_mask OK.");

		// init reserved memory
		// ret = of_reserved_mem_device_init(pdev);
		// if (ret < 0) {
		// 	BSTN_DEV_ERR(
		// 		pdev,
		// 		"of_reserved_mem_device_init fail, ret: %d",
		// 		ret);
		// 	return -ENODEV;
		// }
		// BSTN_TRACE_PRINTK("of_reserved_mem_device_init OK.");
	}
	bstn_dma_contig_set_max_seg_size(pmman->pdev, UINT_MAX);

	INIT_LIST_HEAD(&pmman->mem_ctx_list);
	mutex_init(&pmman->mm_mutex);

	return 0;
}

/*
 * @func    bstn_mem_manager_exit
 * @brief   This is the cleanup function of the memory manager. It frees all
 *          allocated memory blocks including the assigned memory of the DSP.
 * @params  pbstn - the pointer to the BSTN device
 * @return  void
 */
void bstn_mem_manager_exit(struct bstn_device *pbstn)
{
	struct bstn_mem_manager *pmman;
	struct bstn_mem_ctx *ctx;
	struct list_head *tmp, *cur;
	pmman = &pbstn->mem_manager;
	BSTN_TRACE_PRINTK("Exit bstn mem manager.");

#if 0
	if (pbstn->fw_manager.main_os) {
		iommu_detach_group(pbstn->mem_manager.domain,
				   pbstn->mem_manager.group);
		iova_cache_put();
		iommu_group_put(pbstn->mem_manager.group);
	}
#endif

	list_for_each_safe(cur, tmp, &pmman->mem_ctx_list) {
		list_del(cur);
		ctx = container_of(cur, struct bstn_mem_ctx, link);
		// bstn_del_ctx();
		bstn_mem_ctx_del(pbstn, ctx->filp);
	}
	if (!pmman->enable_smmu) {
		of_reserved_mem_device_release(pmman->pdev);
	}
	return;
}
