/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP
 * author: AI Tools Team, BST Ltd.
 *
 * @file    bst_cv_mem_manager.c
 * @brief   This file is the source code file of the memory manager of the
 *          bst_cv driver. It contains definitions of actual memory allocation
 *          functions as well as the initialization and exit functions of the
 *          memory manager.
 * @note    Because the current buffer and memory structures have not been
 *          finalized yet, further implementation for reliability like garbage
 *          collection is not completed.
 */

#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include "bst_cv.h"
#include "bst_cv_mem_manager.h"
#include "bst_cv_dma_manager.h"

extern struct device *dev_cvsmm;

typedef enum {
	DMA_BUFF_ALLOC,
	DMA_BUFF_IMPORT,
} bst_cv_buf_type_e;

/******************************************************************************
* bst_cv mem ops: alloc, iommu_bypass, free...
*******************************************************************************/
static bst_cv_memblock_t *bst_cv_alloc (struct   bst_cv     *pbst_cv,
											     uint32_t    size,
												 uint32_t    align,
										unsigned long        attr);

static void           bst_cv_iommu_map (struct   bst_cv     *pbst_cv,
												 uint32_t    size,
												 uint32_t    align,
												 phys_addr_t addr,
												 dsp_ptr     iova,
										unsigned long        attr);

static void        bst_cv_iommu_bypass (struct bst_cv       *pbst_cv,
												uint32_t     size,
												uint32_t     align,
												phys_addr_t  addr,
										unsigned long        attr);

static void          bst_cv_iommu_free  (struct bst_cv      *pbst_cv,
												uint32_t     size,
												uint32_t     align,
												dsp_ptr      iova);

static void bst_cv_free(bst_cv_memblock_t *block);

static struct bst_cv_mem_ops bst_cv_cma_memops = {
	.alloc 			= bst_cv_alloc,
	.iommu_bypass	= bst_cv_iommu_bypass,
	.iommu_map		= bst_cv_iommu_map,
	.iommu_free		= bst_cv_iommu_free,
	.free			= bst_cv_free,
};

/*
 * @func    bst_cv_alloc
 * @brief   This function allocates a requested continuous memory block.
 * @params  mem_manager - the pointer to the memory mem_manager
 *          size - the requested size
 * @return  the pointer to the memory block - success
 *          NULL - failure
 */
static bst_cv_memblock_t *bst_cv_alloc(struct bst_cv *pbst_cv,
										  uint32_t     size,
										  uint32_t     align,
								   unsigned long       attr)
{
	struct bst_cv_mem_manager *pmman;
	struct bst_cv_memblock    *block;
	struct device           *pdev;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;
	block = NULL;
	if (0 == size) {
		goto exit;
	}
	size  = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	block = pmman->dma_ops->alloc(pmman->pdev,
								  attr,
								  size,
								  DMA_BIDIRECTIONAL,
								  GFP_KERNEL);
	block->pbst_cv = (void *)pbst_cv;

exit:
	if (NULL == block) {
		BST_CV_DEV_ERR(pdev, "bst_cv_alloc fail.");
	} else {
		BST_CV_TRACE_PRINTK("bst_cv_alloc ok");
	}
	return block;
}


static void bst_cv_iommu_map   (struct bst_cv *pbst_cv,
									 uint32_t     size,
									 uint32_t     align,
									 phys_addr_t  addr,
									 dsp_ptr      iova,
							  unsigned long       attr)
{
	struct bst_cv_mem_manager *pmman;
	struct iommu_domain     *domain;
	struct device           *pdev;
	struct iova_domain *iovad;
	dma_addr_t new_iova;
	unsigned long shift;

	pmman  = &pbst_cv->mem_manager;
	pdev   = &pbst_cv->pdev->dev;
	size   = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BST_CV_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}
	iovad = (struct iova_domain *)((void *)domain->iova_cookie + sizeof(uint64_t));
	shift = iova_shift(iovad);
	iovad->start_pfn = iova >> shift;
	BST_CV_TRACE_PRINTK(
		"dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
		(unsigned long long)pmman->pdev,
		(unsigned long long)domain,
		(unsigned long long)iovad,
		(unsigned long)shift
	);
	new_iova  = alloc_iova_fast(
					iovad,
					size >> shift,
					(iova + size) >> shift,
					true
				);
	new_iova <<= shift;
	if(iommu_map(domain, new_iova, addr, size, attr)) {
		BST_CV_DEV_ERR(pdev, "iommu_map fail.");
		return;
	}
	iovad->start_pfn = 0x80000000 >> shift; // iova must be bigger than 0x80000000
	BST_CV_TRACE_PRINTK(
		"bst_cv_iommu_map ok. iova: 0x%08x size: 0x%08x",
		(dsp_ptr)new_iova,
		size
	);
}

static void bst_cv_iommu_bypass(struct bst_cv *pbst_cv,
									 uint32_t     size,
									 uint32_t     align,
									 phys_addr_t  addr,
							  unsigned long       attr)
{
	struct bst_cv_mem_manager *pmman;
	struct iommu_domain     *domain;
	struct device           *pdev;
	struct iova_domain *iovad;
	dma_addr_t iova;
	unsigned long shift;

	pmman  = &pbst_cv->mem_manager;
	pdev   = &pbst_cv->pdev->dev;
	size   = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	iova   = phys_to_bus(addr);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BST_CV_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}
	iovad = (struct iova_domain *)((void *)domain->iova_cookie + sizeof(uint64_t));
	shift = iova_shift(iovad);
	iovad->start_pfn = iova >> shift;
	BST_CV_TRACE_PRINTK(
		"dev 0x%llx domain 0x%llx iovad 0x%llx, shift 0x%lx",
		(unsigned long long)pmman->pdev,
		(unsigned long long)domain,
		(unsigned long long)iovad,
		(unsigned long)shift
	);
	iova  = alloc_iova_fast(
					iovad,
					size >> shift,
					(iova + size) >> shift,
					true
				);
	iova <<= shift;
	if(iommu_map(domain, iova, addr, size, attr)) {
		BST_CV_DEV_ERR(pdev, "iommu_map fail.");
		return;
	}
	iovad->start_pfn = 0x80000000 >> shift; // iova must be bigger than 0x80000000
	BST_CV_TRACE_PRINTK(
		"bst_cv_iommu_bypass ok. iova: 0x%08x size: 0x%08x",
		(dsp_ptr)iova,
		size
	);
}

static void bst_cv_iommu_free(struct bst_cv *pbst_cv,
									   uint32_t  size,
									   uint32_t  align,
									   dsp_ptr   iova)
{
	struct bst_cv_mem_manager *pmman;
	struct iommu_domain     *domain;
	struct device           *pdev;
	struct iova_domain *iovad;
	unsigned long shift;

	pmman  = &pbst_cv->mem_manager;
	pdev   = &pbst_cv->pdev->dev;
	size   = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
	domain = iommu_get_domain_for_dev(pmman->pdev);
	if (!domain) {
		BST_CV_DEV_ERR(pdev, "iommu_get_domain_for_dev fail.");
		return;
	}
	iovad = (struct iova_domain *)((void *)domain->iova_cookie + sizeof(uint64_t));
	shift = iova_shift(iovad);
	iommu_unmap(domain, (unsigned long)iova, size);
	free_iova_fast(iovad, (unsigned long)iova >> shift, (unsigned long)size >> shift);

	BST_CV_STAGE_PRINTK(
		"bst_cv_iommu_free ok. iova: 0x%08x size: 0x%08x",
		(dsp_ptr)iova,
		size
	);
}

/*
 * @func    bst_cv_cma_free
 * @brief   This function frees the target continuous memory block.
 * @params  block - the pointer to the memory block
 * @return  void
 */
static void bst_cv_free(bst_cv_memblock_t *block)
{
	struct bst_cv      *pbst_cv;
	struct bst_cv_mem_manager *pmman;
	pbst_cv = (struct bst_cv *)block->pbst_cv;
	pmman = &pbst_cv->mem_manager;
	if (NULL == block)
		return;
	pmman->dma_ops->put(block);
	return;
}






/***********************************************************************
* bst_cv mem ctx manger: add, delete, find ...
************************************************************************/
static struct bst_cv_mem_ctx *_find_mem_ctx(struct bst_cv *pbst_cv,
											struct file   *filp)
{
	struct bst_cv_mem_ctx *ctx;
	struct list_head    *cur;
	struct bst_cv_mem_manager *pmman;

	pmman = &pbst_cv->mem_manager;
	list_for_each(cur, &pmman->mem_ctx_list) {
		ctx = container_of(cur, struct bst_cv_mem_ctx, link);
		if (ctx->filp == filp) {
			return ctx;
		}
	}
	return NULL;
}

int bst_cv_mem_ctx_add(struct bst_cv *pbst_cv, struct file *filp) {
	struct bst_cv_mem_ctx     *ctx, *search;
	struct device           *pdev;
	struct bst_cv_mem_manager *pmman;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;

	ctx = devm_kzalloc(pdev, sizeof(*ctx), GFP_KERNEL);
	if (NULL == ctx) {
		BST_CV_DEV_ERR(pdev, "kmalloc failed");
		return -ENOMEM;
	}
	ctx->filp = filp;
	hash_init(ctx->ht);

	mutex_lock(&pmman->mm_mutex);
	search = _find_mem_ctx(pbst_cv, filp);
	if (NULL != search) {
		mutex_unlock(&pmman->mm_mutex);
		return -EINVAL;
	}

	list_add(&(ctx->link), &pmman->mem_ctx_list);
	mutex_unlock(&pmman->mm_mutex);
	BST_CV_TRACE_PRINTK("ctx %px added for filp %px", ctx, filp);
	return 0;
}

int bst_cv_mem_ctx_del(struct bst_cv *pbst_cv, struct file *filp)
{
	int i;
	struct bst_cv_mem_ctx     *ctx;
	struct bst_cv_mem_manager *pmman;
	struct bst_cv_buffer      *buffer;
	struct hlist_node       *tmp;
	struct device           *pdev;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;


	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_cv, filp);
	if (NULL == ctx) {
		mutex_unlock(&pmman->mm_mutex);
		BST_CV_DEV_ERR(pdev,
			"fatal error: memory context with filp=%px not found", filp);
		return -EFAULT;
	}

	list_del(&ctx->link);

	hash_for_each_safe(ctx->ht, i, tmp, buffer, node) {
		hash_del(&buffer->node);
		BST_CV_TRACE_PRINTK("buffer %px", buffer);
		pmman->ops->free(buffer->block);
		devm_kfree(pdev, buffer);
	}
	devm_kfree(&pbst_cv->pdev->dev, ctx);
	mutex_unlock(&pmman->mm_mutex);
	BST_CV_TRACE_PRINTK("ctx %px removed for filp %px", ctx, filp);
	return 0;
}








/*
 * @func    bst_cv_user_buffer_alloc
 * @brief   This function allocates a requested memory chunk to be used as a
 *          user buffer.
 * @params  filp - the misc device file pointer
 *          pbst_cv - the pointer to the bst_cv device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_user_buffer_alloc(struct file *filp,
							 struct bst_cv *pbst_cv,
							 struct xrp_ioctl_alloc *ualloc)
{
	int    ret;
	struct bst_cv_buffer      *buffer;
	struct bst_cv_mem_ctx     *ctx;
	struct bst_cv_mem_manager *pmman;
	struct device           *pdev;
	struct file             *filp_mmap;
	// struct vm_area_struct   *vma;
	ret   = -ENOENT;
	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BST_CV_DEV_ERR(pdev, "kmalloc failed!");
		ret = -ENOMEM;
		goto fail_before_struct_alloc;
	}
	//allocate the buffer
	buffer->block = pmman->ops->alloc(pbst_cv, ualloc->size, ualloc->align, DMA_ATTR_PRIVILEGED);
	if (buffer->block == NULL) {
		BST_CV_DEV_ERR(pdev, "alloc failed!");
		ret = -ENOMEM;
		goto fail_before_buffer_alloc;
	}
	if (pmman->enable_smmu) {
		buffer->bus_addr = (dsp_ptr)addr_truncate(pmman->dma_ops->cookie(buffer->block));
	} else {
		/*
		buffer->bus_addr = phys_to_bus(
			dma_to_phys(
				pmman->dma_ops->cookie(buffer->block)
			)
		);
		*/
		buffer->bus_addr = (dsp_ptr)phys_to_bus(pmman->dma_ops->cookie(buffer->block));
	}

	buffer->dbuf = pmman->dma_ops->get_dmabuf(buffer->block, O_RDWR);
	if (IS_ERR(buffer->dbuf)) {
		BST_CV_DEV_ERR(pdev, "Failed to get dmabuf");
		ret = PTR_ERR(buffer->dbuf);
		goto fail_after_buffer_alloc;
	}

	buffer->fd = dma_buf_fd(buffer->dbuf, O_CLOEXEC);
	if (buffer->fd < 0) {
		BST_CV_DEV_ERR(pdev, "Failed to get fd by dmabuf");
		ret = buffer->fd;
		goto fail_after_buffer_alloc;
	}
	buffer->type   = DMA_BUFF_ALLOC;

	if (pmman->enable_smmu) {
		filp_mmap = fget(buffer->fd);
		if (!filp_mmap) {
			BST_CV_DEV_ERR(pdev, "filp_mmap get null.");
		}
		buffer->user_addr = vm_mmap(
			filp_mmap,
			0 /* buffer->block->dma_addr */,
			buffer->block->size,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			0
		);
		fput(filp_mmap);
	} else {
		buffer->user_addr = vm_mmap(
			filp,
			0,
			buffer->block->size,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			buffer->block->dma_addr
		);
	}
	BST_CV_TRACE_PRINTK("user_addr: 0x%llx", (unsigned long long)buffer->user_addr);
	ualloc->addr = buffer->bus_addr;
	ualloc->ptr  = buffer->user_addr;
	ualloc->align = ualloc->align > PAGE_SIZE ? ualloc->align : PAGE_SIZE;
	BST_CV_TRACE_PRINTK("addr: %x, size: %x", ualloc->addr, ualloc->size);
	BST_CV_TRACE_PRINTK("buffer: %px", buffer);

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_cv, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_CV_DEV_ERR(pdev,
			"memory context with filp=%px not found", filp);
		goto fail_buffer_mmap;
	}
	//add the buffer into the hash table
	hash_add(ctx->ht, &buffer->node, buffer->bus_addr);
	mutex_unlock(&pmman->mm_mutex);
	return 0;

fail_buffer_mmap:
	BST_CV_DEV_ERR(pdev, "mmap fail");
	// vm_area_free(vma);
fail_after_buffer_alloc:
	BST_CV_DEV_ERR(pdev, "after buffer alloc fail");
	pmman->ops->free(buffer->block);
fail_before_buffer_alloc:
	BST_CV_DEV_ERR(pdev, "before buffer alloc fail");
	devm_kfree(pdev, buffer);
fail_before_struct_alloc:
	BST_CV_DEV_ERR(pdev, "before struct alloc fail");
	return ret;
}

/*
 * @func    bst_cv_user_buffer_free
 * @brief   This function frees a user buffer.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          alloc - the allocation information
 * @return  0 - success
 *          error code - failure
 */
int close_fd(unsigned fd);
int bst_cv_user_buffer_free(struct file *filp,
							struct bst_cv *pbst_cv,
							struct xrp_ioctl_alloc *ualloc)
{
	struct bst_cv_buffer      *buffer;
	struct bst_cv_mem_ctx     *ctx;
	struct hlist_node       *tmp;
	struct bst_cv_mem_manager *pmman;
	struct device           *pdev;
	// struct vm_area_struct   *vma;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;

	mutex_lock(&pmman->mm_mutex);
	ctx = _find_mem_ctx(pbst_cv, filp);
	if (ctx == NULL) {
		mutex_unlock(&pmman->mm_mutex);
		BST_CV_DEV_ERR(pdev, "memory context with filp=%px not found", filp);
		return -ENOENT;
	}

	hash_for_each_possible_safe(ctx->ht, buffer, tmp, node, ualloc->addr) {
		if (ualloc->addr == buffer->bus_addr) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->mm_mutex);
			BST_CV_TRACE_PRINTK("buffer: %px", buffer);

			vm_munmap((unsigned long)buffer->user_addr, buffer->block->size);
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
	BST_CV_DEV_ERR(pdev, "no buffer @ baddr=0x%x", ualloc->addr);
	return -ENOENT;
}










/*!
 * @brief           This function imports a dma-buf
 * @param[in]       pbst_cv The pointer to the bst_cv_device
 * @param[in,out]   buf The imported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
int bst_cv_dma_buf_import(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf)
{
	struct bst_cv_buffer      *buffer;
	struct bst_cv_memblock    *block;
	struct dma_buf          *dmabuf = NULL;
	struct bst_cv_mem_manager *pmman;
	struct device           *pdev;
	int ret = 0;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;

	buffer = devm_kzalloc(pdev, sizeof(*buffer), GFP_KERNEL);
	if (NULL == buffer) {
		BST_CV_DEV_ERR(pdev, "fail to allocate bst_cv buff.");
		ret = -ENOMEM;
		goto dma_buf_import_fail_at_allocate_bst_cv;
	}


	BST_CV_TRACE_PRINTK("get fd %d", buf->fd);
	dmabuf = dma_buf_get(buf->fd);
	buffer->dbuf = dmabuf;
	if (IS_ERR(dmabuf)) {
		BST_CV_DEV_ERR(pdev, "failed to get dma_buf(fd=%d)", buf->fd);
		ret = PTR_ERR(dmabuf);
		goto dma_buf_import_fail_at_dma_buf_get;
	}

	block = pmman->dma_ops->attach_dmabuf(pmman->pdev,
										  dmabuf,
										  dmabuf->size,
										  DMA_BIDIRECTIONAL);
	buffer->block = block;
	if (IS_ERR(block)) {
		BST_CV_DEV_ERR(pmman->pdev, "failed to attach dma_buf(fd=%d)", buf->fd);
		ret = PTR_ERR(block);
		goto dma_buf_import_fail_at_dma_buf_attach;
	}
	BST_CV_TRACE_PRINTK("attach dmabuf done");

	/* get the associated scatterlist for this buffer */
	ret = pmman->dma_ops->map_dmabuf(block);
	if (ret) {
		goto dma_buf_import_fail_at_dma_buf_map_attachment;
	}
	BST_CV_TRACE_PRINTK("map dmabuf done");

	if (pmman->enable_smmu) {
		buffer->bus_addr = (dsp_ptr)addr_truncate(pmman->dma_ops->cookie(block));
	} else {
		buffer->bus_addr = (dsp_ptr)phys_to_bus(pmman->dma_ops->cookie(block));
	}
	if (!buffer->bus_addr) {
		ret = -EINVAL;
		goto dma_buf_import_fial_at_dma_buf_unmap;
	}
	buffer->type = DMA_BUFF_IMPORT;
	buffer->fd   = buf->fd;
	buf->bus_addr = buffer->bus_addr;

	mutex_lock(&pmman->dma_buf_mutex);
	hash_add(pmman->dma_buf_ht, &buffer->node, (size_t)dmabuf); // ? why using dmabuf
	mutex_unlock(&pmman->dma_buf_mutex);

	return ret;

dma_buf_import_fial_at_dma_buf_unmap:
	pmman->dma_ops->unmap_dmabuf(block);
dma_buf_import_fail_at_dma_buf_map_attachment:
	pmman->dma_ops->detach_dmabuf(block);
dma_buf_import_fail_at_dma_buf_attach:
	dma_buf_put(buffer->dbuf);
dma_buf_import_fail_at_dma_buf_get:
	devm_kfree(pdev, buffer);
dma_buf_import_fail_at_allocate_bst_cv:

	return ret;
}

/*!
 * @brief       This function imports a dma-buf
 * @param[in]   pbst_cv The pointer to the bst_cv_device
 * @param[in]   buf The returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
int bst_cv_dma_buf_return(struct bst_cv *pbst_cv, struct bst_cv_dma_buf *buf)
{
	struct dma_buf          *dmabuf;
	struct bst_cv_buffer      *buffer;
	struct hlist_node       *tmp;
	struct bst_cv_mem_manager *pmman;
	struct device           *pdev;

	pmman = &pbst_cv->mem_manager;
	pdev  = &pbst_cv->pdev->dev;

	dmabuf = dma_buf_get(buf->fd);
	if (IS_ERR(dmabuf)) {
		BST_CV_DEV_ERR(pdev, "failed to get dma_buf(fd=%d)", buf->fd);
		return PTR_ERR(dmabuf);
	}
	BST_CV_TRACE_PRINTK("returned dmabuf bus addr %px", dmabuf->priv);

	mutex_lock(&pmman->dma_buf_mutex);
	hash_for_each_possible_safe(pmman->dma_buf_ht, buffer, tmp,
								node, (size_t) dmabuf) {
		if (buffer->dbuf == dmabuf) {
			hash_del(&buffer->node);
			mutex_unlock(&pmman->dma_buf_mutex);
			BST_CV_TRACE_PRINTK("buffer: %px", buffer);
			pmman->dma_ops->unmap_dmabuf(buffer->block);
			pmman->dma_ops->detach_dmabuf(buffer->block);
			dma_buf_put(dmabuf);
			dma_buf_put(buffer->dbuf);
			devm_kfree(pdev, buffer);
			return 0;
		}
	}
	mutex_unlock(&pmman->dma_buf_mutex);
	dma_buf_put(dmabuf);

	BST_CV_DEV_ERR(pdev, "buffer for dma_buf(fd=%d) not found", buf->fd);
	return -ENOENT;
}






/*
 * @func    bst_cv_mem_manager_init
 * @brief   This is the initialization function of the memory manager. It sets
 *          up the reseved memory and DMA configs of the device.
 * @params  pbst_cv - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_mem_manager_init(struct bst_cv *pbst_cv)
{
	int ret;
	struct device             *pdev;
	struct bst_cv_mem_manager *pmman;

	/* tips: In some drvs, pdev may diff with pmman->pdev */
	pdev        = &pbst_cv->pdev->dev;
	pmman       = &pbst_cv->mem_manager;
	if (pmman->enable_smmu && dev_cvsmm != NULL) {
		pmman->pdev = dev_cvsmm;
		BST_CV_STAGE_PRINTK("memory manager using dev_cvsmm...");
	} else {
		pmman->pdev = pdev;
		BST_CV_STAGE_PRINTK("memory manager using dev_cv   ...");
	};

	pmman->dma_ops = &bst_cv_dma_memops;
	pmman->ops     = &bst_cv_cma_memops;

	if (pmman->enable_smmu) {
		// set dma mask and coherent mask
		ret  = dma_set_mask_and_coherent(pmman->pdev,       DMA_BIT_MASK(32));
		if (ret) {
			BST_CV_DEV_ERR(pmman->pdev, "dma_set_coherent_mask fail, ret %d", ret);
			return -ENODEV;
		}
		BST_CV_TRACE_PRINTK("dma_set_coherent_mask OK.");
	} else {
		// set dma mask and coherent mask
		ret  = dma_set_mask_and_coherent(pmman->pdev,       DMA_BIT_MASK(36));
		if (ret) {
			BST_CV_DEV_ERR(pmman->pdev, "dma_set_coherent_mask fail, ret %d", ret);
			return -ENODEV;
		}
		BST_CV_TRACE_PRINTK("dma_set_coherent_mask OK.");

		// init reserved memory
		ret = of_reserved_mem_device_init(pdev);
		if (ret < 0) {
			BST_CV_DEV_ERR(pdev, "of_reserved_mem_device_init fail, ret: %d", ret);
			return -ENODEV;
		}
		BST_CV_TRACE_PRINTK("of_reserved_mem_device_init OK.");
	}
	bst_cv_dma_contig_set_max_seg_size(pmman->pdev, UINT_MAX);

	INIT_LIST_HEAD(&pmman->mem_ctx_list);
	mutex_init(&pmman->mm_mutex);
	mutex_init(&pmman->dma_buf_mutex);
	return 0;
}

/*
 * @func    bst_cv_mem_manager_exit
 * @brief   This is the cleanup function of the memory manager. It frees all
 *          allocated memory blocks including the assigned memory of the DSP.
 * @params  pbst_cv - the pointer to the BSTN device
 * @return  void
 */
void bst_cv_mem_manager_exit(struct bst_cv *pbst_cv)
{
	struct bst_cv_mem_manager *pmman;
	struct bst_cv_mem_ctx     *ctx;
	struct list_head  *tmp, *cur;
	pmman = &pbst_cv->mem_manager;
	BST_CV_TRACE_PRINTK("Exit bst_cv mem manager.");
	list_for_each_safe(cur, tmp, &pmman->mem_ctx_list) {
		list_del(cur);
		ctx = container_of(cur, struct bst_cv_mem_ctx, link);
		// bst_cv_del_ctx();
		bst_cv_mem_ctx_del(pbst_cv, ctx->filp);
	}
	if (!pmman->enable_smmu) {
		of_reserved_mem_device_release(pmman->pdev);
	}
	return;
}
