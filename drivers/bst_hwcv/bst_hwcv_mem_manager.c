// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/fcntl.h>
#include <linux/dma-direct.h>
#include <linux/mman.h>
#include <linux/of_reserved_mem.h>
#include "bst_hwcv_mem_manager.h"
extern struct device *dev_cvsmm;

static dma_addr_t _convert_to_iova(dma_addr_t addr)
{
	dma_addr_t iova;

	if (addr < 0xc40000000 && addr >= 0xc00000000)
		iova = addr - 0xb40000000;
	else if (addr < 0x840000000 && addr >= 0x800000000)
		iova = addr - 0x780000000;
	else
		iova = 0;

	return iova;
}

struct bst_hwcv_mem_ctx *_find_ctx_by_filp(struct file *filp,
					   struct bst_hwcv_mem_manager *mman)
{
	int is_find = 0;
	struct bst_hwcv_mem_ctx *ctx;

	list_for_each_entry(ctx, &mman->mem_ctx_list, node) {
		if (ctx->filp == filp) {
			is_find = 1;
			break;
		}
	}

	return (is_find) ? ctx : NULL;
}

struct bst_hwcv_buf *find_buf_by_iova(struct file *filp,
				      struct bst_hwcv_mem_manager *mman,
				      dma_addr_t iova)
{
	int i = 0;
	int is_find = 0;
	struct bst_hwcv_mem_ctx *ctx;
	struct bst_hwcv_buf *buf;
	struct hlist_node *tmp;

	ctx = _find_ctx_by_filp(filp, mman);
	if (!ctx) {
		dev_err(mman->dev, "Failed to find ctx by filp[0x%p]", filp);
		return NULL;
	}

	hash_for_each_safe(ctx->mm_ht, i, tmp, buf, node) {
		if (iova == buf->iova) {
			is_find = 1;
			break;
		}
	}

	return (is_find) ? buf : NULL;
}

/*------------------------------------------------------------------------------------------------------*/
struct bst_hwcv_buf *bst_hwcv_buf_alloc(struct bst_hwcv_mem_manager *mman,
					unsigned int size)
{
	void *ret;
	unsigned int aligned_size;
	dma_addr_t *dma_addr;
	struct bst_hwcv_buf *buf;
	struct device *dev = mman->dev;

	buf = devm_kzalloc(dev, sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	aligned_size = PAGE_ALIGN(size);
	buf->mem_priv = mman->ops->alloc(dev, DMA_ATTR_FORCE_CONTIGUOUS, aligned_size,
					 DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (IS_ERR(buf->mem_priv)) {
		ret = buf->mem_priv;
		goto kfree;
	}

	dma_addr = mman->ops->cookie(buf->mem_priv);
	if (!mman->use_smmu)
		buf->iova = _convert_to_iova(*dma_addr);
	else
		buf->iova = *dma_addr;
	if (!buf->iova) {
		dev_err(dev, "Invalid iova 0x%llx", buf->iova);
		ret = ERR_PTR(-EINVAL);
		goto put;
	}

	buf->dbuf = mman->ops->get_dmabuf(buf->mem_priv, O_RDWR);
	if (IS_ERR(buf->dbuf)) {
		ret = buf->dbuf;
		goto put;
	}

	buf->fd = dma_buf_fd(buf->dbuf, O_CLOEXEC);
	if (buf->fd < 0) {
		dev_err(dev, "Failed to get fd by dmabuf");
		ret = ERR_PTR(buf->fd);
		goto put;
	}

	buf->bytesused = size;
	buf->length = aligned_size;
	buf->type = MEM;
	dev_dbg(dev,
		"%s: iova[0x%llx], byteused[%u], length[%u], fd[%u], type[%u]",
		__func__, buf->iova, buf->bytesused, buf->length, buf->fd,
		buf->type);

	return buf;

put:
	mman->ops->put(buf->mem_priv);
kfree:
	devm_kfree(dev, buf);
err:
	return ret;
}

void bst_hwcv_buf_free(struct bst_hwcv_mem_manager *mman,
		       struct bst_hwcv_buf *buf)
{
	struct device *dev = mman->dev;

	mman->ops->put(buf->mem_priv);
	devm_kfree(dev, buf);
}

/*------------------------------------------------------------------------------------------------------*/
struct bst_hwcv_buf *bst_hwcv_buf_import(struct bst_hwcv_mem_manager *mman,
					 int fd, unsigned int size)
{
	void *ret;
	int error;
	dma_addr_t *dma_addr;
	struct bst_hwcv_buf *buf;
	struct device *dev = mman->dev;

	buf = devm_kzalloc(dev, sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	buf->dbuf = dma_buf_get(fd);
	if (IS_ERR(buf->dbuf)) {
		dev_err(dev, "Failed to get dma buf by fd[%d]", fd);
		ret = buf->dbuf;
		goto kfree;
	}

	buf->mem_priv = mman->ops->attach_dmabuf(dev, buf->dbuf, size,
						 DMA_BIDIRECTIONAL);
	if (IS_ERR(buf->mem_priv)) {
		ret = buf->mem_priv;
		goto put_dmabuf;
	}

	error = mman->ops->map_dmabuf(buf->mem_priv);
	if (error) {
		ret = ERR_PTR(error);
		goto detach_dmabuf;
	}

	dma_addr = mman->ops->cookie(buf->mem_priv);
	if (!mman->use_smmu)
		buf->iova = _convert_to_iova(*dma_addr);
	else
		buf->iova = *dma_addr;
	if (!buf->iova) {
		dev_err(dev, "Invalid iova 0x%llx", buf->iova);
		ret = ERR_PTR(-EINVAL);
		goto unmap_dmabuf;
	}

	buf->bytesused = size;
	buf->length = buf->dbuf->size;
	buf->type = DMA;

	return buf;

unmap_dmabuf:
	mman->ops->unmap_dmabuf(buf->mem_priv);
detach_dmabuf:
	mman->ops->detach_dmabuf(buf->mem_priv);
put_dmabuf:
	dma_buf_put(buf->dbuf);
kfree:
	devm_kfree(dev, buf);
err:
	return ret;
}

void bst_hwcv_buf_return(struct bst_hwcv_mem_manager *mman,
			 struct bst_hwcv_buf *buf)
{
	struct device *dev = mman->dev;

	mman->ops->unmap_dmabuf(buf->mem_priv);
	mman->ops->detach_dmabuf(buf->mem_priv);
	dma_buf_put(buf->dbuf);
	devm_kfree(dev, buf);
}

/*------------------------------------------------------------------------------------------------------*/

int bst_hwcv_add_buf_to_ctx(struct file *filp,
			    struct bst_hwcv_mem_manager *mman,
			    struct bst_hwcv_buf *buf)
{
	struct bst_hwcv_mem_ctx *ctx;
	struct device *dev = mman->dev;

	ctx = _find_ctx_by_filp(filp, mman);
	if (!ctx) {
		dev_err(dev, "Failed to find ctx by filp[0x%p]", filp);
		return -EINVAL;
	}

	hash_add(ctx->mm_ht, &buf->node, buf->iova);
	dev_dbg(dev, "Add mem[0x%llx:%d] to ctx[%d]", buf->iova, buf->bytesused,
		ctx->id);

	return 0;
}

struct bst_hwcv_buf *
bst_hwcv_del_buf_from_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman,
			  dma_addr_t iova)
{
	int is_find = 0;
	struct bst_hwcv_mem_ctx *ctx;
	struct bst_hwcv_buf *buf;
	struct device *dev = mman->dev;

	ctx = _find_ctx_by_filp(filp, mman);
	if (!ctx) {
		dev_err(dev, "Failed to find ctx by filp[0x%p]", filp);
		return ERR_PTR(-EINVAL);
	}

	hash_for_each_possible(ctx->mm_ht, buf, node, iova) {
		if (buf->iova == iova) {
			is_find = 1;
			break;
		}
	}

	if (is_find) {
		hash_del(&buf->node);
		dev_dbg(dev, "Del mem[0x%llx:%d] from ctx[%d]", buf->iova,
			buf->bytesused, ctx->id);
	}

	return (is_find) ? buf : ERR_PTR(-EINVAL);
}

int bst_hwcv_add_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman)
{
	struct bst_hwcv_mem_ctx *ctx, *search;
	struct device *dev = mman->dev;

	search = _find_ctx_by_filp(filp, mman);
	if (search) {
		dev_info(dev, "Memory ctx of filp[0x%p] already exist", filp);
		return 0;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mman->mem_ctx_id++;
	ctx->id = mman->mem_ctx_id;
	ctx->filp = filp;
	hash_init(ctx->mm_ht);

	list_add(&ctx->node, &mman->mem_ctx_list);
	dev_dbg(dev, "Add Memory ctx[%d]", ctx->id);

	return 0;
}

int bst_hwcv_del_ctx(struct file *filp, struct bst_hwcv_mem_manager *mman)
{
	int i;
	struct bst_hwcv_mem_ctx *ctx;
	struct bst_hwcv_buf *buf;
	struct hlist_node *tmp;
	struct device *dev = mman->dev;

	ctx = _find_ctx_by_filp(filp, mman);
	if (!ctx) {
		dev_err(dev, "Failed to find ctx by filp[0x%p]", filp);
		return -EINVAL;
	}

	dev_dbg(dev, "Del Memory ctx[%d]", ctx->id);
	list_del(&ctx->node);
	hash_for_each_safe(ctx->mm_ht, i, tmp, buf, node) {
		hash_del(&buf->node);
		if (buf->type == MEM)
			bst_hwcv_buf_free(mman, buf);
		else
			bst_hwcv_buf_return(mman, buf);
	}
	devm_kfree(dev, ctx);

	return 0;
}

/*------------------------------------------------------------------------------------------------------*/

int bst_hwcv_mem_manager_init(struct device *dev,
			      struct bst_hwcv_mem_manager *mman)
{
	int ret;
	u64 mask;
	struct device *mm_dev;
	struct device_node *node;

	dev_info(dev, "Init hwcv mem manager.");
	node = dev_of_node(dev);
	if (of_property_read_bool(node, "use-cvsmm")) {
		dev_info(dev, "Use cvsmm");
		if (!dev_cvsmm) {
			dev_err(dev, "cvsmm is not exit");
			return -ENODEV;
		}
		mm_dev = dev_cvsmm;
	} else {
		mm_dev = dev;
	}

	mman->ops = &hwcv_dma_contig_memops;
	mman->dev = mm_dev;
	if (mm_dev->dma_ops) {
		dev_info(mm_dev, "Use smmu");
		mman->use_smmu = true;
	}

	mask = (mman->use_smmu) ? DMA_BIT_MASK(32) : DMA_BIT_MASK(36);
	ret = dma_set_mask_and_coherent(mm_dev, mask);
	if (ret < 0) {
		dev_err(mm_dev, "Failed to dma set dma mask");
		return ret;
	}

	if (!mman->use_smmu) {
		ret = of_reserved_mem_device_init(mm_dev);
		if (ret < 0) {
			dev_err(mm_dev,
				"Failed to assign reserved memory region to hwcv");
			return ret;
		}
	}

	hwcv_dma_contig_set_max_seg_size(mm_dev, UINT_MAX);

	mutex_init(&mman->lock);
	INIT_LIST_HEAD(&mman->mem_ctx_list);

	return 0;
}

void bst_hwcv_mem_manager_exit(struct bst_hwcv_mem_manager *mman)
{
	struct bst_hwcv_mem_ctx *ctx;
	struct list_head *tmp, *cur;
	struct device *dev = mman->dev;

	dev_info(dev, "Exit hwcv mem manager.");

	list_for_each_safe(cur, tmp, &mman->mem_ctx_list) {
		list_del(cur);
		ctx = container_of(cur, struct bst_hwcv_mem_ctx, node);
		bst_hwcv_del_ctx(ctx->filp, mman);
	}

	of_reserved_mem_device_release(dev);
}
