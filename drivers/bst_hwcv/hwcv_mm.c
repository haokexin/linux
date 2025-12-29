// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#include <linux/dma-direct.h>
#include <linux/fcntl.h>
#include <linux/iommu.h>
#include <linux/mman.h>
#include "hwcv_dma_buf.h"
#include "hwcv_mm.h"

extern struct device *dev_cvsmm;

/* Prevents memory overflow due to burst access.
 * GWARP's maximum burst size is 192K and scaler is 16K.
 */
#define BURST_MAX_SIZE 192
#define SIZE_2M (0x200000)

/*------------------------------------------------------------------------------------------------------*/

int hwcv_mm_alloc_id(void *ptr)
{
	int new_id;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	mutex_lock(&mm->lock);

	idr_preload(GFP_KERNEL);
	new_id = idr_alloc_cyclic(&mm->memory_idr, ptr, 1, 0, GFP_NOWAIT);
	idr_preload_end();
	if (new_id < 0)
		dev_err(core->dev, "Failed to alloc id: %d\n", new_id);
	else
		mm->buffer_count++;

	mutex_unlock(&mm->lock);

	return new_id;
}

void *hwcv_mm_remove_id(int id)
{
	void *ptr;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	mutex_lock(&mm->lock);

	ptr = idr_remove(&mm->memory_idr, id);
	if (!ptr)
		dev_err(core->dev, "Failed to remove id[%d]\n", id);
	else
		mm->buffer_count--;

	mutex_unlock(&mm->lock);

	return ptr;
}

void *hwcv_mm_lookup_id(int id)
{
	void *ptr;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	mutex_lock(&mm->lock);

	ptr = idr_find(&mm->memory_idr, id);
	if (!ptr)
		dev_err(core->dev, "Failed to lookup id[%d]\n", id);

	mutex_unlock(&mm->lock);

	return ptr;
}

/*------------------------------------------------------------------------------------------------------*/

struct hwcv_buf *hwcv_mm_alloc_buf(u32 size)
{
	void *ret;
	unsigned int aligned_size;
	unsigned long attrs;
	dma_addr_t *dma_addr;
	struct hwcv_buf *buf;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	buf = devm_kzalloc(core->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	aligned_size = ALIGN(size + BURST_MAX_SIZE, SIZE_2M);
	attrs = (mm->force_contiguous) ? DMA_ATTR_FORCE_CONTIGUOUS : 0;
	buf->mem_priv = mm->ops->alloc(mm->dev, attrs, aligned_size,
				       DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (IS_ERR(buf->mem_priv)) {
		ret = buf->mem_priv;
		goto kfree;
	}

	dma_addr = mm->ops->cookie(buf->mem_priv);
	if (!mm->iommud) {
		buf->phys_addr = *dma_addr;
		buf->dma_addr = hwcv_phys_to_dma(buf->phys_addr);
	} else {
		buf->phys_addr =
			(mm->force_contiguous) ?
				iommu_iova_to_phys(mm->iommud, *dma_addr) :
				0;
		buf->dma_addr = *dma_addr;
	}

	if (!buf->dma_addr) {
		dev_err(core->dev, "Invalid dma_addr[0x%llx]\n", buf->dma_addr);
		ret = ERR_PTR(-EINVAL);
		goto put;
	}

	buf->dbuf = mm->ops->get_dmabuf(buf->mem_priv, O_RDWR);
	if (IS_ERR(buf->dbuf)) {
		ret = buf->dbuf;
		goto put;
	}

	buf->fd = dma_buf_fd(buf->dbuf, O_CLOEXEC);
	if (buf->fd < 0) {
		dev_err(core->dev, "Failed to get fd by dmabuf: %d\n", buf->fd);
		ret = ERR_PTR(buf->fd);
		goto put;
	}

	buf->bytesused = size;
	buf->length = aligned_size;
	buf->type = HWCV_INTERNAL_BUF;

	return buf;

put:
	mm->ops->put(buf->mem_priv);
kfree:
	devm_kfree(core->dev, buf);
err:
	return ret;
}

struct hwcv_buf *hwcv_mm_import_buf(int fd, u32 size)
{
	void *ret;
	int error;
	dma_addr_t *dma_addr;
	struct hwcv_buf *buf;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	buf = devm_kzalloc(core->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	buf->fd = fd;
	buf->dbuf = dma_buf_get(fd);
	if (IS_ERR(buf->dbuf)) {
		dev_err(core->dev, "Failed to get dma buf by fd[%d]: %ld\n", fd,
			PTR_ERR(buf->dbuf));
		ret = buf->dbuf;
		goto kfree;
	}

	buf->mem_priv = mm->ops->attach_dmabuf(mm->dev, buf->dbuf, size,
					       DMA_BIDIRECTIONAL);
	if (IS_ERR(buf->mem_priv)) {
		ret = buf->mem_priv;
		goto put_dmabuf;
	}

	error = mm->ops->map_dmabuf(buf->mem_priv);
	if (error) {
		ret = ERR_PTR(error);
		goto detach_dmabuf;
	}

	dma_addr = mm->ops->cookie(buf->mem_priv);
	if (!mm->iommud) {
		buf->phys_addr = *dma_addr;
		buf->dma_addr = hwcv_phys_to_dma(buf->phys_addr);
	} else {
		buf->phys_addr =
			(mm->force_contiguous) ?
				iommu_iova_to_phys(mm->iommud, *dma_addr) :
				0;
		buf->dma_addr = *dma_addr;
	}

	if (!buf->dma_addr) {
		dev_err(core->dev, "Invalid dma_addr[0x%llx]\n", buf->dma_addr);
		ret = ERR_PTR(-EINVAL);
		goto unmap_dmabuf;
	}

	buf->bytesused = size;
	buf->length = buf->dbuf->size;
	buf->type = HWCV_EXTERNAL_BUF;

	return buf;

unmap_dmabuf:
	mm->ops->unmap_dmabuf(buf->mem_priv);
detach_dmabuf:
	mm->ops->detach_dmabuf(buf->mem_priv);
put_dmabuf:
	dma_buf_put(buf->dbuf);
kfree:
	devm_kfree(core->dev, buf);
err:
	return ret;
}

void hwcv_mm_release_buf(struct hwcv_buf *buf)
{
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	if (buf->type == HWCV_INTERNAL_BUF)
		mm->ops->put(buf->mem_priv);
	else {
		mm->ops->unmap_dmabuf(buf->mem_priv);
		mm->ops->detach_dmabuf(buf->mem_priv);
		dma_buf_put(buf->dbuf);
	}

	devm_kfree(core->dev, buf);
}

void hwcv_mm_sync_buf(struct hwcv_buf *buf, u8 sync_dir)
{
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	switch (buf->type) {
	case HWCV_INTERNAL_BUF:
		if (sync_dir == HWCV_SYNC_FOR_DEVICE)
			mm->ops->prepare_own(buf->mem_priv);
		else if (sync_dir == HWCV_SYNC_FOR_CPU)
			mm->ops->finish_own(buf->mem_priv);
		break;
	case HWCV_EXTERNAL_BUF:
		if (sync_dir == HWCV_SYNC_FOR_DEVICE)
			mm->ops->prepare(buf->mem_priv);
		else if (sync_dir == HWCV_SYNC_FOR_CPU)
			mm->ops->finish(buf->mem_priv);
		break;
	default:
		dev_err(core->dev, "Invalid sync dir[%u]\n", sync_dir);
	}
}

/*------------------------------------------------------------------------------------------------------*/

int hwcv_mm_session_release_buffer(struct hwcv_session *session)
{
	int id;
	struct hwcv_buf *buf;
	struct hwcv_mm *mm = hwcv_drvdata->mm;
	struct hwcv_core *core = hwcv_drvdata->core;

	mutex_lock(&mm->lock);

	idr_for_each_entry(&mm->memory_idr, buf, id) {
		if (session == buf->session) {
			dev_dbg(core->dev,
				"process[pid:%d] release buffer id[%d] when exception occurs.\n",
				session->tgid, buf->id);
			idr_remove(&mm->memory_idr, id);
			hwcv_mm_release_buf(buf);
		}
	}

	mutex_unlock(&mm->lock);

	return 0;
}

int hwcv_mm_init(struct hwcv_mm **mm_session)
{
	int ret;
	u64 mask;
	struct device_node *node;
	struct hwcv_core *core = hwcv_drvdata->core;
	struct hwcv_mm *mm = NULL;

	if (!core) {
		pr_err("%s: hwcv core is null\n", __func__);
		return -EFAULT;
	}

	node = dev_of_node(core->dev);
	if (!node) {
		dev_err(core->dev, "device node is null\n");
		return -EFAULT;
	}

	*mm_session = kzalloc(sizeof(struct hwcv_mm), GFP_KERNEL);
	if (*mm_session == NULL)
		return -ENOMEM;

	mm = *mm_session;

	if (of_property_read_bool(node, "use-cvsmm")) {
		dev_info(core->dev, "Use cvsmm\n");
		if (!dev_cvsmm) {
			dev_err(core->dev, "Invalid dev_cvsmm[%p]\n",
				dev_cvsmm);
			return -ENODEV;
		}
		mm->dev = dev_cvsmm;
	} else {
		mm->dev = core->dev;
	}

	mm->force_contiguous = true;
	mm->iommud = iommu_get_domain_for_dev(mm->dev);
	if (mm->iommud)
		dev_info(mm->dev, "Use smmu\n");
	mm->ops = &hwcv_dma_contig_memops;
	mutex_init(&mm->lock);
	idr_init_base(&mm->memory_idr, 1);

	mask = (mm->iommud) ? DMA_BIT_MASK(32) : DMA_BIT_MASK(36);
	ret = dma_set_mask_and_coherent(mm->dev, mask);
	if (ret < 0) {
		dev_err(mm->dev, "Failed to dma set dma mask: %d\n", ret);
		return ret;
	}

	hwcv_dma_contig_set_max_seg_size(mm->dev, UINT_MAX);

	return 0;
}

int hwcv_mm_remove(struct hwcv_mm **mm_session)
{
	int id;
	struct hwcv_buf *buf;
	struct hwcv_mm *mm = *mm_session;

	mutex_lock(&mm->lock);

	idr_for_each_entry(&mm->memory_idr, buf, id)
		hwcv_mm_release_buf(buf);

	idr_destroy(&mm->memory_idr);

	mutex_unlock(&mm->lock);

	kfree(*mm_session);
	*mm_session = NULL;

	return 0;
}
