// SPDX-License-Identifier: GPL-2.0+
/*
 * bst_lwnn_dma_manager.c - DMA contig memory allocator for bst_lwnn
 * videobuf2-dma-contig.c - DMA contig memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 * 
 * Author: Pawel Osciak <pawel@osciak.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include "bst_lwnn_dma_manager.h"

static void bst_lwnn_common_vm_open(struct vm_area_struct *vma)
{
	struct bst_lwnn_vmarea_handler *h = vma->vm_private_data;

	pr_debug("%s: %p, refcount: %d, vma: %08lx-%08lx\n", __func__, h,
		 refcount_read(h->refcount), vma->vm_start, vma->vm_end);

	refcount_inc(h->refcount);
}

static void bst_lwnn_common_vm_close(struct vm_area_struct *vma)
{
	struct bst_lwnn_vmarea_handler *h = vma->vm_private_data;

	pr_debug("%s: %p, refcount: %d, vma: %08lx-%08lx\n", __func__, h,
		 refcount_read(h->refcount), vma->vm_start, vma->vm_end);

	h->put(h->arg);
}

const struct vm_operations_struct bst_lwnn_common_vm_ops = {
	.open = bst_lwnn_common_vm_open,
	.close = bst_lwnn_common_vm_close,
};

/**********************************************************************
*
************************************************************************/
static unsigned long bst_lwnn_dma_get_contiguous_size(struct sg_table *sgt)
{
	struct scatterlist *s;
	dma_addr_t expected;
	unsigned int i;
	unsigned long size;

	size = 0;
	expected = sg_dma_address(sgt->sgl);

	for_each_sgtable_dma_sg(sgt, s, i) {
		if (sg_dma_address(s) != expected)
			break;
		expected += sg_dma_len(s);
		size += sg_dma_len(s);
	}
	return size;
}

static void *bst_lwnn_dma_cookie(void *buf_priv)
{
	return (void *)(((struct bst_lwnn_memblock *)(buf_priv))->dma_addr);
}

static void *bst_lwnn_dma_vaddr(void *buf_priv)
{
	struct bst_lwnn_memblock *buf = buf_priv;

	if (buf->vaddr)
		return buf->vaddr;

	if (buf->db_attach) {
		struct iosys_map map = { 0 };

		if (!dma_buf_vmap(buf->db_attach->dmabuf, &map))
			buf->vaddr = map.vaddr;

		return buf->vaddr;
	}
	return buf->vaddr;
}

static unsigned int bst_lwnn_dma_num_users(void *buf_priv)
{
	return refcount_read(
		&(((struct bst_lwnn_memblock *)(buf_priv))->refcount));
}

static void bst_lwnn_dma_prepare(void *buf_priv)
{
	struct bst_lwnn_memblock *buf;
	struct sg_table *sgt = NULL;

	buf = (struct bst_lwnn_memblock *)buf_priv;
	sgt = buf->sgt_base;

	if (IS_ERR_OR_NULL(sgt)) {
		pr_err("%s: invalid sg_table", __func__);
		return;
	}

	dma_sync_sgtable_for_device(buf->dev, sgt, DMA_TO_DEVICE);
	pr_debug("%s: dma_addr[0x%llx], dma_len[%u], dma_dir[%d]", __func__,
		 sg_dma_address(sgt->sgl), sg_dma_len(sgt->sgl), buf->dma_dir);
}

static void bst_lwnn_dma_finish(void *buf_priv)
{
	struct bst_lwnn_memblock *buf;
	struct sg_table *sgt = NULL;

	buf = (struct bst_lwnn_memblock *)buf_priv;
	sgt = buf->sgt_base;

	if (IS_ERR_OR_NULL(sgt)) {
		pr_err("%s: invalid sg_table", __func__);
		return;
	}

	dma_sync_sgtable_for_cpu(buf->dev, sgt, DMA_FROM_DEVICE);
	pr_debug("%s: dma_addr[0x%llx], dma_len[%u], dma_dir[%d]", __func__,
		 sg_dma_address(sgt->sgl), sg_dma_len(sgt->sgl), buf->dma_dir);
}

/**********************************************************************
*               callbacks for MMAP buffers
************************************************************************/
static void bst_lwnn_dma_put(void *buf_priv)
{
	struct bst_lwnn_memblock *buf = buf_priv;

	pr_debug("%s: ref[%d]", __func__, refcount_read(&buf->refcount));
	if (!refcount_dec_and_test(&buf->refcount))
		return;

	pr_debug("%s: dma_addr[0x%llx], size[%lu], ref[%d]", __func__,
		 buf->dma_addr, buf->size, refcount_read(&buf->refcount));
	if (buf->sgt_base) {
		// dma_unmap_sgtable(buf->dev, buf->sgt_base, buf->dma_dir, DMA_ATTR_SKIP_CPU_SYNC);
		sg_free_table(buf->sgt_base);
		kfree(buf->sgt_base);
	}
	dma_free_attrs(buf->dev, buf->size, buf->cookie, buf->dma_addr,
		       buf->attrs);
	put_device(buf->dev);
	kfree(buf);
}

static void *bst_lwnn_dma_alloc(struct device *dev, unsigned long attrs,
				unsigned long size,
				enum dma_data_direction dma_dir,
				gfp_t gfp_flags)
{
	struct bst_lwnn_memblock *buf;

	if (WARN_ON(!dev))
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->attrs = attrs;
	buf->cookie = dma_alloc_attrs(dev, size, &buf->dma_addr,
				      GFP_KERNEL | gfp_flags, buf->attrs);
	if (!buf->cookie) {
		pr_err("Failed to dma alloc, size = %lu", size);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	if ((buf->attrs & DMA_ATTR_NO_KERNEL_MAPPING) == 0)
		buf->vaddr = buf->cookie;

	/* Prevent the device from being released while the buffer is used */
	buf->dev = get_device(dev);
	buf->size = size;
	buf->dma_dir = dma_dir;

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = bst_lwnn_dma_put;
	buf->handler.arg = buf;

	refcount_set(&buf->refcount, 1);
	pr_debug("%s: dma_addr[0x%llx], size[%lu], ref[%d]", __func__,
		 buf->dma_addr, buf->size, refcount_read(&buf->refcount));

	return buf;
}

static int bst_lwnn_dma_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct bst_lwnn_memblock *buf = buf_priv;
	struct sg_table *table = NULL;
	struct sg_page_iter piter;
	unsigned long addr = vma->vm_start;
	int ret;

	if (!buf) {
		printk(KERN_ERR "No buffer to map\n");
		return -EINVAL;
	}
	table = buf->sgt_base;

	for_each_sgtable_page(table, &piter, vma->vm_pgoff) {
		struct page *page = sg_page_iter_page(&piter);
		ret = remap_pfn_range(vma, addr, page_to_pfn(page), PAGE_SIZE,
				      vma->vm_page_prot);

		if (ret)
			return ret;
		addr += PAGE_SIZE;
		if (addr >= vma->vm_end)
			break;
	}

	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_private_data = &buf->handler;
	vma->vm_ops = &bst_lwnn_common_vm_ops;

	vma->vm_ops->open(vma);

	pr_debug("%s: dma_addr[0x%llx], size[%lu], user_vaddr[0x%lx], ref[%d]",
		 __func__, buf->dma_addr, buf->size, vma->vm_start,
		 refcount_read(&buf->refcount));

	return 0;
}

/*********************************************************************
 *                     DMABUF ops for exporters
 *********************************************************************/
struct bst_lwnn_dma_attachment {
	struct sg_table sgt;
	enum dma_data_direction dma_dir;
};

static int
bst_lwnn_dma_dmabuf_ops_attach(struct dma_buf *dbuf,
			       struct dma_buf_attachment *dbuf_attach)
{
	struct bst_lwnn_dma_attachment *attach;
	unsigned int i;
	struct scatterlist *rd, *wr;
	struct sg_table *sgt;
	struct bst_lwnn_memblock *buf;
	int ret;
	buf = (struct bst_lwnn_memblock *)dbuf->priv;
	attach = kzalloc(sizeof(*attach), GFP_KERNEL);
	if (!attach)
		return -ENOMEM;

	sgt = &attach->sgt;
	/* Copy the buf->base_sgt scatter list to the attachment, as we can't
	 * map the same scatter list to multiple attachments at the same time.
	 */
	ret = sg_alloc_table(sgt, buf->sgt_base->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(attach);
		return -ENOMEM;
	}

	rd = buf->sgt_base->sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	attach->dma_dir = DMA_NONE;
	dbuf_attach->priv = attach;

	pr_debug("%s: %s attach", __func__, dev_name(dbuf_attach->dev));

	return 0;
}

static void bst_lwnn_dma_dmabuf_ops_detach(struct dma_buf *dbuf,
					   struct dma_buf_attachment *db_attach)
{
	struct bst_lwnn_dma_attachment *attach;
	struct bst_lwnn_memblock *buf;
	struct sg_table *sgt;

	attach = db_attach->priv;
	buf = dbuf->priv;
	pr_debug("%s: %s detach", __func__, dev_name(db_attach->dev));

	if (!attach)
		return;

	sgt = &attach->sgt;

	/* release the scatterlist cache */
	if (attach->dma_dir != DMA_NONE)
		/*
		 * Cache sync can be skipped here, as the bst_lwnn_dc memory is
		 * allocated from device coherent memory, which means the
		 * memory locations do not require any explicit cache
		 * maintenance prior or after being used by the device.
		 */
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(attach);
	db_attach->priv = NULL;
}

static struct sg_table *
bst_lwnn_dma_dmabuf_ops_map(struct dma_buf_attachment *db_attach,
			    enum dma_data_direction dma_dir)
{
	struct bst_lwnn_dma_attachment *attach = db_attach->priv;
	/* stealing dmabuf mutex to serialize map/unmap operations */
	struct mutex *lock = &db_attach->dmabuf->lock;
	struct sg_table *sgt;

	mutex_lock(lock);

	sgt = &attach->sgt;
	/* return previously mapped sg table */
	if (attach->dma_dir == dma_dir) {
		mutex_unlock(lock);
		return sgt;
	}

	/* release any previous cache */
	if (attach->dma_dir != DMA_NONE) {
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
		attach->dma_dir = DMA_NONE;
	}

	/*
	 * mapping to the client with new direction, no cache sync
	 * required see comment in bst_lwnn_dma_dmabuf_ops_detach()
	 */
	if (dma_map_sgtable(db_attach->dev, sgt, dma_dir,
			    DMA_ATTR_SKIP_CPU_SYNC)) {
		pr_err("Failed to map scatterlist");
		mutex_unlock(lock);
		return ERR_PTR(-EIO);
	}

	attach->dma_dir = dma_dir;

	mutex_unlock(lock);

	pr_debug("%s: %s map attach", __func__, dev_name(db_attach->dev));

	return sgt;
}

static void bst_lwnn_dma_dmabuf_ops_unmap(struct dma_buf_attachment *db_attach,
					  struct sg_table *sgt,
					  enum dma_data_direction dma_dir)
{
	pr_debug("%s: %s unmap attach", __func__, dev_name(db_attach->dev));
}

static void bst_lwnn_dma_dmabuf_ops_release(struct dma_buf *dbuf)
{
	pr_debug("%s", __func__);
	/* drop reference obtained in bst_lwnn_dma_get_dmabuf */
	bst_lwnn_dma_put(dbuf->priv);
}

static int
bst_lwnn_dma_dmabuf_ops_begin_cpu_access(struct dma_buf *dbuf,
					 enum dma_data_direction direction)
{
	return 0;
}

static int
bst_lwnn_dma_dmabuf_ops_end_cpu_access(struct dma_buf *dbuf,
				       enum dma_data_direction direction)
{
	return 0;
}

static int bst_lwnn_dma_dmabuf_ops_vmap(struct dma_buf *dbuf,
					struct iosys_map *map)
{
	struct bst_lwnn_memblock *buf = dbuf->priv;
	void *vaddr;

	vaddr = bst_lwnn_dma_vaddr(buf);
	if (!vaddr)
		return -EINVAL;

	iosys_map_set_vaddr(map, vaddr);

	return 0;
}

static int bst_lwnn_dma_dmabuf_ops_mmap(struct dma_buf *dbuf,
					struct vm_area_struct *vma)
{
	return bst_lwnn_dma_mmap(dbuf->priv, vma);
}

static const struct dma_buf_ops bst_lwnn_dma_dmabuf_ops = {
	.attach = bst_lwnn_dma_dmabuf_ops_attach,
	.detach = bst_lwnn_dma_dmabuf_ops_detach,
	.map_dma_buf = bst_lwnn_dma_dmabuf_ops_map,
	.unmap_dma_buf = bst_lwnn_dma_dmabuf_ops_unmap,
	.begin_cpu_access = bst_lwnn_dma_dmabuf_ops_begin_cpu_access,
	.end_cpu_access = bst_lwnn_dma_dmabuf_ops_end_cpu_access,
	.vmap = bst_lwnn_dma_dmabuf_ops_vmap,
	.mmap = bst_lwnn_dma_dmabuf_ops_mmap,
	.release = bst_lwnn_dma_dmabuf_ops_release,
};

static struct sg_table *bst_lwnn_dma_get_base_sgt(struct bst_lwnn_memblock *buf)
{
	int ret;
	struct sg_table *sgt;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		pr_err("Failed to alloc sg table");
		return NULL;
	}

	ret = dma_get_sgtable_attrs(buf->dev, sgt, buf->cookie, buf->dma_addr,
				    buf->size, buf->attrs);
	if (ret < 0) {
		pr_err("Failed to get scatterlist from DMA API");
		kfree(sgt);
		return NULL;
	}

	return sgt;
}

static struct dma_buf *bst_lwnn_dma_get_dmabuf(void *buf_priv,
					       unsigned long flags)
{
	// int ret;
	struct bst_lwnn_memblock *buf = buf_priv;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	dma_addr_t dma_addr = buf->dma_addr;
	struct sg_table *sgt = NULL;
	struct scatterlist *sgl = NULL;
	int i = 0;

	exp_info.ops = &bst_lwnn_dma_dmabuf_ops;
	exp_info.size = buf->size;
	exp_info.flags = flags;
	exp_info.priv = buf;

	if (!buf->sgt_base)
		buf->sgt_base = bst_lwnn_dma_get_base_sgt(buf);

	if (WARN_ON(!buf->sgt_base)) {
		pr_err("Failed to get sgt");
		return ERR_PTR(-EINVAL);
	}

	/* dma_map_sgtable() will alloc new iova. One dma buffer wastes two iova 
		virtual spaces. Here, don't need to map again. */
#if 1
	sgt = buf->sgt_base;
	for_each_sgtable_sg(sgt, sgl, i) {
		sg_dma_address(sgl) = dma_addr;
		sg_dma_len(sgl) = sgl->length;
		dma_addr += sgl->length;
	}
#else
	ret = dma_map_sgtable(buf->dev, buf->sgt_base,buf->dma_dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret < 0) {
		dev_err(buf->dev, "Failed to map sgt: %d\n", ret);
		return ERR_PTR(ret);
	}
#endif

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		pr_err("Failed to export dmabuf");
		return dbuf;
	}

	/* dmabuf keeps reference to buffer */
	refcount_inc(&buf->refcount);

	pr_debug("%s: dma_addr[0x%llx], size[%lu], ref[%d]", __func__,
		 buf->dma_addr, buf->size, refcount_read(&buf->refcount));

	return dbuf;
}

/*********************************************/
/*       callbacks for DMABUF buffers        */
/*********************************************/

static int bst_lwnn_dma_map_dmabuf(void *mem_priv)
{
	struct bst_lwnn_memblock *buf = mem_priv;
	struct sg_table *sgt;
	unsigned long contig_size;

	if (WARN_ON(!buf->db_attach)) {
		pr_err("Trying to pin a non attached buffer");
		return -EINVAL;
	}

	if (WARN_ON(buf->dma_sgt)) {
		pr_err("Dmabuf buffer is already pinned");
		return 0;
	}
	pr_debug("buf->db_attach: 0x%llx, buf->dma_dir: %d",
		 (unsigned long long)buf->db_attach, buf->dma_dir);
	/* get the associated scatterlist for this buffer */
	sgt = dma_buf_map_attachment(buf->db_attach, buf->dma_dir);
	if (IS_ERR(sgt)) {
		pr_err("Failed to get dmabuf scatterlist");
		return -EINVAL;
	}
	pr_debug("map attachment done");

	/* checking if dmabuf is big enough to store contiguous chunk */
	contig_size = bst_lwnn_dma_get_contiguous_size(sgt);
	if (contig_size < buf->size) {
		pr_err("Contiguous chunk is too small %lu/%lu\n", contig_size,
		       buf->size);
		dma_buf_unmap_attachment(buf->db_attach, sgt, buf->dma_dir);
		return -EFAULT;
	}
	pr_debug("get contig size done");
	buf->dma_addr = sg_dma_address(sgt->sgl);
	buf->dma_sgt = sgt;
	buf->vaddr = NULL;

	pr_debug("%s: dma_addr[0x%llx], size[%lu]", __func__, buf->dma_addr,
		 buf->size);

	return 0;
}

static void bst_lwnn_dma_unmap_dmabuf(void *mem_priv)
{
	struct bst_lwnn_memblock *buf = mem_priv;
	struct sg_table *sgt = buf->dma_sgt;

	pr_debug("%s: dma_addr[0x%llx], size[%lu]", __func__, buf->dma_addr,
		 buf->size);

	if (WARN_ON(!buf->db_attach)) {
		pr_err("Trying to unpin a not attached buffer");
		return;
	}

	if (WARN_ON(!sgt)) {
		pr_err("Dmabuf buffer is already unpinned");
		return;
	}

	if (buf->vaddr) {
		dma_buf_vunmap(buf->db_attach->dmabuf, buf->vaddr);
		buf->vaddr = NULL;
	}
	dma_buf_unmap_attachment(buf->db_attach, sgt, buf->dma_dir);

	buf->dma_addr = 0;
	buf->dma_sgt = NULL;
}

static void bst_lwnn_dma_detach_dmabuf(void *mem_priv)
{
	struct bst_lwnn_memblock *buf = mem_priv;

	pr_debug("%s", __func__);

	/* if works correctly you should never detach mapped buffer */
	if (WARN_ON(buf->dma_addr))
		bst_lwnn_dma_unmap_dmabuf(buf);

	/* detach this attachment */
	dma_buf_detach(buf->db_attach->dmabuf, buf->db_attach);
	kfree(buf);
}

static void *bst_lwnn_dma_attach_dmabuf(struct device *dev,
					struct dma_buf *dbuf,
					unsigned long size,
					enum dma_data_direction dma_dir)
{
	struct bst_lwnn_memblock *buf;
	struct dma_buf_attachment *dba;

	// if (dbuf->size < size) {
	// 	pr_err(
	// 		"%s, Attach size[%lu] is bigger than dma buf size[%lu]",
	// 		__func__, size, dbuf->size
	// 	);
	// 	return ERR_PTR(-EFAULT);
	// }
	size = dbuf->size;

	if (WARN_ON(!dev))
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->dev = dev;
	/* create attachment for the dmabuf with the user device */
	dba = dma_buf_attach(dbuf, buf->dev);
	if (IS_ERR(dba)) {
		pr_err("Failed to attach dmabuf");
		kfree(buf);
		return dba;
	}

	buf->dma_dir = dma_dir;
	buf->size = size;
	buf->db_attach = dba;

	pr_debug("%s: dma_dir[%d]", __func__, buf->dma_dir);

	return buf;
}

/*********************************************/
/*       DMA CONTIG exported functions       */
/*********************************************/

const struct bst_lwnn_dma_ops bst_lwnn_dma_memops = {
	.alloc = bst_lwnn_dma_alloc,
	.put = bst_lwnn_dma_put,
	.get_dmabuf = bst_lwnn_dma_get_dmabuf,
	.cookie = bst_lwnn_dma_cookie,
	.vaddr = bst_lwnn_dma_vaddr,
	.mmap = bst_lwnn_dma_mmap,
	.prepare = bst_lwnn_dma_prepare,
	.finish = bst_lwnn_dma_finish,
	.map_dmabuf = bst_lwnn_dma_map_dmabuf,
	.unmap_dmabuf = bst_lwnn_dma_unmap_dmabuf,
	.attach_dmabuf = bst_lwnn_dma_attach_dmabuf,
	.detach_dmabuf = bst_lwnn_dma_detach_dmabuf,
	.num_users = bst_lwnn_dma_num_users,
};
EXPORT_SYMBOL_GPL(bst_lwnn_dma_memops);

/**
 * bst_lwnn_dma_contig_set_max_seg_size() - configure DMA max segment size
 * @dev:	device for configuring DMA parameters
 * @size:	size of DMA max segment size to set
 *
 * To allow mapping the scatter-list into a single chunk in the DMA
 * address space, the device is required to have the DMA max segment
 * size parameter set to a value larger than the buffer size. Otherwise,
 * the DMA-mapping subsystem will split the mapping into max segment
 * size chunks. This function sets the DMA max segment size
 * parameter to let DMA-mapping map a buffer as a single chunk in DMA
 * address space.
 * This code assumes that the DMA-mapping subsystem will merge all
 * scatterlist segments if this is really possible (for example when
 * an IOMMU is available and enabled).
 * Ideally, this parameter should be set by the generic bus code, but it
 * is left with the default 64KiB value due to historical litmiations in
 * other subsystems (like limited USB host drivers) and there no good
 * place to set it to the proper value.
 * This function should be called from the drivers, which are known to
 * operate on platforms with IOMMU and provide access to shared buffers
 * (either USERPTR or DMABUF). This should be done before initializing
 * videobuf2 queue.
 */
int bst_lwnn_dma_contig_set_max_seg_size(struct device *dev, unsigned int size)
{
	if (!dev->dma_parms) {
		dev_err(dev, "Failed to set max_seg_size: dma_parms is NULL\n");
		return -ENODEV;
	}
	if (dma_get_max_seg_size(dev) < size)
		return dma_set_max_seg_size(dev, size);

	return 0;
}

EXPORT_SYMBOL_GPL(bst_lwnn_dma_contig_set_max_seg_size);
