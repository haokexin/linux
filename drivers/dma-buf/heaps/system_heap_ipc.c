// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF System heap exporter
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2019, 2020 Linaro Ltd.
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 * Portions based off of Andrew Davis' SRAM heap:
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/delay.h>
#include "page_pool.h"
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <uapi/linux/dma-heap.h>

#include "system_heap_ipc.h"

#ifdef CONFIG_BST_C1200_ADAS
#define SERVER_PID CPU_4
#elif defined(CONFIG_BST_C1200_IVI)
#define SERVER_PID CPU_0
#else
#define SERVER_PID CPUMP2_0
#endif

static struct dma_heap *dma_heap_ipc;
static struct dma_heap *dma_heap_ipc_uncached;
static dmabuf_ipc_client_data_t client_data = {};
static volatile dmabuf_ipc_client_t *client = NULL;

static const char* reserved_mem_name[R5MEM_TYPE_MAX] = {"global_normal_dmabuf", "global_secure_dmabuf", "global_hifi_dmabuf"};
static const char* mem_type_id_str[R5MEM_TYPE_MAX] = {"normal", "secure", "hifi"};

/*
 * The selection of the orders used for allocation (1MB, 64K, 4K) is designed
 * to match with the sizes often found in IOMMUs. Using order 4 pages instead
 * of order 0 pages can significantly improve the performance of many IOMMUs
 * by reducing TLB pressure and time spent updating page tables.
 */
static const unsigned int orders[] = {8, 4, 0};
#define NUM_ORDERS ARRAY_SIZE(orders)
//struct dmabuf_page_pool *pools[NUM_ORDERS];
// DECLARE_SEMAPHORE(s_sem);
static struct semaphore s_sem; 

enum dma_ipc_definition
{
	// max number of entries per page while printing buffer usage
	MAX_ENTRY_NUMBER_PER_PRINT = 64,

	// max number of global_dma blocks, 2 * MAX_FD + 1 + (R5MEM_TYPE_MAX - 1)
	MAX_GLOBAL_DMA_BLOCK_NUMBER = (R5MEM_MAX_FD << 1) + R5MEM_TYPE_MAX,
};

#if 0
static void on_dst_changed(bool flag, void *ext)
{
    if (flag) {
        pr_debug("dst is online\n");
    } else {
        pr_debug("dst is offline\n");
    }
    *(bool *)ext = flag;
}
#endif

static int send_2_r5_and_rec(struct ipc_msg* in_msg, struct ipc_msg* out_msg, r5mem_ErrorEnum_t* err)
{
	int32_t msgbox_ret = 0;
	r5mem_driver_ipc_msg_t input = {};
	r5mem_driver_ipc_msg_t output = {};
	uint64_t payload_data[R5MEM_SHARED_BUF_SIZE];
	des_buf_t des_buf;

	memcpy(payload_data, in_msg->payload, sizeof(uint64_t) * R5MEM_SHARED_BUF_SIZE);
	input.payload.data = payload_data;
	input.payload.size = R5MEM_SHARED_BUF_SIZE;
	input.cmd = in_msg->cmd;
	input.mem_type = in_msg->mem_type;

	if (!client)
		return -1;

	down(&s_sem);

	msgbox_ret = client->r5mem_client.send2r5_sync(input, &output, err, 10000, &des_buf);
	if (msgbox_ret != RESULT_SUCCESS || (*err) != R5MEM_NO_ERROR) {
		pr_err("send2r5_sync fail, msgbox_ret: %d\n", msgbox_ret);
		up(&s_sem);
		return -1;
	}
	up(&s_sem);

	out_msg->cmd = output.cmd;
	out_msg->mem_type = output.mem_type;
	memcpy(out_msg->payload, output.payload.data, sizeof(uint64_t) * R5MEM_SHARED_BUF_SIZE);

	return 0;
}


static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_table, table->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sgtable_sg(table, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static int system_heap_attach(struct dma_buf *dmabuf,
			      struct dma_buf_attachment *attachment)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;
	struct sg_table *table;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;

	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	table = dup_sg_table(&buffer->sg_table);
	if (IS_ERR(table)) {
		kfree(a);
		return -ENOMEM;
	}

	a->table = table;
	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);
	a->mapped = false;
	a->uncached = buffer->uncached;
	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	input_msg.cmd = R5MEM_ATTACH;
	input_msg.mem_type = buffer->mem_type;
	input_msg.payload[0] = buffer->global_fd;
	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if (send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
		pr_err("attach 0x%llx fail, err: %d\n", input_msg.payload[0], err);
		return -EBUSY;
	}

	return 0;
}

static void system_heap_detach(struct dma_buf *dmabuf,
			       struct dma_buf_attachment *attachment)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a = attachment->priv;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);

	sg_free_table(a->table);
	kfree(a->table);
	kfree(a);

	input_msg.cmd = R5MEM_DETACH;
	input_msg.mem_type = buffer->mem_type;
	input_msg.payload[0] = buffer->global_fd;
	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR)
		pr_err("detach 0x:%llx fail, err: %d\n", input_msg.payload[0], err);
}

static struct sg_table *system_heap_map_dma_buf(struct dma_buf_attachment *attachment,
						enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;
	struct sg_table *table = a->table;
	int attr = attachment->dma_map_attrs;
	int ret;

	if (a->uncached)
		attr |= DMA_ATTR_SKIP_CPU_SYNC;

	ret = dma_map_sgtable(attachment->dev, table, direction, attr);
	if (ret)
		return ERR_PTR(ret);

	a->mapped = true;
	return table;
}

static void system_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
				      struct sg_table *table,
				      enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;
	int attr = attachment->dma_map_attrs;

	if (a->uncached)
		attr |= DMA_ATTR_SKIP_CPU_SYNC;
	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, attr);
}

static int system_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
						enum dma_data_direction direction)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		invalidate_kernel_vmap_range(buffer->vaddr, buffer->len);

	if (!buffer->uncached) {
		list_for_each_entry(a, &buffer->attachments, list) {
			if (!a->mapped)
				continue;
			dma_sync_sgtable_for_cpu(a->dev, a->table, direction);
		}
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int system_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
					      enum dma_data_direction direction)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt)
		flush_kernel_vmap_range(buffer->vaddr, buffer->len);

	if (!buffer->uncached) {
		list_for_each_entry(a, &buffer->attachments, list) {
			if (!a->mapped)
				continue;
			dma_sync_sgtable_for_device(a->dev, a->table, direction);
		}
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int system_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct sg_table *table = &buffer->sg_table;
	unsigned long addr = vma->vm_start;
	struct sg_page_iter piter;
	int ret;

	if (buffer->uncached)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	for_each_sgtable_page(table, &piter, vma->vm_pgoff) {
		struct page *page = sg_page_iter_page(&piter);

		ret = remap_pfn_range(vma, addr, page_to_pfn(page), PAGE_SIZE,
				      vma->vm_page_prot);
		if (ret)
			return ret;
		addr += PAGE_SIZE;
		if (addr >= vma->vm_end)
			return 0;
	}
	return 0;
}

static void *system_heap_do_vmap(struct system_heap_buffer *buffer)
{
	struct sg_table *table = &buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->len) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;
	struct sg_page_iter piter;
	pgprot_t pgprot = PAGE_KERNEL;
	void *vaddr;

	if (!pages)
		return ERR_PTR(-ENOMEM);

	if (buffer->uncached)
		pgprot = pgprot_writecombine(PAGE_KERNEL);

	for_each_sgtable_page(table, &piter, 0) {
		WARN_ON(tmp - pages >= npages);
		*tmp++ = sg_page_iter_page(&piter);
	}

	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	vfree(pages);

	if (!vaddr)
		return ERR_PTR(-ENOMEM);

	return vaddr;
}

static int system_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		iosys_map_set_vaddr(map, buffer->vaddr);
		goto out;
	}

	vaddr = system_heap_do_vmap(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}

	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
	iosys_map_set_vaddr(map, buffer->vaddr);
out:
	mutex_unlock(&buffer->lock);

	return ret;
}

static void system_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct system_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	iosys_map_clear(map);
}

#if 0
static int system_heap_zero_buffer(struct system_heap_buffer *buffer)
{
	struct sg_table *sgt = &buffer->sg_table;
	struct sg_page_iter piter;
	struct page *p;
	void *vaddr;
	int ret = 0;

	for_each_sgtable_page(sgt, &piter, 0) {
		p = sg_page_iter_page(&piter);
		vaddr = kmap_local_page(p);
		memset(vaddr, 0, PAGE_SIZE);
		kunmap_local(vaddr);
	}

	return ret;
}
#endif

static void system_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, j;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	pr_debug("%s, %d \n",__func__,__LINE__);

	/* Zero the buffer pages before adding back to the pool */
	// system_heap_zero_buffer(buffer);

	table = &buffer->sg_table;
	for_each_sgtable_sg(table, sg, i) {
		struct page *page = sg_page(sg);

		for (j = 0; j < NUM_ORDERS; j++) {
			if (compound_order(page) == orders[j])
				break;
		}
		//dmabuf_page_pool_free(pools[j], page);
	}

	input_msg.cmd = R5MEM_FREE;
	input_msg.mem_type = buffer->mem_type;
	input_msg.payload[0] = buffer->global_fd;
	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR)
		pr_err("free 0x:%llx fail\n", input_msg.payload[0]);

	sg_free_table(table);
	kfree(buffer);
}

static const struct dma_buf_ops system_heap_buf_ops = {
	.attach = system_heap_attach,
	.detach = system_heap_detach,
	.map_dma_buf = system_heap_map_dma_buf,
	.unmap_dma_buf = system_heap_unmap_dma_buf,
	.begin_cpu_access = system_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = system_heap_dma_buf_end_cpu_access,
	.mmap = system_heap_mmap,
	.vmap = system_heap_vmap,
	.vunmap = system_heap_vunmap,
	.release = system_heap_dma_buf_release,
};

#if 0
static struct page *alloc_largest_available(unsigned long size,
					    unsigned int max_order)
{
	struct page *page;
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size <  (PAGE_SIZE << orders[i]))
			continue;
		if (max_order < orders[i])
			continue;
		page = dmabuf_page_pool_alloc(pools[i]);
		if (!page)
			continue;
		return page;
	}
	return NULL;
}
#endif

static r5mem_MemType_t get_mem_type_from_heap_flag(unsigned long heap_flags)
{
	// Default return normal_memory_type.
	if (heap_flags == DMA_HEAP_IPC_SECURE_MEM)
		return R5MEM_TYPE_SECURE;
	else if (heap_flags == DMA_HEAP_IPC_HIFI_MEM)
		return R5MEM_TYPE_HIFI;
	else
		return R5MEM_TYPE_NORMAL;
}

static struct dma_buf *system_heap_do_allocate(struct dma_heap *heap,
					       union ipc_alloc_info* info,
					       unsigned long fd_flags,
					       unsigned long heap_flags,
					       bool uncached,
						   bool is_import_fd)
{
	struct system_heap_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	long long size_remaining= 0 ;
	struct dma_buf *dmabuf;
	struct sg_table *table;
	struct scatterlist *sg;
	// struct list_head pages;
	// struct page *page, *tmp_page;
	// int i, ret = -ENOMEM, fd = 0xFFFFFFFF;
	struct page *page;
	int ret = -ENOMEM, fd = 0xFFFFFFFF;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	dma_addr_t dma_aux = 0;
	int page_counter = 0;

	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->heap = heap;
	if(!is_import_fd)
		buffer->len = info->len;

	buffer->uncached = uncached;

	// INIT_LIST_HEAD(&pages);
	// i = 0;

	if(!is_import_fd) {
		input_msg.cmd = R5MEM_ALLOC_AND_ATTACH;
		input_msg.mem_type = get_mem_type_from_heap_flag(heap_flags);
		input_msg.payload[0] = buffer->len;

		if (input_msg.mem_type == R5MEM_TYPE_MAX) {
			pr_err("[%s]inval heap_flags: %lu, please check\n", __func__, heap_flags);
			goto free_buffer;
		}

		send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
		if (send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
			pr_err("alloc size 0x%x fail, err: %d, send2r5_ret: %d\n",
				(unsigned int)buffer->len, err, send2r5_ret);
			ret = -EBUSY;
			goto free_buffer;
		}
		dma_aux = output_msg.payload[0];           //phy addr
		buffer->global_fd = output_msg.payload[1]; //global fd
		buffer->mem_type  = input_msg.mem_type;    // memory type
	} else {
		/*import global fd, fd has been allocated in others SOC.*/
		input_msg.cmd = R5MEM_GET_PHYS_ADDR_AND_ATTACH;
		input_msg.mem_type = R5MEM_TYPE_MAX;
		input_msg.payload[0] = info->global_fd;
		send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
		if (send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
			pr_err("import global_fd 0x%x fail, err: %d, send2r5_ret: %d\n",
				(unsigned int)info->global_fd, err, send2r5_ret);
			ret = -EBUSY;
			goto free_buffer;
		}
		dma_aux = output_msg.payload[0];         // phy addr
		buffer->len = output_msg.payload[1];     // size
		buffer->global_fd = info->global_fd;     // global fd
		buffer->mem_type = output_msg.mem_type;  // memory type 
	}

	size_remaining = buffer->len;
	buffer->phy_addr = dma_aux;
	pr_debug(".%s, line: %d, R5 alloc addr: 0x%llx, size: 0x%lx,"
		"fd: 0x%lx, is_import_fd: %d, mem_type: %d\n", __func__, __LINE__,
		dma_aux, buffer->len, buffer->global_fd, is_import_fd, buffer->mem_type);
	fd = buffer->global_fd & 0xFFFFFFFF;
	// fd is -1 indicates the R5 allocated fail
	if (fd == 0xFFFFFFFF) {
		pr_err("R5 Can not alloc memory\n");
		goto free_buffer;
	}

	page_counter = (size_remaining + PAGE_SIZE - 1) >> PAGE_SHIFT;
	pr_debug("%s page_counter = %d \n",__func__, page_counter);

	table = &buffer->sg_table;
	if (sg_alloc_table(table, page_counter, GFP_KERNEL)) {
		pr_err("sg_alloc_table failed\n");
		goto free_buffer;
	}

	sg = table->sgl;
	while (size_remaining > 0) {
		/*
		 * Avoid trying to allocate memory if the process
		 * has been killed by SIGKILL
		 */
		if (fatal_signal_pending(current)) {
			ret = -EINTR;
			goto free_pages;
		}

		page = pfn_to_page(PFN_DOWN(dma_aux));
		dma_aux += PAGE_SIZE;

		// list_add_tail(&page->lru, &pages);
		// size_remaining -= page_size(page);

		// i++;

		sg_set_page(sg, page, page_size(page), 0);
		sg = sg_next(sg);
		size_remaining -= PAGE_SIZE;
	}
#if 0
	pr_info("%s i = %d \n",__func__,i);

	table = &buffer->sg_table;
	if (sg_alloc_table(table, i, GFP_KERNEL))
		goto free_buffer;

	sg = table->sgl;
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		sg_set_page(sg, page, page_size(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}
#endif

	/* create the dmabuf */
	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &system_heap_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_pages;
	}

	/*
	 * For uncached buffers, we need to initially flush cpu cache, since
	 * the __GFP_ZERO on the allocation means the zeroing was done by the
	 * cpu and thus it is likely cached. Map (and implicitly flush) and
	 * unmap it now so we don't get corruption later on.
	 */
	if (buffer->uncached) {
		pr_debug("dev:%p, sgl: %p, size:%d \n", dma_heap_get_dev(heap), table->sgl, table->orig_nents);
		// dma_map_sgtable(dma_heap_get_dev(heap), table, DMA_BIDIRECTIONAL, 0);
		// dma_unmap_sgtable(dma_heap_get_dev(heap), table, DMA_BIDIRECTIONAL, 0);
	}

	return dmabuf;

free_pages:
	// for_each_sgtable_sg(table, sg, i) {
	// 	struct page *p = sg_page(sg);
	// 
	// 	__free_pages(p, compound_order(p));
	// }
	sg_free_table(table);

free_buffer:
	// list_for_each_entry_safe(page, tmp_page, &pages, lru)
	// 	__free_pages(page, compound_order(page));
	kfree(buffer);

	return ERR_PTR(ret);
}

static struct dma_buf *system_heap_allocate(struct dma_heap *heap,
					    unsigned long len,
					    unsigned long fd_flags,
					    unsigned long heap_flags)
{
	union ipc_alloc_info  info;
	info.len = len;

	pr_debug("%s,line:%d \n",__func__,__LINE__);
	return system_heap_do_allocate(heap, &info, fd_flags, heap_flags, false,false);
}

static struct dma_buf *system_heap_allocate_uncached(struct dma_heap *heap,
					    unsigned long len,
					    unsigned long fd_flags,
					    unsigned long heap_flags)
{
	union ipc_alloc_info  info;
	info.len = len;

	pr_debug("%s,line:%d \n",__func__,__LINE__);
	return system_heap_do_allocate(heap, &info, fd_flags, heap_flags, true, false);
}

static struct dma_buf *system_heap_allocate_uncached_dummy(struct dma_heap *heap,
					    unsigned long len,
					    unsigned long fd_flags,
					    unsigned long heap_flags)
{
	return ERR_PTR(-EBUSY);
}

static int get_global_fd(struct dma_heap *heap,
			struct dma_buf *dmabuf,
			unsigned long *fd_ptr)
{
	struct system_heap_buffer *buffer = dmabuf->priv;

	pr_debug("%s,line:%d \n",__func__,__LINE__);
	*fd_ptr = buffer->global_fd;
	return 0;
}

static struct dma_buf * import_global_fd(struct dma_heap *heap,
					    unsigned long global_fd,
					    unsigned long fd_flags)
{
	union ipc_alloc_info  info;
	info.global_fd = global_fd;

	pr_debug("%s,line:%d \n",__func__,__LINE__);
	return system_heap_do_allocate(heap, &info, fd_flags, 0, false,true);
}

static struct dma_buf * import_global_fd_uncached(struct dma_heap *heap,
					    unsigned long global_fd,
					    unsigned long fd_flags)
{
	union ipc_alloc_info  info;
	info.global_fd = global_fd;

	pr_debug("%s,line:%d \n",__func__,__LINE__);
	return system_heap_do_allocate(heap, &info, fd_flags, 0, true, true);
}

static int system_get_phy_addrs(struct dma_heap *heap,
				struct dma_buf *dmabuf, union dma_heap_phy_addrs_info *info)
{
	struct system_heap_buffer *buffer = dmabuf->priv;
	info->table_out = &buffer->sg_table;
	info->phyaddr = buffer->phy_addr;
	info->len = buffer->len;
	return IPC_HEAP;
}

static int config_secure_mem(struct dma_heap *heap, __u64 master_id_mask, __u64 access_flag)
{
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	if (access_flag & ~DMA_HEAP_IPC_VALID_ACCESS_FLAGS)
		return -EINVAL;

	memset(&input_msg, 0, sizeof(input_msg));
	memset(&output_msg, 0, sizeof(output_msg));
	input_msg.cmd = R5MEM_CONFIG_SECURE_MEM;
	input_msg.mem_type = R5MEM_TYPE_SECURE;
	input_msg.payload[0] = master_id_mask;
	input_msg.payload[1] = access_flag;

	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
		pr_err("[%s] config secure memory failed,  master_id mask: 0x%llx, flag: 0x%llx\n", 
		           __func__, master_id_mask, access_flag);
		return -1;
	}

	return 0;
}

static const struct dma_heap_ops system_heap_ops = {
	.allocate = system_heap_allocate,
	.get_phy_addrs = system_get_phy_addrs,
	.get_global_fd = get_global_fd,
	.import_global_fd = import_global_fd,
	.config_secure_mem = config_secure_mem,
};

static struct dma_heap_ops system_heap_ops_uncached = {
	.allocate = system_heap_allocate_uncached_dummy,
	.get_phy_addrs = system_get_phy_addrs,
	.get_global_fd = get_global_fd,
	.import_global_fd = import_global_fd_uncached,
	.config_secure_mem = config_secure_mem,
};

static int query_heap_info(struct global_dmabuf_info* dmabuf_info, uint16_t* block_number, struct blocks_stats* blocks)
{
	int i = 0;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	uint64_t v0 = 0;
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	*block_number = 0;
	memset(dmabuf_info, 0, sizeof(struct global_dmabuf_info) * R5MEM_TYPE_MAX);
	memset(blocks, 0, sizeof(struct blocks_stats) * MAX_GLOBAL_DMA_BLOCK_NUMBER);

	for (i = 0; i < R5MEM_TYPE_MAX; ++i) {
		input_msg.cmd = R5MEM_GET_USAGE;
		input_msg.mem_type = (r5mem_MemType_t)i;
		input_msg.payload[0] = 0xFFFFFFFFFFFFFFFF;
		send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
		if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
			pr_err("get heap info fail, err: %d, send2r5_ret: %d\n", err, send2r5_ret);
			goto error_out;
		}

		dmabuf_info[i].total_memory = output_msg.payload[0];
		dmabuf_info[i].used_memory  = output_msg.payload[1];
	}

	memset(&input_msg, 0, sizeof(input_msg));
	input_msg.cmd = R5MEM_GET_USAGE;
	for (i = 0; i < R5MEM_TYPE_MAX; ++i) {
		input_msg.mem_type = (r5mem_MemType_t)i;
		memset(&output_msg, 0, sizeof(output_msg));

		while (output_msg.payload[1] != 0xFFFFFFFFFFFFFFFF) {
			send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
			if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
				pr_err("[%s] get block info fail, mem_type: %u, err: %d, send2r5_ret: %d\n", __func__, (r5mem_MemType_t)i, err, send2r5_ret);
				goto error_out;
			}

			if (output_msg.payload[1] == 0xFFFFFFFFFFFFFFFF) break;

			v0 = output_msg.payload[0];
			blocks[(*block_number)].fd = v0&0xFFF;
			v0 = v0 >> 12;
			blocks[(*block_number)].free = v0&0xF;
			v0 = v0 >> 4;
			blocks[(*block_number)].ref = v0&0xFFFF;
			v0 = v0 >> 16;
			blocks[(*block_number)].size = v0&0xFFFFFFFF;

			blocks[(*block_number)].addr = output_msg.payload[1];
			blocks[(*block_number)].mem_type = output_msg.mem_type;

			(*block_number)++;
			if ((*block_number) >= MAX_GLOBAL_DMA_BLOCK_NUMBER) {
				pr_err("[%s] the number of blocks (%u) exceeds the maximum value (%u)\n", 
				                    __func__, *block_number, MAX_GLOBAL_DMA_BLOCK_NUMBER);
				goto error_out;
			}
		}
	}

	return 0;

error_out:
	*block_number = 0;
	memset(dmabuf_info, 0, sizeof(struct global_dmabuf_info) * R5MEM_TYPE_MAX);
	memset(blocks, 0, sizeof(struct blocks_stats) * MAX_GLOBAL_DMA_BLOCK_NUMBER);

	return -1;
}

static const char* mem_type_id_to_str(r5mem_MemType_t id)
{
    if ((id < 0) || (id >= R5MEM_TYPE_MAX))
        return NULL;
    if (id >= sizeof(mem_type_id_str) / sizeof(char*))
        return NULL;
    
    return mem_type_id_str[id];
}

static ssize_t	heap_info_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	static uint16_t block_number = 0, cur_idx = 0;
	static struct global_dmabuf_info dmabuf_info[R5MEM_TYPE_MAX];
	static struct blocks_stats blocks[MAX_GLOBAL_DMA_BLOCK_NUMBER];

	int len = 0, counter = 0, i = 0;
	int total_print = 0, cur_print = 0;

	if (cur_idx >= block_number) {   //The previous query has been printed, starting a new query.
		block_number = 0;
		cur_idx = 0;
		
		if (query_heap_info(dmabuf_info, &block_number, blocks) < 0)
			return 0;
	}

	len += sysfs_emit_at(buf, len, "IPC heap usage info:\n");
	len += sysfs_emit_at(buf, len, "%-10s  %-10s  %-10s  %-10s\n", "heap_type", "total", "used", "free");
	for (i = 0; i < R5MEM_TYPE_MAX; ++i) {
		len += sysfs_emit_at(buf, len, "%-10s  %010llu  %010llu  %010llu\n", mem_type_id_to_str(i), 
							dmabuf_info[i].total_memory, dmabuf_info[i].used_memory,
							dmabuf_info[i].total_memory - dmabuf_info[i].used_memory);
	}
	len += sysfs_emit_at(buf, len, "\n");

	total_print = (block_number-1)/MAX_ENTRY_NUMBER_PER_PRINT + 1;
	cur_print   = cur_idx / MAX_ENTRY_NUMBER_PER_PRINT + 1;
	len += sysfs_emit_at(buf, len, "IPC heap blocks info(%d/%d):\n", cur_print, total_print);
	len += sysfs_emit_at(buf, len, "%-3s  %-8s  %-11s  %-11s  %-10s  %-5s\n",
			     "gFd", "type", "start", "end", "length", "ref");

	while (counter < MAX_ENTRY_NUMBER_PER_PRINT) {
		if (cur_idx >= block_number)
			break;

        if (blocks[cur_idx].fd >= R5MEM_MAX_FD) {
			len += sysfs_emit_at(buf, len, "%-3s  %-8s  0x%09llx  0x%09llx  %010d  %05d\n",
                                   "---", mem_type_id_to_str(blocks[cur_idx].mem_type), (uint64_t)blocks[cur_idx].addr,
                                   (uint64_t)blocks[cur_idx].addr + blocks[cur_idx].size, blocks[cur_idx].size, blocks[cur_idx].ref);
		}
		else {
            len += sysfs_emit_at(buf, len, "%03d  %-8s  0x%09llx  0x%09llx  %010d  %05d\n",
                                   blocks[cur_idx].fd, mem_type_id_to_str(blocks[cur_idx].mem_type), (uint64_t)blocks[cur_idx].addr,
                                   (uint64_t)blocks[cur_idx].addr + blocks[cur_idx].size, blocks[cur_idx].size, blocks[cur_idx].ref);
		}
		counter++;
		cur_idx++;
	}

	if (cur_idx >= block_number)
		len += sysfs_emit_at(buf, len, "\nAll blocks have been printed\n\n");
	else
		len += sysfs_emit_at(buf, len, "\nSome blocks have not been printed. Please execute it again\n\n");

	return len;
}

static ssize_t	alloc_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "usage: echo size > alloc\n");
}

static ssize_t	alloc_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	const char *ptr = buf;
	char *end = NULL;
	int len = 0, fd = -1;
	uint64_t addr = 0;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	unsigned long sz = 0;

	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	len = strlen(buf);
	sz = simple_strtoul(ptr, &end, 10);

	pr_debug("%s len = %d sz = 0x%lx", __func__,len, sz);

	input_msg.cmd = R5MEM_ALLOC_AND_ATTACH;
	input_msg.mem_type = R5MEM_TYPE_NORMAL;
	input_msg.payload[0] = sz;
	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
		pr_err("alloc size 0x%lx fail, err: %d\n",sz, err);
		return count;
	}
	addr  = output_msg.payload[0]; //phy addr
	fd = output_msg.payload[1];    //global fd
	pr_debug("alloc addr: 0x%llx, global fd: %d", addr, fd);

	return count;
}

static ssize_t	free_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "usage: echo fd > free\n");
}

static ssize_t	free_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	const char *ptr = buf;
	char *end = NULL;
	int len = 0;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	unsigned long fd = 0;
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	len = strlen(buf);
	fd = simple_strtoul(ptr, &end, 10);

	pr_debug("%s len = %d fd = %lu", __func__, len, fd);

	input_msg.cmd = R5MEM_FREE;
	input_msg.mem_type = R5MEM_TYPE_NORMAL;
	input_msg.payload[0] = fd;
	send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
	if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
		pr_err("free %ld fail, err: %d\n", fd, err);
	}

	return count;
}

static struct kobj_attribute alloc_attr =
	__ATTR_RW(alloc);

static struct kobj_attribute free_attr =
	__ATTR_RW(free);

static struct kobj_attribute info_attr =
	__ATTR_RO(heap_info);



static struct attribute *ipc_heap_sysfs_attrs[] = {
	&info_attr.attr,
	&alloc_attr.attr,
	&free_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ipc_heap_sysfs);

static struct kobject *ipc_heap_kobject;

static int ipc_heap_sysfs_setup(void)
{
	int ret;

	ipc_heap_kobject = kobject_create_and_add("ipc_heap", kernel_kobj);
	if (!ipc_heap_kobject)
		return -ENOMEM;

	ret = sysfs_create_groups(ipc_heap_kobject, ipc_heap_sysfs_groups);
	if (ret) {
		kobject_put(ipc_heap_kobject);
		return ret;
	}
	return 0;
}

static int parse_mem_info_from_dts(const char* key, uint64_t* value, size_t max_counter)
{
	struct device_node *global_dma = of_find_node_by_name(NULL, key);

	memset(value, 0, sizeof(uint64_t) * max_counter);
	if (global_dma == NULL) {
		pr_err("fail to find global_dma in device tree\n");
		return -1;
	}
	
	return of_property_read_variable_u64_array(global_dma, "reg", value, 2, max_counter);
}

static void set_global_dma_range(void)
{
	int ret = 0, i = 0, k = 0;
	R5_RET send2r5_ret = R5_SUCCESS;
	r5mem_ErrorEnum_t err = R5MEM_NO_ERROR;
	uint64_t mem_info[R5MEM_MAX_POOL_NUM_PER_MEM_TYPE * 2];
	struct ipc_msg input_msg = {};
	struct ipc_msg output_msg = {};

	for (i = 0; i < R5MEM_TYPE_MAX; ++i) {
		ret = parse_mem_info_from_dts(reserved_mem_name[i], mem_info, sizeof(mem_info)/sizeof(uint64_t));
		if (ret < 0) {
			pr_err("[%s] %s: fail to get memory info from dts, ret: %d\n", __func__, reserved_mem_name[i], ret);
			continue;
		}
		if((ret & 1) != 0)  {
			pr_err("global_dma name: %s, the reg value should be even number\n", reserved_mem_name[i]);
			continue;
		}

		for (k = 0; k < ret; k += 2) {
			if ((mem_info[k] == 0) || (mem_info[k + 1] == 0)) {            // mem_info[k]: base addr, mem_info[k+1]: size
				pr_err("[%s] %s: the memory address or size is zero, skip\n", __func__, reserved_mem_name[i]);
				continue;
			}

			input_msg.cmd = R5MEM_SET_RANGE;
			input_msg.mem_type = (r5mem_MemType_t)i;
			input_msg.payload[0] = mem_info[k];        // base addr;
			input_msg.payload[1] = mem_info[k + 1];    // size;

			send2r5_ret = send_2_r5_and_rec(&input_msg, &output_msg, &err);
			if(send2r5_ret != R5_SUCCESS || err != R5MEM_NO_ERROR) {
				pr_err("[%s] %s: set mem range failed, addr: 0x%llx, size: size 0x%llx, err: %d, send2r5_ret: %d\n",
					__func__, reserved_mem_name[i], input_msg.payload[0], input_msg.payload[1], err, send2r5_ret);
			}
			else {
				pr_info("[%s] %s: set mem range, addr: 0x%llx, size: size 0x%llx\n", 
					__func__, reserved_mem_name[i], input_msg.payload[0], input_msg.payload[1]);
			}
		}
	}
}

static int dmabuf_driver_probe(void)
{
	int ret = 0;
	struct dma_heap_export_info exp_info;

	ipc_inf_version_t version;

	sema_init(&s_sem, 4);

	exp_info.name = "ipc";
	exp_info.ops = &system_heap_ops;
	exp_info.priv = NULL;
	dma_heap_ipc = dma_heap_add(&exp_info);
	if (IS_ERR(dma_heap_ipc)) {
		pr_err("invalid dma heap ipc\n");
		return PTR_ERR(dma_heap_ipc);
	}

	exp_info.name = "ipc-uncached";
	exp_info.ops = &system_heap_ops_uncached;
	exp_info.priv = NULL;

	dma_heap_ipc_uncached = dma_heap_add(&exp_info);
	if (IS_ERR(dma_heap_ipc_uncached)) {
		pr_err("invalid dma heap ipc uncached\n");
		return PTR_ERR(dma_heap_ipc_uncached);
	}

	dma_coerce_mask_and_coherent(dma_heap_get_dev(dma_heap_ipc_uncached), DMA_BIT_MASK(64));
	mb(); /* make sure we only set allocate after dma_mask is set */
	system_heap_ops_uncached.allocate = system_heap_allocate_uncached;

	client_data.com_data.pid = SERVER_PID;
	client = dmabuf_ipc_client_init(&client_data);
	if (!client) {
		pr_err("%s: init client fail.\n", __func__);
		return -1;
	}

	// get version
	version = client->r5mem_client.version();
	pr_debug("%s,Interface version: major %d, minor %d.\n", __func__, version.major, version.minor);

	ret = client->start();
	if (ret < 0) {
		pr_err("start dmabuf ipc client fail.!\n");
		return ret;
	}

	// volatile bool dst_avail = false;
	// client->r5mem_client.register_avail_changed(on_dst_changed, &dst_avail);
	// while (!dst_avail)
	// 	msleep(1000);

	set_global_dma_range();
	ipc_heap_sysfs_setup();
	return 0;
}

static void dmabuf_driver_remove(void)
{
	int ret = 0;
	if(client)
		client->stop();
	ret = dmabuf_ipc_client_destroy();
	if (ret < 0)
		pr_err("destory client fail.!\n");
	client = NULL;
	dma_heap_put(dma_heap_ipc);
	dma_heap_put(dma_heap_ipc_uncached);

	kobject_put(ipc_heap_kobject);
}

static int __init dma_buf_ipc_init(void)
{
	return dmabuf_driver_probe();
}

static void __exit dma_buf_ipc_exit(void)
{
	dmabuf_driver_remove();
}

late_initcall(dma_buf_ipc_init);
module_exit(dma_buf_ipc_exit);

MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(DMA_BUF);
