// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/mman.h>
#include <uapi/asm-generic/mman-common.h>
#include <linux/dma-direct.h>
#include "ipc_mempool.h"
#include "ipc_host_server.h"
#include "ipc.h"
#include "ipc_common.h"




#define IPC_DRIVER_NAME "ipc_mempool"
//#define IPC_BASE_OFFSET_MEM 0x100000
/********************* local variables ***************************/
static DEFINE_MUTEX(memblock_mutex);

#ifdef CONFIG_DMA_DECLARE_COHERENT

struct dma_coherent_mem {
	void		*virt_base;
	dma_addr_t	device_base;
	unsigned long	pfn_base;
	int		size;
	unsigned long	*bitmap;
	spinlock_t	spinlock;
	bool		use_dev_dma_pfn_offset;
};


static inline struct dma_coherent_mem *get_coherent_memory(struct device *dev)
{
	if (dev && dev->dma_mem)
		return dev->dma_mem;
	return NULL;
}

static inline dma_addr_t dma_get_device_base(struct device *dev,
					     struct dma_coherent_mem * mem)
{
	if (mem->use_dev_dma_pfn_offset)
		return phys_to_dma(dev, PFN_PHYS(mem->pfn_base));
	return mem->device_base;
}


 static inline void *bst_dma_alloc_coherent(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp)
{
	unsigned long flags;
	int pageno;
	void *ret;
	int order = get_order(size);
	struct dma_coherent_mem *mem = get_coherent_memory(dev);
	if (!mem)
		return 0;


	spin_lock_irqsave(&mem->spinlock, flags);

	if (unlikely(size > ((dma_addr_t)mem->size << PAGE_SHIFT)))
		goto err;

	pageno = bitmap_find_free_region(mem->bitmap, mem->size, order);
	if (unlikely(pageno < 0))
		goto err;

	/*
	 * Memory was found in the coherent area.
	 */
	*dma_handle = dma_get_device_base(dev, mem) +
			((dma_addr_t)pageno << PAGE_SHIFT);
	ret = mem->virt_base + ((dma_addr_t)pageno << PAGE_SHIFT);
	spin_unlock_irqrestore(&mem->spinlock, flags);

	return ret;
err:
	spin_unlock_irqrestore(&mem->spinlock, flags);
	return NULL;
}
#else 

static inline void *bst_dma_alloc_coherent(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp){
		
		IPC_LOG_ERR("CONFIG_DMA_DECLARE_COHERENT no define !!!!!!!!!!!!!!!!!!!\n");

		return NULL;
}
#endif
 
static int32_t ipc_cma_alloc(struct ipc_mempool *mempool, u32 size, u32 align,
			     struct ipc_memblock **memblock)
{
	struct ipc_memblock *block;
	dma_addr_t dma_addr;
	void *k_addr;
	
	
	size = ALIGN(size, PAGE_SIZE);

	block = devm_kzalloc(&g_ipc_platform_dev->dev,
			     sizeof(struct ipc_memblock), GFP_KERNEL);
	if (!block)
		return -ENOMEM;

	k_addr = bst_dma_alloc_coherent(mempool->dev, size, &dma_addr, GFP_KERNEL);
	if (!k_addr) {
		devm_kfree(&g_ipc_platform_dev->dev, block);
		return -ENOMEM;
	}

	block->pool = mempool;
	block->phy_addr = dma_to_phys(mempool->dev, dma_addr);
	block->size = size;
	block->k_addr = k_addr;
	block->u_addr = 0;
	//memset(block->k_addr+sys_offset, 0, IPC_BASE_OFFSET_MEM);
	
	*memblock = block;

	mutex_lock(&memblock_mutex);
	list_add_tail(&(block->memblock_list), &(mempool->memblock_list_head));
	mutex_unlock(&memblock_mutex);
	IPC_LOG_INFO("alloc phy_addr : 0x%llx", block->phy_addr);

	return 0;
}

static void ipc_cma_free(struct ipc_memblock *memblock)
{
	struct ipc_mempool *pool = memblock->pool;
	struct ipc_memblock *pos;

	dma_free_coherent(pool->dev, memblock->size, memblock->k_addr,
			  phys_to_dma(pool->dev, memblock->phy_addr));

	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos, &(pool->memblock_list_head), memblock_list) {
		if (pos && pos == memblock) {
			list_del(&memblock->memblock_list);
			break;
		}
	}
	mutex_unlock(&memblock_mutex);
	devm_kfree(&g_ipc_platform_dev->dev, memblock);
}

const struct ipc_mempool_ops ipc_cma_mempool_ops = {
	.alloc = ipc_cma_alloc,
	.free = ipc_cma_free,
	.phyaddr = ipc_memblock_phyaddr,
	.kaddr = ipc_memblock_kaddr
};

int32_t ipc_init_cma_mempool(struct ipc_mempool **ppool, struct device *dev)
{
	struct ipc_mempool *cma_pool =
		kmalloc(sizeof(struct ipc_mempool), GFP_KERNEL);

	if (!cma_pool)
		return -ENOMEM;

	cma_pool->dev = dev;
	cma_pool->ops = &ipc_cma_mempool_ops;
	INIT_LIST_HEAD(&(cma_pool->memblock_list_head));
	*ppool = cma_pool;
	return 0;
}

void ipc_destroy_cma_mempool(struct ipc_mempool *pool)
{
	if (pool)
		devm_kfree(&g_ipc_platform_dev->dev, pool);
}

int32_t bst_alloc(struct ipc_buffer *ipc_buffer, bool from_user)
{
	uint64_t uaddr = 0;
	int32_t ret = 0;
	struct ipc_memblock *memblock = NULL;
	struct bstipc *bstipc = platform_get_drvdata(g_ipc_platform_dev);

	ret = bstipc->pool->ops->alloc(bstipc->pool, ipc_buffer->size,
				       ipc_buffer->align, &memblock);
	if (ret) {
		IPC_LOG_ERR("alloc fail, ret = %d\n", ret);
		return ret;
	}

	if (from_user) {
		uaddr = vm_mmap((struct file *)bstipc->private_data, 0,
				ipc_memblock_size(memblock),
				PROT_READ | PROT_WRITE, MAP_SHARED,
				ipc_memblock_phyaddr(memblock));
		if (!uaddr || IS_ERR_VALUE(uaddr)) {
			bstipc->pool->ops->free(memblock);
			IPC_LOG_ERR("vm_mmap fail! ret: %lld\n",
				    uaddr);
			return -EFAULT;
		}
	}
	memblock->u_addr = (void *)uaddr;
	ipc_buffer->uaddr = uaddr;
	ipc_buffer->handle = (uint64_t)memblock;
	ipc_buffer->phy_addr.ptr_64 = memblock->phy_addr;

	IPC_LOG_INFO("uaddr: 0x%llx, handle: 0x%llx, phy_addr: 0x%llx",
		     ipc_buffer->uaddr, ipc_buffer->handle,
		     ipc_buffer->phy_addr.ptr_64);

	return ret;
}

int32_t bst_free(phys_addr_t phy_addr, bool from_user)
{
	struct ipc_memblock *memblock = NULL;
	struct bstipc *bstipc = NULL;
	int32_t ret = 0;
	struct ipc_mempool *pool;
	struct ipc_memblock *pos;

	bstipc = platform_get_drvdata(g_ipc_platform_dev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!\n");
		return -EFAULT;
	}
	pool = bstipc->pool;

	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos, &(pool->memblock_list_head), memblock_list) {
		if (pos && pos->phy_addr == phy_addr) {
			IPC_LOG_INFO("found msg at 0x%llx", phy_addr);
			memblock = pos;
			break;
		}
	}
	mutex_unlock(&memblock_mutex);

	if (memblock == NULL) {
		IPC_LOG_ERR("can not find msg at 0x%llx", phy_addr);
		return -1;
	}

	if (memblock->u_addr) {
		ret = vm_munmap((uint64_t)memblock->u_addr, memblock->size);
		if (ret < 0) {
			IPC_LOG_ERR("vm_munmap fail! ret: %d\n",
				    ret);
			return ret;
		}
	}

	bstipc->pool->ops->free(memblock);

	return ret;
}

void *get_kaddr_from_phy(phys_addr_t phy)
{
	struct bstipc *bstipc = platform_get_drvdata(g_ipc_platform_dev);
	struct ipc_mempool *pool = bstipc->pool;
	struct ipc_memblock *pos;

	mutex_lock(&memblock_mutex);
	list_for_each_entry(pos, &(pool->memblock_list_head), memblock_list) {
		if (pos && pos->phy_addr == phy) {
			IPC_LOG_INFO("found msg at 0x%llx", phy);
			mutex_unlock(&memblock_mutex);
			return pos->k_addr;
		}
	}

	mutex_unlock(&memblock_mutex);
	return NULL;
}
