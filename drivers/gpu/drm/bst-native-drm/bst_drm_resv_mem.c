// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/mman.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/dma-direct.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/hashtable.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>
#include "bst_drm_dev.h"
#include "bst_drm_resv_mem.h"

static struct bst_drm_resv_memblock *bst_drm_cma_alloc(struct bst_dev *mdev, uint32_t size, uint32_t align);
static void bst_drm_cma_free(struct bst_drm_resv_memblock *block);

static struct bst_drm_resv_mem_ops _drm_resv_cma_mem_ops = {
    .alloc = bst_drm_cma_alloc,
    .free = bst_drm_cma_free
};

static struct bst_drm_resv_memblock *bst_drm_cma_alloc(struct bst_dev *mdev, uint32_t size, uint32_t align)
{
    void *vaddr;
    dma_addr_t dma_addr;
    struct bst_drm_resv_memblock *block;

    if (size == 0)
        return NULL;

    size = ALIGN(size, align <= PAGE_SIZE ? PAGE_SIZE : align);
    block = devm_kzalloc(mdev->dev, sizeof(*block), GFP_KERNEL);
    if (block == NULL)
        return NULL;

    vaddr = dma_alloc_coherent(mdev->dev, size, &dma_addr, GFP_KERNEL);
    if (vaddr == NULL) {
        devm_kfree(mdev->dev, block);
        return NULL;
    }

    block->mdev = mdev;
    block->phys_addr = dma_to_phys(mdev->dev, dma_addr);
    block->size = size;
    block->vaddr = vaddr;

    return block;
}

static void bst_drm_cma_free(struct bst_drm_resv_memblock *block)
{
    struct bst_dev* mdev = block->mdev;

    if (block == NULL)
        return;

    dma_free_coherent(mdev->dev, block->size, block->vaddr,
        phys_to_dma(mdev->dev, block->phys_addr));

    devm_kfree(mdev->dev, block);

    return;
}

struct bst_drm_resv_mem_ops *get_resv_mem_ops(void)
{
    return &_drm_resv_cma_mem_ops;
}