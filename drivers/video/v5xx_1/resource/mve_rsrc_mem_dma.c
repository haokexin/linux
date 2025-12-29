/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

#ifdef EMULATOR
#include "emulator_userspace.h"
#else
#include <linux/types.h>
#include <linux/export.h>
#include <linux/kernel.h>
#endif

#include "mve_blk1_rsrc_mem_dma.h"
#include "mve_blk1_rsrc_mem_dma_uncached.h"

/**
 * Function pointers to a DMA memory implementation
 */
typedef struct mve_rsrc_dma_mem_t *(*alloc_func_t)(uint32_t size);
typedef void (*free_func_t)(struct mve_rsrc_dma_mem_t *mem);
typedef void (*clean_cache_func_t)(struct mve_rsrc_dma_mem_t *mem);
typedef void (*invalidate_cache_func_t)(struct mve_rsrc_dma_mem_t *mem);
typedef void *(*map_func_t)(struct mve_rsrc_dma_mem_t *mem);
typedef void (*unmap_func_t)(struct mve_rsrc_dma_mem_t *mem);
typedef phys_addr_t *(*get_pages_func_t)(struct mve_rsrc_dma_mem_t *mem);

struct dma_mem_fptr
{
    alloc_func_t alloc;
    free_func_t free;
    clean_cache_func_t clean_cache;
    invalidate_cache_func_t invalidate_cache;
    map_func_t map;
    unmap_func_t unmap;
    get_pages_func_t get_pages;
};

static struct dma_mem_fptr fptrs[] =
{
    {
        .alloc = mve_blk1_rsrc_dma_mem_alloc_uncached,
        .free = mve_blk1_rsrc_dma_mem_free_uncached,
        .clean_cache = mve_blk1_rsrc_dma_mem_clean_cache_uncached,
        .invalidate_cache = mve_blk1_rsrc_dma_mem_invalidate_cache_uncached,
        .map = mve_blk1_rsrc_dma_mem_map_uncached,
        .unmap = mve_blk1_rsrc_dma_mem_unmap_unchached,
        .get_pages = mve_blk1_rsrc_dma_mem_get_pages_uncached,
    }
};

struct mve_rsrc_dma_mem_t *mve_blk1_rsrc_dma_mem_alloc(uint32_t size, enum mve_rsrc_dma_mem_type type)
{
    if (0 == size)
    {
        return NULL;
    }

    if (type >= DMA_MEM_TYPE_MAX)
    {
        return NULL;
    }

    return fptrs[type].alloc(size);
}

void mve_blk1_rsrc_dma_mem_free(struct mve_rsrc_dma_mem_t *mem)
{
    if (NULL != mem)
    {
        fptrs[mem->type].free(mem);
    }
}

void mve_blk1_rsrc_dma_mem_clean_cache(struct mve_rsrc_dma_mem_t *mem)
{
    if (NULL != mem)
    {
        fptrs[mem->type].clean_cache(mem);
    }
}

void mve_blk1_rsrc_dma_mem_invalidate_cache(struct mve_rsrc_dma_mem_t *mem)
{
    if (NULL != mem)
    {
        fptrs[mem->type].invalidate_cache(mem);
    }
}

void *mve_blk1_rsrc_dma_mem_map(struct mve_rsrc_dma_mem_t *mem)
{
    void *ret = NULL;

    if (NULL != mem)
    {
        ret = fptrs[mem->type].map(mem);
    }

    return ret;
}

void mve_blk1_rsrc_dma_mem_unmap(struct mve_rsrc_dma_mem_t *mem)
{
    if (NULL != mem)
    {
        fptrs[mem->type].unmap(mem);
    }
}

phys_addr_t *mve_blk1_rsrc_dma_mem_get_pages(struct mve_rsrc_dma_mem_t *mem)
{
    phys_addr_t *ret = NULL;

    if (NULL != mem)
    {
        ret = fptrs[mem->type].get_pages(mem);
    }

    return ret;
}

EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_alloc);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_free);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_clean_cache);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_invalidate_cache);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_map);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_unmap);
EXPORT_SYMBOL(mve_blk1_rsrc_dma_mem_get_pages);
