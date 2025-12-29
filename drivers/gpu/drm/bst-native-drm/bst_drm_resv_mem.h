// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#ifndef BST_DRM_RESV_MEM_H
#define BST_DRM_RESV_MEM_H

struct bst_dev;

struct bst_drm_resv_memblock {
    struct bst_dev *mdev;
    phys_addr_t phys_addr;
    void *vaddr;
    uint32_t size;
};

struct bst_drm_resv_mem_ops {
    struct bst_drm_resv_memblock *(*alloc)(struct bst_dev *mdev, uint32_t size, uint32_t align);
    void (*free)(struct bst_drm_resv_memblock *memblock);
    void (*invalid_cache)(struct bst_drm_resv_memblock *memblock);
    void (*clean_cache)(struct bst_drm_resv_memblock *memblock);
    void (*flush_write_buffer)(void);
};

struct bst_drm_resv_mem_ops *get_resv_mem_ops(void);

#endif