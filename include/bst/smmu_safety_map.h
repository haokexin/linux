/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MAP IOVA API for bst smmu implementations.
 *
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _SMMU_SAFETY_MAP_H
#define _SMMU_SAFETY_MAP_H

#define COREIP_NET_BTMEM_SID	(1)
#define COREIP_CV_DSP_SID		(2)
#define COREIP_CORE_DMA_SID		(3)
#define COREIP_NET_DSP_SID		(4)
#define COREIP_ISP_SID			(5)

#ifdef CONFIG_COREIP_SMMMU_MULTIOS_BST
extern int iommu_map_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size);
extern int iommu_unmap_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size);
#else
static inline int iommu_map_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size) { return -1; }
static inline int iommu_unmap_by_proxy(unsigned long sid, unsigned long iova, phys_addr_t paddr, size_t size) { return -1; }
#endif

#endif /* _SMMU_SAFETY_MAP_H */
