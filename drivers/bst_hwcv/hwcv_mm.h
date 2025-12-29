/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_MM_H__
#define __BST_HWCV_MM_H__

#include "hwcv_core.h"
#include "hwcv_uapi.h"

enum hwcv_buf_type {
	HWCV_INTERNAL_BUF,
	HWCV_EXTERNAL_BUF,
};

struct hwcv_buf {
	void *mem_priv;

	/* buffer type */
	enum hwcv_buf_type type;

	/* buffer addr */
	phys_addr_t phys_addr;
	dma_addr_t dma_addr;

	/* buffer size */
	u32 bytesused;
	u32 length;

	/* buffer handle */
	struct dma_buf *dbuf;
	int fd;

	/* buffer manager */
	int id;
	struct hwcv_session *session;
};

struct hwcv_mm {
	/* mm attrs */
	bool force_contiguous;
	struct device *dev;
	struct iommu_domain *iommud;
	const struct hwcv_mem_ops *ops;

	/* mm buffers */
	struct mutex lock;
	struct idr memory_idr;
	int buffer_count;
};

/*
 * Convert physical address which CPU see to HWCV hardware side
 * @pa: Physical from CPU side
 *
 * Returns physical address from HWCV hardware
 */
static inline u32 hwcv_phys_to_dma(phys_addr_t pa)
{
	if (pa >= 0xC00000000)
		return (pa - 0xB40000000);
	else
		return (pa - 0x780000000);
}

/*
 * Convert physical address which HWCV hardware see to CPU side
 * @pa: Physical from HWCV hardware side
 *
 * Returns physical address from CPU side
 */
static inline phys_addr_t hwcv_dma_to_phys(u32 dma_addr)
{
	if (dma_addr >= 0xC0000000)
		return ((phys_addr_t)dma_addr + 0xB40000000);
	else
		return ((phys_addr_t)dma_addr + 0x780000000);
}

int hwcv_mm_alloc_id(void *ptr);
void *hwcv_mm_remove_id(int id);
void *hwcv_mm_lookup_id(int id);

struct hwcv_buf *hwcv_mm_alloc_buf(u32 size);
struct hwcv_buf *hwcv_mm_import_buf(int fd, u32 size);
void hwcv_mm_release_buf(struct hwcv_buf *buf);
void hwcv_mm_sync_buf(struct hwcv_buf *buf, u8 dir);

int hwcv_mm_session_release_buffer(struct hwcv_session *session);
int hwcv_mm_init(struct hwcv_mm **mm_session);
int hwcv_mm_remove(struct hwcv_mm **mm_session);

#endif
