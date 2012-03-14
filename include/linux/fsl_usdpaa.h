/* Copyright 2011-2012 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef FSL_USDPAA_H
#define FSL_USDPAA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/uaccess.h>
#include <linux/ioctl.h>

#ifdef CONFIG_FSL_USDPAA

/* Allocation of resource IDs uses a generic interface. This enum is used to
 * distinguish between the type of underlying object being manipulated. */
enum usdpaa_id_type {
	usdpaa_id_fqid,
	usdpaa_id_bpid,
	usdpaa_id_qpool,
	usdpaa_id_cgrid,
	usdpaa_id_max /* <-- not a valid type, represents the number of types */
};
#define USDPAA_IOCTL_MAGIC 'u'
struct usdpaa_ioctl_get_region {
	uint64_t phys_start;
	uint64_t phys_len;
};
struct usdpaa_ioctl_id_alloc {
	uint32_t base; /* Return value, the start of the allocated range */
	enum usdpaa_id_type id_type; /* what kind of resource(s) to allocate */
	uint32_t num; /* how many IDs to allocate (and return value) */
	uint32_t align; /* must be a power of 2, 0 is treated like 1 */
	int partial; /* whether to allow less than 'num' */
};
struct usdpaa_ioctl_id_release {
	/* Input; */
	enum usdpaa_id_type id_type;
	uint32_t base;
	uint32_t num;
};
#define USDPAA_IOCTL_GET_PHYS_BASE \
	_IOR(USDPAA_IOCTL_MAGIC, 0x01, struct usdpaa_ioctl_get_region)
#define USDPAA_IOCTL_ID_ALLOC \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x02, struct usdpaa_ioctl_id_alloc)
#define USDPAA_IOCTL_ID_RELEASE \
	_IOW(USDPAA_IOCTL_MAGIC, 0x03, struct usdpaa_ioctl_id_release)

#ifdef __KERNEL__

/* Physical address range */
extern u64 usdpaa_phys_start;
extern u64 usdpaa_phys_size;

/* PFN versions */
extern unsigned long usdpaa_pfn_start;
extern unsigned long usdpaa_pfn_len;

/* TLB1 index */
extern unsigned int usdpaa_tlbcam_index;

/* Early-boot hook */
void __init fsl_usdpaa_init_early(void);

#endif /* __KERNEL__ */

#endif /* CONFIG_FSL_USDPAA */

#ifdef __KERNEL__
/* This interface is needed in a few places and though it's not specific to
 * USDPAA as such, creating a new header for it doesn't make any sense. The
 * qbman kernel driver implements this interface and uses it as the backend for
 * both the FQID and BPID allocators. The fsl_usdpaa driver also uses this
 * interface for tracking per-process allocations handed out to user-space. */
struct dpa_alloc {
	struct list_head list;
	spinlock_t lock;
};
#define DECLARE_DPA_ALLOC(name) \
	struct dpa_alloc name = { \
		.list = { \
			.prev = &name.list, \
			.next = &name.list \
		}, \
		.lock = __SPIN_LOCK_UNLOCKED(name.lock) \
	}
static inline void dpa_alloc_init(struct dpa_alloc *alloc)
{
	INIT_LIST_HEAD(&alloc->list);
	spin_lock_init(&alloc->lock);
}
int dpa_alloc_new(struct dpa_alloc *alloc, u32 *result, u32 count, u32 align,
		  int partial);
void dpa_alloc_free(struct dpa_alloc *alloc, u32 base_id, u32 count);
/* Like 'new' but specifies the desired range, returns -ENOMEM if the entire
 * desired range is not available, or 0 for success. */
int dpa_alloc_reserve(struct dpa_alloc *alloc, u32 base_id, u32 count);
/* Pops and returns contiguous ranges from the allocator. Returns -ENOMEM when
 * 'alloc' is empty. */
int dpa_alloc_pop(struct dpa_alloc *alloc, u32 *result, u32 *count);
#endif /* __KERNEL__ */

#ifdef __cplusplus
}
#endif

#endif /* FSL_USDPAA_H */
