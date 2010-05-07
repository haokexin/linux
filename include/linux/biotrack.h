#include <linux/cgroup.h>
#include <linux/mm.h>
#include <linux/page_cgroup.h>

#ifndef _LINUX_BIOTRACK_H
#define _LINUX_BIOTRACK_H

#ifdef CONFIG_CGROUP_BLKIO

struct io_context;
struct block_device;

struct blkio_cgroup {
	struct cgroup_subsys_state css;
	struct io_context *io_context;	/* default io_context */
/*	struct radix_tree_root io_context_root; per device io_context */
};

/**
 * __init_blkio_page_cgroup() - initialize a blkio_page_cgroup
 * @pc:		page_cgroup of the page
 *
 * Reset the owner ID of a page.
 */
static inline void __init_blkio_page_cgroup(struct page_cgroup *pc)
{
	pc->blkio_cgroup_id = 0;
}

/**
 * blkio_cgroup_disabled() - check whether blkio_cgroup is disabled
 *
 * Returns true if disabled, false if not.
 */
static inline bool blkio_cgroup_disabled(void)
{
	if (blkio_cgroup_subsys.disabled)
		return true;
	return false;
}

extern void blkio_cgroup_set_owner(struct page *page, struct mm_struct *mm);
extern void blkio_cgroup_reset_owner(struct page *page, struct mm_struct *mm);
extern void blkio_cgroup_reset_owner_pagedirty(struct page *page,
						 struct mm_struct *mm);
extern void blkio_cgroup_copy_owner(struct page *page, struct page *opage);

extern struct io_context *get_blkio_cgroup_iocontext(struct bio *bio);
extern unsigned long get_blkio_cgroup_id(struct bio *bio);
extern struct cgroup *get_cgroup_from_page(struct page *page);

#else /* !CONFIG_CGROUP_BLKIO */

struct blkio_cgroup;

static inline void __init_blkio_page_cgroup(struct page_cgroup *pc)
{
}

static inline bool blkio_cgroup_disabled(void)
{
	return true;
}

static inline void blkio_cgroup_set_owner(struct page *page,
						struct mm_struct *mm)
{
}

static inline void blkio_cgroup_reset_owner(struct page *page,
						struct mm_struct *mm)
{
}

static inline void blkio_cgroup_reset_owner_pagedirty(struct page *page,
						struct mm_struct *mm)
{
}

static inline void blkio_cgroup_copy_owner(struct page *page,
						struct page *opage)
{
}

static inline struct io_context *get_blkio_cgroup_iocontext(struct bio *bio)
{
	return NULL;
}

static inline unsigned long get_blkio_cgroup_id(struct bio *bio)
{
	return 0;
}

static inline struct cgroup *get_cgroup_from_page(struct page *page)
{
	return NULL;
}

#endif /* CONFIG_CGROUP_BLKIO */

#endif /* _LINUX_BIOTRACK_H */
