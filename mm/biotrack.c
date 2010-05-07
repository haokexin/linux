/* biotrack.c - Block I/O Tracking
 *
 * Copyright (C) VA Linux Systems Japan, 2008-2009
 * Developed by Hirokazu Takahashi <taka@valinux.co.jp>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/smp.h>
#include <linux/bit_spinlock.h>
#include <linux/blkdev.h>
#include <linux/biotrack.h>
#include <linux/mm_inline.h>
#include <linux/seq_file.h>
#include <linux/dm-ioctl.h>
#include <../drivers/md/dm-ioband.h>

/*
 * The block I/O tracking mechanism is implemented on the cgroup memory
 * controller framework. It helps to find the the owner of an I/O request
 * because every I/O request has a target page and the owner of the page
 * can be easily determined on the framework.
 */

/* Return the blkio_cgroup that associates with a cgroup. */
static inline struct blkio_cgroup *cgroup_blkio(struct cgroup *cgrp)
{
	return container_of(cgroup_subsys_state(cgrp, blkio_cgroup_subsys_id),
					struct blkio_cgroup, css);
}

/* Return the blkio_cgroup that associates with a process. */
static inline struct blkio_cgroup *blkio_cgroup_from_task(struct task_struct *p)
{
	return container_of(task_subsys_state(p, blkio_cgroup_subsys_id),
					struct blkio_cgroup, css);
}

static struct io_context default_blkio_io_context;
static struct blkio_cgroup default_blkio_cgroup = {
	.io_context	= &default_blkio_io_context,
};
static DEFINE_MUTEX(ioband_ops_lock);
static const struct ioband_cgroup_ops *ioband_ops;

/**
 * blkio_cgroup_set_owner() - set the owner ID of a page.
 * @page:	the page we want to tag
 * @mm:		the mm_struct of a page owner
 *
 * Make a given page have the blkio-cgroup ID of the owner of this page.
 */
void blkio_cgroup_set_owner(struct page *page, struct mm_struct *mm)
{
	struct blkio_cgroup *biog;
	struct page_cgroup *pc;

	if (blkio_cgroup_disabled())
		return;
	pc = lookup_page_cgroup(page);
	if (unlikely(!pc))
		return;

	pc->blkio_cgroup_id = 0;	/* 0: default blkio_cgroup id */
	if (!mm)
		return;
	/*
	 * Locking "pc" isn't necessary here since the current process is
	 * the only one that can access the members related to blkio_cgroup.
	 */
	rcu_read_lock();
	biog = blkio_cgroup_from_task(rcu_dereference(mm->owner));
	if (unlikely(!biog))
		goto out;
	/*
	 * css_get(&bio->css) isn't called to increment the reference
	 * count of this blkio_cgroup "biog" so pc->blkio_cgroup_id
	 * might turn invalid even if this page is still active.
	 * This approach is chosen to minimize the overhead.
	 */
	pc->blkio_cgroup_id = css_id(&biog->css);
out:
	rcu_read_unlock();
}

/**
 * blkio_cgroup_reset_owner() - reset the owner ID of a page
 * @page:	the page we want to tag
 * @mm:		the mm_struct of a page owner
 *
 * Change the owner of a given page if necessary.
 */
void blkio_cgroup_reset_owner(struct page *page, struct mm_struct *mm)
{
	/*
	 * A little trick:
	 * Just call blkio_cgroup_set_owner() for pages which are already
	 * active since the blkio_cgroup_id member of page_cgroup can be
	 * updated without any locks. This is because an integer type of
	 * variable can be set a new value at once on modern cpus.
	 */
	blkio_cgroup_set_owner(page, mm);
}

/**
 * blkio_cgroup_reset_owner_pagedirty() - reset the owner ID of a pagecache page
 * @page:	the page we want to tag
 * @mm:		the mm_struct of a page owner
 *
 * Change the owner of a given page if the page is in the pagecache.
 */
void blkio_cgroup_reset_owner_pagedirty(struct page *page, struct mm_struct *mm)
{
	if (!page_is_file_cache(page))
		return;
	if (current->flags & PF_MEMALLOC)
		return;

	blkio_cgroup_reset_owner(page, mm);
}

/**
 * blkio_cgroup_copy_owner() - copy the owner ID of a page into another page
 * @npage:	the page where we want to copy the owner
 * @opage:	the page from which we want to copy the ID
 *
 * Copy the owner ID of @opage into @npage.
 */
void blkio_cgroup_copy_owner(struct page *npage, struct page *opage)
{
	struct page_cgroup *npc, *opc;

	if (blkio_cgroup_disabled())
		return;
	npc = lookup_page_cgroup(npage);
	if (unlikely(!npc))
		return;
	opc = lookup_page_cgroup(opage);
	if (unlikely(!opc))
		return;

	/*
	 * Do this without any locks. The reason is the same as
	 * blkio_cgroup_reset_owner().
	 */
	npc->blkio_cgroup_id = opc->blkio_cgroup_id;
}

/* Create a new blkio-cgroup. */
static struct cgroup_subsys_state *
blkio_cgroup_create(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	struct blkio_cgroup *biog;
	struct io_context *ioc;

	if (!cgrp->parent) {
		biog = &default_blkio_cgroup;
		init_io_context(biog->io_context);
		/* Increment the referrence count not to be released ever. */
		atomic_long_inc(&biog->io_context->refcount);
		return &biog->css;
	}

	biog = kzalloc(sizeof(*biog), GFP_KERNEL);
	if (!biog)
		return ERR_PTR(-ENOMEM);
	ioc = alloc_io_context(GFP_KERNEL, -1);
	if (!ioc) {
		kfree(biog);
		return ERR_PTR(-ENOMEM);
	}
	biog->io_context = ioc;
	return &biog->css;
}

/* Delete the blkio-cgroup. */
static void blkio_cgroup_destroy(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	struct blkio_cgroup *biog = cgroup_blkio(cgrp);
	int id;

	mutex_lock(&ioband_ops_lock);
	if (ioband_ops) {
		id = css_id(&biog->css);
		ioband_ops->remove_group(id);
	}
	mutex_unlock(&ioband_ops_lock);

	put_io_context(biog->io_context);
	free_css_id(&blkio_cgroup_subsys, &biog->css);
	kfree(biog);
}

/**
 * get_blkio_cgroup_id() - determine the blkio-cgroup ID
 * @bio:	the &struct bio which describes the I/O
 *
 * Returns the blkio-cgroup ID of a given bio. A return value zero
 * means that the page associated with the bio belongs to default_blkio_cgroup.
 */
unsigned long get_blkio_cgroup_id(struct bio *bio)
{
	struct page_cgroup *pc;
	struct page *page = bio_iovec_idx(bio, 0)->bv_page;
	unsigned long id = 0;

	pc = lookup_page_cgroup(page);
	if (pc)
		id = pc->blkio_cgroup_id;
	return id;
}
EXPORT_SYMBOL(get_blkio_cgroup_id);

/**
 * get_blkio_cgroup_iocontext() - determine the blkio-cgroup iocontext
 * @bio:	the &struct bio which describe the I/O
 *
 * Returns the iocontext of blkio-cgroup that issued a given bio.
 */
struct io_context *get_blkio_cgroup_iocontext(struct bio *bio)
{
	struct cgroup_subsys_state *css;
	struct blkio_cgroup *biog;
	struct io_context *ioc;
	unsigned long id;

	id = get_blkio_cgroup_id(bio);
	rcu_read_lock();
	css = css_lookup(&blkio_cgroup_subsys, id);
	if (css)
		biog = container_of(css, struct blkio_cgroup, css);
	else
		biog = &default_blkio_cgroup;
	ioc = biog->io_context;	/* default io_context for this cgroup */
	atomic_long_inc(&ioc->refcount);
	rcu_read_unlock();
	return ioc;
}
EXPORT_SYMBOL(get_blkio_cgroup_iocontext);

/**
 * get_cgroup_from_page() - determine the cgroup from a page.
 * @page:	the page to be tracked
 *
 * Returns the cgroup of a given page. A return value zero means that
 * the page associated with the page belongs to default_blkio_cgroup.
 *
 * Note:
 * This function must be called under rcu_read_lock().
 */
struct cgroup *get_cgroup_from_page(struct page *page)
{
	struct page_cgroup *pc;
	struct cgroup_subsys_state *css;

	pc = lookup_page_cgroup(page);
	if (!pc || !pc->blkio_cgroup_id)
		return NULL;

	css = css_lookup(&blkio_cgroup_subsys, pc->blkio_cgroup_id);
	if (!css)
		return NULL;

	return css->cgroup;
}
EXPORT_SYMBOL(get_cgroup_from_page);

/**
 * blkio_cgroup_register_ioband() - register ioband
 * @p:	a pointer to struct ioband_cgroup_ops
 *
 * Calling with NULL means unregistration.
 * Returns 0 on success.
 */
int blkio_cgroup_register_ioband(const struct ioband_cgroup_ops *p)
{
	if (blkio_cgroup_disabled())
		return -1;

	mutex_lock(&ioband_ops_lock);
	ioband_ops = p;
	mutex_unlock(&ioband_ops_lock);
	return 0;
}
EXPORT_SYMBOL(blkio_cgroup_register_ioband);

/* Read the ID of the specified blkio cgroup. */
static u64 blkio_id_read(struct cgroup *cgrp, struct cftype *cft)
{
	struct blkio_cgroup *biog = cgroup_blkio(cgrp);

	return (u64)css_id(&biog->css);
}

/* Show all ioband devices and their settings. */
static int blkio_devs_read(struct cgroup *cgrp, struct cftype *cft,
							struct seq_file *m)
{
	mutex_lock(&ioband_ops_lock);
	if (ioband_ops)
		ioband_ops->show_device(m);
	mutex_unlock(&ioband_ops_lock);
	return 0;
}

/* Configure ioband devices specified by an ioband device ID */
static int blkio_devs_write(struct cgroup *cgrp, struct cftype *cft,
							const char *buffer)
{
	char **argv;
	int argc, r = 0;

	if (cgrp != cgrp->top_cgroup)
		return -EACCES;

	argv = argv_split(GFP_KERNEL, buffer, &argc);
	if (!argv)
		return -ENOMEM;

	mutex_lock(&ioband_ops_lock);
	if (ioband_ops)
		r = ioband_ops->config_device(argc, argv);
	mutex_unlock(&ioband_ops_lock);

	argv_free(argv);
	return r;
}

/* Show the information of the specified blkio cgroup. */
static int blkio_group_read(struct cgroup *cgrp, struct cftype *cft,
							struct seq_file *m)
{
	struct blkio_cgroup *biog;
	int id;

	mutex_lock(&ioband_ops_lock);
	if (ioband_ops) {
		biog = cgroup_blkio(cgrp);
		id = css_id(&biog->css);
		ioband_ops->show_group(m, cft->private, id);
	}
	mutex_unlock(&ioband_ops_lock);
	return 0;
}

/* Configure the specified blkio cgroup. */
static int blkio_group_config_write(struct cgroup *cgrp, struct cftype *cft,
							const char *buffer)
{
	struct blkio_cgroup *biog;
	char **argv;
	int argc, parent, id, r = 0;

	argv = argv_split(GFP_KERNEL, buffer, &argc);
	if (!argv)
		return -ENOMEM;

	mutex_lock(&ioband_ops_lock);
	if (ioband_ops) {
		if (cgrp == cgrp->top_cgroup)
			parent = 0;
		else {
			biog = cgroup_blkio(cgrp->parent);
			parent = css_id(&biog->css);
		}
		biog = cgroup_blkio(cgrp);
		id = css_id(&biog->css);
		r = ioband_ops->config_group(argc, argv, parent, id);
	}
	mutex_unlock(&ioband_ops_lock);
	argv_free(argv);
	return r;
}

/* Reset the statictics counter of the specified blkio cgroup. */
static int blkio_group_stats_write(struct cgroup *cgrp, struct cftype *cft,
							const char *buffer)
{
	struct blkio_cgroup *biog;
	char **argv;
	int argc, id, r = 0;

	argv = argv_split(GFP_KERNEL, buffer, &argc);
	if (!argv)
		return -ENOMEM;

	mutex_lock(&ioband_ops_lock);
	if (ioband_ops) {
		biog = cgroup_blkio(cgrp);
		id = css_id(&biog->css);
		r = ioband_ops->reset_group_stats(argc, argv, id);
	}
	mutex_unlock(&ioband_ops_lock);
	argv_free(argv);
	return r;
}

static struct cftype blkio_files[] = {
	{
		.name = "id",
		.read_u64 = blkio_id_read,
	},
	{
		.name = "devices",
		.read_seq_string = blkio_devs_read,
		.write_string = blkio_devs_write,
	},
	{
		.name = "settings",
		.read_seq_string = blkio_group_read,
		.write_string = blkio_group_config_write,
		.private = IOG_INFO_CONFIG,
	},
	{
		.name = "stats",
		.read_seq_string = blkio_group_read,
		.write_string = blkio_group_stats_write,
		.private = IOG_INFO_STATS,
	},
};

static int blkio_cgroup_populate(struct cgroup_subsys *ss, struct cgroup *cgrp)
{
	return cgroup_add_files(cgrp, ss, blkio_files,
					ARRAY_SIZE(blkio_files));
}

struct cgroup_subsys blkio_cgroup_subsys = {
	.name		= "blkio",
	.create		= blkio_cgroup_create,
	.destroy	= blkio_cgroup_destroy,
	.populate	= blkio_cgroup_populate,
	.subsys_id	= blkio_cgroup_subsys_id,
	.use_id		= 1,
};
