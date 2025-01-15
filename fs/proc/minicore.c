// SPDX-License-Identifier: GPL-2.0-only
/*
 * minicore Interface for accessing the crash
 * dump from the system's previous life.
 *
 *	Created by zhaoxu.deng (zhaoxu.deng@bst.ai)
 *	Copy from: Hariprasad Nellitheertha (hari@in.ibm.com)
 *	Copyright (C) IBM Corporation, 2004. All rights reserved
 * 	Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */

#include <linux/mm.h>
#include <linux/kcore.h>
#include <linux/user.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <linux/memblock.h>
#include <linux/init.h>
#include <linux/crash_dump.h>
#include <linux/list.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/uaccess.h>
#include <linux/mem_encrypt.h>
#include <linux/io.h>
#include <linux/cc_platform.h>
#include "internal.h"

/* Stores the pointer to the buffer containing kernel elf core headers. */
static char *minicore;

#if CONFIG_BST_MRDUMP_REGION_SIZE > 0
#define MRDUMP_SIZE (CONFIG_BST_MRDUMP_REGION_SIZE)
#else
#define MRDUMP_SIZE (10 * 1024 * 1024)
#endif

/* Total size of minicore file. */
static size_t minicore_size = MRDUMP_SIZE;
static size_t minicore_base = CONFIG_BST_MRDUMP_PHYS_BASE_ADDR + sizeof(size_t);

static struct proc_dir_entry *proc_minicore;

/*
 * Architectures which support memory encryption override this.
 */
ssize_t __weak
copy_oldmem_page_encrypted(struct iov_iter *iter, unsigned long pfn,
				   size_t csize, unsigned long offset)
{
	return copy_oldmem_page(iter, pfn, csize, offset);
}

/*
 * Returns > 0 for RAM pages, 0 for non-RAM pages, < 0 on error
 * The called function has to take care of module refcounting.
 */
static int (*oldmem_pfn_is_ram)(unsigned long pfn);

int register_oldmem_pfn_is_ram(int (*fn)(unsigned long pfn))
{
	if (oldmem_pfn_is_ram)
		return -EBUSY;
	oldmem_pfn_is_ram = fn;
	return 0;
}
EXPORT_SYMBOL_GPL(register_oldmem_pfn_is_ram);

void unregister_oldmem_pfn_is_ram(void)
{
	oldmem_pfn_is_ram = NULL;
	/*
	 * memory barrier
	 */
	wmb();
}
EXPORT_SYMBOL_GPL(unregister_oldmem_pfn_is_ram);

/*This function is copied from vmcore.c*/
static int pfn_is_ram(unsigned long pfn)
{
	int (*fn)(unsigned long pfn);
	/* pfn is ram unless fn() checks pagetype */
	int ret = 1;

	/*
	 * Ask hypervisor if the pfn is really ram.
	 * A ballooned page contains no data and reading from such a page
	 * will cause high load in the hypervisor.
	 */
	fn = oldmem_pfn_is_ram;
	if (fn)
		ret = fn(pfn);

	return ret;
}

/*
 * Reads a page from the oldmem device from given offset.
 * This function is copied from vmcore.c
 */
ssize_t read_from_oldmem(struct iov_iter *iter, size_t count,
			 u64 *ppos, bool encrypted)
{
	unsigned long pfn, offset;
	size_t nr_bytes;
	ssize_t read = 0, tmp;

	if (!count)
		return 0;

	offset = (unsigned long)(*ppos % PAGE_SIZE);
	pfn = (unsigned long)(*ppos / PAGE_SIZE);

	do {
		if (count > (PAGE_SIZE - offset))
			nr_bytes = PAGE_SIZE - offset;
		else
			nr_bytes = count;

		/* If pfn is not ram, return zeros for sparse dump files */
		if (!pfn_is_ram(pfn))
			tmp = iov_iter_zero(nr_bytes, iter);
		else {
			if (encrypted)
				tmp = copy_oldmem_page_encrypted(iter, pfn,
								 nr_bytes,
								 offset);
			else
				tmp = copy_oldmem_page(iter, pfn, nr_bytes,
						       offset);

			if (tmp < 0)
				return tmp;
		}
		*ppos += nr_bytes;
		count -= nr_bytes;
		read += nr_bytes;
		++pfn;
		offset = 0;
	} while (count);

	return read;
}

/*
 * Read from the ELF header and then the crash dump. On error, negative value is
 * returned otherwise number of bytes read are returned.
 */
static ssize_t __read_minicore(struct iov_iter *iter, loff_t *fpos)
{
	ssize_t acc = 0, tmp;
	size_t tsz;
	u64 start;

	if (!iov_iter_count(iter) || *fpos >= minicore_size)
		return 0;

	iov_iter_truncate(iter, minicore_size - *fpos);

	for (tsz = 0; tsz < MRDUMP_SIZE / PAGE_SIZE; tsz++) {
		start = minicore_base + *fpos;
		tmp = read_from_oldmem(iter, PAGE_SIZE, &start,
				cc_platform_has(CC_ATTR_MEM_ENCRYPT));
		if (tmp < 0) {
			pr_err("read_from_oldmem ret : %ld\n", tmp);
			return tmp;
		}
		*fpos += PAGE_SIZE;
		acc += PAGE_SIZE;

		/* leave now if filled buffer already */
		if (!iov_iter_count(iter))
			return acc;
	}

	return acc;
}

static ssize_t read_minicore(struct kiocb *iocb, struct iov_iter *iter)
{
	return __read_minicore(iter, &iocb->ki_pos);
}

static const struct proc_ops minicore_proc_ops = {
	.proc_read_iter	= read_minicore,
};

/* Init function for minicore module. */
static int __init minicore_init(void)
{
	int rc = 0;

	minicore = memremap(CONFIG_BST_MRDUMP_PHYS_BASE_ADDR,
				MRDUMP_SIZE, MEMREMAP_WB);
	if (!minicore) {
		pr_err("invalid memremap for 0x%lx\n",
				CONFIG_BST_MRDUMP_PHYS_BASE_ADDR);
		return -EINVAL;
	}
	memcpy(&minicore_size, minicore, sizeof(size_t));
	minicore += sizeof(size_t);

	proc_minicore = proc_create("minicore", 0400, NULL, &minicore_proc_ops);
	if (!proc_minicore) {
		pr_err("%s: failed to create minicore\n", __func__);
		rc = -EINVAL;
	}
	proc_minicore->size = minicore_size;

	return rc;
}

static void __exit minicore_exit(void)
{
	if (minicore)
		proc_remove(proc_minicore);
	if (minicore)
		memunmap(minicore);
}

module_init(minicore_init);
module_exit(minicore_exit);
