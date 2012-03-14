/* Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 * Authors: Andy Fleming <afleming@freescale.com>
 *	    Timur Tabi <timur@freescale.com>
 *	    Geoff Thorpe <Geoff.Thorpe@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/fsl_usdpaa.h>
#include <linux/fsl_qman.h>
#include <linux/fsl_bman.h>

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/slab.h>

/* Physical address range */
u64 usdpaa_phys_start;
u64 usdpaa_phys_size;

/* PFN versions */
unsigned long usdpaa_pfn_start;
unsigned long usdpaa_pfn_len;

/* TLB1 index */
unsigned int usdpaa_tlbcam_index;

/* Per-FD state (which should also be per-process but we don't enforce that) */
struct ctx {
	struct dpa_alloc ids[usdpaa_id_max];
};

/* Different resource classes */
static const struct alloc_backend {
	enum usdpaa_id_type id_type;
	int (*alloc)(u32 *, u32, u32, int);
	void (*release)(u32 base, unsigned int count);
	const char *acronym;
} alloc_backends[] = {
	{
		.id_type = usdpaa_id_fqid,
		.alloc = qman_alloc_fqid_range,
		.release = qman_release_fqid_range,
		.acronym = "FQID"
	},
	{
		.id_type = usdpaa_id_bpid,
		.alloc = bman_alloc_bpid_range,
		.release = bman_release_bpid_range,
		.acronym = "BPID"
	},
	{
		.id_type = usdpaa_id_qpool,
		.alloc = qman_alloc_pool_range,
		.release = qman_release_pool_range,
		.acronym = "QPOOL"
	},
	{
		.id_type = usdpaa_id_cgrid,
		.alloc = qman_alloc_cgrid_range,
		.release = qman_release_cgrid_range,
		.acronym = "CGRID"
	},
	{
		/* This terminates the array */
		.id_type = usdpaa_id_max
	}
};

static int usdpaa_open(struct inode *inode, struct file *filp)
{
	const struct alloc_backend *backend = &alloc_backends[0];
	struct ctx *ctx = kmalloc(sizeof(struct ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	filp->private_data = ctx;

	while (backend->id_type != usdpaa_id_max) {
		dpa_alloc_init(&ctx->ids[backend->id_type]);
		backend++;
	}

	filp->f_mapping->backing_dev_info = &directly_mappable_cdev_bdi;

	return 0;
}

static int usdpaa_release(struct inode *inode, struct file *filp)
{
	struct ctx *ctx = filp->private_data;
	const struct alloc_backend *backend = &alloc_backends[0];
	while (backend->id_type != usdpaa_id_max) {
		int ret, leaks = 0;
		do {
			u32 id, num;
			ret = dpa_alloc_pop(&ctx->ids[backend->id_type],
					    &id, &num);
			if (!ret) {
				leaks += num;
				backend->release(id, num);
			}
		} while (ret == 1);
		if (leaks)
			pr_crit("USDPAA process leaking %d %s%s\n", leaks,
				backend->acronym, (leaks > 1) ? "s" : "");
		backend++;
	}
	kfree(ctx);
	return 0;
}

static int usdpaa_mmap(struct file *file, struct vm_area_struct *vma)
{
	if (remap_pfn_range(vma, vma->vm_start,	usdpaa_pfn_start,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

/* Return the nearest rounded-up address >= 'addr' that is 'sz'-aligned. 'sz'
 * must be a power of 2, but both 'addr' and 'sz' can be expressions. */
#define USDPAA_MEM_ROUNDUP(addr, sz) \
	({ \
		unsigned long foo_align = (sz) - 1; \
		((addr) + foo_align) & ~foo_align; \
	})
/* Searching for a size-aligned virtual address range starting from 'addr' */
static unsigned long usdpaa_get_unmapped_area(struct file *file,
					      unsigned long addr,
					      unsigned long len,
					      unsigned long pgoff,
					      unsigned long flags)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	if (pgoff) {
		pr_err("%s: non-zero mmap page-offset 0x%lx is invalid\n",
			__func__, pgoff);
		return -EINVAL;
	}
	/* Only support mappings of the right size */
	if (len != usdpaa_phys_size) {
		pr_err("%s: mmap size 0x%lx doesn't match region (0x%llx)\n",
			__func__, len, usdpaa_phys_size);
		return -EINVAL;
	}
	addr = USDPAA_MEM_ROUNDUP(addr, len);
	vma = find_vma(mm, addr);
	/* Keep searching until we reach the end of currently-used virtual
	 * address-space or we find a big enough gap. */
	while (vma) {
		if ((addr + len) < vma->vm_start)
			return addr;
		addr = USDPAA_MEM_ROUNDUP(vma->vm_end, len);
		vma = vma->vm_next;
	}
	if ((TASK_SIZE - len) < addr)
		return -ENOMEM;
	return addr;
}

static long ioctl_get_region(void __user *arg)
{
	struct usdpaa_ioctl_get_region i = {
		.phys_start = usdpaa_phys_start,
		.phys_len = usdpaa_phys_size
	};
	return copy_to_user(arg, &i, sizeof(i));
}

static long ioctl_id_alloc(struct file *fp, void __user *arg)
{
	struct usdpaa_ioctl_id_alloc i;
	struct ctx *ctx = fp->private_data;
	const struct alloc_backend *backend;
	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Allocate the required resource type */
	ret = backend->alloc(&i.base, i.num, i.align, i.partial);
	if (ret < 0)
		return ret;
	i.num = ret;
	/* Copy the result to user-space */
	ret = copy_to_user(arg, &i, sizeof(i));
	if (ret) {
		backend->release(i.base, i.num);
		return ret;
	}
	/* Assign the allocated range to the FD accounting */
	dpa_alloc_free(&ctx->ids[i.id_type], i.base, i.num);
	return 0;
}

static long ioctl_id_release(struct file *fp, void __user *arg)
{
	struct usdpaa_ioctl_id_release i;
	struct ctx *ctx = fp->private_data;
	const struct alloc_backend *backend;
	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Pull the range out of the FD accounting - the range is valid iff this
	 * succeeds. */
	ret = dpa_alloc_reserve(&ctx->ids[i.id_type], i.base, i.num);
	if (ret)
		return ret;
	/* Release the resource to the backend */
	backend->release(i.base, i.num);
	return 0;
}
static long usdpaa_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	void __user *a = (void __user *)arg;
	switch (cmd) {
	case USDPAA_IOCTL_GET_PHYS_BASE:
		return ioctl_get_region(a);
	case USDPAA_IOCTL_ID_ALLOC:
		return ioctl_id_alloc(fp, a);
	case USDPAA_IOCTL_ID_RELEASE:
		return ioctl_id_release(fp, a);
	}
	return -EINVAL;
}

static const struct file_operations usdpaa_fops = {
	.open		   = usdpaa_open,
	.release	   = usdpaa_release,
	.mmap		   = usdpaa_mmap,
	.get_unmapped_area = usdpaa_get_unmapped_area,
	.unlocked_ioctl	   = usdpaa_ioctl,
	.compat_ioctl	   = usdpaa_ioctl
};

static struct miscdevice usdpaa_miscdev = {
	.name = "fsl-usdpaa",
	.fops = &usdpaa_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

/* Early-boot memory allocation. The boot-arg "usdpaa_mem=<x>" is used to
 * indicate how much memory (if any) to allocate during early boot. */
static __init int usdpaa_mem(char *arg)
{
	usdpaa_phys_size = memparse(arg, &arg);
	return 0;
}
early_param("usdpaa_mem", usdpaa_mem);

__init void fsl_usdpaa_init_early(void)
{
	int log;
	if (!usdpaa_phys_size) {
		pr_info("No USDPAA memory, no 'usdpaa_mem' bootarg\n");
		return;
	}
	/* Size must be 4^x * 4096, for some x */
	log = ilog2(usdpaa_phys_size);
	if ((usdpaa_phys_size & (usdpaa_phys_size - 1)) || (log < 12) ||
			(log & 1)) {
		pr_err("'usdpaa_mem' bootarg must be 4096*4^x\n");
		usdpaa_phys_size = 0;
		return;
	}
	usdpaa_phys_start = memblock_alloc(usdpaa_phys_size, usdpaa_phys_size);
	if (usdpaa_phys_start) {
		usdpaa_pfn_start = (usdpaa_phys_start >> PAGE_SHIFT);
		usdpaa_pfn_len = (usdpaa_phys_size >> PAGE_SHIFT);
		usdpaa_tlbcam_index = tlbcam_index++;
		pr_info("USDPAA region at %llx:%llx\n",
			usdpaa_phys_start, usdpaa_phys_size);
	} else
		pr_err("Failed to reserve USDPAA region (sz:%llx)\n",
		       usdpaa_phys_size);
}

static int __init usdpaa_init(void)
{
	int ret;

	pr_info("Freescale USDPAA process driver\n");

	if (!usdpaa_phys_size) {
		pr_warning("fsl-usdpaa: no region found\n");
		return 0;
	}
	ret = misc_register(&usdpaa_miscdev);
	if (ret)
		pr_err("fsl-usdpaa: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_exit(void)
{
	misc_deregister(&usdpaa_miscdev);
}

module_init(usdpaa_init);
module_exit(usdpaa_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA process driver");
