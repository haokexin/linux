/* Copyright (C) 2008-2011 Freescale Semiconductor, Inc.
 * Authors: Andy Fleming <afleming@freescale.com>
 *          Timur Tabi <timur@freescale.com>
 *          Geoff Thorpe <Geoff.Thorpe@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/fsl_usdpaa.h>

#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/lmb.h>

/* Physical address range */
u64 usdpaa_phys_start;
u64 usdpaa_phys_size;

/* PFN versions */
unsigned long usdpaa_pfn_start;
unsigned long usdpaa_pfn_len;

/* TLB1 index */
unsigned int usdpaa_tlbcam_index;

static int usdpaa_shmem_open(struct inode *inode, struct file *filp)
{
	filp->f_mapping->backing_dev_info = &directly_mappable_cdev_bdi;

	return 0;
}

static int usdpaa_shmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff) {
		pr_err("%s: non-zero mmap page-offset 0x%lx is invalid\n",
			__func__, vma->vm_pgoff);
		return -EINVAL;
	}
	if (size != usdpaa_phys_size) {
		pr_err("%s: mmap size 0x%llx doesn't match region (0x%llx)\n",
			__func__, (unsigned long long)size, usdpaa_phys_size);
		return -EINVAL;
	}
	if (vma->vm_start & (usdpaa_phys_size - 1)) {
		pr_err("%s: un-aligned mapping %llx:%llx -> %lx\n",
			__func__, usdpaa_phys_start, usdpaa_phys_size,
			vma->vm_start);
		return -EINVAL;
	}
	if (remap_pfn_range(vma, vma->vm_start,	usdpaa_pfn_start, size,
				vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static long usdpaa_shmem_ioctl(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
	struct usdpaa_ioctl_get_region ret = {
		.phys_start = usdpaa_phys_start,
		.phys_len = usdpaa_phys_size
	};
	if (cmd != USDPAA_IOCTL_GET_PHYS_BASE)
		return -EINVAL;
	return copy_to_user((void __user *)arg, &ret, sizeof(ret));
}

static const struct file_operations shmem_fops = {
	.open		= usdpaa_shmem_open,
	.mmap		= usdpaa_shmem_mmap,
	.unlocked_ioctl = usdpaa_shmem_ioctl
};

static struct miscdevice usdpaa_shmem_miscdev = {
	.name = "fsl-usdpaa-shmem",
	.fops = &shmem_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

__init void fsl_usdpaa_shmem_init_early(void)
{
	u64 sz = (u64)PAGE_SIZE << (2 * CONFIG_FSL_USDPAA_SHMEM_LOG4);
	u64 addr = (lmb_end_of_DRAM() - sz) & ~(sz - 1);
	/* FIXME: if booting with 8GB of RAM, the upper memory region seems to
	 * be unavailable because of some conflict with the ramdisk. Ensure that
	 * we don't search above 6GB. */
	if (addr >= 0x180000000ULL)
		addr = (0x180000000ULL - sz) & ~(sz - 1);
	/* Search downwards, looking for an appropriately-aligned region that
	 * isn't already reserved. */
	do {
		if (lmb_is_region_reserved(addr, sz) == -1)
			goto found;
		addr -= sz;
		/* If we reach the lower 1GB, call off the search. */
	} while (addr >= 0x40000000);
	pr_err("Failed to reserve USDPAA region (sz:%llx)\n", sz);
	return;
found:
	lmb_reserve(addr, sz);
	usdpaa_phys_start = addr;
	usdpaa_phys_size = sz;
	usdpaa_pfn_start = (addr >> PAGE_SHIFT);
	usdpaa_pfn_len = (sz >> PAGE_SHIFT);
	usdpaa_tlbcam_index = tlbcam_index++;
	pr_info("USDPAA region at %llx:%llx\n",
		usdpaa_phys_start, usdpaa_phys_size);
}

static int __init usdpaa_shmem_init(void)
{
	int ret;

	pr_info("Freescale USDPAA shared memory driver\n");

	if (!usdpaa_phys_size) {
		pr_warning("fsl-usdpaa-shmem: no region found\n");
		return 0;
	}
	ret = misc_register(&usdpaa_shmem_miscdev);
	if (ret)
		pr_err("fsl-usdpaa-shmem: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_shmem_exit(void)
{
	misc_deregister(&usdpaa_shmem_miscdev);
}

module_init(usdpaa_shmem_init);
module_exit(usdpaa_shmem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA shared memory driver");
