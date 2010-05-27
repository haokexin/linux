/*
 * Routines for doing kexec-based kdump.
 *
 * Copyright (C) 2005, IBM Corp.
 * Copyright (C) 2008, MontaVista Software Inc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */
#include <linux/highmem.h>
#include <linux/bootmem.h>
#include <linux/crash_dump.h>
#include <linux/uaccess.h>

#ifdef CONFIG_PROC_VMCORE
unsigned long long elfcorehdr_addr = ELFCORE_ADDR_MAX;
static int __init parse_elfcorehdr(char *p) {
	if (p)
		elfcorehdr_addr = memparse(p, &p);
	return 1;
}

__setup("elfcorehdr=", parse_elfcorehdr);
#endif

static int __init parse_savemaxmem(char *p) {
	if (p)
		saved_max_pfn = (memparse(p, &p) >> PAGE_SHIFT) - 1;

	return 1;
}

__setup("savemaxmem=", parse_savemaxmem);


static void *kdump_buf_page;

/**
 * copy_oldmem_page - copy one page from "oldmem"
 * @pfn: page frame number to be copied
 * @buf: target memory address for the copy; this can be in kernel address
 *    space or user address space (see @userbuf)
 * @csize: number of bytes to copy
 * @offset: offset in bytes into the page (based on pfn) to begin the
copy
 * @userbuf: if set, @buf is in user address space, use copy_to_user(),
 *    otherwise @buf is in kernel address space, use memcpy().
 *
 * Copy a page from "oldmem". For this page, there is no pte mapped
 * in the current kernel.
 *
 * Calling copy_to_user() in atomic context is not desirable. Hence
first
 * copying the data to a pre-allocated kernel page and then copying to
user
 * space in non-atomic context.
 */
ssize_t copy_oldmem_page(unsigned long pfn, char *buf,
			 size_t csize, unsigned long offset, int userbuf) {
	void *vaddr;

	if (!csize)
		return 0;

	/* see if the PFN is valid and present */
	if (pfn_present(pfn) && pfn_valid(pfn)) {
		vaddr = kmap_atomic_pfn(pfn, KM_PTE0);

		if (!userbuf) {
			memcpy(buf, (vaddr + offset), csize);
			kunmap_atomic(vaddr, KM_PTE0);
		} else {
			if (!kdump_buf_page) {
				printk(KERN_WARNING "Kdump: Kdump buffer " \
					"page not allocated\n");
				return -EFAULT;
			}
			copy_page(kdump_buf_page, vaddr);
			kunmap_atomic(vaddr, KM_PTE0);
			if (copy_to_user(buf,
				(kdump_buf_page + offset), csize))
				return -EFAULT;
		}
	} else {
		/* the PFN isn't present and/or valid in the paging tables
		 * use the __va() macro to get the virtual address from the PFN
		 */
		vaddr = (void *)(__va(pfn * PAGE_SIZE));
		if (!userbuf) {
			memcpy(buf, (vaddr + offset), csize);
		} else {
			if (copy_to_user(buf, (vaddr + offset), csize))
				return -EFAULT;
		}
	}

	return csize;
}

static int __init kdump_buf_page_init(void) {
	int ret = 0;

	kdump_buf_page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!kdump_buf_page) {
		printk(KERN_WARNING
		       "Kdump: Failed to allocate kdump buffer" " page\n");
		ret = -ENOMEM;
	}

	return ret;
}

arch_initcall(kdump_buf_page_init);
