/*
 * util.c - utilities routines for guest OS para-virtualization
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

/*
This module implements a library which is handy for para-virtualized
guest os to use. The routines are developed based on the need while
para-virtualize linux, therefore, may need some tweaks to be generic.
*/

#include <asm/page.h>
#include <linux/module.h>
#include <vbi/interface.h>
#include <vbi/vmmu.h>
#include <vbi/syscall.h>
#include <vbi/vbi.h>


/* defines */

/* globals */

/*
 * wr_config is initialized as part of the guest os init, before os turns on
 * MMU. For paravirualized linux, it is initialized in plaform_init().
 */

extern struct vb_config *wr_config;
extern struct vb_status *wr_status;
extern struct vb_control *wr_control;

/* local */

/* extern */
extern void pteAttrSet(VMMU_PTE * pte, u_int attr);
extern void vmmuPageTableDisplay(VMMU_LEVEL_1_DESC *l1, int vmmuon);

/* forward declarations */

/*
 * vb_memsize_get should not be called before wr_config is initialized
 */
unsigned int vb_memsize_get(void)
{
	if (wr_config == (struct vb_config *)(-1)) 
		return 0;
	return VBI_MEM_SIZE_GET();
}

unsigned int vb_context_get(void)
{
	if (wr_config == (struct vb_config *)(-1))
		return 0xdeadbee0;
	return VBI_CONTEXT_ID_GET();
}

void vb_pte_set(void *pPte, unsigned long paddr, int protval)
{

	/* caller has guaranteed pPte != NULL */

	*(uint *) pPte = (uint) VMMU_PTE_VALID_MASK;

	/* linux uses more than the permission bits, in word1 of PTE */

	*((uint *) ((uint *) pPte) + 1) = (((u_int) paddr & VMMU_PTE_RPN_MASK) | (protval & 0xfff));

	return;
}

/*
 * turn on mmu for the particular context
 *
 * note, caller must make sure, context switch inside the guest OS must
 * not happen during this call.
 */

int vb_context_mmu_on(int pid,	/* context id */
	void *pgtable, int pagesize, int asid, int vmmu_handle,
	int debug)
{
	static VMMU_CONFIG vmmu_cfg;

	if (wr_config == (struct vb_config *)(- 1) || pgtable == NULL || pagesize <= 0)
		return -1;

#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
	/* Create needs to be called when dealing with kernel only,
	   once we create a process, in this case we don't need to
	   copy the kernel page tables since swapper_pg_dir is being
	   passed in. */
	if (asid == 1) { /* special case.  Kernel ASID needs to
		be created here.  Once the first userspace process is
		available init, create and destroy will do all the heavy
		lifting. */
		vmmu_cfg.addr = (VMMU_LEVEL_1_DESC *) pgtable;
		vmmu_cfg.flush_type = VMMU_TLB_FLUSH_ASID;
		vmmu_cfg.asid = asid;
		vmmu_cfg.vmmu_handle = vmmu_handle;

		if ((vbi_create_vmmu(&vmmu_cfg)) != 0)
			return -1;
	}
	wr_control->vb_control_regs.asid = asid;
	wr_control->vb_control_regs.vmmu_handle = vmmu_handle;
	vbi_load_ctx();
#else
	vmmu_cfg.addr = (VMMU_LEVEL_1_DESC *) pgtable;
	vmmu_cfg.flush_type = pagesize;
	vmmu_cfg.asid = pid;
	vmmu_cfg.vmmu_handle = 0; /* only vmmu 0 is supported for now */

	if ((vbi_config_vmmu(&vmmu_cfg)) != 0)
		return -1;

	if (debug) {
		printk("L1 page table address %p\n", pgtable);
		vmmuPageTableDisplay(pgtable, 0);
		printk("End of page table display \n");
	}

	vbi_enable_vmmu(vmmu_cfg.vmmu_handle);
#endif
	return 0;
}
EXPORT_SYMBOL(vb_context_mmu_on);

void vb__flush_dcache_icache(void *start)
{
	vbi_flush_icache(start, 4096);
	vbi_flush_dcache(start, 4096);
}

void vb_flush_dcache_range(unsigned long start, unsigned long stop)
{
	vbi_flush_dcache((void *) start, (stop - start + 1));
}

void vb__flush_icache_range(unsigned long start, unsigned long stop)
{
	vbi_update_text_cache((void *) start, (stop - start + 1));
}

void vb__flush_dcache_icache_phys(unsigned long physaddr)
{
	vbi_flush_icache((void *) physaddr, 4096);
	vbi_flush_dcache((void *) physaddr, 4096);
}

EXPORT_SYMBOL(wrhv_int_lock);
EXPORT_SYMBOL(wrhv_int_unlock);
EXPORT_SYMBOL(wrhv_int_lvl_get);
