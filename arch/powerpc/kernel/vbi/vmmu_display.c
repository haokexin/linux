/*
 * vmmu_display.c - hypervisor VMMU operations
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

#include <linux/kernel.h>
#include <vbi/interface.h>
#include <vbi/vmmu.h>

#define __va(paddr) (((unsigned long )(paddr))+0xC0000000)
#define __pa(vaddr) (((unsigned long )(vaddr))-0xC0000000)


/*
 *
 * vmmuPageTableDisplay - display information about the specified page table
 *
 * This routine display all the VMMU PTE entries in the specified table
 *
 */

void vmmuPageTableDisplay(VMMU_LEVEL_1_DESC *l1, int vmmuon)
{
	VMMU_LEVEL_2_DESC *l2;
	VMMU_PTE *pte;
	VMMU_EFFECTIVE_ADDR ea;
	u_int l1_index;
	u_int i,j;

	l1_index = 0;
	ea.addr = 0;

	printk("Logical           Physical          R C U[01234567] WIMGE S[XWR] U[XWR]\n");
	printk("----------------- ----------------- - -  ---------- -----  -----  -----\n");

	/* run through all the entries */
	for (i=0; i<VMMU_L1_ENTRIES; i++) {
		if (l1->field.v) {
			ea.field.l1index = l1_index;
			l2 = (VMMU_LEVEL_2_DESC *)VMMU_LBA_TO_ADDR(l1->field.l2ba);
			if (vmmuon)
				l2 = (VMMU_LEVEL_2_DESC *)__va(l2);

			pte = (VMMU_PTE *)l2;

			for (j=0; j<VMMU_L2_ENTRIES; j++) {
				if (pte->field.v) {
					ea.field.l2index = j;
					printk ("%08x-%08x %08x-%08x %d %d ",
					(u_int)ea.addr, (u_int)ea.addr + 0xfff,
					pte->field.rpn << VMMU_RPN_SHIFT,
					(pte->field.rpn << VMMU_RPN_SHIFT) + 0xfff,
					pte->field.r, pte->field.c);
					printk ("  %d%d%d%d%d%d%d%d  %d%d%d%d%d   %c%c%c    %c%c%c\n",
					pte->field.u0, pte->field.u1,
					pte->field.u2, pte->field.u3,
					pte->field.u4, pte->field.u5,
					pte->field.u6, pte->field.u7,
					pte->field.w, pte->field.i,
					pte->field.m,
					pte->field.g, pte->field.e,
					pte->field.sx ? 'X' : ' ',
					pte->field.sw ? 'W' : ' ',
					pte->field.sr ? 'R' : ' ',
					pte->field.ux ? 'X' : ' ',
					pte->field.uw ? 'W' : ' ',
					pte->field.ur ? 'R' : ' ');
				} /* pte field.v */
				pte++;
			} /* j */
		} /* l1 field.v */
		l1++;
		l1_index++;
	} /* i */
}

/*
 * vmmuPteDisplay - display a specific PTE entry
 *
 * This routine display the VMMU PTE entrie corresponding to the specified
 * virtual address.
 *
 */
unsigned int vmmuPteDisplay(VMMU_LEVEL_1_DESC *l1, void *vaddr)
{
	VMMU_LEVEL_2_DESC  *l2;
	VMMU_PTE *pte;

	/* find the level-1 page table descriptor for the virtual address */
	l1 += VMMU_L1_INDEX(vaddr);

	/* if no level-2 table exists abort and return error */
	if (!l1->field.v)
		return -1;

	/* locate correct PTE entry in level-2 table */
	l2  = (VMMU_LEVEL_2_DESC *)VMMU_LBA_TO_ADDR(l1->field.l2ba) +
		VMMU_L2_INDEX(vaddr);

	l2 = (VMMU_LEVEL_2_DESC *)__va(l2);

	pte = &l2->pte;

	if (!pte->field.v)
		return  -1;

	printk("PTE for virtual address 0x%p:\n", vaddr);
	printk("  Page Number:  0x%08x\n", pte->field.rpn<<VMMU_RPN_SHIFT);
	printk("  Referenced:   %d\n", pte->field.r);
	printk("  Changed:      %d\n", pte->field.c);
	printk("  User bits:    %d%d%d%d%d%d%d%d\n",
		pte->field.u0, pte->field.u1,
		pte->field.u2, pte->field.u3,
		pte->field.u4, pte->field.u5,
		pte->field.u6, pte->field.u7);
	printk("  WIMGE:        %d%d%d%d%d\n",
		pte->field.w, pte->field.i, pte->field.m,
		pte->field.g, pte->field.e);
	printk("  Supv Perms:   %c%c%c\n",
		pte->field.sr ? 'R' : '-',
		pte->field.sw ? 'W' : '-',
		pte->field.sx ? 'X' : '-');
	printk("  User Perms:   %c%c%c\n",
		pte->field.ur ? 'R' : '-',
		pte->field.uw ? 'W' : '-',
		pte->field.ux ? 'X' : '-');

	return 0;
}
