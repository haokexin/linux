/*
 * PPC64 Huge TLB Page Support for Book3E MMU
 *
 * Copyright (C) 2009 David Gibson, IBM Corporation.
 *
 */
#include <linux/mm.h>
#include <linux/hugetlb.h>

static inline int mmu_get_tsize(int psize)
{
	return mmu_psize_defs[psize].enc;
}

static int book3e_tlbsx(unsigned long ea, unsigned long pid)
{
	unsigned long mas6, mas0;
	int found;

	mas6 = pid << 16;

	mtspr(SPRN_MAS6, mas6);
	asm volatile(
		"li	%0,0\n"
		"tlbsx.	0,%1\n"
		"bne	1f\n"
		"li	%0,1\n"
		"1:\n"
		: "=&r"(found) : "r"(ea));

	if (found) {
		mas0 = mfspr(SPRN_MAS0);
		return mas0;
	} else {
		return -1;
	}
}

void book3e_hugetlb_preload(struct mm_struct *mm, unsigned long ea, pte_t pte)
{
	unsigned long mas1, mas2, mas7_3;
	unsigned long psize;

	if (is_kernel_addr(ea))
		return;

	psize = get_slice_psize(mm, ea);

	if (book3e_tlbsx(ea, mm->context.id) != -1)
		return;

	mas1 = MAS1_VALID | MAS1_TID(mm->context.id)
		| MAS1_TSIZE(mmu_get_tsize(psize));
	mas2 = ea & ~((1UL << mmu_psize_defs[psize].shift)-1);
	mas2 |= (pte_val(pte) >> 19) & 0x1f; /* WIMGE bits */
	mas7_3 = pte_pfn(pte) << PAGE_SHIFT;
	mas7_3 |= (pte_val(pte) & 0x03c000) >> 8; /* Un bits */
	mas7_3 |= (pte_val(pte) & 0x0000fc) >> 2; /* BAP bits */
	if (!pte_dirty(pte))
		mas7_3 &= ~(MAS3_SW|MAS3_UW);

	mtspr(SPRN_MAS1, mas1);
	mtspr(SPRN_MAS2, mas2);
	mtspr(SPRN_MAS7_MAS3, mas7_3);
	asm volatile ("tlbwe");
}

void flush_hugetlb_page(struct vm_area_struct *vma, unsigned long vmaddr)
{
	struct hstate *hstate = hstate_file(vma->vm_file);
	unsigned long tsize = huge_page_shift(hstate) - 10;

	__flush_tlb_page(vma ? vma->vm_mm : NULL, vmaddr, tsize, 0);

}
