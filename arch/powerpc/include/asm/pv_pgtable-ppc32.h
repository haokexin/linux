#ifndef _ASM_PV_DEF_PGTABLE_PPC32_H
#define _ASM_PV_DEF_PGTABLE_PPC32_H

#include <asm-generic/pgtable-nopmd.h>

#ifdef CONFIG_WRHV
#include <vbi/vmmu.h>
#include <vbi/interface.h>
/*
 * refer to include/sys/vmmu.h on what format the hypervisor expects
 * the guest OS software page table to be
 */
/*
 * need extra care for _PAGE_SPECIAL -- Liang Li
 */
#define _PAGE_SPECIAL		0x0
#define _PAGE_PRESENT		VMMU_PROT_SUPV_READ
#define _PAGE_USER		(VMMU_PROT_USER_READ|VMMU_PROT_USER_EXECUTE)
#define _PAGE_FILE      	_PAGE_USER
#define _PAGE_ACCESSED	        VMMU_PROT_SUPV_WRITE
#define _PAGE_HWWRITE   	VMMU_PROT_USER_WRITE
#define	 _PAGE_RW		(VMMU_PROT_SUPV_EXECUTE|VMMU_PROT_USER_WRITE)
#define _PAGE_HWEXEC    	VMMU_PROT_USER_EXECUTE

#define _PAGE_ENDIAN		VMMU_CACHE_LE
#define _PAGE_GUARDED		VMMU_CACHE_GUARDED
#define _PAGE_COHERENT		VMMU_CACHE_COHERENT
#define _PAGE_NO_CACHE		VMMU_CACHE_INHIBIT
#define _PAGE_WRITETHRU		VMMU_CACHE_WRITETHROUGH

#define _PAGE_DIRTY		VMMU_PTE_CHG_MASK

#define _PAGE_EXEC		VMMU_CACHE_COHERENT

/* The flag, _PAGE_FILE_, bit0, should be used to mask this pte 
 * as nonlinear file mapping.
 */
#define _PTE_NONE_MASK		0xffffffff00000ffdULL

/* See figure 1 located in include/vbi/vmmu.h for definition
   of RPN (Real Page Number) and the associated shift value*/

/* MAX_SWAPFILES_SHIFT is defined in include/linux/swap.h as
   5 , but because the HV only gives us bits 0-19 of the second
   word for each PTE, that only leaves us with 15 bits.  (32MB)
   Given this limitation we need to reduce the number of swaptypes
   from 5 bits to 3 therefor giving us access to 18 bits. (256MB)
*/

/* Encode and de-code a swap entry */
/* These values need to match the native version found in
   arch/powerpc/include/asm/pgtable-ppc32.h */
#define __swp_type(entry)	((entry).val & 0x1f)

/* When running under the HV we have a PTE therefor inorder
   to support large amounts of swap we need to reduce the
   maximum number of swapfile entries */
#define __swp_offset(entry)	((entry).val >> 2)
#define __swp_entry(type, offset) ((swp_entry_t){(type)|((offset)<<2)})
/* Unfortunately the hypervisor's VMMU maps PTE's differently then
   that of native */
#define __pte_to_swp_entry(pte)	((swp_entry_t){pte_val(pte) >> VMMU_RPN_SHIFT})
#define __swp_entry_to_pte(x)	((pte_t) { (x).val << VMMU_RPN_SHIFT })

#define pte_to_pgoff(pte)	(pte_val(pte) >> VMMU_RPN_SHIFT)
#define pgoff_to_pte(off)	((pte_t) {((off) << VMMU_RPN_SHIFT)|_PAGE_FILE})
#define PTE_FILE_MAX_BITS	(BITS_PER_LONG - VMMU_RPN_SHIFT)

/* based on hypervisor VMMU_LEVEL_1_DESC definition */
#define _PMD_PRESENT		0x00000001   /* big endian */
#define _PMD_PRESENT_MASK	(_PMD_PRESENT)
#define _PMD_BAD		(~PAGE_MASK & ~0x03)

#ifndef CONFIG_WRHV_E500
#define _PAGE_BASE		(_PAGE_PRESENT | _PAGE_ACCESSED | VMMU_PROT_USER_READ)
#endif

#define PFN_SIZE		(1UL << PFN_SHIFT_OFFSET)
#define PFN_MASK		(~(PFN_SIZE-1))
#define pte_to_pa(x)		(pte_val(x) & PFN_MASK)
#define pte_to_prot(x)		(pte_val(x) & (PFN_SIZE-1))

/*
 * Some bits are only used on some cpu families...
 */
#ifndef _PAGE_HASHPTE
#define _PAGE_HASHPTE   0
#endif
#elif defined(CONFIG_FSL_BOOKE)
#include <asm/pte-fsl-booke.h>
#endif
#endif /* _ASM_PV_DEF_PGTABLE_PPC32_H */
