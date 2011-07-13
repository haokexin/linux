/*
 * This file contains low-level functions for performing various
 * types of TLB invalidations on various processors with no hash
 * table.
 *
 * This file implements the following functions for all no-hash
 * processors. Some aren't implemented for some variants. Some
 * are inline in tlbflush.h
 *
 *	- tlbil_va
 *	- tlbil_pid
 *	- tlbil_all
 *	- tlbivax_bcast (not yet)
 *
 * Copyright (C) 2009-2010 Wind River Systems, Inc.
 *
 * Partially rewritten by Cort Dougan (cort@cs.nmt.edu)
 * Paul Mackerras, Kumar Gala and Benjamin Herrenschmidt.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <asm/reg.h>
#include <asm/page.h>
#include <asm/cputable.h>
#include <asm/mmu.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>
#include <asm/processor.h>

#if defined(CONFIG_WRHV_E500) && !defined(CONFIG_PPC85xx_VT_MODE)
_GLOBAL(_tlbil_all)
_GLOBAL(_tlbil_pid)
_GLOBAL(__tlbil_va)
	lis     r0, VBI_SYS_tlb_flush@h
	ori     r0, r0, VBI_SYS_tlb_flush@l
	sc
	blr

#elif defined(CONFIG_FSL_BOOKE) && defined(CONFIG_PPC85xx_VT_MODE)
/*
 * FSL BookE implementations on Wind River HY.
 *
 * Since feature sections are using _SECTION_ELSE we need
 * to have the larger code path before the _SECTION_ELSE
 */

#define TLBWE_CODE	0x7C0007A4
#define TLBSX_CODE	0x7c005724

/*
 * Flush MMU TLB on the local processor
 */
_GLOBAL(_tlbil_all)
BEGIN_MMU_FTR_SECTION
	li	r3,(MMUCSR0_TLBFI)@l
	mtspr	SPRN_MMUCSR0, r3
1:
	mfspr	r3,SPRN_MMUCSR0
	andi.	r3,r3,MMUCSR0_TLBFI@l
	bne	1b
MMU_FTR_SECTION_ELSE
	PPC_TLBILX_ALL(0,0)
ALT_MMU_FTR_SECTION_END_IFCLR(MMU_FTR_USE_TLBILX)
	msync
	isync
	blr

_GLOBAL(_tlbil_pid)
BEGIN_MMU_FTR_SECTION
	slwi	r3,r3,16
	mfmsr	r10
	wrteei	0
	mfspr	r4,SPRN_MAS6	/* save MAS6 */
	mtspr	SPRN_MAS6,r3
	PPC_TLBILX_PID(0,0)
	mtspr	SPRN_MAS6,r4	/* restore MAS6 */
	wrtee	r10
	/* When enable EE we check if the pending interrupt 
	 * should be handled.
	 */
	rlwinm.	r10,r10,0,16,16;	/* test EE bit */
	beq	2f
	mr	r10,r0
	lis	r0,VBI_SYS_int_enable@h;
	ori	r0,r0,VBI_SYS_int_enable@l;
	sc	1
	mr	r0,r10
2:
MMU_FTR_SECTION_ELSE
	li	r3,(MMUCSR0_TLBFI)@l
	mtspr	SPRN_MMUCSR0, r3
1:
	mfspr	r3,SPRN_MMUCSR0
	andi.	r3,r3,MMUCSR0_TLBFI@l
	bne	1b
ALT_MMU_FTR_SECTION_END_IFSET(MMU_FTR_USE_TLBILX)
	msync
	isync
	blr

/*
 * Flush MMU TLB for a particular address, but only on the local processor
 * (no broadcast)
 */
_GLOBAL(__tlbil_va)
	mfmsr	r10
	wrteei	0
	
	/* Save r3 and r4 */
	mr	r11,r3
	mr	r12,r4

	/* Invalidate AS = 1 */
	slwi	r4,r4,16
	ori	r4,r4,(MAS6_ISIZE(BOOK3E_PAGESZ_4K))@l
	ori	r4,r4,MAS6_SAS
	mtspr	SPRN_MAS6,r4		/* assume AS=1 for now */

inv:
BEGIN_MMU_FTR_SECTION
	/* The HY will check which privileged instruction trap privileged
	 * exception via r3.
	 */
	mr	r4,r3
	lis	r3,TLBSX_CODE@h
	ori	r3,r3,TLBSX_CODE@l
	tlbsx	0,r4
	mr	r3,r4
	mfspr	r4,SPRN_MAS1		/* check valid */
	andis.	r3,r4,MAS1_VALID@h
	beq	1f
	rlwinm	r4,r4,0,1,31
	mtspr	SPRN_MAS1,r4

	/* The HY will check which privileged instruction trap privileged 
	 * exception via r3. 
	 */
	mr	r4,r3
	lis	r3,TLBWE_CODE@h
	ori	r3,r3,TLBWE_CODE@l
	tlbwe
	mr	r3,r4
MMU_FTR_SECTION_ELSE
	PPC_TLBILX_VA(0,r3)
ALT_MMU_FTR_SECTION_END_IFCLR(MMU_FTR_USE_TLBILX)
	msync
	isync
	
	/* Check if already invalidated AS = 0 */
	mfspr	r5,SPRN_MAS6
	rlwinm.	r5,r5,0,31,31
	beq	1f

	/* Invalidate AS = 0 */
	mr	r3,r11
	mr	r4,r12
	slwi	r4,r4,16
	ori	r4,r4,(MAS6_ISIZE(BOOK3E_PAGESZ_4K))@l
	mtspr	SPRN_MAS6,r4		/* assume AS=0 for now */
	b	inv

1:	wrtee	r10
	/* When enable EE we check if the pending interrupt 
	 * should be handled. 
	 */
	rlwinm.	r10,r10,0,16,16;	/* test EE bit */       
	beq	2f
	mr	r10,r0
	lis	r0,VBI_SYS_int_enable@h;
	ori	r0,r0,VBI_SYS_int_enable@l;
	sc	1
	mr	r0,r10
2:
	blr
#else
#error Unsupported processor type !
#endif

#if defined(CONFIG_FSL_BOOKE)
/*
 * extern void loadcam_entry(unsigned int index)
 *
 * Load TLBCAM[index] entry in to the L2 CAM MMU
 */
_GLOBAL(loadcam_entry)
	LOAD_REG_ADDR(r4, TLBCAM)
	mulli	r5,r3,TLBCAM_SIZE
	add	r3,r5,r4
	lwz	r4,TLBCAM_MAS0(r3)
	mtspr	SPRN_MAS0,r4
	lwz	r4,TLBCAM_MAS1(r3)
	mtspr	SPRN_MAS1,r4
	PPC_LL	r4,TLBCAM_MAS2(r3)
	mtspr	SPRN_MAS2,r4
	lwz	r4,TLBCAM_MAS3(r3)
	mtspr	SPRN_MAS3,r4
BEGIN_MMU_FTR_SECTION
	lwz	r4,TLBCAM_MAS7(r3)
	mtspr	SPRN_MAS7,r4
END_MMU_FTR_SECTION_IFSET(MMU_FTR_BIG_PHYS)
	isync
	tlbwe
	isync
	blr
#endif
