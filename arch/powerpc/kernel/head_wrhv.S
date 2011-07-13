/*
 * Kernel execution entry point code.
 *
 *    Copyright (c) 1995-1996 Gary Thomas <gdt@linuxppc.org>
 *	Initial PowerPC version.
 *    Copyright (c) 1996 Cort Dougan <cort@cs.nmt.edu>
 *	Rewritten for PReP
 *    Copyright (c) 1996 Paul Mackerras <paulus@cs.anu.edu.au>
 *	Low-level exception handers, MMU support, and rewrite.
 *    Copyright (c) 1997 Dan Malek <dmalek@jlc.net>
 *	PowerPC 8xx modifications.
 *    Copyright (c) 1998-1999 TiVo, Inc.
 *	PowerPC 403GCX modifications.
 *    Copyright (c) 1999 Grant Erickson <grant@lcse.umn.edu>
 *	PowerPC 403GCX/405GP modifications.
 *    Copyright 2000 MontaVista Software Inc.
 *	PPC405 modifications
 *	PowerPC 403GCX/405GP modifications.
 *	Author: MontaVista Software, Inc.
 *		frank_rowand@mvista.com or source@mvista.com
 *		debbie_chu@mvista.com
 *    Copyright 2002-2004 MontaVista Software, Inc.
 *	PowerPC 44x support, Matt Porter <mporter@kernel.crashing.org>
 *    Copyright 2004, 2008-2009 Freescale Semiconductor, Inc
 *	PowerPC e500 modifications, Kumar Gala <galak@kernel.crashing.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/threads.h>
#include <asm/processor.h>
#include <asm/page.h>
#include <asm/mmu.h>
#include <asm/pgtable.h>
#include <asm/cputable.h>
#include <asm/thread_info.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>
#include <asm/cache.h>
#include "head_booke.h"
#include <asm/ppc-opcode.h>

#include "head_wrhv.h"
#include <vbi/interface.h>
#include <vbi/vmmu.h>
#include <vbi/syscalls.h>

/* As with the other PowerPC ports, it is expected that when code
 * execution begins here, the following registers contain valid, yet
 * optional, information:
 *
 *   r3 - Board info structure pointer (DRAM, frequency, MAC address, etc.)
 *   r4 - Starting address of the init RAM disk
 *   r5 - Ending address of the init RAM disk
 *   r6 - Start of kernel command line string (e.g. "mem=128")
 *   r7 - End of kernel command line string
 *
 */
	__HEAD
_ENTRY(_stext);
_ENTRY(_start);
	/*
	 * Reserve a word at a fixed location to store the address
	 * of abatron_pteptrs
	 */
	nop
/*
 * Save parameters we are passed
 */
	mr	r31,r3
	mr	r30,r4
	mr	r29,r5
	mr	r28,r6
	mr	r27,r7
	li	r25,0		/* phys kernel start (low) */
	li	r24,0		/* CPU number */
	li	r23,0		/* phys kernel start (high) */

/* We try to not make any assumptions about how the boot loader
 * setup or used the TLBs.  We invalidate all mappings from the
 * boot loader and load a single entry in TLB1[0] to map the
 * first 64M of kernel memory.  Any boot info passed from the
 * bootloader needs to live in this first 64M.
 *
 * Requirement on bootloader:
 *  - The page we're executing in needs to reside in TLB1 and
 *    have IPROT=1.  If not an invalidate broadcast could
 *    evict the entry we're currently executing in.
 *
 *  r3 = Index of TLB1 were executing in
 *  r4 = Current MSR[IS]
 *  r5 = Index of TLB1 temp mapping
 *
 * Later in mapin_ram we will correctly map lowmem, and resize TLB1[0]
 * if needed
 */

_ENTRY(__early_start)

	/* Establish the interrupt vector base */
	lis	r0,VBI_SYS_hyIoctl@h
	ori     r0,r0,VBI_SYS_hyIoctl@l
	lis	r3,VBI_HYIOCTL_EXCBASE@h
	ori	r3,r3,VBI_HYIOCTL_EXCBASE@l
	lis	r4,_start@h
	ori	r4,r4,_start@l
	sc

#ifdef CONFIG_SMP
	/* Check to see if we're the second processor, and jump
	 * to the secondary_start code if so
	 */
	lis	r24, 0xF0000000@h
	ori	r24, r24, 0xF0000000@l
	lwz	r24, WRHV_COREID_OFFSET(r24)
	cmpwi	r24,0
	bne	__secondary_start
#endif


	/* ptr to current */
	lis	r2,init_task@h
	ori	r2,r2,init_task@l

	li	r4,1
	WRHV_SET_SUP_MODE(r1,r4)
	/* ptr to current thread */
	addi	r4,r2,THREAD	/* init task's THREAD */
	WRHV_MTSPRG3(r4,r1)

	/* stack */
	lis	r1,init_thread_union@h
	ori	r1,r1,init_thread_union@l
	li	r0,0
	stwu	r0,THREAD_SIZE-STACK_FRAME_OVERHEAD(r1)

	/*
	 * Move the calls to SET_IVOR(...) out of line, since they will
	 * push us past 0x100 and hence CriticalInput will align up at
	 * 0x200 instead, which breaks MILS.  Also, since SET_IVOR uses
	 * blr itself, we use two b, instead of a bl + blr combo.
	 */
	b	set_ivors
done_ivors:
	bl	early_init

#ifdef CONFIG_RELOCATABLE
	lis	r3,kernstart_addr@ha
	la	r3,kernstart_addr@l(r3)
#ifdef CONFIG_PHYS_64BIT
	stw	r23,0(r3)
	stw	r25,4(r3)
#else
	stw	r25,0(r3)
#endif
#endif

/*
 * Decide what sort of machine this is and initialize the MMU.
 */
	mr	r3,r31
	mr	r4,r30
	mr	r5,r29
	mr	r6,r28
	mr	r7,r27
	bl	machine_init
	bl	MMU_init

	/* Let's move on */
	b	start_kernel	/* change context and jump to start_kernel */

/* Macros to hide the PTE size differences
 *
 * FIND_PTE -- walks the page tables given EA & pgdir pointer
 *   r10 -- EA of fault
 *   r11 -- PGDIR pointer
 *   r12 -- free
 *   label 2: is the bailout case
 *
 * if we find the pte (fall through):
 *   r11 is low pte word
 *   r12 is pointer to the pte
 */
#if defined(CONFIG_PTE_64BIT) || defined(CONFIG_WRHV)
#define FIND_PTE	\
	rlwinm	r12, r10, 13, 19, 29;	/* Compute pgdir/pmd offset */	\
	lwzx	r11, r12, r11;		/* Get pgd/pmd entry */		\
	rlwinm.	r12, r11, 0, 0, 20;	/* Extract pt base address */	\
	rlwimi	r12, r10, 23, 20, 28;	/* Compute pte address */

#define LOAD_PTE \
	lwz	r11, 4(r12);

#ifdef CONFIG_SMP
#define LWARX_PTE \
	li	r11, 4;
	lwarx	r11, r11, r12;		/* lwarx pte */

#define STWCX_PTE \
	addi	r12, r12, 4;	\
	stwcx.	r11, 0, r12;	\
	addi	r12, r12, -4;
#else
#define LWARX_PTE \
	lwz	r11, 4(r12)

#define STWCX_PTE \
	stw	r11, 4(r12)
#endif

#else
#define FIND_PTE	\
	rlwimi	r11, r10, 12, 20, 29;	/* Create L1 (pgdir/pmd) address */	\
	lwz	r11, 0(r11);		/* Get L1 entry */			\
	rlwinm.	r12, r11, 0, 0, 19;	/* Extract L2 (pte) base address */	\
	beq	2f;			/* Bail if no table */			\
	rlwimi	r12, r10, 22, 20, 29;	/* Compute PTE address */		\
	lwz	r11, 0(r12);		/* Get Linux PTE */
#endif

/*
 * Interrupt vector entry code
 *
 * The Book E MMUs are always on so we don't need to handle
 * interrupts in real mode as with previous PPC processors. In
 * this case we handle interrupts in the kernel virtual address
 * space.
 *
 * Interrupt vectors are dynamically placed relative to the
 * interrupt prefix as determined by the address of interrupt_base.
 * The interrupt vectors offsets are programmed using the labels
 * for each interrupt vector entry.
 *
 * Interrupt vectors must be aligned on a 16 byte boundary.
 * We align on a 32 byte cache line boundary for good measure.
 */
interrupt_base:
	/* Critical Input Interrupt */
	CRITICAL_EXCEPTION(0x0100, CriticalInput, unknown_exception)

	/* Machine Check Interrupt */
	MCHECK_EXCEPTION(0x0200, MachineCheck, machine_check_exception)

	/* Data Storage Interrupt */
	START_EXCEPTION(DataStorage)

       /*    only  r3, r4, CR are saved in vb_status */
        lis     r4,wr_control@ha
        lwz     r4,wr_control@l(r4)
        stw     r10,VB_CONTROL_R10(r4)
        stw     r11,VB_CONTROL_R11(r4)
        stw     r12,VB_CONTROL_R12(r4)
        stw     r13,VB_CONTROL_R13(r4)

        /*
         * Check if it was a store fault, if not then bail
         * because a user tried to access a kernel or
         * read-protected page.  Otherwise, get the
         * offending address and handle it.
         */
        lis     r11,wr_status@ha
        lwz     r11,wr_status@l(r11)
        lwz     r10,VB_STATUS_ESR(r11)
        srwi    r10,r10,16    /* get the hibit of ESR_ST */
        andis.  r10, r10, ESR_ST@h
        beq     2f

        lwz     r10,VB_STATUS_DEAR(r11)

        /* If we are faulting a kernel address, we have to use the
         * kernel page tables.
         */
        lis     r11, PAGE_OFFSET@h
        cmplw   0, r10, r11
        bge     2f

        /* Get the PGD for the current thread */
3:
        WRHV_MFSPRG3(r11)
        lwz     r11,PGDIR(r11)
4:
	FIND_PTE

	beq     2f      /* Bail if there's no entry */
5:
        LOAD_PTE

        /* Are _PAGE_USER & _PAGE_RW set & _PAGE_HWWRITE not? */
        andi.   r13, r11, _PAGE_RW|_PAGE_USER|_PAGE_HWWRITE
        cmpwi   0, r13, _PAGE_RW|_PAGE_USER
        bne     2f                      /* Bail if not */

        /* update search PID in MAS6, AS = 0 */
        mfspr   r13, SPRN_PID0
        slwi    r13, r13, 16
        mtspr   SPRN_MAS6, r13

        /* find the TLB index that caused the fault. */
        tlbsx   0, r10

#if defined(CONFIG_SMP)
        /*
         * It's possible another processor kicked out the entry
         * before we did our tlbsx, so check if we hit
         */
        mfspr   r13, SPRN_MAS1
        rlwinm. r13,r13, 0, 0, 0;       /* Check the Valid bit */
        beq     2f
#endif /* CONFIG_SMP */

        /*
         * MAS2 not updated as the entry does exist in the tlb, this
         * fault taken to detect state transition (eg: COW -> DIRTY)
         */
        andi.   r13, r11, _PAGE_HWEXEC
        rlwimi  r13, r13, 31, 27, 27    /* SX <- _PAGE_HWEXEC */
        ori     r13, r13, (MAS3_UW|MAS3_SW|MAS3_UR|MAS3_SR)@l /* set static perms */

        /* only update the perm bits, assume the RPN is fine */
        mfspr   r10, SPRN_MAS3
        rlwimi  r10, r13, 0, 20, 31
        mtspr   SPRN_MAS3,r10
        tlbwe

#if defined(CONFIG_SMP)
        mr      r13, r11
        LWARX_PTE
        cmpw    r13, r11
        bne-    7f
#endif

        /* Update 'changed'. */
        ori     r11, r11, _PAGE_DIRTY|_PAGE_ACCESSED|_PAGE_HWWRITE
        STWCX_PTE       /* r11 and r12 must be PTE and &PTE */

#if defined(CONFIG_SMP)
        /*
         * If the stwcx. failed, we invalidate the entry we just wrote,
         * and start over
         */
        beq+    6f
7:      mfspr   r13, SPRN_MAS1
        rlwinm  r13, r13, 0, 1, 31      /* Clear Valid bit */
        mtspr   SPRN_MAS1, r13

        tlbwe

        b       5b              /* Try again */
#endif  /* CONFIG_SMP */
6:

        /* Done...restore registers and get out of here.  */
        mfspr   r11, SPRN_SPRG7R
        mtcr    r11
        mfspr   r13, SPRN_SPRG5R
        mfspr   r12, SPRN_SPRG4R
        mfspr   r11, SPRN_SPRG1
        mfspr   r10, SPRN_SPRG0
        rfi                     /* Force context change */

2:
#if defined(CONFIG_SMP)
        /* Clear the reservation */
        lis     r11, dummy_stwcx@h
        ori     r11,r11, dummy_stwcx@l
        stwcx.  r11, 0, r11
#endif

        /*
         * The bailout.  Restore registers to pre-exception conditions
         * and call the heavyweights to help us out.
         */
        lis     r11,wr_control@ha
        lwz     r11,wr_control@l(r11)
        lwz     r13,VB_CONTROL_R13(r11)
        lwz     r12,VB_CONTROL_R12(r11)
        lwz     r10,VB_CONTROL_R10(r11)
        lwz     r11,VB_CONTROL_R11(r11)
        b       data_access

        /* Instruction Storage Interrupt */
        INSTRUCTION_STORAGE_EXCEPTION

	/* External Input Interrupt */
	EXCEPTION(0x0500, ExternalInput, do_IRQ, EXC_XFER_LITE)

	/* Alignment Interrupt */
	ALIGNMENT_EXCEPTION

	/* Program Interrupt */
	PROGRAM_EXCEPTION

	/* Floating Point Unavailable Interrupt */
#ifdef CONFIG_PPC_FPU
	EXCEPTION(0x0800, FloatingPointUnavailableErrata, program_check_exception, EXC_XFER_EE)
#else
	EXCEPTION(0x0800, FloatingPointUnavailable, unknown_exception, EXC_XFER_EE)
#endif

	/* System Call Interrupt */
	START_EXCEPTION(SystemCall)
	NORMAL_EXCEPTION_PROLOG
	EXC_XFER_EE_LITE(0x0c00, DoSyscall)

	/* Auxillary Processor Unavailable Interrupt */
	EXCEPTION(0x2900, AuxillaryProcessorUnavailable, unknown_exception, EXC_XFER_EE)

	/* Decrementer Interrupt */
	DECREMENTER_EXCEPTION

	/* Fixed Internal Timer Interrupt */
	/* TODO: Add FIT support */
	EXCEPTION(0x3100, FixedIntervalTimer, unknown_exception, EXC_XFER_EE)

	/* Watchdog Timer Interrupt */
#ifdef CONFIG_BOOKE_WDT
	CRITICAL_EXCEPTION(0x3200, WatchdogTimer, WatchdogException)
#else
	CRITICAL_EXCEPTION(0x3200, WatchdogTimer, unknown_exception)
#endif

	/* Data TLB Error Interrupt */
	START_EXCEPTION(DataTLBError)
	b	data_access

        /* Instruction TLB Error Interrupt */
        /*
         * Nearly the same as above, except we get our
         * information from different registers and bailout
         * to a different point.
         */
        START_EXCEPTION(InstructionTLBError)
        b       InstructionStorage

	/* Debug Interrupt */
	DEBUG_DEBUG_EXCEPTION
#ifdef CONFIG_SPE
	/* SPE Unavailable */
	START_EXCEPTION(SPEUnavailable)
	NORMAL_EXCEPTION_PROLOG
	beq	load_up_spe
	addi	r3,r1,STACK_FRAME_OVERHEAD
	EXC_XFER_EE_LITE(0x2010, KernelSPE)
#else
	EXCEPTION(0x2020, SPEUnavailable, unknown_exception, EXC_XFER_EE)
#endif /* CONFIG_SPE */

	/* SPE Floating Point Data */
#ifdef CONFIG_SPE
	EXCEPTION(0x2030, SPEFloatingPointData, SPEFloatingPointException, EXC_XFER_EE);
	/* SPE Floating Point Round */
	EXCEPTION(0x2050, SPEFloatingPointRound, SPEFloatingPointRoundException, EXC_XFER_EE)
#else
	EXCEPTION(0x2040, SPEFloatingPointData, unknown_exception, EXC_XFER_EE)
	EXCEPTION(0x2050, SPEFloatingPointRound, unknown_exception, EXC_XFER_EE)
#endif /* CONFIG_SPE */

	/* Performance Monitor */
	EXCEPTION(0x2060, PerformanceMonitor, performance_monitor_exception, EXC_XFER_STD)

	EXCEPTION(0x2070, Doorbell, doorbell_exception, EXC_XFER_STD)

	CRITICAL_EXCEPTION(0x2080, CriticalDoorbell, unknown_exception)

	/* Debug Crit Interrupt */
	DEBUG_CRIT_EXCEPTION

/*
 * Local functions
 */

/*
 * Both the instruction and data TLB miss get to this
 * point to load the TLB.
 *	r10 - available to use
 *	r11 - TLB (info from Linux PTE)
 *	r12 - available to use
 *	r13 - upper bits of PTE (if PTE_64BIT) or available to use
 *	CR5 - results of addr >= PAGE_OFFSET
 *	MAS0, MAS1 - loaded with proper value when we get here
 *	MAS2, MAS3 - will need additional info from Linux PTE
 *	Upon exit, we reload everything and RFI.
 */
finish_tlb_load:
	/*
	 * We set execute, because we don't have the granularity to
	 * properly set this at the page level (Linux problem).
	 * Many of these bits are software only.  Bits we don't set
	 * here we (properly should) assume have the appropriate value.
	 */

	mfspr	r12, SPRN_MAS2
#if defined(CONFIG_PTE_64BIT) || defined(CONFIG_WRHV)
	rlwimi	r12, r11, 32-19, 27, 31	/* extract WIMGE from pte */
#else
	rlwimi	r12, r11, 26, 27, 31	/* extract WIMGE from pte */
#endif
	mtspr	SPRN_MAS2, r12

#if defined(CONFIG_PTE_64BIT) || defined(CONFIG_WRHV)
	rlwinm	r12, r11, 32-2, 26, 31	/* Move in perm bits */
	andi.	r10, r11, _PAGE_DIRTY
	bne	1f
	li	r10, MAS3_SW | MAS3_UW
	andc	r12, r12, r10
1:	rlwimi	r12, r13, 20, 0, 11	/* grab RPN[32:43] */
	rlwimi	r12, r11, 20, 12, 19	/* grab RPN[44:51] */
	mtspr	SPRN_MAS3, r12
BEGIN_MMU_FTR_SECTION
	srwi	r10, r13, 12		/* grab RPN[12:31] */
	mtspr	SPRN_MAS7, r10
END_MMU_FTR_SECTION_IFSET(MMU_FTR_BIG_PHYS)
#else
	li	r10, (_PAGE_EXEC | _PAGE_PRESENT)
	rlwimi	r10, r11, 31, 29, 29	/* extract _PAGE_DIRTY into SW */
	and	r12, r11, r10
	andi.	r10, r11, _PAGE_USER	/* Test for _PAGE_USER */
	slwi	r10, r12, 1
	or	r10, r10, r12
	iseleq	r12, r12, r10
	rlwimi	r11, r12, 0, 20, 31	/* Extract RPN from PTE and merge with perms */
	mtspr	SPRN_MAS3, r11
#endif

	tlbwe

	/* Done...restore registers and get out of here.  */
	mfspr	r11, SPRN_SPRG7R
	mtcr	r11
	mfspr	r13, SPRN_SPRG5R
	mfspr	r12, SPRN_SPRG4R
	mfspr   r11, SPRN_SPRG1
	mfspr   r10, SPRN_SPRG0
	rfi					/* Force context change */


        /*
         * Data TLB exceptions will bail out to this point
         * if they can't resolve the lightweight TLB fault.
         */
data_access:
        NORMAL_EXCEPTION_PROLOG
        lis     r6,wr_status@ha
        lwz     r6,wr_status@l(r6)
        lwz     r5, VB_STATUS_ESR(r6)
        stw     r5,_ESR(r11)
        lwz     r4, VB_STATUS_DEAR(r6)
        andis.  r10,r5,(ESR_ILK|ESR_DLK)@h
        bne     1f
        EXC_XFER_EE_LITE(0x0300, handle_page_fault)
1:
        addi    r3,r1,STACK_FRAME_OVERHEAD
        EXC_XFER_EE_LITE(0x0300, CacheLockingException)



#ifdef CONFIG_SPE
/* Note that the SPE support is closely modeled after the AltiVec
 * support.  Changes to one are likely to be applicable to the
 * other!  */
load_up_spe:
/*
 * Disable SPE for the task which had SPE previously,
 * and save its SPE registers in its thread_struct.
 * Enables SPE for use in the kernel on return.
 * On SMP we know the SPE units are free, since we give it up every
 * switch.  -- Kumar
 */
	stw	r2,GPR2(r11)
	stw	r12,_NIP(r11)
	stw	r9,_MSR(r11)
	mfctr	r12
	mfspr	r2,SPRN_XER
	stw	r12,_CTR(r11)
	stw	r2,_XER(r11)

	li r3, 1
	li r4,(SPEFSCR_FINVE | SPEFSCR_FDBZE | SPEFSCR_FUNFE | SPEFSCR_FOVFE)
	lis r0,VBI_SYS_spefscr_update@h
	ori r0,r0,VBI_SYS_spefscr_update@l
	sc

/*
 * For SMP, we don't do lazy SPE switching because it just gets too
 * horrendously complex, especially when a task switches from one CPU
 * to another.  Instead we call giveup_spe in switch_to.
 */
#ifndef CONFIG_SMP
	lis	r3,last_task_used_spe@ha
	lwz	r4,last_task_used_spe@l(r3)
	cmpi	0,r4,0
	beq	1f
	addi	r4,r4,THREAD	/* want THREAD of last_task_used_spe */
	SAVE_32EVRS(0,r10,r4)
	evxor	evr10, evr10, evr10	/* clear out evr10 */
	evmwumiaa evr10, evr10, evr10	/* evr10 <- ACC = 0 * 0 + ACC */
	li	r5,THREAD_ACC
	evstddx	evr10, r4, r5		/* save off accumulator */
	lwz	r5,PT_REGS(r4)
	lwz	r4,_MSR-STACK_FRAME_OVERHEAD(r5)
	lis	r10,MSR_SPE@h
	andc	r4,r4,r10	/* disable SPE for previous task */
	stw	r4,_MSR-STACK_FRAME_OVERHEAD(r5)
1:
#endif /* !CONFIG_SMP */
	WRHV_MFSPRG3(r5)		/* current task's THREAD (phys) */
	li	r4,1
	li	r10,THREAD_ACC
	stw	r4,THREAD_USED_SPE(r5)
	evlddx	evr4,r10,r5
	evmra	evr4,evr4
	REST_32EVRS(0,r10,r5)
#ifndef CONFIG_SMP
	subi	r4,r5,THREAD
	stw	r4,last_task_used_spe@l(r3)
#endif /* !CONFIG_SMP */

	/* restore registers and return */
	lis	r4,wr_control@ha
	lwz	r4,wr_control@l(r4)
	lwz	r0,GPR0(r1)
	stw	r0,VB_CONTROL_R0(r4)
	lwz	r2,GPR2(r1)
	lwz	r3,GPR3(r1)
	lwz	r6,GPR6(r1)
	lwz	r7,GPR7(r1)
	lwz	r8,GPR8(r1)
	lwz	r9,GPR9(r1)
	lwz	r10,GPR10(r1)
	lwz	r11,GPR11(r1)

	lis	r12,wr_status@ha
	lwz	r12,wr_status@l(r12)
	lwz	r5,VB_STATUS_OLD_INT_DISABLE(r12)
	stw	r5,VB_CONTROL_NEW_INT_DISABLE(r4)

	lwz	r0,_CCR(r1)
	stw	r0,VB_CONTROL_CR(r4)
	lwz	r0,_NIP(r1)
	stw	r0,VB_CONTROL_SRR0(r4)
	lwz	r0,_LINK(r1)
	mtlr	r0
	lwz	r0,_XER(r1)
	mtspr	SPRN_XER,r0
	lwz	r0,_CTR(r1)
	mtctr	r0
	lwz	r0,_MSR(r1)
	/* enable use of SPE after return */
	oris	r0,r0,MSR_SPE@h
	WRHV_LOAD_MSR(r0,r12,r5)
	lwz	r12,GPR12(r1)
	lwz	r5,GPR5(r1)

	lwz	r4,GPR4(r1)
	lwz	r1,GPR1(r1)
	lis	r0,VBI_SYS_ctx_load@h
	ori	r0,r0,VBI_SYS_ctx_load@l
	sc

/*
 * SPE unavailable trap from kernel - print a message, but let
 * the task use SPE in the kernel until it returns to user mode.
 */
KernelSPE:
	lwz	r3,_MSR(r1)
	oris	r3,r3,MSR_SPE@h
	stw	r3,_MSR(r1)	/* enable use of SPE after return */
#ifdef CONFIG_PRINTK
	lis	r3,87f@h
	ori	r3,r3,87f@l
	mr	r4,r2		/* current */
	lwz	r5,_NIP(r1)
	bl	printk
#endif
	b	ret_from_except
#ifdef CONFIG_PRINTK
87:	.string	"SPE used in kernel  (task=%p, pc=%x)  \n"
#endif
	.align	4,0

#endif /* CONFIG_SPE */

/*
 * Set the interrupt vector offset to the exec table
 * r3 - the interrupt vector number
 * r4 - the vector handler label
 */
set_exec_table:
	lis	r5,exec_table@h
	ori	r5,r5,exec_table@l
	slwi	r3,r3,2
	subis	r4,r4,PAGE_OFFSET@h
	stwx	r4,r3,r5
	blr

/*
 * Establish the interrupt vector offsets.  Since SET_IVOR
 * is itself using bl+blr into set_exec_table above, we just
 * use absolute branches to labels to get here and back.  It
 * is kept out-of-line so that CriticalInput can be at 0x100.
 */
set_ivors:
	SET_IVOR(0,  CriticalInput);
	SET_IVOR(1,  MachineCheck);
	SET_IVOR(2,  DataStorage);
	SET_IVOR(3,  InstructionStorage);
	SET_IVOR(4,  ExternalInput);
	SET_IVOR(5,  Alignment);
	SET_IVOR(6,  Program);
	SET_IVOR(7,  FloatingPointUnavailable);
	SET_IVOR(8,  SystemCall);
	SET_IVOR(9,  AuxillaryProcessorUnavailable);
	SET_IVOR(10, Decrementer);
	SET_IVOR(11, FixedIntervalTimer);
	SET_IVOR(12, WatchdogTimer);
	SET_IVOR(13, DataTLBError);
	SET_IVOR(14, InstructionTLBError);
	SET_IVOR(15, DebugDebug);
	SET_IVOR(32, SPEUnavailable);
	SET_IVOR(33, SPEFloatingPointData);
	SET_IVOR(34, SPEFloatingPointRound);
	SET_IVOR(35, PerformanceMonitor);
	SET_IVOR(36, doorbell_exception);
	SET_IVOR(37, CriticalDoorbell);
	b done_ivors

/*
 * Global functions
 */

/*
 * extern void giveup_altivec(struct task_struct *prev)
 *
 * The e500 core does not have an AltiVec unit.
 */
_GLOBAL(giveup_altivec)
	blr

#if defined(CONFIG_SPE) && defined(CONFIG_WRHV)
/*
 * extern void giveup_spe(struct task_struct *prev)
 *
 */
_GLOBAL(giveup_spe)
	stwu	r1,-INT_FRAME_SIZE(r1)
	mflr	r0
	stw	r0,INT_FRAME_SIZE+4(r1)
	SAVE_GPR(3,r1)

	li	r3, 1			/* enable SPE */
	li	r4,(SPEFSCR_FINVE | SPEFSCR_FDBZE | SPEFSCR_FUNFE | \
			SPEFSCR_FOVFE)
	lis	r0,VBI_SYS_spefscr_update@h
	ori	r0,r0,VBI_SYS_spefscr_update@l
	sc

	REST_GPR(3,r1)
	addi	r1,r1,INT_FRAME_SIZE

	cmpi	0,r3,0
	beqlr-				/* if no previous owner, done */
	addi	r3,r3,THREAD		/* want THREAD of task */
	lwz	r5,PT_REGS(r3)
	cmpi	0,r5,0
	SAVE_32EVRS(0, r4, r3)
	evxor	evr6, evr6, evr6	/* clear out evr6 */
	evmwumiaa evr6, evr6, evr6	/* evr6 <- ACC = 0 * 0 + ACC */
	li	r4,THREAD_ACC
	evstddx	evr6, r4, r3		/* save off accumulator */

	lis	r6,wr_status@ha
	lwz     r6,wr_status@l(r6)
	lwz	r6,VB_STATUS_SPEFSCR(r6)
	stw	r6,THREAD_SPEFSCR(r3)	/* save spefscr register value */
	beq	1f
	lwz	r4,_MSR-STACK_FRAME_OVERHEAD(r5)
	lis	r3,MSR_SPE@h
	andc	r4,r4,r3		/* disable SPE for previous task */
	stw	r4,_MSR-STACK_FRAME_OVERHEAD(r5)
1:
#ifndef CONFIG_SMP
	li	r5,0
	lis	r4,last_task_used_spe@ha
	stw	r5,last_task_used_spe@l(r4)
#endif /* !CONFIG_SMP */
	blr
#endif /* CONFIG_SPE */

/*
 * extern void giveup_fpu(struct task_struct *prev)
 *
 * Not all FSL Book-E cores have an FPU
 */
#ifndef CONFIG_PPC_FPU
_GLOBAL(giveup_fpu)
	blr
#endif

/*
 * extern void abort(void)
 *
 * At present, this routine just applies a system reset.
 */
_GLOBAL(abort)
	li	r13,0
	mtspr	SPRN_DBCR0,r13		/* disable all debug events */
	isync
	mfmsr	r13
	ori	r13,r13,MSR_DE@l	/* Enable Debug Events */
	mtmsr	r13
	isync
	mfspr	r13,SPRN_DBCR0
	lis	r13,(DBCR0_IDM|DBCR0_RST_CHIP)@h
	mtspr	SPRN_DBCR0,r13
	isync

#ifndef CONFIG_WRHV_E500
_GLOBAL(set_context)

#ifdef CONFIG_BDI_SWITCH
	/* Context switch the PTE pointer for the Abatron BDI2000.
	 * The PGDIR is the second parameter.
	 */
	lis	r5, abatron_pteptrs@h
	ori	r5, r5, abatron_pteptrs@l
	stw	r4, 0x4(r5)
#endif
	mtspr	SPRN_PID,r3
	isync			/* Force context change */
	blr
#endif

_GLOBAL(flush_dcache_L1)
	mfspr	r3,SPRN_L1CFG0

	rlwinm	r5,r3,9,3	/* Extract cache block size */
	twlgti	r5,1		/* Only 32 and 64 byte cache blocks
				 * are currently defined.
				 */
	li	r4,32
	subfic	r6,r5,2		/* r6 = log2(1KiB / cache block size) -
				 *      log2(number of ways)
				 */
	slw	r5,r4,r5	/* r5 = cache block size */

	rlwinm	r7,r3,0,0xff	/* Extract number of KiB in the cache */
	mulli	r7,r7,13	/* An 8-way cache will require 13
				 * loads per set.
				 */
	slw	r7,r7,r6

	/* save off HID0 and set DCFA */
	mfspr	r8,SPRN_HID0
	ori	r9,r8,HID0_DCFA@l
	mtspr	SPRN_HID0,r9
	isync

	lis	r4,KERNELBASE@h
	mtctr	r7

1:	lwz	r3,0(r4)	/* Load... */
	add	r4,r4,r5
	bdnz	1b

	msync
	lis	r4,KERNELBASE@h
	mtctr	r7

1:	dcbf	0,r4		/* ...and flush. */
	add	r4,r4,r5
	bdnz	1b

	/* restore HID0 */
	mtspr	SPRN_HID0,r8
	isync

	blr

#ifdef CONFIG_SMP
/* When we get here, r24 needs to hold the CPU # */
	.globl __secondary_start
__secondary_start:
	lis	r3,__secondary_hold_acknowledge@h
	ori	r3,r3,__secondary_hold_acknowledge@l
	stw	r24,0(r3)

	li	r3,0
	mr	r4,r24		/* Why? */
	bl	call_setup_cpu

	/* get current_thread_info and current */
	lis	r1,secondary_ti@ha
	lwz	r1,secondary_ti@l(r1)
	lwz	r2,TI_TASK(r1)

	/* stack */
	addi	r1,r1,THREAD_SIZE-STACK_FRAME_OVERHEAD
	li	r0,0
	stw	r0,0(r1)

	li	r4,1
	WRHV_SET_SUP_MODE(r3,r4)

	/* ptr to current thread */
	addi	r4,r2,THREAD	/* address of our thread_struct */
	WRHV_MTSPRG3(r4,r3)

	/* Jump to start_secondary */
	b	wrhv_start_secondary
	sync

	.globl __secondary_hold_acknowledge
__secondary_hold_acknowledge:
	.long	-1
#endif

/*
 * We put a few things here that have to be page-aligned. This stuff
 * goes at the beginning of the data segment, which is page-aligned.
 */
	.data
	.align	12
	.globl	sdata
sdata:
	.globl	empty_zero_page
empty_zero_page:
	.space	4096
	.globl	swapper_pg_dir
swapper_pg_dir:
	.space	8192
/*
 * We need a place to stwcx. to when we want to clear a reservation
 * without knowing where the original was.
 */
	.globl	dummy_stwcx
dummy_stwcx:
	.space	4

/*
 * Room for two PTE pointers, usually the kernel and current user pointers
 * to their respective root page table.
 */
abatron_pteptrs:
	.space	8
