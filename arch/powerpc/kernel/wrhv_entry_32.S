/*
 *  PowerPC version
 *    Copyright (C) 1995-1996 Gary Thomas (gdt@linuxppc.org)
 *  Rewritten by Cort Dougan (cort@fsmlabs.com) for PReP
 *    Copyright (C) 1996 Cort Dougan <cort@fsmlabs.com>
 *  Adapted for Power Macintosh by Paul Mackerras.
 *  Low-level exception handlers and MMU support
 *  rewritten by Paul Mackerras.
 *    Copyright (C) 1996 Paul Mackerras.
 *  MPC8xx modifications Copyright (C) 1997 Dan Malek (dmalek@jlc.net).
 *  
 *  Fork from entry_32.S for Hypervisor/Guest, Copyright (C) 2009
 *  Wind River Systems, Inc.
 *
 *  This file contains the system call entry code, context switch
 *  code, and exception/interrupt return code for PowerPC.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

#include <linux/errno.h>
#include <linux/sys.h>
#include <linux/threads.h>
#include <asm/reg.h>
#include <asm/page.h>
#include <asm/mmu.h>
#include <asm/cputable.h>
#include <asm/thread_info.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>
#include <asm/unistd.h>
#include <asm/ftrace.h>
#ifdef CONFIG_WRHV
#include <vbi/interface.h>
#include <asm/arch_vbi.h>
#include <vbi/syscalls.h>
#endif /* CONFIG_WRHV */

#undef SHOW_SYSCALLS
#undef SHOW_SYSCALLS_TASK
#ifdef	CONFIG_WRHV
#undef VMMU  /* just for debugging */
#endif /* CONFIG_WRHV */

#ifdef CONFIG_WRHV
       .data
       .globl  wrhv_sprg3
wrhv_sprg3:
       .long   0
#ifdef CONFIG_SMP
       .globl wrhv_pir
wrhv_pir:
       .long   0
#endif
       .text
#endif /* CONFIG_WRHV */

/*
 * MSR_KERNEL is > 0x10000 on 4xx/Book-E since it include MSR_CE.
 */
#if MSR_KERNEL >= 0x10000
#define LOAD_MSR_KERNEL(r, x)	lis r,(x)@h; ori r,r,(x)@l
#else
#define LOAD_MSR_KERNEL(r, x)	li r,(x)
#endif

	.globl	paravirt_transfer_to_handler
paravirt_transfer_to_handler:
	stw	r2,GPR2(r11)
	stw	r12,_NIP(r11)
	stw	r9,_MSR(r11)
	andi.	r2,r9,MSR_PR
	mfctr	r12
	mfspr	r2,SPRN_XER
	stw	r12,_CTR(r11)
	stw	r2,_XER(r11)
	WRHV_MFSPRG3(r12)
	addi	r2,r12,-THREAD
	tovirt(r2,r2)			/* set r2 to current */
	beq	2f			/* if from user, fix up THREAD.regs */
	addi	r11,r1,STACK_FRAME_OVERHEAD
	stw	r11,PT_REGS(r12)
#if defined(CONFIG_40x) || defined(CONFIG_BOOKE)
	/* Check to see if the dbcr0 register is set up to debug.  Use the
	internal debug mode bit to do this. */
	lwz	r12,THREAD_DBCR0(r12)
	andis.	r12,r12,DBCR0_IDM@h
	beq+	3f
	/* From user and task is ptraced - load up global dbcr0 */
	li	r12,-1                  /* clear all pending debug events */
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	stw	r12,VB_CONTROL_DBCR0(r9)
	lis	r11,global_dbcr0@ha
	tophys(r11,r11)
	addi	r11,r11,global_dbcr0@l
#ifdef CONFIG_SMP
	rlwinm	r9,r1,0,0,(31-THREAD_SHIFT)
	lwz	r9,TI_CPU(r9)
	slwi	r9,r9,3
	add	r11,r11,r9
#endif
	lwz	r12,0(r11)
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	stw	r12,VB_CONTROL_DBCR0(r9)
	lwz	r12,4(r11)
	addi	r12,r12,-1
	stw	r12,4(r11)
#endif
	b	3f

2:	/* if from kernel, check interrupted DOZE/NAP mode and
         * check for stack overflow
         */
	lwz	r9,KSP_LIMIT(r12)
	cmplw	r1,r9			/* if r1 <= ksp_limit */
	ble-	paravirt_stack_ovf		/* then the kernel stack overflowed */
5:
#if defined(CONFIG_6xx) || defined(CONFIG_E500)
	rlwinm	r9,r1,0,0,31-THREAD_SHIFT
	tophys(r9,r9)			/* check local flags */
	lwz	r12,TI_LOCAL_FLAGS(r9)
	mtcrf	0x01,r12
	bt-	31-TLF_NAPPING,4f
	bt-	31-TLF_SLEEPING,7f
#endif /* CONFIG_6xx || CONFIG_E500 */
	.globl paravirt_transfer_to_handler_cont
paravirt_transfer_to_handler_cont:
3:
	mflr	r9
	lwz	r11,0(r9)		/* virtual address of handler */
	lwz	r9,4(r9)		/* where to go when done */
	mtlr	r9
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	stw	r11,VB_CONTROL_SRR0(r9)
	mfcr	r11
	stw	r11,VB_CONTROL_CR(r9)
	stw	r0,VB_CONTROL_R0(r9)

	lis	r12,wr_status@ha
	lwz	r12,wr_status@l(r12)

	lwz	r11,VB_STATUS_OLD_INT_DISABLE(r12)
	stw	r11,VB_CONTROL_NEW_INT_DISABLE(r9)

#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
	/* Push the virtual ASID into the control struct */
	lwz	r11,VB_STATUS_ASID(r12)
	stw	r11,VB_CONTROL_ASID(r9)

	/* Update the VMMU handle */
	lwz	r11,VB_STATUS_VMMU_HANDLE(r12)
	stw	r11,VB_CONTROL_VMMU_HANDLE(r9)

	lwz	r11,VB_STATUS_VMMU0(r12)
	stw	r11,VB_CONTROL_VMMU0(r9)
#endif
/*
	lwz	r11,VB_STATUS_CR(r12)
	stw	r11,VB_CONTROL_CR(r9)
*/

#ifdef VMMU
        /* restore vmmu from wr_status to wr_control */

	lwz	r11,VB_STATUS_VMMU0(r12)
	stw	r11,VB_CONTROL_VMMU0(r9)

	lwz	r11,VB_STATUS_VMMU1(r12)
	stw	r11,VB_CONTROL_VMMU1(r9)

	lwz	r11,VB_STATUS_EMSR(r12)
	stw	r11,VB_CONTROL_EMSR(r9)

	stw	r1,VB_CONTROL_SP(r9)
	stw	r2,VB_CONTROL_R2(r9)
	stw	r3,VB_CONTROL_R3(r9)
	stw	r4,VB_CONTROL_R4(r9)
	stw	r5,VB_CONTROL_R5(r9)
	stw	r6,VB_CONTROL_R6(r9)
	stw	r7,VB_CONTROL_R7(r9)
	stw	r8,VB_CONTROL_R8(r9)
	stw	r10,VB_CONTROL_R10(r9)
	mflr	r11
#endif

	WRHV_LOAD_MSR(r10,r9,r11)

	lis	r0,VBI_SYS_ctx_load@h
	ori	r0,r0,VBI_SYS_ctx_load@l
	sc

#if defined (CONFIG_6xx) || defined(CONFIG_E500)
4:	rlwinm	r12,r12,0,~_TLF_NAPPING
	stw	r12,TI_LOCAL_FLAGS(r9)
	b	power_save_ppc32_restore

7:	rlwinm	r12,r12,0,~_TLF_SLEEPING
	stw	r12,TI_LOCAL_FLAGS(r9)
	lwz	r9,_MSR(r11)		/* if sleeping, clear MSR.EE */
	rlwinm	r9,r9,0,~MSR_EE
	lwz	r12,_LINK(r11)		/* and return to address in LR */
	b	fast_exception_return
#endif

/*
 * On kernel stack overflow, load up an initial stack pointer
 * and call StackOverflow(regs), which should not return.
 */
paravirt_stack_ovf:
	/* sometimes we use a statically-allocated stack, which is OK. */
	lis	r12,_end@h
	ori	r12,r12,_end@l
	cmplw	r1,r12
	ble	5b			/* r1 <= &_end is OK */
	SAVE_NVGPRS(r11)
	addi	r3,r1,STACK_FRAME_OVERHEAD
	lis	r1,init_thread_union@ha
	addi	r1,r1,init_thread_union@l
	addi	r1,r1,THREAD_SIZE-STACK_FRAME_OVERHEAD
	lis	r9,StackOverflow@ha
	addi	r9,r9,StackOverflow@l
	LOAD_MSR_KERNEL(r10,MSR_KERNEL)
	WRHV_FIX_MSR(r10,r11)
	b	StackOverflow
#if 0
	FIX_SRR1(r10,r12)
	mtspr	SPRN_SRR0,r9
	mtspr	SPRN_SRR1,r10
	SYNC
	RFI
#endif


	.globl	paravirt_ret_from_syscall
paravirt_ret_from_syscall:
#ifdef SHOW_SYSCALLS
	bl	do_show_syscall_exit
#endif
	mr	r6,r3
	rlwinm	r12,r1,0,0,(31-THREAD_SHIFT)	/* current_thread_info() */
	/* disable interrupts so current_thread_info()->flags can't change */
	WRHV_INT_LOCK(r10,r9)
	lwz	r9,TI_FLAGS(r12)
	li	r8,-_LAST_ERRNO
	andi.	r0,r9,(_TIF_SYSCALL_T_OR_A|_TIF_SINGLESTEP|_TIF_USER_WORK_MASK|_TIF_PERSYSCALL_MASK)
	bne-	syscall_exit_work
	cmplw	0,r3,r8
	blt+	paravirt_syscall_exit_cont
	lwz	r11,_CCR(r1)			/* Load CR */
	neg	r3,r3
	oris	r11,r11,0x1000	/* Set SO bit in CR */
	stw	r11,_CCR(r1)
paravirt_syscall_exit_cont:
#if defined(CONFIG_4xx) || defined(CONFIG_BOOKE)
	/* If the process has its own DBCR0 value, load it up.  The internal
	   debug mode bit tells us that dbcr0 should be loaded. */
	lwz	r0,THREAD+THREAD_DBCR0(r2)
	andis.	r10,r0,DBCR0_IDM@h
	bnel-	load_dbcr0
#endif
#ifdef CONFIG_44x
	lis	r4,icache_44x_need_flush@ha
	lwz	r5,icache_44x_need_flush@l(r4)
	cmplwi	cr0,r5,0
	bne-	2f
1:
#endif /* CONFIG_44x */
BEGIN_FTR_SECTION
	lwarx	r7,0,r1
END_FTR_SECTION_IFSET(CPU_FTR_NEED_PAIRED_STWCX)
	stwcx.	r0,0,r1			/* to clear the reservation */
	lwz	r4,_LINK(r1)
	lwz	r5,_CCR(r1)
	mtlr	r4
	lis	r4,wr_control@ha
	lwz	r4,wr_control@l(r4)
	stw	r5,VB_CONTROL_CR(r4)
	lwz	r5,GPR0(r1)
	stw	r5,VB_CONTROL_R0(r4)
	lwz	r5,_NIP(r1)
	stw	r5,VB_CONTROL_SRR0(r4)

	lis	r5,wr_status@ha
	lwz	r5,wr_status@l(r5)
	lwz	r5,VB_STATUS_OLD_INT_DISABLE(r5)
	stw	r5,VB_CONTROL_NEW_INT_DISABLE(r4)
 
	lwz	r5,_MSR(r1)
	WRHV_LOAD_MSR(r5,r7,r8)
	lwz	r2,GPR2(r1)
	lwz	r1,GPR1(r1)
	lis	r0,VBI_SYS_ctx_load@h
	ori	r0,r0,VBI_SYS_ctx_load@l
	sc

66:	li	r3,-ENOSYS
	b	paravirt_ret_from_syscall

	.globl	paravirt_syscall_exit_work
paravirt_syscall_exit_work:
	andi.	r0,r9,_TIF_RESTOREALL
	beq+	0f
	REST_NVGPRS(r1)
	b	2f
0:	cmplw	0,r3,r8
	blt+	1f
	andi.	r0,r9,_TIF_NOERROR
	bne-	1f
	lwz	r11,_CCR(r1)			/* Load CR */
	neg	r3,r3
	oris	r11,r11,0x1000	/* Set SO bit in CR */
	stw	r11,_CCR(r1)

1:	stw	r6,RESULT(r1)	/* Save result */
	stw	r3,GPR3(r1)	/* Update return value */
2:	andi.	r0,r9,(_TIF_PERSYSCALL_MASK)
	beq	4f

	/* Clear per-syscall TIF flags if any are set.  */

	li	r11,_TIF_PERSYSCALL_MASK
	addi	r12,r12,TI_FLAGS
3:	lwarx	r8,0,r12
	andc	r8,r8,r11
#ifdef CONFIG_IBM405_ERR77
	dcbt	0,r12
#endif
	stwcx.	r8,0,r12
	bne-	3b
	subi	r12,r12,TI_FLAGS
	
4:	/* Anything which requires enabling interrupts? */
	andi.	r0,r9,(_TIF_SYSCALL_T_OR_A|_TIF_SINGLESTEP)
	beq	ret_from_except

	/* Re-enable interrupts */
	WRHV_INT_UNLOCK(r10,r4)

	/* Save NVGPRS if they're not saved already */
	lwz	r4,_TRAP(r1)
	andi.	r4,r4,1
	beq	5f
	SAVE_NVGPRS(r1)
	li	r4,0xc00
	stw	r4,_TRAP(r1)
5:
	addi	r3,r1,STACK_FRAME_OVERHEAD
	bl	do_syscall_trace_leave
	b	ret_from_except_full


/*
 * This routine switches between two different tasks.  The process
 * state of one is saved on its kernel stack.  Then the state
 * of the other is restored from its kernel stack.  The memory
 * management hardware is updated to the second process's state.
 * Finally, we can return to the second process.
 * On entry, r3 points to the THREAD for the current task, r4
 * points to the THREAD for the new task.
 *
 * This routine is always called with interrupts disabled.
 *
 * Note: there are two ways to get to the "going out" portion
 * of this code; either by coming in via the entry (_switch)
 * or via "fork" which must set up an environment equivalent
 * to the "_switch" path.  If you change this , you'll have to
 * change the fork code also.
 *
 * The code which creates the new task context is in 'copy_thread'
 * in arch/ppc/kernel/process.c
 */
_GLOBAL(paravirt_switch)
	stwu	r1,-INT_FRAME_SIZE(r1)
	mflr	r0
	stw	r0,INT_FRAME_SIZE+4(r1)
	/* r3-r12 are caller saved -- Cort */
	SAVE_NVGPRS(r1)
	stw	r0,_NIP(r1)	/* Return to switch caller */
	LOAD_MSR_KERNEL(r11,MSR_KERNEL)
	WRHV_FIX_MSR(r11,r10)
#ifdef CONFIG_SPE
BEGIN_FTR_SECTION
	lis	r12,wr_status@ha
	lwz	r12,wr_status@l(r12)
	lwz	r12,VB_STATUS_SPEFSCR(r12)  /* save spefscr register value */
	stw	r12,THREAD+THREAD_SPEFSCR(r2)
END_FTR_SECTION_IFSET(CPU_FTR_SPE)
#endif /* CONFIG_SPE */
1:	stw	r11,_MSR(r1)
	mfcr	r10
	stw	r10,_CCR(r1)
	stw	r1,KSP(r3)	/* Set old stack pointer */

#ifdef CONFIG_SMP
	/* We need a sync somewhere here to make sure that if the
	 * previous task gets rescheduled on another CPU, it sees all
	 * stores it has performed on this one.
	 */
	sync
#endif /* CONFIG_SMP */

	tophys(r0,r4)
	CLR_TOP32(r0)
	WRHV_MTSPRG3(r0,r3)
	lwz	r1,KSP(r4)	/* Load new stack pointer */

	/* save the old current 'last' for return value */
	mr	r3,r2
	addi	r2,r4,-THREAD	/* Update current */

#ifdef CONFIG_SPE
BEGIN_FTR_SECTION
	lwz	r0,THREAD+THREAD_SPEFSCR(r2)
	lis	r12,wr_control@ha
	lwz	r12,wr_control@l(r12)
	stw	r0,VB_CONTROL_SPEFSCR(r12)	/* restore SPEFSCR reg */
END_FTR_SECTION_IFSET(CPU_FTR_SPE)
#endif /* CONFIG_SPE */

	lwz	r0,_CCR(r1)
	mtcrf	0xFF,r0
	/* r3-r12 are destroyed -- Cort */
	REST_NVGPRS(r1)

	lwz	r4,_NIP(r1)	/* Return to _switch caller in new task */
	mtlr	r4
	addi	r1,r1,INT_FRAME_SIZE
	blr


	/* interrupts are hard-disabled at this point */
	.globl	paravirt_restore
paravirt_restore:
#ifdef	CONFIG_WRHV
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

#ifdef VMMU
	stw	r2,VB_CONTROL_R2(r4)
	stw	r3,VB_CONTROL_R3(r4)
	stw	r6,VB_CONTROL_R6(r4)
	stw	r7,VB_CONTROL_R7(r4)
	stw	r8,VB_CONTROL_R8(r4)
	stw	r9,VB_CONTROL_R9(r4)
	stw	r10,VB_CONTROL_R10(r4)
	stw	r11,VB_CONTROL_R11(r4)
#endif /* VMMU */
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
	WRHV_LOAD_MSR(r0,r12,r5)
	lwz	r12,GPR12(r1)
	lwz	r5,GPR5(r1)
#ifdef VMMU
	stw	r12,VB_CONTROL_R12(r4)
	stw	r5,VB_CONTROL_R5(r4)

	lwz	r5,GPR1(r1)
	stw	r5,VB_CONTROL_R1(r4)

	lwz	r5,GPR4(r1)
	stw	r5,VB_CONTROL_R4(r4)

	lis	r12,wr_status@ha
	lwz	r12,wr_status@l(r12)

	lwz	r5,VB_STATUS_EMSR(r12)
	stw	r5,VB_CONTROL_EMSR(r4)

#if 1
        /* resume VMMU, since we always turn VMMU back on during the
         * exception entrance, this is really not needed, but we
         * make them here anyway for consistency
         */
	lwz	r5,VB_STATUS_VMMU0(r12)
	stw	r5,VB_CONTROL_VMMU0(r4)

	lwz	r5,VB_STATUS_VMMU1(r12)
	stw	r5,VB_CONTROL_VMMU1(r4)
#endif

	lis	r0,VBI_SYS_ctx_load_vmmu@h
	ori	r0,r0,VBI_SYS_ctx_load_vmmu@l
#else
	lwz	r4,GPR4(r1)
	lwz	r1,GPR1(r1)
	lis	r0,VBI_SYS_ctx_load@h
	ori	r0,r0,VBI_SYS_ctx_load@l
	sc
#endif  /* VMMU */

	/* Never back from here */
#endif /* CONFIG_WRHV */

	.globl  paravirt_load_dbcr0
paravirt_load_dbcr0:
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	lwz	r10,VB_CONTROL_DBCR0(r9)
	lis	r11,global_dbcr0@ha
	addi	r11,r11,global_dbcr0@l
#ifdef CONFIG_SMP
	rlwinm	r9,r1,0,0,(31-THREAD_SHIFT)
	lwz	r9,TI_CPU(r9)
	slwi	r9,r9,3
	add 	r11,r11,r9
#endif
	stw	r10,0(r11)
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	stw	r0,VB_CONTROL_DBCR0(r9)
	lwz	r10,4(r11)
	addi	r10,r10,1
	stw	r10,4(r11)
	li	r11,-1
	/* clear all pending debug events */
	lis	r9,wr_control@ha
	lwz	r9,wr_control@l(r9)
	stw	r11,VB_CONTROL_DBSR(r9)
	blr

