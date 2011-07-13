/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Copyright (C) 2008 Wind River Systems, Inc.
 */

#ifndef __HEAD_WRHV_H__
#define __HEAD_WRHV_H__

#include <asm/arch_vbi.h>

#ifdef	CONFIG_PPC85xx_VT_MODE
#define TLBWE_CODE	0x7C0007A4
#define TLBSX_CODE	0x7c005724
#endif

#define WRHV_COREID_OFFSET	0x98

	/* Interrupts are disabled by hypervisor at this entry point.
	 * It puts the following registers into the status page:
	 *   VB_STATUS_OLD_INT_DISABLE (the INT_DISABLE from Control)
	 *   CR register
	 *   SRR0 register - pc at time of interrupt
	 *   SRR1 register - msr at time of interrupt
	 *   LR register - link register at time of interrupt
	 *   R3 register - R3 at time of interrupt
	 *   R4 register - R4 at time of interrupt
	 * When code in this macro has been executed, r9 contains MSR, r12 pc
	 * r10 is trashed and r11 pointer on interrupt frame. All other
	 * registers contain their value before the system call was executed.
	 */
#ifndef	CONFIG_PPC85xx_VT_MODE
#ifdef CONFIG_WRHV_ASID_OPTIMIZATION
#define ASID_OPT							\
	lis     r9,wr_control@ha;					\
	lwz     r9,wr_control@l(r9);					\
	lwz     r12,VB_STATUS_VMMU0(r4);				\
	stw     r12,VB_CONTROL_VMMU0(r9);				\
	lwz     r12,VB_STATUS_VMMU_HANDLE(r4);				\
	stw     r12,VB_CONTROL_VMMU_HANDLE(r9);				\
	lwz     r12,VB_STATUS_ASID(r4);					\
	stw     r12,VB_CONTROL_ASID(r9)
#else
#define ASID_OPT							\
	nop
#endif
#undef NORMAL_EXCEPTION_PROLOG
#define NORMAL_EXCEPTION_PROLOG						     \
        mr      r4,r1;                                                       \
        WRHV_SUP_MODE_GET(r3);         /* check whether user or kernel */   \
        cmpwi   0,r3,0;                                                 \
        bne     1f;                                                          \
        WRHV_MFSPRG3(r1);              /* if from user, start at top of   */\
        lwz     r1,THREAD_INFO-THREAD(r1); /* this thread's kernel stack   */\
        addi    r1,r1,THREAD_SIZE;                                           \
1:      subi    r1,r1,INT_FRAME_SIZE;   /* Allocate an exception frame     */\
        mr      r3,r1;                                                       \
        stw     r0,GPR0(r3);                                                 \
        stw     r4,GPR1(r3);                                                 \
        stw     r4,0(r3);                                                    \
        SAVE_4GPRS(5, r3);                                                   \
        SAVE_4GPRS(9, r3);                                                   \
        mr      r11,r3;                                                      \
        lis     r4,wr_status@ha;                                           \
        lwz     r4,wr_status@l(r4);                                        \
        lwz     r12,VB_STATUS_LR(r4);                                        \
        stw     r12,_LINK(r11);                                              \
        lwz     r12,VB_STATUS_R3(r4);                                        \
        stw     r12,GPR3(r11);                                               \
        lwz     r12,VB_STATUS_R4(r4);                                        \
        stw     r12,GPR4(r11);                                               \
        lwz     r12,VB_STATUS_CR(r4);                                        \
        stw     r12,_CCR(r11);                                               \
        ASID_OPT;                                                            \
        lwz     r9,VB_STATUS_SRR1(r4);                                       \
	rlwinm  r9,r9,0,18,15; /* Clear EE & PR bits */                      \
	mr	r12, r4;                                                     \
        lwz     r12,VB_STATUS_OLD_INT_DISABLE(r12);                          \
        cmpwi   0,r12,0;                                                     \
        bne     2f;                                                          \
        ori     r9,r9,MSR_EE;                                                \
2:      lis     r12,wrhv_supervisor@ha;                                     \
        lwz     r12,wrhv_supervisor@l(r12);                                \
        cmpwi   0,r12,0;                                                     \
        bne     3f;                                                          \
        ori     r9,r9,MSR_PR;                                                \
3:      li      r12,1;                                                       \
        WRHV_SET_SUP_MODE(r3,r12);                                          \
        lwz     r12,VB_STATUS_SRR0(r4);                                      \
        lwz     r3,VB_STATUS_R3(r4);                                         \
        mr      r10,r4;                                                      \
        lwz     r4,VB_STATUS_R4(r4)
#endif

/*
 * Macros used for set Book-e exception table
*/
#undef SET_IVOR
#define SET_IVOR(vector_number, vector_label)		\
	li	r3,vector_number@l;			\
	lis	r4,vector_label@h;			\
	ori	r4,r4,vector_label@l;			\
	bl	set_exec_table

#undef DEBUG_DEBUG_EXCEPTION
#define DEBUG_DEBUG_EXCEPTION						      \
	START_EXCEPTION(DebugDebug);						\
	NORMAL_EXCEPTION_PROLOG;					\
	mr      r4,r12;                /* Pass SRR0 as arg2 */		\
	lwz     r5,VB_STATUS_ESR(r10);					\
	stw     r5,_ESR(r11);						\
	addi    r3,r1,STACK_FRAME_OVERHEAD;				\
	/* EXC_XFER_STD(0x1000, DebugException)	*/		\
	EXC_XFER_TEMPLATE(DebugException, 0x2008, (MSR_KERNEL & ~(MSR_ME|MSR_DE|MSR_CE)), NOCOPY, transfer_to_handler_full, ret_from_except_full)

#ifndef CONFIG_PPC85xx_VT_MODE
#undef INSTRUCTION_STORAGE_EXCEPTION
#define INSTRUCTION_STORAGE_EXCEPTION					      \
	START_EXCEPTION(InstructionStorage)				      \
	NORMAL_EXCEPTION_PROLOG;					      \
	mr      r4,r12;                /* Pass SRR0 as arg2 */                \
        lwz     r5,VB_STATUS_ESR(r10);                                        \
        stw     r5,_ESR(r11);                                                 \
        li      r5,0;                   /* Pass zero as arg3 */               \
	EXC_XFER_EE_LITE(0x0400, handle_page_fault)

#undef ALIGNMENT_EXCEPTION
#define ALIGNMENT_EXCEPTION							\
	START_EXCEPTION(Alignment)						\
	NORMAL_EXCEPTION_PROLOG;						\
	lwz r5,VB_STATUS_DEAR(r10);						\
	stw r5,_DEAR(r11);							\
	addi r3,r1,STACK_FRAME_OVERHEAD;					\
	EXC_XFER_EE(0x0600, alignment_exception)

#undef PROGRAM_EXCEPTION
#define PROGRAM_EXCEPTION						      \
	START_EXCEPTION(Program)					      \
	NORMAL_EXCEPTION_PROLOG;					      \
	mr      r4,r12;               /* Pass SRR0 as arg2 */                \
        lwz     r5,VB_STATUS_ESR(r10);                                        \
	stw	r5,_ESR(r11);						      \
	addi	r3,r1,STACK_FRAME_OVERHEAD;				      \
	EXC_XFER_STD(0x0700, program_check_exception)
#endif

#undef DECREMENTER_EXCEPTION
#define DECREMENTER_EXCEPTION						      \
	START_EXCEPTION(Decrementer)					      \
	NORMAL_EXCEPTION_PROLOG;					      \
	addi    r3,r1,STACK_FRAME_OVERHEAD;				      \
	EXC_XFER_LITE(0x0900, timer_interrupt)


/* ensure this structure is always sized to a multiple of the stack alignment */
#define STACK_EXC_LVL_FRAME_SIZE	_ALIGN_UP(sizeof (struct exception_regs), 16)

#endif /* __HEAD_BOOKE_H__ */
