/*
 * arch_vbi.h - ARM architecture specific definitions
 *
 * Copyright 2010-2011 Wind River Systems, Inc.
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

#ifndef _ASM_ARCH_VBI_H
#define _ASM_ARCH_VBI_H

#if (CPU == ARMCA9)
#define USE_TRUSTZONE
#endif

/* For now restrict operation to little endian. */
/* Later this should come from the build system. */

#define __VBI_BYTE_ORDER __VBI_LITTLE_ENDIAN

/* VIOAPIC number of entries */

#define VB_VIOAPIC_ENTRIES_SIZE		    64 

#define _WRHV_ARCH_HAS_STATUS_REGS	    1
#define _WRHV_ARCH_HAS_CTRL_REGS	    1

#define VB_STATUS_REGS_OFFSET_BASE	 0
#define VB_STATUS_MODE			(4 * (VB_STATUS_REGS_OFFSET_BASE + 0))
#define VB_STATUS_DFSR			(4 * (VB_STATUS_REGS_OFFSET_BASE + 1))
#define VB_STATUS_IFSR			(4 * (VB_STATUS_REGS_OFFSET_BASE + 2))
#define VB_STATUS_DFAR			(4 * (VB_STATUS_REGS_OFFSET_BASE + 3))
#define VB_STATUS_IFAR			(4 * (VB_STATUS_REGS_OFFSET_BASE + 4))
#define VB_STATUS_INTVECTOR		(4 * (VB_STATUS_REGS_OFFSET_BASE + 5))
#define VB_STATUS_COPROC_RIGHTS		(4 * (VB_STATUS_REGS_OFFSET_BASE + 6))
#define VB_STATUS_COPROC_GRANTS		(4 * (VB_STATUS_REGS_OFFSET_BASE + 7))
#define VB_STATUS_BANKED_REGS		(4 * (VB_STATUS_REGS_OFFSET_BASE + 8))
/* (SPSR, SP, LR) * 16 = 48 */

#define VB_STATUS_REG_STRUCT_END	(4 * (VB_STATUS_REGS_OFFSET_BASE + 56))

#define VB_CONTROL_REGS_OFFSET_BASE	 0

#define VB_CONTROL_CPSR		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 0))
#define VB_CONTROL_SPSR		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 1))
#define VB_CONTROL_R0		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 2))
#define VB_CONTROL_R1		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 3))
#define VB_CONTROL_R2		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 4))
#define VB_CONTROL_R3		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 5))
#define VB_CONTROL_R4		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 6))
#define VB_CONTROL_R5		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 7))
#define VB_CONTROL_R6		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 8))
#define VB_CONTROL_R7		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 9))
#define VB_CONTROL_R8		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 10))
#define VB_CONTROL_R9		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 11))
#define VB_CONTROL_R10		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 12))
#define VB_CONTROL_R11		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 13))
#define VB_CONTROL_R12		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 14))
#define VB_CONTROL_R13		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 15))
#define VB_CONTROL_R14		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 16))
#define VB_CONTROL_R15		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 17))
#define VB_CONTROL_SP		VB_CONTROL_R13
#define VB_CONTROL_LR		VB_CONTROL_R14
#define VB_CONTROL_PC		VB_CONTROL_R15

#define VB_CONTROL_REG_STRUCT_END	(4 * (VB_CONTROL_REGS_OFFSET_BASE + 18))

/* status register size for arm */

#ifndef _ASMLANGUAGE

#define VB_STATUS_REGS_ACCESS(base, field)	\
	base->vb_status_regs.field		

#define VB_CONTROL_REGS_ACCESS(base, field)	\
	base->vb_control_regs.field		


/*
 * vb_arch_ctrl_regs - Virtual core ARM control structure
 *
 * Virtual board emulated control registers. These registers are used
 * by a guest running on hypervisor to configure the virtual CPU register.
 *
 * ARM Control structure graphical illustration
 *        _______________ 
 *       |       PC      |
 *       |---------------|
 *       |       MSR     |
 *       |---------------|
 *       |       CR      |
 *       |---------------|
 *       |               |
 *       |general purpose|
 *       |     r[0:13]   |
 *       |               |
 *       |---------------|
 *       |emulated MSR   |
 *       |---------------|
 *       |       dbsr    |
 *       |---------------|
 *       | dbcr[0:2]     |
 *       |---------------|
 *       |IAC[1:2]       |
 *       |---------------|
 *       |DAC[1:2]       |
 *       |---------------|
 *       |reserved[0:9]  |
 *       |---------------|
 *
 */

struct vb_arch_ctrl_regs
{
	unsigned long cpsr;		/* saved CPSR */
	unsigned long spsr;		/* saved SPSR */

	/* registers that can be restored by a sys_ret fast system call */

	unsigned long r0;		/* general purpose register R0  */
	unsigned long r1;		/* general purpose register R1  */
	unsigned long r2;		/* general purpose register R2  */
	unsigned long r3;		/* general purpose register R3  */
	unsigned long r4;		/* general purpose register R4  */
	unsigned long r5;		/* general purpose register R5  */
	unsigned long r6;		/* general purpose register R6  */
	unsigned long r7;		/* general purpose register R7  */
	unsigned long r8;		/* general purpose register R8  */
	unsigned long r9;		/* general purpose register R9  */
	unsigned long r10;		/* general purpose register R10 */
	unsigned long r11;		/* general purpose register R11 */
	unsigned long r12;		/* general purpose register R12 */
	unsigned long sp;		/* r13 */
	unsigned long lr;		/* r14 */
	unsigned long pc;		/* r15 */

	unsigned long asid;		/* Virtual ASID */
	unsigned long vmmu_handle;  	/* VMMU Handle returned from
					   vbi_create_vmmu */
};

/*
 *
 * vb_arch_stat_regs - Virtual core emulated status registers
 *
 * Virtual board emulated CPU status registers
 *
 *
 * ARM Status structure graphical illustration
 *        _______________   ---------------------------
 *       |    SRR0       |
 *       |---------------|
 *       |    SRR1       |   Registers save by WRHV
 *       |---------------|   before injecting an exception/interrupt
 *       |    CR         |
 *       |---------------|
 *       |    LR         |
 *       |---------------|
 *       |    r3         |
 *       |---------------|
 *       |    r4         |
 *       |---------------|   ---------------------------
 *       |    mcsrr0     |
 *       |---------------|  Registers saved duing exception handling
 *       |    mcssr1     |
 *       |---------------|
 *       |    esr        |
 *       |---------------|
 *       |    mcsr       |
 *       |---------------|
 *       |    mcar       |
 *       |---------------|
 *       |    dear       |
 *       |---------------|
 *       |    emsr       |
 *       |---------------|
 *       |    esrr0      |
 *       |---------------|
 *       |    esrr1      |
 *       |---------------|    -------------------------
 *       | reserved[8]   |      8 x 32bits for future enhencements
 *       |---------------|    -------------------------
 *       |    svr        |
 *       |---------------|
 *       |    pir        |
 *       |---------------|
 *       |    pvr        |    Configuration registers updated
 *       |---------------|    upon board creation
 *       |    hid0       |
 *       |---------------|
 *       |    hid1       |
 *       |---------------|
 *       |    buscr      |
 *       |---------------|
 *       |    l1csr0     |
 *       |---------------|
 *       |    l1csr1     |
 *       |---------------|   --------------------------
 *       | reserved[0:8] | 9x 32bit
 *       |---------------|
 *
 */

struct vb_arch_stat_regs
{
	/* 
	 * registers saved by Razor for all interrupts exceptions
	 * before setting the PC in the virtual board to the exception/interrupt
	 * vector address
	 */

	unsigned long mode;
	unsigned long dfsr;	/* cp15 c5, Data Fault Status Register */
	unsigned long ifsr;	/* cp15 c5, Instruction Fault Status Register */
	unsigned long dfar;	/* like cp15 c6, Fault Address Register */
	unsigned long ifar;	/* like cp15 c6, Fault Address Register */
	unsigned long intVector; /* vector index indicating highest pri pend */

	/* bit0 = cp0, ..., bit13 = cp13; 1 = grant, 0 = deny */
	unsigned long coprocRights; /* CP access rights for this VB [13:0] */

	/* CPs that have been granted user-mode access by the VB [13:0] */
	/* coprocGrants is changed in sync with regs->coprocAccess */
	unsigned long coprocGrants;

	struct
	{
		unsigned long spsr;	/* mode specific SPSR */
		unsigned long sp;	/* mode specific r13 (stack pointer) */
		unsigned long lr;	/* mode specific r14 (link register) */
	} modeSpecificReg[16];
};


/*
 * VBI_HREG_SET - hardware register set, for read/write
 *
 * Used by vbi_vb_read_reg/vbi_vb_write_reg to read/write registers in
 * another VB
 *
 */

#define VBI_GREG_NUM	13	/* has 32 32/64-bit data registers */

typedef struct
{
	uint32_t r[VBI_GREG_NUM];	/* general purpose registers 0-12 */
	INSTR *pc;			/* program counter. aka r[15] */
	uint32_t cpsr;
	uint32_t sp_usr;		/* or sys mode. aka r[13] */
	uint32_t lr_usr;		/* or sys mode. aka r[14] */
} VBI_HREG_SET;


/* complex register set definition */

typedef union
{
	VBI_HREG_SET    hreg32;	/* 32 bit register set */
} VBI_HREG_SET_CMPLX;


typedef struct
{
	uint32_t	vbiRegType;
	VBI_HREG_SET_CMPLX vbiRegSet;
} VBI_HREG_SET_CMPLX_QUALIFIED;

/* common system wide message header structure */

typedef struct vbi_msg_header {
	uint32_t msgId;	/* message type identification */
	uint32_t msgArg;	/* argument associated with message type */
} VBI_MSG_HEADER;

/* request message */

typedef struct 
    {
    VBI_MSG_HEADER hdr;			/* message header */
    uint32_t   request;			/* request type */

    } VBI_BSP_MSG;

typedef struct 
{
	VBI_MSG_HEADER hdr;			/* message header */
	uint32_t   status;			/* request completion status */
	uint32_t   dataVal;
} VBI_BSP_MSG_REPLY;

#else /*_ASMLANGUAGE */

/*
 *
 * VBI_INT_VCORE_LOCK - lock a core's interrupts macro
 *
 * This macro disables the currently running core interrupts and returns the
 * previous state.
 *
 * A hypercall is not needed to perform this operation. 
 *
 */

#define ARM_IMM	#
#define CPSR_I	0x80
#define CPSR_F	0x40

#ifdef USE_TRUSTZONE

/*
 * Trustzone Implementation
 *
 * REG0 = CPSR
 * REG0 &= (CPSR_I | CPSR_F)
 * set CPSR.I and CPSR.F
 *
 * REG1 and REG2 not used.
 */

#define VBI_INT_VCORE_LOCK(REG0, REG1, REG2) \
	mrs	REG0, cpsr;					\
	and	REG0, REG0, ARM_IMM (CPSR_I | CPSR_F);		\
	cpsid	if

#else /* USE_TRUSTZONE */

/*
 * Non-Trustzone Implementation
 *
 * wrhvVbControl->intDisable = -1; return old value
 *
 * REG1 = -1
 * REG2 = &wrvhVbControl
 * REG2 = wrvhVbControl
 * REG0 = wrhvVbControl->intDisable
 * wrhvVbControl->intDisable = REG1
 *
 * THIS NON-TRUSTZONE VERSION IS NOT TESTED.
 */

#define VBI_INT_VCORE_LOCK(REG0, REG1, REG2) \
	mvn	REG1, ARM_IMM 0;				\
	ldr	REG2, =wrhvVbControl;				\
	ldr	REG2, [REG2];					\
	ldr	REG0, [REG2, ARM_IMM VB_CONTROL_INT_DISABLE];	\
	str	REG1, [REG2, ARM_IMM VB_CONTROL_INT_DISABLE]

#endif /* USE_TRUSTZONE */

/*
 *
 * VBI_INT_VCORE_UNLOCK - unlock a core's interrupts
 *
 * This macro enables a core's interrupts.
 *
 */

#ifdef USE_TRUSTZONE

/*
 * Trustzone Implementation
 *
 * Clear CPSR.I and CPSR.F. No hypercall is needed; any pending interrupts
 * are immediately taken.
 *
 * REG0 and REG1 not used.
 */

#define VBI_INT_VCORE_UNLOCK(REG0, REG1) \
	cpsie	if

#else /* USE_TRUSTZONE */

/*
 * Non-Trustzone Implementation
 *
 * wrhvVbControl->intDisable = 0;
 * if (wrhvVbControl->intPending != 0xffff)
 *     syscall
 *
 * THIS NON-TRUSTZONE VERSION IS NOT COMPLETE. The #if 0 code is partial; the
 * #else code intentionally does nothing, and should make it obvious when it
 * is first used that it needs finishing and testing.
 *
 * It is NOT a good idea to just set intDisable to zero, without figuring out
 * how to decide if a syscall is needed and then making it if so, since that
 * would "kinda work".
 */

#if 0
#define VBI_INT_VCORE_UNLOCK(REG0, REG1) \
	mov	REG0, ARM_IMM 0;				\
	ldr	REG1, =wrhvVbControl;				\
	ldr	REG1, [REG1];					\
	str	REG0, [REG1, ARM_IMM VB_CONTROL_INT_DISABLE]
/* XXX check if any interrupts are pending and if so, make hypercall */
#else
#define VBI_INT_VCORE_UNLOCK(REG0, REG1)
#endif

#endif /* USE_TRUSTZONE */

/*
 *
 * VBI_INT_VCORE_STATE_GET - Get interrupts state
 *
 * This macro reads the interrupt state of the currently running core. It uses
 * a passed in general purpose register to store the current state of
 * interrupts.  The status is nonzero if locked or 0 if not locked.
 *
 */

#ifdef USE_TRUSTZONE

/*
 * Trustzone Implementation
 *
 * REG0 = CPSR
 * REG0 &= (CPSR_I | CPSR_F)
 */

#define VBI_INT_VCORE_STATE_GET(REG0) \
	mrs	REG0, cpsr;					\
	and	REG0, REG0, ARM_IMM (CPSR_I | CPSR_F)

#else /* USE_TRUSTZONE */

/*
 * Non-Trustzone Implementation
 *
 * return wrhvVbControl->intDisable;
 *
 * REG0 = &wrvhVbControl
 * REG0 = wrvhVbControl
 * REG0 = wrhvVbControl->intDisable
 *
 * THIS NON-TRUSTZONE VERSION IS NOT TESTED.
 */

#define VBI_INT_VCORE_STATE_GET(REG0) \
	ldr	REG0, =wrhvVbControl;				\
	ldr	REG0, [REG0];					\
	ldr	REG0, [REG0, ARM_IMM VB_CONTROL_INT_DISABLE]

#endif /* USE_TRUSTZONE */

/*
 *
 * VBI_CONFIG_ADDR_GET - Get virtual core configuration structure base address
 *
 * This macro returns the base address of the configuration structure of the 
 * running core. 
 *
 */

#if 0
#define VBI_CONFIG_ADDR_GET(reg)		    \
        lis     reg, HIADJ(wrhvVbConfig);	    \
        lwz     reg, LO(wrhvVbConfig)(reg)	    
#endif

/*
 *
 * VBI_CNTRL_ADDR_GET - Get virtual core control structure base address
 *
 * This macro returns the base address of the running virtual core's control
 * structure.
 *
 */

#if 0
#define VBI_CNTRL_ADDR_GET(reg)			    \
        lis     reg, HIADJ(wrhvVbControl);	    \
        lwz     reg, LO(wrhvVbControl)(reg) 	     
#endif

/*
 *
 * VBI_STATUS_ADDR_GET - Get virtual core status structure address
 *
 * This macro returns the base address of the status structure of currently
 * running core. This structure is read-only and contains a description of
 * the running virtual core. Hypervisor uses this data to inform the
 * virtual board time variant data that may be updated during hypervisor context
 * Switch. Typical that are available in the status structure are:
 *
 *Timer tick counter
 *
 *Pending interrupt state
 *
 *The interrupt state before this core was schedule
 *
 *VMMU configuration
 *
 *Virtual core registers state
 *
 */

#if 0
#define VBI_STATUS_ADDR_GET(reg)			    \
        lis     reg, HIADJ(wrhvVbStatus);		    \
        lwz     reg, LO(wrhvVbStatus)(reg)	     
#endif

#endif /*_ASMLANGUAGE */
    
#endif /* _ASM_ARCH_VBI_H */
