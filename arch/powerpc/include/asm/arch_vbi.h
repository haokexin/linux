/*
 * arch_vbi.h - PowerPC architecture specific definitions
 *
 * Copyright 2009-2011 Wind River Systems, Inc.
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

/* PPC uses big endian byte ordering */

#define __VBI_BYTE_ORDER __VBI_BIG_ENDIAN

/* VIOAPIC number of entries */

#define VB_VIOAPIC_ENTRIES_SIZE		64

#define _WRHV_ARCH_HAS_STATUS_REGS	1
#define _WRHV_ARCH_HAS_CTRL_REGS	1

#define VB_STATUS_REGS_OFFSET_BASE	0
#define VB_STATUS_SRR0			(4 * (VB_STATUS_REGS_OFFSET_BASE +0))
#define VB_STATUS_SRR1			(4 * (VB_STATUS_REGS_OFFSET_BASE +1))
#define VB_STATUS_CR			(4 * (VB_STATUS_REGS_OFFSET_BASE +2))
#define VB_STATUS_LR			(4 * (VB_STATUS_REGS_OFFSET_BASE +3))
#define VB_STATUS_R3			(4 * (VB_STATUS_REGS_OFFSET_BASE +4))
#define VB_STATUS_R4			(4 * (VB_STATUS_REGS_OFFSET_BASE +5))
#define VB_STATUS_MCSRR0		(4 * (VB_STATUS_REGS_OFFSET_BASE +6))
#define VB_STATUS_MCSRR1		(4 * (VB_STATUS_REGS_OFFSET_BASE +7))
#define VB_STATUS_ESR			(4 * (VB_STATUS_REGS_OFFSET_BASE +8))
#define VB_STATUS_MCSR			(4 * (VB_STATUS_REGS_OFFSET_BASE +9))
#define VB_STATUS_MCAR			(4 * (VB_STATUS_REGS_OFFSET_BASE +10))
#define VB_STATUS_DEAR			(4 * (VB_STATUS_REGS_OFFSET_BASE +11))
#define VB_STATUS_EMSR			(4 * (VB_STATUS_REGS_OFFSET_BASE +12))
#define VB_STATUS_ESRR0			(4 * (VB_STATUS_REGS_OFFSET_BASE +13))
#define VB_STATUS_ESRR1			(4 * (VB_STATUS_REGS_OFFSET_BASE +14))
#define VB_STATUS_SPEFSCR		(4 * (VB_STATUS_REGS_OFFSET_BASE +15))
#define VB_STATUS_ASID			(4 * (VB_STATUS_REGS_OFFSET_BASE +16))
#define VB_STATUS_VMMU_HANDLE		(4 * (VB_STATUS_REGS_OFFSET_BASE +17))
#define VB_STATUS_RESERVED1_3		(4 * (VB_STATUS_REGS_OFFSET_BASE +18))
#define VB_STATUS_RESERVED1_4		(4 * (VB_STATUS_REGS_OFFSET_BASE +19))
#define VB_STATUS_RESERVED1_5		(4 * (VB_STATUS_REGS_OFFSET_BASE +20))
#define VB_STATUS_RESERVED1_6		(4 * (VB_STATUS_REGS_OFFSET_BASE +21))
#define VB_STATUS_RESERVED1_7		(4 * (VB_STATUS_REGS_OFFSET_BASE +22))
#define VB_STATUS_SVR			(4 * (VB_STATUS_REGS_OFFSET_BASE +23))
#define VB_STATUS_PIR			(4 * (VB_STATUS_REGS_OFFSET_BASE +24))
#define VB_STATUS_PVR			(4 * (VB_STATUS_REGS_OFFSET_BASE +25))
#define VB_STATUS_HID0			(4 * (VB_STATUS_REGS_OFFSET_BASE +26))
#define VB_STATUS_HID1			(4 * (VB_STATUS_REGS_OFFSET_BASE +27))
#define VB_STATUS_BUSCR			(4 * (VB_STATUS_REGS_OFFSET_BASE +28))
#define VB_STATUS_L1CSR0		(4 * (VB_STATUS_REGS_OFFSET_BASE +29))
#define VB_STATUS_L1CSR1		(4 * (VB_STATUS_REGS_OFFSET_BASE +30))
#define VB_STATUS_RESERVED2_0		(4 * (VB_STATUS_REGS_OFFSET_BASE +31))
#define VB_STATUS_RESERVED2_1		(4 * (VB_STATUS_REGS_OFFSET_BASE +32))
#define VB_STATUS_RESERVED2_2		(4 * (VB_STATUS_REGS_OFFSET_BASE +33))
#define VB_STATUS_RESERVED2_3		(4 * (VB_STATUS_REGS_OFFSET_BASE +34))
#define VB_STATUS_RESERVED2_4		(4 * (VB_STATUS_REGS_OFFSET_BASE +35))
#define VB_STATUS_RESERVED2_5		(4 * (VB_STATUS_REGS_OFFSET_BASE +36))
#define VB_STATUS_RESERVED2_6		(4 * (VB_STATUS_REGS_OFFSET_BASE +37))
#define VB_STATUS_RESERVED2_7		(4 * (VB_STATUS_REGS_OFFSET_BASE +38))
#define VB_STATUS_RESERVED2_8		(4 * (VB_STATUS_REGS_OFFSET_BASE +39))


#define VB_STATUS_REG_STRUCT_END	(4 * (VB_STATUS_REGS_OFFSET_BASE +40))

/* Bit Mask definitions for VB_STATUS_INT_PENDING */

#define VB_STATUS_INT_PENDING_INT	1	/* Interrupt controller */
#define VB_STATUS_INT_PENDING_TICK	2	/* Tick interrupt */


#define VB_CONTROL_REGS_OFFSET_BASE	0
#define VB_CONTROL_SRR0			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 0))
#define VB_CONTROL_SRR1			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 1))
#define VB_CONTROL_CR			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 2))
#define VB_CONTROL_R0			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 3))
#define VB_CONTROL_R1			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 4))
#define VB_CONTROL_SP			VB_CONTROL_R1
#define VB_CONTROL_R2			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 5))
#define VB_CONTROL_R3			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 6))
#define VB_CONTROL_R4			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 7))
#define VB_CONTROL_R5			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 8))
#define VB_CONTROL_R6			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 9))
#define VB_CONTROL_R7			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 10))
#define VB_CONTROL_R8			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 11))
#define VB_CONTROL_R9			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 12))
#define VB_CONTROL_R10			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 13))
#define VB_CONTROL_R11			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 14))
#define VB_CONTROL_R12			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 15))
#define VB_CONTROL_R13			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 16))
#define VB_CONTROL_EMSR			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 17))
#define VB_CONTROL_DBSR			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 18))
#define VB_CONTROL_DBCR0		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 19))
#define VB_CONTROL_DBCR1		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 20))
#define VB_CONTROL_DBCR2		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 21))
#define VB_CONTROL_IAC1			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 22))
#define VB_CONTROL_IAC2			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 23))
#define VB_CONTROL_DAC1			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 24))
#define VB_CONTROL_DAC2			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 25))
#define VB_CONTROL_SPEFSCR		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 26))
#define VB_CONTROL_ASID			(4 * (VB_CONTROL_REGS_OFFSET_BASE + 27))
#define VB_CONTROL_VMMU_HANDLE		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 28))
#define VB_CONTROL_RESERVED3		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 29))
#define VB_CONTROL_RESERVED4		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 30))
#define VB_CONTROL_RESERVED5		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 31))
#define VB_CONTROL_RESERVED6		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 32))
#define VB_CONTROL_RESERVED7		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 33))
#define VB_CONTROL_RESERVED8		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 34))
#define VB_CONTROL_RESERVED9		(4 * (VB_CONTROL_REGS_OFFSET_BASE + 35))

/* status register size for ppc */

#define VB_CONTROL_REG_STRUCT_END	(4 * (VB_CONTROL_REGS_OFFSET_BASE + 36))

/* mdio messages */
#define MDIO_READ		1
#define MDIO_WRITE		2
#define MDIO_INT_ENABLE		3
#define MDIO_INT_DISABLE	4
#define BSP_CLK_FREQ		5
#define BSP_CORE_FREQ		6

#ifndef _ASMLANGUAGE

#define VB_STATUS_REGS_ACCESS(base, field)	\
	base->vb_status_regs.field

#define VB_CONTROL_REGS_ACCESS(base, field)	\
	base->vb_control_regs.field

/*
 * vb_arch_ctrl_regs - Virtual core PPC control structure
 *
 * Virtual board emulated control registers. These registers are used
 * by a guest running on hypervisor to configure the virtual CPU register.
 *
 * PPC Control structure graphical illustration
 *
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
 *
 */

struct vb_arch_ctrl_regs
{
	uint32_t srr0;		/*  0: PC */
	uint32_t srr1;		/*  1: MSR */
	uint32_t cr;		/*  2: condition registers */
	uint32_t r0;		/*  3: General register R0 */
	uint32_t sp;		/*  4: General register R1 (stack pointer) */
	uint32_t r2;		/*  5: General register R2 */
	uint32_t r3;		/*  6: General register R3 */
	uint32_t r4;		/*  7: General register R4 */
	uint32_t r5;		/*  8: General register R5 */
	uint32_t r6;		/*  9: General register R6 */
	uint32_t r7;		/* 10: General register R7 */
	uint32_t r8;		/* 11: General register R8 */
	uint32_t r9;		/* 12: General register R9 */
	uint32_t r10;		/* 13: General register R10 */
	uint32_t r11;		/* 14: General register R11 */
	uint32_t r12;		/* 15: General register R12 */
	uint32_t r13;		/* 16: General register R13 */

	uint32_t emsr;		/* 17: emulated MSR register */

	/* Debug control registers */

	uint32_t dbsr;		/* 18 - debug status register */

	/* Debug control registers */

	uint32_t dbcr0;		/* 19 - debug control register 0 */
	uint32_t dbcr1;		/* 20 - debug control register 1 */
	uint32_t dbcr2;		/* 21 - debug control register 2 */

	/* instruction address compare registers IAC1-IAC2 */

	uint32_t iac1;		/* 22 - instructions access control 1 */
	uint32_t iac2;		/* 23 - instructions access control 2 */

	/* data address compare registers DAC1-DAC2. */

	uint32_t dac1;		/* 24 - instructions access control 3 */
	uint32_t dac2;		/* 25 - instructions access control 4 */
#if (CPU==PPC85XX)
	uint32_t spefscr;	/* 26 - SPE float status control */
#else
	uint32_t reserved1;	/* 26: Reserved */
#endif
	uint32_t asid;		/* 27 - Emulated ASID */
	uint32_t vmmu_handle;	/* 28 - Vmmu Handle */

	/* reserved fields for future use */

	uint32_t reserved[7];	/* 29: Reserved */
};

/*
 *
 * vb_arch_stat_regs - Virtual core PPC emulated status registers
 *
 * Virtual board emulated CPU status registers
 *
 * PPC Status structure graphical illustration
 *
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
 *       | reserved[8]   |      8 x 32bits for future enhancements
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
	 * registers saved by hypervisor for all interrupts exceptions
	 * before setting the PC in the virtual board to the exception/interrupt
	 * vector address
	 */

	uint32_t srr0; /* 0: PC at time of the interrupt */
	uint32_t srr1; /* 1: MSR at time of the interrupt */
	uint32_t cr;   /* 2: Condition registers at time of the interrupt */
	uint32_t lr;   /* 3: Link register at time of the interrupt */
	uint32_t r3;   /* 4: R3 at time of the interrupt */
	uint32_t r4;   /* 5: R4 at time of the interrupt */

	/* Registers saved by hypervisor during exception processing only */

	uint32_t mcsrr0; /*  6: */
	uint32_t mcsrr1; /*  7: */
	uint32_t esr;    /*  8: */
	uint32_t mcsr;   /*  9: */
	uint32_t mcar;   /* 10  */
	uint32_t dear;   /* 11: */
	uint32_t emsr;   /* 12: */
	uint32_t esrr0;  /* 13: */
	uint32_t esrr1;  /* 14: */
#if (CPU==PPC85XX)
	uint32_t spefscr;  /* 15: */
#else
	uint32_t reserved1; /* 15 */
#endif
	uint32_t asid;     /* 16: */
	uint32_t vmmu_handle; /* 17 */
	/* 
	 * keep this part in a separate structure in order to keep the offsets
	 * to remain the same as in VB_STATUS
	 */

	/* reserved fields for future use */

	uint32_t reserved2[5]; /* 18 - 22  */

	/* Configuration registers (only updated upon board creation */

	uint32_t svr;     /* 23 */
	uint32_t pir;     /* 24 */
	uint32_t pvr;     /* 25 */
	uint32_t hid0;    /* 26 */
	uint32_t hid1;    /* 27 */
	uint32_t bucsr;   /* 28 */
	uint32_t l1csr0;  /* 29 */
	uint32_t l1csr1;  /* 30 */

	/* reserved fields for future use */

	uint32_t reserved3[9]; /* 31 - 39 */

};


/*
 * VBI_HREG_SET - hardware register set, for read/write
 *
 * Used by vbi_vb_read_reg/vbi_vb_write_reg to read/write registers in
 * another VB
 *
 */

#define GREG_NUM	32	/* has 32 32/64-bit data registers */

typedef struct
{
	uint32_t gpr[GREG_NUM];	/* general purpose registers */
	uint32_t msr;		/* machine state register */
	uint32_t lr;		/* link register */
	uint32_t ctr;		/* count register */
	uint32_t pc;		/* program counter */
	uint32_t cr;		/* condition register */
	uint32_t xer;		/* fixed-point exception register */
	uint32_t spefscr;	/* SPE floating-point status & ctrl reg */
	uint32_t casid;		/* PPC405, Book E PID and PPC860 M_CASID */
} VBI_HREG_SET;


/*
 *
 * VBI_HREG_SET_64 - hardware register set, for read/write
 *
 * Used by vbiVbRegisterComplexRead/vbiVbRegisterComplexWrite to read/write
 * registers in another VB
 * 
 */

typedef struct
{
	uint64_t gpr[GREG_NUM];	/* general purpose registers */
	uint64_t msr;		/* machine state register */
	uint64_t lr;		/* link register */
	uint64_t ctr;		/* count register */
	uint64_t pc;		/* program counter */
	uint64_t cr;		/* condition register */
	uint64_t xer;		/* fixed-point exception register */
	uint64_t spefscr; 	/* SPE floating-point status & ctrl reg */
	uint64_t casid;		/* PPC405, Book E PID and PPC860 M_CASID */
} HREG_SET_64;

/* VBI_HREG_SET_64 is same as HREG_SET_64 in ppc */
typedef HREG_SET_64 VBI_HREG_SET_64;


/* complex register set definition */

typedef union
{
	VBI_HREG_SET    hreg32;	/* 32 bit register set */
	VBI_HREG_SET_64 hreg64;	/* 64 bit register set */
} VBI_HREG_SET_CMPLX;


typedef struct
{
	uint32_t vbiRegType;
	VBI_HREG_SET_CMPLX vbiRegSet;
} VBI_HREG_SET_CMPLX_QUALIFIED;

/* mdio messages */
#define VBI_MDIO_READ		MDIO_READ
#define VBI_MDIO_WRITE		MDIO_WRITE
#define VBI_BSP_CLK_FREQ	BSP_CLK_FREQ
#define VBI_BSP_CORE_FREQ	BSP_CORE_FREQ

/* This should exceed the number of IVOR registers defined in the hardware
 * It also defined the size of the excVectorTable size
 * VBI_ARCH_MAX_EXC_OFFSETS * 256 == 0x4000
 */
#define VBI_ARCH_MAX_EXC_OFFSETS	64

typedef struct
{
	uint32_t excOffset[VBI_ARCH_MAX_EXC_OFFSETS];
} VBI_EXC_OFFSETS_TABLE;

extern uint32_t vbi_set_exc_offset(VBI_EXC_OFFSETS_TABLE *excOffsetsTable);
extern uint32_t vbi_get_exc_offset(VBI_EXC_OFFSETS_TABLE *excOffsetsTable);

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

	union
	{
		struct
		{
			uint32_t bus;
			uint32_t phyAddr;
			uint32_t regNum;
			uint32_t page;
		} mdioRead;
		struct
		{
			uint32_t bus;
			uint32_t phyAddr;
			uint32_t regNum;
			uint32_t page;
			uint32_t dataVal;
		} mdioWrite;
#if 0 /* not supported for now */
		struct
		{
			uint32_t bus;
			uint32_t vbIntNum;
			HY_CTX *pCtx;
		} mdioIntEnable;
		struct
		{
			uint32_t bus;
			uint32_t vbIntNum;
			HY_CTX *pCtx;
		} mdioIntDisable;
#endif
	} arg;
} VBI_BSP_MSG;

typedef struct
{
	VBI_MSG_HEADER hdr;			/* message header */
	uint32_t   status;			/* request completion status */
	uint32_t   dataVal;
} VBI_BSP_MSG_REPLY;

extern void vbi_load_ctx(void);

#else /*_ASMLANGUAGE */

/*
 *
 * VBI_CTX_LOAD - Load a guest context
 *
 * This routine is implemented as an assembly macro since it's callers can't
 * reference C functions. This macro  makes a hypercall to load a context for
 * a guest * OS. The guest OS running on the core is expected to store a set
 * of registers that represent the new context to load in the it's control
 * structure then call VBI_CTX_LOAD() macro to switch to the new context.
 * The Following are the registers loaded from the control structure:
 *
 * VB_CONTROL_SRR0	     ------->  SRR0
 *
 * VB_CONTROL_SRR1	     -------> SRR1
 *
 * VB_CONTROL_CR	     -------> CR
 *
 * VB_CONTROL_R0	     -------> R0
 *
 * VB_CONTROL_EMSR	     -------> MSR
 *
 * VB_CONTROL_DBCR0	     -------> DBCR0
 *
 * VB_CONTROL_NEW_INT_DISABLE -------> VB_CONTROL_INT_DISABLE
 *
 * If interrupts are being reenabled then we ensure that any pending
 * interrupts are delivered before the new context is actived.
 *
 * Note that registers that are not saved by Hypervisor must be loaded
 * by the guest OS.
 *
 */

#define VBI_CTX_LOAD(reg)			    \
	lis reg, HI(VBI_SYS_ctx_load);		    \
	ori reg, reg, LO(VBI_SYS_ctx_load);	    \
	sc

/*
 * VBI_INT_VCORE_LOCK - lock a core's interrupts macro
 *
 * This macro disables the currently running core interrupts and returns the
 * previous interrupts state. The interrupt status field in wr_vb_control
 * structure at offset VB_CONTROL_INT_DISABLE is set to -1 and the previous
 * interrupts state is loaded to the register passed in as the first argument to
 * this macro.
 * A hypercall is not needed to perform this operation.
 *
 *
 * VBI_INT_VCORE_LOCK (reg0, reg1, reg2)
 * {
 *	load -1 to reg1
 *	load wr_vb_control address to reg2
 *       read the previous value and store in reg0
 *	Disable interrupts by loading reg1 to VB_CONTROL_INT_DISABLE(reg2)
 * }
 *
 *
 * RETURNS: TRUE if interrupts were locked otherwise FALSE
 *
 */
#define VBI_INT_VCORE_LOCK(reg0, reg1, reg2)		\
	li reg1, -1;					\
	lis reg2, HIADJ(wr_vb_control);			\
	lwz reg2, LO(wr_vb_control)(reg2);		\
	lwz reg0, VB_CONTROL_INT_DISABLE(reg2);		\
	stw reg1, VB_CONTROL_INT_DISABLE(reg2)

/*
 * VBI_INT_VCORE_UNLOCK - unlock a core's interrupts
 *
 * This macro is implemented in assembly to enable a core's interrupts. It
 * enables the interrupts by clearing to zero the value in the core's control
 * structure and and makes a fast hypercall if need be. A hypercall is made only
 * if found that interrupts are pending. The pending interrupts are checked by
 * reading the wr_vb_status structure at offset VB_STATUS_INT_PENDING. A
 * positive integer greater than zero indicates that interrupts are pending
 * which causes this macro to make a hypercall in order to drain the pending
 * interrupts. This macro expects two general purpose registers. The first
 * register is used to load the value to store and the second is used for
 * holding the destination address of the control or status structure.
 *
 *
 * VBI_INT_VCORE_UNLOCK (reg0, reg1)
 * {
 *	load "0" to reg1
 *	load address of wr_vb_control in reg0
 *	store reg1 to VB_CONTROL_INT_DISABLE(reg0)
 *
 * checkstatus:
 *	load address of wr_vb_status in reg0
 *	load VB_STATUS_INT_PENDING(reg0) in reg1
 *	check if reg1 is equal to zero
 *	if true return
 *	otherwise send an hypercall
 *	go to checkstatus
 * }
 *
 */
#define VBI_INT_VCORE_UNLOCK(reg0, reg1);			\
	lis	reg0, HIADJ(wr_vb_control);			\
	lwz	reg0, LO(wr_vb_control)(reg0);			\
	li	reg1, 0;					\
	stw	reg1, VB_CONTROL_INT_DISABLE(reg0);		\
checkIntVcorePending:						\
	lis	reg0, HIADJ(wr_vb_status);			\
	lwz	reg0, LO(wr_vb_status)(reg0);			\
	lwz	reg1, VB_STATUS_INT_PENDING(reg0);		\
								\
	cmplwi  reg1, 0; /* may need to make a hypercall*/	\
	beq     endIntVcoreLock;				\
	lis	r0, HI(VBI_SYS_int_enable);			\
	ori	r0, r0, LO(VBI_SYS_int_enable);			\
	sc;							\
	b	checkIntVcorePending;				\
endIntVcoreLock:

/*
 * VBI_INT_VCORE_STATE_GET - Get interrupts state
 *
 * This macro is implemented in assembly to read the interrupt state of the
 * currently running core. It relies on a passed in general purpose register
 * to store the current state of interrupts. The status is -1 if locked
 * otherwise 0.
 *
 *
 * VBI_INT_VCORE_STATE_GET (reg0)
 * {
 *       Load the interrupt status from wr_vb_control to reg0
 * }
 *
 *
 * RETURNS: TRUE if interrupts are locked otherwise FALSE
 *
 */

#define VBI_INT_VCORE_STATE_GET(reg0)		\
	VBI_CNTRL_ADDR_GET(reg0);		\
	lwz reg0, VB_CONTROL_INT_DISABLE(reg0);

/*
 * VBI_CONFIG_ADDR_GET - Get virtual core configuration structure base address
 *
 * This macro returns the base address of the configuration structure of the
 * running core.
 *
 * RETURNS: virtual core configuration structure base address
 *
 */

#define VBI_CONFIG_ADDR_GET(reg)		\
	lis reg, HIADJ(wr_vb_config);		\
	lwz reg, LO(wr_vb_config)(reg)

/*
 * VBI_CNTRL_ADDR_GET - Get virtual core control structure base address
 *
 * This macro returns the base address of the running virtual core's control
 * structure.
 *
 */

#define VBI_CNTRL_ADDR_GET(reg)			\
	lis reg, HIADJ(wr_vb_control);		\
	lwz reg, LO(wr_vb_control)(reg)

/*
 * VBI_STATUS_ADDR_GET - Get virtual core status structure address
 *
 * This macro returns the base address of the status structure of currently
 * running core. This structure is read-only and contains a description of
 * the running virtual core. Hypervisor uses this data to inform the
 * virtual board time variant data that may be updated during hypervisor context
 * Switch. Typical that are available in the status structure are:
 *
 *
 *Timer tick counter
 *
 *Pending interrupt state
 *
 *The interrupt state before this core was scheduled
 *
 *VMMU configuration
 *
 *Virtual core registers state
 *
 *
 * RETURNS: virtual core configuration structure base address
 *
 */

#define VBI_STATUS_ADDR_GET(reg)		    \
	lis reg, HIADJ(wr_vb_status);		    \
	lwz reg, LO(wr_vb_status)(reg)

#endif /*_ASMLANGUAGE */

#endif /* _ASM_ARCH_VBI_H */
