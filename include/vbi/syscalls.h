/*
 * syscalls.h - hypervisor system calls
 *
 * Copyright (c) 2007-2011 Wind River Systems, Inc.
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

#ifndef _VBI_SYSCALLS_H
#define _VBI_SYSCALLS_H

/*
 * System call number encoding:
 *
 * 31                                                                        0
 *
 *  |               |               |        |               |               |
 *  +---------------+---------------+        +---------------+---------------+
 *  | | | | | | | | | | | | | | | | | ...... | | | | | | | | | | | | | | | | |
 *  +---------------+---------------+        +---------------+---------------+
 *   ^ ^ ^         ^ ^ ^                                                    ^
 *   | | \--------/  | \----------------------------------------------------/
 *   | |      |      |                         |
 *   | |      |      |                         |
 *   | |      |      |                         +-- System call number
 *   | |      |      |
 *   | |      |      +-- Size override
 *   | |      |
 *   | |      +-- Fast hypervisor system call number
 *   | |
 *   | +--------- Hypervisor system call
 *   |
 *   +----------- Fast hypervisor system call
 *
 *
 *
 *
 */

/* fast system call numbers handled by assembler code */
#define HY_FAST_SYSCALL_MASK	0x3f000000
#define HY_FAST_SYSCALL_SHIFT	24
#define HY_FAST_SYSCALL_BIT	0x80000000
#define HY_FAST_SYSCALL(x)	\
	((((x) << HY_FAST_SYSCALL_SHIFT) & HY_FAST_SYSCALL_MASK) | \
	 HY_FAST_SYSCALL_BIT)

/* C driven system calls */
#define HY_SYSCALL_MASK		0x007fffff
#define HY_SYSCALL_SHIFT	0
#define HY_SYSCALL_BIT		0x40000000
#define HY_SYSCALL(x)	\
	((((x) << HY_SYSCALL_SHIFT) & HY_SYSCALL_MASK) | HY_SYSCALL_BIT)

#define HY_SIZE_OVERRIDE	0x00800000

/* fast system calls */
#define VBI_SYS_tlb_flush	HY_FAST_SYSCALL(1) /* TLB flush	*/
#define VBI_SYS_icache_flush	HY_FAST_SYSCALL(2) /* instruction cache flush */
#define VBI_SYS_dcache_flush	HY_FAST_SYSCALL(3) /* data cache flush */
#define VBI_SYS_ctx_load	HY_FAST_SYSCALL(4) /* context load */
#define VBI_SYS_int_enable	HY_FAST_SYSCALL(5) /* int enable */
#define VBI_SYS_cache_text_update		    \
				HY_FAST_SYSCALL(6) /* cache text update */

#define	VBI_SYS_pic_EOI		HY_FAST_SYSCALL(7) /* PIC End of interrupt */
#define VBI_SYS_spefscr_update	HY_FAST_SYSCALL(8) /* update spefscr */


/* hypervisor services */
#define VBI_SYS_hyIoctl		 HY_SYSCALL(1)	/* hypervisor ioctl	     */
#define VBI_SYS_vmContextCreate	 HY_SYSCALL(2)	/* context create	     */
#define VBI_SYS_send		 HY_SYSCALL(3)	/* send a messages	     */
#define VBI_SYS_receive		 HY_SYSCALL(4)	/* receive a messages	     */
#define VBI_SYS_reply		 HY_SYSCALL(5)	/* reply to a messages	     */
#define VBI_SYS_panic		 HY_SYSCALL(6)	/* halt the system	     */
#define VBI_SYS_int		 HY_SYSCALL(7)	/* deliver an interrupt	     */
#define VBI_SYS_int_controller_done HY_SYSCALL(8)/* done interrupt processing*/
#define VBI_SYS_ctxctl		 HY_SYSCALL(9)	/* context control operation */

/* VMMU operations */
#define VBI_SYS_vmmu_config	HY_SYSCALL(10)	/* configure the virtual MMU */
#define VBI_SYS_vmmu_enable	HY_SYSCALL(11)	/* enable the virtual MMU    */
#define VBI_SYS_vmmu_disable	HY_SYSCALL(12)	/* disable the virtual MMU   */
#define VBI_SYS_vmmu_tlbload	HY_SYSCALL(13)	/* load a VMMU TLB entry     */
#define VBI_SYS_vmmu_tlbflush	HY_SYSCALL(14)	/* flush a VMMU TLB entry    */

/* MMU operations */
#define VBI_SYS_mmu_attr_set	HY_SYSCALL(16)	/* set physical memory attr  */
#define VBI_SYS_mmu_attr_get	HY_SYSCALL(17)	/* get physical memory attr  */

/* BSP specific interface */
#define VBI_SYS_bspIoctl	HY_SYSCALL(18)	/* BSP specific opreration   */

/* vbMgmt services */
#define VBI_SYS_vbMgmt          HY_SYSCALL(20)

/* Virtual IO APIC services */
#define VBI_SYS_vIoapicIoctl	HY_SYSCALL(21)

#define VBI_SYS_vbReset         HY_SYSCALL(22)	/* reset  vcores	*/
#define VBI_SYS_vbRestart       HY_SYSCALL(23)	/* restart vcores	*/
#define VBI_SYS_vbSuspend       HY_SYSCALL(24)	/* suspend vcores	*/
#define VBI_SYS_vbResume        HY_SYSCALL(25)	/* Resume vcores	*/
#define VBI_SYS_vbRemote        HY_SYSCALL(26)	/* Get info of board	*/
#ifdef CONFIG_WRHV_SAFETY_PROFILE
#define VBI_SYS_schedControl	HY_SYSCALL(27)  /* scheduler control    */
#define VBI_SYS_port		HY_SYSCALL(28)  /* port send and receive */
#endif

/* Additional VMMU calls */
#define VBI_SYS_vmmu_create	HY_SYSCALL(30)
#define VBI_SYS_vmmu_delete	HY_SYSCALL(31)
#define VBI_SYS_vmmu_maxasid	HY_SYSCALL(32)	/* get max asid available */

#define VBI_SYS_intRedirect     HY_SYSCALL(40)  /* vcores int redirect	*/

/* debug facilities */
#define VBI_SYS_kputs		HY_SYSCALL(50)	/* print a string to the cons*/
#define VBI_SYS_kputc		HY_SYSCALL(51)	/* print a char to the cons  */
#define VBI_SYS_ps		HY_SYSCALL(52)	/* process status display    */
#define VBI_SYS_dbgShStart	HY_SYSCALL(53)	/* start debug shell	     */

/* VTLB MMU operations */
#define VBI_SYS_vtlb_op		HY_SYSCALL(55)	/* VTLB operation */

/* Name services */
#define VBI_SYS_ns_op		HY_SYSCALL(60)	/* Name service operation */

/* remote board memory services */
#define VBI_SYS_memWrite_op	HY_SYSCALL(70)	/* memory write operation */
#define VBI_SYS_memRead_op	HY_SYSCALL(71)	/* memory read operation */

/* remote board register services */
#define VBI_SYS_RegsWrite_op	HY_SYSCALL(72)	/* register write operation */
#define VBI_SYS_RegsRead_op	HY_SYSCALL(73)	/* register read operation */

#define VBI_SYS_Pstate_set	HY_SYSCALL(74)	/* set core P state    */
#define VBI_SYS_Cstate_set	HY_SYSCALL(75)	/* set core C state    */

/* Dynamic VB create/delete and VB Migration */
#define VBI_SYS_vbCreate	HY_SYSCALL(80)
#define VBI_SYS_vbDelete	HY_SYSCALL(81)
#define VBI_SYS_vbBoardSimpleConfigGet	HY_SYSCALL(82)
#define VBI_SYS_vbBoardConfigGet	HY_SYSCALL(83)
#define VBI_SYS_vbMove		HY_SYSCALL(84)
#define VBI_SYS_vbPrioSet	HY_SYSCALL(85)

/* Dynamic modification of VB */
#define VBI_SYS_vbSharedMemoryAlloc	HY_SYSCALL(90)
#define VBI_SYS_vbSharedMemoryFree	HY_SYSCALL(91)
#define VBI_SYS_vbRamAlloc		HY_SYSCALL(92)
#define VBI_SYS_vbRamFree		HY_SYSCALL(93)

/* Max number of syscalls*/

#define VBI_SYS_max		(101 + 1)

/* hyIoctl system call supported ioctl's */
#define VBI_HYIOCTL_GETPID	 1	/* get context's pid		*/
#define VBI_HYIOCTL_GETPRIORITY	 2	/* get context's priority	*/
#define VBI_HYIOCTL_PSDISPLAY	 3	/* print context list on console*/
#define VBI_HYIOCTL_EXCBASE	 4	/* exception vector base addr	*/
#define VBI_HYIOCTL_INTBASE	 5	/* interrupt vector base addr	*/
#define VBI_HYIOCTL_GETSTATS	 6	/* get context statistics	*/
#define VBI_HYIOCTL_DEBUG_SHELL	 7	/* start the debug shell	*/
#define VBI_HYIOCTL_PADDR	 9	/* translate to physical address*/
#define VBI_HYIOCTL_EXCOFFSETS_SET  10  /* set exc vector offsets for the VB*/
#define VBI_HYIOCTL_EXCOFFSETS_GET  11	/* get exc vector offsets for the VB*/
#define VBI_HYIOCTL_GETCONFIG	    12  /* get config page address */

/* vIoapicIoctl system call supported ioctl's */
#define VBI_IOAPICIOCTL_UNMASK	1	/* unmask v io apic interrupt src */
#define VBI_IOAPICIOCTL_SEND	2	/* inject a v io apic interrupt */
#define VBI_IOAPICIOCTL_EOI	3	/* end of interrupt acknowledge */
#define VBI_IOAPICIOCTL_VCORE_SEND  4	/* Send an irq to a core	*/
/* remove this after vb_control is working, we don't need a hypercall for it: */
#define VBI_IOAPICIOCTL_MASK	10	/* mask v io apic interrupt src */

/* VBI_IOAPICIOCTL_SEND options */
#define VBI_IOAPICSEND_ALL	0	/* send interrupt to all incl self */
#define VBI_IOAPICSEND_OTHERS	1	/* send interrupt to all except self */
#define VBI_IOAPICSEND_SELF	2	/* send interrupt to self only */
#define VBI_IOAPICSEND_UNICAST	3	/* send interrupt to a only one vb*/
#define VBI_IOAPICSEND_NONE	4	/* ignore this call	       */

/* sent interrupt to a virtual flags */
#define VBI_IOAPICSEND_VCORE_NONE  0	/* there no option available for now */


/* hyCtxctl system call supported operations */
#define VBI_CTXCTL_IDLE		1	/* make current context idle	*/

/* remote VB syscall call supported operations */

#define VBI_VBREMOTE_BOARDCONFIG    1       /* get guest addr of VBCONFIG */
#define VBI_VBREMOTE_RAMSIZE        2       /* get memory size */

/* MMU protection attributes */
/* For backwards compatibility, these are preserved: */
#define VBI_MMU_PROT_READ	0x00000001	/* read allowed    */
#define VBI_MMU_PROT_WRITE	0x00000002	/* write allowed   */
#define VBI_MMU_PROT_EXECUTE	0x00000004	/* execute allowed */

/* Newer vbiMemAttrSet/Get interface: */
#define	VBI_MMU_PROT_USER_READ		0x00000001  /* user read */
#define	VBI_MMU_PROT_USER_WRITE		0x00000002  /* user write */
#define	VBI_MMU_PROT_USER_EXECUTE	0x00000004  /* user execute */
#define	VBI_MMU_PROT_SUPV_READ		0x00000008  /* supervisor read */
#define	VBI_MMU_PROT_SUPV_WRITE		0x00000010  /* supervisor write */
#define	VBI_MMU_PROT_SUPV_EXECUTE	0x00000020  /* supervisor execute */

/* attributes passed to guest besides protection */

#define VBI_MMU_ATTR_GUEST_MEM		0x80000000  /* accessible by guest  */

/* ETSEC MDIO supported ioctl's */
#define VBI_BSPIOCTL_DRV_MDIO	1		/* mdio messages */

#define VBI_BSPIOCTL_SYS_CLK	2		/* system clk frequency query */
#define VBI_BSPIOCTL_CLK_FREQ	2		/* Request system clk freq */

/* return physical cpu number given virtual number
 * - UP should always be 0
 * - SMP any vcore in the system
 */
#define VBI_BSPIOCTL_VIRT_TO_PHYS_CPU_NUM 3
 
#define MDIO_READ		1
#define MDIO_WRITE		2
#define MDIO_INT_ENABLE		3
#define MDIO_INT_DISABLE	4

/* vbi_hy_ioctl for PADDR */

#define VBI_HYIOCTL_PADDR_DMA        0x0 /* Default used for DMA */
#define VBI_HYIOCTL_PADDR_PHYS       0x1 /* When absolute phys addr needed */

/* vbMgmt commands */

#define VBI_VBMGMT_ATTACH       1       /* Attach to VB for control */
#define VBI_VBMGMT_DETACH       2       /* Detech from VB */
#define VBI_VBMGMT_SUSPEND      3       /* Suspend/halt VB */
#define VBI_VBMGMT_RESUME       4       /* Resume/start a VB */
#define VBI_VBMGMT_RESET        5       /* Reset VB */
#define VBI_VBMGMT_RESTART      6       /* Restart a VB */

/* vbMgmt error codes */

#define VBI_ERROR_CODE              int32_t
#define VBI_ERR_GENERAL             -101   /* General error */
#define VBI_ERR_INVALID_ARG         -102   /* General error */

/* vbMgmt reset macros */

/* APs don't clear mem */

#define VBI_VBMGMT_RESET_NON_CORE0		0x00000000

/* Disable ELF reloading */

#define VBI_VBMGMT_RESET_DOWNLOAD		0x00000001

/* Do not clear memory */
#define VBI_VBMGMT_RESET_CLEAR			0x00000002
#define VBI_VBMGMT_RESET_AND_START_CORE0	0x00000004

/* target vb options */
#define VBI_VB_CORES_ALL	(0x80000000)
#define VBI_VB_CORES_OTHERS	(0x40000000)

/* VTLB operation command and flags (intel-vt specific) */
#define VBI_VTLB_OP_UPDATE_PMD		1	/* 32-bit: L1 */
#define VBI_VTLB_OP_UPDATE_PTE		2	/* 32-bit: L2 */
#define VBI_VTLB_OP_DELETE_PMD		3	/* 32-bit: L1 */
#define VBI_VTLB_OP_SET_PTE_AT		4
#define VBI_VTLB_OP_SET_PTE		5
#define VBI_VTLB_OP_FLUSH_OPS		6
#define VBI_VTLB_OP_INIT		7

#define VBI_VTLB_OP_INIT64		10
#define VBI_VTLB_OP_UPDATE_PML4		11	/* 64-bit: L1 */
#define VBI_VTLB_OP_UPDATE_PDP		12	/* 64-bit: L2 */
#define VBI_VTLB_OP_UPDATE_PDE		13	/* 64-bit: L3 */
#define VBI_VTLB_OP_UPDATE_PTE		 2	/* 64-bit: L4 */
#define VBI_VTLB_OP_DELETE_PML4		15	/* 64-bit: L1 */
#define VBI_VTLB_OP_DELETE_PDP		16	/* 64-bit: L2 */
#define VBI_VTLB_OP_DELETE_PDE		17	/* 64-bit: L3 */


/* VTLB macros */
#define VBI_VTLB_OP_MAX_OPS		100
#define VBI_VTLB_OP_CR3_CACHE_ENTRIES	4

/* VTLB optimization supported options */
#define VBI_VTLB_OPTIM_ENABLED			1
#define VBI_VTLB_CR3_CACHE_ENABLED		2
#define VBI_VTLB_OPS_CACHE_ENABLED		4
#define VBI_VTLB_DIRTY_BIT_SUPPORT_ENABLED	8

/* vbi_ns_op system call supported operations */
#define VBI_NS_REGISTER		1	/* register service name    */
#define VBI_NS_UNREGISTER	2	/* unregister service name  */
#define VBI_NS_LOOKUP_OLD	3	/* look up service name	    */
#define VBI_NS_LOOKUP		4	/* look up service name	w/option */

#ifndef _ASMLANGUAGE

/* statistics structure returned by VBI_HYIOCTL_GETSTATS ioctl */

struct vbi_ctx_stats
{
	unsigned long ctx_type;		/* type of context (user, supv, etc) */
	unsigned long switchin;		/* number of times switched in       */
	unsigned long pended;		/* number of times in pend state     */
	unsigned long tsCtxSwitchOutH;  /* timestamp: context switch (high)  */
	unsigned long tsCtxSwitchOutL;  /* timestamp: context switch (low)   */
	unsigned long tsCtxSwitchInH;   /* timestamp: context switch (high)  */
	unsigned long tsCtxSwitchInL;   /* timestamp: context switch (low)   */
	unsigned long tsCtxExcInH;	/* timestamp: exception entry (high) */
	unsigned long tsCtxExcInL;	/* timestamp: exception entry (low)  */
	unsigned long reset;		/* number of times context reset     */
};

/* VTLB operation structures (x86 specific, 32-bit version) */

struct vbi_vtlb_op
{
	uint32_t op;		/* VTLB operation id */
	uint32_t arg1;		/* VTLB operation arg 1 */
	uint32_t arg2;		/* VTLB operation arg 2 */
	uint32_t arg3;		/* VTLB operation arg 3 */
};

struct vbi_vtlb_cr3_cache
{
	uint32_t guest_cr3;	/* Guest CR3 register */
	uint32_t host_cr3;	/* Host CR3 register */
};

struct vbi_vtlb_control
{
	uint32_t size;		/* VTLB Control structure size */
	uint32_t mode;		/* VTLB module */
	uint32_t vtlb_ops_ix;	/* VTLB operation index */
	struct vbi_vtlb_op vtlb_ops[VBI_VTLB_OP_MAX_OPS]; /* VTLB ops array */
	uint32_t cr3_cache_ix;	/* CR3 cache index */
				/* cr3 cache*/
	struct vbi_vtlb_cr3_cache cr3_cache[VBI_VTLB_OP_CR3_CACHE_ENTRIES];
};

/* VTLB operation structures (x86 specific, 64-bit version) */

struct vbi_vtlb_op64		/* VTLB operation */
{
	uint32_t	op;		/* VTLB operation id */
	uint64_t	arg1;		/* VTLB operation arg 1 */
	uint64_t	arg2;		/* VTLB operation arg 2 */
	uint64_t	arg3;		/* VTLB operation arg 3 */
};

struct vbi_vtlb_cr3_cache64
{
	uint64_t	guest_cr3;	/* Guest CR3 register */
	uint64_t	host_cr3;	/* Host CR3 register */
};

struct vbi_vtlb_control64
{
	uint32_t	size;		/* VTLB Control structure size */
	uint32_t	mode;		/* VTLB module */
	uint32_t	vtlb_ops_ix;	/* VTLB operation index */
					/* VTLB ops array */
	struct vbi_vtlb_op64	vtlb_ops[VBI_VTLB_OP_MAX_OPS];
	uint32_t	cr3_cache_ix;	/* CR3 cache and index */
	struct vbi_vtlb_cr3_cache64 cr3_cache[VBI_VTLB_OP_CR3_CACHE_ENTRIES];
};					/* VBI VTLB control */

/*
 * Control structure used by vbi_vb_mgmt for commands memory read, memory write,
 *  register read, and register write.
 */

typedef struct vbMgmtCtl
{
	union
	{
		struct
		{
			uint32_t *pBuffer;  /* address of target context */
			uint32_t *pAddress; /* address of calling context */
			uint32_t size;	/* number of total bytes */
			uint32_t width;	/* bus width in bytes */
		} vbMgmtMem;
		struct
		{
			uint32_t *pBuffer;  /* address of target context */
			uint32_t regSet;	/* register set */
			uint32_t reg;	/* macro to specify register */
			uint32_t size;	/* number of total bytes */
		} vbMgmtReg;
	} data;
} VBMGMT_CTL;

typedef uint32_t VBMGMT_HANDLE;
typedef uint32_t VBI_NS_HANDLE;

#endif	/* _ASMLANGUAGE */

#endif	/* _VBI_SYSCALLS_H */
