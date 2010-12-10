/*
 * vbi.h - virtual board support definitions
 *
 * Copyright (c) 2007 - 2011 Wind River Systems, Inc.
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

#ifndef _VBI_VBI_H
#define _VBI_VBI_H

/* alignment macros for MSC */
#define _MSC_VER 0	/* change to suit */
#ifndef DECLSPEC_ALIGN
#if (_MSC_VER >= 1300) && !defined(MIDL_PASS)
#define DECLSPEC_ALIGN(x)   __declspec(align(x))
#else
#define DECLSPEC_ALIGN(x)
#endif
#endif

#ifndef SYSTEM_CACHE_ALIGNMENT_SIZE
#if defined(_AMD64_) || defined(_X86_)
#define SYSTEM_CACHE_ALIGNMENT_SIZE 64
#else
#define SYSTEM_CACHE_ALIGNMENT_SIZE 128
#endif
#endif

#ifndef DECLSPEC_CACHEALIGN
#define DECLSPEC_CACHEALIGN DECLSPEC_ALIGN(SYSTEM_CACHE_ALIGNMENT_SIZE)
#endif


#ifdef CONFIG_64BIT
#ifndef LP64
#define LP64
#endif
#endif

#include <vbi/errors.h>
#include <vbi/types.h>
#include <vbi/version.h>
#include <vbi/arch.h>
#include <vbi/interface.h>
#include <vbi/syscall.h>
#include <vbi/interrupt.h>
#include <vbi/shmem.h>

#ifndef _ASMLANGUAGE
/*
DESCRIPTION
This module contains the vbi methods prototypes and access macros to access the
various data structures.
*/

/* Access macros for VB control structure */

/*
 *
 * VBI_CNTRL_ADDR_GET - Get virtual core control structure base address
 *
 * This macro returns the base address of the running virtual core's control
 * structure.
 *
 */
#define VBI_CNTRL_ADDR_GET()        (wr_vb_control)

#define VBI_INT_STATE_GET()             \
        ((VBI_CNTRL_ADDR_GET()->irq_disable) == -1 ? TRUE: FALSE)

#define VBI_INT_STATE_SET(value)        \
        ((VBI_CNTRL_ADDR_GET())->irq_disable = value)


#define VBI_INT_PENDING_SET(value)        \
        (VBI_CNTRL_ADDR_GET()->irq_pend = value)

/*
 *
 * VBI_VIOAPIC_BASE_GET - Get the virtual I/O APIC base address
 *
 * This macro returns the virtual I/O APIC base address in the configuration
 * structure. This is a table that controls the state of individual IRQ
 * connected to a virtual board. If the underlying hardware supports an IRQ can
 * be redirected to a specific vector number. The redirection capability is not
 * used for PPC platforms since all interrupts are funneled via vector 0x500.
 * The VIOAPIC is comprised of a description header and a redirection table.
 * Each entry in the redirection table is 64 bits wide. The size of the
 * redirection table is architecture specific. See the VBI user's guide for
 * more detailed information.
 *
 * API's for controlling the VIOAPIC:
 *
 *
 * vbi_send_vioapic_irq()
 *
 * vbi_set_vioapic_vec()
 *
 * vbi_get_vioapic_vec()
 *
 * vbi_ack_vioapic_irq()
 *
 * vbi_mask_vioapic_irq()
 *
 * vbi_unmask_vioapic_irq()
 *
 */
#define VBI_VIOAPIC_BASE_GET()		\
        (VBI_CNTRL_ADDR_GET()->vIoapic)

/* VMMU0 and VMMU1 tables for context switch access macro's */
#define VBI_VMMU0_GET()                   \
        (VBI_CNTRL_ADDR_GET()->vmmu0)

#define VBI_VMMU0_SET(value)              \
        (VBI_CNTRL_VMMU0_GET() = value)

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
 * RETURNS: virtual core configuration structure base address
 *
 */
#define VBI_STATUS_ADDR_GET()       (wr_vb_status)

/*
 *
 * VBI_INT_VCORE_PENDING - Get the pending interrupt
 *
 * This returns the pending interrupts vector number for the running core.
 *
 * RETURNS: VB Interrupt Vector or 0xffff (PowerPC) Not applicable for X86
 *
 */
#define VBI_INT_VCORE_PENDING()				    \
        (VBI_CNTRL_ADDR_GET()->irq_pend)

/*
 * VBI_INT_VCORE_PENDING_TYPE_GET - Get the pending interrupts
 *
 * This returns the type of pending interrupts for the running core. This API
 * are not use by hardware with virtualization support.
 *
 * RETURNS: 2 for timer interrupt, 1 for other external interrupts or
 *	    0 if none is pending
 */
#define VBI_INT_VCORE_PENDING_TYPE_GET()            \
        (VBI_STATUS_ADDR_GET()->irq_pend_type)

/*
 * VBI_INT_VCORE_PREVSTATE_GET - Get the core's previous interrupt status
 *
 * This returns the previous interrupt state of the currently running core. This
 * flag reflects the state of interrupts before Hypervisor injected an
 * interrupt. The guest OS must restore this flag in the control structure
 * before returning from the interrupt service routine.
 *
 * RETURNS: TRUE if locked otherwise FALSE
 *
 */
#define VBI_INT_VCORE_PREVSTATE_GET()				\
        ((VBI_STATUS_ADDR_GET()->prev_irq_disable) == -1? TRUE: FALSE)

/* timeStamp field  */
#define VBI_TIMESTAMP_GET()					\
        (VBI_STATUS_ADDR_GET()->timeStamp)

/* vmm[0,1] access macro */

/*
 * VBI_VMMU_CONFIG_GET - Get a core's VMMU configuration addr
 *
 * This macro gets the core's VMMU configuration structure address. The VMMU
 * structure is a descriptor of the VMMU context with the following info:
 *
 *
 *
 * The VMMU page table base address
 *
 * The VMMU pages
 *
 * The page size granularity
 *
 * The vmmu virtual address space is restricted to 32 bits and is decoded using
 * a level-1/level-2 page table.  The virtual address is decoded as follows:
 *
 *
 *                          32-bit Virtual Address
 *        +---------------------------------------------------------+
 *        |      L1 offset       | L2 offset |    Page offset       |
 *        +---------------------------------------------------------+
 *                11 bits           9 bits           12 bits
 *                  |                 |
 *                  |                 |
 *    +-------------+                 |
 *    |                               |
 *    |                               |
 *    |           L1 Table            |            L2 Table
 *    |    2047 +----------+          |      511 +----------+
 *    |         |          |          |          |          |
 *    |         |          |          |          |          |
 *    |         |          |          |          |----------|
 *    |         |          |          |   +----->|    PTE   | 8 byte PTE
 *    |         |          |          |   |      |----------|
 *    |         |          |          |   |      |          |
 *    |         |----------| 20 bits  |   |      |          |
 *    +-------->|  L2 ptr  |----------+---+      |          |
 *              |----------|                     |          |
 *              |          |                     |          |
 *              |          |                     |          |
 *            0 +----------+                   0 +----------+
 *               2 page (8KB)                    1 page (4KB)
 *             2048 L2 pointers                 512 PTE entries
 *
 * Each page table entry is 8 bytes (2 words) and uses the following format:
 *
 * word 0 (32-bits):
 *          0 1            7 8           15 1 1 1 1 2 2 2 2 24   26 27  31
 *                                          6 7 8 9 0 1 2 3
 *         +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 *         |V|  Hypervisor  |   Reserved   |U|U|U|U|U|U|U|U| ERPN  | ATTR |
 *         | |   Reserved   |              |0|1|2|3|4|5|6|7|       |      |
 *         +-+--------------+--------------+-+-+-+-+-+-+-+-+-------+------+
 *
 *                V          - valid bit
 *                Hypervisor - reserved for use by hypervisor
 *                U0-U7      - user defined attributes
 *                ERPN       - extended real page number bits
 *                ATTR       - page attributes
 *
 * word 1 (32-bits):
 *
 *          0                                19 20      23 2 2 2 2 2 2 3 3
 *                                                         4 5 6 7 8 9 0 1
 *         +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 *         |                RPN                | Reserved |R|C|U|S|U|S|U|S|
 *         |                                   |          | | |X|X|W|W|R|R|
 *         +-----------------------------------+----------+-+-+-+-+-+-+-+-+
 *
 *                RPN        - real page number
 *                R          - page referenced bit
 *                C          - page changed bit
 *                SX,SW,SR   - supervisor mode protection bits
 *                UX,UW,UR   - user mode protection bits
 *
 *
 * RETURNS: A pointer to the cores VMMU_CONFIG structure
 *
 */
#define VBI_VMMU0_CONFIG_GET()          \
        (VBI_STATUS_ADDR_GET()->vmmu0)

#define VBI_VMMU1_CONFIG_GET()          \
        (VBI_STATUS_ADDR_GET()->vmmu1)

/*
 * VBI_TICK_COUNT_GET - Get the elapsed ticks count
 *
 * This returns the number timer ticks elapsed since the board has started.
 * The frequency of the counter is based on the virtual board configuration XML
 * file "TimerTicksFrequency" flag.
 *
 * RETURNS: count number of elapsed ticks for a VB
 *
 */
#define VBI_TICK_COUNT_GET()             \
        (VBI_STATUS_ADDR_GET()->tick_count)

/*
 * VBI_CONFIG_ADDR_GET - Get virtual core configuration structure base address
 *
 * This macro returns the base address of the configuration structure of the
 * running core.
 *
 * RETURNS: virtual core configuration structure base address
 *
 */
#define VBI_CONFIG_ADDR_GET()       (wr_vb_config)

/*
 * VBI_CONTEXT_ID_GET - Get virtual core context Id
 *
 * This macro returns the virtual core context id.
 *
 * RETURNS: virtual core context id
 *
 */
#define VBI_CONTEXT_ID_GET()             \
        (VBI_CONFIG_ADDR_GET()->pid)

/*
 * VBI_BOARD_ID_GET - Get virtual Id
 *
 * This macro returns the virtual board id that the running core is attached.
 * This is positive integer in the range [1, N]. Where N is the total number of
 * boards in the system.
 *
 * RETURNS: virtual board id;
 *
 */
#define VBI_BOARD_ID_GET()               \
        (VBI_CONFIG_ADDR_GET()->boardID)

/*
 * VBI_VCORES_COUNT_GET - Get the number of cores in a VB.
 *
 * This macro returns the number of virtual cores in the VB that the calling
 * core is attached. This is a positive integer in the range of [1:N]. Where N
 * the total number of cores in the virtual board.
 *
 * RETURNS: number of cores in a virtual board
 *
 */
#define VBI_VCORES_COUNT_GET()               \
        (VBI_CONFIG_ADDR_GET()->numCores)

/*
 *
 * VBI_VCORE_ID_GET - Get a core's id
 *
 * This macro returns the running core id. This is a positive integer in the
 * range of [0:N-1]; Where N is the total number of cores in the system.
 *
 * RETURNS: the running core id
 *
 */
#define VBI_VCORE_ID_GET()			\
        (VBI_CONFIG_ADDR_GET()->coreId)

/*
 * VBI_VCORE_PRIVMEM_SIZE_GET - Get core's private memory size
 *
 * This macro returns the size in bytes of the running core's private memory
 *
 * RETURNS: size in bytes of the core's private memory
 *
 */
#define VBI_VCORE_PRIVMEM_SIZE_GET()               \
        (VBI_CONFIG_ADDR_GET()->corePrivateSize)

/*
 * VBI_VCORE_PRIVMEM_BASE_GET - Get core's private memory base address
 *
 * This macro returns the base of the private memory associated to the running
 * core's private memory
 *
 * RETURNS: a pointer to the core's private memory
 *
 */
#define VBI_VCORE_PRIVMEM_BASE_GET()               \
        (VBI_CONFIG_ADDR_GET()->corePrivate)

/*
 * VBI_PRV_MODE_GET - Get virtual core privilege mode
 *
 * This macro returns the virtual core privilege mode. A privileged core has
 * has full privilege access to hardware. If a core is configured to run in
 * privilege mode then care must be taken to prevent negative impact to the
 * rest of the system.
 *
 * RETURNS: 1 if core has supervisor privilege otherwise 0
 *
 */
#define VBI_PRIV_MODE_GET()			    \
        (VBI_CONFIG_ADDR_GET()->supervisorMode)

/*
 * VBI_BOARD_NAME - Get a VB name
 *
 * This macro returns the name of the virtual board that the running core is
 * attached. This is a NULL terminated string of a maximum length of 64 bytes.
 *
 * RETURNS: a string representing the board name
 *
 */
#define VBI_BOARD_NAME_GET()			    \
        (VBI_CONFIG_ADDR_GET()->board_name)

#define VBI_BOARD_TYPE_GET()			    \
        (VBI_CONFIG_ADDR_GET()->board_type)

/*
 * VBI_BOOT_COUNT_GET - Get the number of times this board has booted
 *
 * This macro returns the number of times a board has booted. This may used to
 * determine if the .bss data region needs to clear after a reset. This is a
 * zero value since the board has booted at least one time to call this API.
 *
 * RETURNS: a positive number greater than 0
 *
 */
#define VBI_BOOT_COUNT_GET()			    \
        (VBI_STATUS_ADDR_GET()->boot_count)

/*
 *
 * VBI_MEM_SIZE_GET - Get the virtual board ram size
 *
 * This macro returns the size of ram memory in bytes that a virtual board
 * has been assigned. If a virtual board has more than one core then it's memory
 * is shared among all cores. Hypervisor maps the size of memory provided in the
 * board's XML configuration file "RamSize" flag.
 * This is a virtual memory region from Hypervisor perspective but serves as the
 * phyisical memory for the virtual board.
 *
 * The size returned by this macro does not include the following:
 *
 *
 *
 * Private memory of each core attached to this virtual board
 *
 * The regions that contain the configuration, status and control structures
 *
 * The shared memory with other boards
 *
 * RETURNS: The size of memory in bytes for virtual board
 *
 */
#define VBI_MEM_SIZE_GET()			    \
        (VBI_CONFIG_ADDR_GET()->phys_mem_size)

/*
 * VBI_MEM_ALIAS_SIZE_GET - Get the size of aliased ram
 *
 * This macro returns a virtual board's aliased memory size.
 *
 * RETURNS: The aliased memory size.
 *
 */
#define VBI_MEM_ALIAS_SIZE_GET()		    \
        (VBI_CONFIG_ADDR_GET()->mem_alias_size)

/*
 * VBI_MEM_ALIAS_ADDR_GET - Get the virtual board ram alias address
 *
 * This macro returns a virtual board's ram alias base address. This is a
 * physical address from the board's perspective but a virtual address mapped
 * to the board's ram address from Hypervisor's perspective.
 * Hypervisor maps this address based on "RamAliasAddr" value in the board's
 * XML configuration file. The virtual board's ram and this address point to the
 * same physical address. This is convenient mechanism for a virtual board that
 * needs to remap it's physical address to a different address.
 *
 * RETURNS: The base address of ram alias address
 *
 */
#define VBI_MEM_ALIAS_ADDR_GET()			    \
        (VBI_CONFIG_ADDR_GET()->mem_alias_addr)


/*
 * VBI_MEM_ENTRY_RTN - Get a core entry point function
 *
 * This macro returns the address of the entry point function of a particular
 * core. This address is derived from the guest OS binary image a compile time
 * while creating the hypervisor system.elf image. Hypervisor calls this
 * function and passes it to the configuration address and the boot flag when
 * launching a virtual core.
 *
 * RETURNS: The entry point function of a virtual core
 *
 */
#define VBI_ENTRY_RTN_GET()			    \
        (VBI_CONFIG_ADDR_GET()->reset_pc)

/*
 * VBI_TICK_TIMER_FREQ_GET - Get a core timer tick frequency
 *
 * This macro returns a core's timer clock frequency. It is the number of timer
 * ticks per second. It is based on the "TickTimerFrequency" flag specified in
 * the board's XML configuration file.
 * The timer tick interrupt is delivered to only the running core of a VB. If a
 * core is not running the counter in the status page is incremented.
 * If "TickTimerFrequency" is set to a value of 0 for a board it  disables the
 * timer tick.
 *
 * RETURNS: The number of ticks per second
 *
 */
#define VBI_TICK_TIMER_FREQ_GET()		        \
        (VBI_CONFIG_ADDR_GET()->tick_freq)

#define VBI_TIMESTAMP_FREQ_GET()			\
        (VBI_CONFIG_ADDR_GET()->stamp_freq)

/*
 * VBI_BOOTLINE_ADDR_GET - Get a virtual board bootline parameters address
 *
 * The bootline parameters is a string that contains configuration data that a
 * user may specify in the virtual XML configuration "BootLine" flag. A guest
 * OS running on Hypervisor must use this bootline instead of it's native
 * bootline parameters. For example VxWorks bootline parameters are not passed
 * to the VB running VxWorks. The size of the bootline parameters string is
 * determined by VB_MAX_BOOTLINE_LENGTH at compile time flag. The bootline is
 * NULL terminated string.
 *
 */
#define VBI_BOOTLINE_ADDR_GET()				\
        (VBI_CONFIG_ADDR_GET()->bootLine)

#define VBI_PANIC(vector, halt)					\
{								\
	printk("core%d: unhandled exception: 0x%x\n",		\
		wr_vb_config->coreId, vector);			\
	if (halt)						\
		vbi_panic("unhandled exception");		\
}

struct vbi_clk_hook
{
	void (*rtn)(void *pArg);
	void * pArg;
};

/* externs */
extern struct vb_config *wr_vb_config;
extern struct vb_status *wr_vb_status;
extern struct vb_control *wr_vb_control;
extern void vbi_init(struct vb_config *config);
extern void vbi_exc_stub(void);
extern void ASSERT_FN(const char *, const char *, int);
extern void vbi_idle(uint64_t);
extern void *vbi_get_vioapic_addr(void);
extern int32_t vbi_set_vioapic_vec(int32_t irq, int32_t vector);
extern int32_t vbi_get_vioapic_vec(int32_t irq);
extern int32_t vbi_unmask_vioapic_irq(int32_t irq);
extern int32_t vbi_mask_vioapic_irq(int32_t irq);
extern int32_t vbi_ack_vioapic_irq(int32_t irq);
extern int32_t vbi_send_vioapic_irq(int32_t irq, uint32_t filter,
				     uint32_t vb);
extern int32_t vbi_redir_vioapic_irq(int32_t irq, int32_t tCore);
extern int32_t vbi_send_vcore_vioapic_irq(int32_t irq, uint32_t vcoreSet,
					  uint32_t options);
extern int32_t vbi_get_pending_vioapic_irq(void);
extern int32_t vbi_get_guest_dma_addr(void *gaddr, uint64_t *paddr);
extern int32_t vbi_guest_phys_to_phys(void *gaddr, uint64_t *paddr);

extern void vbi_disp_vioapic(void);
extern void vbi_show_shmem(void);
extern void vbi_show_stat(void);
extern void vbi_show_ctrl(void);
extern void vbi_show_cfg(void);
extern void vbi_show_mem(void);
extern void vbi_disp_status_regs(void);
extern void vbi_disp_ctrl_regs(void);
extern size_t kprintf(const char *, ...);
extern int32_t vbi_vb_find_board_config(uint32_t board_id, int32_t core_id,
						void *paddr);
extern void vbi_show_config_page_map(void);

#ifdef CONFIG_WRHV_COREVBI_ONLY
static inline void vbi_di_eoi(void)
{
	VBISTAT_VERBOSE(vbi_di_eoi);
	return;
}
#else
extern asmlinkage void vbi_di_eoi(void);
#endif

#endif	/* _ASMLANGUAGE */

#endif  /* _VBI_VBI_H */
