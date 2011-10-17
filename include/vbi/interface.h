/*
 * interface.h - virtual board interface header file
 *
 * Copyright (c) 2007-2010 Wind River Systems, Inc.
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

/*
DESCRIPTION
This module defines the data structures used for information flow between the
Wind River Hypervisor and a virtual board. There are three structures employed
for this purpose:

		configuration status control

The configuration structure is the first data exchanged between a given
virtual board and Hypervisor kernel. The pointers to the status and
control structures are derived from the configuration structure. It
contains pointers to the status, control and other information about
resources assigned to a virtual board.  The configuration data is shared
accross the cores in the same virtual board unlike the data in the
control and status structure which may be different for each core.  The
information in the configuration structure is static in the sense that
the data it contains remain unmodified at least during the life of a
boot cycle.

The status structure is used to inform a virtual core at runtime about
the state it is excecuting (interrupts, vmmu, elapsed time etc...).

The control structure is put in place as a fast method to pass
information from the VB to Hypervisor or vis-versa. For examples during
a virtual board context switch the control structure allows to store the
incoming virtual board context setting before the virtual board sends a
Hypercall using VBI_CTX_LOAD().

Hypervisor passes (as parameter) a pointer of the configuration
structure to the entry function of the virtual core and a boot option
flag.

The VBI library provides a initialization function vbi_init() to ensures
that Hypervisor version number is compatible with the VBI library in
use. If the versions match then the configuration, status and control
pointers are stored respectively to wr_vb_config, wr_status and
wr_vb_control.

Therefore a virtual board must always call vbi_init() before accessing
any data provided by Hypervisor nor send a hypercall to it. Refer to to
vbi_init() description in the VBI API description document for more
information.

Once the VBI library is initialized the virtual board should employ the
provided macros in order to access the fieds in wr_vb_config,
wr_vb_status and wr_vb_control data structures. Accessing these
structures via the VBI function guaranties source level compatibily
between VBI versions.

A guest OS should port vbi_init(), the access macros along with the
necessary header files where the data structrures layout is defined.

To be aware: Changing the order of the field this structure may have
serious impact on the integrity of the system. It exists hand crafted
macros to match the C structures offset. Therefore any change should be
reflected in the macros

When a virtual core boots the VMMU is not enbled but a programmer may
choose to turn-on the VMMU. In that scenario proper care must be taken
to ensure that the address where the control, status or configureation
structure is reflected in the VMMU mapping. Technically these area
should be treated as I/O region therefore it is encouraged that they are
identity mapped.

*/

#ifndef _VBI_INTERFACE_H
#define _VBI_INTERFACE_H

#include <vbi/types.h>
#include <vbi/arch.h>

/* VB versioning information */
#define	VBI_VERSION_MAJOR	2	/* major version */
#define	VBI_VERSION_MINOR	2	/* minor version */
#define	VBI_VERSION_MAINT	1	/* maintenance version */


#ifndef _MSC_TOOL

/* macro to align guest fields for a 64-bit hypervisor */
#if defined(LP64)
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			   __attribute__(( aligned(8) )) \
			   decl_var
#else
#if (__VBI_BYTE_ORDER == __VBI_LITTLE_ENDIAN)
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			    __attribute__(( aligned(8) )) \
			    decl_var; \
			    uint32_t pad_var
#else
#define VB_ALIGN_FIELD_64(decl_var, pad_var)	\
			    __attribute__(( aligned(8) )) \
			    uint32_t pad_var; \
			    decl_var
#endif
#endif

#else

/* macro to align guest fields for a 64-bit hypervisor */
#if defined(LP64)
# define VB_ALIGN_FIELD_64(decl_var, pad_var)  \
				DECLSPEC_ALIGN(8) decl_var
#else
#if (__VBI_BYTE_ORDER == __VBI_LITTLE_ENDIAN)
#define VB_ALIGN_FIELD_64(decl_var, pad_var) \
				DECLSPEC_ALIGN(8) decl_var; \
				uint32_t pad_var
#else
#define VB_ALIGN_FIELD_64(decl_var, pad_var) \
				DECLSPEC_ALIGN(8) \
				uint32_t pad_var; \
				decl_var
#endif
#endif

#endif

#undef VB_DEBUG  /* define it to turn on debugging */
#ifdef VB_DEBUG
#define VB_DEBUG_MSG(fmt, args...)    printk(fmt, ##args)
#else
#define VB_DEBUG_MSG(fmt, args...)
#endif

/* Hard limits for now */
#define VB_MAX_VIRTUAL_BOARDS		1024
#define VB_MAX_BUSES			1024

#define VB_MAX_CORES			64

/* Type definitions for all name identifer strings in the hypervisor */
#define VB_NAMELEN		64

#ifndef	_ASMLANGUAGE

#define VB_MAX_BOOTLINE_LENGTH		256

#endif

/* Virtual Interrupt Controller definitions */

/*
 * The maximum number of interrupt sources allowed is defined by
 * VB_MAX_INTERRUTS. Care must be taken if this is changed, since some
 * algorithms and data structures will require modifications and become
 * more complex
 */
#define VB_MAX_INTERRUPTS		256

/*
 * The maximum number of associated data stored with each interrupt.
 * This can be modified without any impacts, tho it should not be
 * set to 0
 */
#define VB_MAX_INTERRUPT_DATA		16

/* Structure offsets for assembler */
#if !defined(_WRHV_ARCH_HAS_CTRL_REGS)
#define VB_CONTROL_REG_STRUCT_END	0
#endif

#define VB_CONTROL_INT_DISABLE		((4*0) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_NEW_INT_DISABLE	((4*1) + VB_CONTROL_REG_STRUCT_END)
#if (__VBI_BYTE_ORDER == __VBI_BIG_ENDIAN)
#define VB_CONTROL_VMMU0_HIGH		((4*2) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU0		((4*3) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU1_HIGH		((4*4) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU1		((4*5) + VB_CONTROL_REG_STRUCT_END)
#else
#define VB_CONTROL_VMMU0		((4*2) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU0_HIGH		((4*3) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU1		((4*4) + VB_CONTROL_REG_STRUCT_END)
#define VB_CONTROL_VMMU1_HIGH		((4*5) + VB_CONTROL_REG_STRUCT_END)
#endif /* __VBI_BYTE_ORDER */

/* Bit Mask definitions for VB_STATUS_INT_PENDING */
#define VB_STATUS_INT_PENDING_INT	1	/* Virtual Interrupt */
#define VB_STATUS_INT_PENDING_TICK	2	/* Tick interrupt */
#define VB_STATUS_INT_PENDING_DIRECT_INT	4   /* Direct Interrupt */

/* Assembler offsets for vb_status */
#if !defined(_WRHV_ARCH_HAS_STATUS_REGS)
#define VB_STATUS_REG_STRUCT_END	0
#endif

#define VB_STATUS_INT_PENDING		((4*0) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_RESERVED1		((4*1) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_TIMESTAMP_HIGH	((4*2) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_TIMESTAMP_LOW		((4*3) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_OLD_INT_DISABLE	((4*4) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_RESERVED2		((4*5) + VB_STATUS_REG_STRUCT_END)
#if (__VBI_BYTE_ORDER == __VBI_BIG_ENDIAN)
#define VB_STATUS_VMMU0_HIGH		((4*6) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU0			((4*7) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU1_HIGH		((4*8) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU1			((4*9) + VB_STATUS_REG_STRUCT_END)
#else
#define VB_STATUS_VMMU0			((4*6) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU0_HIGH		((4*7) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU1			((4*8) + VB_STATUS_REG_STRUCT_END)
#define VB_STATUS_VMMU1_HIGH		((4*9) + VB_STATUS_REG_STRUCT_END)
#endif /* __VBI_BYTE_ORDER */

/* Assembler offsets for vb_config */
#if (__VBI_BYTE_ORDER == __VBI_BIG_ENDIAN)
#define VB_CONFIG_VBSTATUS		(((2+0) * 8) + 4)
#define VB_CONFIG_VBCONTROL		(((2+1) * 8) + 4)
#define VB_CONFIG_SMINFO		(((2+2) * 8) + 4)
#define VB_CONFIG_MEMINFO		(((2+3) * 8) + 4)
#define VB_CONFIG_INTINFO		(((2+4) * 8) + 4)
#else
#define VB_CONFIG_VBSTATUS		((2+0) * 8)
#define VB_CONFIG_VBCONTROL		((2+1) * 8)
#define VB_CONFIG_SMINFO		((2+2) * 8)
#define VB_CONFIG_MEMINFO		((2+3) * 8)
#define VB_CONFIG_INTINFO		((2+4) * 8)
#endif /* __VBI_BYTE_ORDER */

/* Defines for vbIntInfo irq_dir field */
#define VB_INPUT_INT	1
#define VB_OUTPUT_INT	2

/*
 * Defines for the second parameter passed to the startup program
 * in a virtual board by the hypervisor.
 */
#define VB_BOOT_COLD	1
#define VB_BOOT_WARM	2
#define VB_BOOT_RESTART	3

/* define 128 entries for all platforms */
#define VIOAPIC_MAX_REDTABLE_ENTRIES	VB_MAX_INTERRUPTS

/* define valid guest to guest register operations */

#define VBI_REG_SET_32BIT	0
#define VBI_REG_SET_64BIT	1

#ifdef CONFIG_WRHV_CERT
/*
 * define schedule control commands that is the
 * first parameter in syscall vbiSchedControlOp() which
 * decides the action of control.
 */

/* do frame scdeduler frame transition */
#define SCHED_CONTROL_TRANSITION	1

/* define frame scheduler transition definitions */
#define SCHEDULER_TRANSITION_MAJOR 1
#define SCHEDULER_TRANSITION_MINOR 2
#define SCHEDULER_TRANSITION_TICK  3

#endif

#ifndef	_ASMLANGUAGE

/*
 * vb_control - Virtual board cores control structure
 *
 * vb_control is a data type that defines a virtual core's control structure.
 * This is called the control structure because a core uses to modify it's
 * state. The code running in the context of a virtual core puts the desired
 * setting in this control and passes to Hypervisor via Hypercall mechanism.
 * It is suggested to use the provided APIs for manipulating the virtual core
 * control structure instead of directly accessing it's members.
 * Typically use cases for this structure are for following:
 *
 *
 * During context switch for setting the incoming context's register state.
 * For more information about this refer to VBI_CTX_LOAD() description in the
 * architecture supplement API documentation.
 *
 * Locking/unlocking the virtual core interrupts.
 *
 * Loading the VMMU configuration data.
 *
 * VIOAPIC configuration registers.
 *
 * The control structure definition is generic accross the various architecture
 * flavours supported by Hypervisor with the exception of the emulated
 * registers.
 *
 * There is an architecture specific extention available via a pointer of type
 * vb_arch_stat_regs to accomodate the registers unique for a given hardware
 * platform.
 *
 * However that does imply that other fields defined in this structure are used
 * in every architecture. A field is considered generic if it is usefull at
 * least to two CPU families but not necessary to all CPUs.
 *
 * The VIOAPIC is the only field that is meaningful accross all architectures.
 * Note that some architectures like Intel with VT technology don't require
 * emulated registers. The pointer of emulated registers is included only if
 * _WRHV_ARCH_HAS_CTRL_REGS flag is defined. The control structure is mapped
 * with read/write access attribute for the purpose of serving as duplex
 * communication channel between Hypervisor and a virtual core. This mechanism
 * is fall back when it is not possible to use registers for transmitting data
 * from the virtual core to Hypervisor or vis-versa. More information about
 * vb_arch_ctrl_regs may be found in the architecture supplement document.
 *
 *
 * Control structure graphical illustration
 *
 *    ______________
 *   |              |
 *   |              |<------- 64bits - pointer to arch specific registers
 *   |              |          (See architecture supplement documentation)
 *   |--------------|
 *   |              |<------- 32bits -  interrupt state flag
 *   |--------------|
 *   |              |<------- 32bits - virtual core to be loaded context's
 *   |--------------|                  interrupt state
 *   |              |
 *   |              |<------- 64bits - pointer to VMMU configuration
 *   |--------------|
 *   |              |<------- 64bits - pointer to additional VMMU
 *   |              |                  configuration (reserved for enhancement)
 *   |--------------|
 *   |              | <------ 64bits - VIOAPIC information header
 *   |              |                  (internal use only)
 *   |              |
 *   |--------------|<------- 32bits - Pending interrupt vector number
 *   |______________|
 *
 *
 */

struct vb_control
{

#ifdef  _WRHV_ARCH_HAS_CTRL_REGS
	struct vb_arch_ctrl_regs vb_control_regs;
#endif

	uint32_t irq_disable;		/*  0: -1 => Disable all interrupts */
	uint32_t next_irq_disable;		/*  1: New value for ->irq_disable */

					/*  2: VMMU 0 table for ctxt switch */
	VB_ALIGN_FIELD_64 (void *vmmu0, pad1);

					/*  3: VMMU 1 table for ctxt switch */
	VB_ALIGN_FIELD_64 (void *vmmu1, pad2);

	uint32_t level_irq_disable;	/* interrupt level enabling */

					/* Virtual I/O APIC */
	VB_ALIGN_FIELD_64 (void *vIoapic, pad3);

	uint32_t irq_pend;		/* actual virtual interrupt pending */

};


/*
 * vb_status - Virtual board's core status structure
 *
 * vb_status is a C data type structure that provides the definition of an area
 * where the state of a given core is posted by Hypervisor before a virtual
 * core is scheduled to run. The fields of this structure are architecture
 * agnostic.
 *
 * A pointer is provided to accomodate anything that is specific to the
 * underlying hardware. The architecture specific structure is included only
 * if _WRHV_ARCH_HAS_STATUS_REGS flag is defined. The data type
 * VB_STATUS_ARCH_REGS holding the architecture dependent registers is defined
 * by a file pulled vbi/arch.h at compile type according to CPU value.
 *
 * The definition of vb_arch_stat_regs can be found in the architecture
 * supplement documentation.
 *
 * The purpose of this structure is to inform a virtual core at runtime the
 * status of a core's time variant data such as:
 *
 * Interrupts state
 *
 * current interrupt state
 *
 * previous interrupt state if the core is in an ISR context
 *
 * The pending vector number if any
 *
 * A free running clock timestamp
 *
 * Virtual core elapsed timer ticks
 *
 * Current VMMU configuration
 *
 *
 * Control structure graphical illustration
 *
 *        ______________
 *       |		|
 *       |		|<-------64bits - VB_ARCH_STATUS_REGS; arch dependent
 *       |		|		structure pointer
 *       |--------------|
 *       |		|<-------32bits - irq_pend; pending vector number
 *       |--------------|
 *       |//////////////|<-------32bits - Reserved for future enhencements
 *       |--------------|
 *       |		|<-------64bits - timeStamp; free running clock ticks
 *       |		|
 *       |--------------|
 *       |		|<-------32bits - prev_irq_disable; interrupt state before
 *       |--------------|		   an interrupt was injected
 *       |		|<-------32bits -
 *       |--------------|
 *       |		|
 *       |		|<-------64bits - vmmu0; current VMMU configuration
 *       |--------------|
 *       |		|<-------64bits - vmmu1; reserved for future use
 *       |		|
 *       |--------------|
 *       |		|<-------64bits - tick_count; elapsed virtual core ticks
 *       |		|
 *       |--------------|
 *
 */

struct vb_status {

#ifdef _WRHV_ARCH_HAS_STATUS_REGS
	struct vb_arch_stat_regs vb_status_regs;
#endif

	/* Bits indicating which interrupts are pending */
	uint32_t irq_pend_type;

	/* reserved field for future use, required for timeStamp alignment */
	uint32_t reserved1;

	/* Timestamp when last interrupt was delivered */
	uint64_t timeStamp;

	/*
	 * Interrupt registers
	 * When the hypervisor interrupts a VB, it saves the following
	 * information here for use by the virtual board.
	 *
	 * value of control->irq_disable at time of the interrupt.
	 * control->irq_disable is set to -1 by WRHV at the time of delivery of
	 * the interrupt
	 */
	uint32_t prev_irq_disable;

	/* Padding to naturally align the following pointer fields */

	uint32_t   reserved2;

	/* VMMU tables which were active when an MMU exception occurred */
	VB_ALIGN_FIELD_64 (void *vmmu0, pad1);
	VB_ALIGN_FIELD_64 (void *vmmu1, pad2);

	/* Virtual Board clock tick count */
	uint64_t tick_count;

	/* number of times this core has booted */
	uint32_t boot_count;

	/* initial program load flag */
	uint32_t ipl;

	/* The simulatror flag  */
	uint32_t sim;

};

#define ALIGN_DEV_INFO_SIZE	8	/* vb_dev_info, vb_dev_int_info,
					 * vb_dev_regset_info, etc, will
					 * all start at an 8-byte alignment */

/* Configuration information for devices */
struct vb_dev_info {
	char deviceName[VB_NAMELEN];	/* the name of the GI Thread */
	char deviceTemplate[VB_NAMELEN];/* the name of the template */
	uint16_t deviceClass;		/* class: serial, net, block */
	uint16_t pad;			/* pad to pack */
	uint32_t deviceType;		/* ADD or Emulated or Passthrough */
	uint32_t numInterrupts;		/* Number of Interrupts */
	uint32_t numRegSets;		/* Number of Register Sets */
	uint32_t numDeviceTreeSources;	/* Number of Device Tree Sources */
	uint32_t intInfoOffset;		/* offset to Interrupt Info */
	uint32_t regSetInfoOffset;	/* offset to Register Set Info */
	uint32_t deviceTreeSourceInfoOffset;
};

/* Configuration information for devices - Interrupts */

struct vb_dev_int_info
{
	uint32_t indexDevice;		/* which device does this belong to */
	char intName[VB_NAMELEN];	/* Interrupt Name */
	char intType[VB_NAMELEN];	/* Interrupt Type */
	int32_t intNum;			/* Interrupt Number */
	uint32_t intSense;		/* level or edge */
	uint32_t intPolarity;		/* active high or low */
};

/* Configuration information for devices - Register Sets */

struct vb_dev_regset_info
{
	uint32_t indexDevice;		/* which device does this belong to */
	char regSetName[VB_NAMELEN];	/* RegSet Name */
	uint32_t pad;			/* pad to pack */
	uint64_t regSetAddress;		/* Guest Physical Address */
	uint64_t regSetLength;		/* Length */
	uint64_t regSetAlignment;	/* Alignment of Physical Address */
	uint64_t regSetType;		/* IO, MEM, PCI memory */
};

/* Configuration information for devices - DeviceTreeSource */

struct vb_dev_device_tree_source_info
{
	uint32_t indexDevice;		/* which device does this belong to */
	char deviceTreeSourceName[VB_NAMELEN];	/* DTS Name */
};

/* Configuration information for interrupts */
struct vb_int_info
{
	char irq_name[VB_NAMELEN];	/* the name of this interrupt */
	uint16_t irq_dir;	/* interrupt direction: INPUT_INT, OUTPUT_INT */
	uint16_t irq_num;	/* the local VB interrupt number */
	uint32_t irq_core;	/* the receiving core for this incomming int */
};

/* Information about a shared memory region */
struct vb_sm_info
{
	char name[VB_NAMELEN];	/* the name of the shared memory region */
				/* the vbphysical address of the region */
	VB_ALIGN_FIELD_64 (void *addr, pad1);
	uint32_t length;	/* the length in bytes of the region */
	uint32_t attr;		/* the MMU attributes of the region */
	uint32_t type;		/* shared memory type field */
				/* id's of connected boards */
	VB_ALIGN_FIELD_64 (void *boardIds, pad2);
};

/* Information for the memory map info */
struct vb_mem_info
{
	char name[VB_NAMELEN];	/* the name of the memory region */
				/* the vbphysical address of the region */
	VB_ALIGN_FIELD_64 (void *addr, pad1);
	uint32_t length;	/* the length in bytes of the region */
	uint32_t attr;		/* the MMU attributes of the region */
	char type[VB_NAMELEN];	/* the type of the region */
};


/*
 * Fixed information about the configuration of a Virtual Board as seen by
 * the Virtual Board itself.  The address of this structure is passed as the
 * first parameter to the virtual board when it is started at its entry point.
 */
#define ACCESS_PRIV_READ_ONLY		0
#define ACCESS_PRIV_READ_WRITE		1
#define MAX_VB_CONFIG_REGIONS		16

struct config_page_map
{
	uint32_t address;
	uint32_t size;
	uint32_t accessPriv;
};

struct vb_config
{

	/*
	 *  structure versioning information
	 */

	uint32_t major;	/* major revision number */
	uint32_t minor;	/* minor revision number */
	uint32_t maint;	/* maintenance revision number */
	uint32_t pad;

	/*
	 *  pointers to secondary configuration structures
	 */

	/* read-only status information */
	VB_ALIGN_FIELD_64 (struct vb_status *vb_status, pad1);

	/* read-write control information */
	VB_ALIGN_FIELD_64 (struct vb_control *vb_control, pad2);

	/* shared memory information (memory map) */
	VB_ALIGN_FIELD_64 (struct vb_sm_info *sharedMemoryRegionsConfigAddress, pad3);

	/* memory regions information (memory map) */
	VB_ALIGN_FIELD_64 (struct vb_mem_info *memoryRegionsConfigAddress, pad4);

	/* information about incoming and outgoing interrupt connections */
	VB_ALIGN_FIELD_64 (struct vb_int_info *interruptConfiguration, pad5);

	/*
	 *  general board specific configuration information
	 */

	uint32_t pid;		/* hypervisor context id */
	uint32_t boardID;	/* board ID from the XML configuration */

	int32_t supervisoryMode;/* board is running in privileged mode */

	char board_name[VB_NAMELEN];/* the name of this board */
	uint32_t board_type;	/* the type of this board */
	uint32_t boot_count;	/* number of times this core has booted */

				/* entry point for this VB */
	VB_ALIGN_FIELD_64 (void *reset_pc, pad6);

	/*
	 *  virtual core specific information
	 */

	uint32_t coreId;	/* virtual core cpu id */
	uint32_t numCores;	/* number of virtual cores */
	uint32_t corePrivateSize;/* size of core private memory */
				/* pointer to core private memory */
	VB_ALIGN_FIELD_64 (void *corePrivate, pad8);

	/*
	 *  main memory configuration settings
	 */

	uint32_t phys_mem_size;	/* the vbPhysical size of RAM */
	uint32_t mem_alias_addr;	/* address to which memory is aliased */
	uint32_t mem_alias_size;	/* size of aliases memory */
	uint32_t num_mem;		/* number of memory regions */

	/*
	 *  shared memory configuration settings
	 */

	/* number of shared memory regions */
	uint32_t num_sm;

	/* shared memory state information */
	VB_ALIGN_FIELD_64 (void *sharedMemoryRegionsStateAddress, pad7);

	/*
	 *  clock and interrupt configuration
	 */

	/* the frequency of the periodic tick interrupt */
	uint32_t tick_freq;	/* ticks per second */

	/* the frequency of the timestamp */
	uint32_t stamp_freq;	/* ticks per second */

	/* number of interrupts connected to this board */
	uint32_t num_ints;

	/*
	 *  misc settings
	 */

	/* OS specific boot line */
	char bootLine[VB_MAX_BOOTLINE_LENGTH];

	/* spare 32-bit fields for future use
	 * must keep the 64 bit fields after aligned
	 */

	uint32_t spare32[17];

	/* spare 64-bit fields for future use */
	uint64_t spare64[15];

	uint32_t pad12;

	/* direct function call interface table */

#if defined (_WRHV_ARCH_HAS_VB_SYSTBL)
	VB_ALIGN_FIELD_64 (struct vb_syscall *vbSyscallTable, pad9);
	VB_ALIGN_FIELD_64 (struct vb_syscall_stub2 *vbSyscallStub2, pad10);
#endif

	/* information about devices */

	VB_ALIGN_FIELD_64 (struct vb_dev_info *deviceConfiguration, pad11);

	/* number of devices connected to this board */

	uint32_t numDevices;

	/* provide mappings for all configuration pages */
	struct config_page_map configPageMap[MAX_VB_CONFIG_REGIONS];
	uint32_t configPageNum;

	/* guest type */

	uint32_t   guestOS;
};

#endif /*_ASMLANGUAGE */
#endif  /* _VBI_INTERFACE_H */
