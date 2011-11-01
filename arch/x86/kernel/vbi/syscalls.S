/*
 * syscalls.S - hypervisor system calls
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
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

#define _ASMLANGUAGE

#include <asm/reg_vbi.h>
#include <asm/vbi.h>
#include <vbi/syscall.h>

/* globals */
.globl vbi_flush_tlb
.text
.balign 16

#ifdef LP64
#define VBI_STACK_FRAME_SIZE   8

#define ARG1 %rdi
#define ARG2 %rsi
#define ARG3 %rdx
#define ARG4 %rcx
#define ARG5 %r8
#define ARG6 %r9
#define ARG7(offset) offset(%rsp)
#define ARG8(offset) 0x8+offset(%rsp)

#define vbi_hcall(name, call, args)			\
.globl name;						\
name:							\
	push $##args;					\
	mov $(VBI_SYS_##call | HY_SIZE_OVERRIDE),%rax;	\
	vmcall;						\
	add $VBI_STACK_FRAME_SIZE, %rsp;		\
	ret

#else	/* !LP64 */
#define VBI_STACK_FRAME_SIZE	4

#define vbi_hcall(name, call, args)		\
.globl name;					\
name:						\
	movl $VBI_SYS_##call,%eax;		\
	push $##args;				\
	vmcall;					\
	addl $VBI_STACK_FRAME_SIZE, %esp;	\
	ret

#endif	/* LP64 */

/*
 * vbi_send - Send a message to another context
 *
 * This routine makes a hypercall to send a message to the specified context
 * and waits for a reply.  The caller will block until the sender replies to
 * the sent message.
 *
 *
 */
#ifdef LP64
.globl vbi_send;
vbi_send:
	mov     ARG7(VBI_STACK_FRAME_SIZE),%rax /* ctl */
	push    %r10            /* save the value of r10 */
	mov     %rax,%r10       /* load r10 with the values of "ctl" */
	push    $7              /* number of arguments */

	mov     $(VBI_SYS_send + HY_SIZE_OVERRIDE),%rax /* system call number */
	vmcall

	pop     %r10            /* pop the number of arguments */
	pop     %r10            /* pop the value of r10 */
	ret
#else
vbi_hcall(vbi_send, send, 7)
#endif

/*
 * vbi_rx_op - Receive a message from another context
 *
 * This routine makes a hypercall and waits for a message to be received from
 * another context. It blocks until a message is received.
 *
 * RETURNS: sender context Id or an error number in case of failure
 *
 */
vbi_hcall(vbi_rx_op, receive, 4)

/*
 * vbi_reply - Reply to message received from another context
 *
 * This routine makes a hypercall in order to reply to a message received from
 * another context. A message is received from remote context by calling
 * vbi_receive(). The reply will unblock the recipient which may preempt
 * the caller.
 *
 * RETURNS: OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_reply, reply, 4)


/*
 * vbi_kputs - print a string on the kernel console
 *
 * This system call sends the specified string to the system console.
 *
 * Returns: OK or ERROR
 *
 */
vbi_hcall(vbi_kputs, kputs, 1)
vbi_hcall(safety_debug_vbi_kputs, kputs, 1)


/*
 * vbi_kputc - print a character on the kernel console
 *
 * This system call sends the specified character to the system console.
 *
 * Returns: OK or ERROR
 *
 */
vbi_hcall(vbi_kputc, kputc, 1)
vbi_hcall(safety_debug_vbi_kputc, kputc, 1)


/*
 * vbi_panic - panic the system and halt all activity
 *
 * This system call causes the hypervisor to enter a panic state and display
 * various pieces of information on the system console.  The hypervisor
 * then enters an idle state and stops all CPU processing.
 *
 * Returns: does not return
 *
 */
vbi_hcall(vbi_panic, panic, 1)


/*
 * vbi_ps - display the list of contexts on the console
 *
 * This system call sends a "ps" like output of the hypervisor contexts to
 * the system console.
 *
 * Returns: OK or ERROR
 *
 */
vbi_hcall(vbi_ps, ps, 0)


/*
 * vbi_vcore_irq_unlock - Re-enable interrupts in the virtual board
 *
 * This call re-enables interrupts in the virtual board, and calls the
 * hypervisor if interrupts are pending.  The value level is the value returned
 * by the corresponding
 *
 */
.globl vbi_vcore_irq_unlock
vbi_vcore_irq_unlock:
	sti				/* unlock interrupts */
	ret

/*
 * vbi_vcore_irq_lock - Disable interrupts in the virtual core
 *
 * This call disables interrupts in the virtual board.
 *
 * Returns: the old value of the interrupt disable
 *
 */
.globl vbi_vcore_irq_lock
vbi_vcore_irq_lock:
    pushf				/* push EFLAGS on stack */
#ifdef LP64
    pop		%rax			/* get EFLAGS in EAX */
    andq	$EFLAGS_IF,%rax
#else
    pop		%eax			/* get EFLAGS in EAX */
    and		$EFLAGS_IF,%eax		/* mask it with IF bit */
#endif
    cli					/* lock interrupts */
    ret

/*
 * vbi_vb_mgmt - virtual board management
 *
 * This routine executes the specified command on a given virtual board. The
 * possible commands are:
 *
 * VBI_VBMGMT_ATTACH
 * Attach the requesting Virtual Board to the VB management agent for
 * operations on the specified VB.
 *
 * VBI_VBMGMT_DETACH
 * Detatch the requesting Virtual Board from the VB management agent for
 * operations on the specified VB.
 *
 * VBI_VBMGMT_SUSPEND
 * Suspends target Virtual Board from operation.  Fails if Virtual Board
 * has already been suspended
 *
 * VBI_VBMGMT_RESET
 * Resume a target virtual board.  Fails if a Virtual Board has not been
 * suspended. Currently no options are supported
 *
 * VBI_VBMGMT_RESUME
 * Restarts a target Virtual Board which has Preload=0 set in the xml file.
 * Fails if Virtual Board is preloaded (Preload=1)
 *
 *
 * The fourth argument to this routine specifies an flag that must be defined
 * when executing VBI_VBMGMT_RESUME operation. Otherwise the command fails.
 * The possible flgas are:
 *   VBI_VTLB_OP_UPDATE_PMD
 *   VBI_VTLB_OP_UPDATE_PTE
 *   VBI_VTLB_OP_DELETE_PMD
 *   VBI_VTLB_OP_SET_PTE_AT
 *   VBI_VTLB_OP_SET_PTE
 *   VBI_VTLB_OP_FLUSH_OPS
 *   VBI_VTLB_OP_INIT
 *
 * RETURNS: OK or error in case of failure
 */
vbi_hcall(vbi_vb_mgmt, vbMgmt, 5)

/*
 * vbi_vb_suspend - Suspend a virtual board's core
 *
 * This routine makes a hypercall in order to suspend one or more cores
 * that exist within the specified virtual board. The target core(s)
 * enter HALT state until vbi_vb_resume() is called change the state of
 * the core(s). This function will return only after all victim cores
 * are suspended unless the opration fails to complete. The second
 * argument passed to this function specifies one or more target cores.
 * For suspending every core within the specified VB the second argument
 * must be set to VBI_VB_CORES_ALL. This implies that the core
 * requesting the suspension may also be included in the list to be
 * suspended.  To suspend everyone but the recipient then the second
 * argument passed to this function should be set to
 * VBI_VB_CORES_OTHERS. Otherwise the second argument should be a valid
 * core number within the VB. This hypercall sends a message to a given
 * hypervisor manager that provides virtual board managment service.
 *
 * RETURNS: OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_vb_suspend, vbSuspend, 2)

/*
 * vbi_vb_reset - Reset a virtual board's core
 *
 * This routine makes a hypercall in order to reset one or more cores that
 * exist within the specified virtual board. Calling this function puts the
 * target core(s) program counter to it's ENTRY function. The ENTRY function
 * is determined based on the loaded binary image. A core does not execute
 * beyond it's ENTRY function unless vbiVbRestart() is explitly called.
 * Except for core0 within the target VB where VBI_VBMGMT_RESET_AND_START_CORE0
 * option is set in the flag passed as the third argument to this routine.
 *
 * The hypercall sends a message to a manager that provides VB managment
 * services.  This function will return only after all victim cores are reset
 * unless the operation fails to complete. The order of which the victim cores
 * are reset is not determined. The second argument identifies the cores to
 * perform the operation on.  The value of the second argument should be set
 * to one of the following:
 *
 * VBI_VB_CORES_ALL: Reset all cores in the specified virtual board
 *
 * VBI_VB_CORES_OTHERS: Exclude the recipient if it belongs to the
 * victim VB
 *
 * A valid core number: Reset the specified core that exist within the VB.
 *
 * The third argument argument passed to this function specifies options that
 * are applicable only when the second argument is VBI_VB_CORES_ALL. The
 * options may be one of the following or a combination:
 *
 * VBI_VBMGMT_RESET_CLEAR: Zero out the core's memory, can only be used in
 * conjunction with VBI_VBMGMT_RESET_DOWNLOAD
 *
 * VBI_VBMGMT_RESET_DOWNLOAD: Reset the cores and reload the executable images
 *
 * VBI_VBMGMT_RESET_AND_START_CORE0: Reset and start core0 within the VB
 *
 *
 * IMPORTANT:
 * If a user chooses to restart core without reloading the executable image then
 * the data section must be restored to prevent critical errors. It is the
 * guest OS's responsibility to clear the bss data sections in such scenario.
 *
 * RETURNS: OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_vb_reset, vbReset, 3)

/*
 * vbi_vb_restart - Restart a virtual board's core
 *
 * This routine makes a hypercall in order to restart a virtual cores
 * from reset.  It's called to start running a core or cores that were
 * previously reset by calling vbi_vb_reset(). The target core(s) start(s)
 * executing from the ENTRY function retrieved from the corresponding
 * binary image.  This function will return only after  all cores are
 * out of reset unless the operation fails to complete.  The second
 * argument represents the cores to restart.  For restarting every core
 * in reset mode within the specified VB the second argument is set to
 * VBI_VB_CORES_ALL. To restart a specific core within the VB then the
 * core number must be passed in the second argument.

 * This hypercall sends a message to a manager that provides VB managment
 * services.
 *
 * RETURNS: OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_vb_restart, vbRestart, 3)

/*
 * vbi_vb_resume - Resume a virtual board's core
 *
 * This routine makes a hypercall in order to resume one or cores within
 * the specified virtual board. It reactivates a cores or cores that
 * were previously suspended by calling vbi_vb_resume(). This function
 * will return only after all victim cores are resumed unless the
 * operation fails. The order of which the cores are resumed is not
 * determined. The second argument may a magic number instead of a valid
 * core number to indicate that the operation is intended for more than
 * one core. For resuming every core within the specified VB then the
 * second argument is set to be equal to VBI_VB_RESUME_ALL.  This
 * implies to resume every core within the specified VB. Using this
 * option when some of the cores within the VB are already running is
 * not considered as programming error.
 *
 * RETURNS: OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_vb_resume, vbResume, 2)

/*
 * vbi_hy_ioctl - hypervisor ioctl call
 *
 * This system call interfaces to the general purpose hypervisor ioctl
 * function.
 *
 * Possible ioctl commands:
 *     VBI_HYIOCTL_GETPID
 *     VBI_HYIOCTL_GETPRIORITY
 *     VBI_HYIOCTL_GETSTATS
 *     VBI_HYIOCTL_PADDR
 *
 * Returns: ioctl specific value
 *
 */
vbi_hcall(vbi_hy_ioctl, hyIoctl, 5)

/*
 * vbi_io_apic_ioctl - virtual IO APIC ioctl call
 *
 * This system call interfaces to the virtual IO APIC ioctl
 * function.
 *
 * Possible ioctl commands:
 *     VBI_IOAPICIOCTL_UNMASK
 *     VBI_IOAPICIOCTL_SEND
 *     VBI_IOAPICIOCTL_MASK
 *
 * Returns: ioctl specific value
 *
 */
vbi_hcall(vbi_io_apic_ioctl, vIoapicIoctl, 3)

/*
 * vbi_io_apic_op - virtual IO APIC ioctl call
 *
 * This system call interfaces to the virtual IO APIC.
 *
 * function.
 *
 * Possible commands:
 *     VBI_IOAPICIOCTL_UNMASK
 *     VBI_IOAPICIOCTL_SEND
 *     VBI_IOAPICIOCTL_MASK
 *
 * Returns: ioctl specific value
 *
 */
vbi_hcall(vbi_io_apic_op, vIoapicIoctl, 4)


/*
 * vbi_ctx_ctl - hypervisor context control call
 *
 * This system call interfaces to the general purpose hypervisor context
 * control function.
 *
 * Possbile operations:
 *	VBI_CTXCTL_IDLE /@ Make this virtual board go idle @/
 *
 * Returns: ioctl specific value
 *
 */
vbi_hcall(vbi_ctx_ctl, ctxctl, 2)

/*
 * vbi_set_mem_attr - Set protection for a page of memory
 *
 * This routine makes a hypercall to set the MMU attribures for the specified
 * memory range. The possible attributes one or a combinarion of the following:
 *
 * VBI_MMU_PROT_READ -  allow supervisor and user read
 * VBI_MMU_PROT_WRITE -  Allow supervisor and user write
 * VBI_MMU_PROT_EXECUTE - allow supervisor and user execute
 *
 * RETURNS: OK or error number in case of failure
 *
 */
vbi_hcall(vbi_set_mem_attr, mmu_attr_set, 3)

/*
 * vbi_get_mem_attr - Get MMU page protections
 *
 * This routine makes a hypercall to retrieve the MMU attributes associated for
 * the page where the specified address is mapped. This is not the VMMU
 * attributes. A guest OS has direct access to the mapping table of its VMMU
 * therefore a hypercall is not necessary.
 *
 * RETURNS: OK or error number in case of failure
 *
 */
vbi_hcall(vbi_get_mem_attr, mmu_attr_get, 2)

/*
 * vbi_ns_op - virtua board name service call
 *
 * This system call interfaces to the virtual board
 *
 * commands:
 *	VBI_VBI_NS_REGISTER	    register service
 *	VBI_VBI_NS_UNREGISTER	    unregister service
 *	VBI_VBI_NS_LOOKUP	    lookup service
 *
 * Returns: OK or ERROR in case of failure
 *
 */
vbi_hcall(vbi_ns_op, ns_op, 6)

/*
 * vbi_vb_write_mem - copy data to a remote board's memory
 *
 * This routine makes a hypercall to copy to a remote board memory. If
 * the VBI_DCACHE_FLUSH is set in the control memory control structure
 * then this routine flushes the data caches lines corresponding to the
 * range of memory specified. If VBI_ICACHE_INV then this routine ensure
 * that the instruction cache lines corresponding to the range of
 * address is invalidated after the memory is copied. Invalidating the
 * instruction is required if data containing is updated since the
 * instruction cache is not aware of the content in data cache.
 * Therefore flushing the data cache ensures that memory contains the
 * updated data and invalidating the instruction cache ensures that the
 * stale values in the instruction cache is thrown away.  The size_in
 * parameter specifies the number of bytes desired to be copied.  The
 * size_out parameter indicates the number of bytes successfully copied.
 * A user may set the size_out parameter to zero if the output size is
 * not of interest otherwise to a value different than zero.
 *
 * RETURNS: returns OK or error number in case of failure
 *
 */
vbi_hcall(vbi_vb_write_mem, memWrite_op, 2)

/*
 * vbi_vb_read_mem - Read a virtual board's memory
 *
 * This routine makes a hypercall to read a remote board's memory. The
 * memory control structure contains information about the target memory
 * to read and the destination buffer that hypervisor must populate with
 * the data read. This routine is used to copy data from a remote VB. It
 * is the user's responsibility to ensure that the memory read is
 * accessed orthogonally.  The size_in parameter specifies the number of
 * bytes desired to be copied.  The size_out parameter indicates the
 * number of bytes successfully copied.  A user may set the size_out
 * parameter to zero if the output size is not of interest otherwise to
 * a value different than zero.
 *
 * RETURNS: returns OK or an error number in case of failure
 *
 */
vbi_hcall(vbi_vb_read_mem, memRead_op, 2)

/*
 * vbi_shell_start_debug - start the hypervisor debug shell
 *
 * This routine sends a message to the hypervisor debug shell manager in
 * order to start the WRHV shell program. The shell program spins
 * therefore does not share the processor with any other WRHV context.
 * By default a caller of this routine is detached to allow the calling
 * core to continue executing (as long as they are not scheduled to run
 * on the same processor). An optional flag VBI_SHELL_ATTACH can be
 * specified to force the caller virtual board core to block while the
 * shell program is running.
 *
 */
vbi_hcall(vbi_shell_start_debug, dbgShStart, 1)
vbi_hcall(safety_debug_vbi_shell_start_debug, dbgShStart, 1)


/*
 * vbi_vb_read_reg - Read a remote core's registers
 *
 * This routine makes a hypercall to read a remote core's registers. The
 * register control structure contains information about the registers
 * to read and the destination buffers to store them.
 *
 * RETURNS: returns OK or error number in case of failure
 *
 */
vbi_hcall(vbi_vb_read_reg, RegsRead_op, 3)

/*
 * vbi_vb_write_reg - write to a remote core's registers
 *
 * This routine makes a hypercall to write to a remote core's registers. The
 * register control structure contains the set of registers to write.
 * The user must ensure to read first the destination core's registers using
 * vbi_vb_read_reg() then write back the modified set of registers in the
 * registers control structure.
 *
 * RETURNS: returns OK or error number in case of failure
 *
 */
vbi_hcall(vbi_vb_write_reg, RegsWrite_op, 3)

/*
 * vbi_vcore_irq_redirect - redirect an irq to another vcore
 *
 * RETURNS: returns OK or error number in case of failure
 *
 */
vbi_hcall(vbi_vcore_irq_redirect, intRedirect, 2)

/*
 * vbi_vtlb_op - execute a specified VTLB operation
 *
 * This system call executes the specified VTLB operation. The possible VTLB
 * operations are:
 *
 *   VBI_VTLB_OP_UPDATE_PMD
 *   VBI_VTLB_OP_UPDATE_PTE
 *   VBI_VTLB_OP_DELETE_PMD
 *   VBI_VTLB_OP_SET_PTE_AT
 *   VBI_VTLB_OP_SET_PTE
 *   VBI_VTLB_OP_FLUSH_OPS
 *   VBI_VTLB_OP_INIT
 *
 * Returns: OK or ERROR if the VTLB operation has failed
 */
vbi_hcall(vbi_vtlb_op, vtlb_op, 4)

/*
 * vbi_vb_remote - execute a specified remote VB operation
 *
 * This system call requests for information about a remote VB. The possible
 * operations are:
 *
 *   VBI_VBREMOTE_BOARDCONFIG    returns guest address of VB_CONFIG in out
 *   VBI_VBREMOTE_RAMSIZE        returns memory size in out
 *
 */
vbi_hcall(vbi_vb_remote, vbRemote, 4)

/*
 *
 * vbi_vb_create - VB create
 *
 * options:
 *      VBI_CREATE_RESUME        /@ start VB after create @/
 *      VBI_CREATE_HALT           /@ keep VB halted after creation  @/
 *                              
 * Returns: VB BOARD_ID or 0 if failed.
 */
vbi_hcall(vbi_vb_create, vbCreate, 2)

/*
 *
 * vbi_vb_delete - VB delete
 *
 * Returns: VB BOARD_ID or 0 if failed.
 */
vbi_hcall(vbi_vb_delete, vbDelete, 1)

/*
 *
 * vbi_get_board_simple_config - Get VB basic configuration information.
 *
 * Returns: OK or ERROR.
 */
vbi_hcall(vbi_get_board_simple_config, vbBoardSimpleConfigGet, 2)

/*
 *
 * vbi_get_board_config - Get VB Configuration info and device info
 *
 * Returns: OK or ERROR.
 *
 */
vbi_hcall(vbi_get_board_config, vbBoardConfigGet, 2)

/*
 *
 * vbi_vb_move - Move a VB to another core
 *
 * Returns: OK or ERROR.
 *
 */
vbi_hcall(vbi_vb_move, vbMove, 3)

/*
 *
 * vbi_vb_set_priority - Move a VB to another priority
 *
 * Returns: OK or ERROR.
 *
 */
vbi_hcall(vbi_vb_set_priority, vbPrioSet, 3)


#ifdef CONFIG_WRHV_SAFETY_PROFILE
/*
 * vbi_sched_control_op - do schedule control
 *
 * This system call makes a syscall to do schedule manipulations in Hypervisor.
 * The type of control is defined in the first parameter, after which there are
 * 4 "void *" parameters that are interpreted and used differently in each
 * case.
 *
 * Returns: OK or ERROR if control operation is not successful
 */
vbi_hcall(vbi_sched_control_op, schedControl, 5)

#endif
