/*
 * syscalls.s - hypervisor system calls
 *
 * Copyright (c) 2007-2009 Wind River Systems, Inc.
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

#include <vbi/vbi.h>
#include <asm/vbi.h>
#include <asm/reg_wrhv.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>

#ifdef CONFIG_PPC85xx_VT_MODE	/* not currently implemented */
#define HCALL sc 1
#else
#define HCALL sc
#endif

/* globals */

FUNC_EXPORT(vbi_vcore_irq_redirect)
FUNC_EXPORT(vbi_shell_start_debug)
FUNC_EXPORT(vbi_vb_read_mem)
FUNC_EXPORT(vbi_vb_write_mem)
FUNC_EXPORT(vbi_vb_read_reg)
FUNC_EXPORT(vbi_vb_write_reg)
FUNC_EXPORT(vbi_io_apic_op)
FUNC_EXPORT(vbi_io_apic_ioctl)
FUNC_EXPORT(vbi_di_eoi)
FUNC_EXPORT(vbi_hy_ioctl)
FUNC_EXPORT(vbi_ctx_ctl)
FUNC_EXPORT(vbi_send)
FUNC_EXPORT(vbi_rx_op)
FUNC_EXPORT(vbi_reply)
FUNC_EXPORT(vbi_flush_icache)
FUNC_EXPORT(vbi_flush_dcache)
FUNC_EXPORT(vbi_update_text_cache)
#if (CPU==PPC85XX)
FUNC_EXPORT(vbi_update_spefscr)
FUNC_EXPORT(vbi_load_ctx)
#endif
FUNC_EXPORT(vbi_flush_tlb)
FUNC_EXPORT(vbi_panic)
FUNC_EXPORT(vbi_ps)
FUNC_EXPORT(vbi_kputs)
FUNC_EXPORT(vbi_kputc)
FUNC_EXPORT(vbi_vcore_irq_unlock)
FUNC_EXPORT(vbi_vcore_irq_lock)
FUNC_EXPORT(vbi_vcore_irq_state)
FUNC_EXPORT(vbi_set_exc_base)
FUNC_EXPORT(vbi_set_exc_offset)
FUNC_EXPORT(vbi_get_exc_offset)
FUNC_EXPORT(vbi_set_mem_attr)
FUNC_EXPORT(vbi_get_mem_attr)
FUNC_EXPORT(vbi_config_vmmu)
FUNC_EXPORT(vbi_enable_vmmu)
FUNC_EXPORT(vbi_disable_vmmu)
FUNC_EXPORT(vbi_create_vmmu)
FUNC_EXPORT(vbi_delete_vmmu)
FUNC_EXPORT(vbi_max_asid_vmmu)
FUNC_EXPORT(vbi_tlb_load_vmmu)
FUNC_EXPORT(vbi_tlb_flush_vmmu)
FUNC_EXPORT(vbi_vb_mgmt)
FUNC_EXPORT(vbi_ns_op)
FUNC_EXPORT(vbi_vb_suspend)
FUNC_EXPORT(vbi_vb_reset)
FUNC_EXPORT(vbi_vb_restart)
FUNC_EXPORT(vbi_vb_resume)
FUNC_EXPORT(vbi_vb_remote)
FUNC_EXPORT(vbi_vb_create)
FUNC_EXPORT(vbi_vb_delete)
FUNC_EXPORT(vbi_get_board_simple_config)
FUNC_EXPORT(vbi_get_board_config)
FUNC_EXPORT(vbi_vb_move)
FUNC_EXPORT(vbi_set_vb_priority)
FUNC_EXPORT(vbi_direct_IRQ_EOI)

_WRS_TEXT_SEG_START

#define vbi_hcall(name, call)		\
FUNC_LABEL(name)			\
	lis r0, HI(VBI_SYS_##call);	\
	ori r0, r0, LO(VBI_SYS_##call);	\
	HCALL;				\
	blr;				\
FUNC_END(name)

/*
 * sys_vbMgmt - virtual board management
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
 */
vbi_hcall(vbi_vb_mgmt, vbMgmt)


/*
 * vbi_vb_suspend - Suspend a virtual board's core
 *
 * This routine makes a hypercall in order to suspend one or more cores that
 * exist within the specified virtual board. The target core(s) enter HALT
 * state until vbi_vb_resume() is called change the state of the core(s). This
 * function will return only after all victim cores are suspended unless the
 * opration fails to complete. The second argument passed to this function
 * specifies one or more target cores. For suspending every core within the
 * specified VB the second argument must be set to VBI_VB_CORES_ALL. This
 * implies that the core requesting the suspension may also be included in the
 * list to be suspended.  To suspend everyone but the recipient then the second
 * argument passed to this function should be set to VBI_VB_CORES_OTHERS.
 * Otherwise the second argument should be a valid core number within the VB.
 * This hypercall sends a message to a given hypervisor manager that provides
 * virtual board managment service.
 *
 */
vbi_hcall(vbi_vb_suspend, vbSuspend)

/*
 * vbi_vb_reset - Reset a virtual board's core
 *
 * This routine makes a hypercall in order to reset one or more cores that
 * exist within the specified virtual board. Calling this function puts the
 * target core(s) program counter to it's ENTRY function. The ENTRY function is
 * determined based on the loaded binary image. A core does not execute beyond
 * it's ENTRY function unless vbi_vb_restart() is explitly called.  Except for
 * core0 within the target VB where VBI_VBMGMT_RESET_AND_START_CORE0 option is
 * set in the flag passed as the third argument to this routine.  The hypercall
 * sends a message to a manager that provides VB managment services.  This
 * function will return only after all victim cores are reset unless the
 * operation fails to complete. The order of which the victim cores are reset
 * is not determined. The second argument identifies the cores to perform the
 * operation on.  The value of the second argument should be set to one of the
 * following:
 *
 * VBI_VB_CORES_ALL: Reset all cores in the specified virtual board
 *
 * VBI_VB_CORES_OTHERS: Exclude the recipient if it belongs to the victim VB
 *
 * A valid core number: Reset the specified core that exist within the Virtual
 * Board.
 *
 * The third argument argument passed to this function specifies options that
 * are applicable only when the second argument is VBI_VB_CORES_ALL. The
 * options may be one of the following or a combination:
 *
 *
 * VBI_VBMGMT_RESET_CLEAR: Zero out the core's memory, can only be used in
 * conjunction with VBI_VBMGMT_RESET_DOWNLOAD
 *
 * VBI_VBMGMT_RESET_DOWNLOAD: Reset the cores and reload the executable images
 *
 * VBI_VBMGMT_RESET_AND_START_CORE0: Reset and start core0 within the VB
 *
 * IMPORTANT: If a user chooses to restart core without reloading the
 * executable image then the data section must be restored to prevent critical
 * errors. It is the guest OS's responsibility to clear the bss data sections
 * in such scenario.
 *
 */
vbi_hcall(vbi_vb_reset, vbReset)

/*
 * vbi_vb_restart - Restart a virtual board's core
 *
 * This routine makes a hypercall in order to restart a virtual cores from
 * reset.  It's called to start running a core or cores that were previously
 * reset by calling vbi_vb_reset(). The target core(s) start(s) executing from
 * the ENTRY function retrieved from the corresponding binary image.  This
 * function will return only after  all cores are out of reset unless the
 * operation fails to complete.  The second argument represents the cores to
 * restart.  For restarting every core in reset mode within the specified VB
 * the second argument is set to VBI_VB_CORES_ALL. To restart a specific core
 * within the VB then the core number must be passed in the second argument.
 *
 * This hypercall sends a message to a manager that provides VB managment
 * services.
 *
 */
vbi_hcall(vbi_vb_restart, vbRestart)

/*
 *
 * vbi_vb_resume - Resume a virtual board's core
 *
 * This routine makes a hypercall in order to resume one or cores within the
 * specified virtual board. It reactivates a cores or cores that were
 * previously suspended by calling vbi_vb_resume(). This function will return
 * only after all victim cores are resumed unless the operation fails. The
 * order of which the cores are resumed is not determined. The second argument
 * may a magic number instead of a valid core number to indicate that the
 * operation is intended for more than one core. For resuming every core within
 * the specified VB then the second argument is set to be equal to
 * VBI_VB_RESUME_ALL.  This implies to resume every core within the specified
 * VB. Using this option when some of the cores within the VB are already
 * running is not considered as programming error.
 *
 */
vbi_hcall(vbi_vb_resume, vbResume)

/*
 *
 * vbi_send - Send a message to another context
 *
 * This routine makes a hypercall to send a message to the specified context
 * and waits for a reply.  The caller will block until the sender replies to
 * the sent message.
 *
 */
vbi_hcall(vbi_send, send)

/*
 * vbi_rx_op - Receive a message from another context
 *
 * This routine makes a hypercall and waits for a message to be received from
 * another context. It blocks until a message is received.
 *
 */
vbi_hcall(vbi_rx_op, receive)

/*
 * vbi_reply - Reply to message received from another context
 *
 * This routine makes a hypercall in order to reply to a message received from
 * another context. A message is received from remote context by calling
 * vbi_receive(). The reply will unblock the recipient which may preempt
 * the caller.
 *
 */
vbi_hcall(vbi_reply, reply)

/*
 *
 * vbi_kputs - print a string on the hypervisor kernel console
 *
 * This routine makes a hypercall and prints a string of characters to to
 * hypervisor console.
 *
 */
vbi_hcall(vbi_kputs, kputs)

/*
 *
 * vbi_kputc - print a character on the hypervisor kernel console
 *
 * This routine makes a hypercall in order to print the specified character to
 * hypervisor console.
 *
 */
vbi_hcall(vbi_kputc, kputc)

/*
 * vbi_panic - halt the system and dump debug info
 *
 * This routine makes a hypercall in order to halt the system and display debug
 * information on hypervisor console. If hypervisor is configured with
 * WRHV_DISPLAY_ERROR_MSGS this routine prints the specified string to
 * hypervisor console. If hypervisor is configured with WRHV_DISPLAY_EXC_INFO
 * component the offending cores registers dump is displayed on hypervisor
 * console. If hypervisor is configured with WRHV_INCLUDE_DEBUG_MGR a message
 * is sent to the the debug manager.
 *
 */
vbi_hcall(vbi_panic, panic)

/*
 * vbi_shell_start_debug - start the hypervisor debug shell
 *
 * This routine sends a message to the hypervisor debug shell manager in order
 * to start the WRHV shell program. The shell program spins therefore does not
 * share the processor with any other WRHV context. By default a caller of this
 * routine is detached to allow the caling core to continue executing (as long
 * as the are not scheduled to run on the same processor). An optional flag
 * VBI_SHELL_ATTACH can be specified to force the caller virtual board core to
 * block while the shell program is running.
 *
 */

vbi_hcall(vbi_shell_start_debug, dbgShStart)

/*
 *
 * vbi_flush_icache - flush the instruction cache
 *
 * This routine makes a hypercall to flush the instruction cache of the calling
 * core for the specified address range.
 *
 */
vbi_hcall(vbi_flush_icache, icache_flush)

/*
 * vbi_flush_dcache - flush the specified Data cache
 *
 * This routine makes a hypercall to flush the data cache of the calling core
 * for the specified address range.
 *
 */
vbi_hcall(vbi_flush_dcache, dcache_flush)

/*
 *
 * vbi_flush_tlb - flush an MMU TLB entry
 *
 * This system call flushes the TLB associated with the specified context id
 *
 */
vbi_hcall(vbi_flush_tlb, tlb_flush)

#if (CPU==PPC85XX)
/*
 * vbi_update_spefscr(BOOL speEnable, uint32_t spefscr)
 *
 *  This fast hypercall uses r0,r3,r4,r5. Although SPE is enabled here it is
 *  up to the guest to make sure it adds this bit to the control SRR1 reg
 *  for VBI_SYS_ctx_load. VB_STATUS_SPEFSCR is updated before it returns.
 */

FUNC_LABEL(vbi_update_spefscr)
	lis     r0, HI(VBI_SYS_spefscr_update)
	ori     r0, r0, LO(VBI_SYS_spefscr_update)
	HCALL
	blr
FUNC_END(vbi_update_spefscr)

/*
 * vbi_load_ctx() - loads the context
 *
 * returns OK or ERROR
 *
 */

FUNC_LABEL(vbi_load_ctx)
	mfctr   r8
	mfcr    r7
	mflr    r6

	/* Where do we want to return to when we
	are finished calling sys_ctx_load? */
	lis     r3, leave_load_ctx@h
	ori     r3, r3, leave_load_ctx@l

	lis     r9,wr_control@ha
	lwz     r9,wr_control@l(r9)
	/* Location of where we want to come back to
	   so the HV returns us to the right spot */
	stw     r3,VB_CONTROL_SRR0(r9)
	mfcr    r11
	stw     r11,VB_CONTROL_CR(r9)
	stw     r0,VB_CONTROL_R0(r9)

	lis     r12,wr_status@ha
	lwz     r12,wr_status@l(r12)
	/* Make sure our IRQ status / control are set
	so we don't get intr when we are not expecting them */
	lwz     r11,VB_STATUS_OLD_INT_DISABLE(r12)
	stw     r11,VB_CONTROL_NEW_INT_DISABLE(r9)

	lis     r0, HI(VBI_SYS_ctx_load)
	ori     r0, r0, LO(VBI_SYS_ctx_load)
	HCALL

leave_load_ctx: /* return here - When HV calls rfi within
	the sys_ctx_load fast hypercall, it will return us
	here! */

	mtctr   r8
	mtcr    r7
	mtlr    r6
	blr
FUNC_END(vbi_load_ctx)

#endif 

/*
 * vbi_vcore_irq_unlock - unlock interrupts for running core
 *
 * This routine enables interrupts and makes a fast hypercall if pending
 * interrupts are detected for the calling core. This is a C wrapper function
 * for the assembly MACRO VBI_INT_VCORE_UNLOCK().
 *
 */
FUNC_LABEL(vbi_vcore_irq_unlock)
#ifdef CONFIG_PPC85xx_VT_MODE
	wrteei	1
#else
	lis	p0, HIADJ(wr_vb_control)
	lwz	p0, LO(wr_vb_control)(p0)
	li	p1, 0
	stw	p1, VB_CONTROL_INT_DISABLE(p0)
#endif
tryAgain:
	lis	p0, HIADJ(wr_vb_status)
	lwz	p0, LO(wr_vb_status)(p0)
	lwz	p1, VB_STATUS_INT_PENDING(p0)

	/* Perform System Call to re-enable ints if ints pending */
	cmplwi  p1, 0
	beq     0f

	lis r0, HI(VBI_SYS_int_enable)
	ori r0, r0, LO(VBI_SYS_int_enable)
	HCALL
	b	tryAgain
0:
	blr
FUNC_END(vbi_vcore_irq_unlock)

/*
 *
 * vbi_vcore_irq_lock - lock interrupts
 *
 * This routine locks interrupts for the calling core. This is a C wrapper
 * function for VBI_INT_VCORE_LOCK() assembly macro. It locks interrupts returns
 * and returns the previous state of interrupts.
 *
 */
FUNC_LABEL(vbi_vcore_irq_lock)
#ifdef CONFIG_PPC85xx_VT_MODE
	wrteei	0
#else
	li	p1, -1
	lis	p2, HIADJ(wr_vb_control)
	lwz	p2, LO(wr_vb_control)(p2)
	lwz	p0, VB_CONTROL_INT_DISABLE(p2)
	stw	p1, VB_CONTROL_INT_DISABLE(p2)
#endif
	blr
FUNC_END(vbi_vcore_irq_lock)

/*
 *
 * vbi_vcore_irq_state - Get interrupts state for running core
 *
 * This routine returns the interrupts state for the calling core. This is C
 * wrapper function for VBI_INT_VCORE_STATE_GET(). If interrupts are locked it
 * returns TRUE if interrupts are locked otherwise FALSE.
 *
 */
FUNC_LABEL(vbi_vcore_irq_state)
#ifdef CONFIG_PPC85xx_VT_MODE
#define _PPC_MSR_BIT_EE 16
	mfmsr	r3
	rlwinm.	r3, r3, 0, _PPC_MSR_BIT_EE, _PPC_MSR_BIT_EE
	beq	return_disabled
	li	r3, 0
	blr
return_disabled:
	li	r3, -1
	blr
#else
	lis	p2, HIADJ(wr_vb_control)
	lwz	p2, LO(wr_vb_control)(p2)
	lwz	p0, VB_CONTROL_INT_DISABLE(p2)
	blr
#endif

FUNC_END(vbi_vcore_irq_state)

/*
 * vbi_set_exc_base - Set the exceptions vector table base for a virtual core
 *
 * This routine changes the exceptions vector table base address for the active
 * virtual core. By default the base address of the exception table is at
 * address 0x0.  If the guest OS programmer wishes to move this address then
 * this function must be called to inform hypervisor. This function must be
 * called with interrupts disabled to prevent hypervisor to deliver interrupts
 * to an incorrect table.
 *
 */
FUNC_LABEL(vbi_set_exc_base)
	/*
	 * r3 has the base address to set; make sure it is aligned
	 * align to 32 bytes
	 */
	andi. r4, r3, 31
	cmpwi r4, 0
	bne- baseAlignError

	/* set command for the ioctl */
	mr      r4, r3
	li	r3, VBI_HYIOCTL_EXCBASE

	/* set the hypercall number */
	lis r0, HI(VBI_SYS_hyIoctl)
	ori r0, r0, LO(VBI_SYS_hyIoctl)
	HCALL
	blr
baseAlignError:
	li r3, VBI_EXCBASE_SET_ERROR
	blr

FUNC_END(vbi_set_exc_base)

/*
 * vbi_set_exc_offset - Set the exceptions vector offsetcs for a virtual core
 *
 * This routine changes the exceptions vector table offset addresses for the
 * active virtual core. By default they follow the standard PPC exception
 * offsets. vbi_get_exc_offset will provide the offsets being used for the
 * virtual board. The offset table uses an index based on IVOR number.
 * If the guest OS programmer wishes to move these address then this function
 * must be called to inform hypervisor. This function must be called with
 * interrupts disabled to prevent hypervisor to deliver interrupts to an
 * incorrect exception offset. The table is not validated by the hypervisor so
 * it must be complete and functional otherwize the VB will not behave as
 * expected.
 * For SMP systems it is recommended this API be called for each core as it is
 * started up and before it's added to the scheduler for the guest OS.
 *
 */
FUNC_LABEL(vbi_set_exc_offset)
	/* set command for the ioctl */
	mr      r4, r3
	li	r3, VBI_HYIOCTL_EXCOFFSETS_SET

	/* set the hypercall number */
	lis r0, HI(VBI_SYS_hyIoctl)
	ori r0, r0, LO(VBI_SYS_hyIoctl)
	HCALL
	blr
FUNC_END(vbi_set_exc_offset)

/*
 * vbi_get_exc_offset - Get the exceptions vector offsetcs for a virtual core
 *
 * This routine gets the exceptions vector table offset addresses for the
 * active virtual core. By default they follow the standard PPC exception
 * offsets. vbi_set_exc_offset will enable the offsets to be changed for the
 * virtual board. The offset table uses an index based on IVOR number.
 *
 */
FUNC_LABEL(vbi_get_exc_offset)
	/* set command for the ioctl */
	mr      r4, r3
	li	r3, VBI_HYIOCTL_EXCOFFSETS_GET

	/* set the hypercall number */
	lis r0, HI(VBI_SYS_hyIoctl)
	ori r0, r0, LO(VBI_SYS_hyIoctl)
	HCALL
	blr
FUNC_END(vbi_get_exc_offset)

/*
 * vbi_update_text_cache - flush data cache then invalidate instruction cache
 *
 * This routine makes a hypercall to flush the data cache then invalidates the
 * instruction cache of the calling core for the specified address range.
 *
 */
vbi_hcall(vbi_update_text_cache, cache_text_update)

/*
 * vbi_config_vmmu - configure the hypervisor virtual MMU
 *
 * This system call configures the context's virtual MMU within the hypervisor.
 *
 */
vbi_hcall(vbi_config_vmmu, vmmu_config)

/*
 * vbi_enable_vmmu - enables the virtual MMU
 *
 * This system call enables a context's virtual MMU.
 *
 */
vbi_hcall(vbi_enable_vmmu, vmmu_enable)

/*
 * vbi_disable_vmmu - disable the virtual MMU
 *
 * This system call disables a context's virtual MMU.
 *
 */
vbi_hcall(vbi_disable_vmmu, vmmu_disable)

/*
 * vbi_create_vmmu - create the virtual MMU handle
 *
 */
vbi_hcall(vbi_create_vmmu, vmmu_create)

/*
 * vbi_delete_vmmu - delete the virtual MMU handle
 *
 */
vbi_hcall(vbi_delete_vmmu, vmmu_delete)

/*
 *
 * vbi_tlb_load_vmmu - load a TLB entry into the virtual MMU
 *
 * This system call loads the TLB entries for the specified address range into
 * the virtual MMU.
 *
 */
vbi_hcall(vbi_tlb_load_vmmu, vmmu_tlbload)


/*
 *
 * vbi_tlb_flush_vmmu - load a TLB entry into the virtual MMU
 *
 * This system call flushes the TLB entries for the specified address range
 * from the virtual MMU.
 *
 */
vbi_hcall(vbi_tlb_flush_vmmu, vmmu_tlbflush)

/*
 * vbi_max_asid_vmmu - Get the maximum number of asids available
 *
 */
vbi_hcall(vbi_max_asid_vmmu, vmmu_maxasid)

/*
 *
 * vbi_hy_ioctl - hypervisor ioctl call
 *
 * This system call interfaces to the general purpose hypervisor ioctl
 * function.
 *
 * Possible ioctl commands:
 *     VBI_HYIOCTL_GETPID
 *     VBI_HYIOCTL_GETPRIORITY
 *     VBI_HYIOCTL_PSDISPLAY
 *     VBI_HYIOCTL_GETSTATS
 *     VBI_HYIOCTL_EXCBASE
 *
 */
vbi_hcall(vbi_hy_ioctl, hyIoctl)

/*
 *
 * vbi_ctx_ctl - hypervisor context control call
 *
 * This system call interfaces to the general purpose hypervisor context
 * control function.
 *
 * Possbile operations:
 *	VBI_CTXCTL_IDLE /@ Make this virtual board go idle @/
 *
 */
vbi_hcall(vbi_ctx_ctl, ctxctl)

/*
 *
 * vbi_set_mem_attr - Set protection for a page of memory
 *
 * This routine makes a hypercall to set the MMU attribures for the specified
 * memory range. The possible attributes one or a combinarion of the following:
 *
 * VBI_MMU_PROT_READ -  allow supervisor and user read
 * VBI_MMU_PROT_WRITE -  Allow supervisor and user write
 * VBI_MMU_PROT_EXECUTE - allow supervisor and user execute
 *
 */
vbi_hcall(vbi_set_mem_attr, mmu_attr_set)

/*
 * vbi_get_mem_attr - Get MMU page protections
 *
 * This routine makes a hypercall to retrieve the MMU attributes associated for
 * the page where the specified address is mapped. This is not the VMMU
 * attributes. A guest OS has direct access to the mapping table of its VMMU
 * therefore a hypercall is not necessary.
 *
 */
vbi_hcall(vbi_get_mem_attr, mmu_attr_get)

/*
 * vbi_io_apic_ioctl - virtual IO APIC ioctl call
 *
 * This system call interfaces to the virtual IO APIC ioctl
 * function. For PPC this is a nop routine for now.
 *
 * Possible ioctl commands:
 *     VBI_IOAPICIOCTL_UNMASK
 *     VBI_IOAPICIOCTL_SEND
 *     VBI_IOAPICIOCTL_MASK
 *
 */
vbi_hcall(vbi_io_apic_ioctl, vIoapicIoctl)

/*
 *
 * vbi_ns_op - virtual board name service call
 *
 * This system call interfaces to the virtual board
 *
 * commands:
 *	VBI_VBI_NS_REGISTER	   /@ register service	 @/
 *	VBI_VBI_NS_UNREGISTER	   /@ unregister service @/
 *	VBI_VBI_NS_LOOKUP	   /@ lookup service	 @/
 *
 */
vbi_hcall(vbi_ns_op, ns_op)

/*
 * vbi_vb_read_mem - Read a virtual board's memory
 *
 * This routine makes a hypercall to read a remote board's memory. The memory
 * control structure contains information about the target memory to read and
 * the destination buffer that hypervisor must populate with the data read.
 * This routine is used to copy data from a remote VB. It is the user's
 * responsability to ensure that the memory read is accessed orthogonally.  The
 * size_in parameter specifies the number of bytes desired to be copied.  The
 * size_out parameter indicates the number of bytes successfully copied.  A user
 * may set the size_out parameter to zero if the output size is not of interest
 * otherwise to a value different than zero.
 *
 */
vbi_hcall(vbi_vb_read_mem, memRead_op)

/*
 * vbi_vb_write_mem - copy data to a remote board's memory
 *
 * This routine makes a hypercall to copy to a remote board memory. If the
 * VBI_DCACHE_FLUSH is set in the control memory control structure then this
 * routine flushes the data caches lines corresponding to the range of memory
 * specified. If VBI_ICACHE_INV then this routine ensure that the instruction
 * cache lines corresponding to the range of address is invalidated after the
 * memory is copied. Invalidating the instruction is required if data
 * containing is updated since the instruction cache is not aware of the
 * content in data cache. Therefore flushing the data cache ensures that memory
 * contains the updated data and invalidating the instruction cache ensures
 * that the stale values in the instruction cache is thrown away.  The size_in
 * parameter specifies the number of bytes desired to be copied.  The size_out
 * parameter indicates the number of bytes successfully copied.  A user may set
 * the size_out parameter to zero if the output size is not of interest
 * otherwise to a value different than zero.
 *
 */
vbi_hcall(vbi_vb_write_mem, memWrite_op)

/*
 * vbi_io_apic_op - virtual IO APIC operation
 *
 * This system call interfaces to the virtual IO APIC ioctl
 * function.
 *
 * Possible ioctl commands:
 *     VBI_IOAPICIOCTL_UNMASK
 *     VBI_IOAPICIOCTL_SEND
 *     VBI_IOAPICIOCTL_MASK
 *
 */
vbi_hcall(vbi_io_apic_op, vIoapicIoctl)

/*
 * vbi_vb_read_reg - Read a remote core's registers
 *
 * This routine makes a hypercall to read a remote core's registers. The
 * register control structure contains information about the registers to read
 * and the destination buffers to store them.
 *
 */
vbi_hcall(vbi_vb_read_reg, RegsRead_op)

/*
 * vbi_vb_write_reg - write to a remote core's registers
 *
 * This routine makes a hypercall to write to a remote core's registers. The
 * register control structure contains the set of registers to write.
 * The user must ensure to read first the destination core's registers using
 * vbi_vb_read_reg() then write back the modified set of registers in the
 * registers control structure.
 *
 */
vbi_hcall(vbi_vb_write_reg, RegsWrite_op)

/*
 * vbi_vcore_irq_redirect - redirect an irq to another vcore
 */
vbi_hcall(vbi_vcore_irq_redirect, intRedirect)

/*
 * vbiDirectInterruptEOI - clear last interrupt
 *
 * This system call interfaces to the virtual board to clear the last PIC
 * interrupt enabling another interrupt to be triggered in the hardware.
 * Should be used only when Virtual board is using direct interrupts.
 *
 * C interface:
 *
 *   vbiDirectInterruptEOI ()
 *
 * Returns: NONE
 *
 */
vbi_hcall(vbi_di_eoi, pic_EOI)

/*
 * vbi_vb_remote - VB remote operations
 *
 * This system call interfaces to the virtual board and requests for
 * information about a remote VB
 *
 * commands:
 *      VBI_VBREMOTE_BOARDCONFIG         get guest addr of VB_CONFIG
 *      VBI_VBREMOTE_RAMSIZE             get memory size
 *
 */
vbi_hcall(vbi_vb_remote, vbRemote)

/*
 * vbi_direct_IRQ_EOI - clear last interrupt 
 *
 * This system call interfaces to the virtual board to clear the last PIC 
 * interrupt enabling another interrupt to be triggered in the hardware.
 * Should be used only when Virtual board is using direct interrupts.
 *
 */
FUNC_LABEL(vbi_direct_IRQ_EOI)
#ifdef CONFIG_PPC85xx_VT_MODE
	lis r0, HI(VBI_SYS_pic_EOI)
	ori r0, r0, LO(VBI_SYS_pic_EOI)
	HCALL
#endif
	blr
FUNC_END(vbi_direct_IRQ_EOI)

/*
 *
 * vbi_vb_create - VB create
 *
 * options:
 *      VBI_CREATE_RESUME        /@ start VB after create @/
 *      VBI_CREATE_HALT           /@ keep VB ahlted after creation  @/
 *
 */
vbi_hcall(vbi_vb_create, vbCreate)

/*
 *
 * vbi_vb_delete - VB delete
 *
 */
vbi_hcall(vbi_vb_delete, vbDelete)

/*
 *
 * vbi_get_board_simple_config - Get VB basic configuration information.
 *
 */
vbi_hcall(vbi_get_board_simple_config, vbBoardSimpleConfigGet)

/*
 *
 * vbi_get_board_config - Get VB configuration info and device info
 *
 */
vbi_hcall(vbi_get_board_config, vbBoardConfigGet)

/*
 *
 * vbi_vb_move - Move a VB to another core
 *
 * This system call interface moves a VB to another core
 *
 */
vbi_hcall(vbi_vb_move, vbMove)

/*
 *
 * vbi_set_vb_priority - Move a VB to another core and/or priority
 *
 * This system call interface moves a VB to another core and/or priority
 *                              
 */
vbi_hcall(vbi_set_vb_priority, vbPrioSet)
