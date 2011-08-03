/* syscalls.S - hypervisor system calls */

/*
 * Copyright (c) 2007-2008,2010 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

/*
DESCRIPTION

This file implements the hypervisor system call stubs for the hypervisor.

Note that for multiple reasons (consistent interface vs ppc, pushing work
from hypervisor into guest context, not having to access user-mode SP from
SVC, etc...) r0-r7 are loaded with the (up to) 8 arguments for the syscall.
This breaks with ARM ABI, which would normally store the first 4 args in
r0-r3 and any additional args on the stack. This is most useful for fast
syscalls - normal syscalls have to push the registers onto the stack for
the C function sysCallHandler().

For this reason and others, THIS CODE WILL PROBABLY NOT WORK IN THUMB MODE.

*/

#define _ASMLANGUAGE

#include <vbi/vbi.h>
#include <asm/vbi.h>

#define PRIVILEGED_GUEST

#if (CPU == ARMCA9)
#define USE_TRUSTZONE
#endif

/* TLB Unified (Instruction + Data) Invalidate All */
#undef TLBIALL
#define TLBIALL(x)		mcr    p15, 0, x, c8, c7, 0

/* TLB Unified (Instruction + Data) Invalidate by ASID */
#undef TLBIASID
#define TLBIASID(x)		mcr    p15, 0, x, c8, c7, 2

/* Context ID Register */
#undef CONTEXTIDR_READ
#define CONTEXTIDR_READ(x)	mrc     p15, 0, x, c13, c0, 1

#undef HASH
#define HASH #

#define INVALIDATE_CURR_ASID(temp)	\
	CONTEXTIDR_READ(temp)		;\
	and     temp, temp, HASH(0xFF)	;\
	TLBIASID(temp)

        /* globals */

        /* vbiIdle */
        FUNC_EXPORT(vbi_ctx_ctl)

        /* vbiLib */
        FUNC_EXPORT(vbi_vb_suspend)
        FUNC_EXPORT(vbi_vb_reset)
        FUNC_EXPORT(vbi_vb_restart)
        FUNC_EXPORT(vbi_vb_resume)
        FUNC_EXPORT(vbi_kputs)
        FUNC_EXPORT(vbi_kputc)
        FUNC_EXPORT(vbi_panic)
        FUNC_EXPORT(vbi_shell_start_debug)
        FUNC_EXPORT(vbi_vb_read_mem)
        FUNC_EXPORT(vbi_vb_write_mem)
        FUNC_EXPORT(vbi_vb_read_reg)
        FUNC_EXPORT(vbi_vb_write_reg)
        FUNC_EXPORT(vbi_set_mem_attr)
        FUNC_EXPORT(vbi_get_mem_attr)
        FUNC_EXPORT(vbi_vb_remote)
        FUNC_EXPORT(vbi_vb_mgmt)

	/* Dynamic VB API */
	FUNC_EXPORT(vbi_vb_create)
	FUNC_EXPORT(vbi_vb_delete)
	FUNC_EXPORT(vbi_board_simple_config_get)
	FUNC_EXPORT(vbi_board_config_get)
	FUNC_EXPORT(vbi_vb_move)
	FUNC_EXPORT(vbi_vb_priority_set)

        /* vbiMsg */
        FUNC_EXPORT(vbi_rx_op)
        FUNC_EXPORT(vbi_reply)
        FUNC_EXPORT(vbi_send)

        /* vbiNs */
        FUNC_EXPORT(vbi_ns_op)

        /* vbiPaddr */
        FUNC_EXPORT(vbi_hy_ioctl)

        /* vbiVioApic */
        FUNC_EXPORT(vbi_io_apic_op)
        FUNC_EXPORT(vbi_io_apic_ioctl)
        FUNC_EXPORT(vbi_vcore_irq_redirect)

	FUNC_EXPORT(vbi_flush_icache)
	FUNC_EXPORT(vbi_flush_dcache)
	FUNC_EXPORT(vbi_update_text_cache)

        /* vbiVmmu */
        FUNC_EXPORT(vbi_config_vmmu)
        FUNC_EXPORT(vbi_enable_vmmu)
        FUNC_EXPORT(vbi_disable_vmmu)
        FUNC_EXPORT(vbi_create_vmmu)
        FUNC_EXPORT(vbi_delete_vmmu)
        FUNC_EXPORT(vbi_tlb_load_vmmu)
        FUNC_EXPORT(vbi_flush_tlb)

        /* ARM */
        FUNC_EXPORT(vbi_vcore_irq_lock)
        FUNC_EXPORT(vbi_vcore_irq_unlock)
        FUNC_EXPORT(vbi_vcore_irq_state)
        FUNC_EXPORT(vbi_set_exc_base)
        FUNC_EXPORT(vbi_set_exc_offset)
        FUNC_EXPORT(vbi_get_exc_offset)
	FUNC_EXPORT(vbi_load_ctx)


        /*
         * This macro saves the R8 register that is used for holding the HV function
         * code, saves registers r4-r7 that are needed for passing additional args
         * to the HV, and also loads them with values from the caller's stack
         * frame.
         * It is the caller's responsibility to set the number of args that are
         * needed from the stack.
         */
        .macro  SAVEREGS regs=0
        .if (\regs == 0)
                stmfd   sp!, {r8}
        .elseif (\regs == 1)
                stmfd   sp!, {r4,r8}
                ldr     r4, [sp, #0x8]
        .elseif (\regs == 2)
                stmfd   sp!, {r4-r5,r8}
                ldr     r4, [sp, #0x0c]
                ldr     r5, [sp, #0x10]
        .elseif (\regs == 3)
                stmfd   sp!, {r4-r6,r8}
                ldr     r4, [sp, #0x10]
                ldr     r5, [sp, #0x14]
                ldr     r6, [sp, #0x18]
        .elseif (\regs == 4)
                stmfd   sp!, {r4-r7,r8}
                ldr     r4, [sp, #0x14]
                ldr     r5, [sp, #0x18]
                ldr     r6, [sp, #0x1c]
                ldr     r7, [sp, #0x20]
        .else
                .err
        .endif
        .endm

        /*
         * This macro restores registers saved by the SAVEREGS macro above.
         * The caller must specify the number of regs that were pushed - it
         * must match the value given to SAVEREGS.
         */
        .macro  RESTOREREGS     regs=0
        .if (\regs == 0)
                ldmfd   sp!, {r8}
        .elseif (\regs == 1)
                ldmfd   sp!, {r4, r8}
        .elseif (\regs == 2)
                ldmfd   sp!, {r4-r5, r8}
        .elseif (\regs == 3)
                ldmfd   sp!, {r4-r6, r8}
        .elseif (\regs == 4)
                ldmfd   sp!, {r4-r7, r8}
        .else
                .err
        .endif
        .endm

        /*
         * How a hypercall is made depends on whether trustzone is
         * implemented in the core or not.
         */
        .macro  HCALL
#ifdef USE_TRUSTZONE
	smc     #0
#else /* not trustzone */
        swi     #HY_CALL_ID
#endif
        .endm

        _WRS_TEXT_SEG_START

/******************************************************************************
*
* vbi_load_ctx - hypervisor context load call
*
* This system call implements guest-side context-switch functionality.
*
* For a thread-based guest operating system, this may provide support for
* loading a new set of general purpose registers (see below).
*
* For a process-based guest operating system, this may provide support for
* loading a new set of general purpose registers (see below) and a new
* virtual memory context.
*
* For privileged guests, this call will only affect the virtual memory
* context - all other registers may be loaded directly by the guest
* afterwards.
*
* For unprivileged guests, this call will affect the virtual memory
* context, and will also perform the context switch of the general
* purpose registers.
*
* For privileged guests, the following values will be taken from the
* VB control structure and loaded / take effect:
*
* asid (virtual ASID)		    => selects hardware ASID (CONTEXTIDR)
* asid (virtual ASID) + vmmuHandle  => selects page table (TTBR0)
*
* For unprivileged guests, the following additional values will be
* taken from the VB control structure and loaded / take effect:
*
* SP
* LR
*
* Note that all other registers must be context switched by the guest.
*
* vbiCtxLoad
*   (
*   unsigned int        type,   /@ which regs to load (ignored if priv'd) @/
*   unsigned int        psr     /@ PSR to be loaded (ignored if priv'd)   @/
*   )
*
*   For an unprivileged guest, the 'type' field carries the meaning:
*     0 => PSR switch only
*     1 => PSR + all GPRs
*     2 => PSR + all GPRs + VM
*     3 => VM only
*     >3 => error
*
* unprivileged use cases:
* - write PSR (useful for changing emulated mode to set stacks)
*             (reading spsr/lr/sp is accomplished directly)
*
* - write PSR + load GPR's + PC (thread context switch, VM on or off)
*
* - write PSR + load GPR's + PC + VM (process switch or exception return)
*
* Returns: OK or ERROR if context could not be loaded.
*/

FUNC_LABEL(vbi_load_ctx)
    /* This function is a special exception to the ABI: it will preserve r3,
     * so that no stack is necessary. Otherwise, the calling function may
     * choose to save r8 and deal with any change of stack pointer.
     *
     * Note that the LR may change with the mode, so if it is used it must
     * point to the correct return point.
     *
     * If the 'type' indicated the GPRs should be loaded, then there may
     * be no return after the hypercall (the TLB invalidation will happen
     * in the WRHV guest-side helper code).
     */

        mov     r3, r8
        ldr     r8, =VBI_SYS_ctx_load
        HCALL

#ifdef PRIVILEGED_GUEST
        INVALIDATE_CURR_ASID(r8)
#endif

        mov     r8, r3
        mov     pc, lr
FUNC_END(vbi_load_ctx)

/******************************************************************************
*
* vbi_ctx_ctl - hypervisor context control call
*
* This system call interfaces to the general purpose hypervisor context
* control function.
*
* Returns: ioctl-specific value
*
*/

FUNC_LABEL(vbi_ctx_ctl)
        /*
         * r0 - unsigned int op
         * r1 - void * arg
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_ctxctl
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_ctx_ctl)

/*******************************************************************************
*
* vbi_vb_suspend - Suspend a virtual board's core
*
* This routine makes a hypercall in order to suspend one or more cores that
* exist within the specified virtual board. The target core(s) enter HALT state
* until vbi_vb_resume() is called change the state of the core(s). This function
* will return only after all victim cores are suspended unless the opration
* fails to complete. The second argument passed to this function specifies one
* or more target cores. For suspending every core within the specified VB the
* second argument must be set to VBI_VB_CORES_ALL. This implies that the core
* requesting the suspension may also be included in the list to be suspended.
* To suspend everyone but the recipient then the second argument passed to this
* function should be set to VBI_VB_CORES_OTHERS. Otherwise the second argument
* should be a valid core number within the VB. This hypercall sends a message
* to a given hypervisor manager that provides virtual board managment service.
*
* RETURNS: OK or an error number in case of failure
*
*/

FUNC_LABEL(vbi_vb_suspend)
        /*
         * r0 - virtual board id
         * r1 - virtual core (a flag or a valid vcore id)
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vbSuspend
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_suspend)

/*******************************************************************************
*
* vbi_vb_reset - Reset a virtual board's core
*
* This routine makes a hypercall in order to reset one or more cores that exist
* within the specified virtual board. Calling this function puts the target
* core(s) program counter to it's ENTRY function. The ENTRY function is
* determined based on the loaded binary image. A core does not execute beyond
* it's ENTRY function unless vbi_vb_restart() is explitly called.
* Except for core0 within the target VB where VBI_VBMGMT_RESET_AND_START_CORE0
* option is set in the flag passed as the third argument to this routine.
* The hypercall sends a message to a manager that provides VB managment
* services.
* This function will return only after all victim cores are reset unless the
* operation fails to complete. The order of which the victim cores are reset
* is not determined. The second argument identifies the cores to perform the
* operation on.
* The value of the second argument should be set to one of the following:
*
*\ms
*\m -
* VBI_VB_CORES_ALL: Reset all cores in the specified virtual board
*\m -
* VBI_VB_CORES_OTHERS: Exclude the recipient if it belongs to the victim VB
*\m -
* A valid core number: Reset the specified core that exist within the Virtual
* Board.
*\me
*
* The third argument argument passed to this function specifies options that are
* applicable only when the second argument is VBI_VB_CORES_ALL. The options
* may be
* one of the following or a combination:
*
*\ms
*\m -
* VBI_VBMGMT_RESET_CLEAR: Zero out the core's memory, can only be used in
* conjunction with VBI_VBMGMT_RESET_DOWNLOAD
*\m -
* VBI_VBMGMT_RESET_DOWNLOAD: Reset the cores and reload the executable images
*\m -
* VBI_VBMGMT_RESET_AND_START_CORE0: Reset and start core0 within the VB
*\me
*
*
* IMPORTANT:
* If a user chooses to restart core without reloading the executable image then
* the data section must be restored to prevent critical errors. It is the guest
* OS's responsibility to clear the bss data sections in such scenario.
*
*/

FUNC_LABEL(vbi_vb_reset)
        /*
         * r0 - virtual board id
         * r1 - virtual core (a flag or a valid vcore id)
         * r2 - options
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vbReset
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_reset)

/*******************************************************************************
*
* vbi_vb_restart - Restart a virtual board's core
*
* This routine makes a hypercall in order to restart a virtual cores from reset.
* It's called to start running a core or cores that were previously reset by
* calling vbiVbReset(). The target core(s) start(s) executing from the ENTRY
* function retrieved from the corresponding binary image.
* This function will return only after  all cores are out of reset unless the
* operation fails to complete.  The second argument represents the cores to
* restart.
* For restarting every core in reset mode within the specified VB the second
* argument is set to VBI_VB_CORES_ALL. To restart a specific core within the
* VB then the core number must be passed in the second argument.
*
* This hypercall sends a message to a manager that provides VB managment
* services.
*
*/

FUNC_LABEL(vbi_vb_restart)
        /*
         * r0 - virtual board id
         * r1 - virtual core (a flag or a valid vcore id)
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vbRestart
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_restart)

/*******************************************************************************
*
* vbi_vb_resume - Resume a virtual board's core
*
* This routine makes a hypercall in order to resume one or cores within
* the specified virtual board. It reactivates a cores or cores that were
* previously suspended. This function will return only
* after all victim cores are resumed unless the operation fails. The order of
* which the cores are resumed is not determined. The second argument may a
* magic number instead of a valid core number to indicate that the operation
* is intended for more than one core. For resuming every core within the
* specified VB then the second argument is set to be equal to VBI_VB_RESUME_ALL.
* This implies to resume every core within the specified VB. Using this option
* when some of the cores within the VB are already running is not considered
* as programming error.
*
* RETURNS: OK or an error number in case of failure
*
*/

FUNC_LABEL(vbi_vb_resume)
        /*
         * r0 - virtual board id
         * r1 - virtual core (a flag or a valid vcore id)
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vbResume
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_resume)

/******************************************************************************
*
* vbiVbMgmt - virtual board management
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
* int32_t vbiVbMgmt 
*    (
*    uint32_t	cmd,	   /@ attach, detach, suspend, reset or resume @/
*    uint32_t	handle,    /@ the operation target board handle        @/
*    int32_t   *outError,  /@ where to set error : OK or error flag    @/ 
*    uint32_t	flags,	   /@ options required by the cmd executed     @/
*    void *ctl		   /@ memory / registers data		       @/ 
*    )
*
* RETURNS: OK or error in case of failure
*/
FUNC_LABEL(vbiVbMgmt)
        SAVEREGS 1
        ldr     r8, =VBI_SYS_vbMgmt
        HCALL
        RESTOREREGS 1
        mov     pc, lr
FUNC_END(vbiVbMgmt)

/******************************************************************************
*
* vbiKputs - print a string on the kernel console
*
* This system call sends the specified string to the system console.
*
*/

FUNC_LABEL(vbi_kputs)
        /*
         * r0 - char * s
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_kputs
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_kputs)

/******************************************************************************
*
* vbi_kputc - print a character on the kernel console
*
* This system call sends the specified character to the system console.
*
*/

FUNC_LABEL(vbi_kputc)
        /*
         * r0 - char * s
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_kputc
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_kputc)

/******************************************************************************
*
* vbi_panic - panic the system and halt all activity
*
* This system call causes the hypervisor to enter a panic state and display
* various pieces of information on the system console.  The hypervisor
* then enters an idle state and stops all CPU processing.
*
* Returns: does not return
*
*/

FUNC_LABEL(vbi_panic)
        /*
         * r0 - const char * msg
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_panic
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_panic)

/*******************************************************************************
*
* vbi_shell_start_debug - start the hypervisor debug shell
*
* This routine sends a message to the hypervisor debug shell manager in order to
* start the WRHV shell program. The shell program spins therefore does not share
* the processor with any other WRHV context. By default a caller of this routine
* is detached to allow the caling core to continue executing (as long as they
* are not * scheduled to run on the same processor). An optional flag
* VBI_SHELL_ATTACH can be specified to force the caller virtual board core to
* block while the shell program is running.
*
*/

FUNC_LABEL(vbi_shell_start_debug)
        /*
         * r0 - uint32_t flags
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_dbgShStart
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_shell_start_debug)

/*******************************************************************************
*
* vbi_vb_read_mem - Read a virtual board's memory
*
* This routine makes a hypercall to read a remote board's memory. The memory
* control structure contains information about the target memory to read and
* the destination buffer that hypervisor must populate with the data read.
* This routine is used to copy data from a remote VB. It is the user's
* responsibility to ensure that the memory read is accessed orthogonally.
* The sizeIn parameter specifies the number of bytes desired to be copied. 
* The sizeOut parameter indicates the number of bytes successfully copied.
* A user may set the sizeOut parameter to zero if the output size is not of
* interest otherwise to a value different than zero.
*
* RETURNS: returns OK or an error number in case of failure
*
*/
FUNC_LABEL(vbi_vb_read_mem)
        SAVEREGS
        ldr     r8, =VBI_SYS_memRead_op
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_read_mem)

/*******************************************************************************
*
* vbi_vb_write_mem - copy data to a remote board's memory
*
* This routine makes a hypercall to copy to a remote board memory. If the
* VBI_DCACHE_FLUSH is set in the control memory control structure then this
* routine flushes the data caches lines corresponding to the range of memory
* specified. If VBI_ICACHE_INV then this routine ensure that the instruction
* cache lines corresponding to the range of address is invalidated after the
* memory is copied. Invalidating the instruction is required if data containing
* is updated since the instruction cache is not aware of the content in data
* cache. Therefore flushing the data cache ensures that memory contains the
* updated data and invalidating the instruction cache ensures that the stale
* values in the instruction cache is thrown away. 
* The sizeIn parameter specifies the number of bytes desired to be copied. 
* The sizeOut parameter indicates the number of bytes successfully copied.
* A user may set the sizeOut parameter to zero if the output size is not of
* interest otherwise to a value different than zero.
* 
* RETURNS: returns OK or error number in case of failure
*
*/
FUNC_LABEL(vbi_vb_write_mem)
        SAVEREGS
        ldr     r8, =VBI_SYS_memWrite_op
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_write_mem)

/*******************************************************************************
*
* vbi_set_mem_attr - Set the attributes on entries within a WRHV pagetable
*
* This function moves some of the complexity of page table management
* out of the Guest and replaces it with Hypervisor functionality.
*
* This function provides a simplifed interface to alter attributes of
* mapped memory when the VMMU is disabled. When this call is made,
* the Hypervisor will validate the existing mapping, adjust the page
* table attributes in the page table that hardware is using for the
* Guest, and invalidate the TLB to cause the attribute change to 
* take effect immediately.
*
* Note that this function cannot alter mappings (virtual to physical) -
* for this, the VMMU must be used.
*
* The possible attributes may be a combination of the following:
* 
* VBI_MMU_PROT_USER_READ -  allow supervisor and user read
* VBI_MMU_PROT_USER_WRITE -  allow supervisor and user write
* VBI_MMU_PROT_USER_EXECUTE - allow supervisor and user execute
* VBI_MMU_PROT_SUPV_READ -  allow supervisor read
* VBI_MMU_PROT_SUPV_WRITE -  allow supervisor write
* VBI_MMU_PROT_SUPV_EXECUTE - allow supervisor execute
*
* (It is defined as not possible to set a privilege as 'user only'.)
*
*/
FUNC_LABEL(vbi_set_mem_attr)
        SAVEREGS
        ldr     r8, =VBI_SYS_mmu_attr_set
        HCALL
	INVALIDATE_CURR_ASID(r8)
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_set_mem_attr)

/*******************************************************************************
*
* vbi_get_mem_attr - Determine attributes for a page of memory
*
* This routine provides a simplifed interface to discover attributes of
* mapped memory. When this call is made, the Hypervisor will validate
* the existing mapping, returning the page table attributes in the page
* table that hardware is using for the Guest.
*
* This function moves some of the complexity of page table management
* out of the Guest and replaces it with Hypervisor functionality.
*
* The possible attributes returned may be a combination of the
* following:
* 
* VBI_MMU_PROT_USER_READ -  allow supervisor and user read
* VBI_MMU_PROT_USER_WRITE -  allow supervisor and user write
* VBI_MMU_PROT_USER_EXECUTE - allow supervisor and user execute
* VBI_MMU_PROT_SUPV_READ -  allow supervisor read
* VBI_MMU_PROT_SUPV_WRITE -  allow supervisor write
* VBI_MMU_PROT_SUPV_EXECUTE - allow supervisor execute
*
*/
FUNC_LABEL(vbi_get_mem_attr)
        SAVEREGS
        ldr     r8, =VBI_SYS_mmu_attr_get
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_get_mem_attr)

/******************************************************************************
*
* vbi_vb_remote - VB remote operations
*
* This system call interfaces to the virtual board and requests for information
* about a remote VB
*
* Returns: OK or ERROR in case of failure
*
*/

FUNC_LABEL(vbi_vb_remote)
        /*
         * r0 - uint32_t op
         * r1 - vbiVb_t boardId
         * r2 - vbiCore_t coreId
         * r3 - void * out
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vbRemote
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_remote)

/*******************************************************************************
*
* vbi_rx_op - Receive a message from another context
*
* This routine makes a hypercall and waits for a message to be received from
* another context. It blocks until a message is received.
*
* RETURNS: sender context Id or an error number in case of failure
*
*/

FUNC_LABEL(vbi_rx_op)
        /*
         * r0 - smsg pointer
         * r1 - size of smsg
         * r2 - info pointer
         * r3 - ctl pointer
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_receive
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_rx_op)

/*******************************************************************************
*
* vbi_reply - Reply to message received from another context
*
* This routine makes a hypercall in order to reply to a message received from
* another context. A message is received from remote context by calling
* vbiReceive(). The reply will unblock the recipient which may preempt
* the caller.
*
*/

FUNC_LABEL(vbi_reply)
        /*
         * r0 - vbiCtx_t id
         * r1 - void * buff
         * r2 - size_t len
         * r3 - VBI_MSG_CTL * ctl
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_reply
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_reply)

/*******************************************************************************
*
* vbi_send - Send a message to another context
*
* This routine makes a hypercall to send a message to the specified context and
* waits for a reply.  The caller will block until the sender replies to the sent
* message.
*
* RETURNS: OK or an error number in case of failure
*
*/

FUNC_LABEL(vbi_send)
        /*
         * r0 - ctx id
         * r1 - smsg pointer
         * r2 - size of smsg
         * r3 - rmsg pointer
         * r4 - rmsg length
         * r5 - info pointer
         * r6 - ctl pointer
         */
        SAVEREGS 3
        ldr     r8, =VBI_SYS_send
        HCALL
        RESTOREREGS 3
        mov     pc, lr
FUNC_END(vbi_send)

/******************************************************************************
*
* vbi_ns_op - virtual board name service call
*
* Returns: OK or ERROR in case of failure
*
*/

FUNC_LABEL(vbi_ns_op)
        /*
         * r0 - uint32_t cmd
         * r1 - char * name
         * r2 - uint32_t rev
         * r3 - VBI_HANDLE * handle
	 * stack->r4 - uint32_t timeout
	 * stack->r5 - uint32_t options
         */
        SAVEREGS 2
        ldr     r8, =VBI_SYS_ns_op
        HCALL
        RESTOREREGS 2
        mov     pc, lr
FUNC_END(vbi_ns_op)

/******************************************************************************
*
* vbi_hy_ioctl - hypervisor ioctl call
*
* This system call interfaces to the general purpose hypervisor ioctl
* function.
*
* Returns: ioctl-specific value
*
*/

FUNC_LABEL(vbi_hy_ioctl)
        /*
         * r0 - unsigned int ioctl
         * r1..r7 - args
         */
        SAVEREGS 4
        ldr     r8, =VBI_SYS_hyIoctl
        HCALL
        RESTOREREGS 4
        mov     pc, lr
FUNC_END(vbi_hy_ioctl)

/******************************************************************************
*
* vbi_io_apic_op - virtual IO APIC ioctl call
*
* This system call interfaces to the virtual IO APIC ioctl function.
*
* Returns: ioctl-specific value
*
*/

FUNC_LABEL(vbi_io_apic_op)
        /*
         * r0 - unsigned int ioctl
         * r1 - vbiIrq_t irq
         * r2 - uint32_t filter
         * r3 - vbiVb_t vbId
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vIoapicIoctl
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_io_apic_op)

/******************************************************************************
*
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
*/

FUNC_LABEL(vbi_io_apic_ioctl)
        SAVEREGS
        ldr     r8, =VBI_SYS_vIoapicIoctl
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_io_apic_ioctl)

/*******************************************************************************
*
* vbi_vcore_irq_redirect - redirect an irq to another vcore
*
* RETURNS: returns OK or error number in case of failure
*
*/

FUNC_LABEL(vbi_vcore_irq_redirect)
        /*
         * r0 - vbiIrq_t irq
         * r1 - vbiCore_t core
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_intRedirect
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vcore_irq_redirect)

/******************************************************************************
*
* vbi_config_vmmu - configure the hypervisor virtual MMU
*
* This system call configures the context's virtual MMU within the hypervisor.
*
*/

FUNC_LABEL(vbi_config_vmmu)
        /*
         * r0 - VMMU_CONFIG * config
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_config
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_config_vmmu)

/******************************************************************************
*
* vbi_enable_vmmu - enables the virtual MMU
*
* This system call enables a context's virtual MMU.
*
*/

FUNC_LABEL(vbi_enable_vmmu)
        /*
         * r0 - unsigned int vmmu_handle
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_enable
        HCALL

	/* Invalidate current ASID - it may be shared between the
	 * VMMU enabled and VMMU disabled states.
	*/

	INVALIDATE_CURR_ASID(r8)

        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_enable_vmmu)

/******************************************************************************
*
* vbi_disable_vmmu - disable the virtual MMU
*
* This system call disables a context's virtual MMU.
*
*/

FUNC_LABEL(vbi_disable_vmmu)
        /*
         * r0 - unsigned int vmmu_handle
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_disable
        HCALL

	/* Invalidate current ASID - it may be shared between the
	 * VMMU enabled and VMMU disabled states.
	*/

	INVALIDATE_CURR_ASID(r8)

        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_disable_vmmu)

/******************************************************************************
*
* vbi_flush_tlb - flush an MMU TLB entry
*
* This system call flushes the TLB associated with the specified context id
*
*/

FUNC_LABEL(vbi_flush_tlb)
        /*
         * r0 - unsigned int id
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_tlb_flush
        HCALL
        TLBIALL(r0)
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_flush_tlb)

/*******************************************************************************
*
* vbi_vcore_irq_lock - lock interrupts
*
* This routine locks interrupts for the calling core. This is a C wrapper
* function for VBI_INT_VCORE_LOCK() assembly macro. It locks interrupts returns
* and returns the previous state of interrupts.
*
*/

FUNC_LABEL(vbi_vcore_irq_lock)
#if defined(USE_TRUSTZONE) && defined(PRIVILEGED_GUEST)
	VBI_INT_VCORE_LOCK (r0, r1, r2)
#else
        /*
         * r0 - unsigned int enable (0 or 1)
         */
        SAVEREGS
        mov     r0, #0  /* disable */
        ldr     r8, = VBI_SYS_int_enable
        HCALL
        RESTOREREGS
#endif
	mov	pc, lr
FUNC_END(vbi_vcore_irq_lock)

/*******************************************************************************
*
* vbi_vcore_irq_unlock - unlock interrupts for running core
*
* This routine unlocks interrupts. This is a C wrapper function for the
* assembly macro VBI_INT_VCORE_UNLOCK().
*
*/

FUNC_LABEL(vbi_vcore_irq_unlock)
#if defined(USE_TRUSTZONE) && defined(PRIVILEGED_GUEST)
	VBI_INT_VCORE_UNLOCK (r0, r1)
#else
        /*
         * r0 - unsigned int enable (0 or 1)
         */
        SAVEREGS
        mov     r0, #1  /* enable */
        ldr     r8, = VBI_SYS_int_enable
        HCALL
        RESTOREREGS
#endif
        mov     pc, lr
FUNC_END(vbi_vcore_irq_unlock)

/*******************************************************************************
*
* vbi_vcore_irq_state - Get interrupts state for running core
*
* This routine returns the interrupt lock state for the calling core. If
* interrupts are locked it returns nonzero (true). If interrupts are not locked
* it returns zero (false).
*
*/

FUNC_LABEL(vbi_vcore_irq_state)
	VBI_INT_VCORE_STATE_GET (r0)
        mov     pc, lr
FUNC_END(vbi_vcore_irq_state)

/*******************************************************************************
*
* vbi_set_exc_base - Set the exceptions vector table base for a virtual core
*
* This routine changes the exceptions vector table base address for the active
* virtual core. By default the base address of the exception table is at address
* 0x0.
* If the guest OS programmer wishes to move this address then this function must
* be called to inform hypervisor. This function must be called with interrupts
* disabled to prevent hypervisor to deliver interrupts to an incorrect table.
*
*/

FUNC_LABEL(vbi_set_exc_base)
        /*
         * r0 - void * excBase
         */

        /*
         * alignment check -
         * return error if lower five bits are not zero
         *
         * XXX Hypervisor should enforce this
         */
        tst     r0, #0x1f
        movne   r0, #VBI_EXCBASE_SET_ERROR
        movne   pc, lr

        /* convert to hyioctl */
        mov     r1, r0
        ldr     r0, =VBI_HYIOCTL_EXCBASE

        /*
         * make the hypercall:
         * r0 - VBI_HYIOCTL_EXCBASE
         * r1 - void * excBase
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_hyIoctl
        HCALL
        RESTOREREGS

        mov     pc, lr
FUNC_END(vbi_set_exc_base)

/*******************************************************************************
*
* vbi_vb_read_reg - Read a remote core's registers
*
*/

FUNC_LABEL(vbi_vb_read_reg)
        SAVEREGS
        ldr     r8, =VBI_SYS_RegsRead_op
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_read_reg)

/*******************************************************************************
*
* vbi_vb_write_reg - write to a remote core's registers
*
*/

FUNC_LABEL(vbi_vb_write_reg)
        SAVEREGS
        ldr     r8, =VBI_SYS_RegsWrite_op
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_write_reg)

/*******************************************************************************
*
* vbi_set_exc_offset - Set the exceptions vector offsetcs for a virtual core
*
* This routine changes the exceptions vector table offset addresses for the
* active virtual core. By default they follow the standard ARM exception
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
        /*
         * r0 - VBI_EXC_OFFSETS_TABLE * excOffsetsTable
         */

        /* convert to hyioctl */
        mov     r1, r0
        ldr     r0, =VBI_HYIOCTL_EXCOFFSETS_SET

        /*
         * make the hypercall:
         * r0 - VBI_HYIOCTL_EXCOFFSETS_SET
         * r1 - VBI_EXC_OFFSETS_TABLE * excOffsetsTable
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_hyIoctl
        HCALL
        RESTOREREGS

        mov     pc, lr
FUNC_END(vbi_set_exc_offset)

/*******************************************************************************
*
* vbi_get_exc_offset - Get the exceptions vector offsetcs for a virtual core
*
* This routine gets the exceptions vector table offset addresses for the
* active virtual core. By default they follow the standard ARM exception
* offsets. vbi_set_exc_offset will enable the offsets to be changed for the
* virtual board. The offset table uses an index based on IVOR number.
*
*/

FUNC_LABEL(vbi_get_exc_offset)
        /*
         * r0 - VBI_EXC_OFFSETS_TABLE * excOffsetsTable
         */

        /* convert to hyioctl */
        mov     r1, r0
        ldr     r0, =VBI_HYIOCTL_EXCOFFSETS_GET

        /*
         * make the hypercall:
         * r0 - VBI_HYIOCTL_EXCOFFSETS_GET
         * r1 - VBI_EXC_OFFSETS_TABLE * excOffsetsTable
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_hyIoctl
        HCALL
        RESTOREREGS

        mov     pc, lr
FUNC_END(vbi_get_exc_offset)

/*******************************************************************************
*
* vbi_flush_icache - flush the instruction cache
*
* This routine makes a hypercall to flush the instruction cache of the calling
* core for the specified address range.
*
*/

FUNC_LABEL(vbi_flush_icache)
        /*
         * r0 - void * addr
         * r1 - size_t len
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_icache_flush
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_flush_icache)

/*******************************************************************************
*
* vbi_flush_dcache - flush the specified Data cache
*
* This routine makes a hypercall to flush the data cache of the calling core
* for the specified address range.
*
* RETURNS: OK or error number in case of failure
*
*/

FUNC_LABEL(vbi_flush_dcache)
        /*
         * r0 - void * addr
         * r1 - size_t len
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_dcache_flush
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_flush_dcache)

/*******************************************************************************
*
* vbi_update_text_cache - flush data cache then invalidate instruction cache
*
* This routine makes a hypercall to flush the data cache then invalidates the
* instruction cache of the calling core for the specified address range.
*
*/

FUNC_LABEL(vbi_update_text_cache)
        /*
         * r0 - void * addr
         * r1 - size_t len
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_cache_text_update
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_update_text_cache)

/******************************************************************************
*
* vbi_create_vmmu - create the virtual MMU handle
*
* This system call enables a context's virtual MMU.
*
*/

FUNC_LABEL(vbi_create_vmmu)
        /*
         * r0 - VMMU_CONFIG * cfg
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_create
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_create_vmmu)

/******************************************************************************
*
* vbi_delete_vmmu - delete the virtual MMU handle
*
* This system call deletes a context's virtual MMU.
*
*/

FUNC_LABEL(vbi_delete_vmmu)
        /*
         * r0 - VMMU_CONFIG * cfg
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_delete
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_delete_vmmu)

/******************************************************************************
*
* vbi_tlb_load_vmmu - load a TLB entry into the virtual MMU
*
* This system call loads the TLB entries for the specified address range into
* the virtual MMU.
*
*/

FUNC_LABEL(vbi_tlb_load_vmmu)
        /*
         * r0 - VMMU_CONFIG * cfg
         * r1 - void * addr
         * r2 - unsigned int len
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_tlbload
        HCALL

        INVALIDATE_CURR_ASID(r8)

        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_tlb_load_vmmu)

/******************************************************************************
*
* vbi_tlb_flush_vmmu - load a TLB entry into the virtual MMU
*
* This system call invalidates the TLB entries for the specified address range
* from the virtual MMU.
*
*/

FUNC_LABEL(vbi_tlb_flush_vmmu)
        /*
         * r0 - VMMU_CONFIG * cfg
         * r1 - void * addr
         * r2 - unsigned int len
         */
        SAVEREGS
        ldr     r8, =VBI_SYS_vmmu_tlbflush
        HCALL

        INVALIDATE_CURR_ASID(r8)

        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_tlb_flush_vmmu)

/******************************************************************************
*
* vbi_vb_create - VB create
*
* This system call interface 
*
* options:
*	VBI_CREATE_RESUME        /@ start VB after create @/
*	VBI_CREATE_HALT           /@ keep VB ahlted after creation  @/
*				
*/
FUNC_LABEL(vbi_vb_create)
        SAVEREGS
        ldr     r8, =VBI_SYS_vbCreate
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_create)

/******************************************************************************
*
* vbi_vb_delete - VB delete
*
* This system call interface 
*
*/
FUNC_LABEL(vbi_vb_delete)
        SAVEREGS
        ldr     r8, =VBI_SYS_vbDelete
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_delete)

/******************************************************************************
*
* vbi_board_simple_config_get - Get VB basic configuration information.
*
* This system call interface 
*				
*/
FUNC_LABEL(vbi_board_simple_config_get)	
        SAVEREGS
        ldr     r8, =VBI_SYS_vbBoardSimpleConfigGet
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_board_simple_config_get)

/******************************************************************************
*
* vbi_board_config_get - Get VB Configuration information and device information
*
* This system call interface 
*
*/
FUNC_LABEL(vbi_board_config_get)
        SAVEREGS
        ldr     r8, =VBI_SYS_vbBoardConfigGet
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_board_config_get)
	
/******************************************************************************
*
* vbi_vb_move - Move a VB to another core and/or priority
*
* This system call interface moves a VB to another core and/or priority
*
*/
FUNC_LABEL(vbi_vb_move)
        SAVEREGS
        ldr     r8, =VBI_SYS_vbMove
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_move)

/******************************************************************************
*
* vbi_vb_priority_set - Move a VB to another core and/or priority
*
* This system call interface moves a VB to another core and/or priority
*
*/
FUNC_LABEL(vbi_vb_priority_set)
        SAVEREGS
        ldr     r8, =VBI_SYS_vbPrioSet
        HCALL
        RESTOREREGS
        mov     pc, lr
FUNC_END(vbi_vb_priority_set)
