/*
 *  Normally this file would contain the system call entry code, context
 *  switch code, and exception/interrupt return code for PowerPC.
 *  In this paravirt version of entry_32.S only a few redirects remain.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */

#include <asm/reg.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>

	.globl	paravirt_transfer_to_handler
	.weak	paravirt_transfer_to_handler
paravirt_transfer_to_handler:
	b	native_transfer_to_handler

	.globl	paravirt_ret_from_syscall
	.weak	paravirt_ret_from_syscall
paravirt_ret_from_syscall:
	b	native_ret_from_syscall

	.globl  paravirt_syscall_exit_work
	.weak  paravirt_syscall_exit_work
paravirt_syscall_exit_work:
	b	native_syscall_exit_work

	.globl	paravirt_restore
	.weak	paravirt_restore
paravirt_restore:
	b	native_restore

	.globl	paravirt_load_dbcr0
	.weak	paravirt_load_dbcr0
paravirt_load_dbcr0:
	b	native_load_dbcr0

_GLOBAL(paravirt_switch)
.weak paravirt_switch
	b	native_switch
