/**
 * Copyright (C) 2005 Brian Rogan <bcr6@cornell.edu>, IBM
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
**/

#include <linux/oprofile.h>
#include <linux/sched.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <asm/compat.h>
#include <asm/syscalls.h>

#define STACK_SP(STACK)		*(STACK)

#define STACK_LR64(STACK)	*((unsigned long *)(STACK) + 2)
#define STACK_LR32(STACK)	*((unsigned int *)(STACK) + 1)

#ifdef CONFIG_PPC64
#define STACK_LR(STACK)		STACK_LR64(STACK)
#else
#define STACK_LR(STACK)		STACK_LR32(STACK)
#endif

#if (defined(CONFIG_PPC_STD_MMU) || defined(CONFIG_PPC64))
#define HAS_FRAME_MARKER(stack_frame) \
	(stack_frame[STACK_FRAME_MARKER] == STACK_FRAME_REGS_MARKER)
#else
#define HAS_FRAME_MARKER(stack_frame) (1)
#endif

static unsigned int user_getsp32(unsigned int sp, int is_first)
{
	unsigned int stack_frame[2];
	void __user *p = compat_ptr(sp);

	if (!access_ok(VERIFY_READ, p, sizeof(stack_frame)))
		return 0;

	/*
	 * The most likely reason for this is that we returned -EFAULT,
	 * which means that we've done all that we can do from
	 * interrupt context.
	 */
	if (__copy_from_user_inatomic(stack_frame, p, sizeof(stack_frame)))
		return 0;

	if (!is_first)
		oprofile_add_trace(STACK_LR32(stack_frame));

	/*
	 * We do not enforce increasing stack addresses here because
	 * we may transition to a different stack, eg a signal handler.
	 */
	return STACK_SP(stack_frame);
}

#ifdef CONFIG_PPC64
static unsigned long user_getsp64(unsigned long sp, int is_first)
{
	unsigned long stack_frame[3];

	if (!access_ok(VERIFY_READ, (void __user *)sp, sizeof(stack_frame)))
		return 0;

	if (__copy_from_user_inatomic(stack_frame, (void __user *)sp,
					sizeof(stack_frame)))
		return 0;

	if (!is_first)
		oprofile_add_trace(STACK_LR64(stack_frame));

	return STACK_SP(stack_frame);
}
#endif

/*
 * If trace_sys is set (set through user command opcontrol
 * --enhanced-backtrace), then in addition to logging
 * the kernel trace, the function checks if the next
 * stack frame up the stack is a system call frame.  If so, then
 * return variables are set to permit tracing through user space.
 *
 * Both for the ppc32 and ppc64, when user apps make a syscall,
 * the following setup is done
 *   - current registers pt_regs struct saved at the top of
 *     process's  kernel stack space,
 *   - pt_regs trap value set to 0xc00 or 0xc01,
 *   - CONFIG_PPC64 and  CONFIG_PPC32_STD_MMU markers are set on stack
 *     distinguish exception handler gap in call trace during
 *     unwinding
 *   - sp set to (sizeof(struct pt_regs) + STACK_FRAME_OVERHEAD)
 *
 * These conditions are checked and if found, then the user stack frame
 * and the user instruction pointer are returned to the caller from
 * the pt_regs pointer, which was saved at the discovered
 * syscall stack frame.
 *
 * Finally the routine returns the next frame to the caller.
 */
static unsigned long kernel_getsp(unsigned long sp, int is_first,
	int trace_sys, unsigned long *user_frame_sp, unsigned long *user_ip)
{
	unsigned long *stack_frame = (unsigned long *)sp;
	struct pt_regs *regs;

	if (!validate_sp(sp, current, STACK_FRAME_OVERHEAD)) {
		return 0;
	}
	if (!is_first)
		oprofile_add_trace(STACK_LR(stack_frame));

	/* syscall frame */
	if (trace_sys && HAS_FRAME_MARKER(stack_frame)) {
		if (sp == ((unsigned long)task_stack_page(current) + THREAD_SIZE
			     - STACK_FRAME_OVERHEAD - sizeof(struct pt_regs))) {
			regs = (struct pt_regs *) (sp + STACK_FRAME_OVERHEAD);
			if (regs->trap == 0xc01 || regs->trap == 0xc00) {
				*user_frame_sp = regs->gpr[1];
				*user_ip = regs->nip;
				return 1;
			}
			return 0;
		}
	}
	/*
	 * We do not enforce increasing stack addresses here because
	 * we might be transitioning from an interrupt stack to a kernel
	 * stack. validate_sp() is designed to understand this, so just
	 * use it.
	 */
	return STACK_SP(stack_frame);
}

void op_powerpc_backtrace(struct pt_regs * const regs, unsigned int depth)
{
	int trace_syscall = oprofile_get_trace_thru_syscall();
	unsigned long sp = regs->gpr[1];
	int first_frame = 1;

	/* We ditch the top stackframe so need to loop through an extra time */
	depth += 1;

	if (!user_mode(regs)) {
		unsigned long parent_sp = 0;
		unsigned long user_frame_sp = 0;
		unsigned long user_ip = 0;
		while (depth--) {
			parent_sp = kernel_getsp(sp, first_frame,
				 trace_syscall, &user_frame_sp, &user_ip);
			if (!parent_sp)
				return; /* not in syscall*/
			if (parent_sp == 1) { /* in a syscall boundary*/
				if (oprofile_syscall_trace_boundary()) {
					oprofile_add_trace(user_ip);
					sp = user_frame_sp;
					break;
				}
				return;
			}
			first_frame = 0;
			sp = parent_sp;
		}
	}
#ifdef CONFIG_PPC64
	if (!test_thread_flag(TIF_32BIT)) {
		while (depth--) {
			sp = user_getsp64(sp, first_frame);
			if (!sp)
				break;
			first_frame = 0;
		}

		return;
	}
#endif

	while (depth--) {
		sp = user_getsp32(sp, first_frame);
		if (!sp)
			break;
		first_frame = 0;
	}
}
