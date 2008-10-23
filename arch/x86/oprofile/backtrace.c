/**
 * @file backtrace.c
 *
 * @remark Copyright 2002 OProfile authors
 * @remark Read the file COPYING
 *
 * @author John Levon
 * @author David Smith
 */

#include <linux/oprofile.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>
#include <asm/stacktrace.h>

static void backtrace_warning_symbol(void *data, char *msg,
				     unsigned long symbol)
{
	/* Ignore warnings */
}

static void backtrace_warning(void *data, char *msg)
{
	/* Ignore warnings */
}

static int backtrace_stack(void *data, char *name)
{
	/* Yes, we want all stacks */
	return 0;
}

static void backtrace_address(void *data, unsigned long addr, int reliable)
{
	unsigned int *depth = data;

	if ((*depth)--)
		oprofile_add_trace(addr);
}

static struct stacktrace_ops backtrace_ops = {
	.warning	= backtrace_warning,
	.warning_symbol	= backtrace_warning_symbol,
	.stack		= backtrace_stack,
	.address	= backtrace_address,
	.walk_stack	= print_context_stack,
};

struct frame_head {
	struct frame_head *bp;
	unsigned long ret;
} __attribute__((packed));

struct frame_head32 {
	unsigned int bp;
	unsigned int ret;
} __attribute__((packed));

#if CONFIG_IA32_EMULATION
#define OP_FRAME_SIZE (test_thread_flag(TIF_32BIT) ? \
	sizeof(struct frame_head32) : \
	sizeof(struct frame_head))
#define GET_FRAME_RET(frame) (test_thread_flag(TIF_32BIT) ? \
	((struct frame_head32 *)frame)->ret : \
	((struct frame_head *)frame)->ret)
#define GET_FRAME_BP(frame) \
	(test_thread_flag(TIF_32BIT) ? \
	(struct frame_head *)((unsigned long)\
		((struct frame_head32 *)frame)->bp) : \
	((struct frame_head *)frame)->bp)
#else
#define OP_FRAME_SIZE (sizeof(struct frame_head))
#define GET_FRAME_RET(frame) (((struct frame_head *)frame)->ret)
#define GET_FRAME_BP(frame) (((struct frame_head *)frame)->bp)
#endif

static struct frame_head *dump_user_backtrace(struct frame_head *head)
{
	/* large enough for both ia32 and x86_64 frames */
	struct frame_head bufhead[2];

	/* Also check accessibility of one struct frame_head beyond */
	if (!access_ok(VERIFY_READ, head, OP_FRAME_SIZE))
		return NULL;
	if (__copy_from_user_inatomic(bufhead, head, OP_FRAME_SIZE))
		return NULL;

	oprofile_add_trace(GET_FRAME_RET(bufhead));

	/* frame pointers should strictly progress back up the stack
	 * (towards higher addresses) */
	if (head >= GET_FRAME_BP(bufhead))
		return NULL;

	return GET_FRAME_BP(bufhead);
}

void
x86_backtrace(struct pt_regs * const regs, unsigned int depth)
{
	struct frame_head *head = (struct frame_head *)frame_pointer(regs);

	if (!user_mode_vm(regs)) {
		unsigned long stack = kernel_stack_pointer(regs);
		if (depth)
			dump_trace(NULL, regs, (unsigned long *)stack, 0,
				   &backtrace_ops, &depth);
		return;
	}

	while (depth-- && head)
		head = dump_user_backtrace(head);
}
