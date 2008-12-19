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
#include <asm/proto.h>

struct backtrace_data {
	/* user input to enable kernel-syscall-userland tracing */
	unsigned int trace_thru_syscall;

	/* user frame pointer after return from the system call
	 * frame adjusted by system_call_frame_test */
	unsigned long bp;

	/* handler to determine if unwind handler is system call frame */
	void (*syscall_frame_test)(
		void (*addr)(void),
		unsigned long sp,
		struct backtrace_data *backtrace_data);

	/* backtrace depth, 0 implies no callstack tracing */
	unsigned int depth;
};

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
	struct backtrace_data *bt_data = (struct backtrace_data *) data;
	return bt_data->depth;
}

static void backtrace_address(void *data, unsigned long addr, int reliable)
{
	struct backtrace_data *bt_data = (struct backtrace_data *) data;
	if (bt_data->depth--)
		oprofile_add_trace(addr);
}

static void backtrace_address_with_sp(void *data, unsigned long addr,
		unsigned long sp, int reliable)
{
	struct backtrace_data *bt_data = (struct backtrace_data *) data;

	if (!reliable)
		return;

	/* check if return is from a system call */
	bt_data->syscall_frame_test((void (*)(void)) addr, sp , bt_data);

	/* log the address */
	backtrace_address(data, addr, 1);
}

static struct stacktrace_ops backtrace_ops = {
	.warning	= backtrace_warning,
	.warning_symbol	= backtrace_warning_symbol,
	.stack		= backtrace_stack,
	.address	= backtrace_address,
	.walk_stack	= print_context_stack,
	.address_with_sp = backtrace_address_with_sp,
};

struct frame_head {
	struct frame_head *bp;
	unsigned long ret;
} __attribute__((packed));

/* for 32 bit addresses on a 64 bit kernel */
struct frame_head32 {
	unsigned int bp;
	unsigned int ret;
} __attribute__((packed));


#if defined(CONFIG_IA32_EMULATION)
#define SYSCALL_FRAME_TEST (test_thread_flag(TIF_32BIT) ? \
	in_ia32_syscall_frame : \
	in_x86_64_syscall_frame);
#elif defined(CONFIG_X86_64)
#define SYSCALL_FRAME_TEST in_x86_64_syscall_frame;
#else /* X86-32 only */
#define SYSCALL_FRAME_TEST in_x86_32_syscall_frame;
#endif

#if defined(CONFIG_IA32_EMULATION)
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

/*
 * The system call routines have setup a pt_regs structure
 * after the return address currently pointed to by sp.
 */
static inline void set_basepointer(unsigned long sp,
	struct backtrace_data *bt_data)
{
	struct pt_regs *regs;
	sp += sizeof(void *);
	regs = (struct pt_regs *)sp;
	bt_data->bp = frame_pointer(regs);
}

#if defined(CONFIG_OPROFILE_MODULE)
/*
 * This causes system call traversal to fail, but the only other choice
 * for oprofile as module is to export some dangerous symbols
 */
void system_call_done(void) {}
void tracesys_done(void) {}
void ia32_syscall_done(void) {}
void ia32_sysenter_done(void) {}
void ia32_cstar_done(void) {}
#endif

#if defined(CONFIG_X86_64)
static inline void in_x86_64_syscall_frame(
	void (*addr)(void), unsigned long sp,
	struct backtrace_data *bt_data)
{
	if ((addr == system_call_done) || (addr == tracesys_done))
		set_basepointer(sp, bt_data);
}
#endif

#if defined(CONFIG_IA32_EMULATION)
static inline void in_ia32_syscall_frame(
	void (*addr)(void), unsigned long sp,
	struct backtrace_data *bt_data)
{
	if ((addr == ia32_syscall_done) || (addr == ia32_cstar_done)
		|| (addr == ia32_sysenter_done))
		set_basepointer(sp, bt_data);
}
#endif

#if defined(CONFIG_X86_32)
static inline void in_x86_32_syscall_frame(
	void (*addr)(void), unsigned long sp,
	struct backtrace_data *bt_data)
{
	if ((addr == system_call_done) || (addr == ia32_sysenter_done))
		set_basepointer(sp, bt_data);
}
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
	int trace_syscall = oprofile_get_trace_thru_syscall();
	struct backtrace_data bt_data = {
		.depth = depth,
		.bp = 0,
	};

	bt_data.syscall_frame_test = SYSCALL_FRAME_TEST;

	if (!user_mode_vm(regs)) {
		unsigned long stack = kernel_stack_pointer(regs);
		unsigned long bp = frame_pointer(regs);
		/* if trace-syscall set, trace thru syscall
		 * using callbacks that include sp with pc */
		dump_trace_extended(NULL, regs, (unsigned long *)stack, bp,
			   &backtrace_ops, &bt_data, trace_syscall);
		if (bt_data.bp && oprofile_syscall_trace_boundary())
			head = (struct frame_head *)bt_data.bp;
		else
			return;
	}

	while (bt_data.depth-- && head)
		head = dump_user_backtrace(head);
}
