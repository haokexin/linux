/*
 *  linux/arch/arm/kernel/ptrace.c
 *
 *  By Ross Biro 1/23/92
 * edited by Linus Torvalds
 * ARM modifications Copyright (C) 2000 Russell King
 * WindRiver (C) 2005-2007
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* uncomment below line to enable pr_debug */
/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/elf.h>
#include <linux/smp.h>
#include <linux/ptrace.h>
#include <linux/user.h>
#include <linux/security.h>
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <linux/regset.h>
#include <linux/audit.h>
#include <linux/module.h>
#include <linux/marker.h>
#include <linux/kallsyms.h>
#include <trace/syscall.h>

#include <asm/pgtable.h>
#include <asm/traps.h>
#include <asm/unistd.h>

#include "ptrace.h"

#define REG_PC	15
#define REG_PSR	16

#include <asm/cacheflush.h>

typedef unsigned int UINT32;
typedef int INT32;

#define BIT(n) ((UINT32)1U << (n))
#define BITSET(x,n) (((UINT32)(x) & (1U<<(n))) >> (n))
#define BITS(x,m,n) (((UINT32)((x) & (BIT(n) - BIT(m) + BIT(n)))) >> (m))


/*
 * ccTable is used to determine whether an instruction will be executed,
 * according to the flags in the PSR and the condition field of the
 * instruction. The table has an entry for each possible value of the
 * condition field of the instruction. Each bit indicates whether a particular
 * combination of flags will cause the instruction to be executed. Since
 * ther are four flags, this makes 16 possible TRUE/FALSE values.
 */

static UINT32 ccTable[] = {
	0xF0F0, 0x0F0F, 0xCCCC, 0x3333, 0xFF00, 0x00FF, 0xAAAA, 0x5555,
	0x0C0C, 0xF3F3, 0xAA55, 0x55AA, 0x0A05, 0xF5FA, 0xFFFF, 0x0000
};

static u32 *armGetNpc(u32 instr,	/* the current instruction */
		      struct task_struct *child	/* pointer to task registers */
    );
static u32 *thumbGetNpc(u16 instr,	/* the current instruction */
			struct task_struct *child,	/* pointer to task registers */
			int *backToArm);

/*
 * does not yet catch signals sent when the child dies.
 * in exit.c or in signal.c.
 */

#if 0
/*
 * Breakpoint SWI instruction: SWI &9F0001
 */
#define BREAKINST_ARM	0xef9f0001
#define BREAKINST_THUMB	0xdf00		/* fill this in later */
#else
/*
 * New breakpoints - use an undefined instruction.  The ARM architecture
 * reference manual guarantees that the following instruction space
 * will produce an undefined instruction exception on all CPUs:
 *
 *  ARM:   xxxx 0111 1111 xxxx xxxx xxxx 1111 xxxx
 *  Thumb: 1101 1110 xxxx xxxx
 */
#define BREAKINST_ARM	0xe7f001f0
#define BREAKINST_THUMB	0xde01
#endif

DEFINE_TRACE(syscall_entry);
DEFINE_TRACE(syscall_exit);

extern unsigned long sys_call_table[];

void ltt_dump_sys_call_table(void *call_data)
{
	int i;
	char namebuf[KSYM_NAME_LEN];

	for (i = 0; i < __NR_syscall_max + 1; i++) {
		sprint_symbol(namebuf, sys_call_table[i]);
		__trace_mark(0, syscall_state, sys_call_table, call_data,
			"id %d address %p symbol %s",
			i, (void*)sys_call_table[i], namebuf);
	}
}
EXPORT_SYMBOL_GPL(ltt_dump_sys_call_table);

void ltt_dump_idt_table(void *call_data)
{
}
EXPORT_SYMBOL_GPL(ltt_dump_idt_table);

struct pt_regs_offset {
	const char *name;
	int offset;
};

#define REG_OFFSET_NAME(r) \
	{.name = #r, .offset = offsetof(struct pt_regs, ARM_##r)}
#define REG_OFFSET_END {.name = NULL, .offset = 0}

static const struct pt_regs_offset regoffset_table[] = {
	REG_OFFSET_NAME(r0),
	REG_OFFSET_NAME(r1),
	REG_OFFSET_NAME(r2),
	REG_OFFSET_NAME(r3),
	REG_OFFSET_NAME(r4),
	REG_OFFSET_NAME(r5),
	REG_OFFSET_NAME(r6),
	REG_OFFSET_NAME(r7),
	REG_OFFSET_NAME(r8),
	REG_OFFSET_NAME(r9),
	REG_OFFSET_NAME(r10),
	REG_OFFSET_NAME(fp),
	REG_OFFSET_NAME(ip),
	REG_OFFSET_NAME(sp),
	REG_OFFSET_NAME(lr),
	REG_OFFSET_NAME(pc),
	REG_OFFSET_NAME(cpsr),
	REG_OFFSET_NAME(ORIG_r0),
	REG_OFFSET_END,
};

/**
 * regs_query_register_offset() - query register offset from its name
 * @name:	the name of a register
 *
 * regs_query_register_offset() returns the offset of a register in struct
 * pt_regs from its name. If the name is invalid, this returns -EINVAL;
 */
int regs_query_register_offset(const char *name)
{
	const struct pt_regs_offset *roff;
	for (roff = regoffset_table; roff->name != NULL; roff++)
		if (!strcmp(roff->name, name))
			return roff->offset;
	return -EINVAL;
}

/**
 * regs_query_register_name() - query register name from its offset
 * @offset:	the offset of a register in struct pt_regs.
 *
 * regs_query_register_name() returns the name of a register from its
 * offset in struct pt_regs. If the @offset is invalid, this returns NULL;
 */
const char *regs_query_register_name(unsigned int offset)
{
	const struct pt_regs_offset *roff;
	for (roff = regoffset_table; roff->name != NULL; roff++)
		if (roff->offset == offset)
			return roff->name;
	return NULL;
}

/**
 * regs_within_kernel_stack() - check the address in the stack
 * @regs:      pt_regs which contains kernel stack pointer.
 * @addr:      address which is checked.
 *
 * regs_within_kernel_stack() checks @addr is within the kernel stack page(s).
 * If @addr is within the kernel stack, it returns true. If not, returns false.
 */
bool regs_within_kernel_stack(struct pt_regs *regs, unsigned long addr)
{
	return ((addr & ~(THREAD_SIZE - 1))  ==
		(kernel_stack_pointer(regs) & ~(THREAD_SIZE - 1)));
}

/**
 * regs_get_kernel_stack_nth() - get Nth entry of the stack
 * @regs:	pt_regs which contains kernel stack pointer.
 * @n:		stack entry number.
 *
 * regs_get_kernel_stack_nth() returns @n th entry of the kernel stack which
 * is specified by @regs. If the @n th entry is NOT in the kernel stack,
 * this returns 0.
 */
unsigned long regs_get_kernel_stack_nth(struct pt_regs *regs, unsigned int n)
{
	unsigned long *addr = (unsigned long *)kernel_stack_pointer(regs);
	addr += n;
	if (regs_within_kernel_stack(regs, (unsigned long)addr))
		return *addr;
	else
		return 0;
}

/*
 * this routine will get a word off of the processes privileged stack.
 * the offset is how far from the base addr as stored in the THREAD.
 * this routine assumes that all the privileged stacks are in our
 * data space.
 */
static inline long get_user_reg(struct task_struct *task, int offset)
{
	return task_pt_regs(task)->uregs[offset];
}

/*
 * this routine will put a word on the processes privileged stack.
 * the offset is how far from the base addr as stored in the THREAD.
 * this routine assumes that all the privileged stacks are in our
 * data space.
 */
static inline int
put_user_reg(struct task_struct *task, int offset, long data)
{
	struct pt_regs newregs, *regs = task_pt_regs(task);
	int ret = -EINVAL;

	newregs = *regs;
	newregs.uregs[offset] = data;

	if (valid_user_regs(&newregs)) {
		regs->uregs[offset] = data;
		ret = 0;
	}

	return ret;
}

static inline int
read_u32(struct task_struct *task, unsigned long addr, u32 *res)
{
	int ret;

	ret = access_process_vm(task, addr, res, sizeof(*res), 0);

	return ret == sizeof(*res) ? 0 : -EIO;
}

static inline int
read_instr(struct task_struct *task, unsigned long addr, u32 *res,
	   int is_thumb)
{
	int ret;

	if ((addr & 1) || is_thumb) {
		u16 val;
		pr_debug("%s: reading a THUMB instrution\n", __FUNCTION__);
		ret = access_process_vm(task, addr & ~1, &val, sizeof(val), 0);
		ret = ret == sizeof(val) ? 0 : -EIO;
		*res = val;
	} else {
		u32 val;
		pr_debug("%s: reading an ARM instrution\n", __FUNCTION__);
		ret = access_process_vm(task, addr & ~3, &val, sizeof(val), 0);
		ret = ret == sizeof(val) ? 0 : -EIO;
		*res = val;
	}
	return ret;
}

static int
swap_insn(struct task_struct *task, unsigned long addr,
	  void *old_insn, void *new_insn, int size)
{
	int ret;

	/* new_isns is set by caller */
	pr_debug("%s new insn 0x%x for addr 0x%lx, size %d\n",
	       __FUNCTION__, *((int *)new_insn), addr, size);

	ret = access_process_vm(task, addr, old_insn, size, 0);

	if (size == 2) {
		short old, new;
		old = *((short *)old_insn);
		new = *((short *)new_insn);
		pr_debug("replace thumb 0x%x by 0x%x\n", old, new);
	}

	if (size == 4) {
		int old, new;
		old = *((int *)old_insn);
		new = *((int *)new_insn);
		pr_debug("replace arm 0x%x by 0x%x\n", old, new);
	}

	if (ret == size)
		ret = access_process_vm(task, addr, new_insn, size, 1);
	else
		pr_debug("%s ERROR\n", __FUNCTION__);
	return ret;
}

static void
add_breakpoint(struct task_struct *task,
	       struct debug_info *dbg,
	       unsigned long addr, int is_thumb /* what kind of BP to set */ )
{
	int nr = dbg->nsaved;

	if (nr < 2) {
		u32 new_insn;
		int res;

		if (is_thumb) {
			/* THIERRY */
			u16 thumb_insn = BREAKINST_THUMB;
			pr_debug("%s:Will set 0x%x at 0x%lx\n", __FUNCTION__,
			       thumb_insn, addr);
			res =
			    swap_insn(task, addr, &dbg->bp[nr].insn,
				      &thumb_insn, 2);

			if (res == 2) {
				dbg->bp[nr].address = addr;
				dbg->nsaved += 1;
				dbg->bp[nr].is_thumb = 1;
			}
		} else {
			new_insn = BREAKINST_ARM;
			res =
			    swap_insn(task, addr, &dbg->bp[nr].insn, &new_insn,
				      4);

			if (res == 4) {
				dbg->bp[nr].address = addr;
				dbg->nsaved += 1;
				dbg->bp[nr].is_thumb = 0;
			}
		}

	} else
		printk(KERN_ERR "ptrace: too many breakpoints\n");
}

/*
 * Clear one breakpoint in the user program.
 */
static void clear_breakpoint(struct task_struct *task, struct debug_entry *bp)
{
	unsigned long addr = bp->address;
	union debug_insn old_insn;
	int ret;

	pr_debug("%s at 0x%lx, %s mode\n",
		__FUNCTION__, addr, bp->is_thumb ? "thumb" : "arm");
	if (bp->is_thumb) {
		ret = swap_insn(task, addr & ~1, &old_insn.thumb,
				&bp->insn.thumb, 2);

		if (ret != 2 || old_insn.thumb != BREAKINST_THUMB)
			printk(KERN_ERR "%s:%d: corrupted Thumb breakpoint at "
				"0x%08lx (0x%04x)\n", task->comm,
				task_pid_nr(task), addr, old_insn.thumb);
	} else {
		ret = swap_insn(task, addr & ~3, &old_insn.arm,
				&bp->insn.arm, 4);

		if (ret != 4 || old_insn.arm != BREAKINST_ARM)
			printk(KERN_ERR "%s:%d: corrupted ARM breakpoint at "
				"0x%08lx (0x%08x)\n", task->comm,
				task_pid_nr(task), addr, old_insn.arm);
	}
}

void ptrace_set_bpt(struct task_struct *child)
{
	struct pt_regs *regs;
	unsigned long pc;
	u32 insn;
	int res;
	unsigned long next_pc;
	struct debug_info *dbg = &child->thread.debug;
	int is_thumb;
	int toThumb = 0;
	int backToArm = 0;
	int putAthumb;

	regs = task_pt_regs(child);
	pc = instruction_pointer(regs);
	is_thumb = thumb_mode(regs);

	res = read_instr(child, pc, &insn, is_thumb);
	if (res) {
		pr_debug("ptrace_set_bpt failed: inst=%x pc=%lx\n", insn, pc);
		return;
	}

	if (is_thumb)
		next_pc = (unsigned long)thumbGetNpc(insn, child, &backToArm);
	else
		/* Compute the next address */
		next_pc = (unsigned long)armGetNpc(insn, child);

	dbg = &child->thread.debug;
	dbg->nsaved = 0;

	pr_debug("%s (%s mode) ptrace_set_bpt inst=%x pc=%lx next_pc=%lx\n",
	       __FUNCTION__, is_thumb ? "thumb" : "arm", insn, pc, next_pc);

	if (next_pc & 0x1 || unlikely(BITS(insn,25,31) == 0x7d)) {
		toThumb = 1;
		next_pc &= (next_pc & ~0x1);
		pr_debug("mode switch ! next pc is actually 0x%lx\n", next_pc);
	}

	putAthumb = ((!is_thumb && toThumb) || (is_thumb && !backToArm));

	pr_debug("BP to set is in %s mode\n", putAthumb ? "thumb" : "arm");

	add_breakpoint(child, dbg, next_pc, putAthumb);
}

/*
 * Ensure no single-step breakpoint is pending.  Returns non-zero
 * value if child was being single-stepped.
 */
void ptrace_cancel_bpt(struct task_struct *child)
{
	int i, nsaved = child->thread.debug.nsaved;

	child->thread.debug.nsaved = 0;

	if (nsaved > 2) {
		printk("ptrace_cancel_bpt: bogus nsaved: %d!\n", nsaved);
		nsaved = 2;
	}

	for (i = 0; i < nsaved; i++)
		clear_breakpoint(child, &child->thread.debug.bp[i]);
}

void user_disable_single_step(struct task_struct *task)
{
	task->ptrace &= ~PT_SINGLESTEP;
	ptrace_cancel_bpt(task);
}

void user_enable_single_step(struct task_struct *task)
{
	task->ptrace |= PT_SINGLESTEP;
}

/*
 * Called by kernel/ptrace.c when detaching..
 */
void ptrace_disable(struct task_struct *child)
{
	user_disable_single_step(child);
}

/*
 * Handle hitting a breakpoint.
 */
void ptrace_break(struct task_struct *tsk, struct pt_regs *regs)
{
	siginfo_t info;

	pr_debug("Caught a break at %lx\n", regs->ARM_pc);
	ptrace_cancel_bpt(tsk);

	info.si_signo = SIGTRAP;
	info.si_errno = 0;
	info.si_code  = TRAP_BRKPT;
	info.si_addr  = (void __user *)instruction_pointer(regs);

	force_sig_info(SIGTRAP, &info, tsk);
}

static int break_trap(struct pt_regs *regs, unsigned int instr)
{
	ptrace_break(current, regs);
	return 0;
}

static struct undef_hook arm_break_hook = {
	.instr_mask	= 0x0fffffff,
	.instr_val	= 0x07f001f0,
	.cpsr_mask	= PSR_T_BIT,
	.cpsr_val	= 0,
	.fn		= break_trap,
};

static struct undef_hook thumb_break_hook = {
	.instr_mask	= 0xffff,
	.instr_val	= 0xde01,
	.cpsr_mask	= PSR_T_BIT,
	.cpsr_val	= PSR_T_BIT,
	.fn		= break_trap,
};

static struct undef_hook thumb2_break_hook = {
	.instr_mask	= 0xffffffff,
	.instr_val	= 0xf7f0a000,
	.cpsr_mask	= PSR_T_BIT,
	.cpsr_val	= PSR_T_BIT,
	.fn		= break_trap,
};

static int __init ptrace_break_init(void)
{
	register_undef_hook(&arm_break_hook);
	register_undef_hook(&thumb_break_hook);
	register_undef_hook(&thumb2_break_hook);
	return 0;
}

core_initcall(ptrace_break_init);

/*
 * Read the word at offset "off" into the "struct user".  We
 * actually access the pt_regs stored on the kernel stack.
 */
static int ptrace_read_user(struct task_struct *tsk, unsigned long off,
			    unsigned long __user *ret)
{
	unsigned long tmp;

	if (off & 3)
		return -EIO;

	tmp = 0;
	if (off == PT_TEXT_ADDR)
		tmp = tsk->mm->start_code;
	else if (off == PT_DATA_ADDR)
		tmp = tsk->mm->start_data;
	else if (off == PT_TEXT_END_ADDR)
		tmp = tsk->mm->end_code;
	else if (off < sizeof(struct pt_regs))
		tmp = get_user_reg(tsk, off >> 2);
	else if (off >= sizeof(struct user))
		return -EIO;

	return put_user(tmp, ret);
}

/*
 * Write the word at offset "off" into "struct user".  We
 * actually access the pt_regs stored on the kernel stack.
 */
static int ptrace_write_user(struct task_struct *tsk, unsigned long off,
			     unsigned long val)
{
	if (off & 3 || off >= sizeof(struct user))
		return -EIO;

	if (off >= sizeof(struct pt_regs))
		return 0;

	return put_user_reg(tsk, off >> 2, val);
}

#ifdef CONFIG_IWMMXT

/*
 * Get the child iWMMXt state.
 */
static int ptrace_getwmmxregs(struct task_struct *tsk, void __user *ufp)
{
	struct thread_info *thread = task_thread_info(tsk);

	if (!test_ti_thread_flag(thread, TIF_USING_IWMMXT))
		return -ENODATA;
	iwmmxt_task_disable(thread);  /* force it to ram */
	return copy_to_user(ufp, &thread->fpstate.iwmmxt, IWMMXT_SIZE)
		? -EFAULT : 0;
}

/*
 * Set the child iWMMXt state.
 */
static int ptrace_setwmmxregs(struct task_struct *tsk, void __user *ufp)
{
	struct thread_info *thread = task_thread_info(tsk);

	if (!test_ti_thread_flag(thread, TIF_USING_IWMMXT))
		return -EACCES;
	iwmmxt_task_release(thread);  /* force a reload */
	return copy_from_user(&thread->fpstate.iwmmxt, ufp, IWMMXT_SIZE)
		? -EFAULT : 0;
}

#endif

#ifdef CONFIG_CRUNCH
/*
 * Get the child Crunch state.
 */
static int ptrace_getcrunchregs(struct task_struct *tsk, void __user *ufp)
{
	struct thread_info *thread = task_thread_info(tsk);

	crunch_task_disable(thread);  /* force it to ram */
	return copy_to_user(ufp, &thread->crunchstate, CRUNCH_SIZE)
		? -EFAULT : 0;
}

/*
 * Set the child Crunch state.
 */
static int ptrace_setcrunchregs(struct task_struct *tsk, void __user *ufp)
{
	struct thread_info *thread = task_thread_info(tsk);

	crunch_task_release(thread);  /* force a reload */
	return copy_from_user(&thread->crunchstate, ufp, CRUNCH_SIZE)
		? -EFAULT : 0;
}
#endif

#ifdef CONFIG_HAVE_HW_BREAKPOINT
/*
 * Convert a virtual register number into an index for a thread_info
 * breakpoint array. Breakpoints are identified using positive numbers
 * whilst watchpoints are negative. The registers are laid out as pairs
 * of (address, control), each pair mapping to a unique hw_breakpoint struct.
 * Register 0 is reserved for describing resource information.
 */
static int ptrace_hbp_num_to_idx(long num)
{
	if (num < 0)
		num = (ARM_MAX_BRP << 1) - num;
	return (num - 1) >> 1;
}

/*
 * Returns the virtual register number for the address of the
 * breakpoint at index idx.
 */
static long ptrace_hbp_idx_to_num(int idx)
{
	long mid = ARM_MAX_BRP << 1;
	long num = (idx << 1) + 1;
	return num > mid ? mid - num : num;
}

/*
 * Handle hitting a HW-breakpoint.
 */
static void ptrace_hbptriggered(struct perf_event *bp,
				     struct perf_sample_data *data,
				     struct pt_regs *regs)
{
	struct arch_hw_breakpoint *bkpt = counter_arch_bp(bp);
	long num;
	int i;
	siginfo_t info;

	for (i = 0; i < ARM_MAX_HBP_SLOTS; ++i)
		if (current->thread.debug.hbp[i] == bp)
			break;

	num = (i == ARM_MAX_HBP_SLOTS) ? 0 : ptrace_hbp_idx_to_num(i);

	info.si_signo	= SIGTRAP;
	info.si_errno	= (int)num;
	info.si_code	= TRAP_HWBKPT;
	info.si_addr	= (void __user *)(bkpt->trigger);

	force_sig_info(SIGTRAP, &info, current);
}

/*
 * Set ptrace breakpoint pointers to zero for this task.
 * This is required in order to prevent child processes from unregistering
 * breakpoints held by their parent.
 */
void clear_ptrace_hw_breakpoint(struct task_struct *tsk)
{
	memset(tsk->thread.debug.hbp, 0, sizeof(tsk->thread.debug.hbp));
}

/*
 * Unregister breakpoints from this task and reset the pointers in
 * the thread_struct.
 */
void flush_ptrace_hw_breakpoint(struct task_struct *tsk)
{
	int i;
	struct thread_struct *t = &tsk->thread;

	for (i = 0; i < ARM_MAX_HBP_SLOTS; i++) {
		if (t->debug.hbp[i]) {
			unregister_hw_breakpoint(t->debug.hbp[i]);
			t->debug.hbp[i] = NULL;
		}
	}
}

static u32 ptrace_get_hbp_resource_info(void)
{
	u8 num_brps, num_wrps, debug_arch, wp_len;
	u32 reg = 0;

	num_brps	= hw_breakpoint_slots(TYPE_INST);
	num_wrps	= hw_breakpoint_slots(TYPE_DATA);
	debug_arch	= arch_get_debug_arch();
	wp_len		= arch_get_max_wp_len();

	reg		|= debug_arch;
	reg		<<= 8;
	reg		|= wp_len;
	reg		<<= 8;
	reg		|= num_wrps;
	reg		<<= 8;
	reg		|= num_brps;

	return reg;
}

static struct perf_event *ptrace_hbp_create(struct task_struct *tsk, int type)
{
	struct perf_event_attr attr;

	ptrace_breakpoint_init(&attr);

	/* Initialise fields to sane defaults. */
	attr.bp_addr	= 0;
	attr.bp_len	= HW_BREAKPOINT_LEN_4;
	attr.bp_type	= type;
	attr.disabled	= 1;

	return register_user_hw_breakpoint(&attr, ptrace_hbptriggered, NULL,
					   tsk);
}

static int ptrace_gethbpregs(struct task_struct *tsk, long num,
			     unsigned long  __user *data)
{
	u32 reg;
	int idx, ret = 0;
	struct perf_event *bp;
	struct arch_hw_breakpoint_ctrl arch_ctrl;

	if (num == 0) {
		reg = ptrace_get_hbp_resource_info();
	} else {
		idx = ptrace_hbp_num_to_idx(num);
		if (idx < 0 || idx >= ARM_MAX_HBP_SLOTS) {
			ret = -EINVAL;
			goto out;
		}

		bp = tsk->thread.debug.hbp[idx];
		if (!bp) {
			reg = 0;
			goto put;
		}

		arch_ctrl = counter_arch_bp(bp)->ctrl;

		/*
		 * Fix up the len because we may have adjusted it
		 * to compensate for an unaligned address.
		 */
		while (!(arch_ctrl.len & 0x1))
			arch_ctrl.len >>= 1;

		if (num & 0x1)
			reg = bp->attr.bp_addr;
		else
			reg = encode_ctrl_reg(arch_ctrl);
	}

put:
	if (put_user(reg, data))
		ret = -EFAULT;

out:
	return ret;
}

static int ptrace_sethbpregs(struct task_struct *tsk, long num,
			     unsigned long __user *data)
{
	int idx, gen_len, gen_type, implied_type, ret = 0;
	u32 user_val;
	struct perf_event *bp;
	struct arch_hw_breakpoint_ctrl ctrl;
	struct perf_event_attr attr;

	if (num == 0)
		goto out;
	else if (num < 0)
		implied_type = HW_BREAKPOINT_RW;
	else
		implied_type = HW_BREAKPOINT_X;

	idx = ptrace_hbp_num_to_idx(num);
	if (idx < 0 || idx >= ARM_MAX_HBP_SLOTS) {
		ret = -EINVAL;
		goto out;
	}

	if (get_user(user_val, data)) {
		ret = -EFAULT;
		goto out;
	}

	bp = tsk->thread.debug.hbp[idx];
	if (!bp) {
		bp = ptrace_hbp_create(tsk, implied_type);
		if (IS_ERR(bp)) {
			ret = PTR_ERR(bp);
			goto out;
		}
		tsk->thread.debug.hbp[idx] = bp;
	}

	attr = bp->attr;

	if (num & 0x1) {
		/* Address */
		attr.bp_addr	= user_val;
	} else {
		/* Control */
		decode_ctrl_reg(user_val, &ctrl);
		ret = arch_bp_generic_fields(ctrl, &gen_len, &gen_type);
		if (ret)
			goto out;

		if ((gen_type & implied_type) != gen_type) {
			ret = -EINVAL;
			goto out;
		}

		attr.bp_len	= gen_len;
		attr.bp_type	= gen_type;
		attr.disabled	= !ctrl.enabled;
	}

	ret = modify_user_hw_breakpoint(bp, &attr);
out:
	return ret;
}
#endif

/* regset get/set implementations */

static int gpr_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	struct pt_regs *regs = task_pt_regs(target);

	return user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   regs,
				   0, sizeof(*regs));
}

static int gpr_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct pt_regs newregs;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				 &newregs,
				 0, sizeof(newregs));
	if (ret)
		return ret;

	if (!valid_user_regs(&newregs))
		return -EINVAL;

	*task_pt_regs(target) = newregs;
	return 0;
}

static int fpa_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	return user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &task_thread_info(target)->fpstate,
				   0, sizeof(struct user_fp));
}

static int fpa_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	struct thread_info *thread = task_thread_info(target);

	thread->used_cp[1] = thread->used_cp[2] = 1;

	return user_regset_copyin(&pos, &count, &kbuf, &ubuf,
		&thread->fpstate,
		0, sizeof(struct user_fp));
}

#ifdef CONFIG_VFP
/*
 * VFP register get/set implementations.
 *
 * With respect to the kernel, struct user_fp is divided into three chunks:
 * 16 or 32 real VFP registers (d0-d15 or d0-31)
 *	These are transferred to/from the real registers in the task's
 *	vfp_hard_struct.  The number of registers depends on the kernel
 *	configuration.
 *
 * 16 or 0 fake VFP registers (d16-d31 or empty)
 *	i.e., the user_vfp structure has space for 32 registers even if
 *	the kernel doesn't have them all.
 *
 *	vfp_get() reads this chunk as zero where applicable
 *	vfp_set() ignores this chunk
 *
 * 1 word for the FPSCR
 *
 * The bounds-checking logic built into user_regset_copyout and friends
 * means that we can make a simple sequence of calls to map the relevant data
 * to/from the specified slice of the user regset structure.
 */
static int vfp_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	int ret;
	struct thread_info *thread = task_thread_info(target);
	struct vfp_hard_struct const *vfp = &thread->vfpstate.hard;
	const size_t user_fpregs_offset = offsetof(struct user_vfp, fpregs);
	const size_t user_fpscr_offset = offsetof(struct user_vfp, fpscr);

	vfp_sync_hwstate(thread);

	ret = user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				  &vfp->fpregs,
				  user_fpregs_offset,
				  user_fpregs_offset + sizeof(vfp->fpregs));
	if (ret)
		return ret;

	ret = user_regset_copyout_zero(&pos, &count, &kbuf, &ubuf,
				       user_fpregs_offset + sizeof(vfp->fpregs),
				       user_fpscr_offset);
	if (ret)
		return ret;

	return user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &vfp->fpscr,
				   user_fpscr_offset,
				   user_fpscr_offset + sizeof(vfp->fpscr));
}

/*
 * For vfp_set() a read-modify-write is done on the VFP registers,
 * in order to avoid writing back a half-modified set of registers on
 * failure.
 */
static int vfp_set(struct task_struct *target,
			  const struct user_regset *regset,
			  unsigned int pos, unsigned int count,
			  const void *kbuf, const void __user *ubuf)
{
	int ret;
	struct thread_info *thread = task_thread_info(target);
	struct vfp_hard_struct new_vfp;
	const size_t user_fpregs_offset = offsetof(struct user_vfp, fpregs);
	const size_t user_fpscr_offset = offsetof(struct user_vfp, fpscr);

	vfp_sync_hwstate(thread);
	new_vfp = thread->vfpstate.hard;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				  &new_vfp.fpregs,
				  user_fpregs_offset,
				  user_fpregs_offset + sizeof(new_vfp.fpregs));
	if (ret)
		return ret;

	ret = user_regset_copyin_ignore(&pos, &count, &kbuf, &ubuf,
				user_fpregs_offset + sizeof(new_vfp.fpregs),
				user_fpscr_offset);
	if (ret)
		return ret;

	ret = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				 &new_vfp.fpscr,
				 user_fpscr_offset,
				 user_fpscr_offset + sizeof(new_vfp.fpscr));
	if (ret)
		return ret;

	vfp_flush_hwstate(thread);
	thread->vfpstate.hard = new_vfp;

	return 0;
}
#endif /* CONFIG_VFP */

enum arm_regset {
	REGSET_GPR,
	REGSET_FPR,
#ifdef CONFIG_VFP
	REGSET_VFP,
#endif
};

static const struct user_regset arm_regsets[] = {
	[REGSET_GPR] = {
		.core_note_type = NT_PRSTATUS,
		.n = ELF_NGREG,
		.size = sizeof(u32),
		.align = sizeof(u32),
		.get = gpr_get,
		.set = gpr_set
	},
	[REGSET_FPR] = {
		/*
		 * For the FPA regs in fpstate, the real fields are a mixture
		 * of sizes, so pretend that the registers are word-sized:
		 */
		.core_note_type = NT_PRFPREG,
		.n = sizeof(struct user_fp) / sizeof(u32),
		.size = sizeof(u32),
		.align = sizeof(u32),
		.get = fpa_get,
		.set = fpa_set
	},
#ifdef CONFIG_VFP
	[REGSET_VFP] = {
		/*
		 * Pretend that the VFP regs are word-sized, since the FPSCR is
		 * a single word dangling at the end of struct user_vfp:
		 */
		.core_note_type = NT_ARM_VFP,
		.n = ARM_VFPREGS_SIZE / sizeof(u32),
		.size = sizeof(u32),
		.align = sizeof(u32),
		.get = vfp_get,
		.set = vfp_set
	},
#endif /* CONFIG_VFP */
};

static const struct user_regset_view user_arm_view = {
	.name = "arm", .e_machine = ELF_ARCH, .ei_osabi = ELF_OSABI,
	.regsets = arm_regsets, .n = ARRAY_SIZE(arm_regsets)
};

const struct user_regset_view *task_user_regset_view(struct task_struct *task)
{
	return &user_arm_view;
}

long arch_ptrace(struct task_struct *child, long request,
		 unsigned long addr, unsigned long data)
{
	int ret;
	unsigned long __user *datap = (unsigned long __user *) data;

	switch (request) {
		case PTRACE_PEEKUSR:
			ret = ptrace_read_user(child, addr, datap);
			break;

		case PTRACE_POKEUSR:
			ret = ptrace_write_user(child, addr, data);
			break;

		case PTRACE_GETREGS:
			ret = copy_regset_to_user(child,
						  &user_arm_view, REGSET_GPR,
						  0, sizeof(struct pt_regs),
						  datap);
			break;

		case PTRACE_SETREGS:
			ret = copy_regset_from_user(child,
						    &user_arm_view, REGSET_GPR,
						    0, sizeof(struct pt_regs),
						    datap);
			break;

		case PTRACE_GETFPREGS:
			ret = copy_regset_to_user(child,
						  &user_arm_view, REGSET_FPR,
						  0, sizeof(union fp_state),
						  datap);
			break;

		case PTRACE_SETFPREGS:
			ret = copy_regset_from_user(child,
						    &user_arm_view, REGSET_FPR,
						    0, sizeof(union fp_state),
						    datap);
			break;

#ifdef CONFIG_IWMMXT
		case PTRACE_GETWMMXREGS:
			ret = ptrace_getwmmxregs(child, datap);
			break;

		case PTRACE_SETWMMXREGS:
			ret = ptrace_setwmmxregs(child, datap);
			break;
#endif

		case PTRACE_GET_THREAD_AREA:
			ret = put_user(task_thread_info(child)->tp_value,
				       datap);
			break;

		case PTRACE_SET_SYSCALL:
			task_thread_info(child)->syscall = data;
			ret = 0;
			break;

#ifdef CONFIG_CRUNCH
		case PTRACE_GETCRUNCHREGS:
			ret = ptrace_getcrunchregs(child, datap);
			break;

		case PTRACE_SETCRUNCHREGS:
			ret = ptrace_setcrunchregs(child, datap);
			break;
#endif

#ifdef CONFIG_VFP
		case PTRACE_GETVFPREGS:
			ret = copy_regset_to_user(child,
						  &user_arm_view, REGSET_VFP,
						  0, ARM_VFPREGS_SIZE,
						  datap);
			break;

		case PTRACE_SETVFPREGS:
			ret = copy_regset_from_user(child,
						    &user_arm_view, REGSET_VFP,
						    0, ARM_VFPREGS_SIZE,
						    datap);
			break;
#endif

#ifdef CONFIG_HAVE_HW_BREAKPOINT
		case PTRACE_GETHBPREGS:
			if (ptrace_get_breakpoints(child) < 0)
				return -ESRCH;

			ret = ptrace_gethbpregs(child, addr,
						(unsigned long __user *)data);
			ptrace_put_breakpoints(child);
			break;
		case PTRACE_SETHBPREGS:
			if (ptrace_get_breakpoints(child) < 0)
				return -ESRCH;

			ret = ptrace_sethbpregs(child, addr,
						(unsigned long __user *)data);
			ptrace_put_breakpoints(child);
			break;
#endif

		default:
			ret = ptrace_request(child, request, addr, data);
			break;
	}

	return ret;
}

asmlinkage int syscall_trace(int why, struct pt_regs *regs, int scno)
{
	unsigned long ip;

	if (why)
		audit_syscall_exit(regs);
	else
		audit_syscall_entry(AUDIT_ARCH_ARM, scno, regs->ARM_r0,
				    regs->ARM_r1, regs->ARM_r2, regs->ARM_r3);

	if (!why)
		trace_syscall_entry(regs, scno);
	else
		trace_syscall_exit(regs->ARM_r0);

	if (!test_thread_flag(TIF_SYSCALL_TRACE))
		return scno;
	if (!(current->ptrace & PT_PTRACED))
		return scno;

	current_thread_info()->syscall = scno;

	/*
	 * IP is used to denote syscall entry/exit:
	 * IP = 0 -> entry, =1 -> exit
	 */
	ip = regs->ARM_ip;
	regs->ARM_ip = why;

	/* the 0x80 provides a way for the tracing parent to distinguish
	   between a syscall stop and SIGTRAP delivery */
	ptrace_notify(SIGTRAP | ((current->ptrace & PT_TRACESYSGOOD)
				 ? 0x80 : 0));
	/*
	 * this isn't the same as continuing with a signal, but it will do
	 * for normal use.  strace only continues with a signal if the
	 * stopping signal is not SIGTRAP.  -brl
	 */
	if (current->exit_code) {
		send_sig(current->exit_code, current, 1);
		current->exit_code = 0;
	}
	regs->ARM_ip = ip;

	return current_thread_info()->syscall;
}

/*
 * The following is borrowed from vxWorks
 */

static UINT32 armShiftedRegVal(struct pt_regs *pRegs,	/* pointer to task registers */
			       UINT32 instr,	/* machine instruction */
			       int cFlag	/* value of carry flag */
    )
{
	UINT32 res, shift, rm, rs, shiftType;

	rm = BITS(instr, 0, 3);
	shiftType = BITS(instr, 5, 6);

	if (BITSET(instr, 4)) {
		rs = BITS(instr, 8, 11);
		shift =
		    (rs ==
		     15 ? (UINT32) pRegs->ARM_pc + 8 : pRegs->uregs[rs]) & 0xFF;
	} else
		shift = BITS(instr, 7, 11);

	res = rm == 15 ? (UINT32) pRegs->ARM_pc + (BITSET(instr, 4) ? 12 : 8)
	    : pRegs->uregs[rm];

	switch (shiftType) {
	case 0:		/* LSL */
		res = shift >= 32 ? 0 : res << shift;
		break;

	case 1:		/* LSR */
		res = shift >= 32 ? 0 : res >> shift;
		break;

	case 2:		/* ASR */
		if (shift >= 32)
			shift = 31;
		res = (res & 0x80000000L) ? ~((~res) >> shift) : res >> shift;
		break;

	case 3:		/* ROR */
		shift &= 31;
		if (shift == 0)
			res = (res >> 1) | (cFlag ? 0x80000000L : 0);
		else
			res = (res >> shift) | (res << (32 - shift));
		break;
	}
	return res;

}				/* armShiftedRegVal() */

/* When in thumb mode */

static u32 *thumbGetNpc(u16 instr,	/* the current instruction */
			struct task_struct *child, int *backToArm)
{
	UINT32 pc;		/* current program counter */
	UINT32 nPc;		/* next program counter */
	struct pt_regs *pRegs;	/* registers            */

	u16 instr16 = instr;

	pr_debug("%s:instruction 0x%x\n", __FUNCTION__, instr16);

	/*
	 * Get user registers
	 */

	pRegs = task_pt_regs(child);

	pc = (UINT32) pRegs->ARM_pc & ~1;	/* current PC as a UINT32 */
	nPc = pc + 2;		/* Thumb default */

	/*
	 * Now examine the instruction
	 * Following code is derived from the ARM symbolic debugger.
	 */

	switch (BITS(instr, 12, 15)) {
	case 0x0:
	case 0x1:
	case 0x2:
	case 0x3:
	case 0x5:
	case 0x6:
	case 0x7:
	case 0x8:
	case 0x9:
	case 0xA:
	case 0xC:
		/* no effect on PC - next instruction executes */
		pr_debug("no effect on PC\n");
		break;

	case 4:
		/* (bits 7-11 == 0x0E) == BX
		 * (bits 7-11 == 0x0F) == BLX
		 */
		if (BITS(instr, 8, 11) == 0x7) {
			/* BX || BLX */
			int rn;

			rn = BITS(instr, 3, 6);
			nPc = rn == 15 ? pc + 4 : pRegs->uregs[rn];
			pr_debug("BX, at 0x%x\n", nPc);
			if (!(nPc & 0x1))
				*backToArm = 1;
			break;
		}

		if (BITSET(instr, 7) && (BITS(instr, 0, 11) & 0xC07) == 0x407) {
			/* do something to pc */
			int rn;
			UINT32 operand;

			rn = BITS(instr, 3, 6);
			operand = rn == 15 ? pc + 4 : pRegs->uregs[rn];
			switch (BITS(instr, 8, 9)) {
			case 0:	/* ADD */
				nPc = pc + 4 + operand;
				break;
			case 1:	/* CMP */
				break;
			case 2:	/* MOV */
				nPc = operand;
				break;
			case 3:	/* BX - already handled */
				pr_debug("BX again ???\n");
				break;
			}
		}
		break;

	case 0xB:
		if (BITS(instr, 8, 11) == 0xD) {
			/* POP {rlist, pc} */
			INT32 offset = 0;
			UINT32 regList, regBit;

			for (regList = BITS(instr, 0, 7); regList != 0;
			     regList &= ~regBit) {
				regBit = regList & (-regList);
				offset += 4;
			}
			nPc = *(UINT32 *) (pRegs->uregs[13] + offset);
			if (!(nPc & 0x1))
				*backToArm = 1;
			/* don't check for new pc == pc like ARM debugger does */
		}
		break;

	case 0xD:
		{
			/* SWI or conditional branch */
			UINT32 cond;

			cond = (instr >> 8) & 0xF;
			if (cond == 0xF)
				break;	/* SWI */

			/* Conditional branch
			 * Use the same mechanism as armGetNpc() to determine whether
			 * the branch will be taken
			 */
			if (((ccTable[cond] >> (BITS(pRegs->ARM_cpsr,28,31))) & 1) ==
			    0)
				break;	/* instruction will not be executed */

			/* branch will be taken */
			nPc = pc + 4 + (((instr & 0x00FF) << 1) |
					(BITSET(instr, 7) ? 0xFFFFFE00 : 0));
		}
		break;

	case 0xE:
		if (BITSET(instr, 11) == 0)
			/* Unconditional branch */
			nPc = pc + 4 + (((instr & 0x07FF) << 1) |
					(BITSET(instr, 10) ? 0xFFFFF000 : 0));
		break;

	case 0xF:
		/* BL */
		pr_debug("BL\n");
		if (BITSET(instr, 11)) {
			/* second half of BL - PC should never be here */

			nPc = pRegs->uregs[14] + ((instr & 0x07FF) << 1);
		} else {
			/* first half of BL */

			UINT32 nextBit;

			if (read_instr(child, pc + 2, &nextBit, 1) != 0) {
				pr_debug("Unable to read second half of BL\n");
				break;
			}

			if ((nextBit & 0xE800) != 0xE800) {
				pr_debug("Strange BL\n");
				/* Something strange going on */
				break;
			}

			nPc = pc + 4 + (INT32) ((((instr & 0x7FF) << 11) |
						 (nextBit & 0x7FF)) << 10) /
			    (1 << 9);

			if ((nextBit & 0xF800) == 0xE800) {
				pr_debug("  was BLX to ARM\n");
				nPc &= 0xFFFFFFFC;
				*backToArm = 1;
			}
		}
		break;

	}			/* switch */

	return (u32 *)nPc;

}				/* thumbGetNpc() */

/* When in ARM mode. the returned next address can be either
 * an ARM or a THUMB zone */

static u32 *armGetNpc(u32 instr,	/* the current instruction */
		      struct task_struct *child)
{
	UINT32 pc;		/* current program counter */
	UINT32 nPc;		/* next program counter */
	struct pt_regs *pRegs;	/* registers            */

	/*
	 * Get user registers
	 */

	pRegs = task_pt_regs(child);

	/*
	 * Early versions of this file looked at the PSR to determine whether the
	 * CPU was in ARM state or Thumb state and decode the next instruction
	 * accordingly. This has been removed since there is to be no support for
	 * ARM/Thumb interworking.
	 */

	pc = (UINT32) pRegs->ARM_pc;	/* current PC as a UINT32 */
	nPc = pc + 4;		/* default */

	pr_debug("nPc %x CPSR %lx\n", nPc, pRegs->ARM_cpsr);

	/*
	 * Now examine the instruction
	 * First, check the current condition codes against the condition
	 * field of the instruction since, if this instruction is not going
	 * to be executed, we can return immediately
	 *
	 * The following code is a translation of the code supplied by ARM
	 * for instruction decoding (EAN-26). Note that this version, unlike
	 * the original assembly language version cannot generate unaligned
	 * accesses which might be faulted by some systems.
	 *
	 * Briefly, there are 16 entries in ccTable, one for each possible
	 * value of the condition part of an instruction. Each entry has one
	 * bit for each possible value of the flags in the PSR. The table
	 * entry is extracted using the condition part of the instruction and
	 * the bits are indexed using the value obtained by extracting the
	 * flags from the PSR. If the bit so obtained is 1, the instruction
	 * will be executed.
	 */

	pr_debug("Index %x\n", ((instr >> 28) & 0xF));
	pr_debug("Value %x\n", (ccTable[(instr >> 28) & 0xF]));
	pr_debug("CPSRd %lx\n", (pRegs->ARM_cpsr >> 28) & 0xF);
	pr_debug("Res %x\n",
	       ((ccTable[(instr >> 28) & 0xF] >>
		 ((pRegs->ARM_cpsr >> 28) & 0xF))));

	/*
	 * A BLX is always unconditional in ARM v5 and up and it can
	 * branch to an even address.  To account for this case it must be
	 * checked for explicitly here and when deciding the type of
	 * breakpoint.
	 */

	if (((ccTable[(instr >> 28) & 0xF] >> ((pRegs->ARM_cpsr >> 28) & 0xF)) &
	     1) == 0 && !(BITS(instr,25,31) == 0x7d))
		return (u32 *) nPc;	/* instruction will not be executed */

	/*
	 * This instruction WILL be executed so look at its type
	 * We're looking for anything that affects the PC e.g.
	 *    B
	 *    BL
	 *    any data processing op where PC is the destination
	 *    any LDR with the PC as the destination
	 *    any LDM with the PC in the list of registers to be loaded
	 *
	 * Following code is derived from the ARM symbolic debugger.
	 */

	switch (BITS(instr, 24, 27)) {
	case 1:		/* check for halfword or signed byte load to PC */
		if (BITSET(instr, 4) && BITSET(instr, 7) && BITSET(instr, 20) &&
		    BITS(instr, 5, 6) != 0 && BITS(instr, 12, 15) == 15)
			break;	/* bad instruction */

		/* FALL THROUGH */

	case 0:		/* data processing */
	case 2:
	case 3:
		{
			UINT32 rn, op1, op2, cFlag;

			if (BITS(instr, 12, 15) != 15)
				/* Rd */
				/* operation does not affect PC */
				break;

			if (BITS(instr, 22, 25) == 0 && BITS(instr, 4, 7) == 9)
				/* multiply with PC as destination not allowed */
				break;

			if (BITS(instr, 4, 23) == 0x2FFF1) {
				/* BX */
				rn = BITS(instr, 0, 3);
				nPc = (rn == 15 ? pc + 8 : pRegs->uregs[rn]);
				break;
			}

			if (BITS(instr, 4, 23) == 0x2FFF3) {
				/* BLX */
				rn = BITS(instr, 0, 3);
				nPc = (rn == 15 ? pc + 8 : pRegs->uregs[rn]);
				break;
			}

			cFlag = BITSET(pRegs->ARM_cpsr, 29);
			rn = BITS(instr, 16, 19);
			op1 = rn == 15 ? pc + 8 : pRegs->uregs[rn];

			if (BITSET(instr, 25)) {
				UINT32 immVal, rotate;

				immVal = BITS(instr, 0, 7);
				rotate = 2 * BITS(instr, 8, 11);
				op2 =
				    (immVal >> rotate) | (immVal <<
							  (32 - rotate));
			} else
				op2 = armShiftedRegVal(pRegs, instr, cFlag);

			switch (BITS(instr, 21, 24)) {
			case 0x0:	/* AND */
				nPc = op1 & op2;
				break;
			case 0x1:	/* EOR */
				nPc = op1 ^ op2;
				break;
			case 0x2:	/* SUB */
				nPc = op1 - op2;
				break;
			case 0x3:	/* RSB */
				nPc = op2 - op1;
				break;
			case 0x4:	/* ADD */
				nPc = op1 + op2;
				break;
			case 0x5:	/* ADC */
				nPc = op1 + op2 + cFlag;
				break;
			case 0x6:	/* SBC */
				nPc = op1 - op2 + cFlag;
				break;
			case 0x7:	/* RSC */
				nPc = op2 - op1 + cFlag;
				break;
			case 0x8:	/* TST */
			case 0x9:	/* TEQ */
			case 0xa:	/* CMP */
			case 0xb:	/* CMN */
				break;
			case 0xc:	/* ORR */
				nPc = op1 | op2;
				break;
			case 0xd:	/* MOV */
				nPc = op2;
				break;
			case 0xe:	/* BIC */
				nPc = op1 & ~op2;
				break;
			case 0xf:	/* MVN */
				nPc = ~op2;
				break;
			}
		}
		break;

	case 4:		/* data transfer */
	case 5:
	case 6:
	case 7:
		if (BITSET(instr, 20) && BITS(instr, 12, 15) == 15 &&
		    !BITSET(instr, 22))
			/* load, PC and not a byte load */
		{
			UINT32 rn, cFlag, base;
			INT32 offset;

			rn = BITS(instr, 16, 19);
			base = rn == 15 ? pc + 8 : pRegs->uregs[rn];
			cFlag = BITSET(pRegs->ARM_cpsr, 29);
			offset = BITSET(instr, 25)
			    ? armShiftedRegVal(pRegs, instr, cFlag)
			    : BITS(instr, 0, 11);

			if (!BITSET(instr, 23))	/* down */
				offset = -offset;

			if (BITSET(instr, 24))	/* pre-indexed */
				base += offset;

			/* Can'to do this : nPc = *(INSTR *) base; */
			read_instr(child, base, &nPc, 0);
			pr_debug("nPc1 = %x\n", nPc);

			/*
			 * don't check for nPc == pc like the ARM debugger does but
			 * let the higher level (or user) notice.
			 */
		}
		break;

	case 8:
	case 9:		/* block transfer */
		if (BITSET(instr, 20) && BITSET(instr, 15)) {	/* loading PC */
			UINT32 rn;
			INT32 offset = 0;

			rn = BITS(instr, 16, 19);
			if (BITSET(instr, 23)) {	/* up */
				UINT32 regBit, regList;

				for (regList = BITS(instr, 0, 14); regList != 0;
				     regList &= ~regBit) {
					regBit = regList & (-regList);
					offset += 4;
				}
				if (BITSET(instr, 24))	/* preincrement */
					offset += 4;
			} else /* down */ if (BITSET(instr, 24))	/* predecrement */
				offset = -4;

			/* Can't do this nPc = *(UINT32 *) (pRegs->uregs[rn] + offset); */
			read_instr(child, (pRegs->uregs[rn] + offset), &nPc, 0);
			pr_debug("nPc2 = %x\n", nPc);

			/*
			 * don't check for nPc == pc like the ARM debugger does but
			 * let the higher level (or user) notice.
			 */
		}
		break;

	case 0xA:		/* branch */
	case 0xB:		/* branch & link */
		/*
		 * extract offset, sign extend it and add it to current PC,
		 * adjusting for the pipeline
		 */
		nPc = pc + 8 + ((INT32) (instr << 8) >> 6);
		break;

	case 0xC:
	case 0xD:
	case 0xE:		/* coproc ops */
	case 0xF:		/* SWI */
		break;
	}

	return (u32 *) nPc;

}				/* armGetNpc() */
