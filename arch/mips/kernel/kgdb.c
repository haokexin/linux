/*
 *  Originally written by Glenn Engel, Lake Stevens Instrument Division
 *
 *  Contributed by HP Systems
 *
 *  Modified for Linux/MIPS (and MIPS in general) by Andreas Busse
 *  Send complaints, suggestions etc. to <andy@waldorf-gmbh.de>
 *
 *  Copyright (C) 1995 Andreas Busse
 *
 *  Copyright (C) 2003 MontaVista Software Inc.
 *  Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 *  Copyright (C) 2004-2005 MontaVista Software Inc.
 *  Author: Manish Lachwani, mlachwani@mvista.com or manish@koffee-break.com
 *
 *  Copyright (C) 2007-2008 Wind River Systems, Inc.
 *  Author/Maintainer: Jason Wessel, jason.wessel@windriver.com
 *
 *  This file is licensed under the terms of the GNU General Public License
 *  version 2. This program is licensed "as is" without any warranty of any
 *  kind, whether express or implied.
 */

#include <linux/ptrace.h>		/* for linux pt_regs struct */
#include <linux/kgdb.h>
#include <linux/kdebug.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <asm/inst.h>
#include <asm/fpu.h>
#include <asm/cacheflush.h>
#include <asm/processor.h>
#include <asm/sigcontext.h>

static struct hard_trap_info {
	unsigned char tt;	/* Trap type code for MIPS R3xxx and R4xxx */
	unsigned char signo;	/* Signal that we map this trap into */
} hard_trap_info[] = {
	{ 6, SIGBUS },		/* instruction bus error */
	{ 7, SIGBUS },		/* data bus error */
	{ 9, SIGTRAP },		/* break */
/*	{ 11, SIGILL },	*/	/* CPU unusable */
	{ 12, SIGFPE },		/* overflow */
	{ 13, SIGTRAP },	/* trap */
	{ 14, SIGSEGV },	/* virtual instruction cache coherency */
	{ 15, SIGFPE },		/* floating point exception */
	{ 23, SIGSEGV },	/* watch */
	{ 31, SIGSEGV },	/* virtual data cache coherency */
	{ 0, 0}			/* Must be last */
};

struct dbg_reg_def_t dbg_reg_def[DBG_MAX_REG_NUM] =
{
	{ "zero", sizeof(long), offsetof(struct pt_regs, regs[0]) },
	{ "at", sizeof(long), offsetof(struct pt_regs, regs[1]) },
	{ "v0", sizeof(long), offsetof(struct pt_regs, regs[2]) },
	{ "v1", sizeof(long), offsetof(struct pt_regs, regs[3]) },
	{ "a0", sizeof(long), offsetof(struct pt_regs, regs[4]) },
	{ "a1", sizeof(long), offsetof(struct pt_regs, regs[5]) },
	{ "a2", sizeof(long), offsetof(struct pt_regs, regs[6]) },
	{ "a3", sizeof(long), offsetof(struct pt_regs, regs[7]) },
	{ "t0", sizeof(long), offsetof(struct pt_regs, regs[8]) },
	{ "t1", sizeof(long), offsetof(struct pt_regs, regs[9]) },
	{ "t2", sizeof(long), offsetof(struct pt_regs, regs[10]) },
	{ "t3", sizeof(long), offsetof(struct pt_regs, regs[11]) },
	{ "t4", sizeof(long), offsetof(struct pt_regs, regs[12]) },
	{ "t5", sizeof(long), offsetof(struct pt_regs, regs[13]) },
	{ "t6", sizeof(long), offsetof(struct pt_regs, regs[14]) },
	{ "t7", sizeof(long), offsetof(struct pt_regs, regs[15]) },
	{ "s0", sizeof(long), offsetof(struct pt_regs, regs[16]) },
	{ "s1", sizeof(long), offsetof(struct pt_regs, regs[17]) },
	{ "s2", sizeof(long), offsetof(struct pt_regs, regs[18]) },
	{ "s3", sizeof(long), offsetof(struct pt_regs, regs[19]) },
	{ "s4", sizeof(long), offsetof(struct pt_regs, regs[20]) },
	{ "s5", sizeof(long), offsetof(struct pt_regs, regs[21]) },
	{ "s6", sizeof(long), offsetof(struct pt_regs, regs[22]) },
	{ "s7", sizeof(long), offsetof(struct pt_regs, regs[23]) },
	{ "t8", sizeof(long), offsetof(struct pt_regs, regs[24]) },
	{ "t9", sizeof(long), offsetof(struct pt_regs, regs[25]) },
	{ "k0", sizeof(long), offsetof(struct pt_regs, regs[26]) },
	{ "k1", sizeof(long), offsetof(struct pt_regs, regs[27]) },
	{ "gp", sizeof(long), offsetof(struct pt_regs, regs[28]) },
	{ "sp", sizeof(long), offsetof(struct pt_regs, regs[29]) },
	{ "s8", sizeof(long), offsetof(struct pt_regs, regs[30]) },
	{ "ra", sizeof(long), offsetof(struct pt_regs, regs[31]) },
	{ "sr", sizeof(long), offsetof(struct pt_regs, cp0_status) },
	{ "lo", sizeof(long), offsetof(struct pt_regs, lo) },
	{ "hi", sizeof(long), offsetof(struct pt_regs, hi) },
	{ "bad", sizeof(long), offsetof(struct pt_regs, cp0_badvaddr) },
	{ "cause", sizeof(long), offsetof(struct pt_regs, cp0_cause) },
	{ "pc", sizeof(long), offsetof(struct pt_regs, cp0_epc) },
};

int dbg_set_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno < 0 || regno >= DBG_MAX_REG_NUM)
		return -EINVAL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy((void *)regs + dbg_reg_def[regno].offset, mem,
		       dbg_reg_def[regno].size);
	return 0;
}

char *dbg_get_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return NULL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy(mem, (void *)regs + dbg_reg_def[regno].offset,
		       dbg_reg_def[regno].size);
	return dbg_reg_def[regno].name;
}

void arch_kgdb_breakpoint(void)
{
	__asm__ __volatile__(
		".globl breakinst\n\t"
		".set\tnoreorder\n\t"
		"nop\n"
		"breakinst:\tbreak\n\t"
		"nop\n\t"
		".set\treorder");
}

static void kgdb_call_nmi_hook(void *ignored)
{
	kgdb_nmicallback(raw_smp_processor_id(), NULL);
}

void kgdb_roundup_cpus(unsigned long flags)
{
	local_irq_enable();
	smp_call_function(kgdb_call_nmi_hook, NULL, 0);
	local_irq_disable();
}

static int compute_signal(int tt)
{
	struct hard_trap_info *ht;

	for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		if (ht->tt == tt)
			return ht->signo;

	return SIGHUP;		/* default for things we don't know about */
}

#if 0
void pt_regs_to_gdb_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;

	/* FP REGS */
	if (!(current && (regs->cp0_status & ST0_CU1)))
		return;

	save_fp(current);
	for (reg = 0; reg < 32; reg++)
		*(ptr++) = current->thread.fpu.fpr[reg];
}

void gdb_regs_to_pt_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;

	/* FPREGS from current */
	if (!(current && (regs->cp0_status & ST0_CU1)))
		return;

	for (reg = 0; reg < 32; reg++)
		current->thread.fpu.fpr[reg] = *(ptr++);
	restore_fp(current);
}
#endif
/*
 * Similar to regs_to_gdb_regs() except that process is sleeping and so
 * we may not be able to get all the info.
 */
void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	int reg;
	struct thread_info *ti = task_thread_info(p);
	unsigned long ksp = (unsigned long)ti + THREAD_SIZE - 32;
	struct pt_regs *regs = (struct pt_regs *)ksp - 1;
#if (KGDB_GDB_REG_SIZE == 32)
	u32 *ptr = (u32 *)gdb_regs;
#else
	u64 *ptr = (u64 *)gdb_regs;
#endif

	for (reg = 0; reg < 16; reg++)
		*(ptr++) = regs->regs[reg];

	/* S0 - S7 */
	for (reg = 16; reg < 24; reg++)
		*(ptr++) = regs->regs[reg];

	for (reg = 24; reg < 28; reg++)
		*(ptr++) = 0;

	/* GP, SP, FP, RA */
	for (reg = 28; reg < 32; reg++)
		*(ptr++) = regs->regs[reg];

	*(ptr++) = regs->cp0_status;
	*(ptr++) = regs->lo;
	*(ptr++) = regs->hi;
	*(ptr++) = regs->cp0_badvaddr;
	*(ptr++) = regs->cp0_cause;
	*(ptr++) = regs->cp0_epc;
}

void kgdb_arch_set_pc(struct pt_regs *regs, unsigned long pc)
{
	regs->cp0_epc = pc;
}

/*
 * Calls linux_debug_hook before the kernel dies. If KGDB is enabled,
 * then try to fall into the debugger
 */
static int kgdb_mips_notify(struct notifier_block *self, unsigned long cmd,
			    void *ptr)
{
	struct die_args *args = (struct die_args *)ptr;
	struct pt_regs *regs = args->regs;
	int trap = (regs->cp0_cause & 0x7c) >> 2;

	/* Userpace events, ignore. */
	if (user_mode(regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_active) != -1)
		kgdb_nmicallback(smp_processor_id(), regs);

	if (kgdb_handle_exception(trap, compute_signal(trap), cmd, regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_setting_breakpoint))
		if ((trap == 9) && (regs->cp0_epc == (unsigned long)breakinst))
			regs->cp0_epc += 4;

	/* In SMP mode, __flush_cache_all does IPI */
	local_irq_enable();
	__flush_cache_all();

	return NOTIFY_STOP;
}

#ifdef CONFIG_KGDB_LOW_LEVEL_TRAP
int kgdb_ll_trap(int cmd, const char *str,
		 struct pt_regs *regs, long err, int trap, int sig)
{
	struct die_args args = {
		.regs	= regs,
		.str	= str,
		.err	= err,
		.trapnr	= trap,
		.signr	= sig,

	};

	if (!kgdb_io_module_registered)
		return NOTIFY_DONE;

	return kgdb_mips_notify(NULL, cmd, &args);
}
#endif /* CONFIG_KGDB_LOW_LEVEL_TRAP */

static struct notifier_block kgdb_notifier = {
	.notifier_call = kgdb_mips_notify,
};

/*
 * Handle the 's' and 'c' commands
 */
int kgdb_arch_handle_exception(int vector, int signo, int err_code,
			       char *remcom_in_buffer, char *remcom_out_buffer,
			       struct pt_regs *regs)
{
	char *ptr;
	unsigned long address;
	int cpu = smp_processor_id();

	switch (remcom_in_buffer[0]) {
	case 's':
	case 'c':
		/* handle the optional parameter */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &address))
			regs->cp0_epc = address;

		atomic_set(&kgdb_cpu_doing_single_step, -1);
		if (remcom_in_buffer[0] == 's')
			atomic_set(&kgdb_cpu_doing_single_step, cpu);

		return 0;
	}

	return -1;
}

struct kgdb_arch arch_kgdb_ops;

/*
 * We use kgdb_early_setup so that functions we need to call now don't
 * cause trouble when called again later.
 */
int kgdb_arch_init(void)
{
	union mips_instruction insn = {
		.r_format = {
			.opcode = spec_op,
			.func   = break_op,
		}
	};
	memcpy(arch_kgdb_ops.gdb_bpt_instr, insn.byte, BREAK_INSTR_SIZE);

	register_die_notifier(&kgdb_notifier);

	return 0;
}

/*
 *	kgdb_arch_exit - Perform any architecture specific uninitalization.
 *
 *	This function will handle the uninitalization of any architecture
 *	specific callbacks, for dynamic registration and unregistration.
 */
void kgdb_arch_exit(void)
{
	unregister_die_notifier(&kgdb_notifier);
}
