/*
 * Architecture specific (MIPS) functions for kexec based crash dumps.
 *
 * Copyright (C) 2005, IBM Corp.
 * Copyright (C) 2008, MontaVista Software Inc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/reboot.h>
#include <linux/kexec.h>
#include <linux/bootmem.h>
#include <linux/crash_dump.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/sched.h>

/* This keeps a track of which one is crashing cpu. */
int crashing_cpu = -1;
static cpumask_t cpus_in_crash = CPU_MASK_NONE;

#ifdef CONFIG_SMP

void crash_shutdown_secondary(void *ignore) {
	struct pt_regs *regs;
	int cpu = smp_processor_id();

	regs = task_pt_regs(current);
	if (!cpu_online(cpu))
		return;

	local_irq_disable();
	if (!cpu_isset(cpu, cpus_in_crash))
		crash_save_cpu(regs, cpu);
	cpu_set(cpu, cpus_in_crash);

	while (!atomic_read(&kexec_ready_to_reboot))
		cpu_relax();
	relocated_kexec_smp_wait(NULL);
	/* NOTREACHED */
}

static void crash_kexec_prepare_cpus(void) {
	unsigned int msecs;

	/* Excluding the panic cpu */
	unsigned int ncpus = num_online_cpus() - 1;

	smp_call_function(crash_shutdown_secondary, NULL, 0);
	smp_wmb();

	/*
	 * FIXME: Until we will have the way to stop other CPUSs reliabally,
	 * the crash CPU will send an IPI and wait for other CPUs to
	 * respond.
	 * Delay of at least 10 seconds.
	 */
	printk(KERN_EMERG "Sending IPI to other cpus...\n");
	msecs = 10000;
	while ((cpus_weight(cpus_in_crash) < ncpus) && (--msecs > 0)) {
		cpu_relax();
		mdelay(1);
	}

}

#else
static void crash_kexec_prepare_cpus()
{
}
#endif

void default_machine_crash_shutdown(struct pt_regs *regs) {
	local_irq_disable();
	crashing_cpu = smp_processor_id();
	crash_save_cpu(regs, crashing_cpu);
	crash_kexec_prepare_cpus();
	cpu_set(crashing_cpu, cpus_in_crash);
}
