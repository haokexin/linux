// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * SMC testing module
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/linkage.h>
#include <linux/axxia-oem.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <asm/smp_plat.h>
#include <linux/seq_file.h>
#include <uapi/linux/psci.h>

#define SMC_DEBUG 1
#if SMC_DEBUG
#define smc_dbg(...) pr_info(__VA_ARGS__)
#define seq_printf(...)
#define seq_puts(...)
#else
#define smc_dbg(...)
#define seq_printf(arg0, ...) seq_printf(arg0, __VA_ARGS__)
#define seq_puts(arg0, ...) seq_puts(arg0, __VA_ARGS__)
#endif

struct _pt_regs {
	unsigned long elr;
	unsigned long regs[31];
};

typedef void (*smc_cb_t)(void *);

static bool all_cpus;
module_param(all_cpus, bool, 0644);
MODULE_PARM_DESC(all_cpus,
		 "Run smc call on executing or all cpus (default: all");

static void smc_call_v10_invoke_psci_ver(void *arg);
static void smc_call_v11_execute_mitigation_to_CVE_2017_5715(void *arg);
static void smc_call_v10_smccc_ver_from_psci_feature(void *arg);
static void smc_call_v11_smccc_ver_directly_less_safe(void *arg);
static void smc_call_v11_determine_availability_of_arm_arch_service(void *arg);
static void smc_call_v11_psci_v11_query_if_memprotect_is_enabled(void *args);
static void smc_call_v11_psci_v11_query_if_mem_reset2_is_enabled(void *args);

smc_cb_t smc_call[] = {
	smc_call_v10_invoke_psci_ver,
	smc_call_v11_execute_mitigation_to_CVE_2017_5715,
	smc_call_v10_smccc_ver_from_psci_feature,
	smc_call_v11_smccc_ver_directly_less_safe,
	smc_call_v11_determine_availability_of_arm_arch_service,
	smc_call_v11_psci_v11_query_if_memprotect_is_enabled,
	smc_call_v11_psci_v11_query_if_mem_reset2_is_enabled
};

#define SMC_VARIANT(n) n
#define for_each_smc(smc)					\
	for ((smc) = 0; (smc) < ARRAY_SIZE(smc_call); (smc)++)

unsigned long __percpu *time[ARRAY_SIZE(smc_call)];

void
zeroise_times(void)
{
	int cpu, smc;

	for_each_smc(smc) {
		for_each_online_cpu(cpu) {
			*per_cpu_ptr(time[smc], cpu) = 0;
		}
	}
}

static void run_test(void)
{
	int smc;

	/* init the smc call regs before the next call round */
	zeroise_times();

	switch (all_cpus) {
	case true:
		for_each_smc(smc) {
			int cpu;

			for_each_online_cpu(cpu) {
				int ret;

				ret = smp_call_function_single(cpu,
							       smc_call[smc],
							       (void *)&smc,
							       1);
				if (ret)
					WARN_ON(ret);
			}
		}
		break;
	case false:
		for_each_smc(smc)
			smc_call[smc]((void *)&smc);
		break;
	}
}

void smc_call_v10_invoke_psci_ver(void *arg)
{
	struct _pt_regs _regs = {0}, *regs = &_regs;
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res = {0};
	ktime_t start, end;

	regs->regs[0] = PSCI_0_2_FN_PSCI_VERSION;

	preempt_disable();

	start = ktime_get();
	arm_smccc_smc(regs->regs[0], regs->regs[1], regs->regs[2],
		      regs->regs[3], 0, 0, 0, 0, &res);
	end = ktime_get();

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret %#lx)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		res.a0);

	preempt_enable();
}

void smc_call_v11_execute_mitigation_to_CVE_2017_5715(void *arg)
{
	int smc = *(unsigned int *)arg;
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_1_1_smc(ARM_SMCCC_ARCH_WORKAROUND_1, NULL);
	end = ktime_get();

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (%s)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		"no return value");

	preempt_enable();
}

/*
 * Retrieve the implemented version of the SMCCC from PSCI services. It
 * is the safest way to retrieve this as compatible with the system not
 * having the SMCCC v1.1 implemented.
 */

void smc_call_v10_smccc_ver_from_psci_feature(void *arg)
{
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res = {0};
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_smc(PSCI_1_0_FN_PSCI_FEATURES,
		      ARM_SMCCC_VERSION_FUNC_ID, 0, 0, 0, 0, 0, 0, &res);

	if (res.a0 != PSCI_RET_NOT_SUPPORTED)
		arm_smccc_smc(ARM_SMCCC_VERSION_FUNC_ID,
			      0, 0, 0, 0, 0, 0, 0, &res);

	end = ktime_get();

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret %#lx)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		res.a0);

	preempt_enable();
}

/* Retrieve the implemented version of the SMCCC */
void smc_call_v11_smccc_ver_directly_less_safe(void *arg)
{
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res = {0};
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_1_1_smc(0x80000000, &res);
	end = ktime_get();

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret %#lx)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		res.a0);

	preempt_enable();
}

/* Determine the availability and capability of Arm Architecture Service
 * functions
 */
void smc_call_v11_determine_availability_of_arm_arch_service(void *arg)
{
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res = {0};
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_1_1_smc(0x80000001/*SMCCC_ARCH_FEATURES*/,
			  0x80000001/*again in x1*/,
			  &res);
	end = ktime_get();

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret %#lx)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		res.a0);

	preempt_enable();
}

/* MEM_PROTECT function provides protection against cold reboot attacks, by
 * ensuring that memory is overwritten before it is handed over to an
 * operating system loader. No callback registered for Axxia.
 */
void smc_call_v11_psci_v11_query_if_memprotect_is_enabled(void *arg)
{
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res = {0};
	s32 val;
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_1_1_smc(0x84000013/*MEM_PROTECT*/, &res);
	end = ktime_get();

	val = (s32)res.a0;

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret %d)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		val);

	preempt_enable();
}

/* SYSTEM_RESET2 function extends SYSTEM_RESET. It provides:
 * - Architectural reset definitions.
 * - Support for vendor-specific resets.
 */
void smc_call_v11_psci_v11_query_if_mem_reset2_is_enabled(void *arg)
{
	int smc = *(unsigned int *)arg;
	struct arm_smccc_res res[2] = {0};
	s32 val[2] = {0};
	ktime_t start, end;

	preempt_disable();

	start = ktime_get();
	arm_smccc_1_1_smc(0x84000012 /*SYSTEM_RESET2 for SMC32 version*/,
			  0 << 31    /*arch. resets, bit31 0*/,
			  &res[0]);
	arm_smccc_1_1_smc(0xC4000012 /*SYSTEM_RESET2 for SMC64 version*/,
			  0 << 31    /*arch. resets, bit31 0*/,
			  &res[1]);
	end = ktime_get();

	val[0] = (s32)res[0].a0;
	val[1] = (s32)res[1].a0;

	this_cpu_write(*time[smc], ktime_to_ns(ktime_sub(end, start)));
	smc_dbg("%s: cpu%d %lu ns (ret32 %d ret64 %d)\n",
		__func__, smp_processor_id(),
		this_cpu_read(*time[smc]),
		val[0], val[1]);

	preempt_enable();
}

static int proc_smc_show(struct seq_file *m, void *v)
{
	int smc;

	run_test();

	for_each_smc(smc) {
		int cpu;

		seq_printf(m, "SMC_VARIANT(%u): %ps\n", smc, smc_call[smc]);
		for_each_online_cpu(cpu)
			seq_printf(m, "CPU%-2d ", cpu);
		seq_puts(m, "\n");
		for_each_online_cpu(cpu)
			seq_printf(m, "%-5lu ",
				   *per_cpu_ptr(time[SMC_VARIANT(smc)], cpu));
		seq_puts(m, "\n");
	}

	return 0;
}

static int proc_smc_open(struct inode *inode, struct file *file);
static const struct proc_ops proc_smc_operations = {
	.proc_open		= proc_smc_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int proc_smc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_smc_show, NULL);
}

static struct proc_dir_entry *procent;
static int __init proc_dma_init(void)
{
	procent = proc_create("axxia_smccall", 0444, NULL,
			      &proc_smc_operations);
	return 0;
}

static int __init smctest_init(void)
{
	unsigned int smc;

	for_each_smc(smc)
		time[smc] = alloc_percpu(unsigned long);

	zeroise_times();
	proc_dma_init();

	return 0;
}
late_initcall(smctest_init);

static void __exit smctest_exit(void)
{
	unsigned int smc;

	for_each_smc(smc)
		if (time[smc]) {
			free_percpu(time[smc]);
			time[smc] = NULL;
		}

	if (procent) {
		proc_remove(procent);
		procent = NULL;
	}
}

module_exit(smctest_exit);

MODULE_AUTHOR("Marek Bykowski");
MODULE_LICENSE("GPL v2");
