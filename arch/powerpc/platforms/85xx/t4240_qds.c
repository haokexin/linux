/*
 * T4240 QDS Setup
 *
 * Maintained by Kumar Gala (see MAINTAINERS for contact information)
 *
 * Copyright 2012 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/phy.h>

#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <mm/mmu_decl.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/mpic.h>

#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include <asm/ehv_pic.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "corenet_ds.h"

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init t4240_qds_probe(void)
{
	unsigned long root = of_get_flat_dt_root();
#ifdef CONFIG_SMP
	extern struct smp_ops_t smp_85xx_ops;
#endif

	if (of_flat_dt_is_compatible(root, "fsl,T4240QDS"))
		return 1;

	/* Check if we're running under the Freescale hypervisor */
	if (of_flat_dt_is_compatible(root, "fsl,T4240QDS-hv")) {
		ppc_md.init_IRQ = ehv_pic_init;
		ppc_md.get_irq = ehv_pic_get_irq;
		ppc_md.restart = fsl_hv_restart;
		ppc_md.power_off = fsl_hv_halt;
		ppc_md.halt = fsl_hv_halt;
#ifdef CONFIG_SMP
		/*
		 * Disable the timebase sync operations because we can't write
		 * to the timebase registers under the hypervisor.
		  */
		smp_85xx_ops.give_timebase = NULL;
		smp_85xx_ops.take_timebase = NULL;
#endif
		return 1;
	}

	return 0;
}

define_machine(t4240_qds) {
	.name			= "T4240 QDS",
	.probe			= t4240_qds_probe,
	.setup_arch		= corenet_ds_setup_arch,
	.init_IRQ		= corenet_ds_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
#endif
/* coreint doesn't play nice with lazy EE, use legacy mpic for now */
#ifdef CONFIG_PPC64
	.get_irq		= mpic_get_irq,
#else
	.get_irq		= mpic_get_coreint_irq,
#endif
	.restart		= fsl_rstcr_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
#ifdef CONFIG_PPC64
	.power_save		= book3e_idle,
#else
	.power_save		= e500_idle,
#endif
	.init_early		= corenet_ds_init_early,
};

machine_arch_initcall(t4240_qds, corenet_ds_publish_pci_device);
machine_device_initcall(t4240_qds, declare_of_platform_devices);

#ifdef CONFIG_SWIOTLB
machine_arch_initcall(t4240_qds, swiotlb_setup_bus_notifier);
#endif

#ifdef CONFIG_T4240_RLT_ERRATUM

#define E6500_THREADS_PER_CORE	2
#define E6500_CORES_PER_CLUSTER 4
#define E6500_CLUSTER_SIZE	(E6500_CORES_PER_CLUSTER * E6500_THREADS_PER_CORE)
#define E6500_ERRATUM_INTERVAL	1000000000  /* 1 sec in nanoseconds */
#define E6500_WAIT_TICKS	5000

static __iomem u32 *ccsr_e2000;
static __iomem u32 *ccsr_mmr_l2;
static __iomem u32 *dcsr_rcpm;

static struct errata_info_t {
	struct hrtimer timer;
	struct cpumask cluster_cpus;
	u32 thread_mask;
	u32 core_mask;
	u32 mmr_l2_offset;
	u32 *dcsr_copdbg[4];
} errata_info[NR_CPUS/E6500_CLUSTER_SIZE];

static DEFINE_SPINLOCK(errata_lock);

enum hrtimer_restart t4240_qds_erratum_cb(struct hrtimer *timer)
{
	u32 saved[5];
	int cpu = smp_processor_id();
	struct errata_info_t *ei = &errata_info[cpu / E6500_CLUSTER_SIZE];
#ifndef CONFIG_T4_SIMULATOR_WORKAROUND
	u32 tbl;
	int i;
	u32 saved2[E6500_CORES_PER_CLUSTER][3];
#endif

	/* we don't want to put two clusters to sleep at once */
	spin_lock(&errata_lock);

	/* step 1 */
	saved[0] = in_be32(&ccsr_e2000[0x15c]);
	saved[1] = in_be32(&ccsr_e2000[0x16c]);
	saved[2] = in_be32(&ccsr_e2000[0x17c]);
	saved[3] = in_be32(&ccsr_e2000[0x18c]);

	/* step 2 */
	setbits32(&ccsr_e2000[0x15c], ei->thread_mask);
	setbits32(&ccsr_e2000[0x16c], ei->thread_mask);
	setbits32(&ccsr_e2000[0x17c], ei->thread_mask);
	setbits32(&ccsr_e2000[0x18c], ei->thread_mask);

	/* step 2.1 */
	setbits32(&ccsr_e2000[0x02c], ei->thread_mask);

	/* step 2.2 */
#ifndef CONFIG_T4_SIMULATOR_WORKAROUND
	for_each_cpu(i, &ei->cluster_cpus) {
		u32 offset;
		int core = i / E6500_THREADS_PER_CORE;

		offset = 0x100 + (0x4 * i);
		spin_event_timeout((in_be32(&dcsr_rcpm[offset]) & (1 << 25) ) == 0, 5000);

		/* every other thread to hit one time per core */
		if (i % E6500_THREADS_PER_CORE)
			continue;

		/* step 2.3 */
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00200001);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0xf8000000);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		saved2[core][0] = in_be32(&ei->dcsr_copdbg[core][0x600]);
		saved2[core][1] = in_be32(&ei->dcsr_copdbg[core][0x604]);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x0);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x7c1e92a6);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00100001);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x90000000);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		saved2[core][2] = in_be32(&ei->dcsr_copdbg[core][0x604]);
		out_be32(&ei->dcsr_copdbg[core][0x604], 0x00000000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00100001);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x80000000);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00000000);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x7c0004ac);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00000000);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x7c1e93a6);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
	}

	/* step 5 */
	saved[4] = in_be32(&ccsr_mmr_l2[ei->mmr_l2_offset+ 0x004]);
	/* step 6 */
	clrbits32(&ccsr_mmr_l2[ei->mmr_l2_offset + 0x004], 0xff);

	/* step 7 */
	tbl = mfspr(SPRN_ATBL);
	while(tbl + E6500_WAIT_TICKS > mfspr(SPRN_ATBL));

	/* step 8 */
	setbits32(&ccsr_mmr_l2[ei->mmr_l2_offset + 0xf04], 0x30000000);
	/* step 9 */
	clrbits32(&ccsr_mmr_l2[ei->mmr_l2_offset + 0xf04], 0x30000000);

	/* step 10 */
	clrsetbits_be32(&ccsr_mmr_l2[ei->mmr_l2_offset + 0x004], 0xff, saved[4] & 0xff);
#endif

	/* step 12.1 */
	setbits32(&ccsr_e2000[0x01c], ei->thread_mask);

	/* step 12.2 */
#ifndef CONFIG_T4_SIMULATOR_WORKAROUND
	for_each_cpu(i, &ei->cluster_cpus) {
		u32 offset;
		int core = i / E6500_THREADS_PER_CORE;

		offset = 0x100 + (0x4 * i);
		while((in_be32(&dcsr_rcpm[offset]) & (1 << 25) ) != 0);

		/* every other thread to hit one time per core */
		if (i % E6500_THREADS_PER_CORE) continue;

		/* step 12.3 */
		out_be32(&ei->dcsr_copdbg[core][0x604], saved2[core][2]);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00100001);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x80000000);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00000000);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x7c0004ac);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00000000);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0x7c1e93a6);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
		out_be32(&ei->dcsr_copdbg[core][0x600], saved2[core][0]);
		out_be32(&ei->dcsr_copdbg[core][0x604], saved2[core][1]);
		out_be32(&ei->dcsr_copdbg[core][0x608], 0x00200001);
		out_be32(&ei->dcsr_copdbg[core][0x60c], 0xe8000000);
		spin_event_timeout(in_be32(&ei->dcsr_copdbg[core][0x10]) != 0x08000000, 5000);
	}
#endif

	/* step 13 */
	out_be32(&ccsr_e2000[0x15c], saved[0]);
	out_be32(&ccsr_e2000[0x16c], saved[1]);
	out_be32(&ccsr_e2000[0x17c], saved[2]);
	out_be32(&ccsr_e2000[0x18c], saved[3]);

	spin_unlock(&errata_lock);

	hrtimer_forward(timer, timer->base->get_time(),
			ns_to_ktime(E6500_ERRATUM_INTERVAL));
	return HRTIMER_RESTART;
}

static void __init t4240_qds_install_hrtimer(void *arg)
{
	int cpu = smp_processor_id();
	struct errata_info_t *ei = &errata_info[cpu / E6500_CLUSTER_SIZE];

	hrtimer_init(&ei->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	ei->timer.function = &t4240_qds_erratum_cb;
	/* delay the first run by 1 sec */
	hrtimer_start(&ei->timer,
			ktime_add_us(ns_to_ktime(E6500_ERRATUM_INTERVAL), 1e6),
			HRTIMER_MODE_REL);
}

static int __init t4240_qds_erratum_workaround(void)
{
	phys_addr_t res2_size = (num_present_cpus() / E6500_CLUSTER_SIZE) *
					0x40000;
	int i, j;

	/* TODO: verify we have at least two clusters otherwise this workaround
	 * won't help */
	ccsr_e2000 = ioremap(get_immrbase() + 0xE2000, 0x2fff);
	if (!ccsr_e2000)
		pr_crit("__func__: Unable to ioremap first block!\n");
		return -ENOMEM;

	ccsr_mmr_l2 = ioremap(get_immrbase() + 0xC20000, res2_size);
	if (!ccsr_mmr_l2)
		pr_crit("__func__: Unable to ioremap second block!\n");

	dcsr_rcpm = ioremap(get_dcsrbar() + 0x22000, 0x2fff);
	if (!dcsr_rcpm)
		pr_crit("__func__: Unable to ioremap third block!\n");

	for (i = 0; i < num_present_cpus() / E6500_CLUSTER_SIZE; i++) {
		/* create proper mask for the adjacent cluster */
		int tgt_cpu = i * E6500_CLUSTER_SIZE + E6500_CLUSTER_SIZE;
		struct errata_info_t *ei = &errata_info[i];

		cpumask_clear(&ei->cluster_cpus);
		for (j = tgt_cpu;
		     j < tgt_cpu + E6500_CLUSTER_SIZE;
		     j++) {
			int cpu_thread = j % num_present_cpus();
			int core = cpu_thread / E6500_THREADS_PER_CORE;
			if (cpu_thread % E6500_THREADS_PER_CORE == 0)
				ei->core_mask |= (1 << (cpu_thread / E6500_THREADS_PER_CORE));
			ei->thread_mask |= (1 << cpu_thread);
			cpumask_set_cpu(cpu_thread, &ei->cluster_cpus);
			ei->dcsr_copdbg[core] = ioremap(get_dcsrbar() + 0x100000 + (core * 0x8000), 0xfff);
			if(!ei->dcsr_copdbg[core]) {
				pr_crit("__func__: Unable to ioremap fourth block for cpu %d\n", i);
			}
		}
		ei->mmr_l2_offset = 0x40000 * i;

		smp_call_function_single(i * E6500_CLUSTER_SIZE,
				t4240_qds_install_hrtimer, NULL, 0);
	}

	return 0;
}
__initcall(t4240_qds_erratum_workaround);
#endif
