/*
 * arm paravirt.h - arm paravirtual operations structures
 *
 * Copyright (c) 2011 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#ifndef __ARM_ASM_PARAVIRT_H
#define __ARM_ASM_PARAVIRT_H

#include <linux/kprobes.h>

#ifdef CONFIG_PARAVIRT
/*
 * paravirtual operations structures
 */

struct pv_time_ops {
	void (*percpu_timer_setup)(void);
};

struct pv_smp_ops {
	void (*smp_init_cpus)(void);
	void (*smp_prepare_cpus)(unsigned int max_cpus);
	void (*smp_cross_call)(const struct cpumask *mask);
	int (*boot_secondary)(unsigned int cpu, struct task_struct *idle);
	void (*platform_secondary_init)(unsigned int cpu);
};

struct pv_cpu_ops {
	void (*do_idle)(void);
};

/* general info */
struct pv_info {
	const char *name;
	int paravirt_enabled;
};

struct pv_irq_ops {
	void (*do_IRQ)(struct pt_regs *regs);
};

struct pv_mmu_ops {
	void (*MMU_init)(void);
	void (*do_switch_mm)(unsigned long pgd_phys, struct mm_struct *mm);
	void (*set_pte_ext)(pte_t *ptep, pte_t pte, unsigned int ext);
	pgd_t *(*cpu_get_pgd)(void);
};

extern struct pv_info pv_info;
extern struct pv_time_ops pv_time_ops;
extern struct pv_cpu_ops pv_cpu_ops;
extern struct pv_irq_ops pv_irq_ops;
extern struct pv_mmu_ops pv_mmu_ops;
extern struct pv_smp_ops pv_smp_ops;

#endif /* CONFIG_PARAVIRT */
#endif	/* __ARM_ASM_PARAVIRT_H */
