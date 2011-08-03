/*  Paravirtualization interfaces

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    Copyright (C) 2011 Wind River Systems, Inc.

*/

#include <linux/module.h>
#include <asm/setup.h>
#include <asm/paravirt.h>

#ifdef CONFIG_WRHV
extern void wrhv_init(void);
#endif

/* paravirt init */
void paravirt_init(void)
{
#ifdef CONFIG_WRHV
	wrhv_init();
#endif
}

struct pv_info pv_info = {
	.name = "bare hardware",
	.paravirt_enabled = 0,
};

/* default native operations */
struct pv_time_ops pv_time_ops = {
};

struct pv_irq_ops pv_irq_ops = {
};

struct pv_cpu_ops pv_cpu_ops = {
};

struct pv_mmu_ops pv_mmu_ops = {
};

struct pv_smp_ops pv_smp_ops = {
};


/* pv_irq_ops */
void paravirt_do_IRQ(struct pt_regs *regs)
{
	pv_irq_ops.do_IRQ(regs);
}

void paravirt_do_idle(void)
{
	pv_cpu_ops.do_idle();
}

void __init paravirt_MMU_init(void)
{
	pv_mmu_ops.MMU_init();
}

void paravirt_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext)
{
	pv_mmu_ops.set_pte_ext(ptep, pte, ext);
}

void paravirt_do_switch_mm(unsigned long pgd_phys, struct mm_struct *mm)
{
	pv_mmu_ops.do_switch_mm(pgd_phys, mm);
}

pgd_t *paravirt_cpu_get_pgd(void)
{
	return pv_mmu_ops.cpu_get_pgd();
}

inline int paravirt_enabled(void)
{
	return pv_info.paravirt_enabled;
}

void paravirt_smp_init_cpus(void)
{
	pv_smp_ops.smp_init_cpus();
}

void paravirt_smp_prepare_cpus(unsigned int max_cpus)
{
	pv_smp_ops.smp_prepare_cpus(max_cpus);
}

int paravirt_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	return pv_smp_ops.boot_secondary(cpu, idle);
}

void paravirt_platform_secondary_init(unsigned int cpu)
{
	pv_smp_ops.platform_secondary_init(cpu);
}

void paravirt_smp_cross_call(const struct cpumask *mask)
{
	pv_smp_ops.smp_cross_call(mask);
}

void paravirt_percpu_timer_setup(void)
{
	pv_time_ops.percpu_timer_setup();
}

EXPORT_SYMBOL(pv_info);
EXPORT_SYMBOL(pv_time_ops);
EXPORT_SYMBOL(pv_cpu_ops);
EXPORT_SYMBOL(pv_mmu_ops);
EXPORT_SYMBOL(pv_irq_ops);
EXPORT_SYMBOL(pv_smp_ops);
