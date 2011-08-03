/*
 *  arch/arm/include/asm/cpu-single.h
 *
 *  Copyright (C) 2000 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*
 * Single CPU
 */
#ifdef __STDC__
#define __catify_fn(name,x)	name##x
#else
#define __catify_fn(name,x)	name/**/x
#endif
#define __cpu_fn(name,x)	__catify_fn(name,x)

/*
 * If we are supporting multiple CPUs, then we must use a table of
 * function pointers for this lot.  Otherwise, we can optimise the
 * table away.
 */
#define cpu_proc_init			__cpu_fn(CPU_NAME, _proc_init)
#define cpu_proc_fin			__cpu_fn(CPU_NAME, _proc_fin)
#define cpu_reset			__cpu_fn(CPU_NAME, _reset)
#define native_cpu_do_idle		__cpu_fn(CPU_NAME, _do_idle)
#define cpu_dcache_clean_area		__cpu_fn(CPU_NAME, _dcache_clean_area)
#define native_cpu_do_switch_mm		__cpu_fn(CPU_NAME, _switch_mm)
#define native_cpu_set_pte_ext		__cpu_fn(CPU_NAME, _set_pte_ext)

extern void native_cpu_do_idle(void);
extern void native_cpu_do_switch_mm(unsigned long pgd_phys,
				struct mm_struct *mm);
extern void native_cpu_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext);
extern void paravirt_do_idle(void);
extern void paravirt_do_switch_mm(unsigned long pgd_phys, struct mm_struct *mm);
extern void paravirt_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext);

static inline void cpu_do_idle(void)
{
#ifdef CONFIG_PARAVIRT
	paravirt_do_idle();
#else
	native_cpu_do_idle();
#endif
}

static inline void cpu_do_switch_mm(unsigned long pgd_phys,
				struct mm_struct *mm)
{
#ifdef CONFIG_PARAVIRT
	paravirt_do_switch_mm(pgd_phys, mm);
#else
	native_cpu_do_switch_mm(pgd_phys, mm);
#endif
}

static inline void cpu_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext)
{
#ifdef CONFIG_PARAVIRT
	paravirt_set_pte_ext(ptep, pte, ext);
#else
	native_cpu_set_pte_ext(ptep, pte, ext);
#endif
}

#include <asm/page.h>

struct mm_struct;

/* declare all the functions as extern */
extern void cpu_proc_init(void);
extern void cpu_proc_fin(void);
extern void cpu_dcache_clean_area(void *, int);
extern void cpu_reset(unsigned long addr) __attribute__((noreturn));
