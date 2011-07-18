/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Copyright (C) 2009 Wind River Systems, Inc.
 */
#ifndef _ASM_WRHV_H
#define _ASM_WRHV_H


#define WRHV_RESERVED_PAGES	16
#define WRHV_RESERVED_TOP	(WRHV_RESERVED_PAGES * PAGE_SIZE)
extern void wrhv_init(void);
extern void wrhv_boot_config(void);
extern void wrhv_cpu_workarounds(struct cpuinfo_x86 *);
extern void wrhv_vtlb_op(unsigned int, unsigned long,
			 unsigned long, unsigned long);

extern unsigned long __initrd_start, __initrd_end;

DECLARE_PER_CPU(struct clock_event_device, wrhv_clock_events);

/*
 * Define IRQ names for IPI, the names *must* match
 * these strings in XML
 */
#define WRHV_IPI_NAME_RESCHED          "ipi0"
#define WRHV_IPI_NAME_INV_TLB          "ipi1"
#define WRHV_IPI_NAME_FUNC_CALL        "ipi2"
#define WRHV_IPI_NAME_FUNC_CALL_SINGLE "ipi3"

/*
 * For hyp-guest os, int 0 exclusively belongs to hypervisor-provided
 * timer,If external HW timers are used as system timer(Now it only
 * happens when hrtimer is enabled), we have to use other int number.
 */
#define HRTIMER_IRQ_NAME "PIT_Timer"
#define WRHV_VTIMER_INT  0
#define TIMER_INT_NUM    10
#define DUMMY_TIMER_INT  12

#ifdef CONFIG_SMP
extern void wrhv_calibrate_smp_cpus(void);
#endif
#ifdef CONFIG_PCI
extern void wrhv_init_pci(void);
#endif

#ifdef CONFIG_X86_32
#define WRS_SYSCALL_VECTOR SYSCALL_VECTOR
#else
#define WRS_SYSCALL_VECTOR IA32_SYSCALL_VECTOR
#endif

#endif /* _ASM_WRHV_H */
