/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Copyright (C) 2008 Wind River Systems, Inc.
 */

#ifndef __ASM_WRHV_H
#define __ASM_WRHV_H

#ifdef CONFIG_WRHV

/*
 * This macro will tend to clean_dcache_range function, but
 * we do not need to do that, so delete below macro to try
 * native clean_dcache_range
 */
#define CONFIG_PARAVIRT_DCACHE_CLEAN

#ifndef __ASSEMBLY__

extern unsigned char *law_base;

extern void wrhv_mapping(void);
extern void wrhv_restart(char *cmd);
extern unsigned long __init wrhv_find_end_of_memory(void);
extern void wrhv_power_save(void);
extern unsigned int wrhv_vioapic_get_irq(void);
extern void wrhv_init_irq(void);
extern void __init wrhv_calibrate_decr(void);
extern void __init wrhv_time_init(void);
extern int __init wrhv_earlycon_setup(void);
extern void wrhv_setup_msr_for_ap(VBI_HREG_SET_CMPLX_QUALIFIED *);

extern int __init smp_wrhv_probe(void);
extern void smp_wrhv_message_pass(int target, int msg);
extern void __init smp_wrhv_setup_cpu(int cpu_nr);
extern void wrhv_umask_IPIs_for_vcore(void);
extern void wrhv_request_ipis(void);
extern unsigned long wrhv_cpu_freq;

extern uint32_t service_handle;
extern void get_hv_bsp_server_handle(void);
extern int get_bsp_clock_freq(void);

extern void ppc_setup_law(unsigned int target_id, unsigned long long addr, unsigned int attr);
extern int ppc_search_free_law(int target_id);
extern int ppc_setup_pci_law(struct device_node *dev);

#ifdef CONFIG_PCI
extern int wrhv_enable_pci_law(void);
#endif

extern unsigned int wrhv_get_direct_irq(void);

/* following extern functions is implemented in wrhv.c
 * to access hypervisor serial driver
 */
extern void wrhv_duart_putc(char c);
extern int wrhv_duart_tstc(void);
extern int wrhv_duart_getc(void);
extern int wrhv_duart_init(void);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_WRHV */
#endif /* __ASM_WRHV_H */
