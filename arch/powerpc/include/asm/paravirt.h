#ifndef __ASM_PARAVIRT_H
#define __ASM_PARAVIRT_H

#ifdef CONFIG_PARAVIRT
/*
 * native functions
 */
extern void native_do_IRQ(struct pt_regs *regs);
extern unsigned int native_irq_of_parse_and_map(struct device_node *dev,
						int index);
extern unsigned int native_get_pvr(void);
extern unsigned int native_get_svr(void);
extern void native_timer_interrupt(struct pt_regs * regs);
extern void __init native_time_init(void);
extern void __init native_clocksource_init(void);
extern void native_vmmu_restore (void);
extern void __init native_MMU_init_hw(void);
extern unsigned long __init native_mmu_mapin_ram(unsigned long top);
extern void native_MMU_setup(void);
extern void __init native_MMU_init(void);
extern void native_flush_dcache_page(struct page *page);
extern int native_map_page(unsigned long va, phys_addr_t pa, int flags);
extern int native_kgdb_arch_handle_exception(int vector, int signo,
				int err_code,
				char *remcom_in_buffer,
				char *remcom_out_buffer,
				struct pt_regs *linux_regs);
extern void __kprobes native_DebugException(struct pt_regs *regs,
				unsigned long debug_status);
extern void native_prime_debug_regs(struct thread_struct *thread);
extern int __init native_early_init_dt_scan_memory_ppc(unsigned long node, 
			const char *uname, int depth, void *data);
extern void __init native_time_init_cont(void);
extern void __iomem* native___ioremap(phys_addr_t addr, unsigned long size, unsigned long flags);
extern int __attribute__((weak)) native_fsl_pq_mdio_write(struct mii_bus *bus, int mii_id,
					int devad, int regnum, u16 value);
extern int __attribute__((weak)) native_fsl_pq_mdio_read(struct mii_bus *bus, int mii_id,
					int devad, int regnum);
extern void native_udbg_init_uart(void __iomem *comport, unsigned int speed,
		unsigned int clock);

extern int native_init_new_context(struct task_struct *t, struct mm_struct *mm);
extern void native_destroy_context(struct mm_struct *mm);
extern void native_switch_mmu_context(struct mm_struct *prev,
		struct mm_struct *next);
extern void __init native_mmu_context_init(void);

/*
 * paravirtual operations structure
 */
struct pv_time_ops {
	void (*time_init_cont)(void);
	void (*timer_interrupt)(struct pt_regs *regs);
	void (*clocksource_init)(void);
};

struct pv_cpu_ops {
	unsigned int (*get_pvr)(void);
	unsigned int (*get_svr)(void);
	void (*DebugException)(struct pt_regs *regs, unsigned long debug_status);
	void (*prime_debug_regs)(struct thread_struct *thread);
	int (*kgdb_arch_handle_exception)(int vector, int signo, int err_code,
                               char *remcom_in_buffer, char *remcom_out_buffer,
                               struct pt_regs *linux_regs);
	int (*ppc_proc_freq)(void);
};

/* general info */
struct pv_info {
        const char *name;
        int paravirt_enabled;
};

struct pv_irq_ops {
	void (*do_IRQ)(struct pt_regs *regs);
	unsigned int (*irq_of_parse_and_map)
		(struct device_node *dev, int index);
};

struct pv_apic_ops {
	unsigned int (*get_irq)(void);
	void (*do_irq)(struct pt_regs *regs);
	int (*get_ppc_spurious_interrupts)(void);
	void (*set_ppc_spurious_interrupts)(int value);
	unsigned int (*irq_of_parse_and_map)
		(struct device_node *dev, int index);

};

struct pv_mmu_ops {
	void (*vmmu_restore)(void);
	void (*MMU_init_hw)(void);
	unsigned long (*mmu_mapin_ram)(unsigned long top);
	void (*MMU_setup)(void);
	void (*MMU_init)(void);
	void (*flush_dcache_page)(struct page *page);
	int (*map_page)(unsigned long va, phys_addr_t pa, int flags);
	int (*early_init_dt_scan_memory_ppc)(unsigned long node,
			const char *uname, int depth, void *data);
	void __iomem* (*__ioremap)(phys_addr_t addr, unsigned long size, unsigned long flags);
	void (*__set_pte_at)(struct mm_struct *mm, unsigned long addr, 
		pte_t *ptep, pte_t pte, int percpu);
};

struct pv_mdio_ops {
	int (*fsl_pq_mdio_write)(struct mii_bus *bus, int mii_id,
					int devad, int regnum, u16 value);
	int (*fsl_pq_mdio_read)(struct mii_bus *bus, int mii_id,
					int devad, int regnum);
};

struct pv_context_ops {
	int (*init_new_context)(struct task_struct *t, struct mm_struct *mm);
	void (*destroy_context)(struct mm_struct *mm);
	void (*switch_mmu_context)(struct mm_struct *prev,
		struct mm_struct *next);
	void (*mmu_context_init)(void);
};

struct pv_serial_ops{
	void (*udbg_init_uart)(void __iomem *comport, unsigned int speed,
		unsigned int clock);
};

extern struct pv_info pv_info;
extern struct pv_time_ops pv_time_ops;
extern struct pv_cpu_ops pv_cpu_ops;
extern struct pv_irq_ops pv_irq_ops;
extern struct pv_mmu_ops pv_mmu_ops;
extern struct pv_mdio_ops pv_mdio_ops;
extern struct pv_context_ops pv_context_ops;
extern struct pv_serial_ops pv_serial_ops;

#endif /* CONFIG_PARAVIRT */
#endif	/* __ASM_PARAVIRT_H */
