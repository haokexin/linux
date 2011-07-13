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

    Adaptation for powerpc based on x86 version, Copyright (C) 2009
    Wind River Systems, Inc.

*/

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/efi.h>
#include <linux/bcd.h>
#include <linux/highmem.h>

#include <asm/bug.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/time.h>
#include <asm/pgalloc.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/fixmap.h>
#include <asm/tlbflush.h>
#include <asm/machdep.h>
#include <asm/fs_pd.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/pagemap.h>

#include <linux/kprobes.h>
#include <linux/kexec.h>
#include <linux/backlight.h> 
#include <linux/bug.h>
#include <linux/kdebug.h>
#include <linux/kallsyms.h> 

#include <mm/mmu_decl.h>
#include <linux/lmb.h>

#include <linux/major.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/spi/spi.h>
#include <linux/fsl_devices.h>
#include <linux/fs_enet_pd.h>
#include <linux/fs_uart_pd.h>

#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <sysdev/fsl_soc.h>
#include <asm/cpm2.h>

#include <linux/kgdb.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/ptrace.h>
#include <asm/current.h>
#include <asm/processor.h>

#include <asm/wrhv.h>
#include <asm/paravirt.h>

#ifdef CONFIG_WRHV
extern void wrhv_init(void);
#endif

/* XXX fixme - use an existing implementation */
#if 1
#define DEBUGP printk
#else
#define DEBUGP(fmt , ...)       do { } while (0)
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
	.time_init_cont = native_time_init_cont,
	.timer_interrupt = native_timer_interrupt,
	.clocksource_init = native_clocksource_init,
};

struct pv_irq_ops pv_irq_ops = {
	.do_IRQ = native_do_IRQ,
	.irq_of_parse_and_map = native_irq_of_parse_and_map,
};

struct pv_cpu_ops pv_cpu_ops = {
	.get_pvr = native_get_pvr,
	.get_svr = native_get_svr,
	.DebugException = native_DebugException,
	.prime_debug_regs = native_prime_debug_regs,
#ifdef CONFIG_KGDB
	.kgdb_arch_handle_exception = native_kgdb_arch_handle_exception,
#endif
	.ppc_proc_freq = native_ppc_proc_freq,
};

struct pv_mmu_ops pv_mmu_ops = {
	.vmmu_restore = native_vmmu_restore,
	.MMU_init_hw = native_MMU_init_hw,
	.mmu_mapin_ram = native_mmu_mapin_ram,
	.MMU_setup = native_MMU_setup,
	.MMU_init = native_MMU_init,
	.flush_dcache_page = native_flush_dcache_page,
	.map_page = native_map_page,
	.early_init_dt_scan_memory_ppc =
		native_early_init_dt_scan_memory_ppc,
	.__ioremap = native___ioremap,
	.__set_pte_at = native__set_pte_at,
};

struct pv_mdio_ops pv_mdio_ops = {
	.fsl_pq_mdio_write	= native_fsl_pq_mdio_write,
	.fsl_pq_mdio_read	= native_fsl_pq_mdio_read,
};

struct pv_context_ops pv_context_ops = {
	.init_new_context	= native_init_new_context,
	.destroy_context	= native_destroy_context,
	.switch_mmu_context	= native_switch_mmu_context,
	.mmu_context_init	= native_mmu_context_init,
};

struct pv_serial_ops pv_serial_ops = {
	.udbg_init_uart	= native_udbg_init_uart,
};

/* pv_time_ops */
void __init paravirt_time_init_cont(void)
{
	pv_time_ops.time_init_cont();
}

void paravirt_timer_interrupt(struct pt_regs * regs)
{	
	pv_time_ops.timer_interrupt(regs);
}

void __init paravirt_clocksource_init(void)
{
	pv_time_ops.clocksource_init();
}


/* pv_context_ops */
int paravirt_init_new_context(struct task_struct *t, struct mm_struct *mm)
{
	return pv_context_ops.init_new_context(t, mm);
}

void paravirt_destroy_context(struct mm_struct *mm)
{
	pv_context_ops.destroy_context(mm);
}

void paravirt_switch_mmu_context(struct mm_struct *prev, struct mm_struct *next)
{
	pv_context_ops.switch_mmu_context(prev, next);
}

void __init paravirt_mmu_context_init(void)
{
	pv_context_ops.mmu_context_init();
}

/* pv_irq_ops */
void paravirt_do_IRQ(struct pt_regs *regs)
{
	pv_irq_ops.do_IRQ(regs);
}


unsigned int paravirt_irq_of_parse_and_map(struct device_node *dev, int index)
{
	return pv_irq_ops.irq_of_parse_and_map(dev, index);
}

/* pv_cpu_ops */
unsigned int paravirt_get_pvr(void)
{
	return pv_cpu_ops.get_pvr();
}

unsigned int paravirt_get_svr(void)
{
	return pv_cpu_ops.get_svr();
}

int fsl_pq_mdio_write(struct mii_bus *bus, int mii_id, int devad,
				int regnum, u16 value)
{
	return pv_mdio_ops.fsl_pq_mdio_write(bus, mii_id, devad, regnum, value);
}

int fsl_pq_mdio_read(struct mii_bus *bus, int mii_id,
				int devad, int regnum)
{
	return pv_mdio_ops.fsl_pq_mdio_read(bus, mii_id, devad, regnum);
}

void paravirt_udbg_init_uart(void __iomem *comport, unsigned int speed,
		unsigned int clock)
{
	return pv_serial_ops.udbg_init_uart(comport, speed, clock);
}

void __kprobes paravirt_DebugException(struct pt_regs *regs, unsigned long debug_status)
{
	pv_cpu_ops.DebugException(regs, debug_status);

}

void  paravirt_prime_debug_regs(struct thread_struct *thread)
{
	pv_cpu_ops.prime_debug_regs(thread);
}

int paravirt_kgdb_arch_handle_exception(int vector, int signo, int err_code,
			char *remcom_in_buffer, char *remcom_out_buffer,
			struct pt_regs *linux_regs)
{

	return pv_cpu_ops.kgdb_arch_handle_exception(vector, signo, err_code,
			remcom_in_buffer, remcom_out_buffer, linux_regs);
}


int paravirt_ppc_proc_freq(void)
{
	return pv_cpu_ops.ppc_proc_freq();
}

/* pv_mmu_ops */
void paravirt_vmmu_restore (void)
{
	pv_mmu_ops.vmmu_restore();
}

void __init paravirt_MMU_init_hw(void)
{
	pv_mmu_ops.MMU_init_hw();
}

unsigned long __init paravirt_mmu_mapin_ram(unsigned long top)
{
	return pv_mmu_ops.mmu_mapin_ram(top);
}

void paravirt_MMU_setup(void)
{
	pv_mmu_ops.MMU_setup();
}

void __init paravirt_MMU_init(void)
{
	pv_mmu_ops.MMU_init();
}

void paravirt_flush_dcache_page(struct page *page)
{
	pv_mmu_ops.flush_dcache_page(page);
}

int paravirt_map_page(unsigned long va, phys_addr_t pa, int flags)
{
	return	pv_mmu_ops.map_page(va, pa, flags);
}

int paravirt_early_init_dt_scan_memory_ppc(unsigned long node,
		const char *uname, int depth, void *data)
{
       return pv_mmu_ops.early_init_dt_scan_memory_ppc(node,
					uname, depth, data);
}

void paravirt___ioremap(phys_addr_t addr, unsigned long size, unsigned long flags)
{
	pv_mmu_ops.__ioremap(addr, size, flags);
}

void paravirt__set_pte_at(struct mm_struct *mm, unsigned long addr, 
					pte_t *ptep, pte_t pte, int percpu) 
{
	pv_mmu_ops.__set_pte_at(mm, addr, ptep, pte, percpu);
}

inline int paravirt_enabled(void)
{
        return pv_info.paravirt_enabled;
}

#ifdef CONFIG_PCI
int paravirt_pci_read_irq_line(struct pci_dev *dev)
{
	return 0;
}
#endif

extern struct pv_time_ops pv_time_ops;
extern struct pv_cpu_ops pv_cpu_ops;
extern struct pv_irq_ops pv_irq_ops;
extern struct pv_mmu_ops pv_mmu_ops; 
extern struct pv_mdio_ops pv_mdio_ops;
extern struct pv_context_ops pv_context_ops;

EXPORT_SYMBOL    (pv_info);
EXPORT_SYMBOL    (pv_time_ops);
EXPORT_SYMBOL    (pv_cpu_ops);
EXPORT_SYMBOL    (pv_context_ops);
EXPORT_SYMBOL    (pv_mmu_ops);
EXPORT_SYMBOL    (pv_irq_ops);
EXPORT_SYMBOL    (pv_mdio_ops);
