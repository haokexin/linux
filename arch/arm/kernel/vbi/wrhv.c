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
 *  Copyright (C) 2011 Wind River Systems, Inc.
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/wrhv.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/clockchips.h>
#include <linux/kernel_stat.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/paravirt.h>
#include <asm/smp_scu.h>
#include <asm/wrhv.h>

#include <trace/irq.h>

#include <vbi/vbi.h>
#include <vbi/interface.h>
#include <vbi/vmmu.h>

#define WRHV_BOOTARG_BUF_SIZE	256

#define IPI_IRQ_BASE_NAME	"ipi"

#define TIMERTICK_IRQ		0

DEFINE_PER_CPU(struct clock_event_device, wrhv_clock_events);
spinlock_t vmmu_handle_lock;

/* wr_config is set super early to a pointer passed from the hv */
struct vb_config *wr_config = (void *)(-1); /* keep it out of the bss */
EXPORT_SYMBOL(wr_config);

/* control and status pointers are set from info in the config region */
struct vb_control *wr_control;
struct vb_status *wr_status;
extern struct vb_status *LCstatptr;

static char *direct_interrupts_list;
static int __init wrhv_check_direct_interrupts(char *str)
{
	direct_interrupts_list = str;
	return 0;
}

early_param("direct_interrupts", wrhv_check_direct_interrupts);

int wrhv_find_direct_interrupt(int intr_candidate)
{
	char	direct_interrupts[WRHV_BOOTARG_BUF_SIZE];
	char	*current_tok;
	char	*position;
	int	intr_num;

	if (direct_interrupts_list == NULL)
		return -ENOENT;

	strncpy(direct_interrupts, direct_interrupts_list,
		WRHV_BOOTARG_BUF_SIZE-1);
	direct_interrupts[WRHV_BOOTARG_BUF_SIZE-1] = '\0'; /* be safe */
	position = direct_interrupts;
	/* crack by ',' first */
	while ((current_tok = strsep(&position, ","))) {
		if (sscanf(current_tok, "%d", &intr_num) != 1)
			return -EINVAL; /* broken list */

			if (intr_num == intr_candidate) {
				/* We have a match! */
				return 0;
			}
	}
	/* Didn't find it */
	return -ENOENT;
}

unsigned long __init wrhv_find_end_of_memory(void)
{
	return wr_config->phys_mem_size;
}

char wrhv_super_early_stack[WRHV_SUPER_EARLY_STACK_SIZE];

void wrhv_load_initial_vmmu(uint32_t pgtbl)
{
	/* This is called super early.  We have a stack, and wr_config
	 * has been set, but that's about it.  If the core we're starting
	 * is anything other than the bp (core0), then pgtbl represents
	 * the vmmu handle which was already created.
	 */

	VMMU_CONFIG	vmmu_cfg;

	vbi_enable_vmmu(0);

	if (wr_config->coreId == 0) {
		/* Initial startup of the boot core */
		vmmu_cfg.addr = pgtbl;
		vmmu_cfg.flush_type = 0;
		vmmu_cfg.asid = 0;
		if (vbi_create_vmmu(&vmmu_cfg) != 0) {
			printk(KERN_ERR "Could not create initial vmmu!\n");
			while (1) {}; /* No point in continuing */
		
		} else {
			wr_config->vb_control->vb_control_regs.vmmu_handle =
				vmmu_cfg.vmmu_handle;
			init_mm.context.vmmu_handle = vmmu_cfg.vmmu_handle;
		}
	} else
		wr_config->vb_control->vb_control_regs.vmmu_handle = pgtbl;
	wr_config->vb_control->vb_control_regs.asid = 0;
	if (vbi_load_ctx() != 0) {
		printk(KERN_ERR "Could not load initial context!\n");
		while (1) {}; /* No point in continuing */
	}
}

static void wrhv_do_restart(void *data)
{
	int ret;
	int cpu = smp_processor_id();

	if (!cpu) {
		printk(KERN_INFO "WRHV: rebooting\n");

		ret = vbi_vb_reset(VBI_BOARD_ID_GET(), VBI_VB_CORES_ALL,
				VBI_VBMGMT_RESET_AND_START_CORE0 |
				VBI_VBMGMT_RESET_DOWNLOAD
				);

		if (ret)
			printk(KERN_ERR "WRHV: reboot failed. ret = %d\n", ret);
	}
}

void wrhv_restart(char str, const char *cmd)
{
	int cpu = smp_processor_id();

	if (!cpu)
		wrhv_do_restart(NULL);
	else
		smp_call_function(wrhv_do_restart, NULL, 1);

	while (1);
}

#ifdef CONFIG_SMP
static irqreturn_t wrhv_ipi_interrupt(int irq, void *dev_id)
{
	do_IPI(get_irq_regs());
	return IRQ_HANDLED;
}

int ipi_irq = VBI_INVALID_IRQ;	/* Make sure it gets set before use */

void wrhv_unmask_IPIs_for_vcore(void)
{
	printk(KERN_INFO "CPU%d: Unmasking ipi %d\n", smp_processor_id(),
		ipi_irq);
	vbi_unmask_vioapic_irq(ipi_irq);
}

int wrhv_request_ipis(void)
{
	static char *ipi_names[] = {
		"IPI (all ipi functions)",
	};

	int err;

	ipi_irq = vbi_find_irq(IPI_IRQ_BASE_NAME, VB_INPUT_INT);
	if (ipi_irq == VBI_INVALID_IRQ) {
		printk(KERN_ERR "WRHV lookup of interrupt name '"
				IPI_IRQ_BASE_NAME
				"' failed!\n");
		panic("WRHV resolve irq for IPI failed\n");
	}

	set_irq_chip_and_handler_name(ipi_irq, &wrhv_ipi_irq_chip,
				      handle_percpu_irq, "per_cpu");
	err = request_irq(ipi_irq, wrhv_ipi_interrupt,
			  IRQF_DISABLED | IRQF_NOBALANCING,
			  ipi_names[0], wrhv_ipi_interrupt);
	if (err) {
		printk(KERN_ERR "WRHV request of irq %d for IPI(%s) failed\n",
		       ipi_irq, ipi_names[0]);
	} else
		wrhv_unmask_IPIs_for_vcore(); /* Allow the BP to receive them */
	return err;
}
#endif

void wrhv_smp_init_cpus(void)
{
	unsigned int i, ncores;

	/* Ask the vbi how many cores we have */
	ncores = VBI_VCORES_COUNT_GET();

	/* Check if we have more available than configured to support */
	if (ncores > NR_CPUS) {
		printk(KERN_WARNING
		       "wrhv: no. of cores (%d) greater than configured "
		       "maximum of %d - clipping\n",
		       ncores, NR_CPUS);
		ncores = NR_CPUS;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

void wrhv_smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = num_possible_cpus();
	unsigned int cpu = smp_processor_id();
	int i;

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	if (max_cpus > 1)
		if (wrhv_request_ipis()) {
			printk(KERN_ERR "IPI init issue, continuing in UP\n");
			return;		/* something went wrong */
		}

	/*
	 * Initialize the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);
}

static DEFINE_SPINLOCK(boot_lock);

extern volatile int pen_release;
int __cpuinit wrhv_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int ret;
	unsigned long timeout;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	pen_release = cpu;
	ret = vbi_vb_resume(VBI_BOARD_ID_GET(), cpu);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	if (pen_release != -1)
		return -ENOSYS;


	return ret;
}

extern void __iomem *gic_cpu_base_addr;
void __cpuinit wrhv_platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	vbi_set_exc_base((void *)CONFIG_VECTORS_BASE);

	if (direct_interrupts_list)
		gic_cpu_init(0, gic_cpu_base_addr);

	wrhv_unmask_IPIs_for_vcore();

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

irqreturn_t wrhv_timer_interrupt(int irq, void *dev_id)
{
	u64 ticks;
	static DEFINE_PER_CPU(u64, mark_offset);
	static DEFINE_PER_CPU(int, mark_first_time) = 1;
	int cpu = smp_processor_id();
	struct clock_event_device *evt = &per_cpu(wrhv_clock_events, cpu);

	if (!evt->event_handler) {
		printk(KERN_WARNING
			   "Spurious Hyp timer interrupt on cpu %d\n", cpu);
		return IRQ_NONE;
	}

	if (__get_cpu_var(mark_first_time) == 0) {
		ticks = wr_vb_status->tick_count;
		ticks -= __get_cpu_var(mark_offset);
		__get_cpu_var(mark_offset) = wr_vb_status->tick_count;
		if (ticks > (2*HZ)) {
			printk(KERN_DEBUG "Time falling behind %lld jiffies\n",
				ticks);
			ticks = 1;
		}
	} else {
		ticks = 1;
		__get_cpu_var(mark_first_time) = 0;
		__get_cpu_var(mark_offset) = wr_vb_status->tick_count;
	}

	if (ticks > 1)
		account_steal_time(jiffies_to_cputime(ticks - 1));

	while (ticks != 0) {
		evt->event_handler(evt);
		ticks--;
	}

	return IRQ_HANDLED;
}
static struct irqaction wrhv_timer_irq = {
	.handler = wrhv_timer_interrupt,
	.flags = IRQF_DISABLED | IRQF_NOBALANCING,
	.name = "timer",
};

#ifdef CONFIG_SPARSE_IRQ
#define WRHV_NR_IRQS	NR_IRQS_LEGACY
#else
#define WRHV_NR_IRQS	NR_IRQS
#endif

void (*wrhv_machine_init_irq)(void) __initdata = NULL;
void (*wrhv_machine_init_timer)(void) __initdata = NULL;

void __init wrhv_init_irq(void)
{
	int i;
	struct irq_desc *desc;

#ifdef CONFIG_SMP
	/* By default all the irqs will be routed to core0 */
	cpumask_copy(irq_default_affinity, cpumask_of(0));
#endif

	wrhv_irq_chip.typename = "WRHV-PIC";
	for (i = 0; i < WRHV_NR_IRQS; i++) {
		desc = irq_to_desc_alloc_node(i, 0);
		desc->status = IRQ_DISABLED | IRQ_LEVEL;
		desc->action = NULL;
		desc->depth = 1;
		set_irq_chip_and_handler(i, &wrhv_irq_chip, handle_fasteoi_irq);
	}

	/* Do any hardware specific init to support direct irqs */
	if (wrhv_machine_init_irq && direct_interrupts_list)
		wrhv_machine_init_irq();
	else
		printk(KERN_INFO "WRHV: No direct irq support\n");
}

void wrhv_do_IRQ(struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);
	unsigned int irq;
	int handled_at_least_one = 0;

	trace_irq_entry(0, regs, NULL);

	irq_enter();

	/* To be implemented if there is a simple way...
	check_stack_overflow();
	*/

check_again:
	irq = vbi_get_pending_vioapic_irq();

	if (irq != 0xffff) {
		generic_handle_irq(irq);
		handled_at_least_one = 1;
		goto check_again;
	} else if (!handled_at_least_one)
		printk(KERN_WARNING "WRHV: Spurious interrupt!\n");

	irq_exit();
	set_irq_regs(old_regs);

	trace_irq_exit(IRQ_HANDLED);
}

static void wrhv_set_mode(enum clock_event_mode mode,
				 struct clock_event_device *dev)
{
	return;
}

static int wrhv_set_next_event(unsigned long evt,
				      struct clock_event_device *dev)
{
	return 0;
}

static struct clock_event_device wrhv_clockevent = {
	.name		= "wrhv",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.set_mode	= wrhv_set_mode,
	.set_next_event = wrhv_set_next_event,
	.max_delta_ns	= 0xffffffff,
	.min_delta_ns	= 10000,
	.shift		= 32,   /* nanoseconds to cycles divisor 2^ */
	.mult		= 1,     /* To be filled in */
	.irq		= TIMERTICK_IRQ,
	.rating		= 1,
};

void __init wrhv_time_init(void)
{
	struct clock_event_device *evt;

	evt = &per_cpu(wrhv_clock_events, 0);
	memcpy(evt, &wrhv_clockevent, sizeof(*evt));
	evt->cpumask = cpumask_of(0);

	clockevents_register_device(evt);
	setup_irq(TIMERTICK_IRQ, &wrhv_timer_irq);
	vbi_unmask_vioapic_irq(TIMERTICK_IRQ); /* Allow timer ints through */

	if (wrhv_machine_init_timer)
		wrhv_machine_init_timer();
}

void __devinit wrhv_setup_secondary_clock(void)
{
	int cpu;
	struct clock_event_device *evt;
	cpu = smp_processor_id();
	printk(KERN_INFO "installing wrhv timer for CPU %d\n", cpu);

	evt = &per_cpu(wrhv_clock_events, cpu);
	memcpy(evt, &wrhv_clockevent, sizeof(*evt));
	evt->cpumask = cpumask_of(cpu);

	clockevents_register_device(evt);

	vbi_unmask_vioapic_irq(TIMERTICK_IRQ); /* Allow timer ints through */
}

pgd_t *wrhv_cpu_get_pgd(void)
{
	return current->active_mm->pgd;
}

void wrhv_do_idle(void)
{
	vbi_idle(1);
}

extern int __init arm_add_memory(unsigned long start, unsigned long size);

void __init wrhv_MMU_init(void)
{

	__u32 start;
	__u32 size;

	start = 0x00000000ul;
	size = wrhv_find_end_of_memory();

	printk(KERN_DEBUG "Total %dK memory added\n", size / 1024);

	arm_add_memory(start, size);

}

void wrhv_set_pte_ext(pte_t *ptep, pte_t pte, unsigned int ext)
{
	native_cpu_set_pte_ext(ptep, pte, ext);

	/* We really want to just flush the modified entry for obvious
	 * efficiency reasons.  But hypervisor issues right now mean that
	 * we must flush them all.  The third parameter, length, being set
	 * to -1 will flush them all.  We will leave the second parameter set
	 * to what we really just want to flush even though it's ignored.
	 */
	vbi_flush_tlb(0, (void *)(pte & (-PAGE_SIZE)), -1);
}

void wrhv_do_switch_mm(unsigned long pgd_phys, struct mm_struct *mm)
{
	wr_control->vb_control_regs.vmmu_handle = mm->context.vmmu_handle;
	wr_control->vb_control_regs.asid = 0;
	if (vbi_load_ctx() != 0)
		printk(KERN_WARNING "Bad vmmu handle %lu\n",
				mm->context.vmmu_handle);
	/* Flush everything for now, until we support asids.
	 */
	vbi_flush_tlb(0, 0, -1);
}

void wrhv_smp_cross_call(const struct cpumask *mask)
{
	unsigned long coreset = cpus_addr(*mask)[0];
	unsigned long flags;

	local_irq_save(flags);
	WARN_ON(coreset & ~cpus_addr(cpu_online_map)[0]);
	vbi_send_vcore_vioapic_irq(ipi_irq, coreset, VBI_IOAPICSEND_VCORE_NONE);
	local_irq_restore(flags);
}

void wrhv_calculate_clock_freq(void)
{
	u64 lpj;
	/* Hypervisor doesn't fill in the stamp freq field yet
	unsigned long cpu_khz = wrhv_calculate_cpu_khz();
	*/
	unsigned long cpu_khz = 400 * 1000; /* 400 MHz default */

	lpj = ((u64)cpu_khz * 1000);
	do_div(lpj, HZ);
	preset_lpj = lpj;

	printk(KERN_INFO "Detected %lu.%03lu MHz processor.\n", cpu_khz / 1000,
		cpu_khz % 1000);
}

void __init wrhv_init(void)
{
	/* wr_config was already set, super early */
	vbi_init(wr_config);

	wr_control = wr_config->vb_control;
	wr_status = LCstatptr = wr_config->vb_status;


	pv_info.name = "wrhv";
	pv_info.paravirt_enabled = 1;

	pv_irq_ops.do_IRQ = wrhv_do_IRQ;

	pv_cpu_ops.do_idle = wrhv_do_idle;

	pv_smp_ops.smp_init_cpus = wrhv_smp_init_cpus;
	pv_smp_ops.smp_prepare_cpus = wrhv_smp_prepare_cpus;
	pv_smp_ops.smp_cross_call = wrhv_smp_cross_call;
	pv_smp_ops.boot_secondary = wrhv_boot_secondary;
	pv_smp_ops.platform_secondary_init = wrhv_platform_secondary_init;

	pv_mmu_ops.MMU_init = wrhv_MMU_init;
	pv_mmu_ops.do_switch_mm = wrhv_do_switch_mm;
	pv_mmu_ops.set_pte_ext = wrhv_set_pte_ext;
	pv_mmu_ops.cpu_get_pgd = wrhv_cpu_get_pgd;

	pv_time_ops.percpu_timer_setup = wrhv_setup_secondary_clock;

	snprintf(boot_command_line, COMMAND_LINE_SIZE,
		"retain_initrd %s",
	wr_config->bootLine);

	wrhv_calculate_clock_freq();

	arm_pm_restart = wrhv_restart;
	spin_lock_init(&vmmu_handle_lock);
}
