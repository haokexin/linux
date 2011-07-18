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
 *  Copyright (C) 2009 Wind River Systems, Inc.
 */


#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/wrhv.h>
#include <linux/kgdb.h>
#include <linux/kernel_stat.h>
#include <vbi/vbi.h>
#include <vbi/compat.h>
#include <vbi/stats.h>
#include <asm/setup.h>
#include <asm/paravirt.h>
#include <asm/processor.h>
#include <asm/wrhv.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/trampoline.h>
#include <linux/percpu.h>
#include <linux/smp.h>
#include <linux/clockchips.h>      /* for enum clock_event_mode */
#include <asm/cpu.h>
#include <asm/reboot.h>            /* for struct machine_ops */
#include <asm/x86_init.h>

#include <asm/desc.h>
#include <asm/e820.h>
#include <asm/i8259.h>


#define WRHV_DEBUG_MSR          0
#define WRHV_USE_XMLCONFIG      1

#define WRHV_BOOTARG_BUF_SIZE   256

/* Copied over during early bootstrap */
struct vb_config __wr_config = { .pid = -1 };
/* Pointer passed from hypervisor */
struct vb_config *_wr_config = &__wr_config;
struct vb_config *wr_config = &__wr_config;
struct vb_config *fake_wr_config = &__wr_config;
struct vb_status *wr_status;
struct vb_control *wr_control;

static unsigned int wrhv_ipi_num[4] __read_mostly = {20, 21, 22, 23};
#define WRHV_IPI_RESCHED          wrhv_ipi_num[0]
#define WRHV_IPI_INV_TLB          wrhv_ipi_num[1]
#define WRHV_IPI_FUNC_CALL        wrhv_ipi_num[2]
#define WRHV_IPI_FUNC_CALL_SINGLE wrhv_ipi_num[3]

#ifdef CONFIG_PCI
extern int (*pcibios_enable_irq)(struct pci_dev *dev);
extern void (*pcibios_disable_irq)(struct pci_dev *dev);
#endif

static unsigned long cr3_val[NR_CPUS];
static struct vbi_vtlb_control vtlb_ctrl[NR_CPUS];
static unsigned long is_cr3_cache_enabled[NR_CPUS];
static unsigned long is_vtlb_optim_enabled[NR_CPUS];
static unsigned long is_vtlb_ops_cache_enabled[NR_CPUS];

#ifdef CONFIG_SMP
#define	VTLB_GET_CPU_VAR(var)	var[smp_processor_id()]
#else
#define	VTLB_GET_CPU_VAR(var)	var[0]
#endif

#ifdef CONFIG_SMP
static cpumask_t flush_cpumask;
static struct mm_struct *flush_mm;
static unsigned long flush_va;
static DEFINE_RAW_SPINLOCK(tlbstate_lock);
static DEFINE_SPINLOCK(vioapic_lock);
#endif

static int enable_hrtimer = 0;
static int novtlbopt;
DEFINE_PER_CPU(struct clock_event_device, wrhv_clock_events);

#define VBI_VTLB_OPTIM_OPTION (\
			 VBI_VTLB_OPTIM_ENABLED |  \
			 VBI_VTLB_CR3_CACHE_ENABLED |  \
			 VBI_VTLB_OPS_CACHE_ENABLED |  \
			 VBI_VTLB_DIRTY_BIT_SUPPORT_ENABLED)

#define VBI_VTLB_OPTIM_OPTION_NOOPT (VBI_VTLB_DIRTY_BIT_SUPPORT_ENABLED)

static inline void wrhv_send_IPI_mask(int, cpumask_t);
static void wrhv_setup_timer_irq(void);
static inline void wrhv_mask_timer_for_vcore(int irq);
static inline void wrhv_umask_timer_for_vcore(int irq);

static void wrhv_pre_intr_init_hook(void)
{
	int i;

#ifdef CONFIG_SMP
	/* Be default all the irqs will be routed to core0 */
	cpumask_copy(irq_default_affinity, cpumask_of(0));
#endif

	for (i = 0; i < NR_IRQS; i++) {
		irq_desc[i].status = IRQ_DISABLED | IRQ_MOVE_PCNTXT;
		irq_desc[i].action = NULL;
		irq_desc[i].depth = 1;
		set_irq_chip_and_handler_name(i, &wrhv_irq_chip,
				handle_fasteoi_irq, "fasteoi");
	}
}

#ifdef CONFIG_X86_32
static void __wrhv_map_page(unsigned long vaddr, unsigned long paddr,
				pgprot_t prot)
{
	pte_t pte = pfn_pte(paddr >> PAGE_SHIFT, prot);
	set_pte_vaddr(vaddr, pte);
}
#endif

#ifdef CONFIG_X86_64
static inline void construct_fake_wr_config(void)
{
	unsigned long delta;

	delta = (unsigned long)wr_config - (unsigned long)_wr_config;
	fake_wr_config = (struct vb_config *)
			 (delta + (unsigned long)wr_config->corePrivate);

	*fake_wr_config = *wr_config;
	fake_wr_config->vb_control = (struct vb_control *)
			(delta + (unsigned long)wr_config->vb_control);
	fake_wr_config->vb_status = (struct vb_status *)
			(delta + (unsigned long)wr_config->vb_status);

	if (wr_config->sharedMemoryRegionsConfigAddress != NULL) {
		fake_wr_config->sharedMemoryRegionsConfigAddress =
			(struct vb_sm_info *)(delta +
		(unsigned long)wr_config->sharedMemoryRegionsConfigAddress);
	}

	if (wr_config->memoryRegionsConfigAddress != NULL) {
		fake_wr_config->memoryRegionsConfigAddress =
			(struct vb_mem_info *)(delta +
		(unsigned long)wr_config->sharedMemoryRegionsConfigAddress);
	}

	fake_wr_config->interruptConfiguration = (struct vb_int_info *)
		(delta + (unsigned long)wr_config->interruptConfiguration);

	fake_wr_config->deviceConfiguration = (struct vb_dev_info *)(delta +
			 (unsigned long)wr_config->deviceConfiguration);

	if ((unsigned long)fake_wr_config->vb_control->vIoapic <
	    ((unsigned long)1 << 32))
		fake_wr_config->vb_control->vIoapic = (void *)(delta +
			(unsigned long)fake_wr_config->vb_control->vIoapic);
}

#endif

void __init wrhv_init_IRQ(void)
{
	int i;
	unsigned long addr;

#ifdef CONFIG_X86_32
	/* The following code maps in hypervisor config/status/control space.
	   It has to be carefully crafted to be an identity mapping.  We ask
	   for this space to be supplied to us from the hypervisor at
	   address 0xffff0000 in the virtual board xml, and we essentially
	   setup WRHV_RESERVED_PAGES to be 16, representing 16 4K pages from the
	   end of address space.  This gives us the address 0xffff0000 in Linux
	   which we need for a virt=phys aka identity mapping.  Why do we
	   need this to be identity mapped?  Because this block of memory
	   space is supplied by the hypervisor outside of Linux control, and
	   it contains pointers to places within itself.  We really don't want
	   to have to hunt down and modify all those pointers at run time to
	   be a different (virtual) address.  And finally we tell Linux
	   through the reservetop bootarg to actually move down the end of
	   memory by the size in bytes represented by WRHV_RESERVED_PAGES
	   so we don't interfere with Linux's fixmap facility.

	   The numbers mentioned above are examples, but do reflect reality
	   as of time of writing.  Please check your constants and do not
	   rely on the numbers in the above paragraph.
	*/
	addr = (unsigned long)_wr_config;
	for (i = 0; i < WRHV_RESERVED_PAGES; addr += PAGE_SIZE, i++)
		__wrhv_map_page(addr, addr, PAGE_KERNEL);

	/* We no longer need to use the vbconfig copy, map it straight in */
	wr_config = _wr_config;

	/* Setup the global variables used by the vbi */
	vbi_init(wr_config);
#else
	/* vbi_init() needs a non guest-wise per cpu pointer as its parameter.
	 * for x86-32, it is _wr_config(0xffff0000); Since 0xffff0000 is just
	 * at the top of kernel space in 32-bit linux, we can reserve them for
	 * use; However on 64-bit, 0xffff0000 is no longer in kernel space, so
	 * fixmap is only way to do so. But we can't use fixmaped _wr_config
	 * (0xffff0000) as parameter of vbi_init, because the pointers in
	 * _wr_config are still guest-physical address(0xffff0000-0xffffffff),
	 * not a 64-bit kernel space address, it will cause trouble in future.
	 * And we can't modify the points in _wr_config since _wr_config is
	 * set to read-only by hypervisor. So here use per cpu pointer
	 * corePrivate(writeable) in _wr_config to fake a hypervisor config,
	 * and change related pointers with fixmaped address then. it works!
	 */
	addr = (unsigned long)_wr_config;
	for (i = WRHV_RESERVED_PAGES - 1; i >= 0; addr += PAGE_SIZE, i--)
		set_fixmap((FIX_WRHV_RESERVED_BEGIN + i), addr);

	wr_config = (VB_CONFIG *)fix_to_virt(FIX_WRHV_RESERVED_END);

	construct_fake_wr_config();
	vbi_init(fake_wr_config);

#endif

	/* Now that critical hypervisor global regions are mapped in,
	   proceed with doing the actual interrupt initialization work.
	*/
	wrhv_pre_intr_init_hook();

	for (i = 0; i < (NR_VECTORS - FIRST_EXTERNAL_VECTOR); i++) {
		int vector = FIRST_EXTERNAL_VECTOR + i;
		__get_cpu_var(vector_irq)[vector] = i;

		if (i >= NR_IRQS)
			break;

		if (vector != WRS_SYSCALL_VECTOR)
			set_intr_gate(vector, interrupt[i]);
	}

#ifdef CONFIG_X86_32
	irq_ctx_init(smp_processor_id());
#endif

	wrhv_setup_timer_irq();
}

irqreturn_t wrhv_dummy_timer_interrupt(int irq, void *dev_id)
{
	int cpu = smp_processor_id();
	struct clock_event_device *evt = &per_cpu(wrhv_clock_events, cpu);
	if (!evt->event_handler) {
		printk(KERN_WARNING "wrhv timer handler \
				     has not set yet %d\n", cpu);
		return IRQ_NONE;
	}

	evt->event_handler(evt);

	return IRQ_HANDLED;
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

static void wrhv_init_timer(enum clock_event_mode mode,
				struct clock_event_device *evt)
{
}

static int wrhv_set_next_event(unsigned long delta,
			    struct clock_event_device *evt)
{
	return 0;
}

static void wrhv_timer_broadcast(const struct cpumask *mask)
{
	cpus_and(*mask, cpu_online_map, *mask);
	wrhv_send_IPI_mask(DUMMY_TIMER_INT, *mask);
}

struct clock_event_device wrhv_clock_event = {
	.name           = "wrhv",
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	.set_mode       = wrhv_init_timer,
	.set_next_event = wrhv_set_next_event,
	.broadcast      = wrhv_timer_broadcast,
	.max_delta_ns   = 0xffffffff,
	.min_delta_ns   = 10000,
	.shift          = 32,
	.mult           = 1,
	.irq            = 0,
	.rating         = 1,
};

void __devinit wrhv_setup_boot_clock(void)
{
	int ret;
	struct clock_event_device *evt;

	if (enable_hrtimer) {
		evt = &per_cpu(wrhv_clock_events, 0);
		memcpy(evt, &wrhv_clock_event, sizeof(*evt));
		evt->cpumask = cpumask_of(0);
		evt->features = CLOCK_EVT_FEAT_DUMMY | CLOCK_EVT_FEAT_ONESHOT;
		evt->irq = DUMMY_TIMER_INT;

		clockevents_register_device(evt);

		wrhv_timer_irq.name = "dummy_ipi_timer";
		wrhv_timer_irq.handler = wrhv_dummy_timer_interrupt;
		ret = setup_irq(DUMMY_TIMER_INT, &wrhv_timer_irq);
		if (ret)
			printk(KERN_EMERG "WRHV: setup dummy timer failed\n");
		else
			printk(KERN_INFO "WRHV: setup dummy timer done.\n");
	}
}

void __devinit wrhv_setup_secondary_clock(void)
{
	int cpu;
	struct clock_event_device *evt;
	cpu = smp_processor_id();
	printk(KERN_INFO "installing wrhv timer for CPU %d\n", cpu);

	evt = &per_cpu(wrhv_clock_events, cpu);
	memcpy(evt, &wrhv_clock_event, sizeof(*evt));
	evt->cpumask = cpumask_of(cpu);

	if (enable_hrtimer) {
		evt->features = CLOCK_EVT_FEAT_DUMMY | CLOCK_EVT_FEAT_ONESHOT;
		evt->irq = DUMMY_TIMER_INT;
	}
	clockevents_register_device(evt);
}

static void __init wrhv_time_init(void)
{
	struct clock_event_device *evt;

	evt = &per_cpu(wrhv_clock_events, 0);
	memcpy(evt, &wrhv_clock_event, sizeof(*evt));
	evt->cpumask = cpumask_of(0);

	clockevents_register_device(evt);
	setup_irq(0, &wrhv_timer_irq);
}

#ifdef CONFIG_PCI
static int wrhv_pci_enable_irq(struct pci_dev *dev)
{
	return 0;
}

static void wrhv_pci_disable_irq(struct pci_dev *dev)
{
}

int wrhv_pci_init(void)
{
	return 1;
}

void __init wrhv_init_pci(void)
{
	pcibios_enable_irq = wrhv_pci_enable_irq;
	pcibios_disable_irq = wrhv_pci_disable_irq;
	x86_init.pci.init = wrhv_pci_init;
}

static int __devinitdata __wrhv_kgdboe_poll;
static int __init wrhv_check_kgdboe(char *str)
{
	__wrhv_kgdboe_poll = 1;
	return 0;
}
early_param("kgdboe", wrhv_check_kgdboe);

int __devinit wrhv_has_kgdboe(void)
{
	return __wrhv_kgdboe_poll;
}

static char *shared_interrupts_list;
static int __init wrhv_check_shared_interrupts(char *str)
{
	shared_interrupts_list = str;
	return 0;
}
early_param("shared_interrupts", wrhv_check_shared_interrupts);

int find_shared_interrupt(char *devfn)
{
	char	shared_interrupts[WRHV_BOOTARG_BUF_SIZE];
	char	devfn_list[WRHV_BOOTARG_BUF_SIZE];
	char	*current_tok, *devfn_tok;
	char	*position, *devfn_pos;
	int	intr_num;

	if (shared_interrupts_list == NULL)
		return -1;

	strncpy(shared_interrupts, shared_interrupts_list,
		WRHV_BOOTARG_BUF_SIZE-1);
	shared_interrupts[WRHV_BOOTARG_BUF_SIZE-1] = '\0'; /* be safe */
	position = shared_interrupts;
	/* crack by ':' first */
	while ((current_tok = strsep(&position, ":"))) {
		/* crack by '@' second */
		if (sscanf(current_tok, "%x@%s", &intr_num, devfn_list) != 2)
			return -1; /* broken list */

		/* crack by ',' third */
		devfn_pos = devfn_list;
		while ((devfn_tok = strsep(&devfn_pos, ","))) {
			if (strcmp(devfn_tok, devfn) == 0) {
				/* We have a match! */
				return intr_num;
			}
		}
	}
	/* Didn't find it */
	return -1;
}

#define WRHV_PCI_FAKE_VENDOR_ID     0x1234
#define WRHV_PCI_FAKE_DEVICE_ID     0x5678

static void wrhv_pci_fixup_fake_device(struct pci_dev *dev)
{
	printk("WRHV: simulated pci device, ignore it.\n");
	dev->hdr_type = 0xff;
}

DECLARE_PCI_FIXUP_EARLY(WRHV_PCI_FAKE_VENDOR_ID, WRHV_PCI_FAKE_DEVICE_ID,
		wrhv_pci_fixup_fake_device);
#endif /* CONFIG_PCI */

static unsigned long long wrhv_read_msr(unsigned int msr, int *err)
{
#if WRHV_DEBUG_MSR
	printk("RDMSR from %p\n", __builtin_return_address(0));
#endif
	return native_read_msr(msr);
}

static int wrhv_write_msr(unsigned int msr, unsigned low, unsigned high)
{
#if WRHV_DEBUG_MSR
	printk("WRMSR from %p\n", __builtin_return_address(0));
#endif
	native_write_msr(msr, low, high);
	return 0;
}

void wrhv_cpu_workarounds(struct cpuinfo_x86 *c)
{
#ifdef CONFIG_X86_32
	/* Simics workaround */
	c->hlt_works_ok = 0;

	/* WP test fails currently */
	c->wp_works_ok = 1;

	clear_bit(X86_FEATURE_DE, (void *)boot_cpu_data.x86_capability);
	clear_bit(X86_FEATURE_XSAVE, (void *)boot_cpu_data.x86_capability);
#else
	setup_clear_cpu_cap(X86_FEATURE_RDTSCP);
#endif
}

char *__init wrhv_memory_setup(void)
{
	char *who = "WRHV-e820";
	e820_add_region(0, wr_config->phys_mem_size, E820_RAM);
	return who;

}

/* set device space and vb config as E820_RESERVED to avoid conflict with phy mem */
#define TYPE_IO 1
void __init wrhv_post_memory_setup(void)
{
	struct vb_dev_info *pdev;
	struct vb_dev_regset_info *preg;
	struct vb_config *pconfig;
	int i, j;
	unsigned long delta;
	pconfig = early_memremap((resource_size_t)_wr_config,
				WRHV_RESERVED_PAGES << PAGE_SHIFT);
	delta = (unsigned long)pconfig - (unsigned long)_wr_config;
	pdev = (struct vb_dev_info *)(delta +
			 (unsigned long)wr_config->deviceConfiguration);
	for (i = 0; i < wr_config->numDevices; i++, pdev++) {
		if (pdev->numRegSets > 0) {
			preg = (struct vb_dev_regset_info *)((char *)pdev +
							pdev->regSetInfoOffset);
			for(j=0; j < pdev->numRegSets; j++, preg++)
				if (preg->regSetType !=  TYPE_IO)
					e820_add_region(preg->regSetAddress,
					 preg->regSetLength, E820_RESERVED);
		}
	}
	early_iounmap(pconfig, WRHV_RESERVED_PAGES << PAGE_SHIFT);
	e820_add_region((phys_addr_t)_wr_config, 0x10000, E820_RESERVED);
	update_e820();

}

void __init wrhv_boot_config(void)
{
	boot_params.hdr.type_of_loader = 0xff; /* Unknown */
	if (__initrd_start != __initrd_end) {
#ifdef CONFIG_X86_32
		boot_params.hdr.ramdisk_image =
			(unsigned long)&__initrd_start - PAGE_OFFSET;
#else
		boot_params.hdr.ramdisk_image =
			(unsigned long)&__initrd_start - __START_KERNEL_map;
#endif
		boot_params.hdr.ramdisk_size =
		(unsigned long)&__initrd_end - (unsigned long)&__initrd_start;
	}

	x86_init.resources.memory_setup = wrhv_memory_setup;
	legacy_pic = &null_legacy_pic;

#ifndef WRHV_USE_XMLCONFIG
	strlcpy(boot_command_line,
		"pci=conf1 memmap=exactmap memmap=32M@0 mem=nopentium earlyprintk=vga,keep ramdisk_size=16384"
		" serialnumber nolapic nomce nosep retain_initrd root=/dev/ram init=/bin/busybox console=ttyS0,9600",
		COMMAND_LINE_SIZE);
#else
	/* Use the config space copy here, since we haven't mapped in the
	   actual hypervisor config/status/control space yet */
#ifdef CONFIG_X86_32
        snprintf(boot_command_line, COMMAND_LINE_SIZE,
		"retain_initrd pci=wrhv idle=wrhv serialnumber nolapic nomce "
		"noreplace-smp nosep %s",
		wr_config->bootLine);
#else
	snprintf(boot_command_line, COMMAND_LINE_SIZE,
		"retain_initrd pci=wrhv idle=wrhv serialnumber nolapic "
		"noxsave noreplace-smp nomce nosep nogbpages noexec=off "
		"novtlbopt %s",
		wr_config->bootLine);
#endif

#endif
	if (strstr(boot_command_line, "novtlbopt"))
		novtlbopt = 1;

	reserve_top_address(WRHV_RESERVED_TOP);

#ifdef CONFIG_WRHV_CERT
	cert_hyp_version = CERT_HYP_VER_STD;
	if (strstr(boot_command_line, "cert_debug"))
		cert_hyp_version = CERT_HYP_VER_DEBUG;
#endif
}

#ifdef CONFIG_SMP
irqreturn_t wrhv_ipi_func_call_single_handler(int irq, void *dev_id)
{
	irq_enter();
	generic_smp_call_function_single_interrupt();
	inc_irq_stat(irq_call_count);
	irq_exit();
	return IRQ_HANDLED;
}

irqreturn_t wrhv_ipi_func_call_handler(int irq, void *dev_id)
{
	irq_enter();
	generic_smp_call_function_interrupt();
	inc_irq_stat(irq_call_count);
	irq_exit();
	return IRQ_HANDLED;
}

irqreturn_t wrhv_ipi_inv_tlb_handler(int irq, void *dev_id)
{
	unsigned long cpu;

	cpu = get_cpu();

	if (!cpu_isset(cpu, flush_cpumask))
		goto out;
		/*
		 * This was a BUG() but until someone can quote me the
		 * line from the intel manual that guarantees an IPI to
		 * multiple CPUs is retried _only_ on the erroring CPUs
		 * its staying as a return
		 *
		 * BUG();
		 */

	if (flush_mm == per_cpu(cpu_tlbstate, cpu).active_mm) {
		if (per_cpu(cpu_tlbstate, cpu).state == TLBSTATE_OK) {
			if (flush_va == TLB_FLUSH_ALL)
				local_flush_tlb();
			else
				__flush_tlb_one(flush_va);
		} else {
			leave_mm(cpu);
		}
	} else {
	    if (flush_va == TLB_FLUSH_ALL) {
			wrhv_vtlb_op(VBI_VTLB_OP_DELETE_PMD,
					__pa(flush_mm->pgd),
					0, 0);
			cpu_clear(cpu, flush_mm->cpu_vm_mask);
	    } else
			wrhv_vtlb_op(VBI_VTLB_OP_UPDATE_PTE,
					__pa(flush_mm->pgd),
					(unsigned long)flush_va, 0);

	}

	smp_mb__before_clear_bit();
	cpu_clear(cpu, flush_cpumask);
	smp_mb__after_clear_bit();
out:
	put_cpu();
	inc_irq_stat(irq_tlb_count);

	return IRQ_HANDLED;
}

irqreturn_t wrhv_ipi_resched_handler(int irq, void *dev_id)
{
	inc_irq_stat(irq_resched_count);
	return IRQ_HANDLED;
}
#endif

void wrhv_vtlb_op(unsigned int op, unsigned long arg1,
		  unsigned long arg2, unsigned long arg3)
{
	unsigned long flags;
	int i;

	if (!VTLB_GET_CPU_VAR(is_vtlb_ops_cache_enabled))
		vbi_vtlb_op(op, arg1, arg2, arg3);
	else {
		local_irq_save(flags);
		i = VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops_ix;
		VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops[i].op = op;
		VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops[i].arg1 = arg1;
		VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops[i].arg2 = arg2;
		VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops[i].arg3 = arg3;
		wmb();
		/*
		 * If the buffer is full, flush it. Index will be automatically
		 * updated by the hypervisor.
		 */

		if (VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops_ix == (VBI_VTLB_OP_MAX_OPS - 1))
			vbi_vtlb_op(VBI_VTLB_OP_FLUSH_OPS, 0, 0, 0);
		else
			VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops_ix += 1;

		local_irq_restore(flags);
	}
}

static void wrhv_write_cr3(unsigned long val)
{
	unsigned long cr3 = val;
	int i;

	if (VTLB_GET_CPU_VAR(is_cr3_cache_enabled) && VTLB_GET_CPU_VAR(vtlb_ctrl).vtlb_ops_ix == 0) {
		for (i = 0; i < VBI_VTLB_OP_CR3_CACHE_ENTRIES; i++) {
			if (VTLB_GET_CPU_VAR(vtlb_ctrl).cr3_cache[i].guest_cr3 == cr3) {
				cr3 = VTLB_GET_CPU_VAR(vtlb_ctrl).cr3_cache[i].host_cr3;
				VTLB_GET_CPU_VAR(vtlb_ctrl).cr3_cache_ix = i;
				break;
			}
		}
	} else
		VTLB_GET_CPU_VAR(vtlb_ctrl).cr3_cache_ix = -1;

	asm volatile ("mov %0,%%cr3": :"r" (cr3));
	VTLB_GET_CPU_VAR(cr3_val) = val;
}

static unsigned long wrhv_read_cr3(void)
{
	/* Use cached value to avoid useless hypercall */
	return VTLB_GET_CPU_VAR(cr3_val);
}

static void wrhv_set_pmd(pmd_t *pmdp, pmd_t pmdval)
{
	*pmdp = pmdval;
	wrhv_vtlb_op(VBI_VTLB_OP_UPDATE_PMD,
			__pa(((unsigned long) pmdp) & PAGE_MASK),
			__pa(pmdp), 0);
}

#define is_current_as(mm) ((mm) == current->active_mm || ((mm) == &init_mm))

static void wrhv_pte_update(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
	if (!is_current_as(mm))
		wrhv_vtlb_op(VBI_VTLB_OP_UPDATE_PTE, __pa(mm->pgd),
						addr, __pa(ptep));
}

static void wrhv_pte_update_defer(struct mm_struct *mm, unsigned long addr, pte_t *ptep)
{
		wrhv_pte_update (mm, addr, ptep);
}

static void wrhv_release_pd(unsigned long pfn)
{
	wrhv_vtlb_op(VBI_VTLB_OP_DELETE_PMD, pfn << PAGE_SHIFT, 0, 0);
}

static void wrhv_set_pte(pte_t *ptep, pte_t pte)
{
	*ptep = pte;
}

static void wrhv_flush_tlb_user(void)
{
	native_write_cr3(VTLB_GET_CPU_VAR(cr3_val));
}

static void wrhv_flush_tlb_single(unsigned long addr)
{
	__native_flush_tlb_single(addr);
}


static inline void wrhv_send_IPI_mask(int irq, cpumask_t mask)
{
	unsigned long coreset = cpus_addr(mask)[0];
	unsigned long flags;

	local_irq_save(flags);
	WARN_ON(coreset & ~cpus_addr(cpu_online_map)[0]);
	vbi_send_vcore_vioapic_irq(irq, coreset, 0);
	local_irq_restore(flags);
}

#ifdef CONFIG_SMP
static void wrhv_smp_send_invalidate_tlb_ipi(cpumask_t mask)
{
	wrhv_send_IPI_mask(WRHV_IPI_INV_TLB, mask);
}

static void wrhv_flush_tlb_others(const cpumask_t *cpumaskp, struct mm_struct *mm,
			     unsigned long va)
{
	cpumask_t cpumask = *cpumaskp;

	/*
	 * i'm not happy about this global shared spinlock in the
	 * MM hot path, but we'll see how contended it is.
	 * AK: x86-64 has a faster method that could be ported.
	 */
	raw_spin_lock(&tlbstate_lock);

	flush_mm = mm;
	flush_va = va;
	cpus_and(cpumask, cpu_online_map, cpumask);
	cpu_clear(smp_processor_id(), cpumask);
	cpus_or(flush_cpumask, cpumask, flush_cpumask);
	/*
	 * We have to send the IPI only to
	 * CPUs affected.
	 */
	wrhv_smp_send_invalidate_tlb_ipi(cpumask);

	while (!cpus_empty(flush_cpumask))
		/* nothing. lockup detection does not belong here */
		cpu_relax();
	flush_mm = NULL;
	flush_va = 0;
	raw_spin_unlock(&tlbstate_lock);
}
#endif

static void wrhv_set_pte_at(struct mm_struct *mm, unsigned long addr, pte_t *ptep, pte_t pte)
{
	if (!is_current_as(mm)) {
		*ptep = pte;
		wrhv_vtlb_op(VBI_VTLB_OP_SET_PTE_AT, __pa(mm->pgd),
						addr, __pa(ptep));
	} else
		*ptep = pte;
}

static unsigned wrhv_patch(u8 type, u16 clobbers, void *ibuf,
				unsigned long addr, unsigned len)
{
	switch (type) {
	case PARAVIRT_PATCH(pv_mmu_ops.read_cr3):
	case PARAVIRT_PATCH(pv_mmu_ops.write_cr3):
		return paravirt_patch_default(type, clobbers, ibuf, addr, len);
	default:
		return native_patch(type, clobbers, ibuf, addr, len);
	}

}

static void wrhv_pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
#ifdef CONFIG_SMP
	/*
	 * We are deleting the page directory. We need to delete it in
	 * the current hypervisor cache but also in the cache of the
	 * hypervisors managing the various virtual cores.
	 */

	cpumask_t cpumask;
	int cpu = get_cpu();
	wrhv_vtlb_op(VBI_VTLB_OP_DELETE_PMD, __pa(pgd), 0, 0);
	cpumask = mm->cpu_vm_mask;
	cpu_clear(cpu, cpumask);
	if (!cpus_empty(cpumask))
	    wrhv_flush_tlb_others (&cpumask, mm, TLB_FLUSH_ALL);
	put_cpu();
#else
	wrhv_vtlb_op(VBI_VTLB_OP_DELETE_PMD, __pa(pgd), 0, 0);
#endif
}

static void __cpuinit wrhv_init_vtlb_per_cpu(void)
{
	/* Initialize the cached copy of cr3 */
	VTLB_GET_CPU_VAR(cr3_val) = native_read_cr3();

	/*
	 * set the size of the vtlb_ctrl structure in the structure provided
	 * to the hypervisor; the hypervisor may be able to use this later
	 * for backward compatibility.
	 */

	VTLB_GET_CPU_VAR(vtlb_ctrl).size = sizeof (VTLB_GET_CPU_VAR(vtlb_ctrl));

	/*
	 * First set the options supported by the guest OS. The host will
	 * then update the mode field of vtlb_ctrl option to indicate which
	 * one will actually be in use.
	 */

	if (novtlbopt) {
		VTLB_GET_CPU_VAR(vtlb_ctrl).mode = VBI_VTLB_OPTIM_OPTION_NOOPT;
		printk("WRHV:  CPU %d vtlb optimization disabled\n",
			smp_processor_id());
	} else
		VTLB_GET_CPU_VAR(vtlb_ctrl).mode = VBI_VTLB_OPTIM_OPTION;

	vbi_vtlb_op(VBI_VTLB_OP_INIT, __pa_symbol(&VTLB_GET_CPU_VAR(vtlb_ctrl)),
		 0, 0);

	if (VTLB_GET_CPU_VAR(vtlb_ctrl).mode & VBI_VTLB_CR3_CACHE_ENABLED)
		VTLB_GET_CPU_VAR(is_cr3_cache_enabled) = 1;
	if (VTLB_GET_CPU_VAR(vtlb_ctrl).mode & VBI_VTLB_OPTIM_ENABLED)
		VTLB_GET_CPU_VAR(is_vtlb_optim_enabled) = 1;
	if (VTLB_GET_CPU_VAR(vtlb_ctrl).mode & VBI_VTLB_OPS_CACHE_ENABLED)
		VTLB_GET_CPU_VAR(is_vtlb_ops_cache_enabled) = 1;

}

static void wrhv_init_mm(void)
{
	wrhv_init_vtlb_per_cpu();

	pv_mmu_ops.read_cr3 = wrhv_read_cr3;
	pv_mmu_ops.write_cr3 = wrhv_write_cr3;

	if (VTLB_GET_CPU_VAR(is_vtlb_optim_enabled)) {
		pv_mmu_ops.set_pte = wrhv_set_pte;
		pv_mmu_ops.set_pte_at = wrhv_set_pte_at;

		pv_mmu_ops.pte_update = wrhv_pte_update;
		pv_mmu_ops.pte_update_defer = wrhv_pte_update_defer;
		pv_mmu_ops.set_pmd = wrhv_set_pmd;

		pv_mmu_ops.release_pmd = wrhv_release_pd;

		pv_mmu_ops.pgd_free = wrhv_pgd_free;

		pv_mmu_ops.flush_tlb_user = wrhv_flush_tlb_user;
		pv_mmu_ops.flush_tlb_single = wrhv_flush_tlb_single;
	}
#ifdef CONFIG_SMP
	pv_mmu_ops.flush_tlb_others = wrhv_flush_tlb_others;
#endif
}

static inline void wrhv_umask_timer_for_vcore(int irq)
{
	/*
	 * unmask hypervisor-provided timer interrupt for vcore
	 */
	__get_cpu_var(vector_irq)[FIRST_EXTERNAL_VECTOR + irq] = irq;
	vbi_unmask_vioapic_irq(irq);
}

static inline void wrhv_mask_timer_for_vcore(int irq)
{
	/*
	 * mask hypervisor-provided timer interrupt for vcore
	 */
	vbi_mask_vioapic_irq(irq);
}

#ifdef CONFIG_SMP
static inline void wrhv_ap_vectors(int cpu, int irq)
{
	per_cpu(vector_irq, cpu)[FIRST_EXTERNAL_VECTOR + irq] = irq;
}

static void __init wrhv_smp_setup_irq_vectors(int cpu)
{
	wrhv_ap_vectors(cpu, WRHV_IPI_RESCHED);
	wrhv_ap_vectors(cpu, WRHV_IPI_INV_TLB);
	wrhv_ap_vectors(cpu, WRHV_IPI_FUNC_CALL);
	wrhv_ap_vectors(cpu, WRHV_IPI_FUNC_CALL_SINGLE);
}

static void __init wrhv_smp_prepare_boot_cpu(void)
{
	BUG_ON(smp_processor_id() != 0);
	native_smp_prepare_boot_cpu();
	return;
}

void __init wrhv_calibrate_smp_cpus(void)
{
	/*
	 * Use the config space copy here, since we haven't mapped in
	 * the actual hypervisor config/status/control space yet
	 */
	int cpus = wr_config->numCores;
	int cpuid = wr_config->coreId;

	if (cpuid)
		return;

	printk(KERN_INFO "WRHV: calibrate CPU/Cores info according to vbConfig \n");
	physids_clear(phys_cpu_present_map);

	if (cpus > 1) {
		smp_found_config = 1;
		alternatives_smp_switch(1);
	}

	while (--cpus >= 0) {
		physid_set(cpus, phys_cpu_present_map);
		set_cpu_present(cpus, true);
		set_cpu_possible(cpus, true);
	}
}

static void inline wrhv_mask_IPIs_for_vcore(void)
{
	/* mask ipi interrupt for vcore */
	vbi_mask_vioapic_irq(WRHV_IPI_RESCHED);
	vbi_mask_vioapic_irq(WRHV_IPI_INV_TLB);
	vbi_mask_vioapic_irq(WRHV_IPI_FUNC_CALL);
	vbi_mask_vioapic_irq(WRHV_IPI_FUNC_CALL_SINGLE);
	if (enable_hrtimer)
		vbi_mask_vioapic_irq(DUMMY_TIMER_INT);
}

static void inline wrhv_umask_IPIs_for_vcore(void)
{
	/* unmask ipi interrupt for vcore */
	vbi_unmask_vioapic_irq(WRHV_IPI_RESCHED);
	vbi_unmask_vioapic_irq(WRHV_IPI_INV_TLB);
	vbi_unmask_vioapic_irq(WRHV_IPI_FUNC_CALL);
	vbi_unmask_vioapic_irq(WRHV_IPI_FUNC_CALL_SINGLE);
	if (enable_hrtimer)
		vbi_unmask_vioapic_irq(DUMMY_TIMER_INT);
}

void __init wrhv_smp_setup_ipi(void)
{
	int ret;

	set_irq_chip_and_handler_name(WRHV_IPI_RESCHED,
			&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");
	set_irq_chip_and_handler_name(WRHV_IPI_INV_TLB,
			&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");
	set_irq_chip_and_handler_name(WRHV_IPI_FUNC_CALL,
			&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");
	set_irq_chip_and_handler_name(WRHV_IPI_FUNC_CALL_SINGLE,
			&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");

	if (enable_hrtimer)
		set_irq_chip_and_handler_name(DUMMY_TIMER_INT,
			&wrhv_ipi_irq_chip, handle_percpu_irq, "per_cpu");

	ret = request_irq(WRHV_IPI_RESCHED, wrhv_ipi_resched_handler,
			IRQF_DISABLED|IRQF_NOBALANCING, "ipi_resched",
			wrhv_ipi_resched_handler);
	printk("request_irq ret for WRHV_IPI_RESCHED: %d \n", ret);

	ret = request_irq(WRHV_IPI_INV_TLB, wrhv_ipi_inv_tlb_handler,
			IRQF_DISABLED|IRQF_NOBALANCING, "ipi_inv_tlb",
			wrhv_ipi_inv_tlb_handler);
	printk("request_irq ret for WRHV_IPI_INV_TLB: %d \n", ret);

	ret = request_irq(WRHV_IPI_FUNC_CALL, wrhv_ipi_func_call_handler,
			IRQF_DISABLED|IRQF_NOBALANCING, "ipi_func_call",
			wrhv_ipi_func_call_handler);
	printk("request_irq ret for WRHV_IPI_FUNC_CALL: %d \n", ret);

	ret = request_irq(WRHV_IPI_FUNC_CALL_SINGLE,
			wrhv_ipi_func_call_single_handler,
			IRQF_DISABLED|IRQF_NOBALANCING, "ipi_func_call_single",
			wrhv_ipi_func_call_single_handler);
	printk("request_irq ret for WRHV_IPI_FUNC_CALL_SINGLE: %d \n", ret);

	wrhv_umask_IPIs_for_vcore();
}

void __init wrhv_get_ipis(void)
{
	int irq;
	irq = vbi_find_irq(WRHV_IPI_NAME_RESCHED, 1);
	if (irq != VBI_INVALID_IRQ)
		WRHV_IPI_RESCHED = irq;
	else
		printk(KERN_CRIT "Resched IPI not found, using default %d\n",
			   WRHV_IPI_RESCHED);

	irq = vbi_find_irq(WRHV_IPI_NAME_INV_TLB, 1);
	if (irq != VBI_INVALID_IRQ)
		WRHV_IPI_INV_TLB = irq;
	else
		printk(KERN_CRIT "Invalidate_TLB IPI not found, using default %d\n",
			   WRHV_IPI_INV_TLB);

	irq = vbi_find_irq(WRHV_IPI_NAME_FUNC_CALL, 1);
	if (irq != VBI_INVALID_IRQ)
		WRHV_IPI_FUNC_CALL = irq;
	else
		printk(KERN_CRIT "Call_function IPI not found, using default %d\n",
			   WRHV_IPI_FUNC_CALL);

	irq = vbi_find_irq(WRHV_IPI_NAME_FUNC_CALL_SINGLE, 1);
	if (irq != VBI_INVALID_IRQ)
		WRHV_IPI_FUNC_CALL_SINGLE = irq;
	else
		printk(KERN_CRIT "Call_function_single IPI not found, using default %d\n",
			   WRHV_IPI_FUNC_CALL_SINGLE);

	return;
}

void __init wrhv_smp_prepare_cpus(unsigned int max_cpus)
{
	native_smp_prepare_cpus(max_cpus);

	wrhv_get_ipis();
	wrhv_smp_setup_ipi();
	return;
}

#ifdef CONFIG_HOTPLUG_CPU
static DEFINE_PER_CPU(struct task_struct *, idle_thread_array);
#define get_idle_for_cpu(x)      (per_cpu(idle_thread_array, x))
#define set_idle_for_cpu(x, p)   (per_cpu(idle_thread_array, x) = (p))
#else
struct task_struct *idle_thread_array[NR_CPUS] __cpuinitdata ;
#define get_idle_for_cpu(x)      (idle_thread_array[(x)])
#define set_idle_for_cpu(x, p)   (idle_thread_array[(x)] = (p))
#endif

struct create_idle {
	struct work_struct work;
	struct task_struct *idle;
	struct completion done;
	int cpu;
};

static void __cpuinit do_fork_idle(struct work_struct *work)
{
	struct create_idle *c_idle =
		container_of(work, struct create_idle, work);

	c_idle->idle = fork_idle(c_idle->cpu);
	complete(&c_idle->done);
}

static void __cpuinit wrhv_smp_callin(void)
{
	int cpuid;
	unsigned long timeout;
	int i;

	cpuid = smp_processor_id();
	if (cpumask_test_cpu(cpuid, cpu_callin_mask)) {
		panic("%s: CPU#%d already present??\n", __func__, cpuid);
	}

	/*
	 * Waiting 2s total for startup (udelay is not yet working)
	 */
	timeout = jiffies + 2*HZ;
	while (time_before(jiffies, timeout)) {
		/*
		 * Has the boot CPU finished it's STARTUP sequence?
		 */
		if (cpumask_test_cpu(cpuid, cpu_callout_mask))
			break;
		cpu_relax();
	}

	if (!time_before(jiffies, timeout)) {
		panic("%s: CPU%d started up but did not get a callout!\n",
			__func__, cpuid);
	}

	/*
	 * Need to setup vector mappings before we enable interrupts.
	 */
	for (i = FIRST_EXTERNAL_VECTOR; i < NR_VECTORS; i++)
		__get_cpu_var(vector_irq)[i] = (i-FIRST_EXTERNAL_VECTOR);

	local_irq_enable();
	calibrate_delay();
	local_irq_disable();

	smp_store_cpu_info(cpuid);

	/*
	 * Allow the master to continue.
	 */
	cpumask_set_cpu(cpuid, cpu_callin_mask);
}

#ifdef CONFIG_X86_32
static int low_mappings;
#endif

static void __cpuinit wrhv_smp_start_cpu(void)
{
#ifdef CONFIG_X86_64
	construct_fake_wr_config();
#endif
	cpu_init();

	wrhv_mask_timer_for_vcore(WRHV_VTIMER_INT);

	wrhv_init_vtlb_per_cpu();

	wrhv_umask_IPIs_for_vcore();
	preempt_disable();
	wrhv_smp_callin();

	/* otherwise gcc will move up smp_processor_id before the cpu_init */
	barrier();

	/*
	 * Check TSC synchronization with the BP:
	 */
	check_tsc_sync_target();

#ifdef CONFIG_X86_32
	while (low_mappings) {
		cpu_relax();
	}
	__flush_tlb_all();
#endif

	set_cpu_sibling_map(raw_smp_processor_id());
	wmb();

	ipi_call_lock_irq();
	spin_lock(&vioapic_lock);
	wrhv_smp_setup_irq_vectors(smp_processor_id());
	set_cpu_online(smp_processor_id(), true);
	spin_unlock(&vioapic_lock);
	ipi_call_unlock_irq();
	per_cpu(cpu_state, smp_processor_id()) = CPU_ONLINE;

	x86_cpuinit.setup_percpu_clockev();

	if (!enable_hrtimer)
		wrhv_umask_timer_for_vcore(WRHV_VTIMER_INT);
	else
		wrhv_umask_timer_for_vcore(DUMMY_TIMER_INT);

	wmb();

	local_irq_enable();
	cpu_idle();
}

static int __cpuinit wrhv_wakeup_secondary_cpu(int core)
{
	return vbi_vb_resume(VBI_BOARD_ID_GET(), core);
}

static int __cpuinit wrhv_do_boot_cpu(int cpu)
{
	unsigned long boot_error = 0;
	unsigned int timeout;
	unsigned long start_ip;
	struct create_idle c_idle = {
		.cpu = cpu,
		.done = COMPLETION_INITIALIZER_ONSTACK(c_idle.done),
	};

	INIT_WORK_ON_STACK(&c_idle.work, do_fork_idle);

	alternatives_smp_switch(1);

	c_idle.idle = get_idle_for_cpu(cpu);

	if (c_idle.idle) {
		c_idle.idle->thread.sp = (unsigned long) (((struct pt_regs *)
			(THREAD_SIZE +  task_stack_page(c_idle.idle))) - 1);
		init_idle(c_idle.idle, cpu);
		goto do_rest;
	}

	if (!keventd_up() || current_is_keventd())
		c_idle.work.func(&c_idle.work);
	else {
		schedule_work(&c_idle.work);
		wait_for_completion(&c_idle.done);
	}

	if (IS_ERR(c_idle.idle)) {
		printk("failed fork for CPU %d\n", cpu);
		return PTR_ERR(c_idle.idle);
	}

	set_idle_for_cpu(cpu, c_idle.idle);

do_rest:

	per_cpu(current_task, cpu) = c_idle.idle;
#ifdef CONFIG_X86_32
	irq_ctx_init(cpu);
#else
	clear_tsk_thread_flag(c_idle.idle, TIF_FORK);
	initial_gs = per_cpu_offset(cpu);
	per_cpu(kernel_stack, cpu) =
		(unsigned long)task_stack_page(c_idle.idle) -
		KERNEL_STACK_OFFSET + THREAD_SIZE;
#endif

	early_gdt_descr.address = (unsigned long)get_cpu_gdt_table(cpu);
	initial_code = (unsigned long)wrhv_smp_start_cpu;
	stack_start.sp = (void *) c_idle.idle->thread.sp;

	/* start_ip had better be page-aligned! */
	start_ip = setup_trampoline();

	printk(KERN_INFO "Booting processor %d\n", cpu);

	boot_error = wrhv_wakeup_secondary_cpu(cpu);

	if (!boot_error) {
		/*
		 * allow APs to start initializing.
		 */
		pr_debug("Before Callout %d.\n", cpu);
		cpumask_set_cpu(cpu, cpu_callout_mask);
		pr_debug("After Callout %d.\n", cpu);

		/*
		 * Wait 5s total for a response
		 */
		for (timeout = 0; timeout < 50000; timeout++) {
			if (cpumask_test_cpu(cpu, cpu_callin_mask))
				break;	/* It has booted */
			udelay(100);
		}

		if (cpumask_test_cpu(cpu, cpu_callin_mask)) {
			/* number CPUs logically, starting from 1 (BSP is 0) */
			pr_debug("OK.\n");
			printk(KERN_INFO "CPU%d: ", cpu);
			print_cpu_info(&cpu_data(cpu));
			pr_debug("CPU has booted.\n");
		} else {
			boot_error = 1;
			printk(KERN_ERR "Not responding.\n");
		}
	}

	if (boot_error) {
		/* Try to put things back the way they were before ... */
		cpumask_clear_cpu(cpu, cpu_callout_mask);
		cpumask_clear_cpu(cpu, cpu_initialized_mask);
		set_cpu_present(cpu, false);
	}

	return boot_error;
}

static int __cpuinit wrhv_cpu_up(unsigned int cpu)
{
	int err;
	unsigned long flags;

	preempt_disable();
	if (!physid_isset(cpu, phys_cpu_present_map)) {
		printk(KERN_ERR "%s: bad cpu %d\n", __func__, cpu);
		printk(KERN_ERR "phys_cpu_present_map: %x \n",
				*(unsigned *)&phys_cpu_present_map);
		return -EINVAL;
	}

	/*
	 * Already booted CPU?
	 */
	if (cpumask_test_cpu(cpu, cpu_callin_mask)) {
		panic("wrhv_do_boot_cpu core%d Already started\n", cpu);
		return -ENOSYS;
	}

	per_cpu(cpu_state, cpu) = CPU_UP_PREPARE;

#ifdef CONFIG_X86_32
	/* init low mem mapping */
	clone_pgd_range(swapper_pg_dir, swapper_pg_dir + KERNEL_PGD_BOUNDARY,
		min_t(unsigned long, KERNEL_PGD_PTRS, KERNEL_PGD_BOUNDARY));
	flush_tlb_all();

	low_mappings = 1;
	err = wrhv_do_boot_cpu(cpu);
	zap_low_mappings(false);
	low_mappings = 0;
#else
	err = wrhv_do_boot_cpu(cpu);
#endif
	if (err) {
		pr_debug("wrhv_do_boot_cpu failed %d\n", err);
		return -EIO;
	}

	local_irq_save(flags);
	check_tsc_sync_source(cpu);
	local_irq_restore(flags);

	preempt_enable();
	while (!cpu_online(cpu))
		cpu_relax();

	return 0;
}

static void __init wrhv_smp_cpus_done(unsigned int max_cpus)
{
	printk(KERN_INFO "BP: smp_init done.\n");
	native_smp_cpus_done(max_cpus);
	return;
}

static void wrhv_stop_me(void * t)
{
	printk(KERN_INFO "wrhv_stop_me.\n");
	write_cr3(__pa(swapper_pg_dir));

	vbi_vb_reset(VBI_BOARD_ID_GET(), smp_processor_id(), 0);

	/*
	 * Verbose report that reset self core failed then
	 * into infinite loop
	 */
	BUG();
	while (1);
}

static void wrhv_smp_send_stop(int wait)
{
	smp_call_function(wrhv_stop_me, NULL, 0);
	return;
}

static void wrhv_smp_send_reschedule(int cpu)
{
	wrhv_send_IPI_mask(WRHV_IPI_RESCHED, *cpumask_of(cpu));
}

static void wrhv_smp_send_call_func_single_ipi(int cpu)
{
	wrhv_send_IPI_mask(WRHV_IPI_FUNC_CALL_SINGLE, *cpumask_of(cpu));
}

static void wrhv_smp_send_call_func_ipi(const cpumask_t *mask)
{
	wrhv_send_IPI_mask(WRHV_IPI_FUNC_CALL, *mask);
}

#ifdef CONFIG_HOTPLUG_CPU
static int wrhv_cpu_disable(void)
{
	unsigned int cpu = smp_processor_id();

	printk(KERN_INFO "In wrhv_cpu_disable. cpu = %d\n", cpu);

	if (cpu == 0)
		return -EBUSY;

	wrhv_fixup_irqs(cpu);
	cpu_disable_common();

	return 0;
}

static void wrhv_cpu_play_dead(void)
{
	int ret;

	printk(KERN_INFO "wrhv_cpu_play_dead.\n");

	play_dead_common();

	load_cr3(swapper_pg_dir);

	wrhv_mask_IPIs_for_vcore();
	cpu_clear(smp_processor_id(), flush_cpumask);

	/*
	 * The below vbi call will actually reset the *self* core
	 * so this vbi call is expected to not return as a normal
	 * funtion call hence the rest codes after this vbi call
	 * won't being executed in normal case
	 */
	ret = vbi_vb_reset(VBI_BOARD_ID_GET(), smp_processor_id(), 0);

	printk(KERN_ERR "wrhv_cpu_play_dead: reset self failed."
			"ret = %d\n", ret);
	/*
	 * Normally we won't reach here, so lives a BUG()
	 * here to produce verbose output to indicate that
	 * something wrong happened
	 */
	BUG();
	while (1);
}
#endif

static struct smp_ops wrhv_smp_ops __initdata = {
	.smp_prepare_boot_cpu = wrhv_smp_prepare_boot_cpu,
	.smp_prepare_cpus = wrhv_smp_prepare_cpus,

	.cpu_up = wrhv_cpu_up,
	.cpu_die = native_cpu_die,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable = wrhv_cpu_disable,
	.play_dead = wrhv_cpu_play_dead,
#endif

	.smp_cpus_done = wrhv_smp_cpus_done,

	.stop_other_cpus = wrhv_smp_send_stop,
	.smp_send_reschedule = wrhv_smp_send_reschedule,

	.send_call_func_ipi = wrhv_smp_send_call_func_ipi,
	.send_call_func_single_ipi = wrhv_smp_send_call_func_single_ipi,
};

void __init wrhv_smp_init(void)
{
	smp_ops = wrhv_smp_ops;
	return;
}
#endif /* CONFIG_SMP */

void wrhv_restart(void)
{
	printk(KERN_INFO "WRHV: rebooting \n");

	vbi_vb_reset(VBI_BOARD_ID_GET(), smp_processor_id(),
		VBI_VBMGMT_RESET_AND_START_CORE0 |
		VBI_VBMGMT_RESET_DOWNLOAD
		);

	/*
	 * Normally we won't reach here so verbose report
	 * the failure of reset self core then loop forever
	 */
	BUG();
	while (1);
}

void __init wrhv_setup_timer_irq(void)
{
#ifdef CONFIG_WRHV_X86_HRTIMERS
	int irq;
	irq = vbi_find_irq(HRTIMER_IRQ_NAME, 1);
	if (irq == VBI_INVALID_IRQ) {
		enable_hrtimer = 0;
		x86_init.timers.timer_init = wrhv_time_init;
	} else {
		enable_hrtimer = 1;
		set_irq_chip_and_handler_name(TIMER_INT_NUM, &wrhv_irq_chip,
					      handle_edge_irq, "edge");
		wrhv_mask_timer_for_vcore(WRHV_VTIMER_INT);
	}
#else
	enable_hrtimer = 0;
	x86_init.timers.timer_init = wrhv_time_init;
#endif

	printk(KERN_INFO "WRHV: HRTIMER is%s present.\n",
			enable_hrtimer? "" : " NOT");

	x86_platform.calibrate_tsc = wrhv_calculate_cpu_khz;

#ifdef CONFIG_X86_LOCAL_APIC
	x86_init.timers.setup_percpu_clockev = wrhv_setup_boot_clock;
	x86_cpuinit.setup_percpu_clockev = wrhv_setup_secondary_clock;
#endif
}

void __init wrhv_init(void)
{
	pv_info.name = "wrhv";
	pv_info.paravirt_enabled = 1;

	wrhv_cpu_workarounds(&boot_cpu_data);

	pv_init_ops.patch = wrhv_patch;

	pv_cpu_ops.write_msr = wrhv_write_msr;
	pv_cpu_ops.read_msr = wrhv_read_msr;

	x86_init.irqs.intr_init = wrhv_init_IRQ;

	machine_ops.emergency_restart = wrhv_restart;

#ifdef CONFIG_KGDB
	arch_kgdb_ops.flags &= ~KGDB_HW_BREAKPOINT,
	arch_kgdb_ops.set_hw_breakpoint = NULL;
	arch_kgdb_ops.remove_hw_breakpoint = NULL;
	arch_kgdb_ops.remove_all_hw_break = NULL;
	arch_kgdb_ops.correct_hw_break = NULL;
#endif

	wrhv_init_mm();

#ifdef CONFIG_SMP
	wrhv_smp_init();
#endif
	wrhv_post_memory_setup();
}
