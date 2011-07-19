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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/profile.h>
#include <linux/kernel_stat.h>
#include <linux/wrhv.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <vbi/vbi.h>
#include <linux/pci.h>
#include <linux/suspend.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>
#include <vbi/private.h>
#include <linux/reboot.h>
#include <vbi/dynamic.h>

#include "procfs.h"

#define VIOAPIC_BASE_ADDR	(&wr_vb_control->vIoapic)

#ifdef CONFIG_SMP
enum wrhv_irq_action {
	WRHV_IRQ_SET_AFFINITY = 1,
	WRHV_IRQ_SHUTDOWN,
	WRHV_IRQ_STARTUP
};

struct wrhv_irq_struct {
	enum wrhv_irq_action action;
	int irq;
	int ocpu, ncpu;
	struct wrhv_irq_struct *next;
};

struct wrhv_irq_head {
	struct wrhv_irq_struct *head;
	struct wrhv_irq_struct **next;
};

static struct wrhv_irq_head wrhv_irq_head = {
	.head	= NULL,
	.next	= &(wrhv_irq_head.head),
};

static DEFINE_SPINLOCK(wrhv_irq_lock);
static void wrhv_irq_task(unsigned long arg);
static DECLARE_TASKLET(wrhv_irq_tasklet, wrhv_irq_task, 0);
#endif

static void wrhv_enable_irq(unsigned int irq)
{
	vbi_unmask_vioapic_irq(irq);
}

static void wrhv_disable_irq(unsigned int irq)
{
	vbi_mask_vioapic_irq(irq);
}

static void wrhv_ack_irq(unsigned int irq)
{
	vbi_ack_vioapic_irq(irq);
}

__weak int wrhv_dir_irq = 0;
#ifndef CONFIG_WRHV_COREVBI_ONLY
__weak asmlinkage void vbi_di_eoi(void)
{
}
#endif

static void wrhv_eoi_irq(unsigned int irq)
{
	if (wrhv_dir_irq)
		vbi_di_eoi();
	else
		vbi_ack_vioapic_irq(irq);
}

static void wrhv_maskack_irq(unsigned int irq)
{
	if (irq != 0)
		vbi_mask_vioapic_irq(irq);
	vbi_ack_vioapic_irq(irq);
}

static void wrhv_mask_irq(unsigned int irq)
{
	if (irq != 0)
		vbi_mask_vioapic_irq(irq);
}

static void wrhv_unmask_irq(unsigned int irq)
{
	if (irq != 0)
		vbi_unmask_vioapic_irq(irq);
}

#ifdef CONFIG_SMP
static void wrhv_mask_irq_ipi(void *p)
{
	unsigned int irq = *(unsigned int *)p;

	wrhv_mask_irq(irq);
}

static void wrhv_unmask_irq_ipi(void *p)
{
	unsigned int irq = *(unsigned int *)p;

	wrhv_unmask_irq(irq);
}

static void wrhv_irq_task(unsigned long arg)
{
	struct wrhv_irq_struct *list;
	struct wrhv_irq_head *head = &wrhv_irq_head;

	spin_lock(&wrhv_irq_lock);
	list = head->head;
	head->head = NULL;
	head->next = &(head->head);
	spin_unlock(&wrhv_irq_lock);

	while (list) {
		struct wrhv_irq_struct *cur = list;
		int irq = cur->irq;

		switch (cur->action) {
		case WRHV_IRQ_SET_AFFINITY:
		{
			int ocpu = cur->ocpu, ncpu = cur->ncpu;

			smp_call_function_single(ocpu, wrhv_mask_irq_ipi,
						&irq, 1);
			smp_call_function_single(ncpu, wrhv_unmask_irq_ipi,
						&irq, 1);
			vbi_vcore_irq_redirect(irq, ncpu);
			break;
		}
		case WRHV_IRQ_SHUTDOWN:
		{
			int ocpu = cur->ocpu;

			smp_call_function_single(ocpu, wrhv_mask_irq_ipi,
						&irq, 1);
			break;
		}
		case WRHV_IRQ_STARTUP:
		{
			int ocpu = cur->ocpu;

			smp_call_function_single(ocpu, wrhv_unmask_irq_ipi,
						&irq, 1);
			break;
		}
		default:
			printk(KERN_ERR "Unknown action for irq task\n");
		}

		list = cur->next;
		kfree(cur);
	}
}

/* Currently all the external interrupts are routed to cpu 0 and
 * handled by cpu0, so we need make sure the startup/shutdown functions
 * operate cpu 0's vioapic.
 */
static void smp_wrhv_shutdown_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int cpu = cpumask_first(desc->affinity);

	if (cpu == smp_processor_id())
		wrhv_mask_irq(irq);
	else {
		struct wrhv_irq_struct *data;
		struct wrhv_irq_head *head = &wrhv_irq_head;

		data = kmalloc(sizeof(*data), GFP_ATOMIC);
		if (!data)
			return;

		data->irq = irq;
		data->ocpu = cpu;
		data->next = NULL;
		data->action = WRHV_IRQ_SHUTDOWN;

		spin_lock(&wrhv_irq_lock);
		*head->next = data;
		head->next = &(data->next);
		spin_unlock(&wrhv_irq_lock);

		tasklet_schedule(&wrhv_irq_tasklet);
	}
}

static unsigned int smp_wrhv_startup_irq(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int cpu = cpumask_first(desc->affinity);

	if (cpu == smp_processor_id())
		wrhv_unmask_irq(irq);
	else {
		struct wrhv_irq_struct *data;
		struct wrhv_irq_head *head = &wrhv_irq_head;

		data = kmalloc(sizeof(*data), GFP_ATOMIC);
		if (!data)
			return -ENOMEM;

		data->irq = irq;
		data->ocpu = cpu;
		data->next = NULL;
		data->action = WRHV_IRQ_STARTUP;

		spin_lock(&wrhv_irq_lock);
		*head->next = data;
		head->next = &(data->next);
		spin_unlock(&wrhv_irq_lock);

		tasklet_schedule(&wrhv_irq_tasklet);
	}

	return 0;
}

int wrhv_irq_set_affinity(unsigned int irq,
				const struct cpumask *dest)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct wrhv_irq_struct *p;
	int cpu;
	struct wrhv_irq_head *head = &wrhv_irq_head;

	/* Currently we don't support set affinity in direct irq mode */
	if (wrhv_dir_irq) {
		printk(KERN_WARNING "Currently we don't support set affinity"
			" in direct irq mode on e500mc.\n");

		return -1;
	}

	if (cpumask_equal(desc->affinity, dest))
		return 0;

	cpu = cpumask_first(dest);
	/* we only support to bond the irq to signle vcore */
	if (!cpumask_equal(cpumask_of(cpu), dest))
		return -1;

	p = kzalloc(sizeof(*p), GFP_ATOMIC);
	if (!p) {
		printk(KERN_ERR "Can't get memory for set irq affinity\n");
		return -1;
	}

	p->irq = irq;
	p->ocpu = cpumask_first(desc->affinity);
	p->ncpu = cpu;
	p->next = NULL;
	p->action = WRHV_IRQ_SET_AFFINITY;

	/* irq_set_affinity is invoked with irq disabled */
	spin_lock(&wrhv_irq_lock);
	*head->next = p;
	head->next = &(p->next);
	spin_unlock(&wrhv_irq_lock);

	tasklet_schedule(&wrhv_irq_tasklet);

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
void __weak wrhv_fixup_irqs(int cpu)
{
	unsigned int    irq;
	struct irq_desc *desc;

	printk(KERN_INFO "WRHV: reset irq to core0 on core%d\n", cpu);
	for_each_irq_desc(irq, desc) {
		struct cpumask *affinity;
		int ret = 0;

		if (!desc)
			continue;

		if (desc->status & (IRQ_PER_CPU | IRQ_NO_BALANCING))
			continue;

		raw_spin_lock(&desc->lock);

		affinity = desc->affinity;
		cpumask_and(affinity, affinity, cpu_online_mask);
		if (!irq_has_action(irq) ||
		    cpumask_equal(affinity, cpu_online_mask) ||
			!cpu_isset(cpu, *affinity)) {
			raw_spin_unlock(&desc->lock);
			continue;
		}

		if (desc->chip->set_affinity)
			ret = desc->chip->set_affinity(irq, cpumask_of(0));

		if (!ret)
			cpumask_copy(desc->affinity, cpumask_of(0));

		raw_spin_unlock(&desc->lock);
	}
}
#endif /* CONFIG_HOTPLUG_CPU */
#endif

struct irq_chip wrhv_irq_chip = {
	.name		= "WRHV-PIC",
#ifdef CONFIG_SMP
	.startup	= smp_wrhv_startup_irq,
	.shutdown	= smp_wrhv_shutdown_irq,
	.set_affinity	= wrhv_irq_set_affinity,
#endif
	.mask		= wrhv_mask_irq,
	.ack		= wrhv_ack_irq,
	.disable	= wrhv_disable_irq,
	.enable		= wrhv_enable_irq,
	.unmask		= wrhv_unmask_irq,
	.mask_ack	= wrhv_maskack_irq,
	.eoi		= wrhv_eoi_irq,
};

#ifdef CONFIG_SMP
struct irq_chip wrhv_ipi_irq_chip = {
	.name		= "WRHV-IPI-PIC",
	.mask		= wrhv_mask_irq,
	.disable	= wrhv_disable_irq,
	.enable		= wrhv_enable_irq,
	.unmask		= wrhv_unmask_irq,
	.mask_ack	= wrhv_maskack_irq,
	.eoi		= wrhv_ack_irq,
};
#endif

unsigned long wrhv_calculate_cpu_khz(void)
{
	printk(KERN_DEBUG "WRHV: Timestamp Frequency %u Hz\n",
		wr_vb_config->stamp_freq);
	return wr_vb_config->stamp_freq / 1000;
}

irqreturn_t __weak wrhv_timer_interrupt(int irq, void *dev_id)
{
	static long long mark_offset;
	long long ticks;
	int lost_jiffies = 0;
	struct pt_regs *regs = get_irq_regs();

	ticks = wr_vb_status->tick_count;
	ticks -= mark_offset;
	lost_jiffies = ticks - 1;
	mark_offset = wr_vb_status->tick_count;

	do {
		do_timer(1);
		update_process_times(user_mode(regs));
		profile_tick(CPU_PROFILING);
		if (lost_jiffies > (2*HZ)) {
			printk(KERN_DEBUG "Time falling behind %d jiffies\n",
				lost_jiffies);
			break;
		}
	} while (--ticks > 0);

	if (lost_jiffies)
		account_steal_time(NULL, jiffies_to_cputime(lost_jiffies));
	return IRQ_HANDLED;
}

#ifdef CONFIG_PCI
#define WRHV_POLL_IRQ           7
int __weak find_shared_interrupt(char *devfn)
{
	return -1;
}

int __weak wrhv_has_kgdboe(void)
{
	return 0;
}

static void __devinit pci_fixup_wrhv(struct pci_dev *dev)
{
	int irq = -1;
	char *devclass, devname[32] = { "Unknown" };
	int skip_assign_irq = 0;

	switch (dev->class >> 16) {
	case PCI_BASE_CLASS_NETWORK:
		devclass = "Ethernet";
		if (wrhv_has_kgdboe()) {
			skip_assign_irq = 1;
			irq = WRHV_POLL_IRQ;
		}
		break;

	case PCI_BASE_CLASS_STORAGE:
		devclass = "IDE";
		break;

	case PCI_BASE_CLASS_DISPLAY:
		devclass = "VGA";
		break;

	case PCI_BASE_CLASS_SERIAL:
		/* Examine more bits of the class to see if the
		   device is specifically usb.
		*/
		if ((dev->class >> 8) == PCI_CLASS_SERIAL_USB) {
			devclass = "USB";
			break;
		}
		if ((dev->class >> 8) == PCI_CLASS_SERIAL_SMBUS) {
			devclass = "SMBus";
			break;
		}

	case PCI_BASE_CLASS_SYSTEM:
		/* Examine more bits of the class to see if the
		   device is "other".  This covers Intel's I/O acceleration
		   technology aka ioatdma.
		*/
		if ((dev->class >> 8) == PCI_CLASS_SYSTEM_OTHER) {
			devclass = "System";
			break;
		}

	case PCI_BASE_CLASS_BRIDGE:
		/* Currently its unnecessary to assign irq for
		 * any bridge itself. Maybe we need that in the future :)
		 */
		skip_assign_irq = 1;
		break;

	default:
		skip_assign_irq = 1;
		irq = dev->irq;
		break;
	}

	if (!skip_assign_irq) {
		int sharedint;

		/* See if a shared interrupt first */
		snprintf(devname, sizeof devname, "%x-%x",
			dev->bus->number, dev->devfn);
		sharedint = find_shared_interrupt(devname);
		if (sharedint < 0) {
			/* Not found as a shared int, look it up directly */
			snprintf(devname, sizeof devname, "pci%s_%x:%x",
				devclass, dev->bus->number, dev->devfn);
		} else {
			snprintf(devname, sizeof devname, "pciSharedInt_%x",
				sharedint);
		}
		irq = vbi_find_irq(devname, 1);
		if (irq == VBI_INVALID_IRQ) {
			irq = WRHV_POLL_IRQ;
			printk(KERN_WARNING "WRHV-PCI: find IRQ for %04x:%02x:%02x.%d "
				   "by name [%s] failed. You are now in polling mode.\n",
				   pci_domain_nr(dev->bus),dev->bus->number,
				   PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
				   devname);
		} else
			printk(KERN_INFO "WRHV-PCI: %04x:%02x:%02x.%d shared irq [%d]", pci_domain_nr(dev->bus),
				dev->bus->number, PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn), irq);

	}

	if (irq != -1) {
		dev->irq = irq;
		printk(KERN_INFO "WRHV-PCI: %s CLASS:%x IRQ%d\n",
			devname, dev->class, dev->irq);
	}
}

DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, pci_fixup_wrhv);
#endif

static int wrhv_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_STANDBY || state == PM_SUSPEND_MEM;
}

static int wrhv_suspend_begin(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 0;
		default:
			return -EINVAL;
	}
}

extern int softlockup_thresh;
static int softlockup_thresh_bk;
static void wrhv_save_softlockup_thresh(void)
{
	softlockup_thresh_bk = softlockup_thresh;
	softlockup_thresh = 0;
}

static void wrhv_restore_softlockup_thresh(void)
{
	softlockup_thresh = softlockup_thresh_bk;
}

static int wrhv_suspend_enter(suspend_state_t state)
{
	vbi_vb_suspend(VBI_BOARD_ID_GET(), 0);
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static cpumask_var_t wrhv_cpus;

static int wrhv_save_cpus(void)
{
	int cpu, first_cpu, error;

	cpumask_clear(wrhv_cpus);

	if (num_online_cpus() == 1)
		return 0;

	first_cpu = cpumask_first(cpu_online_mask);

	for_each_online_cpu(cpu) {
		if (cpu == first_cpu)
			continue;
		error = cpu_down(cpu);
		if (!error)
			cpumask_set_cpu(cpu, wrhv_cpus);
		else {
			printk(KERN_ERR "wrhv_bring_nonboot_cpu CPU%d down: %d\n",
				cpu, error);
			break;
		}
	}

	if (!error)
		BUG_ON(num_online_cpus() > 1);
	else
		printk(KERN_ERR "Non-boot CPUs are not disabled\n");

	return error;
}

static void wrhv_restore_cpus(void)
{
	int cpu, error;

	if (cpumask_empty(wrhv_cpus))
		return;

	for_each_cpu(cpu, wrhv_cpus) {
		error = cpu_up(cpu);
		if (!error) {
			printk("CPU%d is up\n", cpu);
			continue;
		}
		printk(KERN_WARNING "Error bringing CPU%d up: %d\n", cpu, error);
	}

}

#else
static inline void wrhv_save_cpus(void)
{}

static inline void wrhv_restore_cpus(void)
{}

#endif

static int wrhv_suspend_prepare(void)
{
	wrhv_save_softlockup_thresh();
	wrhv_save_cpus();

	return 0;
}

static int wrhv_suspend_prepare_late(void)
{
	return 0;
}

static void wrhv_suspend_end(void)
{
	wrhv_restore_cpus();
	wrhv_restore_softlockup_thresh();
}


static struct platform_suspend_ops wrhv_suspend_ops = {
	.valid = wrhv_suspend_valid,
	.begin = wrhv_suspend_begin,
	.prepare = wrhv_suspend_prepare,
	.prepare_late = wrhv_suspend_prepare_late,
	.enter = wrhv_suspend_enter,
	.end = wrhv_suspend_end,
};

int __weak wrhv_arch_late_init(void)
{
	return 0;
}

/* sysfs ksets */
struct kset *windriver_kset;
EXPORT_SYMBOL(windriver_kset);
static int wrhv_init_sysfs(void)
{
	/* Create parent windriver kset */
	windriver_kset = kset_create_and_add(WINDRIVER_NAME, NULL, NULL);
	if (!windriver_kset)
		return -ENOMEM;

	return 0;
}

/* debugfs dentry's */
struct dentry *windriver_dentry;
EXPORT_SYMBOL(windriver_dentry);
static int wrhv_init_debugfs(void)
{
	windriver_dentry = debugfs_create_dir(WINDRIVER_NAME, NULL);
	if (!windriver_dentry)
		return -ENODEV;

	return 0;
}

static char wrhv_shell[] = "wrhv_shell";
static ssize_t hysh_show(struct kobject *kobj, struct attribute *attr,
	char *buf)
{
	sprintf(buf, "0\n");
	return 2;
}

static ssize_t hysh_action(struct kobject *kobj, struct attribute *attr,
	const char *buf, size_t len)
{
	wrhv_save_softlockup_thresh();
	wrhv_save_cpus();
	vbi_shell_start_debug(0);
	wrhv_restore_cpus();
	wrhv_restore_softlockup_thresh();
	return len;
}

static struct kobj_attribute hysh_attr = __ATTR(wrhv_shell, 0644, hysh_show, hysh_action);

static int wrhv_hysh_init_sysfs(void)
{
	int ret;
	ret = sysfs_create_file(&windriver_kset->kobj, &hysh_attr.attr);
	if (ret) {
		printk(KERN_ERR "wrhv_sysfs: create %s fail, ret = %d\n",
			wrhv_shell, ret);
		return -ENOMEM;
	}

	return 0;
}

#ifdef CONFIG_SMP
static int wrhv_reset_all_irqs(struct notifier_block *self,
		unsigned long code, void *t)
{
#ifdef CONFIG_SMP
	unsigned int    irq;
	struct irq_desc *desc;

	for_each_irq_desc(irq, desc) {
		struct cpumask *affinity;
		struct cpumask non_bp_affinity;

		if (!desc)
			continue;
		if (desc->status & (IRQ_PER_CPU | IRQ_NO_BALANCING))
			continue;

		raw_spin_lock(&desc->lock);

		affinity = desc->affinity;
		cpumask_and(affinity, affinity, cpu_online_mask);
		cpumask_copy(&non_bp_affinity, affinity);
		cpu_clear(0, non_bp_affinity);
		if (!irq_has_action(irq) ||
		    cpumask_equal(affinity, cpu_online_mask) ||
			cpumask_empty(&non_bp_affinity)) {
			raw_spin_unlock(&desc->lock);
			continue;
		}

		if (desc->chip->set_affinity)
			desc->chip->set_affinity(irq, cpumask_of(0));

		raw_spin_unlock(&desc->lock);
	}
#endif
	return 0;
}
#else
static int wrhv_reset_all_irqs(struct notifier_block *self,
		unsigned long code, void *t)
{
	return 0;
}
#endif

static struct notifier_block wrhv_reboot_notifier = {
	.notifier_call = wrhv_reset_all_irqs,
};

int wrhv_register_reboot_notifier(void)
{
	int ret;
	ret = register_reboot_notifier(&wrhv_reboot_notifier);
	if (ret)
		printk(KERN_ERR
			"WRHV: unable to install reboot notifier"
			"(err = %d)\n", ret);

	return ret;
}


/*
 * Query the hypervisor to get the vb_config struct for a VB
 * return < 0 on failure
 */
int wrhv_get_vb_config(const int vb, struct vb_config *config)
{
	unsigned long cfg_paddr;

	if (config == NULL)
		return -EFAULT;
	if (vb >= VB_MAX_VIRTUAL_BOARDS)
		return -EINVAL;

	if (vbi_vb_find_board_config(vb, 0, (void *)&cfg_paddr) == 0) {
		struct vbi_mem_ctl mem_ctl;

		memset(config, 0, sizeof(struct vb_config));

		mem_ctl.pBuffer = (void *)cfg_paddr;
		mem_ctl.pAddress = (void *)config;
		mem_ctl.size_in = sizeof(struct vb_config);
		mem_ctl.size_out = 0;
		mem_ctl.flags = 0;

		if (vbi_vb_read_mem(&mem_ctl, vb) != 0) {
			printk(KERN_ERR "offload_manager: failed to read "
			       "vb_config. (VB #%d)\n", vb);
			return -EIO;
		}
	} else {
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(wrhv_get_vb_config);

u64 wrhv_phys_offset;
EXPORT_SYMBOL(wrhv_phys_offset);

int __init wrhv_late_init(void)
{
	wrhv_arch_late_init();
	suspend_set_ops(&wrhv_suspend_ops);
	wrhv_init_procfs();
	wrhv_init_sysfs();
	wrhv_init_debugfs();
	wrhv_hysh_init_sysfs();
	wrhv_register_reboot_notifier();
	vbi_guest_phys_to_phys(0, &wrhv_phys_offset);
	return 0;
}
late_initcall(wrhv_late_init);
