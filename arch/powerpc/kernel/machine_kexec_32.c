/*
 * PPC32 code to handle Linux booting another kernel.
 *
 * Copyright (C) 2002-2003 Eric Biederman  <ebiederm@xmission.com>
 * GameCube/ppc32 port Copyright (C) 2004 Albert Herranz
 * Copyright (C) 2005 IBM Corporation.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#include <linux/kexec.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <asm/hw_irq.h>
#include <asm/io.h>

typedef NORET_TYPE void (*relocate_new_kernel_t)(
				unsigned long indirection_page,
				unsigned long reboot_code_buffer,
				unsigned long start_address) ATTRIB_NORET;

#ifdef CONFIG_SMP
#include <asm/machdep.h>
#include <asm/smp.h>
void smp_stop_cpus(struct kimage *image)
{
	if (!smp_ops->kexec_stop_cpus) {
		panic("PowerPC smp_ops.kexec_stop_cpus not implemented\n"
			"for this type of target !\n");
		local_irq_disable();
		while(1);
		/* not reached, we're screwed, missing implementation:
		 * no sense going forward as any progress would be
		 * sheer luck and would not result in a stable system */
	}

	smp_ops->kexec_stop_cpus(image);
}
extern u32 relocate_new_kernel_secondary_spin;
extern u32 relocate_new_kernel_spin_addr;
extern u32 relocate_new_kernel_ready;
#else /* !CONFIG_SMP */
void smp_stop_cpus(struct kimage *image)
{
	/* non-SMP, only lock interrupts since it is the responsibility
	 * of smp_stop_cpus(). */
	local_irq_disable();
}
#endif /* CONFIG_SMP */

/*
 * This is a generic machine_kexec function suitable at least for
 * non-OpenFirmware embedded platforms.
 * It merely copies the image relocation code to the control page and
 * jumps to it.
 * A platform specific function may just call this one.
 */

extern const unsigned char relocate_new_kernel[];
extern const unsigned int relocate_new_kernel_size;
typedef void(*rnk_t)(unsigned long *, unsigned long *, unsigned long);

#ifdef CONFIG_SMP
/* This is currently only used by SMP code but could be used by UP or
 * shared code if needed: if that's ever the case, remove conditional
 * compile flag.
 */
static u32 kexec_find_reloc(struct kimage *image, u32 symbol)
{
	u32 base = (u32)page_address(image->control_code_page);
	u32 offset = symbol - (u32)relocate_new_kernel;
	return base + offset;
}
#endif

/* This is in its own routine since it can be called by a CPU that is not
 * the one on which 'kexec -e' was invoked, in some special cases, eg. a
 * target which expects CPU0 to be the boot CPU. */
void kexec_leave_kernel(struct kimage *image)
{
	unsigned long page_list;
	unsigned long reboot_code_buffer, reboot_code_buffer_phys;
	relocate_new_kernel_t rnk;

	page_list = image->head;
	reboot_code_buffer =
			(unsigned long)page_address(image->control_code_page);
	reboot_code_buffer_phys = virt_to_phys((void *)reboot_code_buffer);

	flush_icache_range(reboot_code_buffer,
				reboot_code_buffer + KEXEC_CONTROL_PAGE_SIZE);
	printk(KERN_INFO "Bye!\n");

	/* call kernel relocation code in control page */
	rnk = (relocate_new_kernel_t) reboot_code_buffer;
	(*rnk)(page_list, reboot_code_buffer_phys, image->start);

	/* not reached */
}

void default_machine_kexec(struct kimage *image)
{
	u32 reboot_code_buffer;

	/* we need both effective and real address here */
	reboot_code_buffer = (u32)page_address(image->control_code_page);

	/* copy our kernel relocation code to the control code page */
	memcpy((void *)reboot_code_buffer, relocate_new_kernel,
						relocate_new_kernel_size);

	smp_mb(); /* ensure memcpy finished before secondaries try to run
		 * the relocated code */

	/* Interrupts aren't acceptable while we reboot, but can't be locked
	 * before calling smp_stop_cpus() since it invokes smp_call_function()
	 * which complains if IRQs are disabled. smp_stop_cpus() is thus
	 * responsible for locking them, even in the UP case. */

	smp_stop_cpus(image);

	kexec_leave_kernel(image);

	/* not reached */
}

#ifdef CONFIG_SMP
/* CPU 1 will always be the one calling this function */
static void _smp_kexec_secondary_cpu_down(void *arg)
{
	u32 rnkss; 	 /* relocate_new_kernel_secondary_spin */
	u32 spin;	 /* addr of the relocated start address
			  * variable on which CPU1 will spin */
	u32 ready;	 /* addr of the relocated ready variable */
	rnk_t rnk;	 /* relocate_new_kernel() */

	struct kimage *image = (struct kimage *)arg;

	rnkss = (u32)&relocate_new_kernel_secondary_spin;
	rnkss = virt_to_phys((void *)kexec_find_reloc(image, rnkss));

	spin  = (u32)((&relocate_new_kernel_spin_addr));
	spin  = virt_to_phys((void *)kexec_find_reloc(image, spin));

	ready = (u32)&relocate_new_kernel_ready;
	ready = virt_to_phys((void *)kexec_find_reloc(image, ready));

	rnk = (rnk_t)kexec_find_reloc((struct kimage *)arg,
				(u32)relocate_new_kernel);

	local_irq_disable();

	flush_icache_range((u32)rnk, (u32)rnk + KEXEC_CONTROL_PAGE_SIZE);

	rnk((unsigned long *)spin, (unsigned long *)ready, rnkss);
	/* not reached */
}

static void _smp_kexec_wait_for_secondaries(void *arg)
{
	volatile u32 *ready;	 /* addr of the relocated ready variable,
				  * we spin on it, don't want it to be
				  * optimized out. */
	char buffer[32];

	local_irq_disable();
	ready = (void*)kexec_find_reloc((struct kimage *)arg,
				(u32)&relocate_new_kernel_ready);
	while(!(*ready)) {
		cpu_relax();
	}
	mdelay(1);	/* should be plenty for cpu1 to start spinning
			 * on the start address variable */
}

static void _smp_kexec_leave_kernel(void *arg)
{
	_smp_kexec_wait_for_secondaries(arg);
	kexec_leave_kernel(arg);
}

void default_kexec_stop_cpus(void *arg)
{
	int cpu;

	/* Initialization from head_[32|fsl_booke].S expects HW CPU #0 as
	 * the boot CPU: thus, if we're CPU1, call CPU0 and have it do
	 * the rest of the shutdown sequence, then put ourselves on
	 * a spin; if we're CPU0, call CPU1 to put itself on a spin,
	 * then do the rest of the shutdown sequence. */
	preempt_disable();
	/* get hardware CPU# from special Processor Identity Register */
	cpu = mfspr(SPRN_PIR);
	if (0 == cpu) {
		/* shutdown cpu 1 and wait for it */
		smp_call_function(_smp_kexec_secondary_cpu_down, arg, 0);
		_smp_kexec_wait_for_secondaries(arg);

		/* was called from default_machine_kexec, continues there */
	} else {
		smp_call_function(_smp_kexec_leave_kernel, arg, 0);
		_smp_kexec_secondary_cpu_down(arg);

		/* not reached, going to wait on
		 * relocate_new_kernel_secondary_spin() */
	}
}
#endif /* CONFIG_SMP */

int default_machine_kexec_prepare(struct kimage *image)
{
	return 0;
}
