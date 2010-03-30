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

int default_machine_kexec_prepare(struct kimage *image)
{
	return 0;
}
