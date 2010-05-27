/*
 * machine_kexec.c for kexec
 * Created by <nschichan@corp.free.fr> on Thu Oct 12 15:15:06 2006
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#include <linux/kexec.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/page.h>

extern const unsigned char relocate_new_kernel[];
extern const size_t relocate_new_kernel_size;

extern unsigned long kexec_start_address;
extern unsigned long kexec_indirection_page;

int (*_machine_kexec_prepare)(struct kimage *);
void (*_machine_kexec_shutdown)(void);
void (*_machine_crash_shutdown)(struct pt_regs *regs);
void (*_machine_cache_flush)(void) = NULL;
void (*_machine_smp_handle_restart)(unsigned long reloc) = NULL;

int
machine_kexec_prepare(struct kimage *kimage)
{
	if (_machine_kexec_prepare)
		return _machine_kexec_prepare(kimage);
	return 0;
}

void
machine_kexec_cleanup(struct kimage *kimage)
{
}

void
machine_shutdown(void)
{
	if (_machine_kexec_shutdown)
		_machine_kexec_shutdown();
}

void
machine_crash_shutdown(struct pt_regs *regs)
{
	if (_machine_crash_shutdown)
		_machine_crash_shutdown(regs);
	else
		default_machine_crash_shutdown(regs);
}

typedef void (*noretfun_t)(void) __attribute__((noreturn));

#ifdef CONFIG_SMP
void (*relocated_kexec_smp_wait) (void *);
atomic_t kexec_ready_to_reboot = ATOMIC_INIT(0);
static void default_machine_smp_handle_restart(unsigned long reloc)
{
	/* All secondary cpus now may jump to kexec_wait cycle */
	relocated_kexec_smp_wait =
		(void *)(reloc + (kexec_smp_wait - relocate_new_kernel));
	smp_wmb();
	atomic_set(&kexec_ready_to_reboot, 1);
}
#else
static void default_machine_smp_handle_restart(unsigned long reloc)
{
}
#endif

void
machine_kexec(struct kimage *image)
{
	unsigned long reboot_code_buffer;
	unsigned long entry;
	unsigned long *ptr;

	reboot_code_buffer =
	  (unsigned long)page_address(image->control_code_page);

	kexec_start_address = (unsigned long) phys_to_virt(image->start);

	/* kexec_indirection_page is marked with the IND_INDIRECTION flag
	 * in the lower bits of the address. The assembly code in
	 * relocate_kernel.S will take care of masking it off.
	 *
	 * The image->head is a physical address: we need the virtual mapping.
	 *
	 * In the case of a crash_dump kernel, assign a dummy IND_DONE page
	 */
	if(KEXEC_TYPE_CRASH == image->type) {
		kexec_indirection_page = IND_DONE;
	} else {
		kexec_indirection_page =
			(unsigned long)phys_to_virt(image->head);
	}

	memcpy((void*)reboot_code_buffer, relocate_new_kernel,
	       relocate_new_kernel_size);

	/*
	 * The generic kexec code builds a page list with physical addresses.
	 * They are directly accessible through KSEG0 (or CKSEG0 or XPHYS if on
	 * 64bit system), hence the phys_to_virt() call.
	 */
	for (ptr = &image->head; (entry = *ptr) && !(entry &IND_DONE);
	     ptr = (entry & IND_INDIRECTION) ?
	       phys_to_virt(entry & PAGE_MASK) : ptr + 1) {
		if (*ptr & IND_SOURCE || *ptr & IND_INDIRECTION ||
		    *ptr & IND_DESTINATION)
			*ptr = (unsigned long) phys_to_virt(*ptr);
	}

	/*
	 * we do not want to be bothered.
	 */
	local_irq_disable();

	printk("Will call new kernel at %08lx\n", image->start);
	printk("Bye ...\n");

	if(_machine_cache_flush) {
		_machine_cache_flush();
	} else {
		__flush_cache_all();
	}

	if(_machine_smp_handle_restart) {
		_machine_smp_handle_restart(reboot_code_buffer);
	} else {
		default_machine_smp_handle_restart(reboot_code_buffer);
	}

	((noretfun_t) reboot_code_buffer)();
}

/*
 * crashkernel=size at addr specifies the location to reserve for
 * a crash kernel.  By reserving this memory we guarantee
 * that linux never sets it up as a DMA target.
 * Useful for holding code to do something appropriate
 * after a kernel panic.
 */
static int __init mips_parse_crashkernel(char *arg) {
	unsigned long size, base;

	size = memparse(arg, &arg);
	if (*arg == '@') {
		base = memparse(arg+1, &arg);
		/*
		 * FIXME: Do I want a sanity check
		 * to validate the memory range?
		 */
		crashk_res.start = base;
		crashk_res.end   = base + size - 1;
	}

	return 0;
}
early_param("crashkernel", mips_parse_crashkernel);
