/*
 * io_apic.c - Virtual IO APIC library
 *
 * Copyright (c) 2008-2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include <vbi/vbi.h>
#include <vbi/io_apic.h>
#include <vbi/private.h>

/*
 * APIC register get/set macros  Accesses are double words using an
 * indirect addressing scheme.
 * Redirection table entry bits: lower 32 bit
 */

/* IO APIC Register Offset */

#define VIOAPIC_REG_ID		0x00	/* IOAPIC ID */
#define VIOAPIC_REG_VERS	0x01	/* IOAPIC Version */
#define VIOAPIC_REG_ARB		0x02	/* IOAPIC Arbitration ID */
#define VIOAPIC_REG_BOOT	0x03	/* IOAPIC Boot Configuration */
#define VIOAPIC_REG_REDTBL	0x10	/* Redirection Table (24 * 64bit) */

#ifdef DEBUG
#define DEBUGM(fmt, arg...) printk(fmt, ##arg)
#else
#define DEBUGM(fmt, arg...)
#endif

/*
 * IO APIC register get/set macros. Accesses are double words using an
 * indirect addressing scheme.
 *
 * FIXME: convert to readb() and let compiler check types etc.
 */

#define	VIOAPIC_REG_GET(base, offset)		\
	(*((volatile uint32_t *)(((unsigned long *)(base)) + (offset))))

#define	VIOAPIC_REG_SET(base, offset, value)	\
	(*((volatile uint32_t *)(((unsigned long *)(base)) + (offset))) = \
	(value))

/*
 * vbi_get_vioapic_addr - Get VIOAPIC base address
 *
 * This routine gets the base address of the VIOAPIC specified in the VB
 * control structure.
 *
 */
void *vbi_get_vioapic_addr(void)
{
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_get_vioapic_addr);
		return (void *)-1;
	}
	return VBI_VIOAPIC_BASE_GET();
}

/*
 * vbi_set_vioapic_vec - Set a vector for the specified irq entry
 *
 * This routine sets a vector for the specified entry in the VIOAPIC
 * redirection table. The previous entry in the table is trashed. Before
 * setting the new vector in the entry the user can obtain the previous
 * entry by calling vbi_get_vioapic_vec(). The first argument passed to
 * this function represent the index to the VIOAPIC redirection table.  The
 * second argument is the vector to set in the specified entry.  For Intel
 * architectures when an interrupt is raised the vector determines the
 * Interrupt descriptor table (IDT) entry where the IRQ is delivered.
 *
 * This routine is currently not supported for PPC.
 *
 */
int32_t vbi_set_vioapic_vec(int32_t irq, int32_t vector)
{
	volatile VIOAPIC *pVioapic = (volatile VIOAPIC *)VBI_VIOAPIC_BASE_GET();

	if (pVioapic == NULL)
		return VBI_VIOAPIC_NULL;

	if (irq > VIOAPIC_MAX_REDTABLE_ENTRIES-1)
		return VBI_VIOAPIC_IRQ_OUTBOUND;

	VB_DEBUG_MSG("vbi_set_vioapic_vec: base @ 0x%x\n", vioapicBase);
	VB_DEBUG_MSG("Set vector %d: @ 0x%x\n", vioapicBase,
		 &(pVioapic->entry[irq].value));

	pVioapic->entry[irq].vioapic_low.field.vector = vector;

	return 0;
}

/*
 * vbi_get_vioapic_vec - Get a vector in the specified irq entry
 *
 * This routine gets a vector for the specified entry in the VIOAPIC table in
 * VIOAPIC redirection table. It may be used for saving the previous entry
 * before setting a new vector in the specified entry. This takes as an argument
 * the irq number that corresponds to the redirection table offset.
 *
 * This routine is currently not supported for PPC
 *
 */
int32_t vbi_get_vioapic_vec(int32_t irq)
{
	volatile VIOAPIC *pVioapic = (volatile VIOAPIC *)VBI_VIOAPIC_BASE_GET();
	int32_t vector;

	if (pVioapic == NULL)
		return VBI_VIOAPIC_NULL;

	if (irq > VIOAPIC_MAX_REDTABLE_ENTRIES-1)
		return VBI_VIOAPIC_IRQ_OUTBOUND;

	VB_DEBUG_MSG("vbi_get_vioapic_vec: base @ 0x%x\n", vioapicBase);

	vector = pVioapic->entry[irq].vioapic_low.field.vector;

	VB_DEBUG_MSG("vbi_get_vioapic_vec: vector %d: for irq 0x%x\n", vector, irq);

	return vector;
}

/*
 * vbi_unmask_vioapic_irq - Unmask an irq for a virtual board
 *
 * This routine enables the interrupt vector that matches the specified IRQ at
 * the VIOAPIC redirection table. This routine makes a hypercall in order to
 * deliver pending interrupts that might be queued while the irq was masked.
 * Hypervisor delivers any pending IRQ that may have occurred while the IRQ was
 * masked. Then clears the mask bit in the VIOAPIC redirection table for the
 * specified IRQ directed to the calling virtual Core.
 *
 */
int32_t vbi_unmask_vioapic_irq(int32_t irq)
{
	volatile VIOAPIC *pVioapic = (volatile VIOAPIC *)VBI_VIOAPIC_BASE_GET();

	DEBUGM("VIOAPIC base: 0x%x \n", pVioapic);

	if (pVioapic == NULL)
		return VBI_VIOAPIC_NULL;

	if (irq > VIOAPIC_MAX_REDTABLE_ENTRIES-1)
		return VBI_VIOAPIC_IRQ_OUTBOUND;

	return vbi_io_apic_op(VBI_IOAPICIOCTL_UNMASK, irq, 0, 0);
}
EXPORT_SYMBOL(vbi_unmask_vioapic_irq);

/*
 * vbi_mask_vioapic_irq - Mask an irq
 *
 * This routine disables the interrupt vector that matches the specified IRQ at
 * the VIOAPIC for the running core. The mask bit for the IRQ entry in the
 * VIOAPIC redirection table is set to 1. After calling this function hypervisor
 * will deliver this IRQ only if this IRQ is enabled by calling
 * vbi_unmask_vioapic_irq().
 *
 */
int32_t vbi_mask_vioapic_irq(int32_t irq)
{
	volatile VIOAPIC *pVioapic = (volatile VIOAPIC *)VBI_VIOAPIC_BASE_GET();

	if (pVioapic == NULL)
		return VBI_VIOAPIC_NULL;

	if (irq > VIOAPIC_MAX_REDTABLE_ENTRIES-1)
		return -1;

	pVioapic->entry[irq].vioapic_low.field.mask = 1;

	return 0;
}

/*
 * vbi_ack_vioapic_irq - Acknowledge an irq
 *
 * This routine acknowledges the specified IRQ for the running core.
 * Calling this routine causes Hypervisor to purge any pending interrupt
 * that arrived while the acknowledgement was pending. When a virual board
 * receives an interrupt it must call this function. Otherwise Hypervisor
 * will block subsequent interrupt for the same IRQ. Exceptions are not
 * required to be acknowledged.
 *
 */
int32_t vbi_ack_vioapic_irq(int32_t irq)
{

	if (irq > VIOAPIC_MAX_REDTABLE_ENTRIES-1)
		return VBI_VIOAPIC_IRQ_OUTBOUND;

	return vbi_io_apic_op(VBI_IOAPICIOCTL_EOI, irq, 0, 0);
}

/*
 * vbi_send_vioapic_irq - Send an interrupt
 *
 * This routine makes a hypercall to trigger an IRQ to one or more virtual
 * board that are connected to the line. The first argument passed to this
 * function specifies the IRQ number. The second argument specifies the
 * filter to apply to the list of virtual  boards connected to the IRQ.
 * The third argument is applicable only when VIOAPIC_SEND_UNICAST filter
 * is specified.
 *
 * In a SMP system a virtual board may have more than one core. If an IRQ
 * is sent to such system hypervisor will deliver the interrupt to the
 * core that was configured to receive the IRQ. The configuration is
 * provided in the board XML configuration file as the example shown
 * below.
 *
 *  <In Name="IRQ_NAME" Vector="Vector Number" Core= "1" />.
 *
 * The Vector number is not required to be specified in the XML. If not
 * specified the VB manager assignes a vector to an IRQ. The Guest OS can
 * obtain the vector number assigned to an IRQ by calling
 * vbi_find_irq("VECTOR_NAME").
 *
 * The possible values for the filter (second argument) may be:
 *
 *
 *
 * VIOAPIC_SEND_ALL	- Send to the group of virtual boards connected to this
 *                        IRQ include the sender board.
 *
 * VIOAPIC_SEND_OTHERS	- Send to the group of virtual boards connected
 *			  to this IRQ excluding the sender board
 *
 * VIOAPIC_SEND_UNICAST	- Send an interrupt to the specified virtual board. This
 *			  will be delivered only if the destination board is
 *			  connected to this IRQ
 *
 *
 * VIOAPIC_SEND_NONE	- Ignore this call.
 *
 */
int32_t vbi_send_vioapic_irq(int32_t irq, uint32_t filter, uint32_t target)
{
	return vbi_io_apic_op(VBI_IOAPICIOCTL_SEND, irq, filter, target);
}
EXPORT_SYMBOL(vbi_send_vioapic_irq);

/*
 * vbi_send_vcore_vioapic_irq - Send an IRQ to a set of virtual cores within a VB
 *
 * This routine makes a hypercall to send an IRQ to a set of virtual cores
 * specified by a 32bit bitmap value. This routine dispatches the output
 * IRQ to the target cores.  The destination cores must be within the same
 * VB as the core sending the interrupt.  The output irq must be mapped in
 * the VB xml configuration xml file. The name of the output IRQ is
 * matched with an incoming IRQ that is mapped to the VB. Setting a bit
 * corresponding to an invalid virtual core will simply be ignored. The
 * same IRQ is delivered for all cores for any given output IRQ. The third
 * argument passed to this function must be defined as
 * VBI_IOAPICSEND_VCORE_NONE.
 *
 */
int32_t vbi_send_vcore_vioapic_irq(int32_t irq, uint32_t coreSet,
			uint32_t options)
{
	/* Certifiable hypervisor does not support this function */
	if (is_cert_hyp()) {
		VBISTAT_VERBOSE(vbi_send_vcore_vioapic_irq);
		return -1;
	}
	return vbi_io_apic_op(VBI_IOAPICIOCTL_VCORE_SEND, irq, coreSet,
				options);
}

/*
 * vbi_redir_vioapic_irq - Redirect an irq to another core
 *
 * This routine makes a hypercall to redirect an irq from one core to another
 * within the same virtual board.
 *
 */
int32_t vbi_redir_vioapic_irq(int32_t irq, int32_t core)
{
	/* Certifiable hypervisor does not support this function */
	if (is_cert_hyp()) {
		VBISTAT_VERBOSE(vbi_redir_vioapic_irq);
		return -1;
	}
	return vbi_vcore_irq_redirect(irq, core);
}

/*
 * vbi_get_pending_vioapic_irq - Get the pending interrupt vector number
 *
 * This routine returns the pending interrupt vector number for the calling VB. 
 * Behavior of this function depends on the underlying architectures.
 * For some architectures (e.g Intel) each external interrupt is delivered into
 * a specific address therefore it is not required to demultiplex interrupts.
 * Other architectures (e.g PPC) share the same offset for all multiple external
 * interrupts which requires support to read the specific vector pending. This
 * function is provided for all architectures for completeness.
 */
int32_t vbi_get_pending_vioapic_irq(void)
{
	int32_t pendingIrq;

	/* shouldn't these ops be atomic? */

	pendingIrq = VBI_INT_VCORE_PENDING();

	if (pendingIrq != 0xffff)
		VBI_INT_PENDING_SET(0xffff); 

	return (pendingIrq);
}


/*
 * vbi_disp_vioapic - display the virtual I/O APIC table
 */
void vbi_disp_vioapic(void)
{
	union vioapic_redir_low reg_redir_low;
	union vioapic_redir_high reg_redir_high;
	volatile VIOAPIC *pVioapic = (volatile VIOAPIC *)VBI_VIOAPIC_BASE_GET();
	uint32_t i;

	static const char *deliveryModes[] =
	{
		" Fixed",	/* 0 */
		"Lowest",	/* 1 */
		"   SMI",	/* 2 */
		"  rsvd",	/* 3 */
		"   NMI",	/* 4 */
		"  INIT",	/* 5 */
		"  rsvd",	/* 6 */
		"ExtINT"	/* 7 */
	};

	/* Certifiable hypervisor does not support this function */
	if (is_corevbi_only()) {
		VBISTAT_VERBOSE(vbi_disp_vioapic);
		return;
	}

	/* get the general virtual I/O APIC info */
	printk("Virtual I/O APIC:\n");
	printk("  Base address: 0x%p\n", pVioapic);
	printk("  id:          %d\n", pVioapic->id.field.IOAPIC_id);
	printk("  version:     %d\n", pVioapic->version.field.version);
	printk("  max entries: %d\n", pVioapic->version.field.maxRedirEntry);

	/* dump the virtual I/O APIC table */
	printk("\nRedirection Table:\n");
	printk("Entry Vector Mask Status Trig"
		"  Pol IRR DeliverMode Destination\n");
	printk(
	"----- ------ ---- ------ ----- --- --- ----------- -----------\n");

	for (i=0; i < VIOAPIC_MAX_REDTABLE_ENTRIES; i++) {

	reg_redir_low  = pVioapic->entry[i].vioapic_low;
	reg_redir_high = pVioapic->entry[i].vioapic_high;

	printk(" %3.3d   %3.3d   %4s %6s %5s %3s  %1d     %6s   %6s %d\n",
			i,
			reg_redir_low.field.vector,
			reg_redir_low.field.mask ? "MASK" : "none",
			reg_redir_low.field.delivStatus ? "PEND" : "idle",
			reg_redir_low.field.trigger ? "level" : "edge",
			reg_redir_low.field.polarity ? "lo" : "hi",
			reg_redir_low.field.irr,
			deliveryModes[reg_redir_low.field.deliveryMode],
			reg_redir_low.field.destMode ? "Set: " : "ID: ",
			reg_redir_high.field.destination);
	}
	printk("\n");
}
