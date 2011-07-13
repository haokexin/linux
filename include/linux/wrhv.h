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

#ifndef __LINUX_WRHV_H
#define __LINUX_WRHV_H

#include <linux/proc_fs.h>
#include <linux/irqreturn.h>
#include <vbi/vbi.h>

extern irqreturn_t wrhv_timer_interrupt(int irq, void *dev_id);
extern unsigned long wrhv_calculate_cpu_khz(void);
extern int wrhv_get_vb_config(const int vb, struct vb_config *config);

extern struct irq_chip wrhv_irq_chip;
extern struct proc_dir_entry *wrhv_procfs_root;

#ifdef CONFIG_SMP
extern struct irq_chip wrhv_ipi_irq_chip;
#endif

#ifdef CONFIG_HOTPLUG_CPU
extern void wrhv_fixup_irqs(int);
#endif

#define WINDRIVER_NAME "windriver"

/* IO APIC register get/set macros */
/* Accesses are double words using an
 * indirect addressing scheme.
 */
/* Redirection table entry bits: lower 32 bit */

#define VIOAPIC_INT_MASK	0x00010000
#define VIOAPIC_LEVEL		0x00008000
#define VIOAPIC_EDGE		0x00000000
#define VIOAPIC_HIGH		0x00000000
#define VIOAPIC_REMOTE		0x00004000
#define VIOAPIC_LOW		0x00002000
#define VIOAPIC_LOGICAL		0x00000800
#define VIOAPIC_PHYSICAL	0x00000000
#define VIOAPIC_FIXED		0x00000000
#define VIOAPIC_LOWEST		0x00000100
#define VIOAPIC_SMI		0x00000200
#define VIOAPIC_NMI		0x00000400
#define VIOAPIC_INIT		0x00000500
#define VIOAPIC_EXTINT		0x00000700
#define VIOAPIC_VEC_MASK	0x000000ff

/* Redirection table entry size per IO APIC */

#define MAX_REDTABLE_ENTRIES_PER_APIC	24
#define VIOAPIC_REG_REDTBL		0x10	/* Redirection Table (24 * 64bit) */

#define	VIOAPIC_REG_GET(base, offset)		\
		(*((volatile unsigned int *)((base) + (offset))))

#define	VIOAPIC_REG_SET(base, offset, value)	\
		(*((volatile unsigned int *)((base) + (offset))) = (value))

/* IO APIC redirection table entry. Split into high/low 32 */

typedef union {
	struct {
		uint32_t reserved:24;	/* reserved bits */
		uint32_t destination:8;	/* destination field */
	} field;
	uint32_t value;
} ioapic_redir_high;

typedef union {
	struct {
		uint32_t vector:8;	/* vector number */
		uint32_t deliveryMode:3;/* delivery mode:
					 * fixed, lowest, SMI, reserved, NMI,
					 * INIT, reserved, extInt */
		uint32_t destMode:1;	/* destination mode: 0=physical, 1=logical */
		uint32_t delivStatus:1;	/* delivery status: 0=idle, 1=pending*/
		uint32_t polarity:1;	/* polarity: 0=high active */
		uint32_t irr:1;		/* remote IRR (level only): 1 before LAPIC
					 * accepts level, 0 after EOI */
		uint32_t trigger:1;	/* 0=edge, 1=level */
		uint32_t mask:1;	/* 1=masked */
		uint32_t reserved:15;	/* reserved bits */
	} field;

	uint32_t value;
} vioapic_redir_low;

#endif	/* __LINUX_WRHV_H */
