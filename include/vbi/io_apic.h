/*
 * io_apic.h - virtual IO APIC definitions
 *
 * Copyright (c) 2008 Wind River Systems, Inc.
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

#ifndef	_VBI_IO_APIC_H
#define	_VBI_IO_APIC_H

#ifndef	_ASMLANGUAGE

/* data structures */

/* IO APIC ID register */
union vioapic_id
{
	struct
	{
		uint32_t reserved1:24;	/* reserved bits */
		uint32_t IOAPIC_id:4;	/* apic id */
		uint32_t reserved2:4;	/* reserved bits */
	} field;
	uint32_t value;
};


/* IO APIC Version Register */

union vioapic_version
{
	struct
	{
		uint32_t version:8;		/* version identifier */
		uint32_t reserved1:8;		/* reserved bits */
		uint32_t maxRedirEntry:8;	/* max # of entries - 1, or
						 * number of IRQ pins - 1 */
		uint32_t reserved2:8;		/* reserved bits */
	} field;

	uint32_t value;
};

/* Virtual IO APIC redirection table entry. Split into high/low 32 */

union vioapic_redir_high
{
	struct
	{
		uint32_t reserved:24;	/* reserved bits */
		uint32_t destination:8;	/* destination field */
	} field;
	uint32_t value;
};

union vioapic_redir_low
{
	struct
	{
		uint32_t vector:8;	/* vector number */
		uint32_t deliveryMode:3;/* delivery mode:
					 * fixed, lowest, SMI, reserved, NMI,
					 * INIT, reserved, extInt */
		uint32_t destMode:1;	/* dest mode: 0=physical, 1=logical */
		uint32_t delivStatus:1;	/* delivery status: 0=idle, 1=pending*/
		uint32_t polarity:1;	/* polarity: 0=high active */
		uint32_t irr:1;		/* rem. IRR (lvl only): 1 before LAPIC
					 * accepts level, 0 after EOI */
		uint32_t trigger:1;	/* 0=edge, 1=level */
		uint32_t mask:1;	/* 1=masked */
		uint32_t reserved:15;	/* reserved bits */
	} field;
	uint32_t value;
};

struct vioapic_entry
{
	union vioapic_redir_low vioapic_low;
	union vioapic_redir_high vioapic_high;
};

typedef struct vioapic
{
	union vioapic_id id;
	union vioapic_version version;
	uint32_t arb;
	uint32_t boot;
	uint32_t reserved[6];
	struct vioapic_entry entry[VIOAPIC_MAX_REDTABLE_ENTRIES];
} VIOAPIC;

#endif /* _ASMLANGUAGE */

#endif	/* _VBI_IO_APIC_H */


