/*
 * x86 arch_vbi.h - x86 architecture specific definitions
 *
 * Copyright 2009 Wind River Systems, Inc.
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

#ifndef _ASM_ARCH_VBI_H
#define _ASM_ARCH_VBI_H

#ifndef _ASMLANGUAGE

/* struct of system descriptor table registers (VBI_GDTR, VBI_IDTR, VBI_LDTR) */
#ifdef _MSC_TOOL
#pragma pack(push,1)
#endif

struct VBI_XDTR64
{
	uint16_t limit;		/* maximum size of the DT */
	uint64_t base;		/* address of DT */
	uint16_t pad[3];
}
#ifndef _MSC_TOOL
__attribute__((packed));
#endif
;

struct VBI_XDTR32
{
	uint16_t limit;
	uint32_t base;
	uint16_t pad;
}
#ifndef _MSC_TOOL
__attribute__((packed));
#endif
;
#ifdef _MSC_TOOL
#pragma pack(pop)
#endif

typedef struct VBI_XDTR64 vbi_gdtr;
typedef struct VBI_XDTR64 vbi_idtr;
typedef struct VBI_XDTR64 vbi_ldtr;

typedef struct VBI_XDTR32 vbi_gdtr32;
typedef struct VBI_XDTR32 vbi_idtr32;
typedef struct VBI_XDTR32 vbi_ldtr32;


/*
 *
 * VB_HREG_SET - hardware register set, for read/write
 *
 * Used by vbi_vb_read_reg/vbi_vb_write_reg to read/write registers in
 * another VB
 *
 *
 */

typedef struct			/* VBI_REG_SET - used for sys_regsRead/Write */
{
	uint32_t  eax;		/* 00: general register		*/
	uint32_t  ebx;		/* 04: general register		*/
	uint32_t  ecx;		/* 08: general register		*/
	uint32_t  edx;		/* 0C: general register		*/
	uint32_t  esi;		/* 10: general register		*/
	uint32_t  edi;		/* 14: general register		*/
	uint32_t  eip;		/* 18: program counter		*/
	uint32_t  ebp;		/* 1C: frame pointer register	*/
	uint32_t  esp;		/* 20: stack pointer register	*/
	uint32_t  eflags;	/* 24: status register		*/
	uint32_t  cr0;		/* 28: control register 0	*/
	uint32_t  cr3;		/* 2C: control register 3	*/
	uint32_t  cr4;		/* 30: control register 4	*/
	vbi_idtr32  idtr;	/* 34: IDT task register	*/
	vbi_gdtr32  gdtr;	/* 3C: GDT task register	*/
	vbi_ldtr32  ldtr;	/* 44: LDT task register	*/
	uint32_t  cs;		/* 4C: code segment		*/
	uint32_t  ss;		/* 50: stack segment		*/
	uint32_t  ds;		/* 54: data segment		*/
	uint32_t  es;		/* 58: E segment		*/
	uint32_t  fs;		/* 5C: F segment		*/
	uint32_t  gs;		/* 60: G segment		*/
	uint32_t  tr;		/* 64: task register		*/
} VBI_HREG_SET;


typedef struct			/* REG_SET - x86 register set	*/
{
	uint64_t   rax;		/* 00: general register		*/
	uint64_t   rbx;		/* 08: general register		*/
	uint64_t   rcx;		/* 10: general register		*/
	uint64_t   rdx;		/* 18: general register		*/
	uint64_t   rsp;		/* 20: stack pointer register	*/
	uint64_t   rbp;		/* 28: frame pointer register	*/
	uint64_t   rsi;		/* 30: general register		*/
	uint64_t   rdi;		/* 38: general register		*/
	uint64_t   r8;	 	/* 40: general register		*/
	uint64_t   r9;	 	/* 48: general register		*/
	uint64_t   r10;		/* 50: general register		*/
	uint64_t   r11;		/* 58: general register		*/
	uint64_t   r12;		/* 60: general register		*/
	uint64_t   r13;		/* 68: general register		*/
	uint64_t   r14;		/* 70: general register		*/
	uint64_t   r15;		/* 78: general register		*/
	uint64_t   rip;		/* 80: program counter		*/
	uint64_t   rflags;		/* 88: status register		*/
	uint64_t   cr0;		/* 90: control register 0	*/
	uint64_t   cr2;		/* 98: control register 2	*/
	uint64_t   cr3;		/* 100: control register 3	*/
	uint64_t   cr4;		/* 108: control register 4	*/
	vbi_idtr   idtr;	/* 110: IDT task register	*/
	vbi_gdtr   gdtr;	/* 11a: GDT task register	*/
	vbi_ldtr   ldtr;	/* 136: LDT task register	*/
	uint64_t   cs;		/* 140: code segment		*/
	uint64_t   ds;		/* 148: data segment		*/
	uint64_t   ss;		/* 150: stack segment		*/
	uint64_t   es;		/* 158: E segment		*/
	uint64_t   fs;		/* 160: F segment		*/
	uint64_t   gs;		/* 168: G segment		*/
	uint64_t   tr;		/* 170: Task register		*/
	/* xxx(gws): excluding FP support */
} VBI_HREG_SET_64;


/* complex register set definition */

typedef union
{
	VBI_HREG_SET    hreg32;	/* 32 bit register set */
	VBI_HREG_SET_64 hreg64;	/* 64 bit register set */
} VBI_HREG_SET_CMPLX;


typedef struct
{
	uint32_t vbiRegType;  /* 00: register set to use */
	uint32_t	    qualifier;   /* 04: optional field, used for alignment */

	VBI_HREG_SET_CMPLX vbiRegSet;
} VBI_HREG_SET_CMPLX_QUALIFIED;


#endif /* _ASMLANGUAGE */


/* x86 uses little endian byte ordering */

#define __VBI_BYTE_ORDER __VBI_LITTLE_ENDIAN

#endif /* _ASM_ARCH_VBI_H */
