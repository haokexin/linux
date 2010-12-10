/*
 * asm/vbi.h - ARM tool dependent headers
 *
 * Copyright 2008-2010 Wind River Systems, Inc.
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

#ifndef __INCsysArmAsmh
#define __INCsysArmAsmh

#if !defined(_DIAB_TOOL) && !defined(_GNU_TOOL)
#define	_GNU_TOOL
#endif

#define	_ARM_HREG_SIZE		4	/* default register size */
#define _ARM_TEXT_SEG_ALIGN     4	/* 4 byte text segment alignment */
#define REG_STORAGE		(_ARM_HREG_SIZE * 16)
#define _ARM_VECTOR_SEG_ALIGN	0x1000	/* page alignment */

/* macros for stack frame */

/*
SVR4 Stack space allocation:

    Before Dynamic stack allocation

    +----------------------------------+
    |       Back Chain                 |
    |----------------------------------|
    |  Register save areas             |
    |----------------------------------|
    |  local,non-static variables      |
    |----------------------------------|
    |  parameter lists for callees     | (overloaded parameters with #)
    |----------------------------------|
    |      LR save word                |
    |----------------------------------|
SP=>|      Back chain                  |
    +----------------------------------+

    After Dynamic stack allocation

    +----------------------------------+
    |       Back Chain                 |
    |----------------------------------|
    |  Register save areas             |
    |----------------------------------|
    |  local,non-static variables      |
    |----------------------------------|
    |  Dynamic Allocation Area         | 16 byte stack alignment
    |----------------------------------|
    |  parameter lists for callees     |
    |----------------------------------|
    |      LR save word                |
    |----------------------------------|
SP=>|      Back chain                  |
    +----------------------------------+ 16 bytes stack alignment
*/

  /* Stack and Allocation alignment */

/*
 * While it is possible to use different stack alignments for different
 * ARM processors, current compilers use 16-byte alignment for all.
 */

#define _CPU_STACK_ALIGN_SIZE	16	/* stack alignment (for all ARM) */
#define	_CPU_STACK_ALIGN_SHIFT	4

#define	FRAMEBASESZ		16	/* minimum stack frame size */

/*
 *
 * _WRS_ARCH_USER_STACK_FRAME_EXTENSION - creating words on the stack for the 
 *                                        back chain word and the LR register.
 *
 * This macro is used in creating the initial stack frame for an RTP's initial 
 * task. It performs the extra steps of creating words on the stack for the 
 * back chain word and the LR register. Both these locations are set to 0 to 
 * prevent stack traces and debuggers from looking ahead.
 *
 */

#define _WRS_ARCH_USER_STACK_FRAME_EXTENSION(sp)			     \
do									     \
{								     \
	(sp) -= 2*sizeof(int *);					     \
	*((int *)(sp)) = (int)0;					     \
	*((int *)((sp)+4)) = (int)0;					     \
} while ((0))


#ifdef	_ASMLANGUAGE

#define hash #

#ifdef	_GNU_TOOL

/* Macro for hiadjust and lo */

#define HIADJ(arg)	arg@ha
#define HI(arg)		arg@h
#define LO(arg)		arg@l

#endif	/* _GNU_TOOL */

#ifdef	_DIAB_TOOL

/* Macro for hiadjust and lo */

#define HIADJ(arg)      %hiadj(arg)
#define HI(arg)		%hi(arg)
#define LO(arg)      	%lo(arg)

#endif	/* _DIAB_TOOL */

/*
 * define r2 as VTOC/GOT(EABI), system use(SVR4)/TOC/GOT(PO),dedicated. 
 * define r13 as CTOC/GOT anchor pointer, dedicated (EABI), non-volatile
 * register (SVR4, PO) 
 */

#define FUNC(func)	    func
#define FUNC_LABEL(func)    func:
#define VAR(name) name

#if ARM_THUMB 
#define _ARM_FUNCTION_CALLED_FROM_C(a) \
	.code	16	;\
	.balign	4	;\
	.thumb_func	;\
a:			;\
	BX	pc	;\
	NOP		;\
	.code	32	;\
A##a:
#else
#define _ARM_FUNCTION_CALLED_FROM_C(a) \
	.code	32	;\
	.balign	4	;\
a:
#endif

#define _ARM_FUNCTION(a)	\
	.code	32	;\
	.balign	4	;\
a:

#if ARM_THUMB
#define _THUMB_FUNCTION(a)	\
	.code	16	;\
	.balign	2	;\
	.thumb_func	;\
a:
#endif

/* place the address of label into register r */
#define _ARM_PER_CPU_ADRS_GET(r, scratch, label)              \
        _ARM_CPU_INDEX_GET(r)                      ; \
        ADD  r, r, r, LSL ARM_WIND_VARS_ALIGN_SHIFT         ; \
        LDR  scratch, L$_vxKernelVars                       ; \
        ADD  scratch, scratch, ARM_HASH _ARM_WIND_VARS_OFFSET(label) ; \
        ADD  r, r, scratch

/* place the value at label into register r */
#define _ARM_PER_CPU_VALUE_GET(r,scratch,label)               \
        _ARM_CPU_INDEX_GET(r)                      ; \
        ADD  r, r, r, LSL ARM_WIND_VARS_ALIGN_SHIFT         ; \
        LDR  scratch, L$_vxKernelVars                       ; \
        ADD  scratch, scratch, ARM_HASH _ARM_WIND_VARS_OFFSET(label) ; \
        ADD  r, r, scratch                                  ; \
        LDR  r, [r]

/* place value at label in ra, also place address at label in rv */
#define _ARM_PER_CPU_VALUE_AND_ADRS_GET(rv,ra,label)       \
        _ARM_CPU_INDEX_GET(rv)                       ; \
        ADD  rv, rv, rv, LSL ARM_WIND_VARS_ALIGN_SHIFT   ; \
        LDR  ra, L$_vxKernelVars                         ; \
        ADD  ra, ra, ARM_HASH _ARM_WIND_VARS_OFFSET(label)        ; \
        ADD  ra, ra, rv                                  ; \
        LDR  rv, [ra]

#define	FUNC_EXPORT(func)	.global	func ;
#define	DATA_EXPORT(var)	.globl	VAR(var)
#define	FUNC_IMPORT(func)	.extern	FUNC(func)
#define	DATA_IMPORT(var)	.extern	VAR_DECL(var)
#define	FUNC_BEGIN(func)	FUNC_LABEL(func)
#define	FUNC_END(func)		.size	FUNC(func), . - FUNC(func)

/* Macro for beginning a text segment */

#define _WRS_TEXT_SEG_START \
        .text ; .balign _ARM_TEXT_SEG_ALIGN

#define	FRAMESZ(nregs)	\
    	  ALIGN_UP((FRAMEBASESZ + nregs * _ARM_HREG_SIZE), _STACK_ALIGN_SIZE)

#define AUDIT_STUB_CREATE(x)           \
        FUNC_EXPORT(x);            \
        FUNC_BEGIN(x);             \
        FUNC_END(x);              

#define AUDIT_STUB_VAR(x)              \
        DATA_EXPORT(x)              \
        x:

/* Macro to form the contents of the CONTEXTIDR register
 *
 * Inputs: ctx (ctx *), asid
 * Output: ctx (unchanged), asid (CONTEXTIDR)
 */

#define BUILD_CONTEXTIDR(ctx, asid, x, y)			\
	/* find ctx->id */					\
	ldr	x, [ctx, hash(CTX_ID_OFF)]			; \
	ldr	y, =(CONTEXTIDR_CTXID_BASEMASK)			; \
	and	x, x, y						; \
	orr	asid, asid, x, lsl hash(CONTEXTIDR_CTXID_SHIFT)	; \
	/* find ctx->mmu->vmmuHandle */				; \
	ldr	x, [ctx, hash(CTX_MMU_OFF)]			; \
	ldr	x, [x, hash(CTX_MMU_VMMU_HANDLE)]		; \
	ldr	y, =(CONTEXTIDR_PROCHANDLE_BASEMASK)		; \
	and	x, x, y						; \
	orr	asid, asid, x, lsl hash(CONTEXTIDR_PROCHANDLE_SHIFT);

#else	/* _ASMLANGUAGE */

#define _WRS_ASM(x) __asm volatile (x)

/*
 * Use constant sizes if known (when building for a specific CPU type)
 * else fetch from a global variable (when building for generic ARM)
 */

#ifdef	_CPU_STACK_ALIGN_SIZE
#define	_STACK_ALIGN_SIZE	_CPU_STACK_ALIGN_SIZE
#else	/* _CPU_STACK_ALIGN_SIZE */
#define	_STACK_ALIGN_SIZE	_armStackAlignSize
extern	int	_armStackAlignSize;
#endif	/* _CPU_STACK_ALIGN_SIZE */

#ifdef	_CPU_ALLOC_ALIGN_SIZE
#define	_ALLOC_ALIGN_SIZE	_CPU_ALLOC_ALIGN_SIZE
#else	/* _CPU_ALLOC_ALIGN_SIZE */
#define	_ALLOC_ALIGN_SIZE	_armAllocationQuantumSize
extern	int	_armAllocationQuantumSize;
#endif	/* _CPU_ALLOC_ALIGN_SIZE */

#define	FUNCREF(func)	func

#endif	/* _ASMLANGUAGE */

#endif /* __INCsysArmAsmh */
