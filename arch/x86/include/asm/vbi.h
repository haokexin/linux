/*
 * x86 vbi.h - x86 tool dependent headers
 *
 * Copyright 2007-2009 Wind River Systems, Inc.
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

#ifndef __ASM_VBI_H
#define __ASM_VBI_H

#ifdef CONFIG_64BIT
#ifndef LP64
#define LP64
#endif
#endif

/*
 * The following definitions are used for symbol name compatibility.
 *
 * When #if 1, sources are assembled assuming the compiler
 * you are using does not generate global symbols prefixed by "_".
 * (e.g. elf/dwarf)
 *
 * When #if 0, sources are assembled assuming the compiler
 * you are using generates global symbols prefixed by "_".
 * (e.g. coff/stabs)
 */

#if	TRUE
#define FUNC(sym)		sym
#define FUNC_LABEL(sym)		sym:
#else
#define FUNC(sym)		_##sym
#define FUNC_LABEL(sym)		_##sym:
#endif

#define VAR(sym)		FUNC(sym)

/*
 * These macros are used to declare assembly language symbols that need
 * to be typed properly(func or data) to be visible to the OMF tool.
 * So that the build tool could mark them as an entry point to be linked
 * correctly.  This is an elfism. Use #if 0 for a.out.
 */

#if	TRUE
#define GTEXT(sym) FUNC(sym) ;  .type   FUNC(sym),@function
#define GDATA(sym) FUNC(sym) ;  .type   FUNC(sym),@object
#else
#define GTEXT(sym) FUNC(sym)
#define GDATA(sym) FUNC(sym)
#endif

#ifdef LP64
/* x86-64 ABI */
#define INTARG1_64 %rdi
#define INTARG2_64 %rsi
#define INTARG3_64 %rdx
#define INTARG4_64 %rcx
#define INTARG5_64 %r8
#define INTARG6_64 %r9

#define INTARG1_32 %edi
#define INTARG2_32 %esi
#define INTARG3_32 %edx
#define INTARG4_32 %ecx
#define INTARG5_32 %r8d
#define INTARG6_32 %r9d

#define INTARG1_16 %di
#define INTARG2_16 %si
#define INTARG3_16 %dx
#define INTARG4_16 %cx

#define INTRET1	%rax
#define INTRET2	%rdx

/* these should not be used by x86-64 ABI-compliant code */
#define SP_ARG0		0
#define SP_ARG1		4
#define SP_ARG2		8
#define SP_ARG3		12

#else
/* fp offsets to arguments */

#define ARG1	8
#define ARG1W	10
#define ARG2	12
#define ARG2W	14
#define ARG3	16
#define ARG3W	18
#define ARG4	20
#define ARG5	24
#define ARG6	28
#define ARG7	32
#define ARG8	36
#define ARG9	40
#define ARG10	44
#define ARG11	48
#define ARG12	52

#define DARG1	8		/* double arguments */
#define DARG1L	12
#define DARG2	16
#define DARG2L	20
#define DARG3	24
#define DARG3L	28
#define DARG4	32
#define DARG4L	36

/* sp offsets to arguments */

#define SP_ARG0		0
#define SP_ARG1		4
#define SP_ARG1W	6
#define SP_ARG2		8
#define SP_ARG2W	10
#define SP_ARG3		12
#define SP_ARG3W	14
#define SP_ARG4		16
#define SP_ARG5		20
#define SP_ARG6		24
#define SP_ARG7		28
#define SP_ARG8		32
#define SP_ARG9		36
#define SP_ARG10	40
#define SP_ARG11	44
#define SP_ARG12	48
#endif

#define _WRS_ASM(x) __asm volatile (x)

#endif /* __ASM_VBI_H */
