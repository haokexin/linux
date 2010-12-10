/*
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

#ifndef _VBI_CPU_TYPES_H
#define _VBI_CPU_TYPES_H

/* define the cpu types that we currently support */
#define PPC85XX		1
#define PENTIUM		2
#define ARM1136		3
#define MIPSI64R2	4
#define PPCE500MC	5
#define PPC32		6
#define ARMCA9		7
#define PPCE200		8

#ifndef _ASMLANGUAGE

/* MIPS64R2 specific types */
#if (CPU == MIPSI64R2)

#define CPU_MIPS_32BIT 0

#define CPU_MIPS_64BIT 1

typedef unsigned long	INSTR;		/* 32 bit word-aligned instructions */

#if (CPU_MIPS_32BIT)
typedef unsigned int	_RType;		/* registers are 32 bits */
#elif (CPU_MIPS_64BIT)
typedef unsigned long	_RType;		/* registers are 64 bits */
#else /* CPU_MIPSxxBIT */
#error "Invalid CPU value"
#endif

#endif /* CPU == MIPSI64R2 */

/* PowerPC specific types */

#if (CPU == PPC85XX) || (CPU == PPCE500MC) || (CPU == PPCE200)
typedef unsigned long	INSTR;		/* 32 bit word-aligned instructions */
typedef unsigned int	_RType;		/* register type */
#endif

/* Pentium specific types */

#if (CPU == PENTIUM)
typedef unsigned char	INSTR;		/* char instructions */
					/* register type */
#if defined (LP64)
typedef unsigned long	_RType;
#else
typedef unsigned int	_RType;
#endif
#endif /* CPU == PENTIUM */

#if (CPU == ARM1136) || (CPU == ARMCA9)
typedef unsigned long	INSTR;		/* 32 bit word-aligned instructions */
typedef unsigned int	_RType;		/* register type */
#endif

#endif /* _ASMLANGUAGE */

#endif /* _VBI_CPU_TYPES_H */
