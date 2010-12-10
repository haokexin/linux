/*
 * types.h - virtual board interfaces types definitions
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
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

#ifndef _VBI_TYPES_H
#define _VBI_TYPES_H

#ifdef __ASSEMBLY__
#define _ASMLANGUAGE
#endif /* __ASSEMBLY__ */

/* The CPU specifics here should be relocated to arch dirs */
#ifdef CONFIG_X86
#define CPU PENTIUM
#endif /* CONFIG_X86 */

#ifdef CONFIG_PPC
#define CPU PPC85XX   /* Currently all PPC treated the same in vbi */
#endif /* CONFIG_PPC */

#ifdef CONFIG_CPU_CAVIUM_OCTEON
#define CPU MIPSI64R2
#endif /* CONFIG_CPU_CAVIUM_OCTEON */

#ifdef CONFIG_ARM
#define CPU ARMCA9
#endif /* CONFIG_ARM */

#include <vbi/cpu_types.h>	/* for PENTIUM, PPC85XX, etc */

#endif  /* _VBI_TYPES_H */
