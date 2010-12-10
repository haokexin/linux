/*
 *  ppc reg_vbi.h - PowerPC cpu registers
 *
 * Copyright (c) 2007 Wind River Systems, Inc.
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

#ifndef __INCppcRegsh
#define __INCppcRegsh

#define GREG_NUM	32	/* has 32 32/64-bit data registers */
#define	_PPC_HREG_SIZE	4	/* only support 32-bit registers */

#ifndef	_ASMLANGUAGE

/* some common names for registers */

#define spReg	gpr[1]	/* stack pointer */
#define	fpReg	gpr[31]	/* frame pointer */
#define reg_pc	pc	/* program counter */
#define reg_sp	spReg	/* stack pointer */
#define reg_fp	fpReg	/* frame pointer */

#endif	/* _ASMLANGUAGE */


#define HREG_SET_GRBASE	0x00	/* general purpose register base */
#define HREG_SET_GR(n)		(HREG_SET_GRBASE + _PPC_HREG_SIZE*(n))
#define HREG_SET_MSR		(HREG_SET_GR(GREG_NUM))
#define HREG_SET_LR		(HREG_SET_MSR + _PPC_HREG_SIZE)
#define HREG_SET_CTR		(HREG_SET_LR + _PPC_HREG_SIZE)
#define HREG_SET_PC		(HREG_SET_CTR + _PPC_HREG_SIZE)
#define HREG_SET_CR		(HREG_SET_PC + _PPC_HREG_SIZE)
#define HREG_SET_XER		(HREG_SET_CR + 4)
#define HREG_SET_SPEFSCR	(HREG_SET_XER + 4)
#define HREG_SET_CASID		(HREG_SET_SPEFSCR + 4)
#define HREG_SET_SP		(HREG_SET_GR(1))

/* size of the full REG_SET structure, including spare bytes */

#define _PPC_HREG_SET_SIZE       (_PPC_HREG_SIZE * (GREG_NUM + 7))

/* include any cpu specific register declarations */

#if (CPU == PPC85XX) || (CPU == PPCE500MC) || (CPU == PPCE200)
# include <sys/ppc/ppc85xx.h>
#endif

#endif /* __INCppcRegsh */
