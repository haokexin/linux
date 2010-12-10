/*
 * arm reg_vbi.h - ARMv6 cpu registers */
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

#ifndef __INCarmRegsh
#define __INCarmRegsh

#define GREG_NUM        13      /* general purpose registers */
#define	_ARM_HREG_SIZE	4	/* only support 32-bit registers */

#define VFP_D_NUM	32      /* VFP has 32 single-precision registers */

#ifndef	_ASMLANGUAGE

/* some common names for registers */

#define fpReg           r[11]   /* frame pointer */

#endif	/* _ASMLANGUAGE */

#define HREG_SET_G_REG_BASE     0x00
#define HREG_SET_GR(n)		(HREG_SET_G_REG_BASE + _ARM_HREG_SIZE*(n))
#define HREG_SET_PC		(_ARM_HREG_SIZE*13)
#define HREG_SET_CPSR		(_ARM_HREG_SIZE*14)

#define HREG_SET_SP_USR		(_ARM_HREG_SIZE*15)
#define HREG_SET_LR_USR		(_ARM_HREG_SIZE*16)

#define HREG_SET_UNBANKED_SIZE	(HREG_SET_LR_USR + _ARM_HREG_SIZE)

/* Banked registers */
#define HREG_SET_SP_SVC		(_ARM_HREG_SIZE*17)
#define HREG_SET_LR_SVC		(_ARM_HREG_SIZE*18)

#define HREG_SET_SP_ABT		(_ARM_HREG_SIZE*19)
#define HREG_SET_LR_ABT		(_ARM_HREG_SIZE*20)

#define HREG_SET_SP_UND		(_ARM_HREG_SIZE*21)
#define HREG_SET_LR_UND		(_ARM_HREG_SIZE*22)

#define HREG_SET_SP_IRQ		(_ARM_HREG_SIZE*23)
#define HREG_SET_LR_IRQ		(_ARM_HREG_SIZE*24)

/* FIQ register base */
#define HREG_SET_SPSR_FIQ_GREG	(_ARM_HREG_SIZE*25)
#define HREG_SET_SP_FIQ		(_ARM_HREG_SIZE*30)
#define HREG_SET_LR_FIQ		(_ARM_HREG_SIZE*31)

#define HREG_SET_SPSR_SVC	(_ARM_HREG_SIZE*32)
#define HREG_SET_SPSR_ABT	(_ARM_HREG_SIZE*33)
#define HREG_SET_SPSR_UND	(_ARM_HREG_SIZE*34)
#define HREG_SET_SPSR_IRQ	(_ARM_HREG_SIZE*35)
#define HREG_SET_SPSR_FIQ	(_ARM_HREG_SIZE*36)

#define HREG_SET_COPROC_ACCESS	(_ARM_HREG_SIZE*37)

#endif /* __INCarmRegsh */
