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
 *  Copyright (C) 2011 Wind River Systems, Inc.
 */

#ifndef __ARM_ASM_WRHV_H
#define __ARM_ASM_WRHV_H

#ifdef CONFIG_WRHV

#define WRHV_SUPER_EARLY_STACK_SIZE	512

#define FAKE_READ_TLS_REG_UNDEF_INSTR	0xee1d0f72 /* mrc p15,0,r0,c13,c2,3 */

#ifndef __ASSEMBLY__

#include <vbi/vmmu.h>

extern void wrhv_time_init(void);
extern void wrhv_init_irq(void);
extern void (*wrhv_machine_init_irq)(void);
extern void (*wrhv_machine_init_timer)(void);
extern spinlock_t vmmu_handle_lock;

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_WRHV */
#endif /* __ARM_ASM_WRHV_H */
