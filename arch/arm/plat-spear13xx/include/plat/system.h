/*
 * arch/arm/plat-spear/include/plat/system.h
 *
 * SPEAr platform specific architecture functions
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_SYSTEM_H
#define __PLAT_SYSTEM_H

#include <asm/hardware/sp810.h>
#include <asm/proc-fns.h>
#include <mach/hardware.h>

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline int arch_change_mode(int mode)
{
	return sysctl_change_mode((void __iomem *)VA_SPEAR_SYS_CTRL_BASE, mode);
}

#endif /* __PLAT_SYSTEM_H */
