/*
 * arch/arm/plat-spear/restart.c
 *
 * SPEAr platform specific restart functions
 *
 * Copyright (C) 2009 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/io.h>
#ifndef CONFIG_ARCH_SPEAR13XX
#include <asm/hardware/sp810.h>
#else
#include <mach/misc_regs.h>
#endif
#include <mach/generic.h>
#include <mach/hardware.h>
#include <asm/system_misc.h>

void spear_restart(char mode, const char *cmd)
{
	if (mode == 's') {
		/* software reset, Jump into ROM at address 0 */
		soft_restart(0);
	} else {
#ifndef CONFIG_ARCH_SPEAR13XX
		/* hardware reset, Use on-chip reset capability */
		sysctl_soft_reset((void __iomem *)VA_SPEAR_SYS_CTRL_BASE);
#else
		if (cpu_is_spear1340()) {
	#ifdef CONFIG_CPU_SPEAR1340
			writel_relaxed(0x01, VA_SPEAR1340_SYS_SW_RES);
	#endif
		} else if (cpu_is_spear1310()) {
	#ifdef CONFIG_CPU_SPEAR1310
			writel_relaxed(0x01, VA_SPEAR1310_SYS_SW_RES);
	#endif
		} else
			writel_relaxed(0x01, VA_SYS_SW_RES);
#endif
	}
}
