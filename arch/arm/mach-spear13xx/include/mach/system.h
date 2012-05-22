/*
 * arch/arm/mach-spear13xx/include/mach/system.h
 *
 * spear13xx Machine family specific architecture functions
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_SYSTEM_H
#define __MACH_SYSTEM_H

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <asm/proc-fns.h>
#include <mach/hardware.h>
#include <mach/misc_regs.h>

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
	u32 val, mode_sts;
	unsigned long finish;
	void __iomem *sys_reg;

	if (cpu_is_spear1340()) {
#ifdef CONFIG_CPU_SPEAR1340
		sys_reg = VA_SPEAR1340_SYS_CLK_CTRL;
#endif
	} else if (cpu_is_spear1310()) {
#ifdef CONFIG_CPU_SPEAR1310
		sys_reg = VA_SPEAR1310_SYS_CLK_CTRL;
#endif
	} else
		sys_reg = VA_SYS_CLK_CTRL;

	switch (mode) {
	case SYS_MODE_DOZE:
		mode_sts = SYS_MODE_STS_DOZE;
		break;
	case SYS_MODE_SLOW:
		mode_sts = SYS_MODE_STS_SLOW;
		break;
	case SYS_MODE_NORMAL:
		mode_sts = SYS_MODE_STS_NORMAL;
		break;
	default:
		pr_err("Wrong system mode\n");
		return -EINVAL;
	}

	val = readl(sys_reg);
	if ((val & SYS_MODE_STS_MASK) == mode_sts)
		return 0;

	val &= ~SYS_MODE_MASK;
	val |= mode;
	writel(val, sys_reg);

	/* read back if mode is set */
	finish = jiffies + 2 * HZ;
	do {
		val = readl(sys_reg);
		if ((val & SYS_MODE_STS_MASK) == mode_sts)
			return 0;
		udelay(1000);
	} while (!time_after_eq(jiffies, finish));

	return -EFAULT;
}

#endif /* __MACH_SYSTEM_H */
