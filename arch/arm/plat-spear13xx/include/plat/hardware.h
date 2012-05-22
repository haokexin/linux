/*
 * arch/arm/plat-spear/include/plat/hardware.h
 *
 * Hardware definitions for SPEAr
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_HARDWARE_H
#define __PLAT_HARDWARE_H

#include <linux/types.h>
#include <asm/mach-types.h>

#ifndef __ASSEMBLY__
#define IOMEM(x)	((void __iomem __force *)(x))
#else
#define IOMEM(x)	(x)
#endif

#ifndef __ASSEMBLY__
/* cpu related runtime routines */
static inline bool cpu_is_spear1310(void)
{
	return machine_is_spear1310_evb();
}

static inline bool cpu_is_spear1310_reva(void)
{
	return machine_is_spear1310_reva_evb() || machine_is_r1801e();
}

static inline bool cpu_is_spear1340(void)
{
	return machine_is_spear1340_evb();
}

static inline bool arch_is_spear13xx(void)
{
	return cpu_is_spear1310_reva() ||
		cpu_is_spear1310() ||
		cpu_is_spear1340();
}
#endif /* __ASSEMBLY__ */

#endif /* __PLAT_HARDWARE_H */
