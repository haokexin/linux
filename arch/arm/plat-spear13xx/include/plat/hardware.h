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
static inline bool cpu_is_spear300(void)
{
	return machine_is_spear300_evb();
}

static inline bool cpu_is_spear310(void)
{
	return machine_is_spear310_evb();
}

static inline bool cpu_is_spear320(void)
{
	return machine_is_spear320_evb() || machine_is_spear320_hmi();
}

static inline bool cpu_is_spear600(void)
{
	return machine_is_spear600_evb();
}

static inline bool cpu_is_spear1300(void)
{
	return machine_is_spear1300_evb();
}

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
	return machine_is_spear1340_evb() || machine_is_spear_hurricane() ||
		machine_is_spear1340_lcad();
}

static inline bool cpu_is_spear900(void)
{
	return machine_is_spear900_evb();
}

/* arch related runtime routines */
static inline bool arch_is_spear3xx(void)
{
	return cpu_is_spear300() || cpu_is_spear310() || cpu_is_spear320();
}

static inline bool arch_is_spear6xx(void)
{
	return cpu_is_spear600();
}

static inline bool arch_is_spear13xx(void)
{
	return cpu_is_spear1300() || cpu_is_spear1310_reva() ||
		cpu_is_spear1310() || cpu_is_spear900() ||
		cpu_is_spear1340();
}
#endif /* __ASSEMBLY__ */

#endif /* __PLAT_HARDWARE_H */
