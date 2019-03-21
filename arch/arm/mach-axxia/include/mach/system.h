/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2003 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 */

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

#endif
