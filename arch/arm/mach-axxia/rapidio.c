// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation
/*
 * Helper module for board specific RAPIDIO bus registration
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/axxia-ncr.h>
#include <linux/signal.h>

#include <mach/rio.h>

/**
 * axxia_rio_fault -
 *   Intercept SRIO bus faults due to unimplemented register locations.
 *   Return 0 to keep 'reads' alive.
 */

static int
axxia_rio_fault(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	/* unsigned long pc = instruction_pointer(regs); */
	/* unsigned long instr = *(unsigned long *)pc; */
	return 0;
}

/**
 * axxia_rapidio_init -
 *   Perform board-specific initialization to support use of RapidIO busses
 *
 * Returns 0 on success or an error code.
 */
int __init
axxia_rapidio_init(void)
{
	hook_fault_code(0x11, axxia_rio_fault, SIGBUS, 0,
			"asynchronous external abort");

	return 0;
}
