/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Debugging macro include header
 *
 * Copyright (C) 2021 INTEL Corporation
 */

#include <mach/hardware.h>

#if !defined(CONFIG_ARM_LPAE)
#error "Axxia Peripherals Are Only Accessible Using the LPAE!"
#endif

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =(AXXIA_DEBUG_UART_PHYS & 0xffffffff)
	ldr	\rv, =AXXIA_DEBUG_UART_VIRT
	.endm

#include <debug/pl01x.S>
