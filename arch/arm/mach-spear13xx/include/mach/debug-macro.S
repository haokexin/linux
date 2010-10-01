/*
 * arch/arm/mach-spear13xx/include/mach/debug-macro.S
 *
 * Debugging macro include header for spear13xx machine family
 *
 * Copyright (C) 2011 WindRiver
 * Stanley Miao <stanley.miao@windriver.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1				@ MMU enabled?
		moveq	\rx, #0xE0000000		@ Physical base
		movne	\rx, #0xF0000000		@ Virtual base
		.endm

#include <asm/hardware/debug-pl01x.S>
