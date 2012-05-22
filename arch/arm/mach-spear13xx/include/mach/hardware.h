/*
 * arch/arm/mach-spear13xx/include/mach/hardware.h
 *
 * Hardware definitions for spear13xx machine family
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

#include <plat/hardware.h>
#include <mach/spear.h>

/* Vitual to physical translation of statically mapped space
 *
 * Physical			Virtual
 * 0x6C800000-0X6C801000	0xF6C80000-0XF6C81000
 * 0x80000000-0X80010000	0xF8000000-0XF8010000
 * 0x90000000-0X90010000	0xF9000000-0XF9010000
 * 0xB3800000-0XB3808000	0xFB380000-0XFB388000
 * 0xE0000000-0XE0001000	0xFE000000-0XFE001000
 * 0xE0700000-0XE0702000	0xFE070000-0XFE072000
 * 0xE0800000-0XE0801000	0xFE080000-0XFE081000
 * 0xEC800000-0XEC802000	0xFEC80000-0XFEC82000
 * 0xED000000-0XED001000	0xFED00000-0XFED01000
 */

#define IO_ADDRESS(x)		(((x) & 0x0000FFFF) | (((x) & 0xFFF00000) >> 4) | \
				0xF0000000)

/* typesafe io address */
#define __io_address(n)		__io(IO_ADDRESS(n))

#endif /* __MACH_HARDWARE_H */
