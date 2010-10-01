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

#include <mach/spear.h>

/* Vitual to physical translation of statically mapped space */
/*
 * if phy_addr is 0x8...,.... and above then map it to  0xF...,....
 * else map it to 0xE...,....
 */

#define IO_ADDRESS(x)   ((x) | ((((x) >> 31) << 28) | 0xE0000000))

/* typesafe io address */
#define __io_address(n)		__io(IO_ADDRESS(n))

#endif /* __MACH_HARDWARE_H */
