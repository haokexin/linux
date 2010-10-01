/*
 * arch/arm/mach-spear13xx/include/mach/generic.h
 *
 * spear13xx machine family generic header file
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MACH_GENERIC_H
#define __MACH_GENERIC_H

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

/*
 * Each GPT has 2 timer channels
 * Following GPT channels will be used as clock source and clockevent
 */
#define SPEAR_GPT0_BASE		SPEAR13XX_GPT0_BASE
#define SPEAR_GPT0_CHAN0_IRQ	IRQ_GPT0_TMR0
#define SPEAR_GPT0_CHAN1_IRQ	IRQ_GPT0_TMR1

#endif /* __MACH_GENERIC_H */
